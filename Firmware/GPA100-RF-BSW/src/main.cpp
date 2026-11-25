/* 
 * RF Sensing BAND Selector for Xiegu GPA-100 (SSB Compatible)
 * October 12, 2026
 * Nov 13 -- revised to accodate SSB (dead air -- no RF state)
 * Larry D O'Cull (c) 2026 LDO Systems LLC - WR9R 
 * 
 *---

### Key Architectural Changes for SSB

* **Silence-Tolerant Gating (Voice Pause Handling)**
  In the original code, an `else { consecutiveMatches = 0; }` block wiped your sample progress whenever a 4 ms sample window detected zero RF. In SSB mode, pauses between syllables or before you begin speaking produce zero RF. The updated code ignores windows where `samplePWM == 0`, preserving `validSampleCount` and `anchorFreqHz` across speech gaps until 10 valid vocal syllables accumulate.
* **True 5 kHz Frequency Window Comparison**
  The original code compared PWM duty cycle values (`samplePWM` vs. `anchor`) with a 2% tolerance. Because PWM steps are fixed integers per band, that logic checked band identity rather than actual RF stability. The new logic compares `measuredHz` directly against `anchorFreqHz` using `FREQ_TOLERANCE_HZ = 5000` (5 kHz).
* **Fast-Path Same-Band Lock (`FAST_CONFIRM_SAMPLES = 3`)**
  If you key the microphone on the same band you were already operating on, waiting for 10 full voice syllables clips the first word of your transmission. When the firmware collects **3 valid bursts** within 5 kHz that map to the currently `latchedPwmDuty`, it skips the remaining 7 samples and asserts `PIN_AMP_PTT` immediately.
* **Full 10-Sample Validation on Band Changes**
  If the incoming RF maps to a new band, the controller waits for all **10 valid RF bursts** within 5 kHz of each other, calculates the average frequency, updates the PWM band voltage, waits 20 ms for the GPA-100 relays to switch safely, and then keys the amplifier.
* **Partial-Burst Edge Note**
  Because a voice syllable can start or end mid-way through a 4 ms (`4000 µs`) counting window, the very first or last sample of a word may register an artificially low frequency (half-filled window). Whenever a partial burst falls outside a valid amateur band (`calculateTargetPWM() == 0`), it is cleanly ignored without resetting your valid sample streak.

 */

#include <Arduino.h>
#include <EEPROM.h>

// Universal Pin Assignments
const int PIN_RF_IN     = PA8;  // HIGH-SPEED RF INPUT PATH: Direct to TIM1_CH1
const int PIN_RADIO_PTT = PA1;  // Input from transceiver PTT (Active Low)
const int PIN_AMP_PTT   = PA2;  // Output to drive keying transistor (Active High)
const int PIN_PWM_OUT   = PA6;  // Hardware PWM to RC filter (TIM3_CH1)
const int OB_LED        = PC13; // Onboard Diagnostic LED

const uint32_t PRESCALER_DIV = 4; // Your physical divide-by-4 flip-flop

// SSB Frequency Validation Constants
// const int REQUIRED_SAMPLES      = 10;     // Need 10 valid RF bursts within tolerance
// const int FAST_CONFIRM_SAMPLES  = 3;      // Fast-path lock if band hasn't changed
// const uint32_t FREQ_TOLERANCE_HZ = 15000;  // 5 kHz tolerance window (Opened to 15K for 6M performance)
// const int SETTLING_TIME_MS      = 10;     // Relay settling time in milliseconds
// ---- UPDATED TIMING CONSTANTS ----
const int REQUIRED_SAMPLES      = 5;     // 5 valid bursts (10ms total RF) for new band
const int FAST_CONFIRM_SAMPLES  = 2;     // 2 valid bursts (4ms total RF) for same band (~12ms total key time!)
const uint32_t FREQ_TOLERANCE_HZ = 15000; // 15 kHz tolerance window
const int SETTLING_TIME_MS      = 8;     // Reduced from 10ms
const uint32_t SAMPLE_WINDOW_US = 2000;  // 2ms counting window (was 4000us)
const int AMP_RELAY_DELAY_MS    = 15;    // Relay settling delay on band change (was 20ms)

// EEPROM Configurations
const int EEPROM_START_ADDR   = 0;
const uint32_t EEPROM_MAGIC   = 0x42414E44; // "BAND" validation flag

uint32_t currentMeasuredFreq  = 0;
uint16_t activePwmDuty        = 286;
uint16_t latchedPwmDuty       = 286; // Remembers active band step configuration

// Deferred EEPROM State Variables (Isolated to Band-Steps Only)
uint16_t lastSavedEepromDuty  = 286; // Tracks what is currently written inside Flash
uint32_t bandTransitionTime   = 0;
bool eepromWritePending       = false;

void initTimer1_HighSpeedCounter() {
  RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN);
  GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
  GPIOA->CRH |= GPIO_CRH_CNF8_0; // Input Floating

  TIM1->CR1   = 0;
  TIM1->SMCR  = 0;
  TIM1->CCMR1 = 0; // Wipe hidden filters

  TIM1->SMCR  |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2); // SMS = 111
  TIM1->SMCR  |= (TIM_SMCR_TS_0 | TIM_SMCR_TS_2);                    // TS = 101
  TIM1->CCMR1 |= TIM_CCMR1_CC1S_0;

  TIM1->CCER &= ~TIM_CCER_CC1P;
  TIM1->CCER |= TIM_CCER_CC1E;

  TIM1->PSC = 0;
  TIM1->ARR = 0xFFFF;
  TIM1->CNT = 0;
  TIM1->CR1 |= TIM_CR1_CEN;
}

void initTimer3_HardwarePWM() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN);

  GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
  GPIOA->CRL |= (GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1);

  TIM3->PSC = 0;
  TIM3->ARR = 4095;
  TIM3->CCMR1 &= ~TIM_CCMR1_OC1PE; // Instant update activation (No shadow lag)
  TIM3->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
  TIM3->CCER  |= TIM_CCER_CC1E;
  TIM3->CR1   &= ~TIM_CR1_ARPE;
  TIM3->CR1   |= TIM_CR1_CEN;
}

void setAnalogVoltageRegister(uint16_t dutyValue) {
  if (dutyValue > 4095) dutyValue = 4095;
  TIM3->CCR1 = dutyValue;
  activePwmDuty = dutyValue;
}

uint16_t calculateTargetPWM(uint32_t freqHz) {
  if (freqHz < 1000000) return 0;
  if (freqHz >= 900000  && freqHz < 2200000)  return 286 * 1;  // 160M (230 mV)
  if (freqHz >= 3300000 && freqHz < 4300000)  return 286 * 2;  // 80M  (460 mV)
  if (freqHz >= 5000000 && freqHz < 5500000)  return 286 * 3;  // 60M  (690 mV)
  if (freqHz >= 6900000 && freqHz < 7500000)  return 286 * 4;  // 40M  (920 mV)
  if (freqHz >= 9900000 && freqHz < 11000000) return 286 * 5;  // 30M  (1.15 V)
  if (freqHz >= 13900000 && freqHz < 14550000) return 286 * 6; // 20M  (1.38 V)
  if (freqHz >= 17900000 && freqHz < 18500000) return 286 * 7; // 17M  (1.61 V)
  if (freqHz >= 20000000 && freqHz < 22000000) return 286 * 8; // 15M  (1.84 V)
  if (freqHz >= 23900000 && freqHz < 25100000) return 286 * 9; // 12M  (2.07 V)
  if (freqHz >= 27900000 && freqHz < 31000000) return 286 * 10;// 10M  (2.30 V)
  if (freqHz >= 49000000 && freqHz <= 56000000) return 286 * 11;// 6M  (2.53 V)
  return 0;
}

void loadSavedBandSettings() {
  uint32_t magicCheck = 0;
  EEPROM.get(EEPROM_START_ADDR, magicCheck);
  if (magicCheck == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_START_ADDR + sizeof(magicCheck), latchedPwmDuty);
    if (latchedPwmDuty < 286 || latchedPwmDuty > 3146) {
      latchedPwmDuty = 286;
    }
  } else {
    EEPROM.put(EEPROM_START_ADDR, EEPROM_MAGIC);
    EEPROM.put(EEPROM_START_ADDR + sizeof(magicCheck), latchedPwmDuty);
  }
  lastSavedEepromDuty = latchedPwmDuty;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RADIO_PTT, INPUT_PULLUP);
  pinMode(PIN_AMP_PTT, OUTPUT);
  pinMode(OB_LED, OUTPUT);
  digitalWrite(PIN_AMP_PTT, LOW);
  digitalWrite(OB_LED, LOW);

  loadSavedBandSettings();
  initTimer1_HighSpeedCounter(); // PA8
  initTimer3_HardwarePWM();      // PA6
  setAnalogVoltageRegister(latchedPwmDuty);
}

void loop() {
  static uint32_t lastSampleTime    = 0;
  static uint32_t lastPrintTime     = 0;
  static uint16_t lastCounterVal    = 0;
  static bool lastPttState          = true;

  // State machine control variables
  static uint32_t pttAssertTime     = 0;
  static bool sequenceComplete      = false;
  static int validSampleCount       = 0;
  static uint32_t anchorFreqHz      = 0;
  static uint32_t freqAccumulator   = 0;

  uint32_t currentMicros = micros();
  uint32_t currentMillis = millis();

  // Read current radio PTT state (Active LOW)
  bool currentPttState = (digitalRead(PIN_RADIO_PTT) == LOW);

  // ---- EDGE DETECTION: RADIO INITIALIZES TRANSMISSION ----
  if (currentPttState && !lastPttState) {
    pttAssertTime    = currentMillis;
    sequenceComplete = false;
    validSampleCount = 0;
    anchorFreqHz     = 0;
    freqAccumulator  = 0;
    lastCounterVal   = (uint16_t)TIM1->CNT;
    lastSampleTime   = currentMicros;
  }
  // ---- RESET: RADIO RELEASES TRANSMISSION ----
  else if (!currentPttState && lastPttState) {
    digitalWrite(PIN_AMP_PTT, LOW);
    sequenceComplete = false;
    validSampleCount = 0;
  }

  // ---- CORE SSB-COMPATIBLE FREQUENCY ENGINE ----
  if (currentPttState && !sequenceComplete) {
    // Wait 10ms after PTT assertion to allow transceiver relays to settle
    if (currentMillis - pttAssertTime >= SETTLING_TIME_MS) {
      uint32_t elapsedWindow = currentMicros - lastSampleTime;

      if (elapsedWindow >= 4000) {
        uint16_t currentCounterVal = (uint16_t)TIM1->CNT;
        uint16_t pulseDelta        = currentCounterVal - lastCounterVal;
        uint64_t largeMathProduct  = (uint64_t)pulseDelta * 1000000ULL * PRESCALER_DIV;
        uint32_t measuredHz        = (uint32_t)(largeMathProduct / elapsedWindow);

        lastCounterVal = currentCounterVal;
        lastSampleTime = currentMicros;

        uint16_t samplePWM = calculateTargetPWM(measuredHz);

        // Only process windows where valid amateur band RF is present.
        // If samplePWM == 0 (SSB voice pause / silence), ignore and retain progress!
        if (samplePWM > 0) {
          currentMeasuredFreq = measuredHz;

          if (validSampleCount == 0) {
            anchorFreqHz     = measuredHz;
            freqAccumulator  = measuredHz;
            validSampleCount = 1;
          } else {
            // Check if this RF burst is within 5 kHz (5000 Hz) of our anchor frequency
            uint32_t freqDiff = (measuredHz > anchorFreqHz) ? (measuredHz - anchorFreqHz) : (anchorFreqHz - measuredHz);

            if (freqDiff <= FREQ_TOLERANCE_HZ) {
              freqAccumulator += measuredHz;
              validSampleCount++;

              // FAST-PATH LOCK: If we have at least 3 consistent samples AND they map
              // to the exact same band already latched, key the amp immediately!
              if (validSampleCount >= FAST_CONFIRM_SAMPLES && samplePWM == latchedPwmDuty) {
                setAnalogVoltageRegister(latchedPwmDuty);
                digitalWrite(PIN_AMP_PTT, HIGH);
                sequenceComplete = true;
              }
              // FULL-PATH LOCK: Wait for 10 valid RF samples within 5 kHz for a band change
              else if (validSampleCount >= REQUIRED_SAMPLES) {
                uint32_t averageFreq = freqAccumulator / REQUIRED_SAMPLES;
                uint16_t targetPWM   = calculateTargetPWM(averageFreq);

                if (targetPWM > 0) {
                  if (targetPWM != latchedPwmDuty) {
                    setAnalogVoltageRegister(targetPWM);
                    latchedPwmDuty = targetPWM;
                    delay(AMP_RELAY_DELAY_MS); // ms settling delay for GPA-100 band relay switching
                  } else {
                    setAnalogVoltageRegister(latchedPwmDuty);
                  }
                  digitalWrite(PIN_AMP_PTT, HIGH);
                  sequenceComplete = true;
                }
              }
            } else {
              // Frequency jumped outside 5 kHz (e.g., noisy edge or VFO bump); reset anchor to new burst
              anchorFreqHz     = measuredHz;
              freqAccumulator  = measuredHz;
              validSampleCount = 1;
            }
          }
        }
      }
    }
  }

  uint16_t requiredPWM = latchedPwmDuty;

  // ---- TIMED DEFERRED FLASH STORAGE ----
  if (requiredPWM != lastSavedEepromDuty && requiredPWM > 0) {
    if (!eepromWritePending) {
      bandTransitionTime = currentMillis;
      eepromWritePending = true;
    }
    static uint16_t checkLastTarget = requiredPWM;
    if (requiredPWM != checkLastTarget) {
      bandTransitionTime = currentMillis;
      checkLastTarget = requiredPWM;
    }
  } else {
    eepromWritePending = false;
  }

  if (eepromWritePending && (currentMillis - bandTransitionTime >= 5000)) {
    eepromWritePending  = false;
    lastSavedEepromDuty = requiredPWM;
    EEPROM.put(EEPROM_START_ADDR + sizeof(EEPROM_MAGIC), lastSavedEepromDuty);
    if (Serial) {
      Serial.println("EEPROM Updated");
    }
  }

  lastPttState = currentPttState;

  // ---- MONITORING TELEMETRY TERMINAL (Every 500ms) ----
  if (currentMillis - lastPrintTime >= 500) 
  {
    lastPrintTime = currentMillis;
    digitalWrite(OB_LED, !digitalRead(OB_LED));

    if (Serial) {
        Serial.print("Radio PTT: ");
        Serial.print(currentPttState ? "TX_ACTIVE" : "RX_IDLE");
        Serial.print(" | Lock: ");
        Serial.print(sequenceComplete ? "LOCKED" : "VALIDATING");
        Serial.print(" | Freq: ");
        Serial.print((float)currentMeasuredFreq / 1000000.0, 3);
        Serial.print(" MHz | Output Voltage: ");
        Serial.print(((float)latchedPwmDuty / 4095.0) * 3.3, 2);
        Serial.print(" V");
        if (currentPttState && !sequenceComplete) {
            Serial.print(" | Valid Bursts: ");
            Serial.print(validSampleCount);
        }
        Serial.println();
    }
  }
}