/*
    RF Sensing BAND Selector for Xiegu GPA-100
    October 12, 2026
    Larry D O'Cull
    (c) 2026
    LDO Systems LLC - WR9R
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

// EEPROM Configurations
const int EEPROM_START_ADDR  = 0;
const uint32_t EEPROM_MAGIC  = 0x42414E44; // "BAND" validation flag

uint32_t currentMeasuredFreq = 0;
uint16_t activePwmDuty = 286;     
uint16_t latchedPwmDuty = 286;    // Remembers active band step configuration

// Deferred EEPROM State Variables (Isolated to Band-Steps Only)
uint16_t lastSavedEepromDuty = 286; // Tracks what is currently written inside Flash
uint32_t bandTransitionTime = 0;
bool eepromWritePending = false;

void initTimer1_HighSpeedCounter() {
    RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN);  

    GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
    GPIOA->CRH |= GPIO_CRH_CNF8_0; // Input Floating

    TIM1->CR1  = 0;
    TIM1->SMCR = 0;         
    TIM1->CCMR1 = 0; // Wipe hidden filters
    
    TIM1->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2); // SMS = 111
    TIM1->SMCR |= (TIM_SMCR_TS_0 | TIM_SMCR_TS_2);                   // TS = 101

    TIM1->CCMR1 |= TIM_CCMR1_CC1S_0;     
    TIM1->CCER  &= ~TIM_CCER_CC1P;       
    TIM1->CCER  |= TIM_CCER_CC1E;        

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
    
    TIM3->CCER |= TIM_CCER_CC1E;         
    TIM3->CR1 &= ~TIM_CR1_ARPE;           
    TIM3->CR1 |= TIM_CR1_CEN;            
}

void setAnalogVoltageRegister(uint16_t dutyValue) {
    if (dutyValue > 4095) dutyValue = 4095;
    TIM3->CCR1 = dutyValue;              
    activePwmDuty = dutyValue;
}

uint16_t calculateTargetPWM(uint32_t freqHz) {
    if (freqHz < 1000000)                        return 0; 

    if (freqHz >= 900000 && freqHz < 2200000)   return 286 * 1;  // 160M (230 mV)
    if (freqHz >= 3300000 && freqHz < 4300000)   return 286 * 2;  // 80M  (460 mV)
    if (freqHz >= 5000000 && freqHz < 5500000)   return 286 * 3;  // 60M  (690 mV)
    if (freqHz >= 6900000 && freqHz < 7500000)   return 286 * 4;  // 40M  (920 mV)
    if (freqHz >= 9900000 && freqHz < 11000000) return 286 * 5;  // 30M  (1.15 V)
    if (freqHz >= 13900000 && freqHz < 14550000) return 286 * 6;  // 20M  (1.38 V)
    if (freqHz >= 17900000 && freqHz < 18500000) return 286 * 7;  // 17M  (1.61 V)
    if (freqHz >= 20000000 && freqHz < 22000000) return 286 * 8;  // 15M  (1.84 V)
    if (freqHz >= 23900000 && freqHz < 25100000) return 286 * 9;  // 12M  (2.07 V)
    if (freqHz >= 27900000 && freqHz < 31000000) return 286 * 10; // 10M  (2.30 V)
    if (freqHz >= 49000000 && freqHz <= 56000000) return 286 * 11; // 6M   (2.53 V)
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
    lastSavedEepromDuty = latchedPwmDuty; // Synchronize baseline Flash markers
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
    static uint32_t lastSampleTime = 0;
    static uint32_t lastPrintTime = 0;          // RESTORED
    static uint16_t lastCounterVal = 0;
    static bool lastPttState = true;           

    // State machine control variables
    static uint32_t pttAssertTime = 0;          
    static bool sequenceComplete = false;       
    static int consecutiveMatches = 0;         
    static uint16_t sampleBuffer[3] = {0, 0, 0}; 

    uint32_t currentMicros = micros();
    uint32_t currentMillis = millis();
    
    // Read current radio PTT state (Active LOW)
    bool currentPttState = (digitalRead(PIN_RADIO_PTT) == LOW); 

    // ---- EDGE DETECTION: RADIO INITIALIZES TRANSMISSION ----
    if (currentPttState && !lastPttState) {
        pttAssertTime = currentMillis;
        sequenceComplete = false;
        consecutiveMatches = 0;
        lastCounterVal = (uint16_t)TIM1->CNT;
        lastSampleTime = currentMicros;
    } 
    // ---- RESET: RADIO RELEASES TRANSMISSION ----
    else if (!currentPttState && lastPttState) {
        digitalWrite(PIN_AMP_PTT, LOW);
        sequenceComplete = false;
        consecutiveMatches = 0;
    }

    // ---- CORE FILTER AND MEASUREMENT ENGINE ----
    if (currentPttState && !sequenceComplete) {
        if (currentMillis - pttAssertTime >= 10) {
            uint32_t elapsedWindow = currentMicros - lastSampleTime;
            
            if (elapsedWindow >= 4000) { 
                uint16_t currentCounterVal = (uint16_t)TIM1->CNT;
                uint16_t pulseDelta = currentCounterVal - lastCounterVal;
                
                uint64_t largeMathProduct = (uint64_t)pulseDelta * 1000000ULL * PRESCALER_DIV;
                currentMeasuredFreq = (uint32_t)(largeMathProduct / elapsedWindow);
                
                lastCounterVal = currentCounterVal;
                lastSampleTime = currentMicros;

                uint16_t samplePWM = calculateTargetPWM(currentMeasuredFreq);

                if (samplePWM > 0) {
                    if (consecutiveMatches == 0) {
                        sampleBuffer[0] = samplePWM;
                        consecutiveMatches = 1;
                    } else {
                        uint16_t anchor = sampleBuffer[0];
                        uint16_t tolerance = (uint16_t)((uint32_t)anchor * 2 / 100);
                        
                        if (abs((int)samplePWM - (int)anchor) <= tolerance) {
                            sampleBuffer[consecutiveMatches] = samplePWM;
                            consecutiveMatches++;
                            
                           // Step 2: 3 valid consecutive matches achieved
                          if (consecutiveMatches >= 3) {
                              // Average the three safe values to arrive at the desired band voltage
                              uint16_t averagedPWM = (uint16_t)(((uint32_t)sampleBuffer[0] + sampleBuffer[1] + sampleBuffer[2]) / 3);
                              
                              // Check if the band voltage actually changed from what is currently latched
                              if (averagedPWM != latchedPwmDuty) {
                                  setAnalogVoltageRegister(averagedPWM);
                                  latchedPwmDuty = averagedPWM;
                                  delay(20); // 20ms hardware settling guard delay ONLY on an actual band change
                              } else {
                                  // Same band as before: skip delay to minimize hot-switching lock latency
                                  setAnalogVoltageRegister(averagedPWM);
                              }
                              
                              // Step 3: Output the hardware PTT OUT to HIGH
                              digitalWrite(PIN_AMP_PTT, HIGH);
                              
                              // Step 4: Lock state processing down until physical PTT reset occurs
                              sequenceComplete = true; 
                          }

                        } else {
                            sampleBuffer[0] = samplePWM;
                            consecutiveMatches = 1;
                        }
                    }
                } else {
                    consecutiveMatches = 0;
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
        eepromWritePending = false;
        lastSavedEepromDuty = requiredPWM;
        EEPROM.put(EEPROM_START_ADDR + sizeof(EEPROM_MAGIC), lastSavedEepromDuty);
        if (Serial) {
          Serial.print("EEPROM Updated");
          Serial.println();
        }
    }

    lastPttState = currentPttState;

    // ---- RESTORED: MONITORING TELEMETRY TERMINAL (Every 500ms) ----
    if (currentMillis - lastPrintTime >= 500) {
        lastPrintTime = currentMillis;
        digitalWrite(OB_LED, !digitalRead(OB_LED)); // Blink onboard LED as a heartbeat
        
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
                Serial.print(" | Filter Matches: ");
                Serial.print(consecutiveMatches);
            }
            Serial.println();
        }
    }
}
