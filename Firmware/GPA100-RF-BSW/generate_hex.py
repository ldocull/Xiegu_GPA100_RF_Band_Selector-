import os
from SCons.Script import Import

# Import the global build environment variable from PlatformIO
Import("env")

# Extract the HEX file from the compiled ELF target
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(
        '"$OBJCOPY" -O ihex "$BUILD_DIR/${PROGNAME}.elf" "$BUILD_DIR/${PROGNAME}.hex"', 
        "Generating HEX binary"
    )
)