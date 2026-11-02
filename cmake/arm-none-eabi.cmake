# Toolchain file: arm-none-eabi GCC 16 for the chip of this build tree.
# RTOS_GCC_ROOT is set by CMakePresets.json; RTOS_CHIP selects the chip and with it the CPU:
#   stm32h563  Cortex-M33 + FPv5-SP (default)
#   stm32g431  Cortex-M4  + FPv4-SP
# One chip per build tree: every library (wolfSSL, littlefs, ...) is compiled for its CPU.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES RTOS_GCC_ROOT RTOS_CHIP)

set(RTOS_CHIP "stm32h563" CACHE STRING "Chip of this build tree")
set_property(CACHE RTOS_CHIP PROPERTY STRINGS stm32h563 stm32g431)
if(RTOS_CHIP STREQUAL "stm32h563")
    set(RTOS_CPU_FLAGS "-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")
elseif(RTOS_CHIP STREQUAL "stm32g431")
    set(RTOS_CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
else()
    message(FATAL_ERROR "RTOS_CHIP must be stm32h563 or stm32g431 (is '${RTOS_CHIP}')")
endif()

set(CMAKE_C_COMPILER "${RTOS_GCC_ROOT}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "${RTOS_GCC_ROOT}/bin/arm-none-eabi-g++.exe")
set(CMAKE_OBJCOPY "${RTOS_GCC_ROOT}/bin/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE "${RTOS_GCC_ROOT}/bin/arm-none-eabi-size.exe")

set(CMAKE_C_FLAGS_INIT "${RTOS_CPU_FLAGS} -g3 -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT
    "${RTOS_CPU_FLAGS} -g3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit")
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${RTOS_CPU_FLAGS} -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--no-warn-rwx-segments")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
