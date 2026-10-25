# Toolchain file: arm-none-eabi GCC 16 for Cortex-M33 + FPv5-SP.
# RTOS_GCC_ROOT is set by CMakePresets.json.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER "${RTOS_GCC_ROOT}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "${RTOS_GCC_ROOT}/bin/arm-none-eabi-g++.exe")
set(CMAKE_OBJCOPY "${RTOS_GCC_ROOT}/bin/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE "${RTOS_GCC_ROOT}/bin/arm-none-eabi-size.exe")

set(RTOS_CPU_FLAGS "-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT "${RTOS_CPU_FLAGS} -g3 -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT
    "${RTOS_CPU_FLAGS} -g3 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit")
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${RTOS_CPU_FLAGS} -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--no-warn-rwx-segments")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
