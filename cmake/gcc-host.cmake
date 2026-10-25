# Toolchain file: GCC 16 for the host (unit tests).
# RTOS_GCC_ROOT is set by CMakePresets.json.
set(CMAKE_C_COMPILER "${RTOS_GCC_ROOT}/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "${RTOS_GCC_ROOT}/bin/g++.exe")
