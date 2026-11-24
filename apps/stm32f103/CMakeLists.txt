# STM32F103 examples (RTOS_CHIP = stm32f103, preset arm-f103): the kernel self-test on the bare
# chip and the compile check of all its drivers. Boards come with the products.
rtos_add_firmware(f103_selftest SOURCES ../selftest/main.cpp ../selftest/stm32f103/target.cpp
                  LIBRARIES rtos_stm32f103)
target_include_directories(f103_selftest PRIVATE ../selftest ../selftest/stm32f103)

# compile check of all chip drivers (instantiated with a 72 MHz clock tree, not linked)
add_library(rtos_f103_drivers_check OBJECT drivers_check.cpp)
target_link_libraries(rtos_f103_drivers_check PRIVATE rtos_stm32f103)

# ---- self-test in the emulators (ctest --preset arm-f103) ------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(RTOS_RUNNER ${PROJECT_SOURCE_DIR}/emulation/run.py)
if(EXISTS "${RTOS_QEMU}")
    add_test(NAME f103_selftest_qemu
             COMMAND Python3::Interpreter ${RTOS_RUNNER} qemu $<TARGET_FILE:f103_selftest> --platform stm32f103
                     --exe ${RTOS_QEMU})
endif()
if(EXISTS "${RTOS_RENODE}")
    add_test(NAME f103_selftest_renode
             COMMAND Python3::Interpreter ${RTOS_RUNNER} renode $<TARGET_FILE:f103_selftest> --platform stm32f103
                     --exe ${RTOS_RENODE})
endif()
