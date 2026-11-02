# STM32G431 examples (RTOS_CHIP = stm32g431, preset arm-g431): the kernel self-test on the bare
# chip and the compile check of all its drivers. Boards come with the products.
rtos_add_firmware(g431_selftest SOURCES ../selftest/main.cpp ../selftest/stm32g431/target.cpp
                  LIBRARIES rtos_stm32g431)
target_include_directories(g431_selftest PRIVATE ../selftest ../selftest/stm32g431)

# compile check of all chip drivers (instantiated with a 170 MHz clock tree, not linked)
add_library(rtos_g431_drivers_check OBJECT drivers_check.cpp)
target_link_libraries(rtos_g431_drivers_check PRIVATE rtos_stm32g431)

# ---- self-test in the emulators (ctest --preset arm-g431) ------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(RTOS_RUNNER ${PROJECT_SOURCE_DIR}/emulation/run.py)
if(EXISTS "${RTOS_QEMU}")
    add_test(NAME g431_selftest_qemu
             COMMAND Python3::Interpreter ${RTOS_RUNNER} qemu $<TARGET_FILE:g431_selftest> --platform stm32g431
                     --exe ${RTOS_QEMU})
endif()
if(EXISTS "${RTOS_RENODE}")
    add_test(NAME g431_selftest_renode
             COMMAND Python3::Interpreter ${RTOS_RUNNER} renode $<TARGET_FILE:g431_selftest> --platform stm32g431
                     --exe ${RTOS_RENODE})
endif()
