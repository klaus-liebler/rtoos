# STM32H743 examples (RTOS_CHIP = stm32h743, preset arm-h743): the kernel self-test on the bare
# chip, the compile check of all its drivers and fcu_net on the FCU revision 3 board.
rtos_add_firmware(h743_selftest SOURCES ../selftest/main.cpp ../selftest/stm32h743/target.cpp
                  LIBRARIES rtos_stm32h743)
target_include_directories(h743_selftest PRIVATE ../selftest ../selftest/stm32h743)

# fcu_net on the Factory Control Unit revision 3 (boards/fcu_rev3) and its network test in Renode
set(RTOS_NET_TEST_ARGS --resc fcu_rev3-net.resc --mcu STM32H743 --mhz 400)
add_subdirectory(../net net)

# compile check of all chip drivers (instantiated with a 400 MHz clock tree, not linked)
add_library(rtos_h743_drivers_check OBJECT drivers_check.cpp)
target_link_libraries(rtos_h743_drivers_check PRIVATE rtos_stm32h743)

# ---- self-test in the emulators (ctest --preset arm-h743) ------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(RTOS_RUNNER ${PROJECT_SOURCE_DIR}/emulation/run.py)
if(EXISTS "${RTOS_QEMU}")
    add_test(NAME h743_selftest_qemu
             COMMAND Python3::Interpreter ${RTOS_RUNNER} qemu $<TARGET_FILE:h743_selftest> --platform stm32h743
                     --exe ${RTOS_QEMU})
endif()
if(EXISTS "${RTOS_RENODE}")
    add_test(NAME h743_selftest_renode
             COMMAND Python3::Interpreter ${RTOS_RUNNER} renode $<TARGET_FILE:h743_selftest> --platform stm32h743
                     --exe ${RTOS_RENODE})
endif()
