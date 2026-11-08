rtos_add_firmware(fcu_demo SOURCES demo/main.cpp LIBRARIES rtos_board_fcu)
rtos_add_firmware(fcu_selftest SOURCES selftest/main.cpp selftest/fcu/target.cpp LIBRARIES rtos_board_fcu)
target_include_directories(fcu_selftest PRIVATE selftest selftest/fcu)
rtos_add_firmware(fcu_bench SOURCES bench/main.cpp LIBRARIES rtos_board_fcu rtos_tls)
rtos_add_firmware(fcu_stepper SOURCES stepper/main.cpp LIBRARIES rtos_board_fcu)
rtos_add_firmware(fcu_sdcard SOURCES sdcard/main.cpp LIBRARIES rtos_board_fcu rtos_filex)
rtos_add_firmware(fcu_hx711 SOURCES hx711/main.cpp LIBRARIES rtos_board_fcu)
rtos_add_firmware(fcu_i2c SOURCES i2c/main.cpp LIBRARIES rtos_board_fcu)
rtos_add_firmware(fcu_exti SOURCES exti/main.cpp LIBRARIES rtos_board_fcu)
# littlefs in the last 256 KB of bank 2: the image must stay below it
rtos_add_firmware(fcu_flashfs SOURCES flashfs/main.cpp LIBRARIES rtos_board_fcu rtos_littlefs
                  FLASH_DATA_START 0x081C0000)
# compile check of all chip drivers (instantiated with the FCU's clocks, not linked)
add_library(rtos_drivers_check OBJECT drivers_check.cpp)
target_link_libraries(rtos_drivers_check PRIVATE rtos_board_fcu)

add_subdirectory(net)  # fcu_net and net_renode

# ---- self-test in the emulators (ctest --preset arm) ------------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(RTOS_RUNNER ${PROJECT_SOURCE_DIR}/emulation/run.py)
if(EXISTS "${RTOS_QEMU}")
    add_test(NAME selftest_qemu
             COMMAND Python3::Interpreter ${RTOS_RUNNER} qemu $<TARGET_FILE:fcu_selftest> --exe ${RTOS_QEMU})
endif()
if(EXISTS "${RTOS_RENODE}")
    # Renode does not enforce the ARMv8-M stack limit registers (PSPLIM).
    add_test(NAME selftest_renode
             COMMAND Python3::Interpreter ${RTOS_RUNNER} renode $<TARGET_FILE:fcu_selftest> --exe ${RTOS_RENODE}
                     --allow-unsupported "stack limit")
    # Sigmoid stepper ramp measured on the STEP pin (virtual time of every edge).
    add_test(NAME stepper_renode
             COMMAND Python3::Interpreter -I ${PROJECT_SOURCE_DIR}/emulation/stepper_test.py
                     $<TARGET_FILE:fcu_stepper> --exe ${RTOS_RENODE})
    set_tests_properties(stepper_renode PROPERTIES TIMEOUT 180)
    # SD card: driver and protocol against the SDMMC + card model (SDHC and SDSC images).
    add_test(NAME sdcard_renode
             COMMAND Python3::Interpreter -I ${PROJECT_SOURCE_DIR}/emulation/sdcard_test.py
                     $<TARGET_FILE:fcu_sdcard> --exe ${RTOS_RENODE})
    set_tests_properties(sdcard_renode PROPERTIES TIMEOUT 180)
    # littlefs in the internal flash against the flash interface model (unlock, sector erase).
    add_test(NAME flashfs_renode
             COMMAND Python3::Interpreter ${RTOS_RUNNER} renode $<TARGET_FILE:fcu_flashfs> --exe ${RTOS_RENODE}
                     --result "FLASHFS RESULT")
    set_tests_properties(flashfs_renode PROPERTIES TIMEOUT 120)
    # HX711 behind the bit-timed SPI model: readings with interrupts during the transfer
    # (PD_SCK on MOSI: a pause with MOSI high > 60 us powers the chip down).
    add_test(NAME hx711_renode
             COMMAND Python3::Interpreter ${RTOS_RUNNER} renode $<TARGET_FILE:fcu_hx711> --exe ${RTOS_RENODE}
                     --result "HX711 RESULT" --timeout 240)
    set_tests_properties(hx711_renode PROPERTIES TIMEOUT 300)
    # Interrupt-driven I2C against the INA226 model (bus timing): results, NACK, and that the
    # waiting thread sleeps while the bytes move.
    add_test(NAME i2c_renode
             COMMAND Python3::Interpreter ${RTOS_RUNNER} renode $<TARGET_FILE:fcu_i2c> --exe ${RTOS_RENODE}
                     --result "I2C RESULT")
    set_tests_properties(i2c_renode PROPERTIES TIMEOUT 120)
    # EXTI: light barrier pulses (shorter than the I/O cycle) and the INA226 conversion-ready
    # ALERT driving the reads.
    add_test(NAME exti_renode
             COMMAND Python3::Interpreter ${RTOS_RUNNER} renode $<TARGET_FILE:fcu_exti> --exe ${RTOS_RENODE}
                     --result "EXTI RESULT")
    set_tests_properties(exti_renode PROPERTIES TIMEOUT 120)
endif()
