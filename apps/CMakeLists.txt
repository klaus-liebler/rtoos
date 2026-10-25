rtos_add_firmware(fcu_demo SOURCES demo/main.cpp LIBRARIES rtos_board_fcu)
rtos_add_firmware(fcu_selftest SOURCES selftest/main.cpp LIBRARIES rtos_board_fcu)
rtos_add_firmware(fcu_net SOURCES net/main.cpp LIBRARIES rtos_board_fcu rtos_tls rtos_certs)
rtos_add_firmware(fcu_bench SOURCES bench/main.cpp LIBRARIES rtos_board_fcu rtos_tls)

# ---- web assets for fcu_net: brotli-compressed app + 100 KB throughput test blob ----
find_program(RTOS_BROTLI brotli PATHS ${RTOS_GCC_ROOT}/bin NO_DEFAULT_PATH REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(RTOS_WEB_OUT ${CMAKE_BINARY_DIR}/web)
add_custom_command(
    OUTPUT ${RTOS_WEB_OUT}/index.html.br
    COMMAND ${CMAKE_COMMAND} -E make_directory ${RTOS_WEB_OUT}
    COMMAND ${RTOS_BROTLI} --force --quality=11 --output=${RTOS_WEB_OUT}/index.html.br ${PROJECT_SOURCE_DIR}/web/index.html
    DEPENDS ${PROJECT_SOURCE_DIR}/web/index.html
    COMMENT "brotli web/index.html")
add_custom_command(
    OUTPUT ${RTOS_WEB_OUT}/blob.bin
    COMMAND ${CMAKE_COMMAND} -E make_directory ${RTOS_WEB_OUT}
    COMMAND ${Python3_EXECUTABLE} -I -c "import random; random.seed(1); open(r'${RTOS_WEB_OUT}/blob.bin','wb').write(random.randbytes(102400))"
    COMMENT "100 KB test blob")
add_custom_target(rtos_web_assets DEPENDS ${RTOS_WEB_OUT}/index.html.br ${RTOS_WEB_OUT}/blob.bin)
add_dependencies(fcu_net rtos_web_assets)
target_compile_options(fcu_net PRIVATE --embed-dir=${RTOS_WEB_OUT})

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
    # End-to-end network test of fcu_net over the TAP adapter (ping, mDNS, SNTP, HTTPS,
    # WebSocket). Skipped (exit code 77) if emulation/renode/setup-tap.ps1 never ran.
    add_test(NAME net_renode
             COMMAND Python3::Interpreter -I ${PROJECT_SOURCE_DIR}/emulation/net_test.py
                     $<TARGET_FILE:fcu_net> ${RTOS_WEB_OUT} ${RTOS_CERT_DIR}/ca.crt ${RTOS_RENODE})
    set_tests_properties(net_renode PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300 RUN_SERIAL TRUE)
endif()
