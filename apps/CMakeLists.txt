rtos_add_firmware(fcu_demo SOURCES demo/main.cpp LIBRARIES rtos_board_fcu)
rtos_add_firmware(fcu_selftest SOURCES selftest/main.cpp LIBRARIES rtos_board_fcu)
rtos_add_firmware(fcu_net SOURCES net/main.cpp LIBRARIES rtos_board_fcu rtos_tls rtos_opcua rtos_certs)
rtos_add_firmware(fcu_bench SOURCES bench/main.cpp LIBRARIES rtos_board_fcu rtos_tls)
# compile check of all chip drivers (instantiated with the FCU's clocks, not linked)
add_library(rtos_drivers_check OBJECT drivers_check.cpp)
target_link_libraries(rtos_drivers_check PRIVATE rtos_board_fcu)

# ---- network configuration of fcu_net (generated constexpr header) ----------------
set(RTOS_NET_SNTP_HOST "pool.ntp.org" CACHE STRING "SNTP server name (when DHCP names no NTP server)")
set(RTOS_NET_SNTP_PORT "123" CACHE STRING "SNTP server port")
set(RTOS_NET_WS_PING_MS "20000" CACHE STRING "WebSocket: ping a peer silent for this long (ms)")
set(RTOS_NET_WS_PONG_TIMEOUT_MS "10000" CACHE STRING "WebSocket: close if nothing arrives this long after the ping (ms)")
foreach(var RTOS_NET_WS_PING_MS RTOS_NET_WS_PONG_TIMEOUT_MS)
    if(NOT ${var} MATCHES "^[1-9][0-9]*$")
        message(FATAL_ERROR "${var} must be a positive number of milliseconds")
    endif()
endforeach()
set(RTOS_NET_STATIC_FALLBACK "" CACHE STRING
    "Static address without DHCP server after 10 s, 'a.b.c.d/prefix,gateway'; empty: keep asking DHCP")
if(NOT RTOS_NET_SNTP_PORT MATCHES "^[0-9]+$" OR RTOS_NET_SNTP_PORT LESS 1 OR RTOS_NET_SNTP_PORT GREATER 65535)
    message(FATAL_ERROR "RTOS_NET_SNTP_PORT must be 1..65535, not '${RTOS_NET_SNTP_PORT}'")
endif()
function(rtos_ipv4_value text out)  # dotted quad -> 0x... (validated)
    if(NOT text MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
        message(FATAL_ERROR "not an IPv4 address: '${text}'")
    endif()
    foreach(octet ${CMAKE_MATCH_1} ${CMAKE_MATCH_2} ${CMAKE_MATCH_3} ${CMAKE_MATCH_4})
        if(octet GREATER 255)
            message(FATAL_ERROR "not an IPv4 address: '${text}'")
        endif()
    endforeach()
    math(EXPR value "(${CMAKE_MATCH_1} << 24) | (${CMAKE_MATCH_2} << 16) | (${CMAKE_MATCH_3} << 8) | ${CMAKE_MATCH_4}"
         OUTPUT_FORMAT HEXADECIMAL)
    set(${out} "${value}u" PARENT_SCOPE)
endfunction()
if(RTOS_NET_STATIC_FALLBACK)
    if(NOT RTOS_NET_STATIC_FALLBACK MATCHES "^([0-9.]+)/([0-9]+),([0-9.]+)$" OR CMAKE_MATCH_2 GREATER 30)
        message(FATAL_ERROR "RTOS_NET_STATIC_FALLBACK must look like 192.168.77.2/24,192.168.77.1")
    endif()
    set(prefix ${CMAKE_MATCH_2})
    set(gateway_text ${CMAKE_MATCH_3})
    rtos_ipv4_value(${CMAKE_MATCH_1} rtos_net_fallback_address)
    rtos_ipv4_value(${gateway_text} rtos_net_fallback_gateway)
    math(EXPR rtos_net_fallback_netmask "(0xFFFFFFFF << (32 - ${prefix})) & 0xFFFFFFFF" OUTPUT_FORMAT HEXADECIMAL)
    string(APPEND rtos_net_fallback_netmask "u")
    set(rtos_net_fallback_enabled true)
else()
    set(rtos_net_fallback_address 0u)
    set(rtos_net_fallback_netmask 0u)
    set(rtos_net_fallback_gateway 0u)
    set(rtos_net_fallback_enabled false)
endif()
configure_file(net/fcu_net_config.hpp.in ${CMAKE_BINARY_DIR}/generated/fcu_net_config.hpp @ONLY)
target_include_directories(fcu_net PRIVATE ${CMAKE_BINARY_DIR}/generated)

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
