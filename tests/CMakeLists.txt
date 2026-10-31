include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP ON)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(rtos_tests
    kernel_test.cpp
    log_test.cpp
    usb_drd_fs_test.cpp
    usb_pd_test.cpp
    littlefs_test.cpp
    filex_media_test.cpp
    sd_card_test.cpp
    motion_test.cpp
    port_config_test.cpp
    block_pool_test.cpp
    sync_test.cpp
    net_test.cpp
    net_services_test.cpp
    dhcp_dns_test.cpp
    tcp_server_test.cpp
    dhcp_server_test.cpp
    tls_test.cpp
    ssh_test.cpp
    devices_test.cpp
    http_ws_test.cpp
    websocket_outbox_test.cpp
    fuzz_test.cpp
    https_fairness_test.cpp
    modbus_test.cpp
    usb_device_test.cpp
    usb_cdc_test.cpp
    usb_ncm_test.cpp
    usb_fuzz_test.cpp
    opcua_test.cpp
    opcua_o6_view_test.cpp
    opcua_o6_attributes_test.cpp
    opcua_o6_subscriptions_test.cpp
    opcua_codec_test.cpp)
target_link_libraries(rtos_tests PRIVATE rtos_kernel rtos_tls rtos_opcua rtos_filex rtos_littlefs rtos_certs GTest::gtest_main)
# The chip drivers' compile-time configuration (constexpr register values) is tested on the host.
target_include_directories(rtos_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                           ${PROJECT_SOURCE_DIR}/port/stm32h563/include ${PROJECT_SOURCE_DIR}/port/cortex_m33/include)
# UBSan in trap mode needs no runtime library; static linking makes the test
# executable independent of the toolchain's DLLs.
target_compile_options(rtos_tests PRIVATE -Wall -Wextra -Wpedantic -Werror
                                          -fsanitize=undefined -fsanitize-trap=undefined)
target_link_options(rtos_tests PRIVATE -static)

include(GoogleTest)
gtest_discover_tests(rtos_tests)
# ssh_test.cpp includes wolfSSH's headers (see cmake/wolfssh.cmake)
if(RTOS_WOLFSSH_HOST_FLAGS)
    set_source_files_properties(ssh_test.cpp PROPERTIES COMPILE_OPTIONS "${RTOS_WOLFSSH_HOST_FLAGS}")
endif()
