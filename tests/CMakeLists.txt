include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP ON)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(rtos_tests
    kernel_test.cpp
    block_pool_test.cpp
    sync_test.cpp
    net_test.cpp
    net_services_test.cpp
    dhcp_dns_test.cpp
    tcp_server_test.cpp
    dhcp_server_test.cpp
    tls_test.cpp
    http_ws_test.cpp
    fuzz_test.cpp
    https_fairness_test.cpp
    modbus_test.cpp
    usb_device_test.cpp
    usb_cdc_test.cpp
    usb_ncm_test.cpp
    usb_fuzz_test.cpp
    opcua_test.cpp)
target_link_libraries(rtos_tests PRIVATE rtos_kernel rtos_tls rtos_opcua rtos_certs GTest::gtest_main)
target_include_directories(rtos_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
# UBSan in trap mode needs no runtime library; static linking makes the test
# executable independent of the toolchain's DLLs.
target_compile_options(rtos_tests PRIVATE -Wall -Wextra -Wpedantic -Werror
                                          -fsanitize=undefined -fsanitize-trap=undefined)
target_link_options(rtos_tests PRIVATE -static)

include(GoogleTest)
gtest_discover_tests(rtos_tests)
