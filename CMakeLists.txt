cmake_minimum_required(VERSION 3.28)

# Two build flavours from one tree, both GCC 16 (see CMakePresets.json):
#   cmake --preset host  -> GoogleTest unit tests
#   cmake --preset arm   -> STM32H563 image
#
# As a subproject of a product (add_subdirectory, see docs/integration.md) rtoos provides
# its libraries (rtos_kernel, rtos_tls, rtos_opcua, rtos_wolfssl, rtos_filex, rtos_littlefs, the chip and board
# packages, rtos_add_firmware, rtos_certificates) and, with RTOS_BUILD_EXAMPLES, its own
# example firmware and tests.
project(rtoos LANGUAGES C CXX)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(rtos_top_level ON)
else()
    set(rtos_top_level OFF)
endif()
option(RTOS_BUILD_EXAMPLES "Build the rtoos example firmware (apps/) and host tests (tests/)" ${rtos_top_level})

if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_VERSION VERSION_LESS 16)
    message(FATAL_ERROR "rtoos requires GCC 16 or newer (found ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION})")
endif()

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# C++26 contracts. Evaluation semantic: enforce (host: reports via the default
# handler) | quick_enforce (target: trap, no library code) | observe | ignore.
set(RTOS_CONTRACT_SEMANTIC "enforce" CACHE STRING "C++26 contract evaluation semantic")
# GCC 16.2 bug: noexcept + pre()/post() + -fno-exceptions ICEs or miscompiles once
# anything with a destructor is involved (RAII locals, by-value parameters).
# Workaround (in the code, see CLAUDE.md): noexcept functions state their
# preconditions with contract_assert() at the top of the body instead of pre().
# At link time -fcontracts pulls in libstdc++exp (default violation handler).

# ---- portable kernel (header-only) ------------------------------------------
# Every user of the headers needs the contract flags: they are part of the target.
add_library(rtos_kernel INTERFACE)
target_include_directories(rtos_kernel INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(rtos_kernel INTERFACE $<$<COMPILE_LANGUAGE:CXX>:-fcontracts>
                                             $<$<COMPILE_LANGUAGE:CXX>:-fcontract-evaluation-semantic=${RTOS_CONTRACT_SEMANTIC}>)
target_link_options(rtos_kernel INTERFACE -fcontracts)

# ---- third party ------------------------------------------------------------
include(cmake/wolfssl.cmake)
include(cmake/filex.cmake)
include(cmake/littlefs.cmake)

# ---- certificates (tools/make_certs.py, checked at every configure) -------------
# RTOS_CA_CERT / RTOS_CA_KEY: the CA (tools/make_ca.py) that signs the device certificates;
# empty or missing: a local test CA in the certificate directory is used instead.
set(RTOS_CA_CERT "$ENV{RTOS_CA_CERT}" CACHE FILEPATH "CA certificate (PEM) for the device certificates")
set(RTOS_CA_KEY "$ENV{RTOS_CA_KEY}" CACHE FILEPATH "CA private key (PEM); only read by openssl")
find_program(RTOS_OPENSSL openssl PATHS ${RTOS_GCC_ROOT}/bin NO_DEFAULT_PATH REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(RTOS_TOOLS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tools)

# rtos_certificates(<target> DIR <dir> [HOST <name.local>] [IP <a.b.c.d>] [OPCUA_URI <uri>])
# Creates/refreshes the HTTPS (ECDSA P-256) and OPC UA (RSA-2048) certificates of a device in
# <dir> (development only: the keys are not encrypted - keep <dir> out of version control)
# and an INTERFACE target <target> whose users can #embed "server.der", "server.key.der",
# "opcua.der", "opcua.key.der" and "ca.der".
function(rtos_certificates target)
    cmake_parse_arguments(C "" "DIR;HOST;IP;OPCUA_URI" "" ${ARGN})
    set(args)
    if(RTOS_CA_CERT AND EXISTS "${RTOS_CA_CERT}" AND EXISTS "${RTOS_CA_KEY}")
        list(APPEND args --ca-cert "${RTOS_CA_CERT}" --ca-key "${RTOS_CA_KEY}")
    elseif(RTOS_CA_CERT)
        message(WARNING "RTOS_CA_CERT/RTOS_CA_KEY not found: using a local test CA in ${C_DIR}")
    endif()
    if(C_HOST)
        list(APPEND args --host ${C_HOST})
    endif()
    if(C_IP)
        list(APPEND args --ip ${C_IP})
    endif()
    if(C_OPCUA_URI)
        list(APPEND args --opcua-uri ${C_OPCUA_URI})
    endif()
    execute_process(COMMAND ${Python3_EXECUTABLE} -I ${RTOS_TOOLS_DIR}/make_certs.py ${RTOS_OPENSSL} ${C_DIR} ${args}
                    COMMAND_ERROR_IS_FATAL ANY)
    add_library(${target} INTERFACE)
    target_compile_options(${target} INTERFACE --embed-dir=${C_DIR})
endfunction()

add_library(rtos_tls STATIC src/net/tls.cpp src/net/tls_memory.cpp src/net/websocket.cpp)
target_link_libraries(rtos_tls PUBLIC rtos_kernel rtos_wolfssl)

# ---- OPC UA (the FCU's opcua_native library; crypto: wolfCrypt RSA-2048) ----------
add_library(rtos_opcua STATIC
    src/opcua/application.cpp src/opcua/clock.cpp src/opcua/codec.cpp src/opcua/connection.cpp src/opcua/secure_channel.cpp
    src/opcua/security_crypto.cpp src/opcua/service_header.cpp src/opcua/service_structs.cpp
    src/opcua/services.cpp src/opcua/session.cpp src/opcua/subscriptions.cpp src/opcua/tcp_message.cpp
    src/opcua/variant.cpp)
target_link_libraries(rtos_opcua PUBLIC rtos_kernel rtos_wolfssl)

# the example device of the rtoos examples and tests: fcu.local
if(RTOS_BUILD_EXAMPLES)
    set(RTOS_CERT_DIR ${PROJECT_SOURCE_DIR}/certs)
    rtos_certificates(rtos_certs DIR ${RTOS_CERT_DIR})
endif()

if(CMAKE_CROSSCOMPILING)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall> $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
                        $<$<COMPILE_LANGUAGE:CXX>:-Werror>)
    add_subdirectory(port/cortex_m33)
    add_subdirectory(port/stm32h563)
    add_subdirectory(boards/fcu)
    if(RTOS_BUILD_EXAMPLES)
        enable_testing()
        add_subdirectory(apps)
    endif()
elseif(RTOS_BUILD_EXAMPLES)
    enable_testing()
    add_subdirectory(tests)
endif()
