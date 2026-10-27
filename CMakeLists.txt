cmake_minimum_required(VERSION 3.28)

# Two build flavours from one tree, both GCC 16 (see CMakePresets.json):
#   cmake --preset host  -> GoogleTest unit tests
#   cmake --preset arm   -> STM32H563 image
project(rtoos LANGUAGES C CXX)

if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_VERSION VERSION_LESS 16)
    message(FATAL_ERROR "rtoos requires GCC 16 or newer (found ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION})")
endif()

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# C++26 contracts. Evaluation semantic: enforce (host: reports via the default
# handler) | quick_enforce (target: trap, no library code) | observe | ignore.
set(RTOS_CONTRACT_SEMANTIC "enforce" CACHE STRING "C++26 contract evaluation semantic")
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fcontracts> $<$<COMPILE_LANGUAGE:CXX>:-fcontract-evaluation-semantic=${RTOS_CONTRACT_SEMANTIC}>)
# GCC 16.2 bug: noexcept + pre()/post() + -fno-exceptions ICEs or miscompiles once
# anything with a destructor is involved (RAII locals, by-value parameters).
# Workaround (in the code, see CLAUDE.md): noexcept functions state their
# preconditions with contract_assert() at the top of the body instead of pre().
# At link time -fcontracts pulls in libstdc++exp (default violation handler).
add_link_options(-fcontracts)

# ---- portable kernel (header-only) ------------------------------------------
add_library(rtos_kernel INTERFACE)
target_include_directories(rtos_kernel INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ---- third party ------------------------------------------------------------
include(cmake/wolfssl.cmake)

# ---- TLS: server certificate (tools/make_certs.py, checked at every configure) ----
# RTOS_CA_CERT / RTOS_CA_KEY: the CA (tools/make_ca.py) that signs the server
# certificate; empty or missing: a local test CA in certs/ is used instead.
set(RTOS_CA_CERT "$ENV{RTOS_CA_CERT}" CACHE FILEPATH "CA certificate (PEM) for the server certificate")
set(RTOS_CA_KEY "$ENV{RTOS_CA_KEY}" CACHE FILEPATH "CA private key (PEM); only read by openssl")
set(RTOS_CERT_DIR ${PROJECT_SOURCE_DIR}/certs)
find_program(RTOS_OPENSSL openssl PATHS ${RTOS_GCC_ROOT}/bin NO_DEFAULT_PATH REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(rtos_ca_args)
if(RTOS_CA_CERT AND EXISTS "${RTOS_CA_CERT}" AND EXISTS "${RTOS_CA_KEY}")
    set(rtos_ca_args --ca-cert "${RTOS_CA_CERT}" --ca-key "${RTOS_CA_KEY}")
elseif(RTOS_CA_CERT)
    message(WARNING "RTOS_CA_CERT/RTOS_CA_KEY not found: using the local test CA")
endif()
execute_process(COMMAND ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/tools/make_certs.py ${RTOS_OPENSSL}
                        ${RTOS_CERT_DIR} ${rtos_ca_args} COMMAND_ERROR_IS_FATAL ANY)
add_library(rtos_certs INTERFACE)  # #embed "server.der" / "server.key.der" / "ca.der"
target_compile_options(rtos_certs INTERFACE --embed-dir=${RTOS_CERT_DIR})

add_library(rtos_tls STATIC src/net/tls.cpp src/net/websocket.cpp)
target_link_libraries(rtos_tls PUBLIC rtos_kernel rtos_wolfssl)

# ---- OPC UA (the FCU's opcua_native library; crypto: wolfCrypt RSA-2048) ----------
add_library(rtos_opcua STATIC
    src/opcua/clock.cpp src/opcua/codec.cpp src/opcua/connection.cpp src/opcua/secure_channel.cpp
    src/opcua/security_crypto.cpp src/opcua/service_header.cpp src/opcua/service_structs.cpp
    src/opcua/services.cpp src/opcua/session.cpp src/opcua/subscriptions.cpp src/opcua/tcp_message.cpp
    src/opcua/variant.cpp)
target_link_libraries(rtos_opcua PUBLIC rtos_kernel rtos_wolfssl)

enable_testing()
if(CMAKE_CROSSCOMPILING)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall> $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
                        $<$<COMPILE_LANGUAGE:CXX>:-Werror>)
    add_subdirectory(port/cortex_m33)
    add_subdirectory(port/stm32h563)
    add_subdirectory(boards/fcu)
    add_subdirectory(apps)
else()
    add_subdirectory(tests)
endif()
