# wolfSSL 5.9.4 (GPLv3 or commercial license from wolfSSL Inc.)
#
# Source archive: GitHub tag v5.9.4-stable. Its OpenPGP signature
# (wolfssl-5.9.4-stable.tar.gz.asc) was verified against the wolfSSL release key
# A2A4 8E7B CB96 C5BE CB98 7314 EBC8 0E41 5CA2 9677 "wolfSSL <secure@wolfssl.com>"
# (the same key signed 5.7.0, 5.8.0, 5.9.0); the build pins the archive by SHA-256.
#
# Like wolfSSL's IDE projects, all wolfCrypt sources are compiled; user_settings.h
# selects the features (every source is guarded by them) and --gc-sections drops
# what is not used.
include(FetchContent)
FetchContent_Declare(
    wolfssl
    URL https://github.com/wolfSSL/wolfssl/archive/refs/tags/v5.9.4-stable.tar.gz
    URL_HASH SHA256=7256bfc89b183a75183806c7debfa203443873b0b4a562e1b80d68e01b45ac57
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    SOURCE_SUBDIR none)  # do not add wolfSSL's own CMakeLists
FetchContent_MakeAvailable(wolfssl)

set(W ${wolfssl_SOURCE_DIR})
file(GLOB RTOS_WOLFCRYPT_SOURCES ${W}/wolfcrypt/src/*.c)
# #include'd by other sources (compiling them separately is an error)
list(REMOVE_ITEM RTOS_WOLFCRYPT_SOURCES ${W}/wolfcrypt/src/misc.c ${W}/wolfcrypt/src/evp.c ${W}/wolfcrypt/src/evp_pk.c)
set(RTOS_WOLFSSL_SOURCES ${W}/src/internal.c ${W}/src/keys.c ${W}/src/ssl.c ${W}/src/tls.c ${W}/src/tls13.c ${W}/src/wolfio.c)
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    file(GLOB RTOS_WOLFSSL_THUMB2 ${W}/wolfcrypt/src/port/arm/thumb2-*_c.c)
endif()

add_library(rtos_wolfssl STATIC ${RTOS_WOLFSSL_SOURCES} ${RTOS_WOLFCRYPT_SOURCES} ${RTOS_WOLFSSL_THUMB2})
target_compile_definitions(rtos_wolfssl PUBLIC WOLFSSL_USER_SETTINGS)
target_include_directories(rtos_wolfssl SYSTEM PUBLIC ${W} ${CMAKE_CURRENT_LIST_DIR}/../third_party/wolfssl)
set_target_properties(rtos_wolfssl PROPERTIES C_STANDARD 17 C_EXTENSIONS ON)
# Third-party code: not subject to the project's -Werror.
target_compile_options(rtos_wolfssl PRIVATE -w)
