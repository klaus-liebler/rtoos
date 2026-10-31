# wolfSSH 1.6.0 (GPLv3 or commercial license from wolfSSL Inc.): the SSH server of the log
# terminal (net/ssh.hpp).
#
# Source archive: GitHub tag v1.6.0-stable. Its OpenPGP signature (release asset
# wolfssh-1.6.0-stable.tar.gz.asc) was verified against the wolfSSL release key
# A2A4 8E7B CB96 C5BE CB98 7314 EBC8 0E41 5CA2 9677 "wolfSSL <secure@wolfssl.com>" (the key of
# the wolfSSL archive, cmake/wolfssl.cmake); the build pins the archive by SHA-256.
#
# Built against rtos_wolfssl with its user_settings.h (the wolfSSH block there).
include(FetchContent)
FetchContent_Declare(
    wolfssh
    URL https://github.com/wolfSSL/wolfssh/archive/refs/tags/v1.6.0-stable.tar.gz
    URL_HASH SHA256=e345b580643af6715a4293ed7bab4e99ea562bb379adb8898eeae55c38b12f56
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    SOURCE_SUBDIR none)  # do not add wolfSSH's own CMakeLists
FetchContent_MakeAvailable(wolfssh)

set(S ${wolfssh_SOURCE_DIR})
add_library(rtos_wolfssh STATIC ${S}/src/internal.c ${S}/src/ssh.c ${S}/src/io.c ${S}/src/log.c ${S}/src/port.c ${S}/src/ossh.c)
target_include_directories(rtos_wolfssh SYSTEM PUBLIC ${S})
target_link_libraries(rtos_wolfssh PUBLIC rtos_wolfssl)
set_target_properties(rtos_wolfssh PROPERTIES C_STANDARD 17 C_EXTENSIONS ON)
target_compile_options(rtos_wolfssh PRIVATE -w)  # third-party code: not subject to -Werror
# Host tests on Windows (MinGW): wolfSSH's settings.h selects its Windows API branch for every
# _WIN32 build (no switch like wolfSSL's WOLFSSL_NOT_WINDOWS_API), whose types (SOCKET, HANDLE)
# need the Windows headers. Only the wolfSSH sources and the glue that includes its headers get
# them; the I/O stays rtoos' (WOLFSSH_USER_IO). Remove when wolfSSH can build bare on _WIN32.
# Target: wolfSSH's unused diagnostics (wolfSSH_ShowSizes, DumpOctetString, a FIPS callback)
# call printf/fprintf; the reference alone pulls newlib's stdio and its malloc/sbrk into the
# link (there is no heap). They go to no-op functions (src/net/ssh.cpp). Remove when wolfSSH
# guards them (e.g. by WOLFSSH_NO_STDIO).
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    target_compile_definitions(rtos_wolfssh PRIVATE printf=rtos_wolfssh_printf fprintf=rtos_wolfssh_fprintf)
endif()
if(WIN32 OR CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_compile_options(rtos_wolfssh PRIVATE "SHELL:-include stdio.h" "SHELL:-include winsock2.h")
    set(RTOS_WOLFSSH_HOST_FLAGS -DNOMINMAX -include winsock2.h)
endif()
