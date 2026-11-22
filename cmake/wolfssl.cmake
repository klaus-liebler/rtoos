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

# ---- patch: OAEP frees stack buffers (wolfSSL 5.9.4) ---------------------------------------------
# With WOLFSSL_SMALL_STACK + WOLFSSL_NO_MALLOC (user_settings.h) RsaPad_OAEP and RsaUnPad_OAEP declare
# their temporaries on the stack (#if SMALL_STACK && !NO_MALLOC) but free them under a weaker
# condition - WC_FREE_VAR_EX and "#ifdef WOLFSSL_SMALL_STACK" XFREE only look at SMALL_STACK: XFREE of
# a stack address. Under WOLFSSL_STATIC_MEMORY that takes the bytes before the array as a block
# header, counts a free and on a size match links the stack address into the pool's free list.
# OPC UA runs both for every secured OpenSecureChannel. Found by the ASan host tests
# (docs/fuzzing.md); regression test OpcUaTest.OaepFreesOnlyWhatItAllocated. No other function of
# wolfSSL 5.9.4 has the pattern (checked with a scan of all sources).
# The build compiles a patched copy of rsa.c: inside these two functions the frees get the
# declaration's condition. Remove when wolfSSL does that itself (configure stops if the text changes).
file(READ ${W}/wolfcrypt/src/rsa.c rtos_rsa)
# rtos_oaep_patch(<function> <WC_FREE_VAR_EX count> <#ifdef WOLFSSL_SMALL_STACK count>)
function(rtos_oaep_patch fn frees ifdefs)
    string(FIND "${rtos_rsa}" "\nstatic int ${fn}(" start)
    if(start EQUAL -1)
        message(FATAL_ERROR "wolfSSL rsa.c changed (no ${fn}): check the OAEP patch in cmake/wolfssl.cmake")
    endif()
    string(SUBSTRING "${rtos_rsa}" ${start} -1 rest)
    string(FIND "${rest}" "\n}\n" end)
    math(EXPR end "${end} + 3")
    string(SUBSTRING "${rest}" 0 ${end} body)
    string(SUBSTRING "${rest}" ${end} -1 after)
    string(SUBSTRING "${rtos_rsa}" 0 ${start} before)
    string(REGEX MATCHALL "WC_FREE_VAR_EX\\(" f "${body}")
    string(REGEX MATCHALL "#ifdef WOLFSSL_SMALL_STACK\n" d "${body}")
    list(LENGTH f nf)
    list(LENGTH d nd)
    if(NOT nf EQUAL frees OR NOT nd EQUAL ifdefs)
        message(FATAL_ERROR "wolfSSL rsa.c changed (${fn}: ${nf} WC_FREE_VAR_EX, ${nd} #ifdef): check the OAEP patch in cmake/wolfssl.cmake")
    endif()
    string(REPLACE "WC_FREE_VAR_EX(" "RTOS_OAEP_FREE(" body "${body}")
    string(REPLACE "#ifdef WOLFSSL_SMALL_STACK\n" "#if defined(WOLFSSL_SMALL_STACK) && !defined(WOLFSSL_NO_MALLOC)\n" body "${body}")
    set(rtos_rsa "${before}${body}${after}" PARENT_SCOPE)
endfunction()
rtos_oaep_patch(RsaPad_OAEP 16 1)
rtos_oaep_patch(RsaUnPad_OAEP 1 2)
# The same configuration in RsaPrivateDecryptEx: with WOLFSSL_NO_MALLOC it decrypts and unpads in
# place in `out`, then copies the message (behind the padding, inside `out`) to the front with
# XMEMCPY - overlapping ranges, undefined (newlib's forward copy happens to cope). ASan:
# memcpy-param-overlap in every OPC UA test with a secured channel (preset linux-asan).
string(REGEX MATCHALL "XMEMCPY\\(out, pad, \\(size_t\\)ret\\)" found "${rtos_rsa}")  # no ';': a list separator
list(LENGTH found n)
if(NOT n EQUAL 1)
    message(FATAL_ERROR "wolfSSL rsa.c changed (${n} in-place copies): check the decrypt patch in cmake/wolfssl.cmake")
endif()
string(REPLACE "XMEMCPY(out, pad, (size_t)ret);" "XMEMMOVE(out, pad, (size_t)ret); /* rtoos: overlapping */"
       rtos_rsa "${rtos_rsa}")
string(REPLACE "\nstatic int RsaPad_OAEP(" "
/* rtoos (cmake/wolfssl.cmake): free the OAEP temporaries only where they were allocated */
#if defined(WOLFSSL_SMALL_STACK) && !defined(WOLFSSL_NO_MALLOC)
    #define RTOS_OAEP_FREE(v, h, t) WC_FREE_VAR_EX(v, h, t)
#else
    #define RTOS_OAEP_FREE(v, h, t) WC_DO_NOTHING
#endif
static int RsaPad_OAEP(" rtos_rsa "${rtos_rsa}")
set(rtos_rsa_patched ${CMAKE_CURRENT_BINARY_DIR}/wolfssl_patched/wolfcrypt/src/rsa.c)
set(old "")
if(EXISTS ${rtos_rsa_patched})
    file(READ ${rtos_rsa_patched} old)
endif()
if(NOT old STREQUAL rtos_rsa)  # unchanged: keep the timestamp (no rebuild)
    file(WRITE ${rtos_rsa_patched} "${rtos_rsa}")
endif()
list(REMOVE_ITEM RTOS_WOLFCRYPT_SOURCES ${W}/wolfcrypt/src/rsa.c)
list(APPEND RTOS_WOLFCRYPT_SOURCES ${rtos_rsa_patched})

add_library(rtos_wolfssl STATIC ${RTOS_WOLFSSL_SOURCES} ${RTOS_WOLFCRYPT_SOURCES} ${RTOS_WOLFSSL_THUMB2})
target_compile_definitions(rtos_wolfssl PUBLIC WOLFSSL_USER_SETTINGS $<$<BOOL:${RTOS_SSH}>:RTOS_SSH>)
target_include_directories(rtos_wolfssl SYSTEM PUBLIC ${W} ${CMAKE_CURRENT_LIST_DIR}/../third_party/wolfssl)
set_target_properties(rtos_wolfssl PROPERTIES C_STANDARD 17 C_EXTENSIONS ON)
# Third-party code: not subject to the project's -Werror.
target_compile_options(rtos_wolfssl PRIVATE -w)
# Host builds under ASan + UBSan (linux-asan, linux-afl): wolfSSL's portable C paths of a 64-bit
# host have undefined behaviour that UBSan stops at - sp_c32.c sp_256_num_bits_29_9 (signed
# overflow in v * 0x07C4ACDD) and aes.c's 64-bit GMULT (byte* read as word64*, misaligned). The
# firmware does not contain either: it links sp_cortexm.c and the Thumb-2 GCM_gmult_len (checked
# in the map file). Only these two checks are off, only for wolfSSL; ASan stays on.
# Remove when wolfSSL's C paths are clean (5.9.4 is not).
if(NOT CMAKE_CROSSCOMPILING AND RTOS_HOST_SANITIZERS STREQUAL "address")
    target_compile_options(rtos_wolfssl PRIVATE -fno-sanitize=alignment,signed-integer-overflow)
endif()
