# FileX 6.5.2 (Eclipse ThreadX, MIT license): FAT12/16/32 (no exFAT in the open-source FileX),
# standalone (no ThreadX).
#
# Source archive: GitHub tag v6.5.2.202603_rel, pinned by SHA-256. third_party/filex/fx_user.h
# configures it (FX_STANDALONE_ENABLE: single-threaded, no locks - one thread owns a medium).
include(FetchContent)
FetchContent_Declare(
    filex
    URL https://github.com/eclipse-threadx/filex/archive/refs/tags/v6.5.2.202603_rel.tar.gz
    URL_HASH SHA256=76c3bbcc0cbb2a21652f3afb107bda411184b8e7cceb7fd16d6280e585da6982
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    SOURCE_SUBDIR none)  # do not add FileX's own CMakeLists
FetchContent_MakeAvailable(filex)

file(GLOB RTOS_FILEX_SOURCES ${filex_SOURCE_DIR}/common/src/*.c)
add_library(rtos_filex STATIC ${RTOS_FILEX_SOURCES})
target_compile_definitions(rtos_filex PUBLIC FX_INCLUDE_USER_DEFINE_FILE)
target_include_directories(rtos_filex SYSTEM PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../third_party/filex
                           ${filex_SOURCE_DIR}/common/inc ${filex_SOURCE_DIR}/ports/generic/inc)
set_target_properties(rtos_filex PROPERTIES C_STANDARD 17 C_EXTENSIONS ON)
# Third-party code: not subject to the project's -Werror.
target_compile_options(rtos_filex PRIVATE -w)
