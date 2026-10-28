# littlefs 2.11.3 (BSD-3-Clause): a fail-safe file system for raw flash.
#
# Source archive: GitHub tag v2.11.3, pinned by SHA-256. Configured without heap and without
# debug output; asserts off (they would pull newlib's stdio into the image).
include(FetchContent)
FetchContent_Declare(
    littlefs
    URL https://github.com/littlefs-project/littlefs/archive/refs/tags/v2.11.3.tar.gz
    URL_HASH SHA256=15521be8aaed133c453e7337714e029a9f62480ab41f59c4ba5d18297a076207
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    SOURCE_SUBDIR none)
FetchContent_MakeAvailable(littlefs)

add_library(rtos_littlefs STATIC ${littlefs_SOURCE_DIR}/lfs.c ${littlefs_SOURCE_DIR}/lfs_util.c)
target_compile_definitions(rtos_littlefs PUBLIC LFS_NO_MALLOC LFS_NO_DEBUG LFS_NO_WARN LFS_NO_ERROR LFS_NO_ASSERT)
target_include_directories(rtos_littlefs SYSTEM PUBLIC ${littlefs_SOURCE_DIR})
set_target_properties(rtos_littlefs PROPERTIES C_STANDARD 17 C_EXTENSIONS ON)
# Third-party code: not subject to the project's -Werror.
target_compile_options(rtos_littlefs PRIVATE -w)
