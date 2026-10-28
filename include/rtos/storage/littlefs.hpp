// SPDX-License-Identifier: MIT
//
// littlefs on raw flash: a fail-safe file system (power loss at any point leaves the last
// committed state) with wear levelling, for settings, logs and recipes in the MCU's own flash.
//
//   using DataFlash = flash::Region<0x081C'0000, 256 * 1024, board::sysclk_hz>;   // 32 sectors
//   DataFlash region;
//   rtos::storage::LittleFs<DataFlash> fs{region};
//   if (fs.mount() != LFS_ERR_OK) { fs.format(); fs.mount(); }
//   rtos::storage::LittleFs<DataFlash>::File f;
//   fs.open(f, "mission.bin", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
//   lfs_file_write(fs.get(), f.get(), data, size);
//   lfs_file_close(fs.get(), f.get());
//
// No heap: littlefs runs without malloc (cmake/littlefs.cmake); the read, program and
// lookahead buffers live in the LittleFs object, a file's cache in its File object. One
// thread owns a LittleFs (littlefs has no locks).
#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "lfs.h"

namespace rtos::storage {

/// Raw NOR flash as littlefs needs it: erase blocks, program units, read anywhere.
template <class F>
concept FlashMemory = requires(F& f, std::uint32_t block, std::uint32_t offset, std::span<std::byte> in,
                               std::span<const std::byte> out) {
    { F::block_bytes } -> std::convertible_to<std::uint32_t>;    // erase unit
    { F::block_count } -> std::convertible_to<std::uint32_t>;
    { F::program_bytes } -> std::convertible_to<std::uint32_t>;  // program unit
    { f.read(block, offset, in) } noexcept -> std::same_as<bool>;
    { f.program(block, offset, out) } noexcept -> std::same_as<bool>;
    { f.erase(block) } noexcept -> std::same_as<bool>;
};

template <FlashMemory Flash, std::size_t CacheBytes = 512, std::size_t LookaheadBytes = 16>
class LittleFs {
    static_assert(CacheBytes % Flash::program_bytes == 0 && Flash::block_bytes % CacheBytes == 0,
                  "cache: a multiple of the program unit and a divisor of the block size");
    static_assert(LookaheadBytes % 8 == 0, "lookahead: a multiple of 8 bytes");

public:
    /// A file with its own cache (lfs_file_opencfg: no malloc).
    class File {
    public:
        [[nodiscard]] lfs_file_t* get() noexcept { return &file_; }

    private:
        friend class LittleFs;
        lfs_file_t file_{};
        lfs_file_config config_{};
        alignas(4) std::array<std::uint8_t, CacheBytes> cache_{};
    };

    explicit LittleFs(Flash& flash, std::uint32_t block_cycles = 500) noexcept : flash_(flash) {
        config_.context = this;
        config_.read = &read;
        config_.prog = &prog;
        config_.erase = &erase;
        config_.sync = &sync;
        config_.read_size = Flash::program_bytes;
        config_.prog_size = Flash::program_bytes;
        config_.block_size = Flash::block_bytes;
        config_.block_count = Flash::block_count;
        config_.block_cycles = static_cast<std::int32_t>(block_cycles);
        config_.cache_size = CacheBytes;
        config_.lookahead_size = LookaheadBytes;
        config_.read_buffer = read_buffer_.data();
        config_.prog_buffer = prog_buffer_.data();
        config_.lookahead_buffer = lookahead_buffer_.data();
    }
    LittleFs(const LittleFs&) = delete;
    LittleFs& operator=(const LittleFs&) = delete;

    int format() noexcept { return lfs_format(&lfs_, &config_); }
    int mount() noexcept {
        const int r = lfs_mount(&lfs_, &config_);
        mounted_ = r == LFS_ERR_OK;
        return r;
    }
    int unmount() noexcept {
        if (!mounted_) {
            return LFS_ERR_OK;
        }
        mounted_ = false;
        return lfs_unmount(&lfs_);
    }

    /// Open `path` with `flags` (LFS_O_*) using the File's own cache.
    int open(File& f, const char* path, int flags) noexcept {
        f.config_ = {};
        f.config_.buffer = f.cache_.data();
        return lfs_file_opencfg(&lfs_, &f.file_, path, flags, &f.config_);
    }

    [[nodiscard]] lfs_t* get() noexcept { return &lfs_; }
    [[nodiscard]] bool mounted() const noexcept { return mounted_; }

private:
    static LittleFs& self(const lfs_config* c) noexcept { return *static_cast<LittleFs*>(c->context); }

    static int read(const lfs_config* c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size) {
        return self(c).flash_.read(block, off, {static_cast<std::byte*>(buffer), size}) ? LFS_ERR_OK : LFS_ERR_IO;
    }
    static int prog(const lfs_config* c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size) {
        return self(c).flash_.program(block, off, {static_cast<const std::byte*>(buffer), size}) ? LFS_ERR_OK
                                                                                                  : LFS_ERR_IO;
    }
    static int erase(const lfs_config* c, lfs_block_t block) {
        return self(c).flash_.erase(block) ? LFS_ERR_OK : LFS_ERR_IO;
    }
    static int sync(const lfs_config*) { return LFS_ERR_OK; }

    Flash& flash_;
    lfs_t lfs_{};
    lfs_config config_{};
    bool mounted_ = false;
    alignas(4) std::array<std::uint8_t, CacheBytes> read_buffer_{};
    alignas(4) std::array<std::uint8_t, CacheBytes> prog_buffer_{};
    alignas(4) std::array<std::uint8_t, LookaheadBytes> lookahead_buffer_{};
};

}  // namespace rtos::storage
