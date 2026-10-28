// SPDX-License-Identifier: MIT
//
// FileX (FAT12/16/32) on a block device - an SD card, a RAM disk, anything with 512-byte
// blocks that satisfies BlockDevice.
//
//   rtos::storage::FileXMedia<SdCard<Host>> volume{card};
//   if (volume.open() != FX_SUCCESS) {          // cards come formatted; else:
//       volume.format_fat(); volume.open();
//   }
//   FX_FILE file;
//   fx_file_create(volume.media(), const_cast<CHAR*>("log.txt"));
//   fx_file_open(volume.media(), &file, const_cast<CHAR*>("log.txt"), FX_OPEN_FOR_WRITE);
//   fx_file_write(&file, data, size);
//   fx_file_close(&file);
//   fx_media_flush(volume.media());
//
// FileX runs standalone (third_party/filex/fx_user.h): no locks, so one thread owns a volume.
// The open-source FileX has no exFAT: SDXC cards (exFAT from the factory) need format_fat().
// The driver adds the partition offset itself: a card with a partition table (MBR) is opened
// at its first partition, an unpartitioned one ("superfloppy") at sector 0. The offset comes
// from the MBR, not from the boot sector's hidden-sector count, which need not match it.
// FileX may hand the driver buffers of any alignment (direct reads into the caller's buffer);
// devices that need word-aligned buffers (SDMMC's IDMA) get them through a bounce sector.
#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "fx_api.h"
#include "rtos/storage/sd_card.hpp"

extern "C" UINT _fx_partition_offset_calculate(void* partition_sector, UINT partition, ULONG* partition_start,
                                                ULONG* partition_size);

namespace rtos::storage {

template <class D>
concept BlockDevice = requires(D& d, std::uint64_t block, std::span<std::byte> in, std::span<const std::byte> out) {
    { d.read(block, in) } noexcept -> std::same_as<SdStatus>;
    { d.write(block, out) } noexcept -> std::same_as<SdStatus>;
    { d.blocks() } noexcept -> std::convertible_to<std::uint64_t>;
};

template <BlockDevice Device, std::size_t CacheSectors = 16>
class FileXMedia {
    static_assert(CacheSectors >= 1);

public:
    explicit FileXMedia(Device& device) noexcept : device_(device) {}
    FileXMedia(const FileXMedia&) = delete;
    FileXMedia& operator=(const FileXMedia&) = delete;

    /// Open the file system on the device (its first partition, or the whole device).
    UINT open(const char* name = "volume") noexcept {
        initialize_filex();
        const UINT status = fx_media_open(&media_, const_cast<CHAR*>(name), &driver, this, cache_.data(),
                                          static_cast<ULONG>(cache_.size()));
        open_ = status == FX_SUCCESS;
        return status;
    }

    UINT close() noexcept {
        if (!open_) {
            return FX_SUCCESS;
        }
        open_ = false;
        return fx_media_close(&media_);
    }

    UINT flush() noexcept { return open_ ? fx_media_flush(&media_) : FX_MEDIA_NOT_OPEN; }

    /// Create a FAT file system over the whole device, no partition table. FileX picks FAT12,
    /// 16 or 32 by the cluster count (FAT32 from 65525 clusters on).
    UINT format_fat(const char* label = "RTOOS", std::uint32_t sectors_per_cluster = 8) noexcept {
        initialize_filex();
        partition_start_ = 0;  // the whole device
        return fx_media_format(&media_, &driver, this, cache_.data(), static_cast<ULONG>(cache_.size()),
                               const_cast<CHAR*>(label), 2, 512, 0, static_cast<ULONG>(device_.blocks()),
                               sd_block_size, sectors_per_cluster, 1, 1);
    }

    [[nodiscard]] FX_MEDIA* media() noexcept { return &media_; }
    [[nodiscard]] bool is_open() const noexcept { return open_; }
    [[nodiscard]] std::uint32_t partition_start() const noexcept { return partition_start_; }

private:
    static void initialize_filex() noexcept {
        static bool done = false;
        if (!done) {
            fx_system_initialize();
            done = true;
        }
    }

    // ---- FileX driver ------------------------------------------------------------------------

    static VOID driver(FX_MEDIA* m) {
        auto& self = *static_cast<FileXMedia*>(m->fx_media_driver_info);
        m->fx_media_driver_status = self.request(m) ? FX_SUCCESS : FX_IO_ERROR;
    }

    bool request(FX_MEDIA* m) noexcept {
        auto* buffer = reinterpret_cast<std::byte*>(m->fx_media_driver_buffer);
        switch (m->fx_media_driver_request) {
            case FX_DRIVER_INIT:
            case FX_DRIVER_UNINIT:
            case FX_DRIVER_FLUSH:
            case FX_DRIVER_ABORT:
            case FX_DRIVER_RELEASE_SECTORS: return true;
            case FX_DRIVER_READ:
                return transfer(true, m->fx_media_driver_logical_sector + partition_start_, buffer,
                                m->fx_media_driver_sectors);
            case FX_DRIVER_WRITE:
                return transfer(false, m->fx_media_driver_logical_sector + partition_start_, buffer,
                                m->fx_media_driver_sectors);
            case FX_DRIVER_BOOT_READ: {
                if (!transfer(true, 0, buffer, 1)) {
                    return false;
                }
                // sector 0 is a partition table (MBR) or already the boot sector
                ULONG start = 0, size = 0;
                if (_fx_partition_offset_calculate(buffer, 0, &start, &size) != FX_SUCCESS) {
                    return false;
                }
                partition_start_ = start;
                return start == 0 || transfer(true, start, buffer, 1);
            }
            case FX_DRIVER_BOOT_WRITE: return transfer(false, partition_start_, buffer, 1);
            default: return false;
        }
    }

    /// Whole sectors; through the bounce sector if the buffer is not word aligned.
    bool transfer(bool read, std::uint64_t sector, std::byte* buffer, std::uint64_t count) noexcept {
        if (reinterpret_cast<std::uintptr_t>(buffer) % 4 == 0) {
            const std::span<std::byte> data(buffer, static_cast<std::size_t>(count * sd_block_size));
            return (read ? device_.read(sector, data) : device_.write(sector, data)) == SdStatus::Ok;
        }
        for (std::uint64_t i = 0; i < count; ++i) {
            std::byte* part = buffer + i * sd_block_size;
            if (read) {
                if (device_.read(sector + i, bounce_) != SdStatus::Ok) {
                    return false;
                }
                std::copy(bounce_.begin(), bounce_.end(), part);
            } else {
                std::copy(part, part + sd_block_size, bounce_.begin());
                if (device_.write(sector + i, bounce_) != SdStatus::Ok) {
                    return false;
                }
            }
        }
        return true;
    }

    Device& device_;
    FX_MEDIA media_{};
    bool open_ = false;
    std::uint32_t partition_start_ = 0;
    alignas(4) std::array<UCHAR, CacheSectors * sd_block_size> cache_{};
    alignas(4) std::array<std::byte, sd_block_size> bounce_{};
};

}  // namespace rtos::storage
