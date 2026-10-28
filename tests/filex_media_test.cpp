// SPDX-License-Identifier: MIT
//
// FileX on a RAM disk through rtos::storage::FileXMedia: formatting (FAT16, FAT32), files and
// directories across a remount, a disk with a partition table, and FileX's direct transfers
// into unaligned caller buffers (the RAM disk refuses unaligned buffers like the SDMMC's IDMA).
#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "rtos/storage/filex_media.hpp"

namespace {

using namespace rtos::storage;

struct RamDisk {
    std::vector<std::byte> data;
    int unaligned_requests = 0;

    explicit RamDisk(std::uint64_t blocks) : data(blocks * sd_block_size) {}

    SdStatus read(std::uint64_t block, std::span<std::byte> out) noexcept {
        if (reinterpret_cast<std::uintptr_t>(out.data()) % 4 != 0) {
            ++unaligned_requests;
            return SdStatus::CardError;
        }
        if ((block + out.size() / sd_block_size) * sd_block_size > data.size()) {
            return SdStatus::CardError;
        }
        std::memcpy(out.data(), data.data() + block * sd_block_size, out.size());
        return SdStatus::Ok;
    }
    SdStatus write(std::uint64_t block, std::span<const std::byte> in) noexcept {
        if (reinterpret_cast<std::uintptr_t>(in.data()) % 4 != 0) {
            ++unaligned_requests;
            return SdStatus::CardError;
        }
        if ((block + in.size() / sd_block_size) * sd_block_size > data.size()) {
            return SdStatus::CardError;
        }
        std::memcpy(data.data() + block * sd_block_size, in.data(), in.size());
        return SdStatus::Ok;
    }
    std::uint64_t blocks() const noexcept { return data.size() / sd_block_size; }
};
static_assert(BlockDevice<RamDisk>);

CHAR* name(const char* s) { return const_cast<CHAR*>(s); }

std::vector<std::uint8_t> pattern(std::size_t n, unsigned seed) {
    std::vector<std::uint8_t> v(n);
    for (std::size_t i = 0; i < n; ++i) {
        v[i] = static_cast<std::uint8_t>(i * 13 + seed + (i >> 8));
    }
    return v;
}

void write_file(FX_MEDIA* m, const char* path, const std::vector<std::uint8_t>& content) {
    ASSERT_EQ(fx_file_create(m, name(path)), FX_SUCCESS);
    FX_FILE f{};
    ASSERT_EQ(fx_file_open(m, &f, name(path), FX_OPEN_FOR_WRITE), FX_SUCCESS);
    ASSERT_EQ(fx_file_write(&f, const_cast<std::uint8_t*>(content.data()), static_cast<ULONG>(content.size())),
              FX_SUCCESS);
    ASSERT_EQ(fx_file_close(&f), FX_SUCCESS);
}

std::vector<std::uint8_t> read_file(FX_MEDIA* m, const char* path, std::size_t offset_in_buffer = 0) {
    FX_FILE f{};
    EXPECT_EQ(fx_file_open(m, &f, name(path), FX_OPEN_FOR_READ), FX_SUCCESS);
    std::vector<std::uint8_t> buffer(f.fx_file_current_file_size + offset_in_buffer);
    ULONG got = 0;
    EXPECT_EQ(fx_file_read(&f, buffer.data() + offset_in_buffer, static_cast<ULONG>(buffer.size() - offset_in_buffer), &got),
              FX_SUCCESS);
    EXPECT_EQ(fx_file_close(&f), FX_SUCCESS);
    return {buffer.begin() + static_cast<std::ptrdiff_t>(offset_in_buffer),
            buffer.begin() + static_cast<std::ptrdiff_t>(offset_in_buffer + got)};
}

void round_trip(RamDisk& disk, std::uint32_t sectors_per_cluster, std::uint8_t expected_fat_bits) {
    {
        FileXMedia<RamDisk> volume{disk};
        ASSERT_EQ(volume.format_fat("RTOOS", sectors_per_cluster), FX_SUCCESS);
        ASSERT_EQ(volume.open(), FX_SUCCESS);
        EXPECT_EQ(volume.media()->fx_media_32_bit_FAT ? 32 : (volume.media()->fx_media_12_bit_FAT ? 12 : 16), expected_fat_bits);
        write_file(volume.media(), "hello.txt", {'h', 'e', 'l', 'l', 'o'});
        ASSERT_EQ(fx_directory_create(volume.media(), name("logs")), FX_SUCCESS);
        write_file(volume.media(), "logs/big.bin", pattern(70'000, 3));
        ASSERT_EQ(volume.close(), FX_SUCCESS);
    }
    FileXMedia<RamDisk> volume{disk};  // remount: everything comes from the disk
    ASSERT_EQ(volume.open(), FX_SUCCESS);
    EXPECT_EQ(read_file(volume.media(), "hello.txt"), (std::vector<std::uint8_t>{'h', 'e', 'l', 'l', 'o'}));
    EXPECT_EQ(read_file(volume.media(), "logs/big.bin"), pattern(70'000, 3));
    ULONG64 free_bytes = 0;
    EXPECT_EQ(fx_media_extended_space_available(volume.media(), &free_bytes), FX_SUCCESS);
    EXPECT_GT(free_bytes, 0u);
    EXPECT_EQ(volume.close(), FX_SUCCESS);
}

TEST(FileXMedia, Fat16RoundTripAcrossARemount) {
    RamDisk disk(16 * 1024 * 2);  // 16 MB, 4 KB clusters: 4096 clusters
    round_trip(disk, 8, 16);
    EXPECT_EQ(disk.unaligned_requests, 0);
}

TEST(FileXMedia, Fat32RoundTripAcrossARemount) {
    RamDisk disk(64 * 1024 * 2);  // 64 MB, 512-byte clusters: 131072 clusters
    round_trip(disk, 1, 32);
    EXPECT_EQ(disk.unaligned_requests, 0);
}

TEST(FileXMedia, OpensTheFirstPartitionOfAPartitionedDisk) {
    // a file system made on its own, then placed at sector 2048 behind an MBR
    RamDisk fs(8 * 1024 * 2);
    {
        FileXMedia<RamDisk> v{fs};
        ASSERT_EQ(v.format_fat(), FX_SUCCESS);
        ASSERT_EQ(v.open(), FX_SUCCESS);
        write_file(v.media(), "in_part.txt", pattern(1000, 9));
        ASSERT_EQ(v.close(), FX_SUCCESS);
    }
    constexpr std::uint32_t start = 2048;
    RamDisk disk(start + fs.blocks());
    std::memcpy(disk.data.data() + start * sd_block_size, fs.data.data(), fs.data.size());
    auto* mbr = reinterpret_cast<std::uint8_t*>(disk.data.data());
    std::uint8_t* entry = mbr + 446;
    entry[4] = 0x0E;  // FAT16 LBA
    const std::uint32_t sectors = static_cast<std::uint32_t>(fs.blocks());
    std::memcpy(entry + 8, &start, 4);
    std::memcpy(entry + 12, &sectors, 4);
    mbr[510] = 0x55;
    mbr[511] = 0xAA;
    // the copied boot sector still says "0 hidden sectors": FileX takes the offset from the driver
    FileXMedia<RamDisk> volume{disk};
    ASSERT_EQ(volume.open(), FX_SUCCESS);
    EXPECT_EQ(volume.partition_start(), start);
    write_file(volume.media(), "new.txt", pattern(600, 1));
    EXPECT_EQ(read_file(volume.media(), "in_part.txt"), pattern(1000, 9));
    ASSERT_EQ(volume.close(), FX_SUCCESS);
    EXPECT_EQ(mbr[510], 0x55) << "the MBR stays untouched";
    EXPECT_EQ(mbr[446 + 4], 0x0E);
    std::uint32_t still_start = 0;
    std::memcpy(&still_start, mbr + 446 + 8, 4);
    EXPECT_EQ(still_start, start);
}

TEST(FileXMedia, UnalignedCallerBuffersGoThroughTheBounceSector) {
    RamDisk disk(16 * 1024 * 2);
    FileXMedia<RamDisk> volume{disk};
    ASSERT_EQ(volume.format_fat(), FX_SUCCESS);
    ASSERT_EQ(volume.open(), FX_SUCCESS);
    // write from an odd address: whole sectors go directly from the caller's buffer
    const auto content = pattern(64 * 1024, 5);
    std::vector<std::uint8_t> odd(content.size() + 1);
    std::memcpy(odd.data() + 1, content.data(), content.size());
    ASSERT_EQ(fx_file_create(volume.media(), name("odd.bin")), FX_SUCCESS);
    FX_FILE f{};
    ASSERT_EQ(fx_file_open(volume.media(), &f, name("odd.bin"), FX_OPEN_FOR_WRITE), FX_SUCCESS);
    ASSERT_EQ(fx_file_write(&f, odd.data() + 1, static_cast<ULONG>(content.size())), FX_SUCCESS);
    ASSERT_EQ(fx_file_close(&f), FX_SUCCESS);
    ASSERT_EQ(volume.flush(), FX_SUCCESS);
    EXPECT_EQ(read_file(volume.media(), "odd.bin", 3), content) << "read into an odd address";
    EXPECT_EQ(disk.unaligned_requests, 0) << "the device never saw an unaligned buffer";
    ASSERT_EQ(volume.close(), FX_SUCCESS);
}

}  // namespace
