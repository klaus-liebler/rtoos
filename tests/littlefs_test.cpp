// SPDX-License-Identifier: MIT
//
// littlefs through rtos::storage::LittleFs on a simulated NOR flash with the STM32H5's rules:
// erase sets a block to 0xFF, a quad-word (16 bytes) can be programmed once after an erase
// (a second program is refused, like PGSERR). Files across a remount, many rewrites in a small
// flash (wear levelling) and power loss at every programming step (the file holds either the
// old or the new content, never a mix).
#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "rtos/storage/littlefs.hpp"

namespace {

template <std::uint32_t Blocks>
struct NorFlash {
    static constexpr std::uint32_t block_bytes = 8192;
    static constexpr std::uint32_t block_count = Blocks;
    static constexpr std::uint32_t program_bytes = 16;

    std::vector<std::uint8_t> cells = std::vector<std::uint8_t>(Blocks * block_bytes, 0xFF);
    std::vector<bool> programmed = std::vector<bool>(Blocks * block_bytes / program_bytes, false);
    std::vector<int> erases = std::vector<int>(Blocks, 0);
    long programs_left = -1;  ///< >= 0: power fails after this many program operations
    int double_programs = 0;

    bool read(std::uint32_t block, std::uint32_t offset, std::span<std::byte> out) noexcept {
        std::memcpy(out.data(), &cells[block * block_bytes + offset], out.size());
        return true;
    }
    bool program(std::uint32_t block, std::uint32_t offset, std::span<const std::byte> data) noexcept {
        if (offset % program_bytes != 0 || data.size() % program_bytes != 0) {
            return false;
        }
        for (std::size_t q = 0; q < data.size(); q += program_bytes) {
            if (programs_left == 0) {
                return false;  // power is gone: nothing more reaches the flash
            }
            if (programs_left > 0) {
                --programs_left;
            }
            const std::size_t at = block * block_bytes + offset + q;
            if (programmed[at / program_bytes]) {
                ++double_programs;
                return false;
            }
            programmed[at / program_bytes] = true;
            std::memcpy(&cells[at], data.data() + q, program_bytes);
        }
        return true;
    }
    bool erase(std::uint32_t block) noexcept {
        if (programs_left == 0) {
            return false;
        }
        std::fill_n(&cells[block * block_bytes], block_bytes, std::uint8_t{0xFF});
        std::fill_n(programmed.begin() + block * (block_bytes / program_bytes), block_bytes / program_bytes, false);
        ++erases[block];
        return true;
    }
};
static_assert(rtos::storage::FlashMemory<NorFlash<8>>);

template <class Fs>
int write_file(Fs& fs, const char* path, const std::string& content) {
    typename Fs::File f;
    if (const int r = fs.open(f, path, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC); r != LFS_ERR_OK) {
        return r;
    }
    const auto n = lfs_file_write(fs.get(), f.get(), content.data(), static_cast<lfs_size_t>(content.size()));
    const int c = lfs_file_close(fs.get(), f.get());
    return n < 0 ? static_cast<int>(n) : c;
}

template <class Fs>
std::string read_file(Fs& fs, const char* path) {
    typename Fs::File f;
    if (fs.open(f, path, LFS_O_RDONLY) != LFS_ERR_OK) {
        return "<missing>";
    }
    std::string s(lfs_file_size(fs.get(), f.get()), '\0');
    lfs_file_read(fs.get(), f.get(), s.data(), static_cast<lfs_size_t>(s.size()));
    lfs_file_close(fs.get(), f.get());
    return s;
}

std::string text(std::size_t n, char seed) {
    std::string s(n, ' ');
    for (std::size_t i = 0; i < n; ++i) {
        s[i] = static_cast<char>('a' + (i * 7 + static_cast<std::size_t>(seed)) % 26);
    }
    return s;
}

TEST(LittleFs, FilesAndDirectoriesSurviveARemount) {
    NorFlash<32> flash;
    {
        rtos::storage::LittleFs<NorFlash<32>> fs{flash};
        EXPECT_NE(fs.mount(), LFS_ERR_OK) << "blank flash";
        ASSERT_EQ(fs.format(), LFS_ERR_OK);
        ASSERT_EQ(fs.mount(), LFS_ERR_OK);
        ASSERT_EQ(lfs_mkdir(fs.get(), "missions"), LFS_ERR_OK);
        ASSERT_EQ(write_file(fs, "missions/pick.bin", text(20'000, 1)), LFS_ERR_OK);
        ASSERT_EQ(write_file(fs, "settings", "speed=1000"), LFS_ERR_OK);
        ASSERT_EQ(fs.unmount(), LFS_ERR_OK);
    }
    rtos::storage::LittleFs<NorFlash<32>> fs{flash};
    ASSERT_EQ(fs.mount(), LFS_ERR_OK);
    EXPECT_EQ(read_file(fs, "missions/pick.bin"), text(20'000, 1));
    EXPECT_EQ(read_file(fs, "settings"), "speed=1000");
    EXPECT_EQ(flash.double_programs, 0) << "no quad-word programmed twice without an erase";
}

TEST(LittleFs, ManyRewritesInASmallFlashSpreadTheErases) {
    NorFlash<8> flash;
    rtos::storage::LittleFs<NorFlash<8>> fs{flash, 50};
    ASSERT_EQ(fs.format(), LFS_ERR_OK);
    ASSERT_EQ(fs.mount(), LFS_ERR_OK);
    for (int i = 0; i < 400; ++i) {
        ASSERT_EQ(write_file(fs, "counter", text(3000, static_cast<char>(i))), LFS_ERR_OK) << i;
    }
    EXPECT_EQ(read_file(fs, "counter"), text(3000, static_cast<char>(399)));
    // blocks 0 and 1 are the superblock pair, littlefs' anchor: deliberately almost never moved;
    // the rewrites spread over all other blocks
    const auto [lo, hi] = std::minmax_element(flash.erases.begin() + 2, flash.erases.end());
    EXPECT_GT(*lo, 0) << "every data block took part";
    EXPECT_LE(*hi, *lo * 3) << "wear levelling: no block erased far more often than another";
    EXPECT_LT(flash.erases[0] + flash.erases[1], *lo);
    EXPECT_EQ(flash.double_programs, 0);
}

TEST(LittleFs, PowerLossAtAnyStepKeepsTheOldOrTheNewFile) {
    const std::string old_content = text(5000, 'o'), new_content = text(7000, 'n');
    long steps = 0;
    {  // how many program operations the rewrite takes
        NorFlash<16> flash;
        rtos::storage::LittleFs<NorFlash<16>> fs{flash};
        ASSERT_EQ(fs.format(), LFS_ERR_OK);
        ASSERT_EQ(fs.mount(), LFS_ERR_OK);
        ASSERT_EQ(write_file(fs, "recipe", old_content), LFS_ERR_OK);
        flash.programs_left = 1'000'000;
        ASSERT_EQ(write_file(fs, "recipe", new_content), LFS_ERR_OK);
        steps = 1'000'000 - flash.programs_left;
    }
    ASSERT_GT(steps, 10);
    int old_seen = 0, new_seen = 0;
    for (long cut = 0; cut <= steps; ++cut) {
        NorFlash<16> flash;
        {
            rtos::storage::LittleFs<NorFlash<16>> fs{flash};
            ASSERT_EQ(fs.format(), LFS_ERR_OK);
            ASSERT_EQ(fs.mount(), LFS_ERR_OK);
            ASSERT_EQ(write_file(fs, "recipe", old_content), LFS_ERR_OK);
            flash.programs_left = cut;
            (void)write_file(fs, "recipe", new_content);  // power fails during it
        }
        flash.programs_left = -1;  // power is back
        rtos::storage::LittleFs<NorFlash<16>> fs{flash};
        ASSERT_EQ(fs.mount(), LFS_ERR_OK) << "cut after " << cut << " of " << steps;
        const std::string now = read_file(fs, "recipe");
        ASSERT_TRUE(now == old_content || now == new_content) << "cut after " << cut << ": a mixed file";
        (now == old_content ? old_seen : new_seen)++;
    }
    EXPECT_GT(old_seen, 0);
    EXPECT_GT(new_seen, 0);
}

}  // namespace
