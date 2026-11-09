// A/B program slots of the PLC (rtos/plc/slot_store.hpp): commit, configuration journal, and power
// loss at every flash operation - afterwards the old or the new program is there, never half of
// one. Both geometries of the FCU: STM32H5 (8 KB sectors, 16-byte program unit) and STM32H7
// (128 KB sectors, 32-byte flash words).
#include "rtos/plc/slot_store.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <vector>

namespace {

using namespace rtos::plc;

/// NOR flash: erased = 0xFF, each program unit programmable once after an erase; power can fail
/// after a number of operations (erase or program unit), then nothing more reaches the flash.
template <std::uint32_t BlockBytes, std::uint32_t Blocks, std::uint32_t ProgramBytes>
struct Nor {
    static constexpr std::uint32_t block_bytes = BlockBytes;
    static constexpr std::uint32_t block_count = Blocks;
    static constexpr std::uint32_t program_bytes = ProgramBytes;

    std::vector<std::uint8_t> cells = std::vector<std::uint8_t>(std::size_t{Blocks} * BlockBytes, 0xFF);
    std::vector<bool> programmed = std::vector<bool>(std::size_t{Blocks} * BlockBytes / ProgramBytes, false);
    long ops_left = -1;  ///< >= 0: power fails after this many operations
    long ops = 0;
    int double_programs = 0;

    bool power() noexcept {
        ++ops;
        if (ops_left == 0) return false;
        if (ops_left > 0) --ops_left;
        return true;
    }
    bool read(std::uint32_t block, std::uint32_t offset, std::span<std::byte> out) noexcept {
        if (block >= Blocks || offset + out.size() > BlockBytes) return false;
        std::memcpy(out.data(), &cells[std::size_t{block} * BlockBytes + offset], out.size());
        return true;
    }
    bool program(std::uint32_t block, std::uint32_t offset, std::span<const std::byte> data) noexcept {
        if (block >= Blocks || offset % ProgramBytes != 0 || data.size() % ProgramBytes != 0 || offset + data.size() > BlockBytes) {
            return false;
        }
        for (std::size_t q = 0; q < data.size(); q += ProgramBytes) {
            if (!power()) return false;
            const std::size_t at = std::size_t{block} * BlockBytes + offset + q;
            if (programmed[at / ProgramBytes]) {
                ++double_programs;
                return false;
            }
            programmed[at / ProgramBytes] = true;
            std::memcpy(&cells[at], data.data() + q, ProgramBytes);
        }
        return true;
    }
    bool erase(std::uint32_t block) noexcept {
        if (block >= Blocks || !power()) return false;
        std::fill_n(&cells[std::size_t{block} * BlockBytes], BlockBytes, std::uint8_t{0xFF});
        std::fill_n(programmed.begin() + block * (BlockBytes / ProgramBytes), BlockBytes / ProgramBytes, false);
        return true;
    }
};

using H5 = Nor<8192, 32, 16>;      // 2 x 128 KB at 0x081C0000
using H7 = Nor<131072, 2, 32>;     // the last two sectors of bank 2

template <class F>
class SlotStoreTest : public testing::Test {};
using Geometries = testing::Types<H5, H7>;
TYPED_TEST_SUITE(SlotStoreTest, Geometries);

std::vector<std::uint8_t> image(std::size_t n, std::uint8_t seed) {
    std::vector<std::uint8_t> v(n);
    for (std::size_t i = 0; i < n; ++i) v[i] = static_cast<std::uint8_t>(seed + i * 7 + (i >> 8));
    return v;
}

template <class F>
std::vector<std::uint8_t> read_back(SlotStore<F>& s) {
    std::vector<std::uint8_t> out(s.image_bytes());
    EXPECT_EQ(s.read_image(out), SlotError::none);
    return out;
}

TYPED_TEST(SlotStoreTest, EmptyFlashHasNoProgram) {
    TypeParam flash;
    SlotStore<TypeParam> s{flash};
    s.mount();
    EXPECT_FALSE(s.has_slot());
    EXPECT_FALSE(s.has_image());
    EXPECT_EQ(s.config(), SlotConfig{});
}

TYPED_TEST(SlotStoreTest, CommitAlternatesSlotsAndKeepsThePrevious) {
    TypeParam flash;
    const auto a = image(5000, 1), b = image(70000, 2);
    {
        SlotStore<TypeParam> s{flash};
        s.mount();
        ASSERT_EQ(s.commit(a, {1, 1, 0, 0, 0}), SlotError::none);
        EXPECT_EQ(s.active_slot(), 0);
    }
    {
        SlotStore<TypeParam> s{flash};  // a restart
        s.mount();
        ASSERT_TRUE(s.has_image());
        EXPECT_EQ(read_back(s), a);
        EXPECT_EQ(s.config().control_source, 1);
        EXPECT_EQ(s.config().autostart, 1);
        ASSERT_EQ(s.commit(b, {0, 1, 0, 0, 0}), SlotError::none);
        EXPECT_EQ(s.active_slot(), 1);
        EXPECT_EQ(s.sequence(), 2u);
    }
    SlotStore<TypeParam> s{flash};
    s.mount();
    EXPECT_EQ(read_back(s), b);
    EXPECT_EQ(s.config().control_source, 0);
    EXPECT_EQ(flash.double_programs, 0);
}

TYPED_TEST(SlotStoreTest, ConfigurationWithoutAProgram) {
    TypeParam flash;
    SlotStore<TypeParam> s{flash};
    s.mount();
    ASSERT_EQ(s.set_config({1, 0, 0, 0, 0}), SlotError::none);  // no slot yet: an empty one is written
    SlotStore<TypeParam> t{flash};
    t.mount();
    EXPECT_TRUE(t.has_slot());
    EXPECT_FALSE(t.has_image());
    EXPECT_EQ(t.config().control_source, 1);
    std::vector<std::uint8_t> out(16);
    EXPECT_EQ(t.read_image(out), SlotError::no_image);
}

TYPED_TEST(SlotStoreTest, TooLargeImageIsRefused) {
    TypeParam flash;
    SlotStore<TypeParam> s{flash};
    s.mount();
    EXPECT_EQ(s.commit(image(SlotStore<TypeParam>::max_image_bytes + 1, 0), {}), SlotError::too_large);
    EXPECT_EQ(s.commit(image(SlotStore<TypeParam>::max_image_bytes, 0), {}), SlotError::none);
    EXPECT_GE(SlotStore<TypeParam>::max_image_bytes, 96u * 1024) << "the 96 KB image of the specification fits";
}

TYPED_TEST(SlotStoreTest, ConfigurationJournalAndItsMove) {
    TypeParam flash;
    const auto a = image(3000, 9);
    SlotStore<TypeParam> s{flash};
    s.mount();
    ASSERT_EQ(s.commit(a, {}), SlotError::none);
    const int first = s.active_slot();
    // more changes than the journal holds: it moves to the other slot, the image with it
    for (std::uint32_t i = 0; i < SlotStore<TypeParam>::journal_records + 5; ++i) {
        ASSERT_EQ(s.set_config({static_cast<std::uint8_t>(i % 2), static_cast<std::uint8_t>(i & 0xFF), 0, 0, i}), SlotError::none) << i;
    }
    EXPECT_NE(s.active_slot(), first);
    SlotStore<TypeParam> t{flash};
    t.mount();
    EXPECT_EQ(read_back(t), a);
    EXPECT_EQ(t.config(), s.config());
    EXPECT_EQ(t.config().reserved2, SlotStore<TypeParam>::journal_records + 4);
    EXPECT_EQ(flash.double_programs, 0);
}

TYPED_TEST(SlotStoreTest, PowerLossDuringCommitLeavesTheOldOrTheNewProgram) {
    const auto old_image = image(2500, 3), new_image = image(4100, 4);
    // the number of operations a commit takes
    long total;
    {
        TypeParam flash;
        SlotStore<TypeParam> s{flash};
        s.mount();
        ASSERT_EQ(s.commit(old_image, {1, 0, 0, 0, 0}), SlotError::none);
        const long before = flash.ops;
        ASSERT_EQ(s.commit(new_image, {0, 1, 0, 0, 0}), SlotError::none);
        total = flash.ops - before;
    }
    int saw_old = 0, saw_new = 0;
    for (long cut = 0; cut <= total; ++cut) {
        TypeParam flash;
        SlotStore<TypeParam> s{flash};
        s.mount();
        ASSERT_EQ(s.commit(old_image, {1, 0, 0, 0, 0}), SlotError::none);
        flash.ops_left = cut;
        (void)s.commit(new_image, {0, 1, 0, 0, 0});
        flash.ops_left = -1;  // power is back: restart
        SlotStore<TypeParam> r{flash};
        r.mount();
        ASSERT_TRUE(r.has_image()) << "cut after " << cut << " of " << total << " operations";
        const auto got = read_back(r);
        if (got == old_image) {
            ++saw_old;
            EXPECT_EQ(r.config().control_source, 1);
        } else {
            ASSERT_EQ(got, new_image) << "cut after " << cut;
            ++saw_new;
            EXPECT_EQ(r.config().autostart, 1);
        }
    }
    EXPECT_GT(saw_old, 0);
    EXPECT_EQ(saw_new, 1) << "only the complete header makes the new program valid";
}

TYPED_TEST(SlotStoreTest, PowerLossDuringTheJournalMoveKeepsProgramAndAConfiguration) {
    const auto a = image(1500, 5);
    TypeParam probe;
    SlotStore<TypeParam> p{probe};
    p.mount();
    ASSERT_EQ(p.commit(a, {}), SlotError::none);
    for (std::uint32_t i = 0; i < SlotStore<TypeParam>::journal_records; ++i) ASSERT_EQ(p.set_config({0, 0, 0, 0, i + 1}), SlotError::none);
    const long before = probe.ops;
    ASSERT_EQ(p.set_config({1, 1, 1, 0, 999}), SlotError::none);  // the move
    const long total = probe.ops - before;
    for (long cut = 0; cut <= total; cut += std::max<long>(1, total / 40)) {
        TypeParam flash;  // the state just before the move, built again
        SlotStore<TypeParam> s{flash};
        s.mount();
        ASSERT_EQ(s.commit(a, {}), SlotError::none);
        for (std::uint32_t i = 0; i < SlotStore<TypeParam>::journal_records; ++i) ASSERT_EQ(s.set_config({0, 0, 0, 0, i + 1}), SlotError::none);
        flash.ops_left = cut;
        (void)s.set_config({1, 1, 1, 0, 999});
        flash.ops_left = -1;
        SlotStore<TypeParam> r{flash};
        r.mount();
        ASSERT_EQ(read_back(r), a) << "cut " << cut;
        const auto c = r.config();
        EXPECT_TRUE(c.reserved2 == SlotStore<TypeParam>::journal_records || c.reserved2 == 999) << c.reserved2;
    }
}

TYPED_TEST(SlotStoreTest, DamagedActiveImageFallsBackToThePreviousProgram) {
    TypeParam flash;
    const auto a = image(2000, 6), b = image(2100, 7);
    SlotStore<TypeParam> s{flash};
    s.mount();
    ASSERT_EQ(s.commit(a, {}), SlotError::none);
    ASSERT_EQ(s.commit(b, {}), SlotError::none);
    const std::size_t slot1 = std::size_t{SlotStore<TypeParam>::slot_bytes};
    flash.cells[slot1 + SlotStore<TypeParam>::image_offset + 100] ^= 0x01;  // a bit flips in slot B's image
    SlotStore<TypeParam> r{flash};
    r.mount();
    EXPECT_EQ(r.active_slot(), 0);
    EXPECT_EQ(read_back(r), a);
}

}  // namespace
