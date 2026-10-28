// SPDX-License-Identifier: MIT
//
// The SD protocol against a simulated card: the card answers the commands as the SD Physical
// Layer Simplified Specification describes and checks that the protocol uses them correctly
// (order, addressing, stop commands).
#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "rtos/storage/sd_card.hpp"

namespace {

using namespace rtos::storage;

/// Set bits [hi:lo] of a 128-bit register ([0] = bits 127:96).
void set_bits(std::array<std::uint32_t, 4>& r, unsigned hi, unsigned lo, std::uint32_t v) {
    for (unsigned b = lo; b <= hi; ++b) {
        std::uint32_t& word = r[3 - b / 32];
        const std::uint32_t bit = 1u << (b % 32);
        word = ((v >> (b - lo)) & 1u) != 0 ? (word | bit) : (word & ~bit);
    }
}

struct SimulatedCard {
    enum class Kind { Sdhc, SdscV2, SdscV1, Absent } kind = Kind::Sdhc;
    int busy_acmd41 = 3;          ///< ACMD41 answers "busy" this often
    int programming_polls = 2;    ///< CMD13 reports "prg" this often after a write
    std::uint32_t blocks = 2048;
    std::vector<std::byte> data;

    // what the protocol did
    std::vector<std::uint8_t> commands;
    bool powered = false, fast = false, selected = false, app = false;
    int stops = 0;
    std::uint32_t last_address = 0;

    static constexpr std::uint32_t rca = 0x1234;
    int busy_left = 0;
    int prg_left = 0;

    SimulatedCard() { reset(); }
    void reset() {
        data.assign(static_cast<std::size_t>(blocks) * sd_block_size, std::byte{0});
        busy_left = busy_acmd41;
    }

    std::array<std::uint32_t, 4> csd() const {
        std::array<std::uint32_t, 4> r{};
        if (kind == Kind::Sdhc) {
            set_bits(r, 127, 126, 1);
            set_bits(r, 69, 48, blocks / 1024 - 1);
        } else {  // CSD 1.0: blocks = (C_SIZE + 1) * 2^(MULT + 2) * 2^READ_BL_LEN / 512, 512-byte blocks
            set_bits(r, 83, 80, 9);
            set_bits(r, 49, 47, 7);  // * 512
            set_bits(r, 73, 62, blocks / 512 - 1);
        }
        return r;
    }

    std::uint32_t status_r1() const { return ((prg_left > 0 ? 7u : 4u) << 9) | (prg_left > 0 ? 0u : 1u << 8); }
    std::size_t offset(std::uint32_t address) const {
        return kind == Kind::Sdhc ? static_cast<std::size_t>(address) * sd_block_size : address;
    }
};

struct FakeHost {
    SimulatedCard& card;

    void power_on() noexcept { card.powered = true; }
    void set_transfer_clock() noexcept { card.fast = true; }
    void delay_ms(std::uint32_t) noexcept {}

    SdCommandResult command(std::uint8_t index, std::uint32_t arg, SdResponse r) noexcept {
        card.commands.push_back(index);
        if (card.kind == SimulatedCard::Kind::Absent) {
            return {SdStatus::Timeout, {}};
        }
        const bool app = card.app;
        card.app = false;
        switch (index) {
            case 0: EXPECT_EQ(r, SdResponse::None); return {};
            case 8:
                if (card.kind == SimulatedCard::Kind::SdscV1) {
                    return {SdStatus::Timeout, {}};  // v1 cards don't know CMD8
                }
                EXPECT_EQ(r, SdResponse::R7);
                return {SdStatus::Ok, {arg & 0xFFFu}};
            case 55: card.app = true; return {SdStatus::Ok, {0x120}};
            case 41: {
                EXPECT_TRUE(app) << "ACMD41 must follow CMD55";
                EXPECT_EQ(r, SdResponse::R3);
                if (card.busy_left-- > 0) {
                    return {SdStatus::Ok, {0x00FF'8000u}};
                }
                const bool ccs = card.kind == SimulatedCard::Kind::Sdhc && (arg & (1u << 30)) != 0;
                return {SdStatus::Ok, {(1u << 31) | (ccs ? 1u << 30 : 0u) | 0x00FF'8000u}};
            }
            case 2: EXPECT_EQ(r, SdResponse::R2); return {SdStatus::Ok, {0x0353'4453, 1, 2, 3}};
            case 3: return {SdStatus::Ok, {SimulatedCard::rca << 16}};
            case 9:
                EXPECT_EQ(arg, SimulatedCard::rca << 16);
                return {SdStatus::Ok, card.csd()};
            case 7:
                EXPECT_EQ(r, SdResponse::R1b);
                card.selected = arg == SimulatedCard::rca << 16;
                return {SdStatus::Ok, {0x700}};
            case 16: EXPECT_EQ(arg, 512u); return {SdStatus::Ok, {0x900}};
            case 13:
                EXPECT_EQ(arg, SimulatedCard::rca << 16);
                {
                    const std::uint32_t s = card.status_r1();
                    if (card.prg_left > 0) {
                        --card.prg_left;
                    }
                    return {SdStatus::Ok, {s}};
                }
            default: ADD_FAILURE() << "unexpected CMD" << int(index); return {SdStatus::CardError, {}};
        }
    }

    SdCommandResult stop(SdResponse r) noexcept {
        EXPECT_EQ(r, SdResponse::R1b);
        card.commands.push_back(12);
        ++card.stops;
        return {SdStatus::Ok, {0x900}};
    }

    SdCommandResult read_data(std::uint8_t index, std::uint32_t arg, std::span<std::byte> in) noexcept {
        card.commands.push_back(index);
        EXPECT_TRUE(card.selected && card.fast);
        EXPECT_EQ(index, in.size() > sd_block_size ? 18 : 17);
        card.last_address = arg;
        std::memcpy(in.data(), card.data.data() + card.offset(arg), in.size());
        return {};
    }

    SdCommandResult write_data(std::uint8_t index, std::uint32_t arg, std::span<const std::byte> out) noexcept {
        card.commands.push_back(index);
        EXPECT_EQ(index, out.size() > sd_block_size ? 25 : 24);
        card.last_address = arg;
        std::memcpy(card.data.data() + card.offset(arg), out.data(), out.size());
        card.prg_left = card.programming_polls;
        return {};
    }
};
static_assert(SdHost<FakeHost>);

struct Fixture {
    SimulatedCard card;
    FakeHost host{card};
    SdCard<FakeHost> sd{host};
};

TEST(SdCard, InitialisesAnSdhcCard) {
    Fixture f;
    ASSERT_EQ(f.sd.init(), SdStatus::Ok);
    EXPECT_TRUE(f.sd.ready());
    EXPECT_TRUE(f.sd.high_capacity());
    EXPECT_EQ(f.sd.blocks(), 2048u);
    EXPECT_TRUE(f.card.powered && f.card.fast && f.card.selected);
    // CMD0, CMD8, (CMD55 ACMD41) x 4 (3 busy), CMD2, CMD3, CMD9, CMD7 - no CMD16 for SDHC
    const std::vector<std::uint8_t> expected{0, 8, 55, 41, 55, 41, 55, 41, 55, 41, 2, 3, 9, 7};
    EXPECT_EQ(f.card.commands, expected);
}

TEST(SdCard, StandardCapacityCardsAreByteAddressedAndGet512ByteBlocks) {
    Fixture f;
    f.card.kind = SimulatedCard::Kind::SdscV2;
    ASSERT_EQ(f.sd.init(), SdStatus::Ok);
    EXPECT_FALSE(f.sd.high_capacity());
    EXPECT_EQ(f.sd.blocks(), 2048u) << "CSD 1.0 capacity";
    EXPECT_EQ(f.card.commands.back(), 16) << "SET_BLOCKLEN 512";
    std::array<std::byte, 512> b{};
    ASSERT_EQ(f.sd.read(3, b), SdStatus::Ok);
    EXPECT_EQ(f.card.last_address, 3u * 512) << "byte address";
}

TEST(SdCard, VersionOneCardsDontAnswerCmd8) {
    Fixture f;
    f.card.kind = SimulatedCard::Kind::SdscV1;
    ASSERT_EQ(f.sd.init(), SdStatus::Ok);
    EXPECT_FALSE(f.sd.high_capacity());
}

TEST(SdCard, NoCardIsReported) {
    Fixture f;
    f.card.kind = SimulatedCard::Kind::Absent;
    EXPECT_EQ(f.sd.init(), SdStatus::NoCard);
    EXPECT_FALSE(f.sd.ready());
    std::array<std::byte, 512> b{};
    EXPECT_EQ(f.sd.read(0, b), SdStatus::NotReady);
}

TEST(SdCard, CardThatNeverGetsReadyTimesOut) {
    Fixture f;
    f.card.busy_acmd41 = 100'000;
    f.card.reset();
    EXPECT_EQ(f.sd.init(), SdStatus::Timeout);
}

TEST(SdCard, WriteWaitsForProgrammingAndReadsBack) {
    Fixture f;
    ASSERT_EQ(f.sd.init(), SdStatus::Ok);
    std::array<std::byte, 3 * 512> out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = static_cast<std::byte>(i * 7);
    }
    f.card.commands.clear();
    ASSERT_EQ(f.sd.write(100, out), SdStatus::Ok);
    EXPECT_EQ(f.card.last_address, 100u) << "block address";
    // CMD25, CMD12, CMD13 until "tran": 2 x prg, then tran
    EXPECT_EQ(f.card.commands, (std::vector<std::uint8_t>{25, 12, 13, 13, 13}));
    std::array<std::byte, 3 * 512> in{};
    f.card.commands.clear();
    ASSERT_EQ(f.sd.read(100, in), SdStatus::Ok);
    EXPECT_EQ(f.card.commands, (std::vector<std::uint8_t>{18, 12}));
    EXPECT_EQ(in, out);
    std::array<std::byte, 512> one{};
    f.card.commands.clear();
    ASSERT_EQ(f.sd.read(101, one), SdStatus::Ok);
    EXPECT_EQ(f.card.commands, std::vector<std::uint8_t>{17}) << "single block: no CMD12";
    EXPECT_EQ(one[0], out[512]);
}

TEST(SdCard, RejectsPartialBlocksAndRangesBeyondTheCard) {
    Fixture f;
    ASSERT_EQ(f.sd.init(), SdStatus::Ok);
    std::array<std::byte, 100> partial{};
    EXPECT_EQ(f.sd.read(0, partial), SdStatus::CardError);
    std::array<std::byte, 1024> two{};
    EXPECT_EQ(f.sd.read(2047, two), SdStatus::CardError);
    EXPECT_EQ(f.sd.read(2046, two), SdStatus::Ok);
}

TEST(SdCard, CsdCapacity) {
    std::array<std::uint32_t, 4> v2{};
    set_bits(v2, 127, 126, 1);
    set_bits(v2, 69, 48, 15'159);  // a "8 GB" card: 15160 x 512 KB
    EXPECT_EQ(sd_csd_blocks(v2), 15'160ull * 1024);
    std::array<std::uint32_t, 4> v1{};
    set_bits(v1, 83, 80, 10);  // READ_BL_LEN 1024
    set_bits(v1, 49, 47, 7);
    set_bits(v1, 73, 62, 4095);  // 2 GB
    EXPECT_EQ(sd_csd_blocks(v1), 4096ull * 512 * 1024 / 512);
    std::array<std::uint32_t, 4> bad{};
    set_bits(bad, 127, 126, 3);
    EXPECT_EQ(sd_csd_blocks(bad), 0u);
}

}  // namespace
