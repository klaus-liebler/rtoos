// SPDX-License-Identifier: MIT
//
// External chip drivers (rtos/devices/) against simulated buses: INA226, VL53L0X, TCS34725 on
// a register-file I2C bus; HX711 bit decoding; TMC2209 and Feetech protocol frames and a
// TMC2209 that answers on a single-wire line (with echo); WS2812 bit slots.
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>

#include "rtos/devices/feetech_sts.hpp"
#include "rtos/devices/hx711.hpp"
#include "rtos/devices/ina226.hpp"
#include "rtos/devices/tcs34725.hpp"
#include "rtos/devices/tmc2209.hpp"
#include "rtos/devices/vl53l0x.hpp"
#include "rtos/devices/ws2812.hpp"

namespace {

using namespace rtos::devices;

// ---- I2C: a device = registers addressed by the first byte written (the pointer, as the chip
// sees it), each holding the bytes last written to it; a read returns them ------------------

struct FakeI2c {
    enum class Result { Ok, Nack };
    using Device = std::map<std::uint8_t, std::vector<std::uint8_t>>;
    static inline std::map<std::uint8_t, Device> devices;
    static inline std::uint8_t pointer = 0;

    static Result write_read(std::uint8_t address, std::span<const std::byte> tx, std::span<std::byte> rx) {
        auto it = devices.find(address);
        if (it == devices.end()) {
            return Result::Nack;
        }
        if (!tx.empty()) {
            pointer = std::to_integer<std::uint8_t>(tx[0]);
            if (tx.size() > 1) {
                auto& r = it->second[pointer];
                r.clear();
                for (std::size_t i = 1; i < tx.size(); ++i) r.push_back(std::to_integer<std::uint8_t>(tx[i]));
            }
        }
        const auto& r = it->second[pointer];
        for (std::size_t i = 0; i < rx.size(); ++i) {
            rx[i] = std::byte{i < r.size() ? r[i] : std::uint8_t{0}};
        }
        return Result::Ok;
    }
    static void set(std::uint8_t address, std::uint8_t reg, std::vector<std::uint8_t> bytes) {
        devices[address][reg] = std::move(bytes);
    }
    static void set16(std::uint8_t address, std::uint8_t reg, std::uint16_t v) {
        set(address, reg, {static_cast<std::uint8_t>(v >> 8), static_cast<std::uint8_t>(v)});
    }
    static std::vector<std::uint8_t> get(std::uint8_t address, std::uint8_t reg) { return devices[address][reg]; }
    static void reset() { devices.clear(); }
};
static_assert(I2cBus<FakeI2c>);

TEST(Ina226, CalibrationAtCompileTime) {
    // FCU: 10 mOhm, 8 A -> current LSB 244.141 uA, CAL = 0.00512 / (244.141e-6 * 0.01) = 2097
    using Fcu = Ina226<FakeI2c, {.shunt_micro_ohm = 10'000, .max_current_ma = 8'000}>;
    static_assert(Fcu::calibration == 2097);
    // datasheet example (7.5.1): 2 mOhm, 15 A max -> LSB 457.8 uA, CAL 5592
    static_assert(Ina226<FakeI2c, {.shunt_micro_ohm = 2'000, .max_current_ma = 15'000}>::calibration == 5592);
}

TEST(Ina226, InitAndReadScaleTheRegisters) {
    FakeI2c::reset();
    using Monitor = Ina226<FakeI2c, {.shunt_micro_ohm = 10'000, .max_current_ma = 8'000}>;
    FakeI2c::set16(0x40, 0xFE, 0x5449);
    ASSERT_TRUE(Monitor::probe());
    ASSERT_TRUE(Monitor::init());
    EXPECT_EQ(FakeI2c::get(0x40, 0x05), (std::vector<std::uint8_t>{2097 >> 8, 2097 & 0xFF}));
    EXPECT_EQ(FakeI2c::get(0x40, 0x00), (std::vector<std::uint8_t>{0x43, 0x27}));
    FakeI2c::set16(0x40, 0x02, 16'000);  // 20.000 V
    FakeI2c::set16(0x40, 0x01, 0xFF38);  // -200 * 2.5 uV
    FakeI2c::set16(0x40, 0x04, 4096);    // 4096 * 244.141 uA = 1000 mA
    FakeI2c::set16(0x40, 0x03, 820);     // 820 * 25 * 244.141 uA = 5004 mW
    const auto m = Monitor::read();
    ASSERT_TRUE(m.has_value());
    EXPECT_EQ(m->bus_mv, 20'000);
    EXPECT_EQ(m->shunt_uv, -500);
    EXPECT_EQ(m->current_ma, 1000);
    EXPECT_EQ(m->power_mw, 5004u);
    FakeI2c::devices.erase(0x40);
    EXPECT_FALSE(Monitor::read().has_value()) << "no device: nullopt";
}

TEST(Vl53l0x, StartPollAndClear) {
    FakeI2c::reset();
    using Tof = Vl53l0x<FakeI2c>;
    FakeI2c::set(0x29, 0xC0, {0xEE});
    ASSERT_TRUE(Tof::probe());
    ASSERT_TRUE(Tof::start());
    EXPECT_EQ(FakeI2c::get(0x29, 0x00), std::vector<std::uint8_t>{0x01});
    EXPECT_FALSE(Tof::poll().has_value()) << "not ready";
    FakeI2c::set(0x29, 0x13, {0x04});
    FakeI2c::set16(0x29, 0x1E, 312);
    EXPECT_EQ(Tof::poll(), 312);
    EXPECT_EQ(FakeI2c::get(0x29, 0x0B), std::vector<std::uint8_t>{0x01}) << "interrupt cleared";
}

TEST(Tcs34725, IntegrationTimeAndBlockRead) {
    FakeI2c::reset();
    // the FCU's former setting ATIME 0xC0 = 64 cycles = 153.6 ms
    using Color = Tcs34725<FakeI2c, {.integration_ms = 154, .gain = Tcs34725Gain::X4}>;
    static_assert(Color::atime_register == 0xC0);
    static_assert(Tcs34725<FakeI2c, {.integration_ms = 614}>::atime_register == 0x00);
    // the chip sees command byte | register: ID at 0x80 | 0x12
    FakeI2c::set(0x29, 0x92, {0x44});
    ASSERT_TRUE(Color::probe());
    ASSERT_TRUE(Color::power_on());
    EXPECT_EQ(FakeI2c::get(0x29, 0x81), std::vector<std::uint8_t>{0xC0});
    EXPECT_EQ(FakeI2c::get(0x29, 0x8F), std::vector<std::uint8_t>{0x01});
    ASSERT_TRUE(Color::enable());
    EXPECT_EQ(FakeI2c::get(0x29, 0x80), std::vector<std::uint8_t>{0x03});
    // one block read with auto-increment: command | 0x20 | CDATAL = 0xB4
    FakeI2c::set(0x29, 0xB4, {0x10, 0x01, 0x20, 0x00, 0x30, 0x00, 0x40, 0x00});
    const auto c = Color::read();
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->clear, 0x0110);
    EXPECT_EQ(c->red, 0x20);
    EXPECT_EQ(c->blue, 0x40);
}

// ---- HX711 -----------------------------------------------------------------------------------

TEST(Hx711, DecodesEverySecondSampledBit) {
    // value 0x800001 (most negative + 1): data bits on positions 6, 4, 2, 0 of bytes 0..5
    std::array<std::byte, 8> rx{};
    rx[0] = std::byte{0x40};  // bit 23
    rx[5] = std::byte{0x01};  // bit 0
    EXPECT_EQ(hx711::decode(rx), -8'388'607);
    rx = {};
    rx[5] = std::byte{0x05};  // bits 1 and 0: 3
    EXPECT_EQ(hx711::decode(rx), 3);
    rx.fill(std::byte{0xFF});  // all ones: -1 (odd sample bits are ignored)
    EXPECT_EQ(hx711::decode(rx), -1);
    constexpr auto tx = hx711::clock_pattern(Hx711Gain::A64);
    static_assert(tx[0] == std::byte{0x55} && tx[6] == std::byte{0x54} && tx[7] == std::byte{0x00});
}

// ---- TMC2209 ---------------------------------------------------------------------------------

TEST(Tmc2209, DatagramsAndCrc) {
    // datasheet 4.2 example layout; CRC-8 with polynomial x^8 + x^2 + x + 1, LSB first per byte
    constexpr auto w = tmc2209::write_datagram(0, tmc2209::gconf, 0x000001C0);
    static_assert(w[0] == 0x05 && w[1] == 0x00 && w[2] == 0x80 && w[6] == 0xC0);
    EXPECT_EQ(w[7], tmc2209::crc8(std::span(w).first(7)));
    constexpr auto q = tmc2209::read_request(2, tmc2209::ioin);
    static_assert(q[2] == 0x06);
    std::array<std::uint8_t, 8> reply{0x05, 0xFF, 0x06, 0x21, 0x00, 0x00, 0x40, 0};
    reply[7] = tmc2209::crc8(std::span(reply).first(7));
    EXPECT_EQ(tmc2209::parse_reply(reply, 0x06), 0x21000040u);
    EXPECT_FALSE(tmc2209::parse_reply(reply, 0x07).has_value()) << "other register";
    reply[5] ^= 1;
    EXPECT_FALSE(tmc2209::parse_reply(reply, 0x06).has_value()) << "CRC";
    // VACTUAL: 200 full steps/s * 256 * 2^24 / 12 MHz = 71583
    EXPECT_EQ(tmc2209::vactual_value(200), 71'582u);
    EXPECT_EQ(tmc2209::vactual_value(-200), 0x00FF'FFFFu - 71'582u + 1u);
}

/// Single-wire line with a TMC2209 on it: every byte written comes back (echo); a read request
/// is answered from `regs`; writes land in `regs`.
struct TmcLine {
    std::deque<std::byte> rx;
    std::map<std::uint8_t, std::uint32_t> regs;
    std::size_t write(std::span<const std::byte> d) {
        rx.insert(rx.end(), d.begin(), d.end());
        std::vector<std::uint8_t> b;
        for (auto x : d) b.push_back(std::to_integer<std::uint8_t>(x));
        if (b.size() == 8 && (b[2] & 0x80) != 0) {
            regs[b[2] & 0x7F] = (std::uint32_t{b[3]} << 24) | (b[4] << 16) | (b[5] << 8) | b[6];
        } else if (b.size() == 4) {
            std::array<std::uint8_t, 8> r{0x05, 0xFF, b[2]};
            const std::uint32_t v = regs[b[2]];
            r[3] = static_cast<std::uint8_t>(v >> 24);
            r[4] = static_cast<std::uint8_t>(v >> 16);
            r[5] = static_cast<std::uint8_t>(v >> 8);
            r[6] = static_cast<std::uint8_t>(v);
            r[7] = tmc2209::crc8(std::span(r).first(7));
            for (auto x : r) rx.push_back(std::byte{x});
        }
        return d.size();
    }
    std::size_t read(std::span<std::byte> out) {
        std::size_t n = 0;
        while (n < out.size() && !rx.empty()) {
            out[n++] = rx.front();
            rx.pop_front();
        }
        return n;
    }
};
struct TestClock {
    static inline std::uint32_t now = 0;
    static std::uint32_t now_ms() noexcept { return now; }
    static void sleep_ms(std::uint32_t ms) noexcept { now += ms; }
};

TEST(Tmc2209, InitAndCurrentOverTheLine) {
    TmcLine line;
    line.regs[tmc2209::ioin] = 0x21000000;
    line.regs[tmc2209::chopconf] = 0x10000053;  // reset value: MRES 0 (256)
    Tmc2209<TmcLine, TestClock> driver{line, 0b10};
    ASSERT_TRUE(driver.init(Tmc2209Microsteps::M16));
    EXPECT_EQ(line.regs[tmc2209::gconf], 0x1C0u) << "pdn_disable, mstep_reg_select, multistep_filt";
    EXPECT_EQ(line.regs[tmc2209::chopconf] >> 24 & 0xF, 4u) << "MRES 16";
    ASSERT_TRUE(driver.set_current(4, 12, 6));
    EXPECT_EQ(line.regs[tmc2209::ihold_irun], 0x00060C04u);
}

TEST(Tmc2209, SensorlessHomingStopsAtTheStall) {
    TmcLine line;
    line.regs[tmc2209::ioin] = 0x21000000;
    Tmc2209<TmcLine, TestClock> driver{line, 0};
    ASSERT_TRUE(driver.init(Tmc2209Microsteps::M16));
    std::uint32_t now = 0;
    ASSERT_TRUE(driver.start_homing({.speed = -200, .stall_threshold = 100}, now));
    EXPECT_EQ(line.regs[tmc2209::vactual], tmc2209::vactual_value(200)) << "first away from the end stop";
    EXPECT_NE(line.regs[tmc2209::gconf] & tmc2209::gconf_en_spreadcycle, 0u);
    line.regs[tmc2209::sg_result] = 400;  // load low: no stall
    using R = Tmc2209<TmcLine, TestClock>::HomingResult;
    R r = R::InProgress;
    for (; now < 2000 && r == R::InProgress; now += 10) {
        r = driver.tick_homing(now);
    }
    EXPECT_EQ(r, R::InProgress);
    EXPECT_EQ(line.regs[tmc2209::vactual], tmc2209::vactual_value(-200)) << "towards the end stop";
    line.regs[tmc2209::sg_result] = 150;  // < 2 x 100: stall
    EXPECT_EQ(driver.tick_homing(now), R::Done);
    EXPECT_EQ(line.regs[tmc2209::vactual], 0u);
    EXPECT_EQ(line.regs[tmc2209::gconf] & tmc2209::gconf_en_spreadcycle, 0u) << "StealthChop again";
}

// ---- Feetech ---------------------------------------------------------------------------------

TEST(Feetech, PacketChecksumAndSignMagnitude) {
    std::array<std::uint8_t, feetech::max_packet> f{};
    // ping of servo 1 (Feetech protocol manual): FF FF 01 02 01 FB
    ASSERT_EQ(feetech::packet(f, 1, feetech::Instruction::Ping, {}), 6u);
    EXPECT_EQ((std::array<std::uint8_t, 6>{f[0], f[1], f[2], f[3], f[4], f[5]}),
              (std::array<std::uint8_t, 6>{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
    static_assert(feetech::encode(-5) == 0x8005 && feetech::decode(0x05, 0x80) == -5 && feetech::decode(0x00, 0x08) == 2048);
}

struct ServoLine {
    std::deque<std::byte> rx;
    std::vector<std::uint8_t> sent;
    std::size_t write(std::span<const std::byte> d) {
        for (auto x : d) sent.push_back(std::to_integer<std::uint8_t>(x));
        if (sent.size() >= 5 && sent[4] == 0x02) {  // read: 15 bytes of feedback, position 2048, moving
            std::array<std::uint8_t, 15> p{0x00, 0x08, 0x10, 0x80, 0, 0, 120, 31, 0, 0, 1, 0, 0, 0x05, 0x00};
            std::array<std::uint8_t, feetech::max_packet> r{};
            const std::size_t n = feetech::packet(r, sent[2], static_cast<feetech::Instruction>(0), p);
            for (std::size_t i = 0; i < n; ++i) rx.push_back(std::byte{r[i]});
        }
        return d.size();
    }
    std::size_t read(std::span<std::byte> out) {
        std::size_t n = 0;
        while (n < out.size() && !rx.empty()) {
            out[n++] = rx.front();
            rx.pop_front();
        }
        return n;
    }
};

TEST(Feetech, SyncWriteAndFeedback) {
    ServoLine line;
    FeetechBus<ServoLine, TestClock> bus{line};
    const std::array<std::uint8_t, 2> ids{11, 12};
    const std::array<std::int16_t, 2> pos{2048, -100};
    const std::array<std::uint16_t, 2> speed{500, 600};
    ASSERT_TRUE(bus.sync_write_positions(ids, pos, speed, 20));
    ASSERT_EQ(line.sent.size(), 7u + 2 * 8 + 1);
    EXPECT_EQ(line.sent[2], 0xFE);
    EXPECT_EQ(line.sent[3], (7 + 1) * 2 + 4) << "length";
    EXPECT_EQ(line.sent[5], feetech::acceleration);
    // second servo from byte 15: id, acceleration, position low, position high (sign bit)
    EXPECT_EQ(line.sent[15], 12);
    EXPECT_EQ(line.sent[17], 0x64);
    EXPECT_EQ(line.sent[18], 0x80) << "sign bit";
    line.sent.clear();
    const auto f = bus.feedback(11);
    ASSERT_TRUE(f.has_value());
    EXPECT_EQ(f->position, 2048);
    EXPECT_EQ(f->speed, -16);
    EXPECT_EQ(f->temperature_c, 31);
    EXPECT_TRUE(f->moving);
    EXPECT_EQ(f->current, 5);
}

// ---- WS2812 ----------------------------------------------------------------------------------

TEST(Ws2812, GrbBitsMsbFirstAndResetSlot) {
    constexpr auto t = ws2812_timing(250'000'000);  // FCU TIM15: 312 ticks per bit
    static_assert(t.period_ticks == 312 && t.zero == 109 && t.one == 218);
    const std::array<std::uint32_t, 1> red{0xFF0000};
    std::array<std::uint16_t, ws2812_slots(1)> slots{};
    ws2812_encode(red, slots, t);
    for (int i = 0; i < 8; ++i) EXPECT_EQ(slots[i], t.zero) << "green first";
    for (int i = 8; i < 16; ++i) EXPECT_EQ(slots[i], t.one) << "then red";
    EXPECT_EQ(slots[24], 0) << "reset slot";
}

}  // namespace
