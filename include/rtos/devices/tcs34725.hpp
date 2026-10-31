// SPDX-License-Identifier: MIT
//
// ams TCS34725 colour sensor (datasheet DS000390): clear/red/green/blue counts. Integration
// time and gain are compile-time values; the oscillator needs 2.4 ms between power_on() and
// enable() (datasheet: PON before AEN), which the caller waits.
//
//   using Color = rtos::devices::Tcs34725<I2c1, {.integration_ms = 101, .gain = Tcs34725Gain::X4}>;
//   Color::power_on(); kernel.sleep(3); Color::enable();
//   if (auto c = Color::read()) { c->red ... }
//
// Ported from the FCU firmware's libs/hw/tcs34725.
#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "rtos/devices/i2c.hpp"

namespace rtos::devices {

enum class Tcs34725Gain : std::uint8_t { X1 = 0, X4 = 1, X16 = 2, X60 = 3 };

struct Tcs34725Config {
    std::uint32_t integration_ms = 101;  ///< 2.4 .. 614 ms in 2.4 ms steps
    Tcs34725Gain gain = Tcs34725Gain::X4;
};

struct Tcs34725Color {
    std::uint16_t clear = 0, red = 0, green = 0, blue = 0;
};

namespace tcs34725 {
inline constexpr std::uint8_t command = 0x80, auto_increment = 0x20;
inline constexpr std::uint8_t enable = 0x00, atime = 0x01, control = 0x0F, id = 0x12, cdatal = 0x14;
inline constexpr std::uint8_t enable_pon = 0x01, enable_aen = 0x02;
inline constexpr std::uint8_t id_tcs34725 = 0x44, id_tcs34727 = 0x4D;

/// ATIME = 256 - integration time / 2.4 ms (datasheet register 0x01), rounded.
[[nodiscard]] constexpr int atime_value(std::uint32_t ms) noexcept {
    return 256 - static_cast<int>((ms * 10 + 12) / 24);
}
}  // namespace tcs34725

template <I2cBus Bus, Tcs34725Config C = {}>
class Tcs34725 {
    static constexpr int atime_count = tcs34725::atime_value(C.integration_ms);
    static_assert(atime_count >= 0 && atime_count <= 255, "TCS34725 integration time 2.4 .. 614 ms");
    using Regs = I2cRegisters<Bus, 0x29>;

public:
    static constexpr std::uint8_t atime_register = static_cast<std::uint8_t>(atime_count);

    [[nodiscard]] static bool probe() noexcept {
        const auto id = Regs::read8(tcs34725::command | tcs34725::id);
        return id == tcs34725::id_tcs34725 || id == tcs34725::id_tcs34727;
    }
    /// Integration time, gain, oscillator on.
    static bool power_on() noexcept {
        using namespace tcs34725;
        return Regs::write8(command | tcs34725::atime, atime_register) &&
               Regs::write8(command | control, static_cast<std::uint8_t>(C.gain)) &&
               Regs::write8(command | tcs34725::enable, enable_pon);
    }
    /// Start the colour measurements (2.4 ms or more after power_on()).
    static bool enable() noexcept {
        using namespace tcs34725;
        return Regs::write8(command | tcs34725::enable, enable_pon | enable_aen);
    }
    /// The last integration: eight bytes, little-endian, read at once (one consistent set).
    [[nodiscard]] static std::optional<Tcs34725Color> read() noexcept {
        std::array<std::byte, 8> raw{};
        if (!Regs::read(tcs34725::command | tcs34725::auto_increment | tcs34725::cdatal, raw)) {
            return std::nullopt;
        }
        const auto le = [&](std::size_t i) {
            return static_cast<std::uint16_t>(std::to_integer<unsigned>(raw[i]) | (std::to_integer<unsigned>(raw[i + 1]) << 8));
        };
        return Tcs34725Color{le(0), le(2), le(4), le(6)};
    }
};

}  // namespace rtos::devices
