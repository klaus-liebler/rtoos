// SPDX-License-Identifier: MIT
//
// TI INA226 current, voltage and power monitor (datasheet SBOS547A). The calibration comes
// from the shunt and the largest expected current at compile time (datasheet 7.5, equations
// 1-3):
//
//   using Supply = rtos::devices::Ina226<I2c4, {.shunt_micro_ohm = 10'000, .max_current_ma = 8'000}>;
//   Supply::init();
//   if (const auto m = Supply::read()) { m->bus_mv ... }
//
// Ported from the FCU firmware's libs/hw/ina226.
#pragma once

#include <cstdint>
#include <optional>

#include "rtos/devices/i2c.hpp"

namespace rtos::devices {

struct Ina226Config {
    std::uint32_t shunt_micro_ohm;
    std::uint32_t max_current_ma;   ///< sets the current LSB: max / 2^15
    std::uint8_t address = 0x40;    ///< A0 = A1 = GND
    /// CONFIG: averaging 16, conversion times 1.1 ms, shunt and bus continuous (0x4327)
    std::uint16_t config = 0x4327;
};

struct Ina226Measurement {
    std::int32_t bus_mv = 0;
    std::int32_t shunt_uv = 0;
    std::int32_t current_ma = 0;
    std::uint32_t power_mw = 0;
};

namespace ina226 {
inline constexpr std::uint8_t config = 0x00, shunt_voltage = 0x01, bus_voltage = 0x02, power = 0x03, current = 0x04,
                              calibration = 0x05, manufacturer_id = 0xFE, die_id = 0xFF;
inline constexpr std::uint16_t texas_instruments = 0x5449;  ///< "TI"

/// Current LSB in nA: max current / 2^15, rounded up (datasheet equation 2).
[[nodiscard]] constexpr std::uint64_t current_lsb_na(const Ina226Config& c) noexcept {
    return (static_cast<std::uint64_t>(c.max_current_ma) * 1'000'000 + 32'767) / 32'768;
}
/// CAL = 0.00512 / (current LSB [A] * shunt [ohm]) (datasheet equation 1).
[[nodiscard]] constexpr std::uint64_t calibration_value(const Ina226Config& c) noexcept {
    return 5'120'000'000'000ull / (current_lsb_na(c) * c.shunt_micro_ohm);
}
}  // namespace ina226

template <I2cBus Bus, Ina226Config C>
class Ina226 {
    static_assert(C.shunt_micro_ohm > 0 && C.max_current_ma > 0);
    static constexpr std::uint64_t lsb_na = ina226::current_lsb_na(C);
    static constexpr std::uint64_t cal = ina226::calibration_value(C);
    static_assert(cal >= 1 && cal <= 0x7FFF, "INA226 calibration out of range: check shunt and current");
    using Regs = I2cRegisters<Bus, C.address>;

public:
    static constexpr std::uint16_t calibration = static_cast<std::uint16_t>(cal);

    /// The manufacturer ID answers "TI".
    [[nodiscard]] static bool probe() noexcept { return Regs::read16(ina226::manufacturer_id) == ina226::texas_instruments; }

    /// Configuration and calibration (the INA226 measures continuously from then on).
    static bool init() noexcept {
        return Regs::write16(ina226::config, C.config) && Regs::write16(ina226::calibration, calibration);
    }

    [[nodiscard]] static std::optional<Ina226Measurement> read() noexcept {
        const auto bus = Regs::read16(ina226::bus_voltage);
        const auto shunt = Regs::read16(ina226::shunt_voltage);
        const auto current = Regs::read16(ina226::current);
        const auto power = Regs::read16(ina226::power);
        if (!bus || !shunt || !current || !power) {
            return std::nullopt;
        }
        Ina226Measurement m;
        m.bus_mv = static_cast<std::int32_t>(*bus * 5u / 4u);                         // 1.25 mV
        m.shunt_uv = static_cast<std::int32_t>(static_cast<std::int16_t>(*shunt)) * 5 / 2;  // 2.5 uV
        m.current_ma = static_cast<std::int32_t>(static_cast<std::int64_t>(static_cast<std::int16_t>(*current)) *
                                                 static_cast<std::int64_t>(lsb_na) / 1'000'000);
        m.power_mw = static_cast<std::uint32_t>(static_cast<std::uint64_t>(*power) * 25 * lsb_na / 1'000'000);
        return m;
    }

    /// Manufacturer and die ID, for diagnostics.
    [[nodiscard]] static std::optional<std::uint32_t> ids() noexcept {
        const auto m = Regs::read16(ina226::manufacturer_id);
        const auto d = Regs::read16(ina226::die_id);
        if (!m || !d) {
            return std::nullopt;
        }
        return (std::uint32_t{*m} << 16) | *d;
    }
};

}  // namespace rtos::devices
