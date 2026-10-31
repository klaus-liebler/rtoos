// SPDX-License-Identifier: MIT
//
// ST VL53L0X time-of-flight distance sensor, single ranging with its power-on defaults
// (register map: ST's VL53L0X API, vl53l0x_device.h). Non-blocking: start() a measurement,
// poll() until it returns the distance (about 30 ms later).
//
//   using Tof = rtos::devices::Vl53l0x<I2c1>;
//   if (Tof::probe()) { Tof::start(); ... if (auto mm = Tof::poll()) { ... } }
//
// Ported from the FCU firmware's libs/hw/vl53l0x.
#pragma once

#include <cstdint>
#include <optional>

#include "rtos/devices/i2c.hpp"

namespace rtos::devices {

namespace vl53l0x {
inline constexpr std::uint8_t sysrange_start = 0x00, system_interrupt_clear = 0x0B, result_interrupt_status = 0x13,
                              result_range_mm = 0x1E, identification_model_id = 0xC0;
inline constexpr std::uint8_t model_id = 0xEE;
}  // namespace vl53l0x

template <I2cBus Bus, std::uint8_t Address = 0x29>
class Vl53l0x {
    using Regs = I2cRegisters<Bus, Address>;

public:
    [[nodiscard]] static bool probe() noexcept { return Regs::read8(vl53l0x::identification_model_id) == vl53l0x::model_id; }

    /// Start one measurement.
    static bool start() noexcept { return Regs::write8(vl53l0x::sysrange_start, 0x01); }

    /// The distance in mm once the measurement is done (and the interrupt cleared), else nullopt.
    [[nodiscard]] static std::optional<std::uint16_t> poll() noexcept {
        const auto status = Regs::read8(vl53l0x::result_interrupt_status);
        if (!status || (*status & 0x07) == 0) {
            return std::nullopt;
        }
        const auto mm = Regs::read16(vl53l0x::result_range_mm);
        (void)Regs::write8(vl53l0x::system_interrupt_clear, 0x01);
        return mm;
    }
};

}  // namespace rtos::devices
