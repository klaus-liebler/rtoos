// SPDX-License-Identifier: MIT
//
// What the device drivers (rtos/devices/) need from an I2C bus: a static write_read() whose
// result has an Ok value - stm32h5::I2cMaster<Id, Config> on the target, a simulated bus in
// the host tests. Registers of the devices here are big-endian 8- or 16-bit values.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace rtos::devices {

template <class B>
concept I2cBus = requires(std::uint8_t address, std::span<const std::byte> tx, std::span<std::byte> rx) {
    { B::write_read(address, tx, rx) };
    { B::write_read(address, tx, rx) == decltype(B::write_read(address, tx, rx))::Ok } -> std::convertible_to<bool>;
};

/// Register access of one device at `Address`.
template <I2cBus Bus, std::uint8_t Address>
struct I2cRegisters {
    static bool write8(std::uint8_t reg, std::uint8_t value) noexcept {
        const std::array<std::byte, 2> tx{std::byte{reg}, std::byte{value}};
        return ok(Bus::write_read(Address, tx, {}));
    }
    static bool write16(std::uint8_t reg, std::uint16_t value) noexcept {
        const std::array<std::byte, 3> tx{std::byte{reg}, static_cast<std::byte>(value >> 8), static_cast<std::byte>(value)};
        return ok(Bus::write_read(Address, tx, {}));
    }
    static bool read(std::uint8_t reg, std::span<std::byte> rx) noexcept {
        const std::array<std::byte, 1> tx{std::byte{reg}};
        return ok(Bus::write_read(Address, tx, rx));
    }
    [[nodiscard]] static std::optional<std::uint8_t> read8(std::uint8_t reg) noexcept {
        std::array<std::byte, 1> rx{};
        if (!read(reg, rx)) {
            return std::nullopt;
        }
        return std::to_integer<std::uint8_t>(rx[0]);
    }
    [[nodiscard]] static std::optional<std::uint16_t> read16(std::uint8_t reg) noexcept {
        std::array<std::byte, 2> rx{};
        if (!read(reg, rx)) {
            return std::nullopt;
        }
        return static_cast<std::uint16_t>((std::to_integer<unsigned>(rx[0]) << 8) | std::to_integer<unsigned>(rx[1]));
    }

private:
    template <class R>
    static constexpr bool ok(R r) noexcept {
        return r == R::Ok;
    }
};

}  // namespace rtos::devices
