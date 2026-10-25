// SPDX-License-Identifier: MIT
//
// STM32H563 GPIO. Pins are compile-time values; every operation is a few register
// accesses (BSRR for atomic set/reset - safe from interrupts without locking).
#pragma once

#include <cstdint>

#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5::gpio {

using rcc::GpioPort;

struct Pin {
    GpioPort port;
    std::uint8_t number;  // 0..15
};

enum class Mode : std::uint32_t { Input = 0, Output = 1, Alternate = 2, Analog = 3 };
enum class Speed : std::uint32_t { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };
enum class Pull : std::uint32_t { None = 0, Up = 1, Down = 2 };

namespace detail {
[[nodiscard]] constexpr std::uintptr_t base(GpioPort p) noexcept {
    return periph_alias + 0x0202'0000 + 0x400u * static_cast<std::uintptr_t>(p);
}
inline volatile std::uint32_t& reg(GpioPort p, std::uintptr_t offset) noexcept {
    return *reinterpret_cast<volatile std::uint32_t*>(base(p) + offset);
}
inline constexpr std::uintptr_t moder = 0x00, otyper = 0x04, ospeedr = 0x08, pupdr = 0x0C, idr = 0x10,
                                odr = 0x14, bsrr = 0x18, afrl = 0x20, afrh = 0x24;

inline void field2(GpioPort p, std::uintptr_t offset, unsigned pin, std::uint32_t value) noexcept {
    auto& r = reg(p, offset);
    r = (r & ~(3u << (2 * pin))) | (value << (2 * pin));
}
}  // namespace detail

inline void configure(Pin pin, Mode mode, Speed speed = Speed::Low, Pull pull = Pull::None,
                      bool open_drain = false) noexcept {
    rcc::enable(pin.port);
    detail::field2(pin.port, detail::ospeedr, pin.number, static_cast<std::uint32_t>(speed));
    detail::field2(pin.port, detail::pupdr, pin.number, static_cast<std::uint32_t>(pull));
    auto& otyper = detail::reg(pin.port, detail::otyper);
    otyper = (otyper & ~(1u << pin.number)) | (open_drain ? 1u << pin.number : 0u);
    detail::field2(pin.port, detail::moder, pin.number, static_cast<std::uint32_t>(mode));
}

/// Route `pin` to alternate function `af` (0..15).
inline void configure_alternate(Pin pin, std::uint32_t af, Speed speed = Speed::High,
                                Pull pull = Pull::None) noexcept {
    rcc::enable(pin.port);
    auto& afr = detail::reg(pin.port, pin.number < 8 ? detail::afrl : detail::afrh);
    const unsigned shift = 4u * (pin.number % 8);
    afr = (afr & ~(0xFu << shift)) | ((af & 0xFu) << shift);
    configure(pin, Mode::Alternate, speed, pull);
}

inline void write(Pin pin, bool high) noexcept {
    detail::reg(pin.port, detail::bsrr) = high ? (1u << pin.number) : (1u << (pin.number + 16));
}

inline void toggle(Pin pin) noexcept {
    const bool high = (detail::reg(pin.port, detail::odr) >> pin.number) & 1u;
    write(pin, !high);
}

[[nodiscard]] inline bool read(Pin pin) noexcept {
    return (detail::reg(pin.port, detail::idr) >> pin.number) & 1u;
}

}  // namespace rtos::port::stm32h5::gpio
