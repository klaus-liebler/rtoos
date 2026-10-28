// SPDX-License-Identifier: MIT
//
// STM32H563 GPIO (RM0481 chapter GPIO; registers: SVD v1.8).
//
// Pin configuration is a compile-time list: the compiler folds it into one mask and one
// value per port and register (MODER, OTYPER, OSPEEDR, PUPDR, AFRL/AFRH, initial levels in
// BSRR) and rejects contradictions; at run time configure<Pins>() does one read-modify-write
// per register and port that the list touches.
//
//   inline constexpr std::array pins{
//       gpio::output({GpioPort::H, 1}, /*high*/ true),                   // LED, off (active low)
//       gpio::alternate({GpioPort::D, 8}, 7),                            // USART3 TX
//       gpio::alternate({GpioPort::D, 9}, 7, Speed::Low, Pull::Up),      // USART3 RX
//   };
//   gpio::configure<pins>();
//
// write/read/toggle stay plain functions: with a constant Pin they compile to one access
// (BSRR for atomic set/reset - safe from interrupts without locking).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5::gpio {

using rcc::GpioPort;

struct Pin {
    GpioPort port;
    std::uint8_t number;  // 0..15
    friend constexpr bool operator==(const Pin&, const Pin&) = default;
};

enum class Mode : std::uint8_t { Input = 0, Output = 1, Alternate = 2, Analog = 3 };
enum class Speed : std::uint8_t { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };
enum class Pull : std::uint8_t { None = 0, Up = 1, Down = 2 };

struct PinConfig {
    Pin pin;
    Mode mode = Mode::Input;
    Speed speed = Speed::Low;
    Pull pull = Pull::None;
    bool open_drain = false;
    std::uint8_t af = 0;       ///< alternate function 0..15 (Mode::Alternate)
    bool initial_high = false; ///< output level set before the pin becomes an output
};

[[nodiscard]] constexpr PinConfig input(Pin p, Pull pull = Pull::None) noexcept {
    return {.pin = p, .mode = Mode::Input, .pull = pull};
}
[[nodiscard]] constexpr PinConfig output(Pin p, bool initial_high = false, Speed speed = Speed::Low,
                                         bool open_drain = false) noexcept {
    return {.pin = p, .mode = Mode::Output, .speed = speed, .open_drain = open_drain, .initial_high = initial_high};
}
[[nodiscard]] constexpr PinConfig alternate(Pin p, std::uint8_t af, Speed speed = Speed::High, Pull pull = Pull::None,
                                            bool open_drain = false) noexcept {
    return {.pin = p, .mode = Mode::Alternate, .speed = speed, .pull = pull, .open_drain = open_drain, .af = af};
}
[[nodiscard]] constexpr PinConfig analog(Pin p) noexcept { return {.pin = p, .mode = Mode::Analog}; }

inline constexpr std::size_t port_count = 9;  // A..I

namespace detail {
// Not constexpr: calling them during constant evaluation stops the build, and the compiler
// error names the function - the reason.
void error_pin_or_alternate_function_above_15();
void error_pin_configured_twice();
}  // namespace detail

/// What configure() writes into one port (computed at compile time).
struct PortPlan {
    bool used = false;
    std::uint32_t mode_mask = 0, mode = 0;
    std::uint32_t type_mask = 0, type = 0;
    std::uint32_t speed_mask = 0, speed = 0;
    std::uint32_t pull_mask = 0, pull = 0;
    std::uint32_t afl_mask = 0, afl = 0, afh_mask = 0, afh = 0;
    std::uint32_t bsrr = 0;  ///< initial levels of the outputs
};

/// Fold a pin list into per-port register values. Contradictions are errors: the same pin
/// twice, a pin number above 15, an alternate function above 15.
template <std::size_t N>
consteval std::array<PortPlan, port_count> plan(const std::array<PinConfig, N>& pins) {
    std::array<PortPlan, port_count> plans{};
    for (std::size_t i = 0; i < N; ++i) {
        const PinConfig& c = pins[i];
        if (c.pin.number > 15 || c.af > 15) {
            detail::error_pin_or_alternate_function_above_15();
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (pins[j].pin == c.pin) {
                detail::error_pin_configured_twice();
            }
        }
        PortPlan& p = plans[static_cast<std::size_t>(c.pin.port)];
        const unsigned n = c.pin.number;
        p.used = true;
        p.mode_mask |= 3u << (2 * n);
        p.mode |= static_cast<std::uint32_t>(c.mode) << (2 * n);
        p.type_mask |= 1u << n;
        p.type |= (c.open_drain ? 1u : 0u) << n;
        p.speed_mask |= 3u << (2 * n);
        p.speed |= static_cast<std::uint32_t>(c.speed) << (2 * n);
        p.pull_mask |= 3u << (2 * n);
        p.pull |= static_cast<std::uint32_t>(c.pull) << (2 * n);
        if (c.mode == Mode::Alternate) {
            const unsigned shift = 4 * (n % 8);
            (n < 8 ? p.afl_mask : p.afh_mask) |= 0xFu << shift;
            (n < 8 ? p.afl : p.afh) |= static_cast<std::uint32_t>(c.af) << shift;
        }
        if (c.mode == Mode::Output) {
            p.bsrr |= c.initial_high ? (1u << n) : (1u << (n + 16));
        }
    }
    return plans;
}

namespace detail {
[[nodiscard]] constexpr std::uintptr_t base(GpioPort p) noexcept {
    return periph_alias + 0x0202'0000 + 0x400u * static_cast<std::uintptr_t>(p);
}
inline volatile std::uint32_t& reg(GpioPort p, std::uintptr_t offset) noexcept {
    return *reinterpret_cast<volatile std::uint32_t*>(base(p) + offset);
}
inline constexpr std::uintptr_t moder = 0x00, otyper = 0x04, ospeedr = 0x08, pupdr = 0x0C, idr = 0x10,
                                odr = 0x14, bsrr = 0x18, afrl = 0x20, afrh = 0x24;

inline void merge(GpioPort port, std::uintptr_t offset, std::uint32_t mask, std::uint32_t value) noexcept {
    if (mask != 0) {
        auto& r = reg(port, offset);
        r = (r & ~mask) | value;
    }
}

template <PortPlan P, GpioPort Port>
void apply() noexcept {
    if constexpr (P.used) {
        rcc::enable(Port);
        // levels and alternate functions first, the mode last: no glitch on the pins
        if constexpr (P.bsrr != 0) {
            reg(Port, bsrr) = P.bsrr;
        }
        merge(Port, afrl, P.afl_mask, P.afl);
        merge(Port, afrh, P.afh_mask, P.afh);
        merge(Port, otyper, P.type_mask, P.type);
        merge(Port, ospeedr, P.speed_mask, P.speed);
        merge(Port, pupdr, P.pull_mask, P.pull);
        merge(Port, moder, P.mode_mask, P.mode);
    }
}

template <auto Plans, std::size_t... I>
void apply_all(std::index_sequence<I...>) noexcept {
    (apply<Plans[I], static_cast<GpioPort>(I)>(), ...);
}
}  // namespace detail

/// Configure every pin in `Pins` (a constexpr std::array<PinConfig, N>).
template <auto Pins>
void configure() noexcept {
    static constexpr auto plans = plan(Pins);
    detail::apply_all<plans>(std::make_index_sequence<port_count>{});
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
