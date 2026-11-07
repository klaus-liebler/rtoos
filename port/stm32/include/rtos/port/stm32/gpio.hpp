// SPDX-License-Identifier: MIT
//
// STM32 GPIO, the port IP of the STM32H5 and STM32G4 (RM0481/RM0440 chapter GPIO; the register
// layout MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, AFRL/AFRH is the same, the SVDs differ
// only in field names; the H5 adds HSLVR and SECCFGR, which rtoos leaves at reset).
//
// Pin configuration is a compile-time list: the compiler folds it into one mask and one
// value per port and register (MODER, OTYPER, OSPEEDR, PUPDR, AFRL/AFRH, initial levels in
// BSRR) and rejects contradictions and impossible pins. Alternate functions are named by their
// signal and looked up in the chip's pin database (ST's pin data, tools/gen_pin_functions.py),
// so a pin without that function, or one the package lacks, does not compile. At run time
// apply() does one read-modify-write per register and port that the list touches.
//
// This is the engine; the chips wrap it with their database and register addresses
// (stm32h5::gpio, stm32g4::gpio), so applications write
//
//   inline constexpr std::array pins{
//       gpio::output({GpioPort::H, 1}, /*high*/ true),                   // LED, off (active low)
//       gpio::alternate({GpioPort::D, 8}, Signal::USART3_TX),
//       gpio::alternate({GpioPort::D, 9}, Signal::USART3_RX, Speed::Low, Pull::Up),
//   };
//   gpio::configure<pins>();
//
// write/read/toggle stay plain functions: with a constant Pin they compile to one access
// (BSRR for atomic set/reset - safe from interrupts without locking).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>

#include "rtos/port/stm32/rcc.hpp"

namespace rtos::port::stm32::gpio {

using stm32::GpioPort;

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
[[nodiscard]] constexpr PinConfig analog(Pin p) noexcept { return {.pin = p, .mode = Mode::Analog}; }

// ---- pin database ------------------------------------------------------------------------

/// af of a function without an AF number: an additional (analog) function - ADC input, UCPD
/// CC line, USB DP/DM, ...
inline constexpr std::uint8_t additional = 0xFF;

/// One function of one pin in a chip's database (pin_functions.hpp).
template <class Signal>
struct PinFunction {
    GpioPort port;
    std::uint8_t number;
    Signal signal;
    std::uint8_t af;  ///< 0..15, or additional
};

namespace detail {
// Not constexpr: calling them during constant evaluation stops the build, and the compiler
// error names the function - the reason.
void error_pin_has_no_such_alternate_function();
void error_pin_has_no_such_analog_function();
void error_pin_or_alternate_function_above_15();
void error_pin_configured_twice();
void error_pin_not_in_package();
}  // namespace detail

/// The function `signal` of pin `p` in `table`; -1 if the pin has none.
template <class Table, class Signal>
[[nodiscard]] consteval int function_af(const Table& table, Pin p, Signal signal) {
    for (const auto& f : table) {
        if (f.port == p.port && f.number == p.number && f.signal == signal) {
            return f.af;
        }
    }
    return -1;
}

/// Pin `p` as peripheral signal `signal`; the AF number comes from `table`, a pin without that
/// function does not compile.
template <class Table, class Signal>
[[nodiscard]] consteval PinConfig alternate(const Table& table, Pin p, Signal signal, Speed speed, Pull pull,
                                            bool open_drain) {
    const int af = function_af(table, p, signal);
    if (af < 0 || af == additional) {
        detail::error_pin_has_no_such_alternate_function();
    }
    return {.pin = p, .mode = Mode::Alternate, .speed = speed, .pull = pull, .open_drain = open_drain,
            .af = static_cast<std::uint8_t>(af)};
}

/// Pin `p` analog for its additional function `signal`; a pin without it does not compile.
template <class Table, class Signal>
[[nodiscard]] consteval PinConfig analog(const Table& table, Pin p, Signal signal) {
    if (function_af(table, p, signal) != additional) {
        detail::error_pin_has_no_such_analog_function();
    }
    return analog(p);
}

// ---- pin names ---------------------------------------------------------------------------

/// 'A' for port A, ...
[[nodiscard]] constexpr char port_letter(Pin p) noexcept { return static_cast<char>('A' + static_cast<int>(p.port)); }

/// "PC7", "pa15": a pin of the package (`package_pins`: {port, number} pairs; by name, e.g.
/// typed into a terminal); else nullopt.
template <class Package>
[[nodiscard]] constexpr std::optional<Pin> parse_pin(const Package& package_pins, std::string_view s) noexcept {
    if (s.size() < 3 || s.size() > 4 || (s[0] != 'P' && s[0] != 'p')) {
        return std::nullopt;
    }
    const char letter = static_cast<char>(s[1] & ~0x20);
    unsigned number = 0;
    for (const char c : s.substr(2)) {
        if (c < '0' || c > '9') {
            return std::nullopt;
        }
        number = number * 10 + static_cast<unsigned>(c - '0');
    }
    for (const auto& [port, n] : package_pins) {
        if (letter == port_letter({port, n}) && number == n) {
            return Pin{port, n};
        }
    }
    return std::nullopt;
}

// ---- compile-time plan -------------------------------------------------------------------

/// What apply() writes into one port (computed at compile time).
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
/// twice, a pin the package does not have (`package_pins`: {port, number} pairs), a pin
/// number or alternate function above 15.
template <std::size_t Ports, std::size_t N, class Package>
consteval std::array<PortPlan, Ports> plan(const std::array<PinConfig, N>& pins, const Package& package_pins) {
    std::array<PortPlan, Ports> plans{};
    for (std::size_t i = 0; i < N; ++i) {
        const PinConfig& c = pins[i];
        if (c.pin.number > 15 || c.af > 15) {
            detail::error_pin_or_alternate_function_above_15();
        }
        bool bonded = false;
        for (const auto& [port, number] : package_pins) {
            bonded = bonded || (port == c.pin.port && number == c.pin.number);
        }
        if (!bonded || static_cast<std::size_t>(c.pin.port) >= Ports) {
            detail::error_pin_not_in_package();
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

// ---- registers -----------------------------------------------------------------------------

/// Where a chip's GPIO ports are: GPIOx at base + 0x400 x, the clock of port x is bit x of the
/// RCC register at `clock_enr`.
struct GpioHw {
    std::uintptr_t base;       ///< GPIOA
    std::uintptr_t clock_enr;  ///< RCC_AHB2ENR (H5, G4): GPIOxEN = bit x
    std::size_t port_count;
};

namespace detail {
inline constexpr std::uintptr_t moder = 0x00, otyper = 0x04, ospeedr = 0x08, pupdr = 0x0C, idr = 0x10,
                                odr = 0x14, bsrr = 0x18, afrl = 0x20, afrh = 0x24;

template <GpioHw Hw>
inline volatile std::uint32_t& reg(GpioPort p, std::uintptr_t offset) noexcept {
    return *reinterpret_cast<volatile std::uint32_t*>(Hw.base + 0x400u * static_cast<std::uintptr_t>(p) + offset);
}

template <GpioHw Hw>
inline void merge(GpioPort port, std::uintptr_t offset, std::uint32_t mask, std::uint32_t value) noexcept {
    if (mask != 0) {
        auto& r = reg<Hw>(port, offset);
        r = (r & ~mask) | value;
    }
}

template <GpioHw Hw, PortPlan P, GpioPort Port>
void apply_port() noexcept {
    if constexpr (P.used) {
        rcc::enable(rcc::Gate{Hw.clock_enr, Hw.clock_enr, static_cast<std::uint8_t>(Port)});
        // levels and alternate functions first, the mode last: no glitch on the pins
        if constexpr (P.bsrr != 0) {
            reg<Hw>(Port, bsrr) = P.bsrr;
        }
        merge<Hw>(Port, afrl, P.afl_mask, P.afl);
        merge<Hw>(Port, afrh, P.afh_mask, P.afh);
        merge<Hw>(Port, otyper, P.type_mask, P.type);
        merge<Hw>(Port, ospeedr, P.speed_mask, P.speed);
        merge<Hw>(Port, pupdr, P.pull_mask, P.pull);
        merge<Hw>(Port, moder, P.mode_mask, P.mode);
    }
}

template <GpioHw Hw, auto Plans, std::size_t... I>
void apply_all(std::index_sequence<I...>) noexcept {
    (apply_port<Hw, Plans[I], static_cast<GpioPort>(I)>(), ...);
}
}  // namespace detail

/// Write the planned values (plan()) into the ports of the chip `Hw`.
template <GpioHw Hw, auto Plans>
void apply() noexcept {
    static_assert(Plans.size() == Hw.port_count);
    detail::apply_all<Hw, Plans>(std::make_index_sequence<Hw.port_count>{});
}

template <GpioHw Hw>
inline void write(Pin pin, bool high) noexcept {
    detail::reg<Hw>(pin.port, detail::bsrr) = high ? (1u << pin.number) : (1u << (pin.number + 16));
}

template <GpioHw Hw>
inline void toggle(Pin pin) noexcept {
    const bool high = (detail::reg<Hw>(pin.port, detail::odr) >> pin.number) & 1u;
    write<Hw>(pin, !high);
}

template <GpioHw Hw>
[[nodiscard]] inline bool read(Pin pin) noexcept {
    return (detail::reg<Hw>(pin.port, detail::idr) >> pin.number) & 1u;
}

}  // namespace rtos::port::stm32::gpio
