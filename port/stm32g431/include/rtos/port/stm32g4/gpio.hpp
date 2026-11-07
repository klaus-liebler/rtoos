// SPDX-License-Identifier: MIT
//
// STM32G431 GPIO: the shared STM32 GPIO engine (stm32/gpio.hpp) with the G431's ports A..G at
// 0x48000000 (RM0440, SVD v1.8; clocks: RCC_AHB2ENR.GPIOxEN = bit x) and the pin database of
// the package the board uses (pin_functions.hpp, tools/gen_pin_functions.py).
//
// The package is a build setting (CMake RTOS_STM32G431_PACKAGE -> RTOS_STM32G431_PACKAGE, an
// enumerator of gpio::Package, e.g. UFQFPN48 for an STM32G431CBU6). The tables of all packages
// are filtered for it at compile time: a pin the package lacks, or a function the package does
// not bring out on that pin, does not compile.
//
//   inline constexpr std::array pins{
//       gpio::output({GpioPort::C, 6}),                                   // LED
//       gpio::alternate({GpioPort::C, 4}, Signal::USART1_TX),
//       gpio::alternate({GpioPort::B, 7}, Signal::USART1_RX, Speed::Low, Pull::Up),
//   };
//   gpio::configure<pins>();
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "rtos/port/stm32/gpio.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/pin_functions.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

#ifndef RTOS_STM32G431_PACKAGE
#error "RTOS_STM32G431_PACKAGE (gpio::Package of the board's chip) is set by the rtos_stm32g431 CMake target"
#endif

namespace rtos::port::stm32g4::gpio {

using stm32::gpio::input, stm32::gpio::output, stm32::gpio::analog;
using stm32::gpio::Mode, stm32::gpio::Pin, stm32::gpio::PinConfig, stm32::gpio::PortPlan, stm32::gpio::Pull,
    stm32::gpio::Speed;

/// The package of this build.
inline constexpr Package package = Package::RTOS_STM32G431_PACKAGE;

inline constexpr std::size_t port_count = 7;  // A..G
inline constexpr stm32::gpio::GpioHw hw{.base = 0x4800'0000,
                                        .clock_enr = rcc::base + 0x4C,  // RCC_AHB2ENR
                                        .port_count = port_count};

namespace detail {
[[nodiscard]] consteval bool has(std::uint16_t packages, Package p) {
    return (packages & (1u << static_cast<unsigned>(p))) != 0;
}
template <Package P>
consteval auto functions_of() {
    constexpr std::size_t n = [] {
        std::size_t count = 0;
        for (const auto& f : all_pin_functions) {
            count += has(f.packages, P) ? 1 : 0;
        }
        return count;
    }();
    std::array<PinFunction, n> out{};
    std::size_t i = 0;
    for (const auto& f : all_pin_functions) {
        if (has(f.packages, P)) {
            out[i++] = f.function;
        }
    }
    return out;
}
template <Package P>
consteval auto pins_of() {
    constexpr std::size_t n = [] {
        std::size_t count = 0;
        for (const auto& p : all_pins) {
            count += has(p.packages, P) ? 1 : 0;
        }
        return count;
    }();
    std::array<std::pair<GpioPort, std::uint8_t>, n> out{};
    std::size_t i = 0;
    for (const auto& p : all_pins) {
        if (has(p.packages, P)) {
            out[i++] = {p.port, p.number};
        }
    }
    return out;
}
}  // namespace detail

/// The functions of the pins of this package.
inline constexpr auto pin_functions = detail::functions_of<package>();
/// Every I/O pin of this package.
inline constexpr auto package_pins = detail::pins_of<package>();

/// The pin's function `signal` in this package; -1 if it has none.
[[nodiscard]] consteval int function_af(Pin p, Signal signal) {
    return stm32::gpio::function_af(pin_functions, p, signal);
}

/// Pin `p` as peripheral signal `signal`, e.g. alternate({GpioPort::C, 4}, Signal::USART1_TX).
/// The AF number comes from ST's pin database; a pin without that function does not compile.
[[nodiscard]] consteval PinConfig alternate(Pin p, Signal signal, Speed speed = Speed::High, Pull pull = Pull::None,
                                            bool open_drain = false) {
    return stm32::gpio::alternate(pin_functions, p, signal, speed, pull, open_drain);
}

/// Pin `p` analog for its additional function `signal` (ADC input, UCPD CC line, ...).
[[nodiscard]] consteval PinConfig analog(Pin p, Signal signal) { return stm32::gpio::analog(pin_functions, p, signal); }

/// Fold a pin list into per-port register values (rejects pins the package lacks, a pin
/// twice, numbers above 15).
template <std::size_t N>
consteval std::array<PortPlan, port_count> plan(const std::array<PinConfig, N>& pins) {
    return stm32::gpio::plan<port_count>(pins, package_pins);
}

/// Configure every pin in `Pins` (a constexpr std::array<PinConfig, N>).
template <auto Pins>
void configure() noexcept {
    static constexpr auto plans = plan(Pins);
    stm32::gpio::apply<hw, plans>();
}

using stm32::gpio::port_letter;
/// "PC4", "pa15": a pin of this package (by name, e.g. typed into a terminal); else nullopt.
[[nodiscard]] constexpr std::optional<Pin> parse_pin(std::string_view s) noexcept {
    return stm32::gpio::parse_pin(package_pins, s);
}

inline void write(Pin pin, bool high) noexcept { stm32::gpio::write<hw>(pin, high); }
inline void toggle(Pin pin) noexcept { stm32::gpio::toggle<hw>(pin); }
[[nodiscard]] inline bool read(Pin pin) noexcept { return stm32::gpio::read<hw>(pin); }

}  // namespace rtos::port::stm32g4::gpio
