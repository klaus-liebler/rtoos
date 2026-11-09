// SPDX-License-Identifier: MIT
//
// STM32H563 GPIO: the shared STM32 GPIO engine (stm32/gpio.hpp) with the H563's ports A..I at
// 0x42020000 (RM0481, SVD v1.8; clocks: RCC_AHB2ENR.GPIOxEN = bit x) and the pin database of the
// LQFP100 package (pin_functions.hpp, tools/gen_pin_functions.py).
//
//   inline constexpr std::array pins{
//       gpio::output({GpioPort::H, 1}, /*high*/ true),                   // LED, off (active low)
//       gpio::alternate({GpioPort::D, 8}, Signal::USART3_TX),
//       gpio::alternate({GpioPort::D, 9}, Signal::USART3_RX, Speed::Low, Pull::Up),
//   };
//   gpio::configure<pins>();
//
// A pin without the named function, or one the LQFP100 lacks, does not compile.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32/gpio.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/pin_functions.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5::gpio {

using stm32::gpio::input, stm32::gpio::output, stm32::gpio::analog, stm32::gpio::pins_distinct;
using stm32::gpio::Mode, stm32::gpio::Pin, stm32::gpio::PinConfig, stm32::gpio::PortPlan, stm32::gpio::Pull,
    stm32::gpio::Speed;

inline constexpr std::size_t port_count = 9;  // A..I
inline constexpr stm32::gpio::GpioHw hw{.base = periph_alias + 0x0202'0000,
                                        .clock_enr = periph_alias + 0x0402'0C00 + 0x8C,  // RCC_AHB2ENR
                                        .port_count = port_count};

/// The pin's function `signal` (pin_functions.hpp, ST's pin database); -1 if it has none.
[[nodiscard]] consteval int function_af(Pin p, Signal signal) {
    return stm32::gpio::function_af(pin_functions, p, signal);
}

/// Pin `p` as peripheral signal `signal`, e.g. alternate({GpioPort::D, 8}, Signal::USART3_TX).
/// The AF number comes from ST's pin database; a pin without that function does not compile.
[[nodiscard]] consteval PinConfig alternate(Pin p, Signal signal, Speed speed = Speed::High, Pull pull = Pull::None,
                                            bool open_drain = false) {
    return stm32::gpio::alternate(pin_functions, p, signal, speed, pull, open_drain);
}

/// Pin `p` analog for its additional function `signal` (ADC input, UCPD CC line, ...); a pin
/// without that function does not compile.
[[nodiscard]] consteval PinConfig analog(Pin p, Signal signal) { return stm32::gpio::analog(pin_functions, p, signal); }

/// Fold a pin list into per-port register values (rejects pins the LQFP100 lacks, a pin
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
/// "PC7", "pa15": a pin of the LQFP100 (by name, e.g. typed into a terminal); else nullopt.
[[nodiscard]] constexpr std::optional<Pin> parse_pin(std::string_view s) noexcept {
    return stm32::gpio::parse_pin(package_pins, s);
}

inline void write(Pin pin, bool high) noexcept { stm32::gpio::write<hw>(pin, high); }
inline void toggle(Pin pin) noexcept { stm32::gpio::toggle<hw>(pin); }
[[nodiscard]] inline bool read(Pin pin) noexcept { return stm32::gpio::read<hw>(pin); }

}  // namespace rtos::port::stm32h5::gpio
