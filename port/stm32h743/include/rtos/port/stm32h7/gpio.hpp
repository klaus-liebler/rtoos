// SPDX-License-Identifier: MIT
//
// STM32H743 GPIO: the shared STM32 GPIO engine (stm32/gpio.hpp) with the H743's ports at
// 0x5802'0000, 0x400 apart (RM0433, SVD v2.4; clocks: RCC_AHB4ENR.GPIOxEN = bit x) and the pin
// database of the LQFP100 package (pin_functions.hpp, tools/gen_pin_functions.py).
//
//   inline constexpr std::array pins{
//       gpio::output({GpioPort::E, 1}, /*high*/ true),                   // LED, off (active low)
//       gpio::alternate({GpioPort::D, 8}, Signal::USART3_TX),
//       gpio::alternate({GpioPort::D, 9}, Signal::USART3_RX, Speed::Low, Pull::Up),
//   };
//   gpio::configure<pins>();
//
// A pin without the named function, or one the LQFP100 lacks, does not compile. The LQFP100 has
// ports A..E and H (PH0/PH1); the chip's ports F, G, I, J, K are not bonded out.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32/gpio.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/pin_functions.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7::gpio {

using stm32::gpio::input, stm32::gpio::output, stm32::gpio::analog;
using stm32::gpio::Mode, stm32::gpio::Pin, stm32::gpio::PinConfig, stm32::gpio::PortPlan, stm32::gpio::Pull,
    stm32::gpio::Speed;

inline constexpr std::size_t port_count = 8;  // A..H (the package's highest port is H)
inline constexpr stm32::gpio::GpioHw hw{.base = 0x5802'0000, .clock_enr = rcc::gpio_clock_enr, .port_count = port_count};

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

/// Pin `p` analog for its additional function `signal` (ADC input, USB DP/DM, ...); a pin
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
/// "PD8", "pe15": a pin of the LQFP100 (by name, e.g. typed into a terminal); else nullopt.
[[nodiscard]] constexpr std::optional<Pin> parse_pin(std::string_view s) noexcept {
    return stm32::gpio::parse_pin(package_pins, s);
}

inline void write(Pin pin, bool high) noexcept { stm32::gpio::write<hw>(pin, high); }
inline void toggle(Pin pin) noexcept { stm32::gpio::toggle<hw>(pin); }
[[nodiscard]] inline bool read(Pin pin) noexcept { return stm32::gpio::read<hw>(pin); }

}  // namespace rtos::port::stm32h7::gpio
