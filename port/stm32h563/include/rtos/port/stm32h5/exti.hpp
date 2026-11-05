// SPDX-License-Identifier: MIT
//
// STM32H563 EXTI: GPIO edge interrupts, lines 0-15 (RM0481 chapter EXTI; registers, bits,
// IRQ numbers: SVD v1.8). The hardware watches the pin and latches the edge; the interrupt
// comes even for a pulse far shorter than any polling period.
//
//   using Barrier = ExtiLine<{GpioPort::C, 7}, ExtiEdge::Both>;
//   Barrier::init();       // port selection, edges, pending cleared
//   Barrier::enable();     // interrupt unmasked (NVIC: Barrier::irq)
//   EXTI7 IRQ: if (const auto e = Barrier::take()) { e.falling ... }
//
// One pin per line number: PC7 and PA7 cannot both have an EXTI interrupt. The pin's mode does
// not matter (input, alternate function: the input path always sees the level). EXTI has no
// clock enable (always on).
#pragma once

#include <cstdint>

#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/gpio.hpp"

namespace rtos::port::stm32h5 {

enum class ExtiEdge : std::uint8_t { Rising = 1, Falling = 2, Both = 3 };

/// The edges a take() found (both can be set: a pulse between two looks).
struct ExtiEvents {
    bool rising = false, falling = false;
    explicit constexpr operator bool() const noexcept { return rising || falling; }
};

namespace exti_detail {
inline constexpr std::uintptr_t base = 0x0402'2000;
using RTSR1 = PReg<base + 0x00>;
using FTSR1 = PReg<base + 0x04>;
using SWIER1 = PReg<base + 0x08>;
using RPR1 = PReg<base + 0x0C>;
using FPR1 = PReg<base + 0x10>;
template <unsigned N>
using EXTICR = PReg<base + 0x60 + 4 * N>;
using IMR1 = PReg<base + 0x80>;
}  // namespace exti_detail

template <gpio::Pin P, ExtiEdge E>
class ExtiLine {
    static_assert(P.number < 16, "EXTI lines 0-15 are the GPIO pins");
    static constexpr unsigned line = P.number;
    static constexpr std::uint32_t bit = 1u << line;
    using CR = exti_detail::EXTICR<line / 4>;
    static constexpr unsigned cr_shift = 8 * (line % 4);

public:
    /// EXTI0..EXTI15: IRQ 11..26.
    static constexpr std::uint8_t irq = static_cast<std::uint8_t>(11 + line);
    static constexpr gpio::Pin pin = P;

    /// The pin to its line, the edges, nothing pending; the interrupt stays masked.
    static void init() noexcept {
        using namespace exti_detail;
        disable();
        CR::write((CR::read() & ~(0xFFu << cr_shift)) | (static_cast<std::uint32_t>(P.port) << cr_shift));
        const bool rising = (static_cast<unsigned>(E) & 1u) != 0, falling = (static_cast<unsigned>(E) & 2u) != 0;
        RTSR1::write(rising ? RTSR1::read() | bit : RTSR1::read() & ~bit);
        FTSR1::write(falling ? FTSR1::read() | bit : FTSR1::read() & ~bit);
        (void)take();
    }

    static void enable() noexcept { exti_detail::IMR1::write(exti_detail::IMR1::read() | bit); }
    static void disable() noexcept { exti_detail::IMR1::write(exti_detail::IMR1::read() & ~bit); }

    /// The edges latched since the last take(), acknowledged (in the ISR, or to forget edges).
    static ExtiEvents take() noexcept {
        using namespace exti_detail;
        const ExtiEvents e{(RPR1::read() & bit) != 0, (FPR1::read() & bit) != 0};
        if (e.rising) {
            RPR1::write(bit);
        }
        if (e.falling) {
            FPR1::write(bit);
        }
        return e;
    }

    /// The interrupt by software (a "rising" event): e.g. to look at a level that may already
    /// be there when the line is enabled.
    static void trigger() noexcept { exti_detail::SWIER1::write(bit); }
};

}  // namespace rtos::port::stm32h5
