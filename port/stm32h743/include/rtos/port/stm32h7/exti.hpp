// SPDX-License-Identifier: MIT
//
// STM32H743 EXTI: GPIO edge interrupts, lines 0-15 (RM0433 chapters EXTI, SYSCFG; registers,
// bits: SVD v2.4; IRQ numbers: ST's stm32h743xx.h). The hardware watches the pin and latches
// the edge; the interrupt comes even for a pulse far shorter than any polling period.
//
//   using Barrier = ExtiLine<{GpioPort::B, 10}, ExtiEdge::Falling>;
//   Barrier::init();       // port selection, edges, pending cleared
//   Barrier::enable();     // interrupt unmasked (NVIC: Barrier::irq)
//   EXTI15_10 IRQ: if (Barrier::take()) { ... }   // every line of the group: its take()
//
// The same interface as the STM32H5's driver, with the H7's differences:
// * One pending flag per line (CPUPR1), not one per edge: take() reports the configured edge.
//   With ExtiEdge::Both it reports both rising and falling whenever an edge of either kind
//   came - configure one edge where the direction matters.
// * Shared interrupts: lines 0-4 have their own (EXTI0..4, IRQ 6..10), lines 5-9 share
//   EXTI9_5 (23), lines 10-15 EXTI15_10 (40); the handler asks every line of its group.
// * The port of a line is selected in SYSCFG_EXTICR1-4 (4 bits per line; needs the SYSCFG
//   clock, init() turns it on). EXTI itself has no clock enable.
// One pin per line number: PC7 and PA7 cannot both have an EXTI interrupt.
#pragma once

#include <cstdint>

#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/gpio.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

enum class ExtiEdge : std::uint8_t { Rising = 1, Falling = 2, Both = 3 };

/// The edges a take() found (see the note on ExtiEdge::Both above).
struct ExtiEvents {
    bool rising = false, falling = false;
    explicit constexpr operator bool() const noexcept { return rising || falling; }
};

namespace exti_detail {
inline constexpr std::uintptr_t base = 0x5800'0000;
using RTSR1 = Reg<base + 0x00>;
using FTSR1 = Reg<base + 0x04>;
using SWIER1 = Reg<base + 0x08>;
using CPUIMR1 = Reg<base + 0x80>;
using CPUPR1 = Reg<base + 0x88>;
template <unsigned N>
using EXTICR = Reg<0x5800'0400 + 0x08 + 4 * N>;  // SYSCFG_EXTICR1..4

/// NVIC line of EXTI line `line` (0..15).
[[nodiscard]] constexpr std::uint8_t irq(unsigned line) noexcept {
    return line <= 4 ? static_cast<std::uint8_t>(6 + line) : (line <= 9 ? std::uint8_t{23} : std::uint8_t{40});
}
}  // namespace exti_detail

template <gpio::Pin P, ExtiEdge E>
class ExtiLine {
    static_assert(P.number < 16, "EXTI lines 0-15 are the GPIO pins");
    static constexpr unsigned line = P.number;
    static constexpr std::uint32_t bit = 1u << line;
    using CR = exti_detail::EXTICR<line / 4>;
    static constexpr unsigned cr_shift = 4 * (line % 4);
    static constexpr bool rising = (static_cast<unsigned>(E) & 1u) != 0, falling = (static_cast<unsigned>(E) & 2u) != 0;

public:
    /// EXTI0..4: IRQ 6..10; EXTI9_5: 23; EXTI15_10: 40.
    static constexpr std::uint8_t irq = exti_detail::irq(line);
    static constexpr gpio::Pin pin = P;

    /// The pin to its line, the edges, nothing pending; the interrupt stays masked.
    static void init() noexcept {
        using namespace exti_detail;
        rcc::enable(rcc::gate::syscfg);
        disable();
        CR::write((CR::read() & ~(0xFu << cr_shift)) | (static_cast<std::uint32_t>(P.port) << cr_shift));
        RTSR1::write(rising ? RTSR1::read() | bit : RTSR1::read() & ~bit);
        FTSR1::write(falling ? FTSR1::read() | bit : FTSR1::read() & ~bit);
        (void)take();
    }

    static void enable() noexcept { exti_detail::CPUIMR1::write(exti_detail::CPUIMR1::read() | bit); }
    static void disable() noexcept { exti_detail::CPUIMR1::write(exti_detail::CPUIMR1::read() & ~bit); }

    /// The edges latched since the last take(), acknowledged (in the ISR, or to forget edges).
    static ExtiEvents take() noexcept {
        using namespace exti_detail;
        if ((CPUPR1::read() & bit) == 0) {
            return {};
        }
        CPUPR1::write(bit);  // rc_w1
        return {rising, falling};
    }

    /// The interrupt by software: e.g. to look at a level that may already be there when the
    /// line is enabled.
    static void trigger() noexcept { exti_detail::SWIER1::write(bit); }
};

}  // namespace rtos::port::stm32h7
