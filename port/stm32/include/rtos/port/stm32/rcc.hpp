// SPDX-License-Identifier: MIT
//
// What the STM32 reset and clock controllers have in common: every peripheral has one bit in a
// clock enable register (RCC_xxxENR) and the same bit in a reset register (RCC_xxxRSTR), and
// many have a kernel clock multiplexer (a field in RCC_CCIPRx). Where those registers are is
// chip-specific; the chips (stm32h5::rcc, stm32g4::rcc) define the gates and selections with
// absolute addresses, the shared drivers take them as part of their hardware description.
#pragma once

#include <cstdint>

namespace rtos::port::stm32 {

/// GPIO ports by letter; which ones a chip has: its gpio::port_count / package_pins.
enum class GpioPort : std::uint8_t { A, B, C, D, E, F, G, H, I };

namespace rcc {

/// A peripheral's bit in a clock enable register and in the matching reset register.
struct Gate {
    std::uintptr_t enr;   ///< RCC_xxxENR (absolute address)
    std::uintptr_t rstr;  ///< RCC_xxxRSTR (absolute address)
    std::uint8_t bit;
};

/// A kernel clock multiplexer: `width` bits at `pos` in the register at `reg`.
struct KernelClock {
    std::uintptr_t reg;  ///< RCC_CCIPRx (absolute address)
    std::uint8_t pos;
    std::uint8_t width;
};

/// A kernel clock choice: the multiplexer and the value that selects the clock.
struct ClockSelection {
    KernelClock mux;
    std::uint32_t value;
};

namespace detail {
inline volatile std::uint32_t& reg(std::uintptr_t address) noexcept {
    return *reinterpret_cast<volatile std::uint32_t*>(address);
}
}  // namespace detail

inline void enable(Gate g) noexcept {
    auto& enr = detail::reg(g.enr);
    enr = enr | (1u << g.bit);
    [[maybe_unused]] const std::uint32_t settle = enr;  // RM0481/RM0440: 2 cycles after enabling
}

inline void disable(Gate g) noexcept {
    auto& enr = detail::reg(g.enr);
    enr = enr & ~(1u << g.bit);
}

/// Pulse the peripheral's reset: every register back to its reset value (e.g. a stuck I2C).
inline void reset(Gate g) noexcept {
    auto& rstr = detail::reg(g.rstr);
    rstr = rstr | (1u << g.bit);
    rstr = rstr & ~(1u << g.bit);
}

inline void select(KernelClock k, std::uint32_t value) noexcept {
    auto& r = detail::reg(k.reg);
    const std::uint32_t mask = ((1u << k.width) - 1u) << k.pos;
    r = (r & ~mask) | ((value << k.pos) & mask);
}

inline void select(ClockSelection s) noexcept { select(s.mux, s.value); }

}  // namespace rcc
}  // namespace rtos::port::stm32
