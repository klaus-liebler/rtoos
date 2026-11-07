// SPDX-License-Identifier: MIT
//
// STM32 true random number generator, the RNG of the STM32H5, STM32G4 and STM32H7 (RM0481/RM0440/RM0433 chapter
// RNG; CR.RNGEN, SR.DRDY/SECS/SEIS and DR are the same in the SVDs - H5/G4 v1.8, H7 v2.4; the H5 adds
// configuration fields that stay at reset). Kernel clock: HSI48 on both chips (H5
// RCC_CCIPR5.RNGSEL, G4 RCC_CCIPR.CLK48SEL: reset value 00), which RngHw::clock_on starts.
#pragma once

#include <cstdint>

#include "rtos/port/cortex_m/registers.hpp"
#include "rtos/port/stm32/rcc.hpp"

namespace rtos::port::stm32 {

/// The RNG of a chip.
struct RngHw {
    std::uintptr_t base;
    rcc::Gate gate;
    void (*clock_on)() noexcept;  ///< starts the 48 MHz kernel clock (HSI48)
};

template <RngHw Hw>
class Rng {
public:
    static void init() noexcept {
        Hw.clock_on();
        rcc::enable(Hw.gate);
        CR::set(cr_rngen);
    }

    /// 32 random bits. Recovers from a seed error by re-enabling the generator.
    [[nodiscard]] static std::uint32_t next() noexcept {
        for (;;) {
            const std::uint32_t sr = SR::read();
            if ((sr & (sr_secs | sr_seis)) != 0) {
                SR::clear(sr_seis);
                CR::clear(cr_rngen);
                CR::set(cr_rngen);
                continue;
            }
            if ((sr & sr_drdy) != 0) {
                return DR::read();
            }
        }
    }

private:
    using CR = cortex_m::Reg<Hw.base + 0x0>;
    using SR = cortex_m::Reg<Hw.base + 0x4>;
    using DR = cortex_m::Reg<Hw.base + 0x8>;
    static constexpr std::uint32_t cr_rngen = 1u << 2;
    static constexpr std::uint32_t sr_drdy = 1u << 0;
    static constexpr std::uint32_t sr_secs = 1u << 2;
    static constexpr std::uint32_t sr_seis = 1u << 6;
};

}  // namespace rtos::port::stm32
