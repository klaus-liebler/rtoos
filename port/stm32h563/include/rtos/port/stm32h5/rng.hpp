// SPDX-License-Identifier: MIT
//
// STM32H5 true random number generator (RNG at 0x420C0800, ST SVD STM32H563 v1.8).
// Kernel clock: HSI48 (RCC_CCIPR5.RNGSEL reset value 00).
#pragma once

#include <cstdint>

#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

class Rng {
public:
    static void init() noexcept {
        rcc::enable_hsi48();
        rcc::AHB2ENR::set(rcc::ahb2_rng);
        (void)rcc::AHB2ENR::read();
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
    using CR = PReg<0x020C'0800 + 0x0>;
    using SR = PReg<0x020C'0800 + 0x4>;
    using DR = PReg<0x020C'0800 + 0x8>;
    static constexpr std::uint32_t cr_rngen = 1u << 2;
    static constexpr std::uint32_t sr_drdy = 1u << 0;
    static constexpr std::uint32_t sr_secs = 1u << 2;
    static constexpr std::uint32_t sr_seis = 1u << 6;
};

}  // namespace rtos::port::stm32h5
