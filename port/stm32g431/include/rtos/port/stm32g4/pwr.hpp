// SPDX-License-Identifier: MIT
//
// STM32G431 power control: core voltage range and boost mode (RM0440 "PWR", SVD STM32G431 v1.8;
// frequency limits: ST's stm32g4xx_ll_utils.c, UTILS_MAX_FREQUENCY_SCALE1/2).
//
// Range 1 (reset) allows SYSCLK up to 150 MHz in normal mode, up to 170 MHz in boost mode
// (PWR_CR5.R1MODE = 0); range 2 up to 26 MHz. The PWR registers need the PWR clock
// (RCC_APB1ENR1.PWREN, enable_clock()).
#pragma once

#include <cstdint>

#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4::pwr {

using CR1 = PReg<0x0000'7000 + 0x00>;  // reset 0x0000'0200: VOS = 01 (range 1)
using CR3 = PReg<0x0000'7000 + 0x08>;
using SR2 = PReg<0x0000'7000 + 0x14>;
using CR5 = PReg<0x0000'7000 + 0x80>;  // reset 0x0000'0100: R1MODE = 1 (normal)

inline constexpr std::uintptr_t cr3_address = periph_base + 0x0000'7000 + 0x08;
inline constexpr std::uint8_t cr3_ucpd1_dbdis_bit = 14;  ///< dead-battery pull-downs off, UCPD takes the CC lines
inline constexpr std::uint32_t cr1_vos_pos = 9, cr1_vos_msk = 3u << cr1_vos_pos;
inline constexpr std::uint32_t sr2_vosf = 1u << 10;
inline constexpr std::uint32_t cr5_r1mode = 1u << 8;

inline constexpr std::uint32_t max_hz_normal = 150'000'000, max_hz_boost = 170'000'000;

inline void enable_clock() noexcept { rcc::enable(rcc::gate::pwr); }

/// Range 1, boost mode on (`boost`, above 150 MHz) or off; waits until the regulator is ready.
inline void set_range1(bool boost) noexcept {
    CR1::write((CR1::read() & ~cr1_vos_msk) | (1u << cr1_vos_pos));
    while ((SR2::read() & sr2_vosf) != 0) {
    }
    CR5::write((CR5::read() & ~cr5_r1mode) | (boost ? 0u : cr5_r1mode));
}

}  // namespace rtos::port::stm32g4::pwr
