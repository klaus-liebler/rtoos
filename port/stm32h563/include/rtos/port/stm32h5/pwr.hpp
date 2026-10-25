// SPDX-License-Identifier: MIT
//
// STM32H563 power control: core voltage scaling (RM0481 "PWR", SVD STM32H563 v1.8).
#pragma once

#include <cstdint>

#include "rtos/port/stm32h5/chip.hpp"

namespace rtos::port::stm32h5::pwr {

using VOSCR = PReg<0x0402'0800 + 0x10>;
using VOSSR = PReg<0x0402'0800 + 0x14>;  // reset 0x0000'0008

inline constexpr std::uint32_t voscr_vos_pos = 4;
inline constexpr std::uint32_t voscr_vos_msk = 3u << voscr_vos_pos;
inline constexpr std::uint32_t vossr_vosrdy = 1u << 3;

/// VOS0 allows HCLK up to 250 MHz, VOS3 (reset) up to 100 MHz.
enum class VoltageScale : std::uint32_t { Vos3 = 0, Vos2 = 1, Vos1 = 2, Vos0 = 3 };

/// Select the regulator voltage scale and wait until the output is ready.
inline void set_voltage_scale(VoltageScale vos) noexcept {
    VOSCR::write((VOSCR::read() & ~voscr_vos_msk) | (static_cast<std::uint32_t>(vos) << voscr_vos_pos));
    while ((VOSSR::read() & vossr_vosrdy) == 0) {
    }
}

}  // namespace rtos::port::stm32h5::pwr
