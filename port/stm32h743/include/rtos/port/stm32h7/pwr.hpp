// SPDX-License-Identifier: MIT
//
// STM32H743 power control: the supply configuration and the core voltage scaling (RM0433 "PWR",
// SVD STM32H743 v2.4; sequence and encodings: ST's stm32h7xx_hal_pwr_ex.c HAL_PWREx_ConfigSupply,
// HAL_PWREx_ControlVoltageScaling and stm32h7xx_hal_pwr.h).
//
// The H743 has no SMPS: the core is supplied from the LDO. PWR_CR3 must be written once after
// power-on (SCUEN = 1 until then; the write locks it), before the clock is raised. Voltage
// scales (PWR_D3CR.VOS): 01 = VOS3 (reset), 10 = VOS2, 11 = VOS1. VOS0 (480 MHz, silicon
// revision V only) is VOS1 plus the overdrive (SYSCFG_PWRCR.ODEN) - not supported here.
#pragma once

#include <cstdint>

#include "rtos/port/stm32h7/chip.hpp"

namespace rtos::port::stm32h7::pwr {

inline constexpr std::uintptr_t base = 0x5802'4800;
using CSR1 = Reg<base + 0x004>;  // reset 0x0000'4000
using CR3 = Reg<base + 0x00C>;   // reset 0x0000'0006: LDOEN, SCUEN
using D3CR = Reg<base + 0x018>;  // reset 0x0000'4000: VOS3

inline constexpr std::uint32_t cr3_bypass = 1u << 0, cr3_ldoen = 1u << 1, cr3_scuen = 1u << 2;
inline constexpr std::uint32_t csr1_actvosrdy = 1u << 13;
inline constexpr std::uint32_t d3cr_vosrdy = 1u << 13, d3cr_vos_pos = 14, d3cr_vos_msk = 3u << d3cr_vos_pos;

enum class Scale : std::uint8_t { Vos3 = 1, Vos2 = 2, Vos1 = 3 };

/// Frequency limits per scale (STM32CubeMX db IP/RCC-STM32H7_rcc_v1_0_Modes.xml, the
/// CpuClockFreq/AXIClockFreq/APBxFreq maxima; VOS1 = the silicon revision Y maximum).
inline constexpr std::uint32_t vos1_max_cpu_hz = 400'000'000, vos1_max_hclk_hz = 200'000'000,
                               vos1_max_pclk_hz = 100'000'000;

/// Supply from the LDO (the H743 has no SMPS); waits until the voltage levels are ready. A
/// supply configuration that is already locked (written before since power-on) stays.
inline void use_ldo() noexcept {
    if ((CR3::read() & cr3_scuen) != 0) {
        CR3::write((CR3::read() & ~(cr3_bypass | cr3_ldoen | cr3_scuen)) | cr3_ldoen);
    }
    while ((CSR1::read() & csr1_actvosrdy) == 0) {
    }
}

/// Core voltage scale `s`; waits until it is reached. Raise it before the clock, lower it after.
inline void set_scale(Scale s) noexcept {
    D3CR::write((D3CR::read() & ~d3cr_vos_msk) | (static_cast<std::uint32_t>(s) << d3cr_vos_pos));
    while ((D3CR::read() & d3cr_vosrdy) == 0) {
    }
}

}  // namespace rtos::port::stm32h7::pwr
