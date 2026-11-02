// SPDX-License-Identifier: MIT
//
// STM32G431 FDCAN1 in classic CAN mode: the shared STM32 driver (stm32/fdcan.hpp) with the G431's
// FDCAN1 at 0x40006400, message RAM SRAMCAN at 0x4000A400 (ST's stm32g431xx.h), IRQs 21/22.
//
//   using Can = Fdcan<{.kernel_hz = clocks.pclk1_hz(), .cpu_hz = clocks.sysclk_hz(), .bitrate = 500'000}>;
//   Can::init();                         // bit timing, accept all, start
//
// Kernel clock: RCC_CCIPR.FDCANSEL = PCLK1 (init() selects it; the reset choice is HSE, which a
// board need not have). For a kernel clock from PLL Q use stm32::Fdcan with an own FdcanHw.
#pragma once

#include "rtos/port/stm32/fdcan.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4 {

using stm32::can_bit_timing;
using stm32::CanBitTiming, stm32::CanConfig, stm32::CanFrame, stm32::CanStatus;

inline constexpr stm32::FdcanHw fdcan1_hw{.base = periph_base + 0x0000'6400,
                                         .sram = periph_base + 0x0000'A400,
                                         .gate = rcc::gate::fdcan,
                                         .kernel_clock = {rcc::kernel_clock::fdcan, rcc::kernel_clock::fdcan_pclk1},
                                         .irq0 = 21,
                                         .irq1 = 22};

template <CanConfig Config>
using Fdcan = stm32::Fdcan<fdcan1_hw, Config>;

}  // namespace rtos::port::stm32g4
