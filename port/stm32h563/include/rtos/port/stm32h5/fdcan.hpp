// SPDX-License-Identifier: MIT
//
// STM32H563 FDCAN1 in classic CAN mode: the shared STM32 driver (stm32/fdcan.hpp) with the H563's
// FDCAN1 at 0x4000A400, message RAM SRAMCAN at 0x4000AC00 (FDCAN1 first), IRQs 39/40 (RM0481
// chapter FDCAN, SVD v1.8).
//
//   using Can = Fdcan<{.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz, .bitrate = 500'000}>;
//   Can::init();                         // bit timing, accept all, start
//
// Kernel clock: RCC_CCIPR5.FDCAN12SEL = pll1_q (init() selects it; the reset choice HSE does
// not exist on the FCU).
#pragma once

#include "rtos/port/stm32/fdcan.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

using stm32::can_bit_timing;
using stm32::CanBitTiming, stm32::CanConfig, stm32::CanFrame, stm32::CanStatus;

inline constexpr stm32::FdcanHw fdcan1_hw{.base = periph_alias + 0x0000'A400,
                                         .sram = periph_alias + 0x0000'AC00,
                                         .gate = rcc::gate::fdcan12,
                                         .kernel_clock = {rcc::kernel_clock::fdcan12, rcc::kernel_clock::fdcan_pll1_q},
                                         .irq0 = 39,
                                         .irq1 = 40};

template <CanConfig Config>
using Fdcan = stm32::Fdcan<fdcan1_hw, Config>;

}  // namespace rtos::port::stm32h5
