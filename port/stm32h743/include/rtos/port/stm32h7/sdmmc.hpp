// SPDX-License-Identifier: MIT
//
// STM32H743 SDMMC host controller for rtos::storage::SdCard: the shared STM32 driver
// (stm32/sdmmc.hpp) with the H743's instances (RM0433 chapter SDMMC; addresses, clock gates:
// SVD v2.4; IRQ numbers: ST's stm32h743xx.h).
//
//   using Host = Sdmmc<SdmmcId::Sdmmc1, {.kernel_hz = clocks.pll1_q_hz(), .cpu_hz = clocks.cpu_hz()}>;
//
// The IDMA cannot reach the DTCM; with the data cache on, the buffers must be non-cacheable
// (or cleaned/invalidated around a transfer). Kernel clock: RCC_D1CCIPR.SDMMCSEL reset
// selection pll1_q (ST's stm32h7xx_ll_rcc.h).
#pragma once

#include <cstdint>

#include "rtos/port/stm32/sdmmc.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

using stm32::sdmmc_clkdiv, stm32::SdmmcConfig;

enum class SdmmcId : std::uint8_t { Sdmmc1, Sdmmc2 };

[[nodiscard]] consteval stm32::SdmmcHw sdmmc_hw(SdmmcId id) {
    return id == SdmmcId::Sdmmc1 ? stm32::SdmmcHw{0x5200'7000, rcc::gate::sdmmc1, 49}
                                 : stm32::SdmmcHw{0x4802'2400, rcc::gate::sdmmc2, 124};
}

template <SdmmcId Id, SdmmcConfig Config>
using Sdmmc = stm32::Sdmmc<sdmmc_hw(Id), Config>;

}  // namespace rtos::port::stm32h7
