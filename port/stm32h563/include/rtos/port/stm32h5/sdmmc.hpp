// SPDX-License-Identifier: MIT
//
// STM32H563 SDMMC host controller for rtos::storage::SdCard: the shared STM32 driver
// (stm32/sdmmc.hpp) with the H563's instances (RM0481 chapter SDMMC; addresses, clock gates,
// IRQ numbers: SVD v1.8).
//
//   using Host = Sdmmc<SdmmcId::Sdmmc1, {.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz}>;
//   Host host;
//   rtos::storage::SdCard<Host> card{host};
//   card.init();
//
// Kernel clock: RCC_CCIPR4 reset selection pll1_q.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/sdmmc.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

using stm32::sdmmc_clkdiv, stm32::SdmmcConfig;

enum class SdmmcId : std::uint8_t { Sdmmc1, Sdmmc2 };

[[nodiscard]] consteval stm32::SdmmcHw sdmmc_hw(SdmmcId id) {
    return id == SdmmcId::Sdmmc1 ? stm32::SdmmcHw{periph_alias + 0x0600'8000, rcc::gate::sdmmc1, 79}
                                 : stm32::SdmmcHw{periph_alias + 0x0600'8C00, rcc::gate::sdmmc2, 102};
}

template <SdmmcId Id, SdmmcConfig Config>
using Sdmmc = stm32::Sdmmc<sdmmc_hw(Id), Config>;

}  // namespace rtos::port::stm32h5
