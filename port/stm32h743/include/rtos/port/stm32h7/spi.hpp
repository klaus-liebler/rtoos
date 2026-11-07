// SPDX-License-Identifier: MIT
//
// STM32H743 SPI master: the shared STM32 driver (stm32/spi.hpp) with the H743's instances
// (RM0433 chapter SPI; addresses, clock gates: SVD v2.4; IRQ numbers: ST's stm32h743xx.h).
//
//   constexpr SpiConfig hx711{.kernel_hz = clocks.pll1_q_hz(), .max_bus_hz = 1'000'000,
//                             .cpu_hz = clocks.cpu_hz()};
//   using Scale = SpiMaster<SpiId::Spi2, hx711>;
//
// Kernel clock: the reset selection (ST's stm32h7xx_ll_rcc.h) - pll1_q for SPI1-3
// (RCC_D2CCIP1R.SPI123SEL), PCLK2 for SPI4/5 (SPI45SEL), PCLK4 for SPI6 (RCC_D3CCIPR.SPI6SEL).
#pragma once

#include <cstdint>

#include "rtos/port/stm32/spi.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

using stm32::spi_bus_hz, stm32::spi_mbr, stm32::SpiConfig;

enum class SpiId : std::uint8_t { Spi1, Spi2, Spi3, Spi4, Spi5, Spi6 };

/// FIFO: stm32h743xx.h IS_SPI_HIGHEND_INSTANCE (SPI1-3) 16 bytes, the others 8
/// (stm32h7xx_hal_spi.h SPI_HIGHEND/LOWEND_FIFO_SIZE).
[[nodiscard]] consteval stm32::SpiHw spi_hw(SpiId id) {
    switch (id) {
        case SpiId::Spi1: return {0x4001'3000, rcc::gate::spi1, 35, 16};
        case SpiId::Spi2: return {0x4000'3800, rcc::gate::spi2, 36, 16};
        case SpiId::Spi3: return {0x4000'3C00, rcc::gate::spi3, 51, 16};
        case SpiId::Spi4: return {0x4001'3400, rcc::gate::spi4, 84, 8};
        case SpiId::Spi5: return {0x4001'5000, rcc::gate::spi5, 85, 8};
        case SpiId::Spi6: return {0x5800'1400, rcc::gate::spi6, 86, 8};
    }
    return {};
}

template <SpiId Id, SpiConfig Config>
using SpiMaster = stm32::SpiMaster<spi_hw(Id), Config>;

}  // namespace rtos::port::stm32h7
