// SPDX-License-Identifier: MIT
//
// STM32H563 SPI master: the shared STM32 driver (stm32/spi.hpp) with the H563's instances
// (RM0481 chapter SPI; addresses, clock gates, IRQ numbers: SVD v1.8).
//
//   constexpr SpiConfig hx711{.kernel_hz = board::pll1_q_hz, .max_bus_hz = 1'000'000};
//   using Scale = SpiMaster<SpiId::Spi2, hx711>;
//   Scale::init();
//   Scale::transfer(tx, rx);              // full duplex, tx.size() == rx.size()
//
// Kernel clock: RCC_CCIPR3 reset selection - pll1_q for SPI1-3, PCLK2 for SPI4.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/spi.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

using stm32::spi_bus_hz, stm32::spi_mbr, stm32::SpiConfig;

enum class SpiId : std::uint8_t { Spi1, Spi2, Spi3, Spi4 };

/// FIFO: ST HAL IS_SPI_FULL_INSTANCE (SPI1-3) 16 bytes, SPI4 8 bytes.
[[nodiscard]] consteval stm32::SpiHw spi_hw(SpiId id) {
    switch (id) {
        case SpiId::Spi1: return {periph_alias + 0x0001'3000, rcc::gate::spi1, 55, 16};
        case SpiId::Spi2: return {periph_alias + 0x0000'3800, rcc::gate::spi2, 56, 16};
        case SpiId::Spi3: return {periph_alias + 0x0000'3C00, rcc::gate::spi3, 57, 16};
        case SpiId::Spi4: return {periph_alias + 0x0001'4C00, rcc::gate::spi4, 82, 8};
    }
    return {};
}

template <SpiId Id, SpiConfig Config>
using SpiMaster = stm32::SpiMaster<spi_hw(Id), Config>;

}  // namespace rtos::port::stm32h5
