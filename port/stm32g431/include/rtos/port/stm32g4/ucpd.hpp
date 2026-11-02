// SPDX-License-Identifier: MIT
//
// STM32G431 UCPD1 as USB Power Delivery sink PHY: the shared STM32 driver (stm32/ucpd.hpp) with
// the G431's UCPD1 at 0x4000A000, IRQ 63 (RM0440 chapter UCPD, SVD v1.8, ST's stm32g431xx.h), its
// messages moved by DMA1 through the DMAMUX (requests UCPD1_RX/TX):
//
//   Ucpd<{.kernel_hz = clocks.ucpd_hz(), .rx_dma_channel = 1, .tx_dma_channel = 2}> ucpd;
//   ucpd.start();                 // Rd on CC1 and CC2, attach events; then enable the IRQ
//   UCPD1_IRQHandler: ucpd.on_irq(sink);
//
// The CC pins (PB6 CC1, PB4 CC2) belong in the board's pin list as analog pins. Kernel clock
// HSI16 (ClockTree::ucpd_hz()); dead-battery pull-downs: PWR_CR3.UCPD1_DBDIS (PWR clock on:
// configure_clocks()). The G431 has no data cache, any SRAM is DMA-coherent.
#pragma once

#include "rtos/port/stm32/ucpd.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/dma.hpp"
#include "rtos/port/stm32g4/pwr.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4 {

using stm32::ucpd_timing;
using stm32::UcpdConfig, stm32::UcpdTiming;

inline constexpr stm32::UcpdHw ucpd1_hw{.base = periph_base + 0x0000'A000,
                                       .gate = rcc::gate::ucpd1,
                                       .irq = 63,
                                       .dead_battery_reg = pwr::cr3_address,
                                       .dead_battery_bit = pwr::cr3_ucpd1_dbdis_bit};

template <UcpdConfig Config>
using Ucpd = stm32::Ucpd<ucpd1_hw,
                         DmaChannel<DmaConfig{.channel = Config.rx_dma_channel, .request = DmaRequest::Ucpd1Rx,
                                              .direction = DmaDirection::PeripheralToMemory}>,
                         DmaChannel<DmaConfig{.channel = Config.tx_dma_channel, .request = DmaRequest::Ucpd1Tx,
                                              .direction = DmaDirection::MemoryToPeripheral}>,
                         Config>;

}  // namespace rtos::port::stm32g4
