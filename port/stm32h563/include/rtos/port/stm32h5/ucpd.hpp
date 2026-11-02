// SPDX-License-Identifier: MIT
//
// STM32H563 UCPD1 as USB Power Delivery sink PHY: the shared STM32 driver (stm32/ucpd.hpp) with
// the H563's UCPD1 at 0x4000DC00, IRQ 76 (RM0481 chapter UCPD, SVD v1.8), its messages moved by
// GPDMA1 (requests UCPD1_RX/TX). The object must live in DMA-reachable, non-cacheable memory:
//
//   RTOS_DMA_BUFFER Ucpd<{.kernel_hz = board::clocks.ucpd_hz()}> ucpd;
//   ucpd.start();                 // Rd on CC1 and CC2, attach events; then enable the IRQ
//   UCPD1_IRQHandler: ucpd.on_irq(sink);
//
// The CC pins (PB13 CC1, PB14 CC2) belong in the board's pin list as analog pins. Kernel clock
// HSI / 4 (ClockTree::ucpd_hz()); dead-battery pull-downs: PWR_UCPDR.UCPD_DBDIS.
#pragma once

#include "rtos/port/stm32/ucpd.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/gpdma.hpp"
#include "rtos/port/stm32h5/pwr.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

using stm32::ucpd_timing;
using stm32::UcpdConfig, stm32::UcpdTiming;

inline constexpr stm32::UcpdHw ucpd1_hw{.base = periph_alias + 0x0000'DC00,
                                       .gate = rcc::gate::ucpd1,
                                       .irq = 76,
                                       .dead_battery_reg = periph_alias + 0x0402'0800 + 0x2C,  // PWR_UCPDR
                                       .dead_battery_bit = 0};
static_assert(pwr::ucpdr_dbdis == 1u << ucpd1_hw.dead_battery_bit);

template <UcpdConfig Config>
using Ucpd = stm32::Ucpd<ucpd1_hw,
                         DmaChannel<DmaConfig{.channel = Config.rx_dma_channel, .request = DmaRequest::Ucpd1Rx,
                                              .direction = DmaDirection::PeripheralToMemory}>,
                         DmaChannel<DmaConfig{.channel = Config.tx_dma_channel, .request = DmaRequest::Ucpd1Tx,
                                              .direction = DmaDirection::MemoryToPeripheral}>,
                         Config>;

}  // namespace rtos::port::stm32h5
