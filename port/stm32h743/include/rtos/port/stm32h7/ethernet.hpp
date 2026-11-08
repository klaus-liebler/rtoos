// SPDX-License-Identifier: MIT
//
// STM32H743 Ethernet MAC with an RMII PHY (LAN8742A): the shared STM32 driver
// (stm32/ethernet.hpp; the H7's MAC/MTL/DMA registers match the H5's, at the same address)
// with the H743's clock gates (RCC_AHB1ENR.ETH1MACEN/ETH1TXEN/ETH1RXEN, SVD v2.4) and RMII
// selection (SYSCFG_PMCR.EPIS = SYSCFG_ETH_RMII, stm32h7xx_hal.h; needs the SYSCFG clock).
// IRQ 61 (stm32h743xx.h ETH_IRQn).
//
//   RTOS_DMA_BUFFER Ethernet<Pool, 8> eth{pool, board::mac_address(), board::phy_address};
//   eth.init(clocks.hclk_hz());
//
// The DMA reads and writes the descriptors (members of the object) and the packet buffers:
// both in .dma_buffer (RTOS_DMA_BUFFER, non-cacheable SRAM1/2; the DTCM is out of its reach).
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32/ethernet.hpp"
#include "rtos/port/stm32/rcc.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

namespace eth_reg = stm32::eth_reg;
namespace lan8742 = stm32::lan8742;

namespace eth_detail {
using SYSCFG_PMCR = Reg<0x5800'0400 + 0x04>;
inline constexpr std::uint32_t pmcr_epis_msk = 7u << 21;
inline constexpr std::uint32_t pmcr_epis_rmii = 4u << 21;  // SYSCFG_PMCR_EPIS_SEL_2

inline void enable() noexcept {
    stm32::rcc::enable(rcc::gate::syscfg);
    SYSCFG_PMCR::write((SYSCFG_PMCR::read() & ~pmcr_epis_msk) | pmcr_epis_rmii);
    stm32::rcc::enable(rcc::gate::eth1mac);
    stm32::rcc::enable(rcc::gate::eth1tx);
    stm32::rcc::enable(rcc::gate::eth1rx);
}
}  // namespace eth_detail

inline constexpr stm32::EthernetHw ethernet_hw{.base = 0x4002'8000, .enable = &eth_detail::enable, .irq = 61};

template <net::PacketAllocator Pool, std::size_t RxCount = 8, std::size_t TxCount = 8>
using Ethernet = stm32::Ethernet<ethernet_hw, Pool, RxCount, TxCount>;

}  // namespace rtos::port::stm32h7
