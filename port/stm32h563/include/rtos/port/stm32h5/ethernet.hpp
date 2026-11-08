// SPDX-License-Identifier: MIT
//
// STM32H563 Ethernet MAC with an RMII PHY (LAN8742A): the shared STM32 driver
// (stm32/ethernet.hpp) with the H563's clock gates and RMII selection (SBS_PMCR.ETH_SEL_PHY =
// SBS_ETH_RMII, stm32h5xx_hal.h; RCC_AHB1ENR.ETHEN/ETHTXEN/ETHRXEN, SVD v1.8). IRQ 106.
//
//   RTOS_DMA_BUFFER Ethernet<Pool, 8> eth{pool, board::mac_address(), board::phy_address};
//   eth.init(board::clocks.hclk_hz());
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32/ethernet.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

namespace eth_reg = stm32::eth_reg;
namespace lan8742 = stm32::lan8742;

namespace eth_detail {
using sbs_pmcr = PReg<0x0400'0400 + 0x100>;  // SBS base 0x44000400 + PMCR
inline constexpr std::uint32_t sbs_eth_sel_phy_msk = 7u << 21;
inline constexpr std::uint32_t sbs_eth_rmii = 4u << 21;  // SBS_ETH_RMII = ETH_SEL_PHY_2

inline void enable() noexcept {
    rcc::APB3ENR::set(rcc::apb3_sbs);
    (void)rcc::APB3ENR::read();
    sbs_pmcr::write((sbs_pmcr::read() & ~sbs_eth_sel_phy_msk) | sbs_eth_rmii);
    rcc::AHB1ENR::set(rcc::ahb1_eth | rcc::ahb1_ethtx | rcc::ahb1_ethrx);
    (void)rcc::AHB1ENR::read();
}
}  // namespace eth_detail

inline constexpr stm32::EthernetHw ethernet_hw{.base = periph_alias + 0x0002'8000, .enable = &eth_detail::enable, .irq = 106};

template <net::PacketAllocator Pool, std::size_t RxCount = 8, std::size_t TxCount = 8>
using Ethernet = stm32::Ethernet<ethernet_hw, Pool, RxCount, TxCount>;

}  // namespace rtos::port::stm32h5
