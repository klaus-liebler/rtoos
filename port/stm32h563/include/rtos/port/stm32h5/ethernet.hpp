// SPDX-License-Identifier: MIT
//
// STM32H5 Ethernet MAC (Synopsys DWC Ethernet QoS) with an RMII PHY (LAN8742A).
// Satisfies rtos::net::NetDriver.
//
// Sources: register layout ST SVD STM32H563 v1.8; descriptor bits, MDIO op codes
// and clock ranges ST stm32h5xx_hal_eth.h / stm32h563xx.h; RMII select SBS_ETH_RMII
// (stm32h5xx_hal.h); PHY registers ST stm32-lan8742 lan8742.h.
//
// Zero copy: every RX descriptor owns a pool buffer the DMA writes into; a received
// frame is handed up in that buffer and a fresh one is armed. TX descriptors point
// at the caller's buffer and keep a reference until the DMA is done.
//
// Threading: transmit()/receive()/poll_link() run in the network thread. The ETH
// interrupt only calls irq() (acknowledge + report) and wakes that thread.
//
// Placement: the descriptor rings are members, so put the object in DMA-reachable
// memory (any SRAM; RTOS_DMA_BUFFER for the non-cacheable region).
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "rtos/net/address.hpp"
#include "rtos/net/packet.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

namespace eth_reg {
inline constexpr std::uintptr_t base = 0x0002'8000;  // + periph_alias
template <std::uintptr_t Offset>
using R = PReg<base + Offset>;
using MACCR = R<0x0000>;
using MACPFR = R<0x0008>;
using MAC1USTCR = R<0x00DC>;
using MACMDIOAR = R<0x0200>;
using MACMDIODR = R<0x0204>;
using MACA0HR = R<0x0300>;
using MACA0LR = R<0x0304>;
using MTLTXQOMR = R<0x0D00>;
using MTLRXQOMR = R<0x0D30>;
using DMAMR = R<0x1000>;
using DMASBMR = R<0x1004>;
using DMACCR = R<0x1100>;
using DMACTXCR = R<0x1104>;
using DMACRXCR = R<0x1108>;
using DMACTXDLAR = R<0x1114>;
using DMACRXDLAR = R<0x111C>;
using DMACTXDTPR = R<0x1120>;
using DMACRXDTPR = R<0x1128>;
using DMACTXRLR = R<0x112C>;
using DMACRXRLR = R<0x1130>;
using DMACIER = R<0x1134>;
using DMACSR = R<0x1160>;

inline constexpr std::uint32_t maccr_re = 1u << 0, maccr_te = 1u << 1, maccr_dm = 1u << 13, maccr_fes = 1u << 14,
                               maccr_acs = 1u << 20, maccr_cst = 1u << 21;
inline constexpr std::uint32_t macpfr_pm = 1u << 4;  // pass all multicast (mDNS)
inline constexpr std::uint32_t mdio_mb = 1u << 0;
inline constexpr std::uint32_t mdio_goc_write = 1u << 2;  // ETH_MACMDIOAR_MOC_WR
inline constexpr std::uint32_t mdio_goc_read = 3u << 2;   // ETH_MACMDIOAR_MOC_RD
/// MDIO clock divider (MACMDIOAR.CR) for HCLK, as HAL_ETH_SetMDIOClockRange selects it
/// (stm32h563xx.h ETH_MACMDIOAR_CR_DIVxx): MDC <= 2.5 MHz.
[[nodiscard]] constexpr std::uint32_t mdio_clock_range(std::uint32_t hclk_hz) noexcept {
    return hclk_hz < 35'000'000    ? 0x200u   // /16
           : hclk_hz < 60'000'000  ? 0x300u   // /26
           : hclk_hz < 100'000'000 ? 0x000u   // /42
           : hclk_hz < 150'000'000 ? 0x100u   // /62
           : hclk_hz < 250'000'000 ? 0x400u   // /102
                                   : 0x500u;  // /124
}
inline constexpr std::uint32_t mtl_tsf = 1u << 1, mtl_rsf = 1u << 5;
inline constexpr std::uint32_t dmamr_swr = 1u << 0;
inline constexpr std::uint32_t dmasbmr_aal = 1u << 12;
inline constexpr std::uint32_t dma_st = 1u << 0;  // DMACTXCR.ST / DMACRXCR.SR
inline constexpr std::uint32_t dmactxcr_osf = 1u << 4;
inline constexpr std::uint32_t dma_pbl32 = 32u << 16;
/// DMACRXCR.RBSZ, bits 14:1 (SVD): receive buffer size in bytes, a multiple of 4.
constexpr std::uint32_t dmacrxcr_rbsz(std::size_t bytes) { return (static_cast<std::uint32_t>(bytes) & 0x3FFCu) << 1; }
inline constexpr std::uint32_t dmacier_tie = 1u << 0, dmacier_rie = 1u << 6, dmacier_rbue = 1u << 7,
                               dmacier_nie = 1u << 15, dmacier_aie = 1u << 14, dmacier_fbee = 1u << 12;
inline constexpr std::uint32_t dmacsr_ti = 1u << 0, dmacsr_ri = 1u << 6, dmacsr_rbu = 1u << 7;

// Normal descriptors (stm32h5xx_hal_eth.h)
inline constexpr std::uint32_t desc_own = 0x8000'0000u;
inline constexpr std::uint32_t tdes2_ioc = 0x8000'0000u;
inline constexpr std::uint32_t tdes3_fd = 0x2000'0000u, tdes3_ld = 0x1000'0000u;
inline constexpr std::uint32_t rdes3_ioc = 0x4000'0000u, rdes3_buf1v = 0x0100'0000u;
inline constexpr std::uint32_t rdes3_wb_fd = 0x2000'0000u, rdes3_wb_ld = 0x1000'0000u, rdes3_wb_es = 0x0000'8000u,
                               rdes3_wb_pl = 0x0000'7FFFu, rdes3_wb_ctxt = 0x4000'0000u;
}  // namespace eth_reg

namespace lan8742 {  // ST stm32-lan8742/lan8742.h
inline constexpr unsigned bcr = 0x00, bsr = 0x01, physcsr = 0x1F;
inline constexpr std::uint16_t bcr_soft_reset = 0x8000, bcr_autoneg_en = 0x1000, bcr_restart_autoneg = 0x0200;
inline constexpr std::uint16_t bsr_link = 0x0004;
inline constexpr std::uint16_t physcsr_speed_mask = 0x001C, physcsr_10_fd = 0x0014, physcsr_100_hd = 0x0008,
                               physcsr_100_fd = 0x0018;
}  // namespace lan8742

template <net::PacketAllocator Pool, std::size_t RxCount = 8, std::size_t TxCount = 8>
class Ethernet {
public:
    struct alignas(16) Descriptor {
        volatile std::uint32_t d0, d1, d2, d3;
    };

    Ethernet(Pool& pool, net::MacAddress mac, unsigned phy_address = 0) noexcept
        : pool_(pool), mac_(mac), phy_(phy_address) {}

    Ethernet(const Ethernet&) = delete;
    Ethernet& operator=(const Ethernet&) = delete;

    /// Clocks, RMII, MAC/MTL/DMA, descriptor rings; starts RX/TX. The board must
    /// have configured the RMII pins (AF11) and released the PHY reset.
    /// `hclk_hz` sets the MDIO clock divider and the MAC 1 us tick.
    /// Returns false if the DMA reset does not finish (no RMII reference clock).
    bool init(std::uint32_t hclk_hz) noexcept {
        using namespace eth_reg;
        rcc::APB3ENR::set(rcc::apb3_sbs);
        (void)rcc::APB3ENR::read();
        sbs_pmcr::write((sbs_pmcr::read() & ~sbs_eth_sel_phy_msk) | sbs_eth_rmii);
        rcc::AHB1ENR::set(rcc::ahb1_eth | rcc::ahb1_ethtx | rcc::ahb1_ethrx);
        (void)rcc::AHB1ENR::read();

        DMAMR::set(dmamr_swr);
        for (std::uint32_t spin = 0; (DMAMR::read() & dmamr_swr) != 0; ++spin) {
            if (spin > 1'000'000) {
                return false;
            }
        }
        mdio_cr_ = mdio_clock_range(hclk_hz);
        MACMDIOAR::write(mdio_cr_);
        MAC1USTCR::write(hclk_hz / 1'000'000 - 1u);  // as HAL_ETH_Init

        // PHY: reset, then autonegotiation
        mdio_write(lan8742::bcr, lan8742::bcr_soft_reset);
        for (std::uint32_t spin = 0; (mdio_read(lan8742::bcr) & lan8742::bcr_soft_reset) != 0 && spin < 100'000; ++spin) {
        }
        mdio_write(lan8742::bcr, lan8742::bcr_autoneg_en | lan8742::bcr_restart_autoneg);

        // MAC: strip FCS (type frames) and pad/FCS (length frames); full duplex
        // 100 Mbit/s until the PHY reports otherwise (poll_link()).
        MACCR::write(maccr_cst | maccr_acs | maccr_dm | maccr_fes);
        MACPFR::write(macpfr_pm);
        const auto& o = mac_.octets;
        MACA0HR::write(std::to_integer<std::uint32_t>(o[4]) | (std::to_integer<std::uint32_t>(o[5]) << 8));
        MACA0LR::write(std::to_integer<std::uint32_t>(o[0]) | (std::to_integer<std::uint32_t>(o[1]) << 8) |
                       (std::to_integer<std::uint32_t>(o[2]) << 16) | (std::to_integer<std::uint32_t>(o[3]) << 24));

        // MTL: store and forward (checksum-safe, full frames only)
        MTLTXQOMR::set(mtl_tsf);
        MTLRXQOMR::set(mtl_rsf);

        // DMA
        DMASBMR::set(dmasbmr_aal);
        DMACCR::write(0);  // DSL = 0: descriptors contiguous
        DMACTXCR::write(dma_pbl32 | dmactxcr_osf);
        DMACRXCR::write(dma_pbl32 | dmacrxcr_rbsz(net::PacketBuffer::capacity));

        for (std::size_t i = 0; i < TxCount; ++i) {
            tx_[i] = {0, 0, 0, 0};
            tx_buffers_[i] = nullptr;
        }
        DMACTXDLAR::write(address(&tx_[0]));
        DMACTXRLR::write(TxCount - 1);
        DMACTXDTPR::write(address(&tx_[0]));

        for (std::size_t i = 0; i < RxCount; ++i) {
            rx_buffers_[i] = nullptr;
            arm_rx(i);
        }
        DMACRXDLAR::write(address(&rx_[0]));
        DMACRXRLR::write(RxCount - 1);
        DMACRXDTPR::write(address(&rx_[RxCount - 1]));

        DMACIER::write(dmacier_nie | dmacier_aie | dmacier_tie | dmacier_rie | dmacier_rbue | dmacier_fbee);
        MACCR::set(maccr_te | maccr_re);
        DMACTXCR::set(dma_st);
        DMACRXCR::set(dma_st);
        return true;
    }

    /// Read the PHY; adapt MAC speed/duplex. Call periodically (e.g. every 500 ms).
    bool poll_link() noexcept {
        using namespace eth_reg;
        const bool up = (mdio_read(lan8742::bsr) & lan8742::bsr_link) != 0;
        if (up && !link_) {
            const std::uint16_t speed = mdio_read(lan8742::physcsr) & lan8742::physcsr_speed_mask;
            std::uint32_t cr = MACCR::read() & ~(maccr_dm | maccr_fes);
            if (speed == lan8742::physcsr_100_fd || speed == lan8742::physcsr_10_fd) {
                cr |= maccr_dm;
            }
            if (speed == lan8742::physcsr_100_fd || speed == lan8742::physcsr_100_hd) {
                cr |= maccr_fes;
            }
            MACCR::write(cr);
        }
        link_ = up;
        return up;
    }

    // ---- NetDriver ------------------------------------------------------------------

    [[nodiscard]] net::MacAddress mac() const noexcept { return mac_; }
    [[nodiscard]] bool link_up() const noexcept { return link_; }

    bool transmit(net::PacketRef frame) noexcept {
        reclaim_tx();
        Descriptor& d = tx_[tx_next_];
        if ((d.d3 & eth_reg::desc_own) != 0 || tx_buffers_[tx_next_] != nullptr) {
            return false;  // ring full
        }
        const auto length = static_cast<std::uint32_t>(frame->size());
        d.d0 = address(frame->data());
        d.d1 = 0;
        d.d2 = eth_reg::tdes2_ioc | (length & 0x3FFFu);
        tx_buffers_[tx_next_] = frame.release();
        __asm volatile("dmb" ::: "memory");
        d.d3 = eth_reg::desc_own | eth_reg::tdes3_fd | eth_reg::tdes3_ld | (length & 0x7FFFu);
        __asm volatile("dsb" ::: "memory");
        tx_next_ = (tx_next_ + 1) % TxCount;
        eth_reg::DMACTXDTPR::write(address(&tx_[tx_next_]));
        return true;
    }

    net::PacketRef receive() noexcept {
        for (;;) {
            Descriptor& d = rx_[rx_next_];
            const std::uint32_t status = d.d3;
            if ((status & eth_reg::desc_own) != 0) {
                return {};
            }
            const std::size_t index = rx_next_;
            rx_next_ = (rx_next_ + 1) % RxCount;
            const bool complete = (status & (eth_reg::rdes3_wb_fd | eth_reg::rdes3_wb_ld)) ==
                                  (eth_reg::rdes3_wb_fd | eth_reg::rdes3_wb_ld);
            if (!complete || (status & (eth_reg::rdes3_wb_es | eth_reg::rdes3_wb_ctxt)) != 0) {
                ++rx_errors_;
                rearm_same(index);
                continue;
            }
            net::PacketRef fresh = pool_.allocate(0);
            if (!fresh) {
                ++rx_dropped_;
                rearm_same(index);  // keep the old buffer, drop the frame
                return {};
            }
            net::PacketRef frame(rx_buffers_[index]);
            frame->set_view(0, status & eth_reg::rdes3_wb_pl);
            rx_buffers_[index] = fresh.release();
            rearm_same(index);
            return frame;
        }
    }

    /// From ETH_IRQHandler: acknowledge. Returns true if the network thread should run.
    bool irq() noexcept {
        const std::uint32_t status = eth_reg::DMACSR::read();
        eth_reg::DMACSR::write(status);  // write-1-to-clear
        return (status & (eth_reg::dmacsr_ri | eth_reg::dmacsr_ti | eth_reg::dmacsr_rbu)) != 0;
    }

    [[nodiscard]] std::uint32_t rx_errors() const noexcept { return rx_errors_; }
    [[nodiscard]] std::uint32_t rx_dropped() const noexcept { return rx_dropped_; }

private:
    using sbs_pmcr = PReg<0x0400'0400 + 0x100>;                    // SBS base 0x44000400 + PMCR
    static constexpr std::uint32_t sbs_eth_sel_phy_msk = 7u << 21;
    static constexpr std::uint32_t sbs_eth_rmii = 4u << 21;        // SBS_ETH_RMII = ETH_SEL_PHY_2

    static std::uint32_t address(const volatile void* p) noexcept {
        return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(p));
    }

    void arm_rx(std::size_t i) noexcept {
        if (rx_buffers_[i] == nullptr) {
            net::PacketRef b = pool_.allocate(0);
            rx_buffers_[i] = b.release();
        }
        Descriptor& d = rx_[i];
        d.d0 = rx_buffers_[i] != nullptr ? address(rx_buffers_[i]->storage()) : 0;
        d.d1 = 0;
        d.d2 = 0;
        __asm volatile("dmb" ::: "memory");
        d.d3 = rx_buffers_[i] != nullptr ? (eth_reg::desc_own | eth_reg::rdes3_ioc | eth_reg::rdes3_buf1v) : 0;
    }

    void rearm_same(std::size_t i) noexcept {
        arm_rx(i);
        __asm volatile("dsb" ::: "memory");
        eth_reg::DMACRXDTPR::write(address(&rx_[i]));
    }

    void reclaim_tx() noexcept {
        while (tx_buffers_[tx_done_] != nullptr && (tx_[tx_done_].d3 & eth_reg::desc_own) == 0) {
            net::PacketRef done(tx_buffers_[tx_done_]);  // drops the driver's reference
            tx_buffers_[tx_done_] = nullptr;
            tx_done_ = (tx_done_ + 1) % TxCount;
        }
    }

    std::uint16_t mdio_read(unsigned reg) noexcept {
        using namespace eth_reg;
        mdio_wait();
        MACMDIOAR::write(mdio_cr_ | (phy_ << 21) | (reg << 16) | mdio_goc_read | mdio_mb);
        mdio_wait();
        return static_cast<std::uint16_t>(MACMDIODR::read());
    }

    void mdio_write(unsigned reg, std::uint16_t value) noexcept {
        using namespace eth_reg;
        mdio_wait();
        MACMDIODR::write(value);
        MACMDIOAR::write(mdio_cr_ | (phy_ << 21) | (reg << 16) | mdio_goc_write | mdio_mb);
        mdio_wait();
    }

    static void mdio_wait() noexcept {
        for (std::uint32_t spin = 0; (eth_reg::MACMDIOAR::read() & eth_reg::mdio_mb) != 0 && spin < 100'000; ++spin) {
        }
    }

    std::array<Descriptor, RxCount> rx_{};
    std::array<Descriptor, TxCount> tx_{};
    std::array<net::PacketBuffer*, RxCount> rx_buffers_{};
    std::array<net::PacketBuffer*, TxCount> tx_buffers_{};
    std::size_t rx_next_ = 0;
    std::size_t tx_next_ = 0;
    std::size_t tx_done_ = 0;
    Pool& pool_;
    net::MacAddress mac_;
    unsigned phy_;
    std::uint32_t mdio_cr_ = eth_reg::mdio_clock_range(0);
    bool link_ = false;
    std::uint32_t rx_errors_ = 0;
    std::uint32_t rx_dropped_ = 0;
};

}  // namespace rtos::port::stm32h5
