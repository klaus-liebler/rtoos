// SPDX-License-Identifier: MIT
//
// STM32H563 UCPD1 as USB Power Delivery sink PHY (RM0481 chapter UCPD; registers, bits, IRQ:
// SVD v1.8; ordered-set K-codes: ST's stm32h5xx_ll_ucpd.h). Messages move by GPDMA1: RX into
// `rx_`, TX from `tx_`, so the object must live in DMA-reachable, non-cacheable memory:
//
//   RTOS_DMA_BUFFER h5::Ucpd<{.kernel_hz = board::clocks.ucpd_hz()}> ucpd;
//   ucpd.start();                 // Rd on CC1 and CC2, attach events; then enable the IRQ
//   UCPD1_IRQHandler: ucpd.on_irq(sink);
//
// The CC pins (PB13 CC1, PB14 CC2) belong in the board's pin list as analog pins. The
// clock dividers are computed and checked at compile time from the kernel clock (HSI / 4):
// ucpd_clk 6-9 MHz (RM0481), bit rate 270-330 kbit/s (USB PD fBitRate), transition window
// 12-20 us (tTransitionWindow), inter-frame gap clock as in ST's reference setting.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/gpdma.hpp"
#include "rtos/port/stm32h5/pwr.hpp"
#include "rtos/port/stm32h5/rcc.hpp"
#include "rtos/usb_pd/sink.hpp"

namespace rtos::port::stm32h5 {

struct UcpdConfig {
    std::uint32_t kernel_hz;            ///< ClockTree::ucpd_hz()
    std::uint32_t bit_rate = 300'000;   ///< BMC bit rate, nominal 300 kbit/s
    unsigned rx_dma_channel = 0;
    unsigned tx_dma_channel = 1;
};

struct UcpdTiming {
    std::uint32_t psc = 0;         ///< CFGR1.PSC_USBPDCLK: ucpd_clk = kernel / 2^psc
    std::uint32_t ucpd_hz = 0;
    std::uint32_t hbit_div = 0;    ///< half-bit clock = ucpd_clk / hbit_div
    std::uint32_t bit_rate = 0;
    std::uint32_t transwin_div = 0;///< transition window = transwin_div half-bit periods
    std::uint32_t transwin_ns = 0;
    std::uint32_t ifrgap_div = 0;  ///< inter-frame gap clock = ucpd_clk / ifrgap_div
    bool valid = false;

    [[nodiscard]] constexpr std::uint32_t cfgr1() const noexcept {
        return (hbit_div - 1) | ((ifrgap_div - 1) << 6) | ((transwin_div - 1) << 11) | (psc << 17);
    }
};

[[nodiscard]] constexpr UcpdTiming ucpd_timing(const UcpdConfig& c) noexcept {
    UcpdTiming t;
    while (t.psc < 4 && (c.kernel_hz >> t.psc) > 9'000'000) {
        ++t.psc;
    }
    t.ucpd_hz = c.kernel_hz >> t.psc;
    if (t.ucpd_hz < 6'000'000 || t.ucpd_hz > 9'000'000 || c.bit_rate == 0) {
        return t;
    }
    t.hbit_div = (t.ucpd_hz + c.bit_rate) / (2 * c.bit_rate);  // nearest
    t.bit_rate = t.ucpd_hz / (2 * t.hbit_div);
    const std::uint32_t hbit_hz = t.ucpd_hz / t.hbit_div;
    // smallest window of at least 12 us
    t.transwin_div = static_cast<std::uint32_t>((12'000ull * hbit_hz + 999'999'999) / 1'000'000'000);
    t.transwin_ns = static_cast<std::uint32_t>(t.transwin_div * 1'000'000'000ull / hbit_hz);
    // ST's reference: ucpd_clk 8 MHz / 17
    t.ifrgap_div = static_cast<std::uint32_t>((t.ucpd_hz * 17ull + 4'000'000) / 8'000'000);
    t.valid = t.hbit_div >= 1 && t.hbit_div <= 64 && t.bit_rate >= 270'000 && t.bit_rate <= 330'000 &&
              t.transwin_div >= 2 && t.transwin_div <= 32 && t.transwin_ns <= 20'000 && t.ifrgap_div >= 2 &&
              t.ifrgap_div <= 32;
    return t;
}

namespace ucpd_detail {
inline constexpr std::uintptr_t base = 0x0000'DC00;  // UCPD1 0x4000DC00
// CFGR1
inline constexpr std::uint32_t cfgr1_rxordset_sop = 1u << 20, cfgr1_rxordset_hardrst = 1u << 23;
inline constexpr std::uint32_t cfgr1_txdmaen = 1u << 29, cfgr1_rxdmaen = 1u << 30, cfgr1_ucpden = 1u << 31;
// CR
inline constexpr std::uint32_t cr_txsend = 1u << 2, cr_txhrst = 1u << 3, cr_phyrxen = 1u << 5, cr_phyccsel = 1u << 6;
inline constexpr std::uint32_t cr_anamode_sink = 1u << 9, cr_ccenable_both = 3u << 10;
// SR / IMR / ICR (same bit positions)
inline constexpr std::uint32_t txmsgdisc = 1u << 1, txmsgsent = 1u << 2, txmsgabt = 1u << 3;
inline constexpr std::uint32_t hrstdisc = 1u << 4, hrstsent = 1u << 5;
inline constexpr std::uint32_t rxhrstdet = 1u << 10, rxmsgend = 1u << 12, rxerr = 1u << 13;
inline constexpr std::uint32_t typecevt1 = 1u << 14, typecevt2 = 1u << 15;
inline constexpr std::uint32_t communication = txmsgdisc | txmsgsent | txmsgabt | hrstdisc | hrstsent | rxhrstdet | rxmsgend;
// SOP ordered set: Sync-1, Sync-1, Sync-1, Sync-2
inline constexpr std::uint32_t sync1 = 0x18, sync2 = 0x11;
inline constexpr std::uint32_t ordered_set_sop = sync1 | (sync1 << 5) | (sync1 << 10) | (sync2 << 15);
}  // namespace ucpd_detail

template <UcpdConfig Config>
class Ucpd {
    static constexpr UcpdTiming timing = ucpd_timing(Config);
    static_assert(timing.valid, "UCPD kernel clock or bit rate out of range (ucpd_clk 6-9 MHz, 270-330 kbit/s)");
    using RxDma = DmaChannel<DmaConfig{.channel = Config.rx_dma_channel, .request = DmaRequest::Ucpd1Rx,
                                       .direction = DmaDirection::PeripheralToMemory}>;
    using TxDma = DmaChannel<DmaConfig{.channel = Config.tx_dma_channel, .request = DmaRequest::Ucpd1Tx,
                                       .direction = DmaDirection::MemoryToPeripheral}>;

public:
    static constexpr std::uint8_t irq = 76;
    static constexpr std::uint32_t cfgr1 = timing.cfgr1();

    /// Clocks, dividers, sink role (Rd on both CC lines), attach detection.
    void start() noexcept {
        using namespace ucpd_detail;
        rcc::enable(rcc::gate::ucpd1);
        RxDma::init();
        TxDma::init();
        CFGR1::write(cfgr1 | cfgr1_rxordset_sop | cfgr1_rxordset_hardrst);
        pwr::UCPDR::write(pwr::UCPDR::read() | pwr::ucpdr_dbdis);
        IMR::write(typecevt1 | typecevt2);
        CFGR1::write(CFGR1::read() | cfgr1_ucpden);
        CR::write(cr_anamode_sink | cr_ccenable_both);
        CFGR1::write(CFGR1::read() | cfgr1_rxdmaen | cfgr1_txdmaen);
    }

    /// Send `m` on the attached CC line; false if none or the transmitter is busy.
    bool transmit(const usb_pd::Message& m) noexcept {
        using namespace ucpd_detail;
        const auto payload = m.payload();
        if (cc_ == 0 || TxDma::busy()) {
            return false;
        }
        std::memcpy(tx_.data(), payload.data(), payload.size());
        TxDma::start(periph_alias + ucpd_detail::base + 0x24, tx_.data(), payload.size());
        TX_ORDSETR::write(ordered_set_sop);
        TX_PAYSZR::write(static_cast<std::uint32_t>(payload.size()));
        CR::write(CR::read() | cr_txsend);
        return true;
    }

    bool transmit_hard_reset() noexcept {
        if (cc_ == 0) {
            return false;
        }
        CR::write(CR::read() | ucpd_detail::cr_txhrst);
        return true;
    }

    void on_irq(usb_pd::PhyEvents& events) noexcept {
        using namespace ucpd_detail;
        const std::uint32_t sr = SR::read();
        if ((sr & (typecevt1 | typecevt2)) != 0) {
            ICR::write(typecevt1 | typecevt2);
            const std::uint32_t now = SR::read();
            const std::uint8_t cc = ((now >> 16) & 3u) != 0 ? 1 : (((now >> 18) & 3u) != 0 ? 2 : 0);
            if (cc != cc_) {
                cc_ = cc;
                if (cc != 0) {
                    enable_communication(cc);
                } else {
                    disable_communication();
                }
                events.on_attach(cc);
            }
        }
        // transmission results first: a GoodCRC received in the same interrupt answers it
        if ((sr & txmsgsent) != 0) {
            ICR::write(txmsgsent);
            TxDma::stop();
            events.on_transmitted(true);
        }
        if ((sr & (txmsgabt | txmsgdisc)) != 0) {
            ICR::write(sr & (txmsgabt | txmsgdisc));
            TxDma::stop();
            events.on_transmitted(false);
        }
        if ((sr & hrstdisc) != 0) {
            ICR::write(hrstdisc);  // line busy: the sink asks again later
        }
        if ((sr & (rxhrstdet | hrstsent)) != 0) {
            ICR::write(sr & (rxhrstdet | hrstsent));
            events.on_hard_reset();
        }
        if ((sr & rxmsgend) != 0) {
            ICR::write(rxmsgend);
            RxDma::stop();
            usb_pd::Message m;
            const std::size_t size = RX_PAYSZR::read() & 0x3FFu;
            const bool ok = (sr & rxerr) == 0 && (RX_ORDSETR::read() & 7u) == 0 /* SOP */ &&
                            size <= rx_.size() && m.assign(std::span{rx_}.first(size));
            m.cc = cc_;
            receive();  // re-arm before the sink answers
            if (ok) {
                events.on_message(m);
            } else if ((sr & rxerr) != 0) {
                events.on_rx_error();
            }
        }
    }

    [[nodiscard]] std::uint8_t cc() const noexcept { return cc_; }

private:
    void enable_communication(std::uint8_t cc) noexcept {
        using namespace ucpd_detail;
        CR::write((CR::read() & ~cr_phyccsel) | (cc == 2 ? cr_phyccsel : 0u));
        IMR::write(IMR::read() | communication);
        receive();
        CR::write(CR::read() | cr_phyrxen);
    }

    void disable_communication() noexcept {
        using namespace ucpd_detail;
        RxDma::stop();
        CR::write(CR::read() & ~cr_phyrxen);
        TxDma::stop();
        IMR::write(IMR::read() & ~communication);
    }

    void receive() noexcept {
        RxDma::stop();
        RxDma::start(periph_alias + ucpd_detail::base + 0x30, rx_.data(), rx_.size());
    }

    using CFGR1 = PReg<ucpd_detail::base + 0x00>;
    using CR = PReg<ucpd_detail::base + 0x0C>;
    using IMR = PReg<ucpd_detail::base + 0x10>;
    using SR = PReg<ucpd_detail::base + 0x14>;
    using ICR = PReg<ucpd_detail::base + 0x18>;
    using TX_ORDSETR = PReg<ucpd_detail::base + 0x1C>;
    using TX_PAYSZR = PReg<ucpd_detail::base + 0x20>;
    using RX_ORDSETR = PReg<ucpd_detail::base + 0x28>;
    using RX_PAYSZR = PReg<ucpd_detail::base + 0x2C>;

    alignas(4) std::array<std::byte, usb_pd::Message::max_payload> rx_{};
    alignas(4) std::array<std::byte, usb_pd::Message::max_payload> tx_{};
    std::uint8_t cc_ = 0;
};

}  // namespace rtos::port::stm32h5
