// SPDX-License-Identifier: MIT
//
// STM32 USB OTG core (Synopsys DWC2) in device mode, full speed with the embedded PHY, slave mode
// (the CPU moves the FIFO data, no DMA): the rtos::usb::DeviceController for the USB stack. The
// STM32H743's USB2_OTG_FS (stm32h7/usb_device.hpp); registers, bits, reset values: ST's
// stm32h743xx.h (USB_OTG_*), the sequences after ST's stm32h7xx_ll_usb.c / stm32h7xx_hal_pcd.c:
//
//   core init   GUSBCFG.PHYSEL, core reset (GRSTCTL.CSRST after AHBIDL), GCCFG.PWRDWN (PHY on),
//               GUSBCFG.FDMOD (device mode, CMOD = 0 within ~25 ms), no VBUS sensing (the board
//               watches VBUS itself: GCCFG.VBDEN off, GOTGCTL.BVALOEN + BVALOVAL), DCFG.DSPD = 3
//               (full speed, embedded PHY), FIFO sizes, flush, masks, GINTMSK, GAHBCFG.GINT
//   bus reset   flush TX FIFOs, endpoint interrupts cleared, STALL off, DAINTMSK EP0, DOEPMSK /
//               DIEPMSK, DCFG.DAD = 0, EP0 OUT prepared for SETUP (DOEPTSIZ0: STUPCNT 3,
//               PKTCNT 1, 24 bytes - in slave mode without EPENA: the core always takes SETUPs)
//   enum done   DIEPCTL0.MPSIZ = 0 (64 bytes), DCTL.CGINAK
//   receive     GINTSTS.RXFLVL: pop GRXSTSP (EPNUM 3:0, BCNT 14:4, PKTSTS 20:17); PKTSTS 2 = OUT
//               data (BCNT bytes follow in the RX FIFO), 6 = SETUP data (8 bytes), 3/4 = the
//               transfer / SETUP stage done; then DOEPINTn.XFRC / STUP
//   transmit    DIEPTSIZn (PKTCNT 1, XFRSIZ), DIEPCTLn.CNAK + EPENA, the packet into TX FIFO n;
//               DIEPINTn.XFRC when the host took it
//
// One packet at a time per endpoint, as the USB_DRD_FS driver (usb_device.hpp): each IN packet
// goes out from the transfer buffer, each OUT packet is read straight into it; the interrupt
// handler queues an event (SPSC ring) for every SETUP, completed transfer, bus reset, suspend and
// resume, and the stack's thread takes them with poll().
//
// SET_ADDRESS: the OTG core wants the new address in DCFG.DAD before the status stage (RM0433,
// device programming model) - the interrupt handler sets it as soon as it sees the request; the
// stack's later set_address() writes the same value again.
//
// `Regs` (stm32h7::UsbOtgHardware, a simulated core in the host tests): irq, trdt (GUSBCFG.TRDT
// for the AHB clock), tx_fifo_words, Masked, read/write of a register offset, rx_pop() (a word of
// the RX FIFO), tx_push(n, word) (into TX FIFO n), delay_ms().
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "rtos/port/stm32/usb_device.hpp"  // UsbDeviceStats
#include "rtos/sync/spsc_ring.hpp"
#include "rtos/usb/usb.hpp"

namespace rtos::port::stm32 {

namespace otg {
// global registers (USB_OTG_GlobalTypeDef)
inline constexpr std::uint32_t gotgctl = 0x000, gahbcfg = 0x008, gusbcfg = 0x00C, grstctl = 0x010, gintsts = 0x014,
                               gintmsk = 0x018, grxstsp = 0x020, grxfsiz = 0x024, dieptxf0 = 0x028, gccfg = 0x038;
[[nodiscard]] constexpr std::uint32_t dieptxf(unsigned n) noexcept { return 0x104 + 4 * (n - 1); }  // n >= 1
// device registers (USB_OTG_DeviceTypeDef at 0x800), endpoints at 0x900 / 0xB00 + 0x20 n
inline constexpr std::uint32_t dcfg = 0x800, dctl = 0x804, diepmsk = 0x810, doepmsk = 0x814, daint = 0x818,
                               daintmsk = 0x81C, pcgcctl = 0xE00;
[[nodiscard]] constexpr std::uint32_t diepctl(unsigned n) noexcept { return 0x900 + 0x20 * n; }
[[nodiscard]] constexpr std::uint32_t diepint(unsigned n) noexcept { return 0x908 + 0x20 * n; }
[[nodiscard]] constexpr std::uint32_t dieptsiz(unsigned n) noexcept { return 0x910 + 0x20 * n; }
[[nodiscard]] constexpr std::uint32_t doepctl(unsigned n) noexcept { return 0xB00 + 0x20 * n; }
[[nodiscard]] constexpr std::uint32_t doepint(unsigned n) noexcept { return 0xB08 + 0x20 * n; }
[[nodiscard]] constexpr std::uint32_t doeptsiz(unsigned n) noexcept { return 0xB10 + 0x20 * n; }

// bits (stm32h743xx.h USB_OTG_*_Pos)
inline constexpr std::uint32_t gotgctl_bvaloen = 1u << 6, gotgctl_bvaloval = 1u << 7;
inline constexpr std::uint32_t gahbcfg_gint = 1u << 0;
inline constexpr std::uint32_t gusbcfg_physel = 1u << 6, gusbcfg_trdt_pos = 10, gusbcfg_trdt_msk = 0xFu << 10,
                               gusbcfg_fhmod = 1u << 29, gusbcfg_fdmod = 1u << 30;
inline constexpr std::uint32_t grstctl_csrst = 1u << 0, grstctl_rxfflsh = 1u << 4, grstctl_txfflsh = 1u << 5,
                               grstctl_txfnum_pos = 6, grstctl_ahbidl = 1u << 31;
inline constexpr std::uint32_t gint_cmod = 1u << 0, gint_rxflvl = 1u << 4, gint_usbsusp = 1u << 11,
                               gint_usbrst = 1u << 12, gint_enumdne = 1u << 13, gint_iepint = 1u << 18,
                               gint_oepint = 1u << 19, gint_wkuint = 1u << 31;
inline constexpr std::uint32_t gccfg_pwrdwn = 1u << 16, gccfg_vbden = 1u << 21;
inline constexpr std::uint32_t dcfg_dspd_full = 3u, dcfg_dad_pos = 4, dcfg_dad_msk = 0x7Fu << 4;
inline constexpr std::uint32_t dctl_sdis = 1u << 1, dctl_cginak = 1u << 8;
inline constexpr std::uint32_t pcgcctl_stopclk = 1u << 0, pcgcctl_gateclk = 1u << 1;
// DIEPCTLn / DOEPCTLn
inline constexpr std::uint32_t ctl_mpsiz_msk = 0x7FFu, ctl_usbaep = 1u << 15, ctl_eptyp_pos = 18,
                               ctl_stall = 1u << 21, ctl_txfnum_pos = 22, ctl_cnak = 1u << 26, ctl_snak = 1u << 27,
                               ctl_sd0pid = 1u << 28, ctl_epdis = 1u << 30, ctl_epena = 1u << 31;
// DIEPTSIZn / DOEPTSIZn
inline constexpr std::uint32_t tsiz_pktcnt_1 = 1u << 19, tsiz_stupcnt_3 = 3u << 29;
// DIEPINTn / DOEPINTn, the masks
inline constexpr std::uint32_t epint_xfrc = 1u << 0, epint_epdisd = 1u << 1, epint_toc_stup = 1u << 3,
                               doepint_otepspr = 1u << 5, ep_int_clear_all = 0xFB7Fu;
// GRXSTSP
inline constexpr std::uint32_t pktsts_out_data = 2, pktsts_setup_data = 6;

inline constexpr std::size_t endpoints = 8;           ///< endpoint numbers 0..7 (the FS core has 9)
inline constexpr std::uint32_t rx_fifo_words = 128;  ///< 512 bytes: SETUPs + several OUT packets
}  // namespace otg

/// The device controller (see above).
template <class Regs>
class UsbOtgDevice {
public:
    static constexpr std::uint8_t irq = Regs::irq;

    explicit UsbOtgDevice(Regs regs = {}) noexcept : regs_(regs) {}
    UsbOtgDevice(const UsbOtgDevice&) = delete;
    UsbOtgDevice& operator=(const UsbOtgDevice&) = delete;

    /// Core reset, device mode, FIFOs, interrupts (soft-disconnected: connect()). After the
    /// clocks (Regs::enable_clocks()); takes up to ~100 ms (the core's mode switch).
    void start() noexcept {
        using namespace otg;
        write(gahbcfg, read(gahbcfg) & ~gahbcfg_gint);
        write(gusbcfg, read(gusbcfg) | gusbcfg_physel);
        core_reset();
        write(gccfg, read(gccfg) | gccfg_pwrdwn);  // the embedded PHY on
        write(gusbcfg, (read(gusbcfg) & ~(gusbcfg_fhmod | gusbcfg_fdmod | gusbcfg_trdt_msk)) | gusbcfg_fdmod |
                           (Regs::trdt << gusbcfg_trdt_pos));
        for (int ms = 0; ms < 200 && (read(gintsts) & gint_cmod) != 0; ms += 10) {
            regs_.delay_ms(10);
        }
        for (unsigned n = 1; n < 16; ++n) {
            write(dieptxf(n), 0);
        }
        write(dctl, read(dctl) | dctl_sdis);
        write(gccfg, read(gccfg) & ~gccfg_vbden);
        write(gotgctl, read(gotgctl) | gotgctl_bvaloen | gotgctl_bvaloval);
        write(pcgcctl, 0);
        write(dcfg, (read(dcfg) & ~3u) | dcfg_dspd_full);
        // FIFO RAM: RX first, then one TX FIFO per IN endpoint (words)
        write(grxfsiz, rx_fifo_words);
        std::uint32_t at = rx_fifo_words;
        write(dieptxf0, (Regs::tx_fifo_words << 16) | at);
        for (unsigned n = 1; n < endpoints; ++n) {
            at += Regs::tx_fifo_words;
            write(dieptxf(n), (Regs::tx_fifo_words << 16) | at);
        }
        flush_tx(0x10);
        flush_rx();
        write(diepmsk, 0);
        write(doepmsk, 0);
        write(daintmsk, 0);
        for (unsigned n = 0; n < endpoints; ++n) {
            write(diepctl(n), (read(diepctl(n)) & ctl_epena) != 0 ? (n == 0 ? ctl_snak : ctl_epdis | ctl_snak) : 0u);
            write(dieptsiz(n), 0);
            write(diepint(n), ep_int_clear_all);
            write(doepctl(n), (read(doepctl(n)) & ctl_epena) != 0 ? (n == 0 ? ctl_snak : ctl_epdis | ctl_snak) : 0u);
            write(doeptsiz(n), 0);
            write(doepint(n), ep_int_clear_all);
        }
        write(gintmsk, 0);
        write(gintsts, 0xBFFF'FFFFu);
        write(gintmsk, gint_rxflvl | gint_usbsusp | gint_usbrst | gint_enumdne | gint_iepint | gint_oepint | gint_wkuint);
        write(gahbcfg, read(gahbcfg) | gahbcfg_gint);
        reset_state();
    }

    // ---- rtos::usb::DeviceController -----------------------------------------------------

    std::optional<usb::Event> poll() noexcept { return events_.pop(); }

    bool open(const usb::EndpointConfig& c) noexcept {
        using namespace otg;
        const unsigned n = usb::endpoint_number(c.address);
        if (n == 0 || n >= endpoints || c.max_packet == 0 || c.max_packet > 64 ||
            c.max_packet > 4 * Regs::tx_fifo_words) {
            return false;
        }
        const std::uint32_t ctl = c.max_packet | (static_cast<std::uint32_t>(c.type) << ctl_eptyp_pos) | ctl_sd0pid |
                                  ctl_usbaep | ctl_snak;
        [[maybe_unused]] const Masked masked;
        if (usb::is_in(c.address)) {
            in_[n] = {.mps = c.max_packet};
            write(diepctl(n), ctl | (n << ctl_txfnum_pos));
            write(daintmsk, read(daintmsk) | (1u << n));
        } else {
            out_[n] = {.mps = c.max_packet};
            write(doepctl(n), ctl);
            write(daintmsk, read(daintmsk) | (1u << (16 + n)));
        }
        return true;
    }

    void close_all() noexcept {
        using namespace otg;
        [[maybe_unused]] const Masked masked;
        for (unsigned n = 1; n < endpoints; ++n) {
            write(diepctl(n), (read(diepctl(n)) & ctl_epena) != 0 ? ctl_epdis | ctl_snak : 0u);
            write(doepctl(n), (read(doepctl(n)) & ctl_epena) != 0 ? ctl_epdis | ctl_snak : 0u);
            write(diepint(n), ep_int_clear_all);
            write(doepint(n), ep_int_clear_all);
            in_[n] = {};
            out_[n] = {};
        }
        write(daintmsk, read(daintmsk) & 0x0001'0001u);
        flush_tx(0x10);
    }

    bool transfer(std::uint8_t ep, std::span<std::byte> buffer, bool zlp) noexcept {
        const unsigned n = usb::endpoint_number(ep);
        if (n >= otg::endpoints) {
            return false;
        }
        Direction& d = usb::is_in(ep) ? in_[n] : out_[n];
        if (d.mps == 0 || d.busy) {
            return false;
        }
        d.buffer = buffer.data();
        d.size = static_cast<std::uint16_t>(buffer.size());
        d.done = 0;
        d.busy = true;
        [[maybe_unused]] const Masked masked;  // the ISR works on the same endpoint
        if (usb::is_in(ep)) {
            d.zlp = buffer.empty() || (zlp && buffer.size() % d.mps == 0);
            send_packet(n);
        } else {
            arm_out(n);
        }
        return true;
    }

    void stall(std::uint8_t ep) noexcept {
        using namespace otg;
        const unsigned n = usb::endpoint_number(ep);
        [[maybe_unused]] const Masked masked;
        if (n == 0) {  // protocol stall: both directions until the next SETUP (the core clears it)
            write(diepctl(0), read(diepctl(0)) | ctl_stall);
            write(doepctl(0), read(doepctl(0)) | ctl_stall);
            in_[0].busy = out_[0].busy = false;
            return;
        }
        const std::uint32_t reg = usb::is_in(ep) ? diepctl(n) : doepctl(n);
        std::uint32_t v = read(reg);
        if ((v & ctl_epena) == 0) {
            v &= ~ctl_epdis;
        }
        write(reg, v | ctl_stall);
        (usb::is_in(ep) ? in_[n] : out_[n]).busy = false;
    }

    void clear_stall(std::uint8_t ep) noexcept {
        using namespace otg;
        const unsigned n = usb::endpoint_number(ep);
        const std::uint32_t reg = usb::is_in(ep) ? diepctl(n) : doepctl(n);
        [[maybe_unused]] const Masked masked;
        write(reg, (read(reg) & ~ctl_stall) | (n != 0 ? ctl_sd0pid : 0u));  // DATA0 again
    }

    void set_address(std::uint8_t address) noexcept {
        using namespace otg;
        [[maybe_unused]] const Masked masked;
        write(dcfg, (read(dcfg) & ~dcfg_dad_msk) | (std::uint32_t{address} << dcfg_dad_pos));
    }

    void connect() noexcept {
        using namespace otg;
        write(pcgcctl, read(pcgcctl) & ~(pcgcctl_stopclk | pcgcctl_gateclk));
        write(dctl, read(dctl) & ~dctl_sdis);
    }
    void disconnect() noexcept { write(otg::dctl, read(otg::dctl) | otg::dctl_sdis); }

    // ---- interrupt ---------------------------------------------------------------------

    void on_irq() noexcept {
        using namespace otg;
        std::uint32_t pending = read(gintsts) & read(gintmsk);
        if ((pending & gint_usbrst) != 0) {
            write(gintsts, gint_usbrst);
            bus_reset();
            push({.type = usb::EventType::BusReset});
        }
        if ((pending & gint_enumdne) != 0) {
            write(gintsts, gint_enumdne);
            write(diepctl(0), read(diepctl(0)) & ~ctl_mpsiz_msk);  // MPSIZ 0: 64 bytes
            write(dctl, read(dctl) | dctl_cginak);
        }
        while ((read(gintsts) & gint_rxflvl) != 0) {
            receive(read(grxstsp));
        }
        pending = read(gintsts) & read(gintmsk);  // the RX FIFO entries raise the endpoint interrupts
        if ((pending & gint_oepint) != 0) {
            const std::uint32_t bits = read(daint) & read(daintmsk);
            for (unsigned n = 0; n < endpoints; ++n) {
                if ((bits & (1u << (16 + n))) != 0) {
                    const std::uint32_t e = read(doepint(n));
                    write(doepint(n), e);  // rc_w1: everything seen
                    if ((e & epint_xfrc) != 0) {
                        out_complete(n);
                    }
                    if ((e & epint_toc_stup) != 0 && n == 0) {
                        on_setup();
                    }
                }
            }
        }
        if ((pending & gint_iepint) != 0) {
            const std::uint32_t bits = read(daint) & read(daintmsk);
            for (unsigned n = 0; n < endpoints; ++n) {
                if ((bits & (1u << n)) != 0) {
                    const std::uint32_t e = read(diepint(n));
                    write(diepint(n), e);
                    if ((e & epint_xfrc) != 0) {
                        on_in(n);
                    }
                }
            }
        }
        if ((pending & gint_usbsusp) != 0) {
            write(gintsts, gint_usbsusp);
            push({.type = usb::EventType::Suspend});
        }
        if ((pending & gint_wkuint) != 0) {
            write(gintsts, gint_wkuint);
            push({.type = usb::EventType::Resume});
        }
    }

    [[nodiscard]] const UsbDeviceStats& stats() const noexcept { return stats_; }
    [[nodiscard]] Regs& registers() noexcept { return regs_; }

private:
    struct Direction {
        std::byte* buffer = nullptr;
        std::uint16_t size = 0;
        std::uint16_t done = 0;
        std::uint16_t mps = 0;        ///< 0: closed
        std::uint16_t in_flight = 0;  ///< IN: bytes of the packet being sent
        std::uint16_t last = 0;       ///< OUT: bytes of the last packet received
        bool busy = false;
        bool zlp = false;  ///< IN: a zero-length packet still to send at the end
    };

    using Masked = typename Regs::Masked;
    static constexpr std::uint16_t ep0_packet = 64;

    std::uint32_t read(std::uint32_t offset) noexcept { return regs_.read(offset); }
    void write(std::uint32_t offset, std::uint32_t value) noexcept { regs_.write(offset, value); }

    /// Waits for a GRSTCTL bit (bounded: a dead core must not hang the firmware).
    void wait_grstctl(std::uint32_t bit, bool set) noexcept {
        for (int i = 0; i < 200'000 && ((read(otg::grstctl) & bit) != 0) != set; ++i) {
        }
    }
    void core_reset() noexcept {
        wait_grstctl(otg::grstctl_ahbidl, true);
        write(otg::grstctl, otg::grstctl_csrst);
        wait_grstctl(otg::grstctl_csrst, false);
    }
    void flush_tx(std::uint32_t fifo) noexcept {
        wait_grstctl(otg::grstctl_ahbidl, true);
        write(otg::grstctl, otg::grstctl_txfflsh | (fifo << otg::grstctl_txfnum_pos));
        wait_grstctl(otg::grstctl_txfflsh, false);
    }
    void flush_rx() noexcept {
        wait_grstctl(otg::grstctl_ahbidl, true);
        write(otg::grstctl, otg::grstctl_rxfflsh);
        wait_grstctl(otg::grstctl_rxfflsh, false);
    }

    void reset_state() noexcept {
        for (unsigned n = 1; n < otg::endpoints; ++n) {
            in_[n] = {};
            out_[n] = {};
        }
        in_[0] = {.mps = ep0_packet};
        out_[0] = {.mps = ep0_packet};
    }

    /// EP0 OUT ready for the next SETUP (slave mode: transfer size only, no EPENA).
    void ep0_out_start() noexcept {
        write(otg::doeptsiz(0), otg::tsiz_stupcnt_3 | otg::tsiz_pktcnt_1 | (3 * 8));
    }

    void bus_reset() noexcept {
        using namespace otg;
        flush_tx(0x10);
        for (unsigned n = 0; n < endpoints; ++n) {
            write(diepint(n), ep_int_clear_all);
            write(diepctl(n), read(diepctl(n)) & ~ctl_stall);
            write(doepint(n), ep_int_clear_all);
            write(doepctl(n), (read(doepctl(n)) & ~ctl_stall) | ctl_snak);
        }
        write(daintmsk, 0x0001'0001u);
        write(doepmsk, epint_xfrc | epint_epdisd | epint_toc_stup | doepint_otepspr);
        write(diepmsk, epint_xfrc | epint_epdisd | epint_toc_stup);
        write(dcfg, read(dcfg) & ~dcfg_dad_msk);
        reset_state();
        ep0_out_start();
    }

    // ---- transfers -------------------------------------------------------------------------

    void send_packet(unsigned n) noexcept {
        using namespace otg;
        Direction& d = in_[n];
        const std::uint16_t left = static_cast<std::uint16_t>(d.size - d.done);
        const std::uint16_t chunk = left < d.mps ? left : d.mps;
        if (chunk == 0) {
            d.zlp = false;  // this is the zero-length packet
        }
        d.in_flight = chunk;
        write(dieptsiz(n), tsiz_pktcnt_1 | chunk);
        write(diepctl(n), read(diepctl(n)) | ctl_cnak | ctl_epena);
        const std::byte* p = d.buffer + d.done;
        for (std::size_t i = 0; i < chunk; i += 4) {
            std::uint32_t w = 0;
            for (std::size_t b = 0; b < 4 && i + b < chunk; ++b) {
                w |= std::to_integer<std::uint32_t>(p[i + b]) << (8 * b);
            }
            regs_.tx_push(n, w);
        }
    }

    void arm_out(unsigned n) noexcept {
        using namespace otg;
        Direction& d = out_[n];
        write(doeptsiz(n), (n == 0 ? tsiz_stupcnt_3 : 0u) | tsiz_pktcnt_1 | d.mps);
        write(doepctl(n), read(doepctl(n)) | ctl_cnak | ctl_epena);
    }

    void on_in(unsigned n) noexcept {
        Direction& d = in_[n];
        if (!d.busy) {
            return;
        }
        d.done = static_cast<std::uint16_t>(d.done + d.in_flight);
        if (d.done < d.size || d.zlp) {
            send_packet(n);
            return;
        }
        d.busy = false;
        push({.type = usb::EventType::TransferComplete, .endpoint = static_cast<std::uint8_t>(0x80 | n), .length = d.done});
    }

    /// One GRXSTSP entry: OUT or SETUP data out of the RX FIFO.
    void receive(std::uint32_t status) noexcept {
        const unsigned n = status & 0xFu;
        const std::uint16_t bytes = static_cast<std::uint16_t>((status >> 4) & 0x7FFu);
        const std::uint32_t pktsts = (status >> 17) & 0xFu;
        const std::size_t words = (bytes + 3u) / 4u;
        if (pktsts == otg::pktsts_setup_data) {
            for (std::size_t i = 0; i < words; ++i) {
                const std::uint32_t w = regs_.rx_pop();
                for (std::size_t b = 0; b < 4 && 4 * i + b < setup_.size(); ++b) {
                    setup_[4 * i + b] = static_cast<std::byte>(w >> (8 * b));
                }
            }
            return;
        }
        if (pktsts != otg::pktsts_out_data) {
            return;  // transfer / SETUP stage done, global OUT NAK: no data
        }
        Direction& d = out_[n < otg::endpoints ? n : 0];
        const bool keep = n < otg::endpoints && d.busy;
        d.last = bytes;
        for (std::size_t i = 0; i < words; ++i) {
            const std::uint32_t w = regs_.rx_pop();
            if (!keep) {
                continue;  // nobody asked: dropped (the endpoint was not armed - cannot happen)
            }
            for (std::size_t b = 0; b < 4 && 4 * i + b < bytes; ++b) {
                if (d.done < d.size) {
                    d.buffer[d.done++] = static_cast<std::byte>(w >> (8 * b));
                }
            }
        }
    }

    void out_complete(unsigned n) noexcept {
        Direction& d = out_[n];
        if (!d.busy) {
            if (n == 0) {
                ep0_out_start();
            }
            return;
        }
        if (d.last < d.mps || d.done >= d.size) {
            d.busy = false;
            if (n == 0) {
                ep0_out_start();
            }
            push({.type = usb::EventType::TransferComplete, .endpoint = static_cast<std::uint8_t>(n), .length = d.done});
        } else {
            arm_out(n);
        }
    }

    void on_setup() noexcept {
        usb::Event e{.type = usb::EventType::Setup};
        e.setup = setup_;
        in_[0].busy = out_[0].busy = false;  // a SETUP ends whatever control transfer ran
        const usb::Setup s = usb::Setup::parse(std::span<const std::byte, 8>(setup_));
        if (s.request_type == 0x00 && s.request == usb::request::set_address && s.value <= 127 && s.index == 0 &&
            s.length == 0) {
            set_address(static_cast<std::uint8_t>(s.value));  // before the status stage (see above)
        }
        ep0_out_start();
        push(e);
    }

    void push(const usb::Event& e) noexcept {
        if (!events_.push(e)) {
            ++stats_.events_dropped;
        }
    }

    Regs regs_;
    std::array<Direction, otg::endpoints> in_{}, out_{};
    std::array<std::byte, 8> setup_{};
    SpscRing<usb::Event, 32> events_;
    UsbDeviceStats stats_{};
};

}  // namespace rtos::port::stm32
