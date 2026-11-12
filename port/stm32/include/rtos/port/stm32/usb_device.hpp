// SPDX-License-Identifier: MIT
//
// STM32 full-speed USB device controller: the rtos::usb::DeviceController for the USB stack.
// One driver for the STM32H5 USB_DRD_FS (device mode) and the STM32G4 USB device - the same IP
// generation (RM0481/RM0440 chapter USB; registers, bits: SVD v1.8 of both chips; endpoint
// register toggle rules, buffer descriptor layout, PMA size: ST's stm32h563xx.h /
// stm32h5xx_ll_usb.h, stm32g431xx.h / stm32g4xx_ll_usb.c):
//
//   register             H5 USB_DRD_FS                   G4 USB
//   endpoint n           CHEPnR at 4 n                   EPnR at 4 n (bits 15:0 the same)
//   CNTR ISTR DADDR BCDR at 0x40 0x44 0x4C 0x58          the same, same bits
//   PMA                  2 KB, 32-bit accesses           1 KB, 16-bit accesses (BTABLE = 0)
//   buffer descriptors   TXBD/RXBD as 32-bit words       ADDR/COUNT_TX, ADDR/COUNT_RX: 16-bit
//                        at 8 n and 8 n + 4              halves of the same words
//
// The chip's hardware access (`Regs`: stm32h5::UsbHardware, stm32g4::UsbHardware) presents the
// PMA as 32-bit words either way; host tests put a simulated controller there.
//
//   stm32h5::UsbDrdFs<> usb;            // UsbHardware::enable_clocks() first (board)
//   rtos::usb::Device<stm32h5::UsbDrdFs<>> device{usb, descriptors, functions};
//   extern "C" void USB_DRD_FS_IRQHandler() { usb.on_irq(); wake the network thread }
//
// The endpoint registers (n = endpoint number, both directions) have fields of three
// kinds: plain (EA, UTYPE, KIND), toggle-on-write-1 (STATRX/TX, DTOGRX/TX) and clear-on-write-0
// (VTRX/VTTX). Every write is computed from a read so that it changes only what it means to.
// The packet memory starts with the buffer descriptor table: TXBD/RXBD of endpoint n at 8 n
// (address in bits 15:0, count in 31:16; RX: count 25:16, NUM_BLOCK 30:26, BLSIZE 31), the
// packet buffers follow.
//
// The interrupt handler moves packets between the PMA and the transfer buffers and re-arms the
// endpoint; it queues an event (SPSC ring) for every SETUP, completed transfer, bus reset,
// suspend and resume. The stack's thread takes them with poll(). EP0 OUT stays ready while
// idle: a packet that arrives before the stack asked for it is kept and handed over by the
// next transfer() - so the driver works whether or not the controller accepts SETUP packets
// while STAT_RX is NAK.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "rtos/port/cortex_m/registers.hpp"
#include "rtos/port/stm32/rcc.hpp"
#include "rtos/sync/spsc_ring.hpp"
#include "rtos/usb/usb.hpp"

namespace rtos::port::stm32 {

namespace usb_detail {
// CHEPnR
inline constexpr std::uint32_t ea_mask = 0xFu;
inline constexpr std::uint32_t stat_tx_mask = 3u << 4, dtog_tx = 1u << 6, vttx = 1u << 7, kind = 1u << 8,
                               utype_mask = 3u << 9, setup = 1u << 11, stat_rx_mask = 3u << 12,
                               dtog_rx = 1u << 14, vtrx = 1u << 15;
inline constexpr std::uint32_t reg_mask = 0x07FF'8F8Fu;  // H5 USB_CHEP_REG_MASK: everything but the toggles
                                                       // (G4 USB_EPREG_MASK 0x8F8F: bits 31:16 read 0)
inline constexpr std::uint32_t stat_disabled = 0, stat_stall = 1, stat_nak = 2, stat_valid = 3;
inline constexpr std::uint32_t utype_bulk = 0u << 9, utype_control = 1u << 9, utype_iso = 2u << 9,
                               utype_interrupt = 3u << 9;
// CNTR
inline constexpr std::uint32_t cntr_usbrst = 1u << 0, cntr_pdwn = 1u << 1, cntr_suspm = 1u << 11,
                               cntr_wkupm = 1u << 12, cntr_rst_dconm = 1u << 10, cntr_ctrm = 1u << 15,
                               cntr_suspen = 1u << 3;
// ISTR
inline constexpr std::uint32_t istr_idn = 0xFu, istr_rst_dcon = 1u << 10, istr_susp = 1u << 11,
                               istr_wkup = 1u << 12, istr_ctr = 1u << 15;
// DADDR, BCDR
inline constexpr std::uint32_t daddr_ef = 1u << 7;
inline constexpr std::uint32_t bcdr_dppu = 1u << 15;
inline constexpr std::size_t endpoints = 8;
inline constexpr std::uint32_t rx_blsize = 1u << 31;

/// RXBD count field for a buffer of `bytes` (ST's USB_DRD_SET_CHEP_CNT_RX_REG).
[[nodiscard]] constexpr std::uint32_t rx_count_field(std::uint32_t bytes) noexcept {
    if (bytes == 0) {
        return rx_blsize;
    }
    if (bytes <= 62) {
        return ((bytes + 1) / 2) << 26;  // 2-byte blocks
    }
    const std::uint32_t blocks = bytes / 32 - (bytes % 32 == 0 ? 1 : 0);  // 32-byte blocks, minus one
    return (blocks << 26) | rx_blsize;
}
}  // namespace usb_detail

struct UsbDeviceStats {
    std::uint32_t events_dropped = 0;  ///< event ring full: the stack's thread is too slow
    std::uint32_t ep0_kept = 0;        ///< EP0 OUT packets kept until the stack asked for them
};

/// `Regs`: the chip's register access - irq, pma_bytes, Masked (interrupts off), the endpoint
/// and control registers and the PMA as 32-bit words (see stm32h5::UsbHardware).
template <class Regs>
class UsbDevice {
public:
    static constexpr std::uint8_t irq = Regs::irq;
    static constexpr std::size_t pma_bytes = Regs::pma_bytes;

    explicit UsbDevice(Regs regs = {}) noexcept : regs_(regs) {}
    UsbDevice(const UsbDevice&) = delete;
    UsbDevice& operator=(const UsbDevice&) = delete;

    /// Power the controller up and enable its interrupts (pull-up still off: connect()).
    void start() noexcept {
        using namespace usb_detail;
        regs_.set_cntr(cntr_usbrst);  // PDWN off, still in reset
        for (int i = 0; i < 250; ++i) {  // t_STARTUP 1 us: 250 peripheral reads take longer
            (void)regs_.cntr();
        }
        regs_.set_cntr(0);
        regs_.set_istr(0);
        regs_.set_cntr(cntr_ctrm | cntr_rst_dconm | cntr_suspm | cntr_wkupm);
        reset_endpoints();
    }

    // ---- rtos::usb::DeviceController -----------------------------------------------------

    std::optional<usb::Event> poll() noexcept { return events_.pop(); }

    bool open(const usb::EndpointConfig& c) noexcept {
        using namespace usb_detail;
        const unsigned n = usb::endpoint_number(c.address);
        if (n == 0 || n >= endpoints || c.max_packet == 0 || c.max_packet > 64) {
            return false;
        }
        const std::uint16_t pma = allocate(c.max_packet);
        if (pma == 0) {
            return false;
        }
        const std::uint32_t utype = c.type == usb::TransferType::Interrupt  ? utype_interrupt
                                    : c.type == usb::TransferType::Isochronous ? utype_iso
                                                                                : utype_bulk;
        const std::uint32_t current = regs_.chep(n) & reg_mask & ~(ea_mask | utype_mask | kind | vtrx | vttx);
        regs_.set_chep(n, current | utype | n | vtrx | vttx);
        Direction& d = usb::is_in(c.address) ? in_[n] : out_[n];
        d = {};
        d.mps = c.max_packet;
        d.pma = pma;
        if (usb::is_in(c.address)) {
            set_tx_address(n, pma);
            clear_dtog(n, dtog_tx);
            set_stat(n, true, stat_nak);
        } else {
            set_rx_buffer(n, pma, c.max_packet);
            clear_dtog(n, dtog_rx);
            set_stat(n, false, stat_nak);
        }
        return true;
    }

    void close_all() noexcept {
        using namespace usb_detail;
        for (unsigned n = 1; n < endpoints; ++n) {
            set_stat(n, true, stat_disabled);
            set_stat(n, false, stat_disabled);
            in_[n] = {};
            out_[n] = {};
        }
        pma_next_ = ep0_end;
    }

    bool transfer(std::uint8_t ep, std::span<std::byte> buffer, bool zlp) noexcept {
        using namespace usb_detail;
        const unsigned n = usb::endpoint_number(ep);
        if (n >= endpoints) {
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
        if (usb::is_in(ep)) {
            d.zlp = buffer.empty() || (zlp && buffer.size() % d.mps == 0);
            [[maybe_unused]] const Masked masked;  // the ISR works on the same register
            send_packet(n);
        } else {
            [[maybe_unused]] const Masked masked;
            if (n == 0 && ep0_kept_) {  // the packet came before the stack asked for it
                ep0_kept_ = false;
                take_packet(0, ep0_kept_length_, ep0_kept_data_.data());
            } else {
                set_stat(n, false, stat_valid);
            }
        }
        return true;
    }

    void stall(std::uint8_t ep) noexcept {
        using namespace usb_detail;
        const unsigned n = usb::endpoint_number(ep);
        [[maybe_unused]] const Masked masked;
        if (n == 0) {  // protocol stall: both directions until the next SETUP
            set_stat(0, true, stat_stall);
            set_stat(0, false, stat_stall);
            in_[0].busy = out_[0].busy = false;
            return;
        }
        set_stat(n, usb::is_in(ep), stat_stall);
        (usb::is_in(ep) ? in_[n] : out_[n]).busy = false;
    }

    void clear_stall(std::uint8_t ep) noexcept {
        using namespace usb_detail;
        const unsigned n = usb::endpoint_number(ep);
        [[maybe_unused]] const Masked masked;
        clear_dtog(n, usb::is_in(ep) ? dtog_tx : dtog_rx);
        set_stat(n, usb::is_in(ep), n == 0 && !usb::is_in(ep) ? stat_valid : stat_nak);
    }

    void set_address(std::uint8_t address) noexcept { regs_.set_daddr(usb_detail::daddr_ef | address); }

    void connect() noexcept { regs_.set_bcdr(regs_.bcdr() | usb_detail::bcdr_dppu); }
    void disconnect() noexcept { regs_.set_bcdr(regs_.bcdr() & ~usb_detail::bcdr_dppu); }

    // ---- interrupt ---------------------------------------------------------------------

    void on_irq() noexcept {
        using namespace usb_detail;
        std::uint32_t istr = regs_.istr();
        if ((istr & istr_rst_dcon) != 0) {
            regs_.set_istr(~istr_rst_dcon);  // rc_w0
            reset_endpoints();
            push({.type = usb::EventType::BusReset});
        }
        while (((istr = regs_.istr()) & istr_ctr) != 0) {
            const unsigned n = istr & istr_idn;
            const std::uint32_t chep = regs_.chep(n);
            if ((chep & vtrx) != 0) {
                // the packet's count and data in the PMA may still be on their way (Regs::settle_rx)
                regs_.settle_rx();
                clear_ctr(n, vtrx);
                if ((chep & setup) != 0) {
                    on_setup();
                } else {
                    on_out(n);
                }
            }
            if ((chep & vttx) != 0) {
                clear_ctr(n, vttx);
                on_in(n);
            }
        }
        if ((istr & istr_susp) != 0) {
            regs_.set_istr(~istr_susp);
            push({.type = usb::EventType::Suspend});
        }
        if ((istr & istr_wkup) != 0) {
            regs_.set_istr(~istr_wkup);
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
        std::uint16_t mps = 0;  ///< 0: closed
        std::uint16_t pma = 0;
        std::uint16_t in_flight = 0;  ///< IN: bytes of the packet being sent
        bool busy = false;
        bool zlp = false;  ///< IN: a zero-length packet still to send at the end
    };

    using Masked = typename Regs::Masked;

    static constexpr std::uint16_t ep0_packet = 64;
    static constexpr std::uint16_t ep0_out_pma = 8 * usb_detail::endpoints;  // after the BD table
    static constexpr std::uint16_t ep0_in_pma = ep0_out_pma + ep0_packet;
    static constexpr std::uint16_t ep0_end = ep0_in_pma + ep0_packet;

    void reset_endpoints() noexcept {
        using namespace usb_detail;
        close_all();
        in_[0] = {.mps = ep0_packet, .pma = ep0_in_pma};
        out_[0] = {.mps = ep0_packet, .pma = ep0_out_pma};
        ep0_kept_ = false;
        const std::uint32_t current = regs_.chep(0) & reg_mask & ~(ea_mask | utype_mask | kind | vtrx | vttx);
        regs_.set_chep(0, current | utype_control | vtrx | vttx);
        set_tx_address(0, ep0_in_pma);
        set_rx_buffer(0, ep0_out_pma, ep0_packet);
        clear_dtog(0, dtog_tx);
        clear_dtog(0, dtog_rx);
        set_stat(0, true, stat_nak);
        set_stat(0, false, stat_valid);
        regs_.set_daddr(daddr_ef);
    }

    std::uint16_t allocate(std::uint16_t bytes) noexcept {
        const std::uint16_t size = static_cast<std::uint16_t>((bytes + 3u) & ~3u);
        if (pma_next_ + size > pma_bytes) {
            return 0;
        }
        const std::uint16_t at = pma_next_;
        pma_next_ = static_cast<std::uint16_t>(pma_next_ + size);
        return at;
    }

    // ---- endpoint register writes (ST's toggle rules) -------------------------------------

    /// STATTX (tx) or STATRX to `value`: write 1 where a bit has to flip.
    void set_stat(unsigned n, bool tx, std::uint32_t value) noexcept {
        using namespace usb_detail;
        const std::uint32_t r = regs_.chep(n);
        const std::uint32_t shift = tx ? 4 : 12;
        const std::uint32_t flip = (((r >> shift) & 3u) ^ value) << shift;
        regs_.set_chep(n, (r & reg_mask) | flip | vtrx | vttx);
    }

    void clear_dtog(unsigned n, std::uint32_t dtog) noexcept {
        using namespace usb_detail;
        const std::uint32_t r = regs_.chep(n);
        if ((r & dtog) != 0) {
            regs_.set_chep(n, (r & reg_mask) | dtog | vtrx | vttx);
        }
    }

    /// Clear VTRX or VTTX (write 0 there, 1 to the other).
    void clear_ctr(unsigned n, std::uint32_t ctr) noexcept {
        using namespace usb_detail;
        const std::uint32_t r = regs_.chep(n);
        regs_.set_chep(n, (r & reg_mask & ~ctr) | ((vtrx | vttx) & ~ctr));
    }

    // ---- packet memory ---------------------------------------------------------------------

    void set_tx_address(unsigned n, std::uint16_t pma) noexcept {
        regs_.pma_write(8 * n, (regs_.pma_read(8 * n) & 0xFFFF'0000u) | pma);
    }
    void set_tx_count(unsigned n, std::uint16_t count) noexcept {
        regs_.pma_write(8 * n, (regs_.pma_read(8 * n) & 0x0000'FFFFu) | (std::uint32_t{count} << 16));
    }
    void set_rx_buffer(unsigned n, std::uint16_t pma, std::uint16_t bytes) noexcept {
        regs_.pma_write(8 * n + 4, usb_detail::rx_count_field(bytes) | pma);
    }
    std::uint16_t rx_count(unsigned n) noexcept {
        return static_cast<std::uint16_t>((regs_.pma_read(8 * n + 4) >> 16) & 0x3FFu);
    }

    void copy_to_pma(std::uint16_t pma, const std::byte* src, std::size_t length) noexcept {
        for (std::size_t i = 0; i < length; i += 4) {
            std::uint32_t w = 0;
            for (std::size_t b = 0; b < 4 && i + b < length; ++b) {
                w |= std::to_integer<std::uint32_t>(src[i + b]) << (8 * b);
            }
            regs_.pma_write(pma + i, w);
        }
    }
    void copy_from_pma(std::uint16_t pma, std::byte* dst, std::size_t length) noexcept {
        for (std::size_t i = 0; i < length; i += 4) {
            const std::uint32_t w = regs_.pma_read(pma + i);
            for (std::size_t b = 0; b < 4 && i + b < length; ++b) {
                dst[i + b] = static_cast<std::byte>(w >> (8 * b));
            }
        }
    }

    // ---- transfers -------------------------------------------------------------------------

    void send_packet(unsigned n) noexcept {
        Direction& d = in_[n];
        const std::uint16_t left = static_cast<std::uint16_t>(d.size - d.done);
        const std::uint16_t chunk = left < d.mps ? left : d.mps;
        if (chunk == 0) {
            d.zlp = false;  // this is the zero-length packet
        }
        copy_to_pma(d.pma, d.buffer + d.done, chunk);
        set_tx_count(n, chunk);
        d.in_flight = chunk;
        set_stat(n, true, usb_detail::stat_valid);
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

    void on_out(unsigned n) noexcept {
        Direction& d = out_[n];
        const std::uint16_t count = rx_count(n);
        if (!d.busy) {
            if (n == 0) {  // keep it until the stack asks (EP0 stays ready while idle)
                ep0_kept_length_ = count > ep0_packet ? ep0_packet : count;
                copy_from_pma(d.pma, ep0_kept_data_.data(), ep0_kept_length_);
                ep0_kept_ = true;
                ++stats_.ep0_kept;
            }
            return;
        }
        std::array<std::byte, 64> packet;
        const std::uint16_t take = count > d.mps ? d.mps : count;
        copy_from_pma(d.pma, packet.data(), take);
        take_packet(n, take, packet.data());
    }

    /// One received packet into the transfer of OUT endpoint `n`.
    void take_packet(unsigned n, std::uint16_t count, const std::byte* data) noexcept {
        Direction& d = out_[n];
        const std::uint16_t room = static_cast<std::uint16_t>(d.size - d.done);
        const std::uint16_t take = count < room ? count : room;
        for (std::uint16_t i = 0; i < take; ++i) {
            d.buffer[d.done + i] = data[i];
        }
        d.done = static_cast<std::uint16_t>(d.done + take);
        if (count < d.mps || d.done >= d.size) {
            d.busy = false;
            if (n == 0) {
                set_stat(0, false, usb_detail::stat_valid);  // ready again for SETUP/OUT
            }
            push({.type = usb::EventType::TransferComplete, .endpoint = static_cast<std::uint8_t>(n), .length = d.done});
        } else {
            set_stat(n, false, usb_detail::stat_valid);
        }
    }

    void on_setup() noexcept {
        usb::Event e{.type = usb::EventType::Setup};
        copy_from_pma(ep0_out_pma, e.setup.data(), 8);
        in_[0].busy = out_[0].busy = false;  // a SETUP ends whatever control transfer ran
        ep0_kept_ = false;
        set_stat(0, true, usb_detail::stat_nak);  // the stack answers with a transfer
        set_stat(0, false, usb_detail::stat_valid);
        push(e);
    }

    void push(const usb::Event& e) noexcept {
        if (!events_.push(e)) {
            ++stats_.events_dropped;
        }
    }

    Regs regs_;
    std::array<Direction, usb_detail::endpoints> in_{}, out_{};
    std::uint16_t pma_next_ = ep0_end;
    bool ep0_kept_ = false;
    std::uint16_t ep0_kept_length_ = 0;
    std::array<std::byte, ep0_packet> ep0_kept_data_{};
    SpscRing<usb::Event, 32> events_;
    UsbDeviceStats stats_{};
};

}  // namespace rtos::port::stm32
