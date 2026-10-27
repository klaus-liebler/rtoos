// SPDX-License-Identifier: MIT
//
// CDC-NCM function (USB NCM 1.0): an Ethernet link to the host PC, and at the same
// time a NetDriver for rtos::net::Stack - attach() it as another interface:
//
//   rtos::usb::CdcNcm<Pool> ncm{info, pool, device_mac};
//   const auto usb_if = stack.attach(ncm, {192.168.7.1/24});
//
// Data path (NTB16 only, no CRC):
//   * receive: the host sends NTBs (several datagrams each). Stack::poll() takes the
//     datagrams one by one through receive() - each copied into a packet buffer;
//     the OUT endpoint is re-armed only when the NTB is used up, so the host waits
//     meanwhile (no queue of whole NTBs). Validation: NCM 1.0 3.2/3.3 - signatures,
//     header length, block length, NDP position/alignment/length, every datagram
//     inside the block; chained NDPs are followed (at most max_ndps).
//   * transmit: frames from the stack wait in a ring and leave as one NTB per IN
//     transfer (datagrams 4-byte aligned: wNdpInDivisor 4 - Windows' usbncm.sys
//     rejects divisors < 4 since KB5124008, see TinyUSB PR #3914), never longer than
//     the host's NTB input size (SET_NTB_INPUT_SIZE).
//
// Link: up while the data interface is in alternate setting 1 (the host's driver
// selects it when the interface goes up). Then the notification endpoint reports
// CONNECTION_SPEED_CHANGE (12 Mbit/s) followed by NETWORK_CONNECTION (CDC 1.2 order).
//
// MAC addresses: the configuration descriptor's iMACAddress string is the HOST's
// address; the device (this driver, mac()) uses a different one.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/net/address.hpp"
#include "rtos/net/bytes.hpp"
#include "rtos/net/packet.hpp"
#include "rtos/usb/descriptors.hpp"
#include "rtos/usb/usb.hpp"

namespace rtos::usb {

namespace ncm {
inline constexpr std::uint32_t nth16_signature = 0x484D434E;   ///< "NCMH"
inline constexpr std::uint32_t ndp16_nocrc = 0x304D434E;       ///< "NCM0"
inline constexpr std::size_t nth16_size = 12;
inline constexpr std::size_t ndp16_header = 8;
inline constexpr std::uint8_t get_ntb_parameters = 0x80, get_ntb_format = 0x83, set_ntb_format = 0x84,
                              get_ntb_input_size = 0x85, set_ntb_input_size = 0x86,
                              set_ethernet_packet_filter = 0x43;
inline constexpr std::uint8_t network_connection = 0x00, connection_speed_change = 0x2A;
}  // namespace ncm

struct NcmStats {
    std::uint32_t rx_ntbs = 0;
    std::uint32_t rx_datagrams = 0;
    std::uint32_t rx_bad_ntbs = 0;    ///< rejected NTBs
    std::uint32_t rx_skipped = 0;     ///< datagrams too short/long
    std::uint32_t tx_ntbs = 0;
    std::uint32_t tx_datagrams = 0;
    std::uint32_t tx_dropped = 0;     ///< link down or queue full
};

template <net::PacketAllocator Pool, std::size_t TxFrames = 8>
class CdcNcm final : public Function {
public:
    static constexpr std::size_t ntb_size = 2048;  ///< dwNtbIn/OutMaxSize (the NCM minimum)
    static constexpr std::size_t max_datagram = 1514;
    static constexpr std::size_t max_ndps = 4;
    static constexpr std::uint16_t max_tx_datagrams = 8;
    static constexpr std::uint32_t speed_bps = 12'000'000;  ///< full speed

    CdcNcm(const CdcNcmInfo& info, Pool& pool, const net::MacAddress& mac) noexcept
        : info_(info), pool_(pool), mac_(mac) {}

    // ---- NetDriver -----------------------------------------------------------------
    [[nodiscard]] net::MacAddress mac() const noexcept { return mac_; }
    [[nodiscard]] bool link_up() const noexcept { return configured_ && alternate_ == 1; }

    bool transmit(net::PacketRef frame) noexcept {
        if (!link_up() || tx_count_ == TxFrames || !frame || frame->size() > max_datagram) {
            ++stats_.tx_dropped;
            return false;
        }
        tx_[(tx_head_ + tx_count_) % TxFrames] = std::move(frame);
        ++tx_count_;
        return true;
    }

    net::PacketRef receive() noexcept {
        while (rx_pending_) {
            if (rx_entry_ >= rx_entries_) {
                if (!next_ndp()) {
                    rx_pending_ = false;
                    rx_rearm_ = true;  // NTB used up: service() takes the next one
                    return {};
                }
                continue;
            }
            const std::size_t at = rx_ndp_ + ncm::ndp16_header + 4 * rx_entry_;
            const std::uint16_t index = le16(at);
            const std::uint16_t length = le16(at + 2);
            if (index == 0 || length == 0) {
                rx_entry_ = rx_entries_;  // end of this NDP's table
                continue;
            }
            if (index < ncm::nth16_size || std::size_t{index} + length > rx_block_) {
                ++stats_.rx_bad_ntbs;  // a datagram outside the NTB: drop the rest
                rx_entry_ = rx_entries_;
                rx_next_ndp_ = 0;
                continue;
            }
            if (length < 14 || length > max_datagram) {
                ++stats_.rx_skipped;
                ++rx_entry_;
                continue;
            }
            net::PacketRef p = pool_.allocate(0);
            if (!p || p->tailroom() < length) {
                return {};  // no buffer now: the datagram waits for the next poll
            }
            p->append(std::span<const std::byte>(rx_).subspan(index, length));
            ++rx_entry_;
            ++stats_.rx_datagrams;
            return p;
        }
        return {};
    }

    [[nodiscard]] const NcmStats& stats() const noexcept { return stats_; }
    [[nodiscard]] std::size_t ntb_input_size() const noexcept { return ntb_in_max_; }

    // ---- Function ---------------------------------------------------------------------
    std::uint8_t first_interface() const noexcept override { return info_.interface; }
    std::uint8_t interface_count() const noexcept override { return 2; }
    bool owns_endpoint(std::uint8_t ep) const noexcept override {
        return ep == info_.notify_ep || ep == info_.out_ep || ep == info_.in_ep;
    }

    void configured(Endpoints& eps) noexcept override {
        eps.open({info_.notify_ep, TransferType::Interrupt, 16});
        configured_ = true;
        alternate_ = 0;
    }

    void reset() noexcept override {
        configured_ = false;
        alternate_ = 0;
        stop_data();
        notify_busy_ = false;
        notify_speed_ = notify_connection_ = false;
        ntb_in_max_ = ntb_size;
    }

    bool set_interface(Endpoints& eps, std::uint8_t itf, std::uint8_t alt) noexcept override {
        if (itf == info_.interface) {
            return alt == 0;
        }
        if (alt > 1) {
            return false;
        }
        stop_data();
        alternate_ = alt;
        if (alt == 1) {
            eps.open({info_.in_ep, TransferType::Bulk, info_.packet});
            eps.open({info_.out_ep, TransferType::Bulk, info_.packet});
            rx_rearm_ = true;
            notify_speed_ = true;
        }
        notify_connection_ = true;
        return true;
    }

    std::uint8_t interface_alternate(std::uint8_t itf) const noexcept override {
        return itf == info_.interface ? 0 : alternate_;
    }

    ControlResult control_setup(const Setup& s, std::span<std::byte> buffer, std::size_t& in_length) noexcept override {
        if (s.type() != Setup::Type::Class || s.recipient() != Setup::Recipient::Interface ||
            (s.index & 0xFF) != info_.interface) {
            return ControlResult::Stall;
        }
        const auto reply = [&](std::span<const std::uint8_t> data) noexcept {
            in_length = std::min(buffer.size(), data.size());
            std::transform(data.begin(), data.begin() + static_cast<std::ptrdiff_t>(in_length), buffer.begin(),
                           [](std::uint8_t b) { return std::byte{b}; });
            return ControlResult::Accept;
        };
        switch (s.request) {
            case ncm::get_ntb_parameters: {  // NCM 1.0 table 6-3
                constexpr auto lo = [](std::uint32_t v, int byte) { return static_cast<std::uint8_t>(v >> (8 * byte)); };
                const std::uint8_t p[28] = {28, 0,
                                            0x01, 0,  // bmNtbFormatsSupported: NTB16
                                            lo(ntb_size, 0), lo(ntb_size, 1), 0, 0,
                                            4, 0, 0, 0, 4, 0,  // in: divisor 4, remainder 0, alignment 4
                                            0, 0,
                                            lo(ntb_size, 0), lo(ntb_size, 1), 0, 0,
                                            4, 0, 0, 0, 4, 0,  // out: divisor 4, remainder 0, alignment 4
                                            max_tx_datagrams, 0};
                return reply(p);
            }
            case ncm::get_ntb_format: {
                const std::uint8_t f[2] = {0, 0};  // NTB16
                return reply(f);
            }
            case ncm::set_ntb_format: return s.value == 0 ? ControlResult::Accept : ControlResult::Stall;
            case ncm::get_ntb_input_size: {
                const std::uint8_t v[4] = {static_cast<std::uint8_t>(ntb_in_max_), static_cast<std::uint8_t>(ntb_in_max_ >> 8),
                                           0, 0};
                return reply(v);
            }
            case ncm::set_ntb_input_size: return s.length >= 4 ? ControlResult::Accept : ControlResult::Stall;
            case ncm::set_ethernet_packet_filter: return ControlResult::Accept;  // the link carries everything
            default: return ControlResult::Stall;
        }
    }

    void control_data(const Setup& s, std::span<const std::byte> data) noexcept override {
        if (s.request == ncm::set_ntb_input_size && data.size() >= 4) {
            const std::uint32_t size = std::to_integer<std::uint32_t>(data[0]) | (std::to_integer<std::uint32_t>(data[1]) << 8) |
                                       (std::to_integer<std::uint32_t>(data[2]) << 16) |
                                       (std::to_integer<std::uint32_t>(data[3]) << 24);
            // Smaller than one maximal datagram makes no sense; larger than ours: keep ours.
            ntb_in_max_ = std::clamp<std::size_t>(size, 64 + max_datagram, ntb_size);
        }
    }

    void transfer_complete(Endpoints& eps, std::uint8_t ep, std::size_t length) noexcept override {
        if (ep == info_.out_ep) {
            rx_armed_ = false;
            parse(length);
        } else if (ep == info_.in_ep) {
            tx_busy_ = false;
            send(eps);
        } else if (ep == info_.notify_ep) {
            notify_busy_ = false;
            notify(eps);
        }
    }

    void service(Endpoints& eps) noexcept override {
        if (rx_rearm_ && !rx_armed_ && link_up()) {
            rx_armed_ = eps.transfer(info_.out_ep, rx_);
            rx_rearm_ = !rx_armed_;
        }
        send(eps);
        notify(eps);
    }

private:
    [[nodiscard]] std::uint16_t le16(std::size_t at) const noexcept {
        return static_cast<std::uint16_t>(std::to_integer<std::uint16_t>(rx_[at]) |
                                          (std::to_integer<std::uint16_t>(rx_[at + 1]) << 8));
    }
    [[nodiscard]] std::uint32_t le32(std::size_t at) const noexcept {
        return le16(at) | (std::uint32_t{le16(at + 2)} << 16);
    }

    void stop_data() noexcept {
        for (auto& f : tx_) {
            f.reset();
        }
        tx_head_ = tx_count_ = 0;
        tx_busy_ = rx_armed_ = rx_rearm_ = rx_pending_ = false;
    }

    /// A received NTB: check the header and the first NDP; datagrams follow in receive().
    void parse(std::size_t length) noexcept {
        ++stats_.rx_ntbs;
        rx_block_ = 0;
        if (length < ncm::nth16_size || le32(0) != ncm::nth16_signature || le16(4) != ncm::nth16_size) {
            reject();
            return;
        }
        const std::uint16_t block = le16(8);
        if (block < ncm::nth16_size || block > length) {
            reject();
            return;
        }
        rx_block_ = block;
        rx_next_ndp_ = le16(10);
        rx_ndps_ = 0;
        if (!next_ndp()) {
            reject();
            return;
        }
        rx_pending_ = true;
    }

    /// Move to the NDP at rx_next_ndp_. False if there is none (or it is invalid).
    bool next_ndp() noexcept {
        const std::size_t at = rx_next_ndp_;
        if (at == 0 || rx_ndps_ == max_ndps) {
            return false;
        }
        // Chained NDPs must move forward: a loop would deliver the same datagrams again.
        const bool forward = rx_ndps_ == 0 || at > rx_ndp_;
        ++rx_ndps_;
        if (!forward || at < ncm::nth16_size || at % 4 != 0 || at + ncm::ndp16_header > rx_block_ ||
            le32(at) != ncm::ndp16_nocrc) {
            ++stats_.rx_bad_ntbs;
            return false;
        }
        const std::uint16_t ndp_length = le16(at + 4);
        if (ndp_length < 16 || ndp_length % 4 != 0 || at + ndp_length > rx_block_) {
            ++stats_.rx_bad_ntbs;
            return false;
        }
        rx_ndp_ = at;
        rx_entries_ = (ndp_length - ncm::ndp16_header) / 4;
        rx_entry_ = 0;
        rx_next_ndp_ = le16(at + 6);
        return true;
    }

    void reject() noexcept {
        ++stats_.rx_bad_ntbs;
        rx_pending_ = false;
        rx_rearm_ = true;
    }

    /// Queued frames -> one NTB: NTH16, NDP16 (k entries + terminator), datagrams at
    /// 4-byte boundaries.
    void send(Endpoints& eps) noexcept {
        if (tx_busy_ || tx_count_ == 0 || !link_up()) {
            return;
        }
        std::size_t k = 0;
        std::size_t end = 0;
        for (; k < tx_count_ && k < max_tx_datagrams; ++k) {
            const std::size_t ndp_end = ncm::nth16_size + ncm::ndp16_header + 4 * (k + 2);
            std::size_t pos = align4(ndp_end);
            for (std::size_t i = 0; i <= k; ++i) {
                pos = align4(pos) + tx_[(tx_head_ + i) % TxFrames]->size();
            }
            if (pos > ntb_in_max_) {
                break;
            }
            end = pos;
        }
        if (k == 0) {  // a single frame larger than the host's NTB size: drop it
            tx_[tx_head_].reset();
            tx_head_ = (tx_head_ + 1) % TxFrames;
            --tx_count_;
            ++stats_.tx_dropped;
            return;
        }
        const std::size_t ndp_length = ncm::ndp16_header + 4 * (k + 1);
        std::fill_n(tx_ntb_.begin(), ncm::nth16_size + ndp_length, std::byte{0});
        put16(0, 0x434E);  // "NCMH" little endian
        put16(2, 0x484D);
        put16(4, ncm::nth16_size);
        put16(6, sequence_++);
        put16(8, static_cast<std::uint16_t>(end));
        put16(10, ncm::nth16_size);
        const std::size_t ndp = ncm::nth16_size;
        put16(ndp, 0x434E);  // "NCM0"
        put16(ndp + 2, 0x304D);
        put16(ndp + 4, static_cast<std::uint16_t>(ndp_length));
        put16(ndp + 6, 0);
        std::size_t pos = ndp + ndp_length;
        for (std::size_t i = 0; i < k; ++i) {
            net::PacketRef& f = tx_[tx_head_];
            const std::size_t gap = align4(pos) - pos;
            std::fill_n(tx_ntb_.begin() + static_cast<std::ptrdiff_t>(pos), gap, std::byte{0});
            pos += gap;
            put16(ndp + 8 + 4 * i, static_cast<std::uint16_t>(pos));
            put16(ndp + 10 + 4 * i, static_cast<std::uint16_t>(f->size()));
            std::copy_n(f->data(), f->size(), tx_ntb_.begin() + static_cast<std::ptrdiff_t>(pos));
            pos += f->size();
            f.reset();
            tx_head_ = (tx_head_ + 1) % TxFrames;
            --tx_count_;
        }
        // NCM 1.0 3.2.2: a ZLP ends an NTB on a packet boundary shorter than the maximum.
        tx_busy_ = eps.transfer(info_.in_ep, std::span(tx_ntb_).first(end), end < ntb_in_max_);
        ++stats_.tx_ntbs;
        stats_.tx_datagrams += static_cast<std::uint32_t>(k);
    }

    void notify(Endpoints& eps) noexcept {
        if (notify_busy_ || !configured_) {
            return;
        }
        std::size_t length = 8;
        notification_.fill(std::byte{0});
        notification_[0] = std::byte{0xA1};  // device-to-host, class, interface
        notification_[4] = std::byte{info_.interface};
        if (notify_speed_) {
            notify_speed_ = false;
            notification_[1] = std::byte{ncm::connection_speed_change};
            notification_[6] = std::byte{8};
            for (int i = 0; i < 4; ++i) {  // downlink and uplink bit rate
                notification_[8 + i] = notification_[12 + i] = static_cast<std::byte>(speed_bps >> (8 * i));
            }
            length = 16;
        } else if (notify_connection_) {
            notify_connection_ = false;
            notification_[1] = std::byte{ncm::network_connection};
            notification_[2] = std::byte{alternate_ == 1 ? std::uint8_t{1} : std::uint8_t{0}};
        } else {
            return;
        }
        notify_busy_ = eps.transfer(info_.notify_ep, std::span(notification_).first(length));
    }

    [[nodiscard]] static constexpr std::size_t align4(std::size_t v) noexcept { return (v + 3) & ~std::size_t{3}; }
    void put16(std::size_t at, std::uint16_t v) noexcept {
        tx_ntb_[at] = static_cast<std::byte>(v & 0xFF);
        tx_ntb_[at + 1] = static_cast<std::byte>(v >> 8);
    }

    CdcNcmInfo info_;
    Pool& pool_;
    net::MacAddress mac_;
    bool configured_ = false;
    std::uint8_t alternate_ = 0;
    std::size_t ntb_in_max_ = ntb_size;
    // receive
    alignas(4) std::array<std::byte, ntb_size> rx_{};
    bool rx_armed_ = false;
    bool rx_rearm_ = false;
    bool rx_pending_ = false;  ///< datagrams of the last NTB not yet taken
    std::size_t rx_block_ = 0;
    std::size_t rx_ndp_ = 0;
    std::size_t rx_next_ndp_ = 0;
    std::size_t rx_ndps_ = 0;
    std::size_t rx_entries_ = 0;
    std::size_t rx_entry_ = 0;
    // transmit
    std::array<net::PacketRef, TxFrames> tx_{};
    std::size_t tx_head_ = 0;
    std::size_t tx_count_ = 0;
    bool tx_busy_ = false;
    std::uint16_t sequence_ = 0;
    alignas(4) std::array<std::byte, ntb_size> tx_ntb_{};
    // notifications
    std::array<std::byte, 16> notification_{};
    bool notify_busy_ = false;
    bool notify_speed_ = false;
    bool notify_connection_ = false;
    NcmStats stats_{};
};

}  // namespace rtos::usb
