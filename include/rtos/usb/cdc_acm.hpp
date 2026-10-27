// SPDX-License-Identifier: MIT
//
// CDC-ACM function (virtual serial port, CDC 1.2 / PSTN 1.2): for the debug console
// and Modbus RTU over USB.
//
//   * class requests: SET/GET_LINE_CODING (stored, no UART behind it),
//     SET_CONTROL_LINE_STATE (DTR/RTS: connected() is DTR - a terminal is open),
//     SEND_BREAK (accepted)
//   * receive: OUT packets go into a ring; reception pauses (the host's OUT is NAKed)
//     while the ring has no room for another packet and resumes after read()
//   * transmit: write() fills a ring; service() sends it in transfers of up to
//     `chunk` bytes, with a zero-length packet when the ring ran empty on a packet
//     boundary (else the host would wait for more data)
//
// Rings are single-producer/single-consumer (rtos::ByteRing): write() and read() may
// run in another thread than the USB stack. Several writers (log lines from several
// threads) need their own lock around write().
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/sync/spsc_ring.hpp"
#include "rtos/usb/descriptors.hpp"
#include "rtos/usb/usb.hpp"

namespace rtos::usb {

namespace cdc_request {
inline constexpr std::uint8_t set_line_coding = 0x20, get_line_coding = 0x21, set_control_line_state = 0x22,
                              send_break = 0x23;
}

struct LineCoding {
    std::uint32_t baud = 115200;
    std::uint8_t stop_bits = 0;  ///< 0: 1, 1: 1.5, 2: 2
    std::uint8_t parity = 0;     ///< 0: none, 1: odd, 2: even, 3: mark, 4: space
    std::uint8_t data_bits = 8;
};

template <std::size_t RxSize = 512, std::size_t TxSize = 512>
class CdcAcm final : public Function {
public:
    static constexpr std::size_t chunk = 256;  ///< bytes per IN transfer (several packets)

    explicit CdcAcm(const CdcAcmInfo& info) noexcept : info_(info) {}

    // ---- application ----------------------------------------------------------------
    [[nodiscard]] std::size_t available() const noexcept { return rx_.size(); }
    std::size_t read(std::span<std::byte> out) noexcept { return rx_.read(out); }
    /// Queue `data`; returns the bytes taken (less when the ring is full).
    std::size_t write(std::span<const std::byte> data) noexcept { return tx_.write(data); }
    [[nodiscard]] std::size_t write_space() const noexcept { return tx_.space(); }
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    /// A terminal has the port open (DTR set).
    [[nodiscard]] bool connected() const noexcept { return configured_ && (line_state_ & 1) != 0; }
    [[nodiscard]] std::uint16_t line_state() const noexcept { return line_state_; }
    [[nodiscard]] const LineCoding& line_coding() const noexcept { return coding_; }

    // ---- Function ---------------------------------------------------------------------
    std::uint8_t first_interface() const noexcept override { return info_.interface; }
    std::uint8_t interface_count() const noexcept override { return 2; }
    bool owns_endpoint(std::uint8_t ep) const noexcept override {
        return ep == info_.notify_ep || ep == info_.out_ep || ep == info_.in_ep;
    }

    void configured(Endpoints& eps) noexcept override {
        eps.open({info_.notify_ep, TransferType::Interrupt, 8});
        eps.open({info_.out_ep, TransferType::Bulk, info_.packet});
        eps.open({info_.in_ep, TransferType::Bulk, info_.packet});
        configured_ = true;
        rx_armed_ = tx_busy_ = false;
        arm_rx(eps);
    }

    void reset() noexcept override {
        configured_ = rx_armed_ = tx_busy_ = false;
        line_state_ = 0;  // USBX: DTR/RTS are gone with the host
    }

    ControlResult control_setup(const Setup& s, std::span<std::byte> buffer, std::size_t& in_length) noexcept override {
        if (s.type() != Setup::Type::Class || s.recipient() != Setup::Recipient::Interface) {
            return ControlResult::Stall;
        }
        switch (s.request) {
            case cdc_request::set_line_coding:
                return s.length == 7 ? ControlResult::Accept : ControlResult::Stall;
            case cdc_request::get_line_coding: {  // wLength < 7: what fits
                const std::array<std::byte, 7> c{
                    static_cast<std::byte>(coding_.baud), static_cast<std::byte>(coding_.baud >> 8),
                    static_cast<std::byte>(coding_.baud >> 16), static_cast<std::byte>(coding_.baud >> 24),
                    std::byte{coding_.stop_bits}, std::byte{coding_.parity}, std::byte{coding_.data_bits}};
                in_length = std::min(buffer.size(), c.size());
                std::copy_n(c.begin(), in_length, buffer.begin());
                return ControlResult::Accept;
            }
            case cdc_request::set_control_line_state:
                line_state_ = s.value;
                return ControlResult::Accept;
            case cdc_request::send_break: return ControlResult::Accept;
            default: return ControlResult::Stall;
        }
    }

    void control_data(const Setup& s, std::span<const std::byte> data) noexcept override {
        if (s.request == cdc_request::set_line_coding && data.size() == 7) {
            const auto u = [&](std::size_t i) { return std::to_integer<std::uint32_t>(data[i]); };
            coding_ = {u(0) | (u(1) << 8) | (u(2) << 16) | (u(3) << 24), std::to_integer<std::uint8_t>(data[4]),
                       std::to_integer<std::uint8_t>(data[5]), std::to_integer<std::uint8_t>(data[6])};
        }
    }

    void transfer_complete(Endpoints& eps, std::uint8_t ep, std::size_t length) noexcept override {
        if (ep == info_.out_ep) {
            rx_armed_ = false;
            rx_.write(std::span<const std::byte>(packet_).first(std::min(length, packet_.size())));
            arm_rx(eps);
        } else if (ep == info_.in_ep) {
            tx_busy_ = false;
            send(eps);
        }
    }

    void service(Endpoints& eps) noexcept override {
        arm_rx(eps);  // room again after read()
        send(eps);
    }

private:
    void arm_rx(Endpoints& eps) noexcept {
        if (configured_ && !rx_armed_ && rx_.space() >= packet_.size()) {
            rx_armed_ = eps.transfer(info_.out_ep, packet_);
        }
    }

    void send(Endpoints& eps) noexcept {
        if (!configured_ || tx_busy_ || tx_.size() == 0) {
            return;
        }
        const std::size_t n = tx_.read(out_);
        const bool zlp = tx_.size() == 0;  // ring empty: end the transfer (ZLP if on a packet boundary)
        tx_busy_ = eps.transfer(info_.in_ep, std::span(out_).first(n), zlp);
    }

    CdcAcmInfo info_;
    bool configured_ = false;
    bool rx_armed_ = false;
    bool tx_busy_ = false;
    std::uint16_t line_state_ = 0;
    LineCoding coding_{};
    std::array<std::byte, 64> packet_{};  ///< one OUT packet (full speed)
    std::array<std::byte, chunk> out_{};  ///< the IN transfer in flight
    ByteRing<RxSize> rx_;
    ByteRing<TxSize> tx_;
};

}  // namespace rtos::usb
