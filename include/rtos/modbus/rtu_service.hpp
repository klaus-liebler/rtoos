// SPDX-License-Identifier: MIT
//
// Modbus RTU server on a byte channel (USB CDC-ACM, a UART driver): a NetService
// that feeds received bytes to RtuServer (rtu.hpp), sends its responses and ends a
// frame after `silence_ms` without bytes (the inter-frame gap t3.5 of a serial line;
// over USB, where bytes come in packets, a few milliseconds).
//
//   rtos::usb::CdcAcm<> acm{...};
//   rtos::modbus::RtuService rtu{acm, stack, registers, 1};   // stack: clock (now_ms)
//   runtime.add(usb_device); runtime.add(rtu);                  // after the USB device
#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/modbus/rtu.hpp"

namespace rtos::modbus {

template <class C>
concept ByteChannel = requires(C& c, std::span<std::byte> in, std::span<const std::byte> out) {
    { c.read(in) } noexcept -> std::same_as<std::size_t>;
    { c.write(out) } noexcept -> std::same_as<std::size_t>;
    { c.write_space() } noexcept -> std::same_as<std::size_t>;
};

template <class C>
concept MillisecondClock = requires(const C& c) {
    { c.now_ms() } noexcept -> std::convertible_to<std::uint32_t>;
};

template <ByteChannel Channel, MillisecondClock Clock>
class RtuService {
public:
    RtuService(Channel& channel, const Clock& clock, RegisterMap& map, std::uint8_t address,
               std::uint32_t silence_ms = 5) noexcept
        : channel_(channel), clock_(clock), rtu_(map, address), silence_ms_(silence_ms) {}

    // ---- NetService ------------------------------------------------------------
    void service() noexcept {
        if (!flush()) {
            return;  // the last response is not out yet: leave the input for later
        }
        const std::uint32_t now = clock_.now_ms();
        std::array<std::byte, 64> buffer{};
        while (const std::size_t n = channel_.read(buffer)) {
            last_rx_ms_ = now;
            reply(rtu_.receive(std::span(buffer).first(n)));
            while (pending_ == 0 && rtu_.buffered() != 0) {  // further frames already received
                const auto more = rtu_.receive({});
                if (more.empty()) {
                    break;
                }
                reply(more);
            }
            if (pending_ != 0) {
                return;
            }
        }
        if (rtu_.buffered() != 0 && static_cast<std::int32_t>(now - (last_rx_ms_ + silence_ms_)) >= 0) {
            reply(rtu_.silence());
        }
    }

    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        return rtu_.buffered() != 0 ? last_rx_ms_ + silence_ms_ : 0;
    }

    [[nodiscard]] RtuServer& server() noexcept { return rtu_; }

private:
    void reply(std::span<const std::byte> response) noexcept {
        if (response.empty()) {
            return;
        }
        std::copy(response.begin(), response.end(), out_.begin());
        pending_ = response.size();
        sent_ = 0;
        flush();
    }

    /// Hand the pending response to the channel. True when nothing is pending.
    bool flush() noexcept {
        if (sent_ < pending_) {
            sent_ += channel_.write(std::span<const std::byte>(out_).subspan(sent_, pending_ - sent_));
        }
        if (sent_ < pending_) {
            return false;
        }
        pending_ = sent_ = 0;
        return true;
    }

    Channel& channel_;
    const Clock& clock_;
    RtuServer rtu_;
    std::uint32_t silence_ms_;
    std::uint32_t last_rx_ms_ = 0;
    std::array<std::byte, RtuServer::max_adu> out_{};
    std::size_t pending_ = 0;
    std::size_t sent_ = 0;
};

}  // namespace rtos::modbus
