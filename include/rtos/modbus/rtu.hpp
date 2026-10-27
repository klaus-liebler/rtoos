// SPDX-License-Identifier: MIT
//
// Modbus RTU server framing (Modbus over Serial Line V1.02): address + PDU + CRC-16,
// over any byte stream - a UART or a USB CDC-ACM channel. No I/O here: the caller
// feeds received bytes and sends the returned response.
//
//   receive(bytes)  frames end when their length is known from the function code
//                   (03/04/06: 8 bytes, 16: 9 + byte count); one frame per call -
//                   further bytes stay for the next call (receive({}) continues)
//   silence()       inter-frame silence (t3.5 on a UART, an idle timeout on USB):
//                   ends a frame of unknown length (an unsupported function code,
//                   answered with exception 01); drops anything incomplete
//
// Address 0 is a broadcast: writes are executed, nothing is answered. Frames for
// other addresses and frames with a bad CRC are dropped without an answer.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/modbus/modbus.hpp"

namespace rtos::modbus {

struct RtuStats {
    std::uint32_t frames = 0;       ///< answered or executed (broadcast)
    std::uint32_t crc_errors = 0;
    std::uint32_t other_address = 0;
    std::uint32_t broadcasts = 0;
    std::uint32_t overruns = 0;     ///< more than max_adu bytes without silence
};

class RtuServer {
public:
    static constexpr std::size_t max_adu = 256;
    static constexpr std::uint8_t broadcast = 0;

    RtuServer(RegisterMap& map, std::uint8_t address) noexcept : map_(map), address_(address) {}

    [[nodiscard]] std::uint8_t address() const noexcept { return address_; }
    void set_address(std::uint8_t address) noexcept { address_ = address; }
    [[nodiscard]] const RtuStats& stats() const noexcept { return stats_; }
    /// Bytes of an unfinished frame.
    [[nodiscard]] std::size_t buffered() const noexcept { return size_; }

    /// Append `bytes`; returns the response to send for a frame they completed
    /// (empty: no frame complete, or a frame that gets no answer).
    std::span<const std::byte> receive(std::span<const std::byte> bytes) noexcept {
        for (const std::byte b : bytes) {
            if (size_ == rx_.size()) {
                overrun_ = true;  // a broken frame: discard everything until silence
                continue;
            }
            rx_[size_++] = b;
        }
        if (overrun_) {
            return {};
        }
        const std::size_t length = frame_length();
        if (length == 0 || size_ < length) {
            return {};
        }
        const std::size_t tx = handle({rx_.data(), length});
        std::copy(rx_.begin() + static_cast<std::ptrdiff_t>(length), rx_.begin() + static_cast<std::ptrdiff_t>(size_),
                  rx_.begin());
        size_ -= length;
        return {tx_.data(), tx};
    }

    /// Inter-frame silence: finish a frame of unknown length, drop anything else.
    std::span<const std::byte> silence() noexcept {
        std::size_t tx = 0;
        if (overrun_) {
            ++stats_.overruns;
        } else if (size_ >= 4 && frame_length() == unknown) {
            tx = handle({rx_.data(), size_});
        }
        size_ = 0;
        overrun_ = false;
        return {tx_.data(), tx};
    }

private:
    static constexpr std::size_t unknown = static_cast<std::size_t>(-1);

    /// Length of the frame in rx_ by its function code: 0 = not enough bytes yet,
    /// `unknown` = ends at silence.
    [[nodiscard]] std::size_t frame_length() const noexcept {
        if (size_ < 2) {
            return 0;
        }
        switch (std::to_integer<std::uint8_t>(rx_[1])) {
            case function::read_holding:
            case function::read_input:
            case function::write_single: return 8;
            case function::write_multiple: {
                if (size_ < 7) {
                    return 0;
                }
                const std::size_t length = 9 + std::to_integer<std::size_t>(rx_[6]);
                return length <= max_adu ? length : unknown;
            }
            default: return unknown;
        }
    }

    /// One complete frame: returns the response length in tx_ (0 = no answer).
    std::size_t handle(std::span<const std::byte> frame) noexcept {
        const std::size_t n = frame.size();
        const auto received = static_cast<std::uint16_t>(std::to_integer<std::uint16_t>(frame[n - 2]) |
                                                         (std::to_integer<std::uint16_t>(frame[n - 1]) << 8));
        if (crc16(frame.first(n - 2)) != received) {
            ++stats_.crc_errors;
            return 0;
        }
        const auto address = std::to_integer<std::uint8_t>(frame[0]);
        const auto pdu = frame.subspan(1, n - 3);
        if (address == broadcast) {
            if (is_write(std::to_integer<std::uint8_t>(pdu[0]))) {
                std::array<std::byte, max_pdu> ignored{};
                process_pdu(map_, pdu, ignored);
                ++stats_.broadcasts;
                ++stats_.frames;
            }
            return 0;
        }
        if (address != address_) {
            ++stats_.other_address;
            return 0;
        }
        const std::size_t length = process_pdu(map_, pdu, std::span(tx_).subspan(1, max_pdu));
        tx_[0] = frame[0];
        const std::uint16_t crc = crc16({tx_.data(), 1 + length});
        tx_[1 + length] = static_cast<std::byte>(crc & 0xFF);
        tx_[2 + length] = static_cast<std::byte>(crc >> 8);
        ++stats_.frames;
        return 3 + length;
    }

    RegisterMap& map_;
    std::uint8_t address_;
    std::array<std::byte, max_adu> rx_{};
    std::size_t size_ = 0;
    bool overrun_ = false;
    std::array<std::byte, max_adu> tx_{};
    RtuStats stats_{};
};

}  // namespace rtos::modbus
