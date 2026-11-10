// SPDX-License-Identifier: MIT
#pragma once
// Message frames on a byte stream (a UART): a UART has no message boundaries and loses or garbles
// bytes, so each message travels as
//
//     COBS( payload | CRC-16 little endian ) 0x00
//
// COBS (Consistent Overhead Byte Stuffing) removes every 0x00 from the frame at a cost of one byte
// per 254, so 0x00 marks the end of a frame and only that: a receiver that starts in the middle of a
// frame, or loses a byte, is back in step at the next 0x00. The CRC (CCITT: polynomial 0x1021,
// initial value 0xFFFF) throws away what was garbled. Nothing here allocates; the decoder is fed
// one byte at a time from an interrupt-filled ring.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace rtos::serial {

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR): "123456789" -> 0x29B1.
[[nodiscard]] constexpr std::uint16_t crc16_ccitt(std::span<const std::uint8_t> data, std::uint16_t crc = 0xFFFF) noexcept {
    for (const std::uint8_t b : data) {
        crc = static_cast<std::uint16_t>(crc ^ (b << 8));
        for (int i = 0; i < 8; ++i) {
            crc = static_cast<std::uint16_t>((crc & 0x8000) != 0 ? (crc << 1) ^ 0x1021 : crc << 1);
        }
    }
    return crc;
}

/// The bytes a payload of `n` bytes takes on the wire at most: COBS overhead, CRC, delimiter.
[[nodiscard]] constexpr std::size_t max_frame_size(std::size_t n) noexcept {
    const std::size_t body = n + 2;
    return body + body / 254 + 1 + 1;
}

/// Writes the frame of `payload` to `out`; the number of bytes, 0 when `out` is too small.
[[nodiscard]] constexpr std::size_t encode_frame(std::span<const std::uint8_t> payload, std::span<std::uint8_t> out) noexcept {
    if (out.size() < max_frame_size(payload.size())) {
        return 0;
    }
    const std::uint16_t crc = crc16_ccitt(payload);
    std::size_t code_at = 0;  // where the current block's length byte goes
    std::size_t w = 1;
    std::uint8_t code = 1;
    auto put = [&](std::uint8_t b) {
        if (b == 0) {
            out[code_at] = code;
            code_at = w++;
            code = 1;
            return;
        }
        out[w++] = b;
        if (++code == 0xFF) {  // a full block of 254 non-zero bytes: no zero follows it
            out[code_at] = code;
            code_at = w++;
            code = 1;
        }
    };
    for (const std::uint8_t b : payload) {
        put(b);
    }
    put(static_cast<std::uint8_t>(crc & 0xFF));
    put(static_cast<std::uint8_t>(crc >> 8));
    out[code_at] = code;
    out[w++] = 0x00;
    return w;
}

/// Collects the bytes of a stream and hands out each frame's payload whose CRC is right.
/// `Capacity`: the largest frame (encoded, without the delimiter) - use max_frame_size().
template <std::size_t Capacity>
class FrameDecoder {
public:
    struct Errors {
        std::uint32_t crc = 0;       ///< garbled frames
        std::uint32_t overflow = 0;  ///< frames longer than Capacity
        std::uint32_t framing = 0;   ///< not COBS (a block running past the delimiter, or too short)
    };

    /// One byte of the stream. At the end of a good frame: its payload (valid until the next push).
    [[nodiscard]] constexpr std::optional<std::span<const std::uint8_t>> push(std::uint8_t b) noexcept {
        if (b != 0) {
            if (n_ < Capacity) {
                buf_[n_] = b;
            }
            ++n_;  // counted on: an overflowing frame is dropped at its delimiter
            return std::nullopt;
        }
        const std::size_t n = n_;
        n_ = 0;
        if (n == 0) {
            return std::nullopt;  // an empty frame: a delimiter sent to flush a line, no error
        }
        if (n > Capacity) {
            ++errors_.overflow;
            return std::nullopt;
        }
        // COBS decode in place (the write index never passes the read index)
        std::size_t r = 0, w = 0;
        while (r < n) {
            const std::uint8_t code = buf_[r++];
            if (r + code - 1 > n) {
                ++errors_.framing;
                return std::nullopt;
            }
            for (std::uint8_t i = 1; i < code; ++i) {
                buf_[w++] = buf_[r++];
            }
            if (code != 0xFF && r < n) {
                buf_[w++] = 0;
            }
        }
        if (w < 2) {
            ++errors_.framing;
            return std::nullopt;
        }
        const std::size_t len = w - 2;
        const auto crc = static_cast<std::uint16_t>(buf_[len] | (buf_[len + 1] << 8));
        if (crc16_ccitt(std::span<const std::uint8_t>(buf_, len)) != crc) {
            ++errors_.crc;
            return std::nullopt;
        }
        return std::span<const std::uint8_t>(buf_, len);
    }

    [[nodiscard]] constexpr const Errors& errors() const noexcept { return errors_; }

private:
    std::uint8_t buf_[Capacity] = {};
    std::size_t n_ = 0;
    Errors errors_{};
};

}  // namespace rtos::serial
