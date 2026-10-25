// SPDX-License-Identifier: MIT
//
// WebSocket (RFC 6455), server side.
//
//   websocket_accept_key()  Sec-WebSocket-Accept = base64(SHA-1(key + GUID))
//   WebSocketDecoder        byte-stream -> messages; client frames are unmasked
//                           while copying into the decoder's message buffer (one
//                           pass); fragmented messages are reassembled; control
//                           frames handled
//   websocket_frame_header() header for an unmasked server frame
//
// Limits: messages up to MaxMessage bytes (larger: protocol error 1009).
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace rtos::net {

namespace ws_opcode {
inline constexpr std::uint8_t continuation = 0x0, text = 0x1, binary = 0x2, close = 0x8, ping = 0x9, pong = 0xA;
}

/// Sec-WebSocket-Accept for `key` (base64, 28 characters + NUL into `out`).
/// Returns false if the key is malformed. Implemented with wolfCrypt (websocket.cpp).
bool websocket_accept_key(std::string_view key, std::array<char, 29>& out) noexcept;

/// Server frame header (FIN set, no mask). Returns the header length (2 or 4).
[[nodiscard]] constexpr std::size_t websocket_frame_header(std::byte* out, std::uint8_t opcode,
                                                           std::size_t payload) noexcept {
    out[0] = static_cast<std::byte>(0x80 | opcode);
    if (payload < 126) {
        out[1] = static_cast<std::byte>(payload);
        return 2;
    }
    out[1] = std::byte{126};
    out[2] = static_cast<std::byte>(payload >> 8);
    out[3] = static_cast<std::byte>(payload);
    return 4;
}

template <std::size_t MaxMessage = 512>
class WebSocketDecoder {
public:
    enum class Event : std::uint8_t {
        None,      ///< need more bytes
        Message,   ///< message() / message_opcode() valid until the next feed()
        Ping,      ///< reply with a pong carrying control_payload()
        Close,     ///< peer sent close (control_payload(): status code + reason)
        Error,     ///< protocol violation: close_code() tells which
    };

    /// Consume bytes from `in`; stops after an event. Returns the bytes consumed.
    std::size_t feed(std::span<const std::byte> in, Event& event) noexcept {
        event = Event::None;
        std::size_t used = 0;
        while (used < in.size() && event == Event::None) {
            if (stage_ == Stage::Payload) {
                // Copy and unmask in one pass into the message (or control) buffer.
                std::byte* dst = control_ ? control_buffer_.data() : message_.data();
                const std::size_t take = std::min(remaining_, in.size() - used);
                for (std::size_t i = 0; i < take; ++i) {
                    dst[write_pos_++] = in[used + i] ^ mask_[payload_pos_++ & 3];
                }
                used += take;
                remaining_ -= take;
                if (remaining_ == 0) {
                    finish_frame(event);
                }
                continue;
            }
            const auto v = std::to_integer<std::uint8_t>(in[used++]);
            switch (stage_) {
                case Stage::Header0:
                    fin_ = (v & 0x80) != 0;
                    opcode_ = v & 0x0F;
                    if ((v & 0x70) != 0) {
                        return fail(used, event, 1002);  // reserved bits: no extensions negotiated
                    }
                    stage_ = Stage::Header1;
                    break;
                case Stage::Header1:
                    if ((v & 0x80) == 0) {
                        return fail(used, event, 1002);  // client frames must be masked
                    }
                    length_ = v & 0x7F;
                    length_bytes_ = length_ == 126 ? 2 : (length_ == 127 ? 8 : 0);
                    if (length_bytes_ != 0) {
                        length_ = 0;
                    }
                    mask_index_ = 0;
                    stage_ = length_bytes_ != 0 ? Stage::Length : Stage::Mask;
                    break;
                case Stage::Length:
                    length_ = (length_ << 8) | v;
                    if (--length_bytes_ == 0) {
                        stage_ = Stage::Mask;
                    }
                    break;
                case Stage::Mask:
                    mask_[mask_index_++] = std::byte{v};
                    if (mask_index_ == 4 && !begin_payload(used, event)) {
                        return used;
                    }
                    break;
                case Stage::Payload:
                    break;  // handled above
            }
        }
        return used;
    }

    [[nodiscard]] std::span<const std::byte> message() const noexcept { return {message_.data(), message_size_}; }
    [[nodiscard]] std::uint8_t message_opcode() const noexcept { return message_opcode_; }
    [[nodiscard]] std::span<const std::byte> control_payload() const noexcept { return {control_buffer_.data(), control_size_}; }
    [[nodiscard]] std::uint16_t close_code() const noexcept { return close_code_; }

private:
    enum class Stage : std::uint8_t { Header0, Header1, Length, Mask, Payload };

    std::size_t fail(std::size_t used, Event& event, std::uint16_t code) noexcept {
        close_code_ = code;
        event = Event::Error;
        return used;
    }

    bool begin_payload(std::size_t used, Event& event) noexcept {
        control_ = (opcode_ & 0x8) != 0;
        if (control_) {
            if (!fin_ || length_ > 125) {
                fail(used, event, 1002);
                return false;
            }
            write_pos_ = 0;
        } else {
            if (opcode_ == ws_opcode::continuation) {
                if (!in_message_) {
                    fail(used, event, 1002);
                    return false;
                }
            } else {
                if (in_message_ || (opcode_ != ws_opcode::text && opcode_ != ws_opcode::binary)) {
                    fail(used, event, 1002);
                    return false;
                }
                in_message_ = true;
                message_opcode_ = opcode_;
                message_size_ = 0;
            }
            if (message_size_ + length_ > MaxMessage) {
                fail(used, event, 1009);  // message too big
                return false;
            }
            write_pos_ = message_size_;
        }
        remaining_ = static_cast<std::size_t>(length_);
        payload_pos_ = 0;
        stage_ = Stage::Payload;
        if (remaining_ == 0) {
            finish_frame(event);
        }
        return true;
    }

    void finish_frame(Event& event) noexcept {
        stage_ = Stage::Header0;
        if (control_) {
            control_size_ = write_pos_;
            if (opcode_ == ws_opcode::ping) {
                event = Event::Ping;
            } else if (opcode_ == ws_opcode::close) {
                event = Event::Close;
            }  // pong: ignored
            return;
        }
        message_size_ = write_pos_;
        if (fin_) {
            in_message_ = false;
            event = Event::Message;
        }
    }

    Stage stage_ = Stage::Header0;
    bool fin_ = false;
    bool control_ = false;
    bool in_message_ = false;
    std::uint8_t opcode_ = 0;
    std::uint8_t message_opcode_ = 0;
    std::uint8_t length_bytes_ = 0;
    std::uint8_t mask_index_ = 0;
    std::uint64_t length_ = 0;
    std::size_t remaining_ = 0;
    std::size_t payload_pos_ = 0;
    std::size_t write_pos_ = 0;
    std::array<std::byte, 4> mask_{};
    std::array<std::byte, MaxMessage> message_{};
    std::size_t message_size_ = 0;
    std::array<std::byte, 125> control_buffer_{};
    std::size_t control_size_ = 0;
    std::uint16_t close_code_ = 0;
};

}  // namespace rtos::net
