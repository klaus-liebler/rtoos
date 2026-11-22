// SPDX-License-Identifier: MIT
// AFL++ harness: the WebSocket frame decoder (rtos/net/websocket.hpp, RFC 6455) over a byte stream.
// Input: byte 0 picks the segment sizes (1..32 bytes), the rest is the stream of frames.
#include <algorithm>

#include "fuzz_main.hpp"
#include "rtos/net/websocket.hpp"

void fuzz_one(std::span<const std::uint8_t> input) {
    if (input.empty()) return;
    using Decoder = rtos::net::WebSocketDecoder<512>;
    const std::size_t segment = 1 + input[0] % 32;
    const auto stream = std::as_bytes(input.subspan(1));
    Decoder d;
    std::size_t pos = 0;
    for (int rounds = 0; rounds < 65536 && pos < stream.size(); ++rounds) {
        const std::size_t n = std::min(stream.size() - pos, segment);
        Decoder::Event e{};
        const std::size_t used = d.feed(stream.subspan(pos, n), e);
        if (used > n) __builtin_trap();
        pos += used;
        if (e == Decoder::Event::Message) {
            if (d.message().size() > 512) __builtin_trap();
        } else if (e == Decoder::Event::Ping || e == Decoder::Event::Close) {
            if (d.control_payload().size() > 125) __builtin_trap();
        } else if (e == Decoder::Event::Error) {
            break;
        } else if (used == 0 && e == Decoder::Event::None) {
            __builtin_trap();  // no progress: the connection would hang
        }
    }
}
