// SPDX-License-Identifier: MIT
// AFL++ harness: the HTTP request parser (rtos/net/http.hpp) over a TCP byte stream.
// Input: byte 0 picks the segment sizes (1..64 bytes), the rest is the stream - pipelined
// requests included.
#include <algorithm>
#include <cstring>

#include "fuzz_main.hpp"
#include "rtos/net/http.hpp"

void fuzz_one(std::span<const std::uint8_t> input) {
    if (input.empty()) return;
    const std::size_t segment = 1 + input[0] % 64;
    const auto stream = std::as_bytes(input.subspan(1));
    rtos::net::HttpRequestParser<512> p;
    using S = rtos::net::HttpRequestParser<512>::State;
    std::size_t pos = 0;
    for (int rounds = 0; rounds < 4096; ++rounds) {
        const auto space = p.free_space();
        const std::size_t n = std::min({stream.size() - pos, space.size(), segment});
        std::memcpy(space.data(), stream.data() + pos, n);
        pos += n;
        const S st = p.commit(n);
        if (st == S::Complete) {
            (void)p.header("host");
            (void)p.header("sec-websocket-key");
            (void)p.keep_alive();
            if (p.method().size() + p.target().size() + p.version().size() > 512) __builtin_trap();
            p.next();
        } else if (st != S::Incomplete || n == 0) {
            break;
        }
    }
}
