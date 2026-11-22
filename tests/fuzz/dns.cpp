// SPDX-License-Identifier: MIT
// AFL++ harness: DNS name handling (rtos/net/dns.hpp: compression pointers, label lengths) as the
// resolver and the mDNS responder use it. Input: byte 0 = the name's offset (modulo the message
// size + 2), the rest = the DNS message.
#include <array>

#include "fuzz_main.hpp"
#include "rtos/net/dns.hpp"

namespace dns = rtos::net::dns;

void fuzz_one(std::span<const std::uint8_t> input) {
    if (input.empty()) return;
    const auto msg = std::as_bytes(input.subspan(1));
    const std::size_t at = input[0] % (msg.size() + 2);
    if (const auto end = dns::skip_name(msg, at); end && *end > msg.size()) __builtin_trap();
    if (const auto end = dns::match_name(msg, at, "fcu.local"); end && *end > msg.size()) __builtin_trap();
    std::array<char, 128> out{};
    if (const auto len = dns::read_name(msg, at, out); len && *len > out.size()) __builtin_trap();
}
