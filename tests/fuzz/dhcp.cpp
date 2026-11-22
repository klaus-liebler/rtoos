// SPDX-License-Identifier: MIT
// AFL++ harness: the DHCP options parser (rtos/net/dhcp.hpp) - client and server read the options
// of every DHCP message with it. Input: the options area behind the magic cookie.
#include "fuzz_main.hpp"
#include "rtos/net/dhcp.hpp"

void fuzz_one(std::span<const std::uint8_t> input) {
    rtos::net::dhcp::Options o{};
    (void)rtos::net::dhcp::parse_options(std::as_bytes(input), o);
}
