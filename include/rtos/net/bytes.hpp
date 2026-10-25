// SPDX-License-Identifier: MIT
//
// Network byte order access on raw packet memory. Byte-wise, so headers can sit at
// any alignment (an IPv4 header follows the 14-byte Ethernet header).
#pragma once

#include <cstddef>
#include <cstdint>

namespace rtos::net {

[[nodiscard]] constexpr std::uint16_t load_be16(const std::byte* p) noexcept {
    return static_cast<std::uint16_t>((std::to_integer<std::uint16_t>(p[0]) << 8) | std::to_integer<std::uint16_t>(p[1]));
}

[[nodiscard]] constexpr std::uint32_t load_be32(const std::byte* p) noexcept {
    return (std::to_integer<std::uint32_t>(p[0]) << 24) | (std::to_integer<std::uint32_t>(p[1]) << 16) |
           (std::to_integer<std::uint32_t>(p[2]) << 8) | std::to_integer<std::uint32_t>(p[3]);
}

constexpr void store_be16(std::byte* p, std::uint16_t v) noexcept {
    p[0] = static_cast<std::byte>(v >> 8);
    p[1] = static_cast<std::byte>(v);
}

constexpr void store_be32(std::byte* p, std::uint32_t v) noexcept {
    p[0] = static_cast<std::byte>(v >> 24);
    p[1] = static_cast<std::byte>(v >> 16);
    p[2] = static_cast<std::byte>(v >> 8);
    p[3] = static_cast<std::byte>(v);
}

/// Sequence-number arithmetic modulo 2^32 (RFC 793 / RFC 1982).
[[nodiscard]] constexpr bool seq_lt(std::uint32_t a, std::uint32_t b) noexcept {
    return static_cast<std::int32_t>(a - b) < 0;
}
[[nodiscard]] constexpr bool seq_le(std::uint32_t a, std::uint32_t b) noexcept {
    return static_cast<std::int32_t>(a - b) <= 0;
}

}  // namespace rtos::net
