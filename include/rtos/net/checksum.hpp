// SPDX-License-Identifier: MIT
//
// Internet checksum (RFC 1071). Accumulates 16-bit big-endian words into a 32-bit
// sum (carries are folded once at the end), so the loop has no branch per word.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/net/bytes.hpp"

namespace rtos::net {

/// Unfolded one's-complement sum of `data`, continuing from `sum`.
[[nodiscard]] constexpr std::uint32_t checksum_add(std::span<const std::byte> data, std::uint32_t sum = 0) noexcept {
    const std::byte* p = data.data();
    std::size_t n = data.size();
    while (n >= 8) {
        sum += load_be16(p) + load_be16(p + 2) + load_be16(p + 4) + load_be16(p + 6);
        p += 8;
        n -= 8;
    }
    while (n >= 2) {
        sum += load_be16(p);
        p += 2;
        n -= 2;
    }
    if (n != 0) {
        sum += std::to_integer<std::uint32_t>(p[0]) << 8;  // odd length: pad with zero
    }
    return sum;
}

/// Fold and complement: the value to store in a header checksum field.
[[nodiscard]] constexpr std::uint16_t checksum_finish(std::uint32_t sum) noexcept {
    while ((sum >> 16) != 0) {
        sum = (sum & 0xFFFFu) + (sum >> 16);
    }
    return static_cast<std::uint16_t>(~sum);
}

[[nodiscard]] constexpr std::uint16_t checksum(std::span<const std::byte> data) noexcept {
    return checksum_finish(checksum_add(data));
}

}  // namespace rtos::net
