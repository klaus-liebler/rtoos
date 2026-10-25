// SPDX-License-Identifier: MIT
//
// Minimal DNS message helpers (RFC 1035) for mDNS: name parsing with compression
// pointers (bounded), name comparison and encoding.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "rtos/net/bytes.hpp"

namespace rtos::net::dns {

inline constexpr std::size_t header_size = 12;
inline constexpr std::uint16_t type_a = 1;
inline constexpr std::uint16_t type_any = 255;
inline constexpr std::uint16_t class_in = 1;
inline constexpr std::uint16_t class_mask = 0x7FFF;   // top bit: mDNS unicast-response / cache-flush
inline constexpr std::uint16_t flag_qr = 0x8000;
inline constexpr std::uint16_t flag_aa = 0x0400;

[[nodiscard]] constexpr char lower(char c) noexcept { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c + 32) : c; }

/// Compare the (possibly compressed) name at `offset` in `msg` with the dotted
/// `expected` ("fcu.local"), case-insensitively. On success returns the offset
/// just after the name in the original position; nullopt on mismatch or a
/// malformed name.
[[nodiscard]] inline std::optional<std::size_t> match_name(std::span<const std::byte> msg, std::size_t offset,
                                                           std::string_view expected) noexcept {
    std::optional<std::size_t> end;
    std::size_t pos = offset;
    std::size_t matched = 0;  // chars of `expected` consumed
    for (int hops = 0; hops < 16; ++hops) {
        if (pos >= msg.size()) {
            return std::nullopt;
        }
        const auto len = std::to_integer<std::size_t>(msg[pos]);
        if ((len & 0xC0) == 0xC0) {  // compression pointer
            if (pos + 1 >= msg.size()) {
                return std::nullopt;
            }
            if (!end) {
                end = pos + 2;
            }
            pos = ((len & 0x3F) << 8) | std::to_integer<std::size_t>(msg[pos + 1]);
            continue;
        }
        if (len == 0) {
            if (!end) {
                end = pos + 1;
            }
            return matched == expected.size() ? end : std::nullopt;
        }
        if (len > 63 || pos + 1 + len > msg.size()) {
            return std::nullopt;
        }
        if (matched != 0) {  // label separator
            if (matched >= expected.size() || expected[matched] != '.') {
                return std::nullopt;
            }
            ++matched;
        }
        if (matched + len > expected.size()) {
            return std::nullopt;
        }
        for (std::size_t i = 0; i < len; ++i) {
            if (lower(static_cast<char>(msg[pos + 1 + i])) != lower(expected[matched + i])) {
                return std::nullopt;
            }
        }
        matched += len;
        pos += 1 + len;
    }
    return std::nullopt;  // pointer loop
}

/// Offset just after the (possibly compressed) name at `offset`, or nullopt.
[[nodiscard]] inline std::optional<std::size_t> skip_name(std::span<const std::byte> msg, std::size_t offset) noexcept {
    std::size_t pos = offset;
    while (pos < msg.size()) {
        const auto len = std::to_integer<std::size_t>(msg[pos]);
        if ((len & 0xC0) == 0xC0) {
            return pos + 2 <= msg.size() ? std::optional(pos + 2) : std::nullopt;
        }
        if (len == 0) {
            return pos + 1;
        }
        pos += 1 + len;
    }
    return std::nullopt;
}

/// Decode the (possibly compressed) name at `offset` into dotted form ("www.example.com",
/// no trailing dot, lower case). Returns its length, or nullopt if the name is malformed,
/// loops, or does not fit into `out`.
[[nodiscard]] inline std::optional<std::size_t> read_name(std::span<const std::byte> msg, std::size_t offset,
                                                          std::span<char> out) noexcept {
    std::size_t pos = offset;
    std::size_t n = 0;
    int jumps = 0;
    while (pos < msg.size()) {
        const auto len = std::to_integer<std::size_t>(msg[pos]);
        if ((len & 0xC0) == 0xC0) {
            if (pos + 1 >= msg.size() || ++jumps > 16) {
                return std::nullopt;
            }
            pos = ((len & 0x3F) << 8) | std::to_integer<std::size_t>(msg[pos + 1]);
            continue;
        }
        if (len == 0) {
            return n;
        }
        if (len > 63 || pos + 1 + len > msg.size() || n + (n != 0) + len > out.size()) {
            return std::nullopt;
        }
        if (n != 0) {
            out[n++] = '.';
        }
        for (std::size_t i = 0; i < len; ++i) {
            out[n++] = lower(static_cast<char>(msg[pos + 1 + i]));
        }
        pos += 1 + len;
    }
    return std::nullopt;
}

/// Encode a dotted name as labels. Returns the bytes written, 0 if it doesn't fit.
[[nodiscard]] inline std::size_t write_name(std::byte* out, std::size_t capacity, std::string_view name) noexcept {
    std::size_t n = 0;
    while (!name.empty()) {
        const std::size_t dot = name.find('.');
        const std::string_view label = name.substr(0, dot);
        if (label.empty() || label.size() > 63 || n + 1 + label.size() + 1 > capacity) {
            return 0;
        }
        out[n++] = static_cast<std::byte>(label.size());
        for (char c : label) {
            out[n++] = static_cast<std::byte>(c);
        }
        name = dot == std::string_view::npos ? std::string_view{} : name.substr(dot + 1);
    }
    out[n++] = std::byte{0};
    return n;
}

}  // namespace rtos::net::dns
