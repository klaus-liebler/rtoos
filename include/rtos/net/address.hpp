// SPDX-License-Identifier: MIT
#pragma once

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>

#include "rtos/net/bytes.hpp"

namespace rtos::net {

struct MacAddress {
    std::array<std::byte, 6> octets{};

    static constexpr MacAddress from(std::uint8_t a, std::uint8_t b, std::uint8_t c, std::uint8_t d,
                                     std::uint8_t e, std::uint8_t f) noexcept {
        return {{std::byte{a}, std::byte{b}, std::byte{c}, std::byte{d}, std::byte{e}, std::byte{f}}};
    }
    static constexpr MacAddress broadcast() noexcept { return from(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF); }

    void store(std::byte* p) const noexcept {
        for (std::size_t i = 0; i < 6; ++i) {
            p[i] = octets[i];
        }
    }
    static MacAddress load(const std::byte* p) noexcept {
        MacAddress m;
        for (std::size_t i = 0; i < 6; ++i) {
            m.octets[i] = p[i];
        }
        return m;
    }
    friend constexpr bool operator==(const MacAddress&, const MacAddress&) noexcept = default;
};

/// IPv4 address in host byte order.
struct Ipv4Address {
    std::uint32_t value = 0;

    static constexpr Ipv4Address from(std::uint8_t a, std::uint8_t b, std::uint8_t c, std::uint8_t d) noexcept {
        return {(std::uint32_t{a} << 24) | (std::uint32_t{b} << 16) | (std::uint32_t{c} << 8) | d};
    }
    static constexpr Ipv4Address any() noexcept { return {0}; }
    static constexpr Ipv4Address broadcast() noexcept { return {0xFFFF'FFFFu}; }

    [[nodiscard]] constexpr bool is_multicast() const noexcept { return (value >> 28) == 0xE; }
    [[nodiscard]] constexpr bool is_broadcast() const noexcept { return value == 0xFFFF'FFFFu; }

    /// Ethernet multicast MAC for this group (RFC 1112: 01:00:5E + low 23 bits).
    [[nodiscard]] constexpr MacAddress multicast_mac() const noexcept {
        return MacAddress::from(0x01, 0x00, 0x5E, static_cast<std::uint8_t>((value >> 16) & 0x7F),
                                static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value));
    }

    void store(std::byte* p) const noexcept { store_be32(p, value); }
    static Ipv4Address load(const std::byte* p) noexcept { return {load_be32(p)}; }

    friend constexpr auto operator<=>(const Ipv4Address&, const Ipv4Address&) noexcept = default;
};

/// Static interface configuration (DHCP can fill the same structure later).
struct Ipv4Config {
    Ipv4Address address{};
    Ipv4Address netmask{};
    Ipv4Address gateway{};

    [[nodiscard]] constexpr bool on_link(Ipv4Address a) const noexcept {
        return (a.value & netmask.value) == (address.value & netmask.value);
    }
    [[nodiscard]] constexpr Ipv4Address subnet_broadcast() const noexcept {
        return {address.value | ~netmask.value};
    }
};

}  // namespace rtos::net
