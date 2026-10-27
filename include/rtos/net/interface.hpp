// SPDX-License-Identifier: MIT
//
// Interface identifiers and sets: which interfaces a server, socket or listener
// serves. Default everywhere: all interfaces; otherwise a list, e.g.
//     https.start(443);                                         // all interfaces
//     modbus.listen(502, InterfaceSet::of({eth}));              // Ethernet only
//     DhcpServer{stack, usb, ...}                               // one interface
#pragma once

#include <cstdint>
#include <initializer_list>

namespace rtos::net {

using InterfaceId = std::uint8_t;
/// For sending: "route it" (the stack picks the interface).
inline constexpr InterfaceId any_interface = 0xFF;

class InterfaceSet {
public:
    static constexpr std::uint8_t capacity = 32;

    [[nodiscard]] static constexpr InterfaceSet all() noexcept { return InterfaceSet{0xFFFF'FFFFu}; }
    [[nodiscard]] static constexpr InterfaceSet none() noexcept { return InterfaceSet{0}; }
    [[nodiscard]] static constexpr InterfaceSet of(std::initializer_list<InterfaceId> ids) noexcept {
        std::uint32_t bits = 0;
        for (InterfaceId id : ids) {
            bits |= id < capacity ? 1u << id : 0u;
        }
        return InterfaceSet{bits};
    }

    [[nodiscard]] constexpr bool contains(InterfaceId id) const noexcept {
        return id < capacity && ((bits_ >> id) & 1u) != 0;
    }
    [[nodiscard]] constexpr bool intersects(InterfaceSet other) const noexcept { return (bits_ & other.bits_) != 0; }
    [[nodiscard]] constexpr bool empty() const noexcept { return bits_ == 0; }
    friend constexpr bool operator==(InterfaceSet, InterfaceSet) noexcept = default;

private:
    constexpr explicit InterfaceSet(std::uint32_t bits) noexcept : bits_(bits) {}
    std::uint32_t bits_;
};

}  // namespace rtos::net
