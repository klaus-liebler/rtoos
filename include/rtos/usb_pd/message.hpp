// SPDX-License-Identifier: MIT
//
// USB Power Delivery messages (USB PD 3.1, chapter 6.2).
//
// Ported from "USB Power Delivery for Arduino", Copyright (c) 2023 Manuel Bleichenbacher,
// MIT License (https://github.com/manuelbl/usb-pd-arduino), via the FCU firmware.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

namespace rtos::usb_pd {

/// Message type: the 5-bit type, plus 0x80 for data messages (at least one data object).
enum class MessageType : std::uint8_t {
    GoodCrc = 0x01,
    GotoMin = 0x02,
    Accept = 0x03,
    Reject = 0x04,
    Ping = 0x05,
    PsReady = 0x06,
    GetSourceCap = 0x07,
    GetSinkCap = 0x08,
    DrSwap = 0x09,
    PrSwap = 0x0A,
    VconnSwap = 0x0B,
    Wait = 0x0C,
    SoftReset = 0x0D,
    NotSupported = 0x10,
    SourceCapabilities = 0x81,
    Request = 0x82,
    Bist = 0x83,
    SinkCapabilities = 0x84,
    VendorDefined = 0x8F,
};

/// Start-of-packet ordered set.
enum class Sop : std::uint8_t { Invalid, Sop, SopPrime, SopDoublePrime, SopPrimeDebug, SopDoublePrimeDebug, CableReset };

/// One message: 16-bit header and up to 7 data objects. `header` and `objects` are
/// contiguous, so payload() is the wire format (little endian) as the PHY sends it.
struct Message {
    static constexpr std::size_t max_objects = 7;
    static constexpr std::size_t max_payload = 2 + 4 * max_objects;  ///< 30 bytes

    Sop sop = Sop::Sop;
    std::uint8_t cc = 0;  ///< CC line it was received on (1, 2)
    std::uint16_t header = 0;
    std::array<std::uint32_t, max_objects> objects{};

    [[nodiscard]] constexpr std::size_t object_count() const noexcept { return (header >> 12) & 0x07u; }
    [[nodiscard]] constexpr MessageType type() const noexcept {
        return static_cast<MessageType>((object_count() != 0 ? 0x80u : 0u) | (header & 0x1Fu));
    }
    [[nodiscard]] constexpr unsigned message_id() const noexcept { return (header >> 9) & 0x07u; }
    /// Specification revision field (0: 1.0, 1: 2.0, 2: 3.x).
    [[nodiscard]] constexpr unsigned spec_revision() const noexcept { return (header >> 6) & 0x03u; }
    [[nodiscard]] constexpr bool extended() const noexcept { return (header & 0x8000u) != 0; }
    [[nodiscard]] constexpr std::size_t payload_size() const noexcept { return 2 + 4 * object_count(); }

    /// Header for a message from a sink (power role sink, data role UFP).
    constexpr void init(MessageType t, std::size_t count, unsigned revision) noexcept {
        sop = Sop::Sop;
        header = static_cast<std::uint16_t>(((count & 0x07u) << 12) | (static_cast<unsigned>(t) & 0x1Fu) |
                                            ((revision & 0x03u) << 6));
    }
    constexpr void set_message_id(unsigned id) noexcept {
        header = static_cast<std::uint16_t>((header & ~0x0E00u) | ((id & 0x07u) << 9));
    }

    /// The wire bytes (header, then the data objects).
    [[nodiscard]] std::span<const std::byte> payload() const noexcept {
        return {reinterpret_cast<const std::byte*>(&header), payload_size()};
    }
    /// Header and objects from received wire bytes; false if too short for its header.
    bool assign(std::span<const std::byte> bytes) noexcept {
        if (bytes.size() < 2) {
            return false;
        }
        std::memcpy(&header, bytes.data(), 2);
        if (bytes.size() < payload_size()) {
            return false;
        }
        std::memcpy(objects.data(), bytes.data() + 2, payload_size() - 2);
        return true;
    }
};
static_assert(offsetof(Message, objects) == offsetof(Message, header) + 2, "payload must be contiguous");

}  // namespace rtos::usb_pd
