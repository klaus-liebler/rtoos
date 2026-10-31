// SPDX-License-Identifier: MIT
//
// Source capabilities (power data objects, USB PD 3.1 6.4.1) and the request data object a
// sink answers with (6.4.2).
//
// Ported from "USB Power Delivery for Arduino", Copyright (c) 2023 Manuel Bleichenbacher,
// MIT License, via the FCU firmware.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/usb_pd/message.hpp"

namespace rtos::usb_pd {

enum class SupplyType : std::uint8_t { Fixed = 0, Battery = 1, Variable = 2, Pps = 3 };

[[nodiscard]] constexpr const char* supply_type_name(SupplyType t) noexcept {
    switch (t) {
        case SupplyType::Fixed: return "fixed";
        case SupplyType::Battery: return "battery";
        case SupplyType::Variable: return "variable";
        case SupplyType::Pps: return "PPS";
    }
    return "?";
}

struct SourceCapability {
    SupplyType type = SupplyType::Fixed;
    std::uint8_t position = 0;  ///< object position in the message (1..7)
    std::uint16_t max_ma = 0;
    std::uint16_t min_mv = 0;
    std::uint16_t max_mv = 0;
};

/// Parse a Source_Capabilities message into `out`; returns the number of capabilities.
/// Augmented PDOs other than SPR PPS are skipped.
constexpr std::size_t parse_source_capabilities(const Message& m, std::span<SourceCapability, Message::max_objects> out) noexcept {
    std::size_t n = 0;
    for (std::size_t i = 0; i < m.object_count(); ++i) {
        const std::uint32_t pdo = m.objects[i];
        const auto type = static_cast<SupplyType>(pdo >> 30);
        SourceCapability cap{.type = type, .position = static_cast<std::uint8_t>(i + 1)};
        if (type == SupplyType::Pps) {
            if ((pdo & (3u << 28)) != 0) {
                continue;  // EPR AVS or SPR AVS, not SPR PPS
            }
            cap.max_ma = static_cast<std::uint16_t>((pdo & 0x7Fu) * 50);
            cap.min_mv = static_cast<std::uint16_t>(((pdo >> 8) & 0xFFu) * 100);
            cap.max_mv = static_cast<std::uint16_t>(((pdo >> 17) & 0xFFu) * 100);
        } else {
            cap.max_ma = static_cast<std::uint16_t>((pdo & 0x3FFu) * 10);
            cap.min_mv = static_cast<std::uint16_t>(((pdo >> 10) & 0x3FFu) * 50);
            cap.max_mv = type == SupplyType::Fixed ? cap.min_mv : static_cast<std::uint16_t>(((pdo >> 20) & 0x3FFu) * 50);
        }
        out[n++] = cap;
    }
    return n;
}

/// A request data object, and the voltage and current it effectively asks for (rounded to the
/// object's units). `object == 0`: no capability matches.
struct PowerRequest {
    std::uint32_t object = 0;
    std::size_t index = 0;  ///< into the capability list
    int mv = 0;
    int ma = 0;
    [[nodiscard]] constexpr bool valid() const noexcept { return object != 0; }
};

namespace pd_detail {
inline constexpr std::uint32_t no_usb_suspend = 1u << 24;

constexpr PowerRequest fixed_request(const SourceCapability& cap, std::size_t index, int ma) noexcept {
    std::uint32_t ma10 = static_cast<std::uint32_t>(ma + 5) / 10;
    ma10 = ma10 > 0x3FF ? 0x3FF : ma10;
    const std::uint32_t object = ((cap.position & 0x07u) << 28) | no_usb_suspend | (ma10 << 10) | ma10;
    return {object, index, cap.min_mv, static_cast<int>(ma10 * 10)};
}

constexpr PowerRequest pps_request(const SourceCapability& cap, std::size_t index, int mv, int ma) noexcept {
    std::uint32_t mv20 = static_cast<std::uint32_t>(mv + 10) / 20;
    mv20 = mv20 > 0x7FF ? 0x7FF : mv20;
    std::uint32_t ma50 = static_cast<std::uint32_t>(ma + 25) / 50;
    ma50 = ma50 > 0x7F ? 0x7F : ma50;
    const std::uint32_t object = ((cap.position & 0x07u) << 28) | no_usb_suspend | (mv20 << 9) | ma50;
    return {object, index, static_cast<int>(mv20 * 20), static_cast<int>(ma50 * 50)};
}
}  // namespace pd_detail

/// Request `mv` with at most `ma` (0: the capability's maximum current). Fixed supplies are
/// preferred; a PPS range is used when no fixed supply has the voltage.
constexpr PowerRequest power_request(std::span<const SourceCapability> caps, int mv, int ma = 0) noexcept {
    for (std::size_t i = 0; i < caps.size(); ++i) {
        const auto& cap = caps[i];
        if (cap.type == SupplyType::Fixed && mv == cap.min_mv) {
            const int current = ma == 0 ? cap.max_ma : ma;
            if (current >= 25 && current <= cap.max_ma) {
                return pd_detail::fixed_request(cap, i, current);
            }
        }
    }
    for (std::size_t i = 0; i < caps.size(); ++i) {
        const auto& cap = caps[i];
        if (cap.type == SupplyType::Pps && mv >= cap.min_mv && mv <= cap.max_mv) {
            const int current = ma == 0 ? cap.max_ma : ma;
            if (current >= 25 && current <= cap.max_ma) {
                return pd_detail::pps_request(cap, i, mv, current);
            }
        }
    }
    return {};
}

}  // namespace rtos::usb_pd
