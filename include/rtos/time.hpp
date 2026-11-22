// SPDX-License-Identifier: MIT
//
// Wrap-around safe comparisons of 32-bit points in time (kernel ticks, milliseconds): correct
// while the two are less than 2^31 apart (24.8 days in ms). Never compare such times with < or
// >= directly - after the wrap a deadline in the future looks long past.
#pragma once

#include <cstdint>

namespace rtos {

/// `now` is at or after `deadline`.
[[nodiscard]] constexpr bool reached(std::uint32_t now, std::uint32_t deadline) noexcept {
    return static_cast<std::int32_t>(now - deadline) >= 0;
}

/// `a` lies before `b`.
[[nodiscard]] constexpr bool earlier(std::uint32_t a, std::uint32_t b) noexcept {
    return static_cast<std::int32_t>(a - b) < 0;
}

}  // namespace rtos
