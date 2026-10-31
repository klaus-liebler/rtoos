// SPDX-License-Identifier: MIT
//
// WS2812 LEDs on a PWM timer channel: one timer period per bit at 800 kHz, the compare value
// sets the high time (0: ~0.35 of the period, 1: ~0.7, WS2812B datasheet T0H/T1H), a DMA
// request per period loads the next value. Colours are 0xRRGGBB, sent G, R, B, MSB first; a
// final 0 keeps the line low (reset).
//
//   constexpr Ws2812Timing timing = ws2812_timing(board::clocks.timer2_hz());
//   std::array<std::uint16_t, ws2812_slots(8)> slots;
//   ws2812_encode(colours, slots, timing);          // then DMA slots into the CCR
//
// Ported from the FCU firmware's libs/hw/ws2812.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace rtos::devices {

struct Ws2812Timing {
    std::uint32_t period_ticks = 0;  ///< timer ticks per bit (ARR + 1)
    std::uint16_t zero = 0, one = 0; ///< compare values
};

/// Timing for a timer counting at `timer_hz` (no prescaler).
[[nodiscard]] constexpr Ws2812Timing ws2812_timing(std::uint32_t timer_hz) noexcept {
    const std::uint32_t period = timer_hz / 800'000;
    return {period, static_cast<std::uint16_t>(period * 35 / 100), static_cast<std::uint16_t>(period * 70 / 100)};
}

/// DMA slots for `leds` LEDs: 24 bits each plus the reset slot.
[[nodiscard]] constexpr std::size_t ws2812_slots(std::size_t leds) noexcept { return leds * 24 + 1; }

/// Compare values for `colours` into `slots` (at least ws2812_slots(colours.size())).
constexpr void ws2812_encode(std::span<const std::uint32_t> colours, std::span<std::uint16_t> slots,
                             const Ws2812Timing& t) noexcept {
    std::size_t i = 0;
    for (const std::uint32_t rgb : colours) {
        const std::uint32_t grb = ((rgb >> 8) & 0xFF) << 16 | ((rgb >> 16) & 0xFF) << 8 | (rgb & 0xFF);
        for (int bit = 23; bit >= 0; --bit) {
            slots[i++] = (grb >> bit) & 1u ? t.one : t.zero;
        }
    }
    slots[i] = 0;
}

}  // namespace rtos::devices
