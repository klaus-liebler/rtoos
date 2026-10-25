// SPDX-License-Identifier: MIT
//
// Compile-time kernel configuration. An application passes its own config type to
// `rtos::Kernel<Port, Config>`; the `KernelConfig` concept rejects invalid values at
// compile time, so no configuration error can survive to run time.
#pragma once

#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>

namespace rtos {

using Tick = std::uint32_t;

/// Timeout value: block until the event arrives.
inline constexpr Tick wait_forever = 0xFFFF'FFFFu;
/// Timeout value: poll, never block.
inline constexpr Tick no_wait = 0;

struct DefaultConfig {
    /// Maximum number of application threads (the kernel-owned idle thread is extra).
    static constexpr std::size_t max_threads = 16;
    /// Number of priority levels. 0 is the most urgent; the least urgent level
    /// (`priority_levels - 1`) is reserved for the idle thread.
    static constexpr std::size_t priority_levels = 32;
    /// Task-notification slots per thread (FreeRTOS: configTASK_NOTIFICATION_ARRAY_ENTRIES).
    static constexpr std::size_t notify_slots = 4;
    /// Buckets of the timeout wheel. Power of two; more buckets = fewer threads per bucket.
    static constexpr std::size_t timer_wheel_slots = 32;
    /// Stack of the kernel-owned idle thread in bytes.
    static constexpr std::size_t idle_stack_bytes = 512;
};

template <class C>
concept KernelConfig = requires {
    { C::max_threads } -> std::convertible_to<std::size_t>;
    { C::priority_levels } -> std::convertible_to<std::size_t>;
    { C::notify_slots } -> std::convertible_to<std::size_t>;
    { C::timer_wheel_slots } -> std::convertible_to<std::size_t>;
    { C::idle_stack_bytes } -> std::convertible_to<std::size_t>;
}
    // One 32-bit word holds the ready bitmap -> O(1) via a single CLZ/CTZ.
    && (C::priority_levels >= 2 && C::priority_levels <= 32)
    // Thread ids are uint8_t; id 0xFF is never handed out.
    && (C::max_threads >= 1 && C::max_threads + 1 < 0xFF)
    && (C::notify_slots >= 1 && C::notify_slots <= 32)
    && std::has_single_bit(C::timer_wheel_slots)
    && (C::idle_stack_bytes >= 256);

}  // namespace rtos
