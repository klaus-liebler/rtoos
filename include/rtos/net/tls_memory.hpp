// SPDX-License-Identifier: MIT
//
// High-water marks of wolfSSL's static memory pool (WOLFSSL_STATIC_MEMORY): wolfSSL tracks
// no peaks itself, so a callback (WOLFSSL_STATIC_MEMORY_DEBUG_CALLBACK) counts every bucket
// as it is created, taken and returned. Used to size the pool from measurements (fcu_net
// prints the marks every 10 s) instead of guesses.
//
// The callback is global: start the tracking before the pools are loaded (their buckets are
// counted while loading), and it covers every pool there is -- fcu_net has one, shared by TLS
// and OPC UA. Network thread only (wolfSSL is only used there).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rtos::net {

struct TlsPoolBucket {
    std::uint32_t size = 0;   ///< bytes per block
    std::uint32_t total = 0;  ///< blocks in the pool
    std::uint32_t used = 0;   ///< blocks taken now
    std::uint32_t peak = 0;   ///< most blocks taken at once since the last reset
    std::uint32_t largest = 0;  ///< largest request served from this bucket
};

struct TlsPoolStats {
    static constexpr std::size_t max_buckets = 16;
    std::array<TlsPoolBucket, max_buckets> buckets{};
    std::size_t count = 0;            ///< buckets in use of `buckets`
    std::uint32_t total_bytes = 0;    ///< sum of all blocks
    std::uint32_t used_bytes = 0;     ///< blocks taken now (bucket sizes, not request sizes)
    std::uint32_t peak_bytes = 0;     ///< most bytes taken at once
    std::uint32_t failures = 0;       ///< allocations the pool could not serve (MEMORY_E)
};

/// Count from now on; call before the pools are loaded.
void tls_pool_tracking_start() noexcept;
const TlsPoolStats& tls_pool_stats() noexcept;
/// Start new peaks at the current use (e.g. after start-up).
void tls_pool_reset_peaks() noexcept;

}  // namespace rtos::net
