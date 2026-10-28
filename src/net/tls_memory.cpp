// SPDX-License-Identifier: MIT
#include "rtos/net/tls_memory.hpp"

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/memory.h>

#include <algorithm>

namespace rtos::net {

namespace {

TlsPoolStats g_pool;

TlsPoolBucket* bucket(std::uint32_t size) noexcept {
    for (std::size_t i = 0; i < g_pool.count; ++i) {
        if (g_pool.buckets[i].size == size) {
            return &g_pool.buckets[i];
        }
    }
    if (g_pool.count == g_pool.buckets.size()) {
        return nullptr;
    }
    TlsPoolBucket& b = g_pool.buckets[g_pool.count++];
    b.size = size;
    // ascending sizes, as wolfSSL's buckets are
    std::sort(g_pool.buckets.begin(), g_pool.buckets.begin() + static_cast<std::ptrdiff_t>(g_pool.count),
              [](const TlsPoolBucket& a, const TlsPoolBucket& c) { return a.size < c.size; });
    for (std::size_t i = 0; i < g_pool.count; ++i) {
        if (g_pool.buckets[i].size == size) {
            return &g_pool.buckets[i];
        }
    }
    return nullptr;
}

void on_memory(std::size_t request, int bucket_size, byte state, int) {
    const auto size = static_cast<std::uint32_t>(bucket_size);
    switch (state) {
        case WOLFSSL_DEBUG_MEMORY_INIT:
            if (TlsPoolBucket* b = bucket(size)) {
                ++b->total;
                g_pool.total_bytes += size;
            }
            break;
        case WOLFSSL_DEBUG_MEMORY_ALLOC:
            if (TlsPoolBucket* b = bucket(size)) {
                b->peak = std::max(b->peak, ++b->used);
                b->largest = std::max(b->largest, static_cast<std::uint32_t>(request));
                g_pool.used_bytes += size;
                g_pool.peak_bytes = std::max(g_pool.peak_bytes, g_pool.used_bytes);
            }
            break;
        case WOLFSSL_DEBUG_MEMORY_FREE:
            if (TlsPoolBucket* b = bucket(size); b != nullptr && b->used > 0) {
                --b->used;
                g_pool.used_bytes -= size;
            }
            break;
        case WOLFSSL_DEBUG_MEMORY_FAIL: ++g_pool.failures; break;
        default: break;
    }
}

}  // namespace

void tls_pool_tracking_start() noexcept {
    g_pool = TlsPoolStats{};
    wolfSSL_SetDebugMemoryCb(&on_memory);
}

const TlsPoolStats& tls_pool_stats() noexcept { return g_pool; }

void tls_pool_reset_peaks() noexcept {
    g_pool.peak_bytes = g_pool.used_bytes;
    for (std::size_t i = 0; i < g_pool.count; ++i) {
        g_pool.buckets[i].peak = g_pool.buckets[i].used;
    }
}

}  // namespace rtos::net
