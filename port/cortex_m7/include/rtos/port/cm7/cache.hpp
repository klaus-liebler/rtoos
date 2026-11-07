// SPDX-License-Identifier: MIT
//
// Cortex-M7 L1 caches: enable, and the maintenance a DMA buffer in cacheable memory needs.
// Registers, bits and sequences: ARM CMSIS 5.9.0 core_cm7.h and cachel1_armv7.h
// (SCB_EnableICache, SCB_EnableDCache, SCB_CleanDCache_by_Addr, SCB_InvalidateDCache_by_Addr).
//
//   cache::enable_icache();
//   cache::enable_dcache();
//   cache::clean(tx, sizeof tx);        // before a DMA reads memory the CPU wrote
//   cache::invalidate(rx, sizeof rx);   // after a DMA wrote memory the CPU will read
//
// The data cache line is 32 bytes: a buffer that is invalidated must be 32-byte aligned and a
// multiple of 32 bytes long, or the invalidate throws away the CPU's writes to its neighbours.
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/cortex_m/registers.hpp"

namespace rtos::port::cm7::cache {

namespace scb {
using CCR = cortex_m::Reg<0xE000'ED14>;       // Configuration and Control
using CCSIDR = cortex_m::Reg<0xE000'ED80>;    // Cache Size ID (of the cache CSSELR selects)
using CSSELR = cortex_m::Reg<0xE000'ED84>;    // Cache Size Selection
using ICIALLU = cortex_m::Reg<0xE000'EF50>;   // I-cache invalidate all to PoU
using DCIMVAC = cortex_m::Reg<0xE000'EF5C>;   // D-cache invalidate by address to PoC
using DCISW = cortex_m::Reg<0xE000'EF60>;     // D-cache invalidate by set/way
using DCCMVAC = cortex_m::Reg<0xE000'EF68>;   // D-cache clean by address to PoC
using DCCIMVAC = cortex_m::Reg<0xE000'EF70>;  // D-cache clean and invalidate by address to PoC
inline constexpr std::uint32_t ccr_dc = 1u << 16, ccr_ic = 1u << 17;
inline constexpr std::uint32_t ccsidr_sets_pos = 13, ccsidr_sets_msk = 0x7FFFu;
inline constexpr std::uint32_t ccsidr_ways_pos = 3, ccsidr_ways_msk = 0x3FFu;
inline constexpr std::uint32_t dcisw_set_pos = 5, dcisw_set_msk = 0x1FFu << dcisw_set_pos;
inline constexpr std::uint32_t dcisw_way_pos = 30, dcisw_way_msk = 3u << dcisw_way_pos;
}  // namespace scb

/// The data cache line (fixed on the Cortex-M7).
inline constexpr std::size_t line_bytes = 32;

inline void barrier() noexcept { __asm volatile("dsb \n isb" ::: "memory"); }

[[nodiscard]] inline bool icache_enabled() noexcept { return (scb::CCR::read() & scb::ccr_ic) != 0; }
[[nodiscard]] inline bool dcache_enabled() noexcept { return (scb::CCR::read() & scb::ccr_dc) != 0; }

/// Invalidate and enable the instruction cache (no-op if it is on).
inline void enable_icache() noexcept {
    if (icache_enabled()) {
        return;
    }
    barrier();
    scb::ICIALLU::write(0);
    barrier();
    scb::CCR::set(scb::ccr_ic);
    barrier();
}

/// Invalidate the whole data cache by set/way and enable it (no-op if it is on). Call before
/// any data that must survive is written through the cache (the invalidate discards lines).
inline void enable_dcache() noexcept {
    if (dcache_enabled()) {
        return;
    }
    scb::CSSELR::write(0);  // level 1 data cache
    __asm volatile("dsb" ::: "memory");
    const std::uint32_t ccsidr = scb::CCSIDR::read();
    std::uint32_t sets = (ccsidr >> scb::ccsidr_sets_pos) & scb::ccsidr_sets_msk;
    do {
        std::uint32_t ways = (ccsidr >> scb::ccsidr_ways_pos) & scb::ccsidr_ways_msk;
        do {
            scb::DCISW::write(((sets << scb::dcisw_set_pos) & scb::dcisw_set_msk) |
                              ((ways << scb::dcisw_way_pos) & scb::dcisw_way_msk));
        } while (ways-- != 0);
    } while (sets-- != 0);
    __asm volatile("dsb" ::: "memory");
    scb::CCR::set(scb::ccr_dc);
    barrier();
}

namespace detail {
template <class Op>
inline void by_address(const void* address, std::size_t bytes) noexcept {
    if (bytes == 0) {
        return;
    }
    auto line = reinterpret_cast<std::uintptr_t>(address) & ~(line_bytes - 1);
    const auto end = reinterpret_cast<std::uintptr_t>(address) + bytes;
    __asm volatile("dsb" ::: "memory");
    for (; line < end; line += line_bytes) {
        Op::write(static_cast<std::uint32_t>(line));
    }
    barrier();
}
}  // namespace detail

/// Write the cached lines of [address, address + bytes) back to memory (before a DMA reads it).
inline void clean(const void* address, std::size_t bytes) noexcept {
    detail::by_address<scb::DCCMVAC>(address, bytes);
}
/// Discard the cached lines of [address, address + bytes) (after a DMA wrote it); see the
/// alignment note at the top.
inline void invalidate(const void* address, std::size_t bytes) noexcept {
    detail::by_address<scb::DCIMVAC>(address, bytes);
}
/// Clean, then discard (a buffer the DMA reads and writes).
inline void clean_invalidate(const void* address, std::size_t bytes) noexcept {
    detail::by_address<scb::DCCIMVAC>(address, bytes);
}

}  // namespace rtos::port::cm7::cache
