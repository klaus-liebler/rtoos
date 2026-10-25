// SPDX-License-Identifier: MIT
//
// STM32H5 cache control (RM0481: ICACHE and DCACHE chapters).
//
// Cache topology of the STM32H563 - important for DMA correctness:
//   * The Cortex-M33 core itself has NO L1 cache.
//   * ICACHE sits on the C-AHB bus (code region 0x0000_0000-0x1FFF_FFFF: internal
//     flash, and external memory remapped there). It caches *instruction and data*
//     reads from that region -> invalidate it after flash programming.
//   * DCACHE1 sits on the S-AHB bus in front of the external memories (FMC,
//     OCTOSPI). Internal SRAM (0x2000_0000) is not cached.
//
// Consequence for DMA: buffers in internal SRAM are coherent without maintenance.
// Buffers in external RAM need either an MPU non-cacheable region or explicit
// clean/invalidate via DCache::clean()/invalidate() below. Cacheability is
// signalled to both caches through the MPU memory attributes (AHB HPROT).
//
// Register offsets/bits: ST SVD STM32H563 v1.8 (verified), RM0481 ICACHE/DCACHE.
// Base addresses follow the security state the code is built for.
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32h5/chip.hpp"

namespace rtos::port::stm32h5 {

inline constexpr std::size_t cache_line_bytes = 16;  // both caches: 128-bit lines

class ICache {
public:
    enum class Ways : std::uint32_t { DirectMapped = 0, TwoWay = 1 };

    /// Enable (after a full invalidation). Must be called with the cache disabled.
    static void enable(Ways ways = Ways::TwoWay) noexcept {
        disable();
        CR::write((static_cast<std::uint32_t>(ways) << 2) & cr_waysel);
        invalidate();
        CR::set(cr_en);
    }

    static void disable() noexcept {
        CR::clear(cr_en);
        while ((SR::read() & sr_busyf) != 0) {
        }
    }

    /// Full invalidation, e.g. after programming flash. Blocks ~ a few hundred cycles.
    static void invalidate() noexcept {
        CR::set(cr_cacheinv);
        while ((SR::read() & sr_busyf) != 0) {
        }
        FCR::write(fcr_cbsyendf);
    }

private:
    static constexpr std::uintptr_t base = periph_alias + 0x0003'0400;
    using CR = cm33::Reg<base + 0x00>;
    using SR = cm33::Reg<base + 0x04>;
    using FCR = cm33::Reg<base + 0x0C>;
    static constexpr std::uint32_t cr_en = 1u << 0;
    static constexpr std::uint32_t cr_cacheinv = 1u << 1;
    static constexpr std::uint32_t cr_waysel = 1u << 2;
    static constexpr std::uint32_t sr_busyf = 1u << 0;
    static constexpr std::uint32_t fcr_cbsyendf = 1u << 1;
};

class DCache {
public:
    static void enable() noexcept {
        invalidate_all();
        CR::set(cr_en);
    }

    static void invalidate_all() noexcept {
        CR::set(cr_cacheinv);
        while ((SR::read() & sr_busyf) != 0) {
        }
        FCR::write(fcr_cbsyendf);
    }

    /// Write back dirty lines covering [addr, addr+size): before a DMA *reads* the buffer.
    static void clean(const void* addr, std::size_t size) noexcept { range_command(cmd_clean, addr, size); }

    /// Drop lines covering [addr, addr+size): after a DMA *wrote* the buffer.
    /// The buffer must be cache-line aligned and padded (see DmaBuffer) - otherwise
    /// unrelated data sharing the first/last line is lost.
    static void invalidate(const void* addr, std::size_t size) noexcept {
        range_command(cmd_invalidate, addr, size);
    }

    static void clean_invalidate(const void* addr, std::size_t size) noexcept {
        range_command(cmd_clean_invalidate, addr, size);
    }

private:
    static constexpr std::uintptr_t base = periph_alias + 0x0003'1400;
    using CR = cm33::Reg<base + 0x00>;
    using SR = cm33::Reg<base + 0x04>;
    using FCR = cm33::Reg<base + 0x0C>;
    using CMDRSADDRR = cm33::Reg<base + 0x28>;
    using CMDREADDRR = cm33::Reg<base + 0x2C>;
    static constexpr std::uint32_t cr_en = 1u << 0;
    static constexpr std::uint32_t cr_cacheinv = 1u << 1;
    static constexpr std::uint32_t cr_cachecmd_pos = 8;
    static constexpr std::uint32_t cr_cachecmd_msk = 7u << cr_cachecmd_pos;
    static constexpr std::uint32_t cr_startcmd = 1u << 11;
    static constexpr std::uint32_t sr_busyf = 1u << 0;
    static constexpr std::uint32_t sr_busycmdf = 1u << 3;
    static constexpr std::uint32_t fcr_cbsyendf = 1u << 1;
    static constexpr std::uint32_t fcr_ccmdendf = 1u << 4;
    static constexpr std::uint32_t cmd_clean = 1;
    static constexpr std::uint32_t cmd_invalidate = 2;
    static constexpr std::uint32_t cmd_clean_invalidate = 3;

    static void range_command(std::uint32_t cmd, const void* addr, std::size_t size) noexcept {
        if (size == 0) {
            return;
        }
        const auto start = reinterpret_cast<std::uintptr_t>(addr) & ~(cache_line_bytes - 1);
        const auto end = reinterpret_cast<std::uintptr_t>(addr) + size - 1;
        while ((SR::read() & sr_busycmdf) != 0) {
        }
        CMDRSADDRR::write(static_cast<std::uint32_t>(start));
        CMDREADDRR::write(static_cast<std::uint32_t>(end));
        CR::write((CR::read() & ~cr_cachecmd_msk) | (cmd << cr_cachecmd_pos));
        CR::set(cr_startcmd);
        while ((SR::read() & sr_busycmdf) != 0) {
        }
        FCR::write(fcr_ccmdendf);
        __asm volatile("dsb" ::: "memory");
    }
};

/// Cache-line aligned, line-padded DMA buffer: invalidation can never clip neighbours.
template <class T, std::size_t N>
struct alignas(cache_line_bytes) DmaBuffer {
    static constexpr std::size_t bytes = ((sizeof(T) * N + cache_line_bytes - 1) / cache_line_bytes) * cache_line_bytes;
    union {
        T data[N];
        std::byte raw[bytes];
    };
};

}  // namespace rtos::port::stm32h5

/// Place an object in the linker's non-cacheable DMA region (.dma_buffer, MPU-mapped
/// as Normal Non-cacheable by stm32h5::configure_mpu()).
#define RTOS_DMA_BUFFER [[gnu::section(".dma_buffer"), gnu::aligned(32)]]
