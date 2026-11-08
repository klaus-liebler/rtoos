// SPDX-License-Identifier: MIT
//
// ARMv7-M PMSA (MPU) of the Cortex-M7: regions of a power-of-two size at a size-aligned base,
// with memory type (TEX/C/B), shareability, access permission and execute-never. Registers and
// encodings: ARM CMSIS 5.9.0 core_cm7.h (MPU_TYPE..MPU_RASR) and mpu_armv7.h (ARM_MPU_RBAR,
// ARM_MPU_RASR_EX, ARM_MPU_ACCESS_*, ARM_MPU_AP_*, ARM_MPU_Enable). The register values are
// computed at compile time (rasr(), valid()) and tested on the host.
//
//   cm7::mpu::disable();
//   cm7::mpu::set_region(0, {.base = 0x3000'0000, .bytes = 256 * 1024, .memory = Memory::NormalNonCacheable,
//                            .access = Access::PrivRw, .execute_never = true, .shareable = true});
//   cm7::mpu::enable();   // privileged default map for everything without a region
//
// Where regions overlap, the higher region number wins.
#pragma once

#include <contracts>
#include <cstdint>

#include "rtos/port/cortex_m/registers.hpp"

namespace rtos::port::cm7::mpu {

using TYPE = cortex_m::Reg<0xE000'ED90>;
using CTRL = cortex_m::Reg<0xE000'ED94>;
using RNR = cortex_m::Reg<0xE000'ED98>;
using RBAR = cortex_m::Reg<0xE000'ED9C>;
using RASR = cortex_m::Reg<0xE000'EDA0>;
inline constexpr std::uint32_t ctrl_enable = 1u << 0, ctrl_privdefena = 1u << 2;
inline constexpr std::uint32_t shcsr_memfaultena = 1u << 16;

/// Memory types as TEX:C:B (mpu_armv7.h ARM_MPU_ACCESS_ / ARM_MPU_ACCESS_NORMAL).
enum class Memory : std::uint8_t {
    StronglyOrdered,      ///< TEX 000, C 0, B 0
    Device,               ///< TEX 000, C 0, B 1 (shareable device)
    NormalWriteThrough,   ///< TEX 000, C 1, B 0 (no write allocate)
    NormalNonCacheable,   ///< TEX 001, C 0, B 0
    NormalWriteBack,      ///< TEX 001, C 1, B 1 (read and write allocate)
};

/// Data access permission (mpu_armv7.h ARM_MPU_AP_*).
enum class Access : std::uint8_t { None = 0, PrivRw = 1, PrivRwUserRo = 2, Full = 3, PrivRo = 5, Ro = 6 };

struct Region {
    std::uint32_t base;
    std::uint32_t bytes;  ///< power of two, at least 32; base aligned to it
    Memory memory;
    Access access = Access::PrivRw;
    bool execute_never = true;
    bool shareable = false;
    std::uint8_t subregions_disabled = 0;  ///< SRD: bit i switches off the i-th eighth (regions >= 256 B)
};

[[nodiscard]] constexpr bool power_of_two(std::uint32_t v) noexcept { return v != 0 && (v & (v - 1)) == 0; }

[[nodiscard]] constexpr bool valid(const Region& r) noexcept {
    return power_of_two(r.bytes) && r.bytes >= 32 && r.base % r.bytes == 0;
}

/// RASR.SIZE: the region has 2^(SIZE + 1) bytes.
[[nodiscard]] constexpr std::uint32_t size_field(std::uint32_t bytes) noexcept {
    std::uint32_t log2 = 0;
    while ((1ull << log2) < bytes) {
        ++log2;
    }
    return log2 - 1;
}

/// TEX (21:19), S (18), C (17), B (16) of a region.
[[nodiscard]] constexpr std::uint32_t attributes(Memory m, bool shareable) noexcept {
    std::uint32_t tex = 0, c = 0, b = 0;
    switch (m) {
        case Memory::StronglyOrdered: break;
        case Memory::Device: b = 1; break;
        case Memory::NormalWriteThrough: c = 1; break;
        case Memory::NormalNonCacheable: tex = 1; break;
        case Memory::NormalWriteBack: tex = 1, c = 1, b = 1; break;
    }
    return (tex << 19) | ((shareable ? 1u : 0u) << 18) | (c << 17) | (b << 16);
}

/// RASR: XN (28), AP (26:24), TEX/S/C/B, SRD (15:8), SIZE (5:1), ENABLE.
[[nodiscard]] constexpr std::uint32_t rasr(const Region& r) noexcept {
    return ((r.execute_never ? 1u : 0u) << 28) | (static_cast<std::uint32_t>(r.access) << 24) |
           attributes(r.memory, r.shareable) | (std::uint32_t{r.subregions_disabled} << 8) |
           (size_field(r.bytes) << 1) | 1u;
}

/// SRD bits that switch off every eighth of `r` reaching at or above `address` (e.g. a data
/// area at the end of a code region); 0 if `address` lies above the region.
[[nodiscard]] constexpr std::uint8_t subregions_from(const Region& r, std::uint32_t address) noexcept {
    const std::uint32_t eighth = r.bytes / 8;
    std::uint32_t srd = 0;
    for (std::uint32_t i = 0; i < 8; ++i) {
        if (r.base + (i + 1) * eighth > address) {
            srd |= 1u << i;
        }
    }
    return static_cast<std::uint8_t>(srd);
}

/// RBAR with VALID and the region number (ARM_MPU_RBAR).
[[nodiscard]] constexpr std::uint32_t rbar(unsigned n, const Region& r) noexcept {
    return (r.base & ~0x1Fu) | (1u << 4) | (n & 0xFu);
}

/// Region `n` (0..15 on the STM32H7) <- `r`. Call with the MPU disabled.
template <unsigned N, Region R>
inline void set_region() noexcept {
    static_assert(valid(R), "MPU region: size a power of two >= 32, base aligned to the size");
    static_assert(N < 16, "the Cortex-M7 of the STM32H7 has 16 MPU regions");
    RBAR::write(rbar(N, R));
    RASR::write(rasr(R));
}

/// The same with a region known at run time only (e.g. from a linker symbol).
inline void set_region(unsigned n, const Region& r) noexcept {
    contract_assert(valid(r) && n < 16);
    RBAR::write(rbar(n, r));
    RASR::write(rasr(r));
}

/// MPU on; with `privileged_default_map` the default memory map applies where no region does.
inline void enable(bool privileged_default_map = true) noexcept {
    __asm volatile("dmb" ::: "memory");
    CTRL::write(ctrl_enable | (privileged_default_map ? ctrl_privdefena : 0u));
    cortex_m::scb::SHCSR::set(shcsr_memfaultena);
    __asm volatile("dsb \n isb" ::: "memory");
}

inline void disable() noexcept {
    __asm volatile("dmb" ::: "memory");
    CTRL::write(0);
}

}  // namespace rtos::port::cm7::mpu
