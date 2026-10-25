// SPDX-License-Identifier: MIT
//
// ARMv8-M PMSA (MPU) helper and the STM32H563 default memory map.
//
// ARMv8-M regions are [base, limit] with 32-byte granularity (no power-of-two size
// constraint as on ARMv7-M). Attributes are indexed through MAIR0/MAIR1.
#pragma once

#include <cstdint>

#include "rtos/port/cm33/registers.hpp"

namespace rtos::port::stm32h5 {

/// MAIR attribute encodings (ARMv8-M ARM, B3.5.x MPU_MAIR).
enum class MemAttr : std::uint8_t {
    DeviceNGnRnE = 0x00,
    NormalNonCacheable = 0x44,       // outer/inner non-cacheable
    NormalWriteThrough = 0xAA,       // outer/inner WT, read-allocate (flash)
    NormalWriteBackRwAlloc = 0xFF,   // outer/inner WB, read+write allocate
};

enum class Access : std::uint8_t {
    PrivRw = 0b00,
    AnyRw = 0b01,
    PrivRo = 0b10,
    AnyRo = 0b11,
};

class Mpu {
public:
    /// Attribute slot `index` (0..7) <- `attr`.
    static void set_attr(unsigned index, MemAttr attr) noexcept {
        const unsigned shift = (index % 4) * 8;
        if (index < 4) {
            cm33::mpu::MAIR0::write((cm33::mpu::MAIR0::read() & ~(0xFFu << shift)) |
                                    (std::uint32_t(attr) << shift));
        } else {
            cm33::mpu::MAIR1::write((cm33::mpu::MAIR1::read() & ~(0xFFu << shift)) |
                                    (std::uint32_t(attr) << shift));
        }
    }

    /// Region `n` covers [base, limit] (inclusive; base 32-B aligned, limit = x*32-1).
    static void set_region(unsigned n, std::uint32_t base, std::uint32_t limit, unsigned attr_index,
                           Access access, bool execute_never, bool shareable = false) noexcept {
        cm33::mpu::RNR::write(n);
        cm33::mpu::RBAR::write((base & ~0x1Fu) | (shareable ? (0b10u << 3) : 0u) |
                               (std::uint32_t(access) << 1) | (execute_never ? 1u : 0u));
        cm33::mpu::RLAR::write((limit & ~0x1Fu) | ((attr_index & 7u) << 1) | 1u);
    }

    static void enable(bool privileged_default_map = true) noexcept {
        __asm volatile("dmb" ::: "memory");
        cm33::mpu::CTRL::write(cm33::mpu::ctrl_enable | (privileged_default_map ? cm33::mpu::ctrl_privdefena : 0u));
        __asm volatile("dsb \n isb" ::: "memory");
    }

    static void disable() noexcept {
        __asm volatile("dmb" ::: "memory");
        cm33::mpu::CTRL::write(0);
        __asm volatile("dsb \n isb" ::: "memory");
    }
};

/// Linker-script symbols describing the memory layout (see stm32h563.ld).
struct MemoryLayout {
    std::uint32_t flash_start, flash_end;  // inclusive end
    std::uint32_t ram_start, ram_end;
    std::uint32_t dma_start, dma_end;
};

/// Default STM32H563 MPU configuration:
///   R0 flash (code)         : RO, executable, write-through cacheable (ICACHE)
///   R1 OTP + RO info area   : RO, XN, NON-cacheable   (RM0481: these flash areas must
///   R2 flash high-cycle data: RW, XN, NON-cacheable    not be accessed through ICACHE)
///   R3 SRAM                 : RW, XN (no code execution from RAM), write-back
///   R4 DMA region           : RW, XN, NON-cacheable -> coherent even if moved to
///                             cached external RAM
///   Peripherals             : privileged default map (Device memory)
/// Note: R0 is read-only. A flash driver (file-system module) must temporarily
/// open a RW window over the sectors it programs and invalidate ICACHE afterwards.
inline void configure_mpu(const MemoryLayout& m) noexcept {
    constexpr unsigned attr_flash = 0, attr_nc = 1, attr_ram = 2;
    Mpu::disable();
    Mpu::set_attr(attr_flash, MemAttr::NormalWriteThrough);
    Mpu::set_attr(attr_nc, MemAttr::NormalNonCacheable);
    Mpu::set_attr(attr_ram, MemAttr::NormalWriteBackRwAlloc);

    Mpu::set_region(0, m.flash_start, m.flash_end, attr_flash, Access::PrivRo, /*xn=*/false);
    Mpu::set_region(1, 0x08FF'F000u, 0x08FF'FFFFu, attr_nc, Access::PrivRo, /*xn=*/true);
    Mpu::set_region(2, 0x0900'0000u, 0x0901'FFFFu, attr_nc, Access::PrivRw, /*xn=*/true);
    Mpu::set_region(3, m.ram_start, m.ram_end, attr_ram, Access::PrivRw, /*xn=*/true);
    Mpu::set_region(4, m.dma_start, m.dma_end, attr_nc, Access::PrivRw, /*xn=*/true, /*shareable=*/true);
    Mpu::enable();
}

}  // namespace rtos::port::stm32h5
