// SPDX-License-Identifier: MIT
//
// The STM32H743's default MPU configuration and the DMA buffer section (cm7/mpu.hpp; memory map:
// stm32h743xx.h, stm32h743xi.ld).
//
//   R0 flash 2 MB             : read-only, executable, write-through - without the eighths
//                               (256 KB) from rtos_flash_data_start on: there the default map
//                               (write-through, writable) lets a flash driver program
//   R1 DTCM 128 KB            : RW, XN (not cached anyway: tightly coupled)
//   R2 AXI SRAM 512 KB        : RW, XN, write-back read/write allocate (.data, .bss)
//   R3 SRAM1 + SRAM2 256 KB   : RW, XN, NON-cacheable, shareable: the DMA region (.dma_buffer)
//   Peripherals, the rest     : privileged default map (Device memory)
//
// The default map would make SRAM1/2 write-back cacheable: R3 is what keeps the CPU and the
// DMAs (DMA1/2, Ethernet, SDMMC IDMA, USB) coherent without cache maintenance. Put every buffer
// a DMA reads or writes there:
//
//   RTOS_DMA_BUFFER std::array<std::uint16_t, 24> led_bits;
//
// The flash data area (rtos_add_firmware(... FLASH_DATA_START ...), littlefs) is writable: keep
// it 256 KB aligned, or the eighth it shares with the image becomes writable too.
#pragma once

#include <cstdint>

#include "rtos/port/cm7/mpu.hpp"
#include "rtos/port/stm32h7/chip.hpp"

/// Place an object in the linker's non-cacheable DMA region (.dma_buffer in SRAM1/2, MPU region
/// R3), aligned to the data cache line.
#define RTOS_DMA_BUFFER [[gnu::section(".dma_buffer"), gnu::aligned(32)]]

namespace rtos::port::stm32h7 {

namespace mpu_regions {
using rtos::port::cm7::mpu::Access, rtos::port::cm7::mpu::Memory, rtos::port::cm7::mpu::Region;
inline constexpr Region flash{.base = flash_base, .bytes = flash_bytes, .memory = Memory::NormalWriteThrough,
                              .access = Access::PrivRo, .execute_never = false};
inline constexpr Region dtcm{.base = dtcm_base, .bytes = dtcm_bytes, .memory = Memory::NormalWriteBack};
inline constexpr Region axi_sram{.base = axi_sram_base, .bytes = axi_sram_bytes, .memory = Memory::NormalWriteBack};
/// SRAM1 + SRAM2 (stm32h7xx: 128 KB each, contiguous); SRAM3 above it stays default.
inline constexpr Region dma{.base = sram1_base, .bytes = 256 * 1024, .memory = Memory::NormalNonCacheable,
                            .shareable = true};
}  // namespace mpu_regions

extern "C" std::uint32_t rtos_flash_data_start;  // linker script: the end of the flash without a data area

/// Default STM32H743 MPU configuration (the table above). Call before the data cache is enabled.
inline void configure_mpu() noexcept {
    namespace mpu = rtos::port::cm7::mpu;
    mpu::disable();
    mpu::Region flash = mpu_regions::flash;
    flash.subregions_disabled =
        mpu::subregions_from(flash, static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&rtos_flash_data_start)));
    mpu::set_region(0, flash);
    mpu::set_region<1, mpu_regions::dtcm>();
    mpu::set_region<2, mpu_regions::axi_sram>();
    mpu::set_region<3, mpu_regions::dma>();
    mpu::enable();
}

}  // namespace rtos::port::stm32h7
