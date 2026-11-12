/* SPDX-License-Identifier: MIT
 * STM32H743xI (e.g. VIT6): 2 MB flash in two banks, RAM in separate blocks (stm32h743xx.h):
 *   DTCM      128 KB  0x20000000  CPU only (no DMA1/DMA2 access), not cached: the main stack
 *   AXI SRAM  512 KB  0x24000000  D1, every bus master: the first 32 KB .sdmmc_buffer (SDMMC1's
 *                                 IDMA reaches only the AXI SRAM; non-cacheable, MPU region R4),
 *                                 then .data, .bss
 *   SRAM1-2   256 KB  0x30000000  D2 (next to DMA1/DMA2, Ethernet, USB): .dma_buffer, non-cacheable
 *                                 (MPU region R3, stm32h7/mpu.hpp configure_mpu())
 *   SRAM3      32 KB  0x30040000  D2
 *   SRAM4      64 KB  0x38000000  D3
 * The CPU boots from BOOT_ADD0 = 0x0800 (0x08000000), which also sets VTOR.
 * The data cache is off until the firmware enables it (cm7/cache.hpp); memory a DMA uses then
 * belongs in .dma_buffer (RTOS_DMA_BUFFER), which configure_mpu() makes non-cacheable.
 */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCM    (rw)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_SDMMC (rw): ORIGIN = 0x24000000, LENGTH = 32K   /* = stm32h7 sdmmc_buffer_base/_bytes */
  RAM     (rw)  : ORIGIN = 0x24008000, LENGTH = 480K
  RAM_DMA (rw)  : ORIGIN = 0x30000000, LENGTH = 256K
  SRAM3   (rw)  : ORIGIN = 0x30040000, LENGTH = 32K
  SRAM_D3 (rw)  : ORIGIN = 0x38000000, LENGTH = 64K
}

/* Main (MSP) stack, at the top of the DTCM: used by main() until Kernel::start(), then by
   interrupts only. */
_min_msp_size = 2K;
_estack = ORIGIN(DTCM) + LENGTH(DTCM);

SECTIONS
{
  /* 15 exceptions + 150 interrupts + the initial SP = 166 words: VTOR needs 1024-byte alignment */
  .isr_vector : ALIGN(1024)
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text .text.*)
    *(.rodata .rodata.*)
    . = ALIGN(4);
  } > FLASH

  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > FLASH

  .init_array :
  {
    . = ALIGN(4);
    __init_array_start = .;
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    __init_array_end = .;
  } > FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(8);
    _sdata = .;
    *(.data .data.*)
    . = ALIGN(8);
    _edata = .;
  } > RAM AT > FLASH

  /* End of flash content, 32-byte rounded (flash programming granule: one 256-bit flash word). */
  __flash_used_end = ALIGN(LOADADDR(.data) + SIZEOF(.data), 32);
  /* Flash kept for data (file systems, settings) from rtos_flash_data_start to the end:
     rtos_add_firmware(... FLASH_DATA_START <address>) defines it; without one, none. */
  PROVIDE(rtos_flash_data_start = ORIGIN(FLASH) + LENGTH(FLASH));
  ASSERT(__flash_used_end <= rtos_flash_data_start, "flash overflow: the image reaches into the data region")

  .bss (NOLOAD) :
  {
    . = ALIGN(8);
    _sbss = .;
    *(.bss .bss.* COMMON)
    . = ALIGN(8);
    _ebss = .;
  } > RAM

  /* DMA buffers: zeroed at start-up like .bss (startup.cpp), then __init_array constructs what
     has a constructor. Not loaded: an initial value other than 0 would be lost -
     tools/check_noload.py refuses the image then. */
  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    __dma_start = .;
    *(.dma_buffer .dma_buffer.*)
    . = ALIGN(32);
    __dma_end = .;
  } > RAM_DMA

  /* SDMMC1 buffers (RTOS_SDMMC_BUFFER): zeroed at start-up as .dma_buffer. */
  .sdmmc_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    __sdmmc_start = .;
    *(.sdmmc_buffer .sdmmc_buffer.*)
    . = ALIGN(32);
    __sdmmc_end = .;
  } > RAM_SDMMC

  /* The main stack's room in the DTCM (nothing else is placed there yet). */
  .msp_stack (NOLOAD) :
  {
    . = ALIGN(8);
    . = . + _min_msp_size;
  } > DTCM
}
