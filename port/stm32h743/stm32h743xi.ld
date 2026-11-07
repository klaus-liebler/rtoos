/* SPDX-License-Identifier: MIT
 * STM32H743xI (e.g. VIT6): 2 MB flash in two banks, RAM in separate blocks (stm32h743xx.h):
 *   DTCM      128 KB  0x20000000  CPU only (no DMA1/DMA2 access), not cached: the main stack
 *   AXI SRAM  512 KB  0x24000000  D1, every bus master: .data, .bss
 *   SRAM1-3   288 KB  0x30000000  D2 (next to DMA1/DMA2, Ethernet, USB)
 *   SRAM4      64 KB  0x38000000  D3
 * The CPU boots from BOOT_ADD0 = 0x0800 (0x08000000), which also sets VTOR.
 * The data cache is off until the firmware enables it (cm7/cache.hpp); memory a DMA uses then
 * needs cache maintenance or a non-cacheable MPU region.
 */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCM    (rw)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM     (rw)  : ORIGIN = 0x24000000, LENGTH = 512K
  SRAM_D2 (rw)  : ORIGIN = 0x30000000, LENGTH = 288K
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
    *(.dma_buffer .dma_buffer.*)
    . = ALIGN(8);
    _ebss = .;
  } > RAM

  /* The main stack's room in the DTCM (nothing else is placed there yet). */
  .msp_stack (NOLOAD) :
  {
    . = ALIGN(8);
    . = . + _min_msp_size;
  } > DTCM
}
