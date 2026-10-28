/* SPDX-License-Identifier: MIT
 * STM32H563xI (e.g. VIT6, ZIT6): 2 MB flash, 640 KB contiguous SRAM (SRAM1 256K + SRAM2 64K + SRAM3 320K).
 * The top 64 KB of SRAM3 are reserved for DMA buffers (MPU: Normal Non-cacheable).
 */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM     (rw)  : ORIGIN = 0x20000000, LENGTH = 576K
  RAM_DMA (rw)  : ORIGIN = 0x20090000, LENGTH = 64K
}

/* Main (MSP) stack: used by main() until Kernel::start(), then by interrupts only. */
_min_msp_size = 2K;
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
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

  /* End of flash content, 32-byte rounded for the MPU limit. */
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

  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    __dma_start = .;
    *(.dma_buffer .dma_buffer.*)
    . = ALIGN(32);
  } > RAM_DMA

  ASSERT(_ebss + _min_msp_size <= _estack, "RAM overflow: no room for the main stack")
}
