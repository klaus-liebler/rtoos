/* SPDX-License-Identifier: MIT
 * STM32F103xB (e.g. CBT6): 128 KB flash in 1 KB pages, 20 KB SRAM (stm32f103xb.h, DS5319).
 * No cache: DMA works on any SRAM.
 */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
  RAM     (rw)  : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Main (MSP) stack: used by main() until Kernel::start(), then by interrupts only. */
_min_msp_size = 1K;
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  /* VTOR alignment: 43 + 16 vectors -> 64 words -> 256 bytes */
  .isr_vector : ALIGN(256)
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

  /* End of flash content, 4-byte rounded (the F1 programs half words). */
  __flash_used_end = ALIGN(LOADADDR(.data) + SIZEOF(.data), 4);
  /* Flash kept for data (settings) from rtos_flash_data_start to the end:
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
  /* the DMA buffers are part of .bss here (zeroed with it): no ranges of their own */
  PROVIDE(__dma_start = _ebss);
  PROVIDE(__dma_end = _ebss);
  PROVIDE(__sdmmc_start = _ebss);
  PROVIDE(__sdmmc_end = _ebss);

  ASSERT(_ebss + _min_msp_size <= _estack, "RAM overflow: no room for the main stack")
}
