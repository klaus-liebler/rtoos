/*
 * STM32H5 reset and clock control (register model)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0481, register layout from the STM32H563 SVD (v1.8).
 */

#ifndef HW_MISC_STM32H5_RCC_H
#define HW_MISC_STM32H5_RCC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32H5_RCC "stm32h5-rcc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h5RccState, STM32H5_RCC)

#define STM32H5_RCC_SIZE 0x400

struct Stm32h5RccState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32H5_RCC_SIZE / 4];
};

#endif /* HW_MISC_STM32H5_RCC_H */
