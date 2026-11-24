/*
 * STM32F1 reset and clock control and flash interface (register models)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0008 rev 21 (STM32F101/102/103/105/107), chapters 7.3 (RCC) and 3.3.3 (FLASH_ACR).
 */

#ifndef HW_MISC_STM32F1_SYSCTL_H
#define HW_MISC_STM32F1_SYSCTL_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define STM32F1_SYSCTL_SIZE 0x400

#define TYPE_STM32F1_RCC "stm32f1-rcc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32f1RccState, STM32F1_RCC)

struct Stm32f1RccState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32F1_SYSCTL_SIZE / 4];
};

#define TYPE_STM32F1_FLASH "stm32f1-flash"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32f1FlashState, STM32F1_FLASH)

struct Stm32f1FlashState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32F1_SYSCTL_SIZE / 4];
};

#endif /* HW_MISC_STM32F1_SYSCTL_H */
