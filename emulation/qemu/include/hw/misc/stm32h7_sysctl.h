/*
 * STM32H7 reset and clock control, power control and flash interface (register models)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0433, register layout and reset values from the STM32H743 SVD (v2.4).
 */

#ifndef HW_MISC_STM32H7_SYSCTL_H
#define HW_MISC_STM32H7_SYSCTL_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define STM32H7_SYSCTL_SIZE 0x400

#define TYPE_STM32H7_RCC "stm32h7-rcc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7RccState, STM32H7_RCC)

struct Stm32h7RccState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32H7_SYSCTL_SIZE / 4];
};

#define TYPE_STM32H7_PWR "stm32h7-pwr"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7PwrState, STM32H7_PWR)

struct Stm32h7PwrState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32H7_SYSCTL_SIZE / 4];
};

#define TYPE_STM32H7_FLASH "stm32h7-flash"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h7FlashState, STM32H7_FLASH)

struct Stm32h7FlashState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32H7_SYSCTL_SIZE / 4];
};

#endif /* HW_MISC_STM32H7_SYSCTL_H */
