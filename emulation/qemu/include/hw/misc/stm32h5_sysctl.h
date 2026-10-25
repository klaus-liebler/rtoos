/*
 * STM32H5 power control (voltage scaling) and flash interface (ACR) models
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0481, register layout and reset values from the STM32H563
 * SVD (v1.8).
 */

#ifndef HW_MISC_STM32H5_SYSCTL_H
#define HW_MISC_STM32H5_SYSCTL_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define STM32H5_SYSCTL_SIZE 0x400

#define TYPE_STM32H5_PWR "stm32h5-pwr"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h5PwrState, STM32H5_PWR)

struct Stm32h5PwrState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32H5_SYSCTL_SIZE / 4];
};

#define TYPE_STM32H5_FLASH "stm32h5-flash"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h5FlashState, STM32H5_FLASH)

struct Stm32h5FlashState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32H5_SYSCTL_SIZE / 4];
};

#endif /* HW_MISC_STM32H5_SYSCTL_H */
