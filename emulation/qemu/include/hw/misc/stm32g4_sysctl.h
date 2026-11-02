/*
 * STM32G4 reset and clock control, power control and flash interface (register models)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0440, register layout and reset values from the STM32G431 SVD (v1.8).
 */

#ifndef HW_MISC_STM32G4_SYSCTL_H
#define HW_MISC_STM32G4_SYSCTL_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define STM32G4_SYSCTL_SIZE 0x400

#define TYPE_STM32G4_RCC "stm32g4-rcc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g4RccState, STM32G4_RCC)

struct Stm32g4RccState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32G4_SYSCTL_SIZE / 4];
};

#define TYPE_STM32G4_PWR "stm32g4-pwr"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g4PwrState, STM32G4_PWR)

struct Stm32g4PwrState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32G4_SYSCTL_SIZE / 4];
};

#define TYPE_STM32G4_FLASH "stm32g4-flash"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g4FlashState, STM32G4_FLASH)

struct Stm32g4FlashState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t regs[STM32G4_SYSCTL_SIZE / 4];
};

#endif /* HW_MISC_STM32G4_SYSCTL_H */
