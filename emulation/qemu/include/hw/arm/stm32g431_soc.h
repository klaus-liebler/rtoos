/*
 * STM32G431 SoC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0440 (STM32G4), memory map from the STM32G431 SVD (v1.8), interrupt
 * numbers from ST's stm32g431xx.h.
 */

#ifndef HW_ARM_STM32G431_SOC_H
#define HW_ARM_STM32G431_SOC_H

#include "system/memory.h"
#include "hw/arm/armv7m.h"
#include "hw/misc/armv8m_dwt.h"
#include "hw/misc/stm32g4_sysctl.h"
#include "hw/gpio/stm32l4x5_gpio.h"
#include "hw/char/stm32l4x5_usart.h"
#include "qom/object.h"

#define TYPE_STM32G431_SOC "stm32g431-soc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g431SocState, STM32G431_SOC)

#define STM32G431_NUM_GPIOS  7   /* GPIOA..GPIOG */
#define STM32G431_NUM_USARTS 3   /* USART1..3 */

struct Stm32g431SocState {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;

    /* The DWT cycle counter has the same CTRL/CYCCNT on ARMv7-M as on ARMv8-M. */
    Armv8mDwtState dwt;
    Stm32g4RccState rcc;
    Stm32g4PwrState pwr;
    Stm32g4FlashState flash_if;
    /* The STM32L4x5 GPIO and USART register layouts match the STM32G4's. */
    Stm32l4x5GpioState gpio[STM32G431_NUM_GPIOS];
    Stm32l4x5UsartBaseState usart[STM32G431_NUM_USARTS];

    MemoryRegion flash;
    MemoryRegion flash_alias;
    MemoryRegion sram;
    MemoryRegion ccmsram;
    MemoryRegion system;

    Clock *sysclk;
    Clock *refclk;
};

#endif /* HW_ARM_STM32G431_SOC_H */
