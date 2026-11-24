/*
 * STM32F103 SoC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0008 rev 21, memory map and interrupt numbers from ST's stm32f103xb.h.
 */

#ifndef HW_ARM_STM32F103_SOC_H
#define HW_ARM_STM32F103_SOC_H

#include "system/memory.h"
#include "hw/arm/armv7m.h"
#include "hw/misc/armv8m_dwt.h"
#include "hw/misc/stm32f1_sysctl.h"
#include "hw/char/stm32f2xx_usart.h"
#include "qom/object.h"

#define TYPE_STM32F103_SOC "stm32f103-soc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32f103SocState, STM32F103_SOC)

#define STM32F103_NUM_USARTS 3   /* USART1..3 */

struct Stm32f103SocState {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;

    /* The DWT cycle counter has the same CTRL/CYCCNT on ARMv7-M as on ARMv8-M. */
    Armv8mDwtState dwt;
    Stm32f1RccState rcc;
    Stm32f1FlashState flash_if;
    /* The STM32F2xx USART register layout (SR, DR, BRR, CR1-3) is the STM32F1's. */
    STM32F2XXUsartState usart[STM32F103_NUM_USARTS];

    MemoryRegion flash;
    MemoryRegion flash_alias;
    MemoryRegion sram;
    MemoryRegion system;

    Clock *sysclk;
    Clock *refclk;
};

#endif /* HW_ARM_STM32F103_SOC_H */
