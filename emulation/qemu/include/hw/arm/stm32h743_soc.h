/*
 * STM32H743 SoC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0433 (STM32H743), memory map from the STM32H743 SVD (v2.4), interrupt
 * numbers and memory sizes from ST's stm32h743xx.h.
 */

#ifndef HW_ARM_STM32H743_SOC_H
#define HW_ARM_STM32H743_SOC_H

#include "system/memory.h"
#include "hw/arm/armv7m.h"
#include "hw/misc/armv8m_dwt.h"
#include "hw/misc/stm32h7_sysctl.h"
#include "hw/gpio/stm32l4x5_gpio.h"
#include "hw/char/stm32l4x5_usart.h"
#include "qom/object.h"

#define TYPE_STM32H743_SOC "stm32h743-soc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h743SocState, STM32H743_SOC)

#define STM32H743_NUM_GPIOS  11  /* GPIOA..GPIOK */
#define STM32H743_NUM_USARTS 3   /* USART1..3 */

struct Stm32h743SocState {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;

    /* The DWT cycle counter has the same CTRL/CYCCNT on ARMv7-M as on ARMv8-M. */
    Armv8mDwtState dwt;
    Stm32h7RccState rcc;
    Stm32h7PwrState pwr;
    Stm32h7FlashState flash_if;
    /* The STM32L4x5 GPIO and USART register layouts match the STM32H7's. */
    Stm32l4x5GpioState gpio[STM32H743_NUM_GPIOS];
    Stm32l4x5UsartBaseState usart[STM32H743_NUM_USARTS];

    MemoryRegion flash;
    MemoryRegion itcm;
    MemoryRegion dtcm;
    MemoryRegion axi_sram;
    MemoryRegion d2_sram;
    MemoryRegion sram4;
    MemoryRegion backup_sram;
    MemoryRegion system;

    uint32_t console_usart;  /* 1..3: this USART gets serial_hd(0) */
    Clock *sysclk;
    Clock *refclk;
};

#endif /* HW_ARM_STM32H743_SOC_H */
