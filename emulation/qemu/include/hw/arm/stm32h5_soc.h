/*
 * STM32H563 SoC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0481 (STM32H52x/H53x/H56x/H57x), memory map and interrupt
 * numbers from the STM32H563 SVD (v1.8).
 */

#ifndef HW_ARM_STM32H5_SOC_H
#define HW_ARM_STM32H5_SOC_H

#include "system/memory.h"
#include "hw/arm/armv7m.h"
#include "hw/misc/armv8m_dwt.h"
#include "hw/misc/stm32h5_rcc.h"
#include "hw/misc/stm32h5_sysctl.h"
#include "hw/gpio/stm32l4x5_gpio.h"
#include "hw/char/stm32l4x5_usart.h"
#include "qom/object.h"

#define TYPE_STM32H563_SOC "stm32h563-soc"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h5SocState, STM32H563_SOC)

#define STM32H5_NUM_GPIOS  9   /* GPIOA..GPIOI */
#define STM32H5_NUM_USARTS 3   /* USART1..3 */

struct Stm32h5SocState {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;

    Armv8mDwtState dwt;
    Stm32h5RccState rcc;
    Stm32h5PwrState pwr;
    Stm32h5FlashState flash_if;
    /* The STM32L4x5 GPIO and USART register layouts match the STM32H5's. */
    Stm32l4x5GpioState gpio[STM32H5_NUM_GPIOS];
    Stm32l4x5UsartBaseState usart[STM32H5_NUM_USARTS];

    MemoryRegion flash;
    MemoryRegion sram;

    Clock *sysclk;
    Clock *refclk;

    /* Properties */
    uint32_t console_usart;   /* 1..3: USART connected to serial_hd(0) */
};

#endif /* HW_ARM_STM32H5_SOC_H */
