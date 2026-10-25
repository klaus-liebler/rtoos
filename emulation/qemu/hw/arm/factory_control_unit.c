/*
 * Factory Control Unit (factory_in_a_box/pcb_factory_control_unit)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 *   MCU      STM32H563VIT6, no HSE crystal: SYSCLK = HSI 64 MHz -> PLL1 250 MHz
 *   Console  USART3 (PD8 TX / PD9 RX -> ST-LINK VCP)  -> first -serial
 *   LED      LED_INFO on PH1, active low               -> trace "led_*"
 *
 * Usage:
 *   qemu-system-arm -M fcu -nographic -kernel firmware.elf
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/stm32h5_soc.h"
#include "hw/misc/led.h"

/* The firmware switches to PLL1 at start-up (boards/fcu); the clock is fixed. */
#define SYSCLK_FRQ 250000000ULL

#define TYPE_FCU_MACHINE MACHINE_TYPE_NAME("fcu")
OBJECT_DECLARE_SIMPLE_TYPE(FcuMachineState, FCU_MACHINE)

struct FcuMachineState {
    MachineState parent_obj;

    Stm32h5SocState soc;
    Clock *sysclk;
};

static void fcu_init(MachineState *machine)
{
    FcuMachineState *s = FCU_MACHINE(machine);
    LEDState *led_info;

    /* Fixed-frequency clock: no migration state */
    s->sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(s->sysclk, SYSCLK_FRQ);

    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_STM32H563_SOC);
    qdev_prop_set_uint32(DEVICE(&s->soc), "console-usart", 3);
    qdev_connect_clock_in(DEVICE(&s->soc), "sysclk", s->sysclk);
    sysbus_realize(SYS_BUS_DEVICE(&s->soc), &error_fatal);

    led_info = led_create_simple(OBJECT(machine), GPIO_POLARITY_ACTIVE_LOW,
                                 LED_COLOR_GREEN, "LED_INFO (PH1)");
    qdev_connect_gpio_out(DEVICE(&s->soc.gpio[7]), 1,
                          qdev_get_gpio_in(DEVICE(led_info), 0));

    armv7m_load_kernel(s->soc.armv7m.cpu, machine->kernel_filename,
                       0x08000000, 2 * 1024 * 1024);
}

static void fcu_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char *const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m33"),
        NULL
    };

    mc->desc = "Factory Control Unit (STM32H563VIT6, Cortex-M33)";
    mc->init = fcu_init;
    mc->valid_cpu_types = valid_cpu_types;
    /* SRAM is part of the SoC */
    mc->default_ram_size = 0;
}

static const TypeInfo fcu_machine_types[] = {
    {
        .name          = TYPE_FCU_MACHINE,
        .parent        = TYPE_MACHINE,
        .instance_size = sizeof(FcuMachineState),
        .class_init    = fcu_machine_class_init,
        .interfaces    = arm_machine_interfaces,
    },
};

DEFINE_TYPES(fcu_machine_types)
