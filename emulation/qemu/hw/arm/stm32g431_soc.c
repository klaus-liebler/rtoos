/*
 * STM32G431 SoC and the "stm32g431" machine (the bare chip)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0440, memory map from the STM32G431 SVD (v1.8), interrupt numbers and memory
 * sizes from ST's stm32g431xx.h.
 *
 * Modelled:
 *   - Cortex-M4 with FPU, DSP, MPU (8 regions)
 *   - 128 KB flash at 0x08000000, aliased at 0 (boot from main flash: the ARMv7-M core
 *     fetches the vector table at VTOR = 0), 32 KB SRAM at 0x20000000 (SRAM1 +
 *     SRAM2 + the CCM SRAM alias, contiguous), CCM SRAM at 0x10000000 (separate memory here),
 *     system memory (UID, package data; zero-filled)
 *   - DWT cycle counter (CYCCNT at SYSCLK; QEMU's core leaves DWT RAZ/WI)
 *   - RCC, PWR and FLASH_ACR (register models: new settings are ready at once)
 *   - GPIOA..G and USART1..3 (STM32L4x5 models, register-compatible)
 * Everything else in the peripheral space is an unimplemented device: reads return 0,
 * accesses are logged with "-d unimp".
 *
 * Clocks: fixed SYSCLK 170 MHz (the self-test runs the PLL at 170 MHz from HSI16); SysTick
 * reference clock = SYSCLK / 8. The RCC registers do not change emulated frequencies.
 *
 * Usage:
 *   qemu-system-arm -M stm32g431 -nographic -kernel firmware.elf    (console: USART1)
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "hw/arm/stm32g431_soc.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/misc/unimp.h"

#define FLASH_BASE_ADDRESS   0x08000000
#define FLASH_SIZE           (128 * KiB)
#define SRAM_BASE_ADDRESS    0x20000000
#define SRAM_SIZE            (32 * KiB)
#define CCMSRAM_BASE_ADDRESS 0x10000000
#define CCMSRAM_SIZE         (10 * KiB)
#define SYSTEM_BASE_ADDRESS  0x1FFF7000
#define SYSTEM_SIZE          (4 * KiB)

#define RCC_BASE_ADDRESS     0x40021000
#define PWR_BASE_ADDRESS     0x40007000
#define FLASH_IF_ADDRESS     0x40022000

#define NUM_IRQ            102   /* stm32g431xx.h: FMAC_IRQn = 101 is the last */
#define NUM_PRIO_BITS      4

#define SYSCLK_FRQ 170000000ULL

static const struct {
    hwaddr addr;
    uint32_t moder_reset;
    uint32_t ospeedr_reset;
    uint32_t pupdr_reset;
} stm32g431_gpio_cfg[STM32G431_NUM_GPIOS] = {
    { 0x48000000, 0xABFFFFFF, 0x0C000000, 0x64000000 },
    { 0x48000400, 0xFFFFFEBF, 0x000000C0, 0x00000100 },
    { 0x48000800, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x48000C00, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x48001000, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x48001400, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x48001800, 0xFFFFFFFF, 0x00000000, 0x00000000 },
};

static const struct {
    hwaddr addr;
    int irq;
} stm32g431_usart_cfg[STM32G431_NUM_USARTS] = {
    { 0x40013800, 37 }, /* USART1: console, serial_hd(0) */
    { 0x40004400, 38 }, /* USART2 */
    { 0x40004800, 39 }, /* USART3 */
};

static void stm32g431_soc_initfn(Object *obj)
{
    Stm32g431SocState *s = STM32G431_SOC(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
    object_initialize_child(obj, "dwt", &s->dwt, TYPE_ARMV8M_DWT);
    object_initialize_child(obj, "rcc", &s->rcc, TYPE_STM32G4_RCC);
    object_initialize_child(obj, "pwr", &s->pwr, TYPE_STM32G4_PWR);
    object_initialize_child(obj, "flash-if", &s->flash_if, TYPE_STM32G4_FLASH);
    for (unsigned i = 0; i < STM32G431_NUM_GPIOS; i++) {
        g_autofree char *name = g_strdup_printf("gpio%c", 'a' + i);
        object_initialize_child(obj, name, &s->gpio[i], TYPE_STM32L4X5_GPIO);
    }
    for (unsigned i = 0; i < STM32G431_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i], TYPE_STM32L4X5_USART);
    }

    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

static bool stm32g431_map(SysBusDevice *busdev, hwaddr addr, Error **errp)
{
    if (!sysbus_realize(busdev, errp)) {
        return false;
    }
    sysbus_mmio_map(busdev, 0, addr);
    return true;
}

static void stm32g431_soc_realize(DeviceState *dev_soc, Error **errp)
{
    Stm32g431SocState *s = STM32G431_SOC(dev_soc);
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *armv7m, *dev;
    SysBusDevice *busdev;

    if (!clock_has_source(s->sysclk)) {
        error_setg(errp, "sysclk clock must be wired up by the board code");
        return;
    }
    if (clock_has_source(s->refclk)) {
        error_setg(errp, "refclk clock must not be wired up by the board code");
        return;
    }
    /* SysTick external reference: HCLK / 8 */
    clock_set_mul_div(s->refclk, 8, 1);
    clock_set_source(s->refclk, s->sysclk);

    /* Memory */
    if (!memory_region_init_rom(&s->flash, OBJECT(dev_soc), "flash", FLASH_SIZE, errp)) {
        return;
    }
    memory_region_add_subregion(system_memory, FLASH_BASE_ADDRESS, &s->flash);
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc), "flash.alias", &s->flash, 0, FLASH_SIZE);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);
    if (!memory_region_init_ram(&s->sram, OBJECT(dev_soc), "sram", SRAM_SIZE, errp)) {
        return;
    }
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, &s->sram);
    if (!memory_region_init_ram(&s->ccmsram, OBJECT(dev_soc), "ccmsram", CCMSRAM_SIZE, errp)) {
        return;
    }
    memory_region_add_subregion(system_memory, CCMSRAM_BASE_ADDRESS, &s->ccmsram);
    if (!memory_region_init_ram(&s->system, OBJECT(dev_soc), "system", SYSTEM_SIZE, errp)) {
        return;
    }
    memory_region_add_subregion(system_memory, SYSTEM_BASE_ADDRESS, &s->system);

    /* CPU */
    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", NUM_IRQ);
    qdev_prop_set_uint32(armv7m, "num-prio-bits", NUM_PRIO_BITS);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_prop_set_uint32(armv7m, "mpu-ns-regions", 8);
    qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", s->refclk);
    object_property_set_link(OBJECT(&s->armv7m), "memory", OBJECT(system_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }

    /* DWT: into the CPU's PPB container, above its RAZ/WI default region */
    qdev_connect_clock_in(DEVICE(&s->dwt), "clk", s->sysclk);
    busdev = SYS_BUS_DEVICE(&s->dwt);
    if (!sysbus_realize(busdev, errp)) {
        return;
    }
    memory_region_add_subregion_overlap(&s->armv7m.container, ARMV8M_DWT_BASE,
                                        sysbus_mmio_get_region(busdev, 0), 0);

    /* RCC, PWR, FLASH interface */
    if (!stm32g431_map(SYS_BUS_DEVICE(&s->rcc), RCC_BASE_ADDRESS, errp) ||
        !stm32g431_map(SYS_BUS_DEVICE(&s->pwr), PWR_BASE_ADDRESS, errp) ||
        !stm32g431_map(SYS_BUS_DEVICE(&s->flash_if), FLASH_IF_ADDRESS, errp)) {
        return;
    }

    /* GPIO */
    for (unsigned i = 0; i < STM32G431_NUM_GPIOS; i++) {
        g_autofree char *name = g_strdup_printf("%c", 'A' + i);
        dev = DEVICE(&s->gpio[i]);
        qdev_prop_set_string(dev, "name", name);
        qdev_prop_set_uint32(dev, "mode-reset", stm32g431_gpio_cfg[i].moder_reset);
        qdev_prop_set_uint32(dev, "ospeed-reset", stm32g431_gpio_cfg[i].ospeedr_reset);
        qdev_prop_set_uint32(dev, "pupd-reset", stm32g431_gpio_cfg[i].pupdr_reset);
        qdev_connect_clock_in(dev, "clk", s->sysclk);
        if (!stm32g431_map(SYS_BUS_DEVICE(dev), stm32g431_gpio_cfg[i].addr, errp)) {
            return;
        }
    }

    /* USART: USART1 gets serial_hd(0), the others follow in order */
    for (unsigned i = 0; i < STM32G431_NUM_USARTS; i++) {
        dev = DEVICE(&s->usart[i]);
        qdev_prop_set_chr(dev, "chardev", serial_hd(i));
        qdev_connect_clock_in(dev, "clk", s->sysclk);
        busdev = SYS_BUS_DEVICE(dev);
        if (!stm32g431_map(busdev, stm32g431_usart_cfg[i].addr, errp)) {
            return;
        }
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, stm32g431_usart_cfg[i].irq));
    }

    /* Not modelled: named for readable "-d unimp" logs */
    create_unimplemented_device("SYSCFG", 0x40010000, 0x30);
    create_unimplemented_device("EXTI",   0x40010400, 0x400);
    create_unimplemented_device("DBGMCU", 0xE0042000, 0x400);
    /*
     * Catch-all for the rest of the peripheral space. Mapped below the named
     * unimplemented devices (which use priority -1000), so those keep their
     * names in the log.
     */
    dev = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);
    qdev_prop_set_string(dev, "name", "peripherals");
    qdev_prop_set_uint64(dev, "size", 0x20000000);
    busdev = SYS_BUS_DEVICE(dev);
    if (!sysbus_realize_and_unref(busdev, errp)) {
        return;
    }
    sysbus_mmio_map_overlap(busdev, 0, 0x40000000, -1001);
}

static void stm32g431_soc_class_init(ObjectClass *klass, const void *data)
{
    DEVICE_CLASS(klass)->realize = stm32g431_soc_realize;
    /* No vmstate or reset method: all state is in child objects */
}

/* ---- machine "stm32g431": the bare chip ----------------------------------------- */

#define TYPE_STM32G431_MACHINE MACHINE_TYPE_NAME("stm32g431")
OBJECT_DECLARE_SIMPLE_TYPE(Stm32g431MachineState, STM32G431_MACHINE)

struct Stm32g431MachineState {
    MachineState parent_obj;

    Stm32g431SocState soc;
    Clock *sysclk;
};

static void stm32g431_machine_init(MachineState *machine)
{
    Stm32g431MachineState *s = STM32G431_MACHINE(machine);

    /* Fixed-frequency clock: no migration state */
    s->sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(s->sysclk, SYSCLK_FRQ);

    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_STM32G431_SOC);
    qdev_connect_clock_in(DEVICE(&s->soc), "sysclk", s->sysclk);
    sysbus_realize(SYS_BUS_DEVICE(&s->soc), &error_fatal);

    armv7m_load_kernel(s->soc.armv7m.cpu, machine->kernel_filename, FLASH_BASE_ADDRESS, FLASH_SIZE);
}

static void stm32g431_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char *const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m4"),
        NULL
    };

    mc->desc = "STM32G431xB (Cortex-M4, bare chip; console USART1)";
    mc->init = stm32g431_machine_init;
    mc->valid_cpu_types = valid_cpu_types;
    /* SRAM is part of the SoC */
    mc->default_ram_size = 0;
}

static const TypeInfo stm32g431_types[] = {
    {
        .name          = TYPE_STM32G431_SOC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32g431SocState),
        .instance_init = stm32g431_soc_initfn,
        .class_init    = stm32g431_soc_class_init,
    },
    {
        .name          = TYPE_STM32G431_MACHINE,
        .parent        = TYPE_MACHINE,
        .instance_size = sizeof(Stm32g431MachineState),
        .class_init    = stm32g431_machine_class_init,
        .interfaces    = arm_machine_interfaces,
    },
};

DEFINE_TYPES(stm32g431_types)
