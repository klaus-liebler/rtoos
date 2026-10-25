/*
 * STM32H563 SoC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0481, memory map and interrupt numbers from the STM32H563
 * SVD (v1.8).
 *
 * Modelled:
 *   - Cortex-M33 with FPU, DSP, MPU (8 regions per security state), TrustZone
 *     (the CPU boots Secure; with SAU disabled all memory is Secure)
 *   - 2 MB flash at 0x08000000 (boot: VTOR = 0x08000000), 640 KB SRAM
 *   - DWT cycle counter (CYCCNT at SYSCLK; QEMU's core leaves DWT RAZ/WI)
 *   - RCC (register model with ready/status flags), PWR voltage scaling and
 *     FLASH_ACR (register models: new settings are ready at once)
 *   - GPIOA..I and USART1..3 (STM32L4x5 models, register-compatible)
 * Everything else in the peripheral space is an unimplemented device: reads
 * return 0, accesses are logged with "-d unimp". A read of 0 means "not busy"
 * for the ICACHE/DCACHE status registers, so drivers run unchanged.
 *
 * Clocks: fixed SYSCLK from the board (PLL1 250 MHz); SysTick reference clock =
 * SYSCLK / 8. The RCC registers do not change emulated frequencies.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "hw/arm/stm32h5_soc.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/misc/unimp.h"

#define FLASH_BASE_ADDRESS 0x08000000
#define FLASH_SIZE         (2 * MiB)
#define SRAM_BASE_ADDRESS  0x20000000
#define SRAM_SIZE          (640 * KiB)   /* SRAM1 256K + SRAM2 64K + SRAM3 320K */

#define RCC_BASE_ADDRESS   0x44020C00
#define PWR_BASE_ADDRESS   0x44020800
#define FLASH_IF_ADDRESS   0x40022000

#define NUM_IRQ            131
#define NUM_PRIO_BITS      4

static const struct {
    hwaddr addr;
    uint32_t moder_reset;
    uint32_t ospeedr_reset;
    uint32_t pupdr_reset;
} stm32h5_gpio_cfg[STM32H5_NUM_GPIOS] = {
    { 0x42020000, 0xABFFFFFF, 0x0C000000, 0x64000000 },
    { 0x42020400, 0xFFFFFEBF, 0x000000C0, 0x00000100 },
    { 0x42020800, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x42020C00, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x42021000, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x42021400, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x42021800, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x42021C00, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x42022000, 0xFFFFFFFF, 0x00000000, 0x00000000 },
};

static const struct {
    hwaddr addr;
    int irq;
} stm32h5_usart_cfg[STM32H5_NUM_USARTS] = {
    { 0x40013800, 58 }, /* USART1 */
    { 0x40004400, 59 }, /* USART2 */
    { 0x40004800, 60 }, /* USART3 */
};

static void stm32h5_soc_initfn(Object *obj)
{
    Stm32h5SocState *s = STM32H563_SOC(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
    object_initialize_child(obj, "dwt", &s->dwt, TYPE_ARMV8M_DWT);
    object_initialize_child(obj, "rcc", &s->rcc, TYPE_STM32H5_RCC);
    object_initialize_child(obj, "pwr", &s->pwr, TYPE_STM32H5_PWR);
    object_initialize_child(obj, "flash-if", &s->flash_if, TYPE_STM32H5_FLASH);
    for (unsigned i = 0; i < STM32H5_NUM_GPIOS; i++) {
        g_autofree char *name = g_strdup_printf("gpio%c", 'a' + i);
        object_initialize_child(obj, name, &s->gpio[i], TYPE_STM32L4X5_GPIO);
    }
    for (unsigned i = 0; i < STM32H5_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i],
                                TYPE_STM32L4X5_USART);
    }

    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

static void stm32h5_soc_realize(DeviceState *dev_soc, Error **errp)
{
    Stm32h5SocState *s = STM32H563_SOC(dev_soc);
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
    if (s->console_usart < 1 || s->console_usart > STM32H5_NUM_USARTS) {
        error_setg(errp, "console-usart must be 1..%d", STM32H5_NUM_USARTS);
        return;
    }
    /* SysTick external reference: HCLK / 8 */
    clock_set_mul_div(s->refclk, 8, 1);
    clock_set_source(s->refclk, s->sysclk);

    /* Memory */
    if (!memory_region_init_rom(&s->flash, OBJECT(dev_soc), "flash",
                                FLASH_SIZE, errp)) {
        return;
    }
    memory_region_add_subregion(system_memory, FLASH_BASE_ADDRESS, &s->flash);

    if (!memory_region_init_ram(&s->sram, OBJECT(dev_soc), "sram", SRAM_SIZE,
                                errp)) {
        return;
    }
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, &s->sram);

    /* CPU */
    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", NUM_IRQ);
    qdev_prop_set_uint32(armv7m, "num-prio-bits", NUM_PRIO_BITS);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m33"));
    qdev_prop_set_uint32(armv7m, "init-svtor", FLASH_BASE_ADDRESS);
    qdev_prop_set_uint32(armv7m, "init-nsvtor", FLASH_BASE_ADDRESS);
    qdev_prop_set_uint32(armv7m, "mpu-s-regions", 8);
    qdev_prop_set_uint32(armv7m, "mpu-ns-regions", 8);
    qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", s->refclk);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
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
    busdev = SYS_BUS_DEVICE(&s->rcc);
    if (!sysbus_realize(busdev, errp)) {
        return;
    }
    sysbus_mmio_map(busdev, 0, RCC_BASE_ADDRESS);
    busdev = SYS_BUS_DEVICE(&s->pwr);
    if (!sysbus_realize(busdev, errp)) {
        return;
    }
    sysbus_mmio_map(busdev, 0, PWR_BASE_ADDRESS);
    busdev = SYS_BUS_DEVICE(&s->flash_if);
    if (!sysbus_realize(busdev, errp)) {
        return;
    }
    sysbus_mmio_map(busdev, 0, FLASH_IF_ADDRESS);

    /* GPIO */
    for (unsigned i = 0; i < STM32H5_NUM_GPIOS; i++) {
        g_autofree char *name = g_strdup_printf("%c", 'A' + i);
        dev = DEVICE(&s->gpio[i]);
        qdev_prop_set_string(dev, "name", name);
        qdev_prop_set_uint32(dev, "mode-reset", stm32h5_gpio_cfg[i].moder_reset);
        qdev_prop_set_uint32(dev, "ospeed-reset",
                             stm32h5_gpio_cfg[i].ospeedr_reset);
        qdev_prop_set_uint32(dev, "pupd-reset", stm32h5_gpio_cfg[i].pupdr_reset);
        qdev_connect_clock_in(dev, "clk", s->sysclk);
        busdev = SYS_BUS_DEVICE(dev);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, stm32h5_gpio_cfg[i].addr);
    }

    /* USART: the console USART gets serial_hd(0), the others follow in order */
    for (unsigned i = 0, next_serial = 1; i < STM32H5_NUM_USARTS; i++) {
        const bool console = (i + 1 == s->console_usart);
        dev = DEVICE(&s->usart[i]);
        qdev_prop_set_chr(dev, "chardev",
                          serial_hd(console ? 0 : next_serial++));
        qdev_connect_clock_in(dev, "clk", s->sysclk);
        busdev = SYS_BUS_DEVICE(dev);
        if (!sysbus_realize(busdev, errp)) {
            return;
        }
        sysbus_mmio_map(busdev, 0, stm32h5_usart_cfg[i].addr);
        sysbus_connect_irq(busdev, 0,
                           qdev_get_gpio_in(armv7m, stm32h5_usart_cfg[i].irq));
    }

    /* Not modelled: named for readable "-d unimp" logs */
    create_unimplemented_device("ICACHE",  0x40030400, 0x400);
    create_unimplemented_device("DCACHE1", 0x40031400, 0x400);
    create_unimplemented_device("GTZC1",   0x40032400, 0x1000);
    create_unimplemented_device("SBS",     0x44000400, 0x400);
    create_unimplemented_device("EXTI",    0x44022000, 0x400);
    create_unimplemented_device("DBGMCU",  0x44024000, 0x400);
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

static const Property stm32h5_soc_properties[] = {
    DEFINE_PROP_UINT32("console-usart", Stm32h5SocState, console_usart, 1),
};

static void stm32h5_soc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32h5_soc_realize;
    device_class_set_props(dc, stm32h5_soc_properties);
    /* No vmstate or reset method: all state is in child objects */
}

static const TypeInfo stm32h5_soc_types[] = {
    {
        .name          = TYPE_STM32H563_SOC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h5SocState),
        .instance_init = stm32h5_soc_initfn,
        .class_init    = stm32h5_soc_class_init,
    },
};

DEFINE_TYPES(stm32h5_soc_types)
