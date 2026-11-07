/*
 * STM32H743 SoC and the "stm32h743" machine (the bare chip)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0433, memory map from the STM32H743 SVD (v2.4), interrupt numbers, memory
 * sizes and the core configuration from ST's stm32h743xx.h.
 *
 * Modelled:
 *   - Cortex-M7 with FPU (double precision), MPU (16 regions); the L1 cache maintenance
 *     registers are QEMU's (no-ops, no cache is modelled)
 *   - 2 MB flash at 0x08000000 (boot address BOOT_ADD0 = 0x0800: VTOR = 0x08000000 at reset),
 *     ITCM 64 KB at 0, DTCM 128 KB at 0x20000000, AXI SRAM 512 KB at 0x24000000, SRAM1-3
 *     288 KB at 0x30000000 (one region), SRAM4 64 KB at 0x38000000, backup SRAM 4 KB at
 *     0x38800000, system memory 0x1FF00000 (UID at 0x1FF1E800; zero-filled)
 *   - DWT cycle counter (CYCCNT at SYSCLK; QEMU's core leaves DWT RAZ/WI)
 *   - RCC, PWR and FLASH_ACR (register models: new settings are ready at once)
 *   - GPIOA..K and USART1..3 (STM32L4x5 models, register-compatible)
 * Everything else in the peripheral space is an unimplemented device: reads return 0,
 * accesses are logged with "-d unimp".
 *
 * Clocks: fixed SYSCLK 400 MHz (the self-test runs PLL1 at 400 MHz from the HSI, VOS1);
 * SysTick reference clock = SYSCLK / 8. The RCC registers do not change emulated frequencies.
 *
 * Usage:
 *   qemu-system-arm -M stm32h743 -nographic -kernel firmware.elf    (console: USART3)
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "hw/arm/stm32h743_soc.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/misc/unimp.h"

#define FLASH_BASE_ADDRESS    0x08000000
#define FLASH_SIZE            (2 * MiB)
#define ITCM_BASE_ADDRESS     0x00000000
#define ITCM_SIZE             (64 * KiB)
#define DTCM_BASE_ADDRESS     0x20000000
#define DTCM_SIZE             (128 * KiB)
#define AXI_SRAM_BASE_ADDRESS 0x24000000
#define AXI_SRAM_SIZE         (512 * KiB)
#define D2_SRAM_BASE_ADDRESS  0x30000000
#define D2_SRAM_SIZE          (288 * KiB)
#define SRAM4_BASE_ADDRESS    0x38000000
#define SRAM4_SIZE            (64 * KiB)
#define BACKUP_BASE_ADDRESS   0x38800000
#define BACKUP_SIZE           (4 * KiB)
#define SYSTEM_BASE_ADDRESS   0x1FF00000
#define SYSTEM_SIZE           (128 * KiB)

#define RCC_BASE_ADDRESS      0x58024400
#define PWR_BASE_ADDRESS      0x58024800
#define FLASH_IF_ADDRESS      0x52002000

#define NUM_IRQ            150   /* stm32h743xx.h: WAKEUP_PIN_IRQn = 149 is the last */
#define NUM_PRIO_BITS      4

#define SYSCLK_FRQ 400000000ULL

/*
 * GPIO reset values: GPIOA from the SVD; the SVD derives GPIOB..K from GPIOA, so GPIOB has the
 * usual STM32 debug-pin defaults (PB3 JTDO, PB4 NJTRST) and the others are all analog. Only
 * pins no firmware configures depend on them.
 */
static const struct {
    hwaddr addr;
    uint32_t moder_reset;
    uint32_t ospeedr_reset;
    uint32_t pupdr_reset;
} stm32h743_gpio_cfg[STM32H743_NUM_GPIOS] = {
    { 0x58020000, 0xABFFFFFF, 0x0C000000, 0x12100000 },
    { 0x58020400, 0xFFFFFEBF, 0x000000C0, 0x00000100 },
    { 0x58020800, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x58020C00, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x58021000, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x58021400, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x58021800, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x58021C00, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x58022000, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x58022400, 0xFFFFFFFF, 0x00000000, 0x00000000 },
    { 0x58022800, 0xFFFFFFFF, 0x00000000, 0x00000000 },
};

static const struct {
    hwaddr addr;
    int irq;
} stm32h743_usart_cfg[STM32H743_NUM_USARTS] = {
    { 0x40011000, 37 }, /* USART1 */
    { 0x40004400, 38 }, /* USART2 */
    { 0x40004800, 39 }, /* USART3 */
};

static void stm32h743_soc_initfn(Object *obj)
{
    Stm32h743SocState *s = STM32H743_SOC(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);
    object_initialize_child(obj, "dwt", &s->dwt, TYPE_ARMV8M_DWT);
    object_initialize_child(obj, "rcc", &s->rcc, TYPE_STM32H7_RCC);
    object_initialize_child(obj, "pwr", &s->pwr, TYPE_STM32H7_PWR);
    object_initialize_child(obj, "flash-if", &s->flash_if, TYPE_STM32H7_FLASH);
    for (unsigned i = 0; i < STM32H743_NUM_GPIOS; i++) {
        g_autofree char *name = g_strdup_printf("gpio%c", 'a' + i);
        object_initialize_child(obj, name, &s->gpio[i], TYPE_STM32L4X5_GPIO);
    }
    for (unsigned i = 0; i < STM32H743_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i], TYPE_STM32L4X5_USART);
    }

    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

static bool stm32h743_map(SysBusDevice *busdev, hwaddr addr, Error **errp)
{
    if (!sysbus_realize(busdev, errp)) {
        return false;
    }
    sysbus_mmio_map(busdev, 0, addr);
    return true;
}

static bool stm32h743_ram(Stm32h743SocState *s, MemoryRegion *mr, const char *name, hwaddr addr,
                          uint64_t size, Error **errp)
{
    if (!memory_region_init_ram(mr, OBJECT(s), name, size, errp)) {
        return false;
    }
    memory_region_add_subregion(get_system_memory(), addr, mr);
    return true;
}

static void stm32h743_soc_realize(DeviceState *dev_soc, Error **errp)
{
    Stm32h743SocState *s = STM32H743_SOC(dev_soc);
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
    if (s->console_usart < 1 || s->console_usart > STM32H743_NUM_USARTS) {
        error_setg(errp, "console-usart must be 1..%d", STM32H743_NUM_USARTS);
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
    if (!stm32h743_ram(s, &s->itcm, "itcm", ITCM_BASE_ADDRESS, ITCM_SIZE, errp) ||
        !stm32h743_ram(s, &s->dtcm, "dtcm", DTCM_BASE_ADDRESS, DTCM_SIZE, errp) ||
        !stm32h743_ram(s, &s->axi_sram, "axi-sram", AXI_SRAM_BASE_ADDRESS, AXI_SRAM_SIZE, errp) ||
        !stm32h743_ram(s, &s->d2_sram, "d2-sram", D2_SRAM_BASE_ADDRESS, D2_SRAM_SIZE, errp) ||
        !stm32h743_ram(s, &s->sram4, "sram4", SRAM4_BASE_ADDRESS, SRAM4_SIZE, errp) ||
        !stm32h743_ram(s, &s->backup_sram, "backup-sram", BACKUP_BASE_ADDRESS, BACKUP_SIZE, errp) ||
        !stm32h743_ram(s, &s->system, "system", SYSTEM_BASE_ADDRESS, SYSTEM_SIZE, errp)) {
        return;
    }

    /* CPU */
    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", NUM_IRQ);
    qdev_prop_set_uint32(armv7m, "num-prio-bits", NUM_PRIO_BITS);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m7"));
    /*
     * VTOR reset value: BOOT_ADD0 = 0x0800. Without the Security Extension QEMU runs the core
     * on the non-secure register bank, so init-nsvtor is the one that counts (both set).
     */
    qdev_prop_set_uint32(armv7m, "init-svtor", FLASH_BASE_ADDRESS);
    qdev_prop_set_uint32(armv7m, "init-nsvtor", FLASH_BASE_ADDRESS);
    qdev_prop_set_uint32(armv7m, "mpu-ns-regions", 16);
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
    if (!stm32h743_map(SYS_BUS_DEVICE(&s->rcc), RCC_BASE_ADDRESS, errp) ||
        !stm32h743_map(SYS_BUS_DEVICE(&s->pwr), PWR_BASE_ADDRESS, errp) ||
        !stm32h743_map(SYS_BUS_DEVICE(&s->flash_if), FLASH_IF_ADDRESS, errp)) {
        return;
    }

    /* GPIO */
    for (unsigned i = 0; i < STM32H743_NUM_GPIOS; i++) {
        g_autofree char *name = g_strdup_printf("%c", 'A' + i);
        dev = DEVICE(&s->gpio[i]);
        qdev_prop_set_string(dev, "name", name);
        qdev_prop_set_uint32(dev, "mode-reset", stm32h743_gpio_cfg[i].moder_reset);
        qdev_prop_set_uint32(dev, "ospeed-reset", stm32h743_gpio_cfg[i].ospeedr_reset);
        qdev_prop_set_uint32(dev, "pupd-reset", stm32h743_gpio_cfg[i].pupdr_reset);
        qdev_connect_clock_in(dev, "clk", s->sysclk);
        if (!stm32h743_map(SYS_BUS_DEVICE(dev), stm32h743_gpio_cfg[i].addr, errp)) {
            return;
        }
    }

    /* USART: the console USART gets serial_hd(0), the others follow in order */
    for (unsigned i = 0, next_serial = 1; i < STM32H743_NUM_USARTS; i++) {
        const bool console = (i + 1 == s->console_usart);
        dev = DEVICE(&s->usart[i]);
        qdev_prop_set_chr(dev, "chardev", serial_hd(console ? 0 : next_serial++));
        qdev_connect_clock_in(dev, "clk", s->sysclk);
        busdev = SYS_BUS_DEVICE(dev);
        if (!stm32h743_map(busdev, stm32h743_usart_cfg[i].addr, errp)) {
            return;
        }
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, stm32h743_usart_cfg[i].irq));
    }

    /* Not modelled: named for readable "-d unimp" logs */
    create_unimplemented_device("EXTI",   0x58000000, 0x400);
    create_unimplemented_device("SYSCFG", 0x58000400, 0x400);
    create_unimplemented_device("DBGMCU", 0x5C001000, 0x400);
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

static const Property stm32h743_soc_properties[] = {
    DEFINE_PROP_UINT32("console-usart", Stm32h743SocState, console_usart, 3),
};

static void stm32h743_soc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32h743_soc_realize;
    device_class_set_props(dc, stm32h743_soc_properties);
    /* No vmstate or reset method: all state is in child objects */
}

/* ---- machine "stm32h743": the bare chip ----------------------------------------- */

#define TYPE_STM32H743_MACHINE MACHINE_TYPE_NAME("stm32h743")
OBJECT_DECLARE_SIMPLE_TYPE(Stm32h743MachineState, STM32H743_MACHINE)

struct Stm32h743MachineState {
    MachineState parent_obj;

    Stm32h743SocState soc;
    Clock *sysclk;
};

static void stm32h743_machine_init(MachineState *machine)
{
    Stm32h743MachineState *s = STM32H743_MACHINE(machine);

    /* Fixed-frequency clock: no migration state */
    s->sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(s->sysclk, SYSCLK_FRQ);

    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_STM32H743_SOC);
    qdev_connect_clock_in(DEVICE(&s->soc), "sysclk", s->sysclk);
    sysbus_realize(SYS_BUS_DEVICE(&s->soc), &error_fatal);

    armv7m_load_kernel(s->soc.armv7m.cpu, machine->kernel_filename, FLASH_BASE_ADDRESS, FLASH_SIZE);
}

static void stm32h743_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char *const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m7"),
        NULL
    };

    mc->desc = "STM32H743xI (Cortex-M7, bare chip; console USART3)";
    mc->init = stm32h743_machine_init;
    mc->valid_cpu_types = valid_cpu_types;
    /* SRAM is part of the SoC */
    mc->default_ram_size = 0;
}

static const TypeInfo stm32h743_types[] = {
    {
        .name          = TYPE_STM32H743_SOC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h743SocState),
        .instance_init = stm32h743_soc_initfn,
        .class_init    = stm32h743_soc_class_init,
    },
    {
        .name          = TYPE_STM32H743_MACHINE,
        .parent        = TYPE_MACHINE,
        .instance_size = sizeof(Stm32h743MachineState),
        .class_init    = stm32h743_machine_class_init,
        .interfaces    = arm_machine_interfaces,
    },
};

DEFINE_TYPES(stm32h743_types)
