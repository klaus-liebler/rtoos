/*
 * STM32G4 reset and clock control, power control and flash interface (register models)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0440, register layout and reset values from the STM32G431 SVD (v1.8).
 *
 * All three are register files with the hardware's reset values and the status flags firmware
 * polls during clock configuration (must match Renode's stm32g431.repl):
 *   - RCC_CR: HSIRDY, HSERDY, PLLRDY follow HSION, HSEON, PLLON immediately (HSI16 runs at
 *     reset: HSIRDY set); RCC_CFGR.SWS mirrors SW (the clock switch completes at once);
 *     RCC_CRRCR.HSI48RDY follows HSI48ON.
 *   - PWR: a new voltage range is ready at once (SR2.VOSF reads 0).
 *   - FLASH_ACR: a new latency is active at once (reads back as written). All other FLASH
 *     registers read 0 (not busy, no errors).
 * Clock frequencies are not derived from the registers: the SoC feeds a fixed SYSCLK to the
 * CPU and peripherals (see stm32g431_soc.c).
 */

#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/misc/stm32g4_sysctl.h"

#define RCC_CR    0x000
#define RCC_CFGR  0x008
#define RCC_CRRCR 0x098
#define CFGR_SW   0x3u
#define CFGR_SWS_SHIFT 2

#define PWR_SR2   0x014

#define FLASH_ACR_RESET 0x00040601u

/* ON bit positions in RCC_CR and their RDY flags: HSI16, HSE, PLL */
static const struct {
    unsigned on, rdy;
} cr_flags[] = { { 8, 10 }, { 16, 17 }, { 24, 25 } };

static const struct {
    hwaddr offset;
    uint32_t value;
} stm32g4_rcc_reset_values[] = {
    { 0x000, 0x00000563 }, /* CR: SVD reset 0x63 + HSION, HSIRDY (HSI16 runs) */
    { 0x004, 0x40000000 }, /* ICSCR */
    { 0x008, 0x00000005 }, /* CFGR: SYSCLK = HSI16 */
    { 0x00C, 0x00001000 }, /* PLLCFGR */
    { 0x048, 0x00000100 }, /* AHB1ENR */
    { 0x058, 0x00000400 }, /* APB1ENR1 */
    { 0x068, 0x0000130F }, /* AHB1SMENR */
    { 0x06C, 0x050F667F }, /* AHB2SMENR */
    { 0x070, 0x00000101 }, /* AHB3SMENR */
    { 0x078, 0xD2FECD3F }, /* APB1SMENR1 */
    { 0x07C, 0x00000103 }, /* APB1SMENR2 */
    { 0x080, 0x0437F801 }, /* APB2SMENR */
    { 0x094, 0x0C000000 }, /* CSR */
};

static const struct {
    hwaddr offset;
    uint32_t value;
} stm32g4_pwr_reset_values[] = {
    { 0x000, 0x00000200 }, /* CR1: range 1 */
    { 0x008, 0x00008000 }, /* CR3 */
    { 0x080, 0x00000100 }, /* CR5: R1MODE = 1 (normal) */
};

/* ---- RCC ------------------------------------------------------------------ */

static void stm32g4_rcc_reset_hold(Object *obj, ResetType type)
{
    Stm32g4RccState *s = STM32G4_RCC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    for (size_t i = 0; i < ARRAY_SIZE(stm32g4_rcc_reset_values); i++) {
        s->regs[stm32g4_rcc_reset_values[i].offset / 4] = stm32g4_rcc_reset_values[i].value;
    }
}

static uint64_t stm32g4_rcc_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32g4RccState *s = opaque;

    return s->regs[addr / 4];
}

static void stm32g4_rcc_write(void *opaque, hwaddr addr, uint64_t val64, unsigned size)
{
    Stm32g4RccState *s = opaque;
    uint32_t value = val64;

    switch (addr) {
    case RCC_CR:
        for (size_t i = 0; i < ARRAY_SIZE(cr_flags); i++) {
            if (value & (1u << cr_flags[i].on)) {
                value |= 1u << cr_flags[i].rdy;
            } else {
                value &= ~(1u << cr_flags[i].rdy);
            }
        }
        break;
    case RCC_CFGR:
        value = (value & ~(CFGR_SW << CFGR_SWS_SHIFT)) | ((value & CFGR_SW) << CFGR_SWS_SHIFT);
        break;
    case RCC_CRRCR:
        value = (value & 1u) ? (value | 2u) : (value & ~2u);
        break;
    default:
        break;
    }
    s->regs[addr / 4] = value;
}

/* ---- PWR ------------------------------------------------------------------ */

static void stm32g4_pwr_reset_hold(Object *obj, ResetType type)
{
    Stm32g4PwrState *s = STM32G4_PWR(obj);

    memset(s->regs, 0, sizeof(s->regs));
    for (size_t i = 0; i < ARRAY_SIZE(stm32g4_pwr_reset_values); i++) {
        s->regs[stm32g4_pwr_reset_values[i].offset / 4] = stm32g4_pwr_reset_values[i].value;
    }
}

static uint64_t stm32g4_pwr_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32g4PwrState *s = opaque;

    return addr == PWR_SR2 ? 0 : s->regs[addr / 4];
}

static void stm32g4_pwr_write(void *opaque, hwaddr addr, uint64_t val64, unsigned size)
{
    Stm32g4PwrState *s = opaque;

    s->regs[addr / 4] = val64;
}

/* ---- FLASH interface ---------------------------------------------------------- */

static void stm32g4_flash_reset_hold(Object *obj, ResetType type)
{
    Stm32g4FlashState *s = STM32G4_FLASH(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = FLASH_ACR_RESET;
}

static uint64_t stm32g4_flash_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32g4FlashState *s = opaque;

    return addr == 0 ? s->regs[0] : 0;
}

static void stm32g4_flash_write(void *opaque, hwaddr addr, uint64_t val64, unsigned size)
{
    Stm32g4FlashState *s = opaque;

    if (addr == 0) {
        s->regs[0] = val64;
    }
}

/* ---- QOM ------------------------------------------------------------------ */

#define STM32G4_OPS(dev)                                         \
    static const MemoryRegionOps dev##_ops = {                   \
        .read = dev##_read,                                      \
        .write = dev##_write,                                    \
        .endianness = DEVICE_LITTLE_ENDIAN,                       \
        .valid = { .min_access_size = 4, .max_access_size = 4 },  \
        .impl = { .min_access_size = 4, .max_access_size = 4 },   \
    }
STM32G4_OPS(stm32g4_rcc);
STM32G4_OPS(stm32g4_pwr);
STM32G4_OPS(stm32g4_flash);

#define STM32G4_DEVICE(dev, State, TYPE, CAST)                                       \
    static void dev##_init(Object *obj)                                              \
    {                                                                                 \
        State *s = CAST(obj);                                                         \
        memory_region_init_io(&s->mmio, obj, &dev##_ops, s, TYPE, STM32G4_SYSCTL_SIZE); \
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);                              \
    }                                                                                 \
    static const VMStateDescription vmstate_##dev = {                                \
        .name = TYPE,                                                                 \
        .version_id = 1,                                                              \
        .minimum_version_id = 1,                                                      \
        .fields = (const VMStateField[]) {                                            \
            VMSTATE_UINT32_ARRAY(regs, State, STM32G4_SYSCTL_SIZE / 4),               \
            VMSTATE_END_OF_LIST()                                                     \
        }                                                                             \
    };                                                                                \
    static void dev##_class_init(ObjectClass *klass, const void *data)               \
    {                                                                                 \
        RESETTABLE_CLASS(klass)->phases.hold = dev##_reset_hold;                     \
        DEVICE_CLASS(klass)->vmsd = &vmstate_##dev;                                  \
    }
STM32G4_DEVICE(stm32g4_rcc, Stm32g4RccState, TYPE_STM32G4_RCC, STM32G4_RCC)
STM32G4_DEVICE(stm32g4_pwr, Stm32g4PwrState, TYPE_STM32G4_PWR, STM32G4_PWR)
STM32G4_DEVICE(stm32g4_flash, Stm32g4FlashState, TYPE_STM32G4_FLASH, STM32G4_FLASH)

static const TypeInfo stm32g4_sysctl_types[] = {
    {
        .name          = TYPE_STM32G4_RCC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32g4RccState),
        .instance_init = stm32g4_rcc_init,
        .class_init    = stm32g4_rcc_class_init,
    },
    {
        .name          = TYPE_STM32G4_PWR,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32g4PwrState),
        .instance_init = stm32g4_pwr_init,
        .class_init    = stm32g4_pwr_class_init,
    },
    {
        .name          = TYPE_STM32G4_FLASH,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32g4FlashState),
        .instance_init = stm32g4_flash_init,
        .class_init    = stm32g4_flash_class_init,
    },
};

DEFINE_TYPES(stm32g4_sysctl_types)
