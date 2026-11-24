/*
 * STM32F1 reset and clock control and flash interface (register models)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0008 rev 21, register layout and reset values of RCC (7.3) and FLASH_ACR.
 *
 * Both are register files with the hardware's reset values and the status flags firmware polls
 * during clock configuration (must match Renode's stm32f103.repl):
 *   - RCC_CR: HSIRDY, HSERDY, PLLRDY follow HSION, HSEON, PLLON immediately (the HSI runs at
 *     reset: HSIRDY set); RCC_CFGR.SWS mirrors SW (the clock switch completes at once);
 *     RCC_CSR.LSIRDY follows LSION, RMVF clears the reset flags (reset value: PORRSTF, PINRSTF).
 *   - FLASH_ACR: a new latency is active at once (reads back as written). All other FLASH
 *     registers read 0 (not busy, no errors).
 * Clock frequencies are not derived from the registers: the SoC feeds a fixed SYSCLK to the
 * CPU and peripherals (see stm32f103_soc.c).
 */

#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/misc/stm32f1_sysctl.h"

#define RCC_CR    0x000
#define RCC_CFGR  0x004
#define RCC_CSR   0x024
#define CFGR_SW   0x3u
#define CFGR_SWS_SHIFT 2
#define CSR_LSION 0x1u
#define CSR_LSIRDY 0x2u
#define CSR_RMVF  (1u << 24)

#define FLASH_ACR_RESET 0x00000030u

/* ON bit positions in RCC_CR and their RDY flags: HSI, HSE, PLL */
static const struct {
    unsigned on, rdy;
} cr_flags[] = { { 0, 1 }, { 16, 17 }, { 24, 25 } };

static const struct {
    hwaddr offset;
    uint32_t value;
} stm32f1_rcc_reset_values[] = {
    { 0x000, 0x00000083 }, /* CR: HSITRIM 16, HSION, HSIRDY */
    { 0x014, 0x00000014 }, /* AHBENR: SRAMEN, FLITFEN */
    { 0x024, 0x0C000000 }, /* CSR: PORRSTF, PINRSTF */
};

/* ---- RCC ------------------------------------------------------------------ */

static void stm32f1_rcc_reset_hold(Object *obj, ResetType type)
{
    Stm32f1RccState *s = STM32F1_RCC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    for (size_t i = 0; i < ARRAY_SIZE(stm32f1_rcc_reset_values); i++) {
        s->regs[stm32f1_rcc_reset_values[i].offset / 4] = stm32f1_rcc_reset_values[i].value;
    }
}

static uint64_t stm32f1_rcc_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32f1RccState *s = opaque;

    return s->regs[addr / 4];
}

static void stm32f1_rcc_write(void *opaque, hwaddr addr, uint64_t val64, unsigned size)
{
    Stm32f1RccState *s = opaque;
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
    case RCC_CSR:
        value = (value & CSR_LSION) ? (value | CSR_LSIRDY) : (value & ~CSR_LSIRDY);
        if (value & CSR_RMVF) {
            value &= 0x00FFFFFFu;
        }
        break;
    default:
        break;
    }
    s->regs[addr / 4] = value;
}

/* ---- FLASH interface ---------------------------------------------------------- */

static void stm32f1_flash_reset_hold(Object *obj, ResetType type)
{
    Stm32f1FlashState *s = STM32F1_FLASH(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = FLASH_ACR_RESET;
}

static uint64_t stm32f1_flash_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32f1FlashState *s = opaque;

    return addr == 0 ? s->regs[0] : 0;
}

static void stm32f1_flash_write(void *opaque, hwaddr addr, uint64_t val64, unsigned size)
{
    Stm32f1FlashState *s = opaque;

    if (addr == 0) {
        s->regs[0] = val64;
    }
}

/* ---- QOM ------------------------------------------------------------------ */

#define STM32F1_OPS(dev)                                         \
    static const MemoryRegionOps dev##_ops = {                   \
        .read = dev##_read,                                      \
        .write = dev##_write,                                    \
        .endianness = DEVICE_LITTLE_ENDIAN,                       \
        .valid = { .min_access_size = 4, .max_access_size = 4 },  \
        .impl = { .min_access_size = 4, .max_access_size = 4 },   \
    }
STM32F1_OPS(stm32f1_rcc);
STM32F1_OPS(stm32f1_flash);

#define STM32F1_DEVICE(dev, State, TYPE, CAST)                                       \
    static void dev##_init(Object *obj)                                              \
    {                                                                                 \
        State *s = CAST(obj);                                                         \
        memory_region_init_io(&s->mmio, obj, &dev##_ops, s, TYPE, STM32F1_SYSCTL_SIZE); \
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);                              \
    }                                                                                 \
    static const VMStateDescription vmstate_##dev = {                                \
        .name = TYPE,                                                                 \
        .version_id = 1,                                                              \
        .minimum_version_id = 1,                                                      \
        .fields = (const VMStateField[]) {                                            \
            VMSTATE_UINT32_ARRAY(regs, State, STM32F1_SYSCTL_SIZE / 4),               \
            VMSTATE_END_OF_LIST()                                                     \
        }                                                                             \
    };                                                                                \
    static void dev##_class_init(ObjectClass *klass, const void *data)               \
    {                                                                                 \
        RESETTABLE_CLASS(klass)->phases.hold = dev##_reset_hold;                     \
        DEVICE_CLASS(klass)->vmsd = &vmstate_##dev;                                  \
    }
STM32F1_DEVICE(stm32f1_rcc, Stm32f1RccState, TYPE_STM32F1_RCC, STM32F1_RCC)
STM32F1_DEVICE(stm32f1_flash, Stm32f1FlashState, TYPE_STM32F1_FLASH, STM32F1_FLASH)

static const TypeInfo stm32f1_sysctl_types[] = {
    {
        .name          = TYPE_STM32F1_RCC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32f1RccState),
        .instance_init = stm32f1_rcc_init,
        .class_init    = stm32f1_rcc_class_init,
    },
    {
        .name          = TYPE_STM32F1_FLASH,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32f1FlashState),
        .instance_init = stm32f1_flash_init,
        .class_init    = stm32f1_flash_class_init,
    },
};

DEFINE_TYPES(stm32f1_sysctl_types)
