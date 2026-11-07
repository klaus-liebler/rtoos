/*
 * STM32H7 reset and clock control, power control and flash interface (register models)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0433, register layout and reset values from the STM32H743 SVD (v2.4).
 *
 * All three are register files with the hardware's reset values and the status flags firmware
 * polls during clock configuration (must match Renode's stm32h743.repl):
 *   - RCC_CR: HSIRDY, CSIRDY, RC48RDY, HSERDY, PLL1RDY..PLL3RDY follow their ON bits
 *     immediately; HSIDIVF, D1CKRDY and D2CKRDY read 1 (the HSI divider and the domain clocks
 *     are ready at once); RCC_CFGR.SWS mirrors SW (the clock switch completes at once).
 *   - PWR: a new supply configuration and a new voltage scale are ready at once
 *     (CSR1.ACTVOSRDY and D3CR.VOSRDY read 1; CSR1.ACTVOS follows D3CR.VOS).
 *   - FLASH_ACR: a new latency is active at once (reads back as written). All other FLASH
 *     registers read 0 (not busy, no errors).
 * Clock frequencies are not derived from the registers: the SoC feeds a fixed SYSCLK to the
 * CPU and peripherals (see stm32h743_soc.c).
 */

#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/misc/stm32h7_sysctl.h"

#define RCC_CR    0x000
#define RCC_CFGR  0x010
#define CFGR_SW   0x7u
#define CFGR_SWS_SHIFT 3
/* RCC_CR flags that read 1 once written or at once: HSIDIVF, D1CKRDY, D2CKRDY */
#define CR_ALWAYS_READY ((1u << 5) | (1u << 14) | (1u << 15))

#define PWR_CSR1  0x004
#define PWR_D3CR  0x018
#define PWR_ACTVOSRDY (1u << 13)
#define PWR_VOSRDY    (1u << 13)
#define PWR_VOS_MASK  (3u << 14)

#define FLASH_ACR_RESET 0x00000037u

/* ON bit positions in RCC_CR and their RDY flags: HSI, CSI, RC48, HSE, PLL1, PLL2, PLL3 */
static const struct {
    unsigned on, rdy;
} cr_flags[] = { { 0, 2 }, { 7, 8 }, { 12, 13 }, { 16, 17 }, { 24, 25 }, { 26, 27 }, { 28, 29 } };

static const struct {
    hwaddr offset;
    uint32_t value;
} stm32h7_rcc_reset_values[] = {
    /*
     * CR: HSI on and ready, HSIDIV = 1, HSIDIVF, D1/D2 clocks ready (the state after reset; the
     * SVD's reset value 0x83 does not describe the ready flags)
     */
    { 0x000, 0x0000C025 },
    { 0x028, 0x02020200 }, /* PLLCKSELR: HSI, DIVM1..3 = 32 */
    { 0x02C, 0x01FF0000 }, /* PLLCFGR */
    { 0x030, 0x01010280 }, /* PLL1DIVR */
    { 0x038, 0x01010280 }, /* PLL2DIVR */
    { 0x040, 0x01010280 }, /* PLL3DIVR */
};

static const struct {
    hwaddr offset;
    uint32_t value;
} stm32h7_pwr_reset_values[] = {
    { 0x004, 0x00004000 }, /* CSR1: ACTVOS = VOS3 */
    { 0x00C, 0x00000006 }, /* CR3: LDOEN, SCUEN */
    { 0x018, 0x00004000 }, /* D3CR: VOS3 */
};

/* ---- RCC ------------------------------------------------------------------ */

static void stm32h7_rcc_reset_hold(Object *obj, ResetType type)
{
    Stm32h7RccState *s = STM32H7_RCC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    for (size_t i = 0; i < ARRAY_SIZE(stm32h7_rcc_reset_values); i++) {
        s->regs[stm32h7_rcc_reset_values[i].offset / 4] = stm32h7_rcc_reset_values[i].value;
    }
}

static uint64_t stm32h7_rcc_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32h7RccState *s = opaque;

    return s->regs[addr / 4];
}

static void stm32h7_rcc_write(void *opaque, hwaddr addr, uint64_t val64, unsigned size)
{
    Stm32h7RccState *s = opaque;
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
        value |= CR_ALWAYS_READY;
        break;
    case RCC_CFGR:
        value = (value & ~(CFGR_SW << CFGR_SWS_SHIFT)) | ((value & CFGR_SW) << CFGR_SWS_SHIFT);
        break;
    default:
        break;
    }
    s->regs[addr / 4] = value;
}

/* ---- PWR ------------------------------------------------------------------ */

static void stm32h7_pwr_reset_hold(Object *obj, ResetType type)
{
    Stm32h7PwrState *s = STM32H7_PWR(obj);

    memset(s->regs, 0, sizeof(s->regs));
    for (size_t i = 0; i < ARRAY_SIZE(stm32h7_pwr_reset_values); i++) {
        s->regs[stm32h7_pwr_reset_values[i].offset / 4] = stm32h7_pwr_reset_values[i].value;
    }
}

static uint64_t stm32h7_pwr_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32h7PwrState *s = opaque;

    switch (addr) {
    case PWR_CSR1:
        return (s->regs[PWR_CSR1 / 4] & ~PWR_VOS_MASK) | (s->regs[PWR_D3CR / 4] & PWR_VOS_MASK) | PWR_ACTVOSRDY;
    case PWR_D3CR:
        return s->regs[PWR_D3CR / 4] | PWR_VOSRDY;
    default:
        return s->regs[addr / 4];
    }
}

static void stm32h7_pwr_write(void *opaque, hwaddr addr, uint64_t val64, unsigned size)
{
    Stm32h7PwrState *s = opaque;

    s->regs[addr / 4] = val64;
}

/* ---- FLASH interface ---------------------------------------------------------- */

static void stm32h7_flash_reset_hold(Object *obj, ResetType type)
{
    Stm32h7FlashState *s = STM32H7_FLASH(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = FLASH_ACR_RESET;
}

static uint64_t stm32h7_flash_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32h7FlashState *s = opaque;

    return addr == 0 ? s->regs[0] : 0;
}

static void stm32h7_flash_write(void *opaque, hwaddr addr, uint64_t val64, unsigned size)
{
    Stm32h7FlashState *s = opaque;

    if (addr == 0) {
        s->regs[0] = val64;
    }
}

/* ---- QOM ------------------------------------------------------------------ */

#define STM32H7_OPS(dev)                                         \
    static const MemoryRegionOps dev##_ops = {                   \
        .read = dev##_read,                                      \
        .write = dev##_write,                                    \
        .endianness = DEVICE_LITTLE_ENDIAN,                       \
        .valid = { .min_access_size = 4, .max_access_size = 4 },  \
        .impl = { .min_access_size = 4, .max_access_size = 4 },   \
    }
STM32H7_OPS(stm32h7_rcc);
STM32H7_OPS(stm32h7_pwr);
STM32H7_OPS(stm32h7_flash);

#define STM32H7_DEVICE(dev, State, TYPE, CAST)                                       \
    static void dev##_init(Object *obj)                                              \
    {                                                                                 \
        State *s = CAST(obj);                                                         \
        memory_region_init_io(&s->mmio, obj, &dev##_ops, s, TYPE, STM32H7_SYSCTL_SIZE); \
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);                              \
    }                                                                                 \
    static const VMStateDescription vmstate_##dev = {                                \
        .name = TYPE,                                                                 \
        .version_id = 1,                                                              \
        .minimum_version_id = 1,                                                      \
        .fields = (const VMStateField[]) {                                            \
            VMSTATE_UINT32_ARRAY(regs, State, STM32H7_SYSCTL_SIZE / 4),               \
            VMSTATE_END_OF_LIST()                                                     \
        }                                                                             \
    };                                                                                \
    static void dev##_class_init(ObjectClass *klass, const void *data)               \
    {                                                                                 \
        RESETTABLE_CLASS(klass)->phases.hold = dev##_reset_hold;                     \
        DEVICE_CLASS(klass)->vmsd = &vmstate_##dev;                                  \
    }
STM32H7_DEVICE(stm32h7_rcc, Stm32h7RccState, TYPE_STM32H7_RCC, STM32H7_RCC)
STM32H7_DEVICE(stm32h7_pwr, Stm32h7PwrState, TYPE_STM32H7_PWR, STM32H7_PWR)
STM32H7_DEVICE(stm32h7_flash, Stm32h7FlashState, TYPE_STM32H7_FLASH, STM32H7_FLASH)

static const TypeInfo stm32h7_sysctl_types[] = {
    {
        .name          = TYPE_STM32H7_RCC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h7RccState),
        .instance_init = stm32h7_rcc_init,
        .class_init    = stm32h7_rcc_class_init,
    },
    {
        .name          = TYPE_STM32H7_PWR,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h7PwrState),
        .instance_init = stm32h7_pwr_init,
        .class_init    = stm32h7_pwr_class_init,
    },
    {
        .name          = TYPE_STM32H7_FLASH,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h7FlashState),
        .instance_init = stm32h7_flash_init,
        .class_init    = stm32h7_flash_class_init,
    },
};

DEFINE_TYPES(stm32h7_sysctl_types)
