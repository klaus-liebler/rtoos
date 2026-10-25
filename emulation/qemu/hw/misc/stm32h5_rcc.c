/*
 * STM32H5 reset and clock control (register model)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0481, register layout and reset values from the STM32H563
 * SVD (v1.8).
 *
 * The model is a register file with the hardware's reset values and the status
 * flags firmware polls during clock configuration:
 *   - RCC_CR: every xxxRDY flag follows its xxxON bit immediately (HSI, CSI,
 *     HSI48, HSE, PLL1..3); HSIDIVF reads 1 (divider change applied).
 *   - RCC_CFGR1: SWS mirrors SW (clock switch completes immediately).
 * Clock frequencies are not derived from the registers: the SoC feeds a fixed
 * SYSCLK to the CPU and peripherals (see stm32h5_soc.c).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "hw/misc/stm32h5_rcc.h"

#define A_CR    0x000
#define A_CFGR1 0x01C

#define CR_HSIDIVF  (1u << 5)
#define CFGR1_SW    0x7u
#define CFGR1_SWS_SHIFT 3

/* ON bit positions in RCC_CR; the RDY flag is always the next bit. */
static const unsigned cr_on_bits[] = { 0, 8, 12, 16, 24, 26, 28 };

static const struct {
    hwaddr offset;
    uint32_t value;
} stm32h5_rcc_reset_values[] = {
    { 0x000, 0x00000023 }, /* CR */
    { 0x010, 0x00400000 }, /* HSICFGR */
    { 0x018, 0x00200000 }, /* CSICFGR */
    { 0x034, 0x01010280 }, /* PLL1DIVR */
    { 0x03C, 0x01010280 }, /* PLL2DIVR */
    { 0x044, 0x01010280 }, /* PLL3DIVR */
    { 0x088, 0xD0000100 }, /* AHB1ENR */
    { 0x08C, 0xC0000000 }, /* AHB2ENR */
    { 0x0B0, 0xF13AD103 }, /* AHB1LPENR */
    { 0x0B4, 0xC01F1DFF }, /* AHB2LPENR */
    { 0x0BC, 0x00111880 }, /* AHB4LPENR */
    { 0x0C4, 0xDFFEC9FF }, /* APB1LLPENR */
    { 0x0C8, 0x4080022B }, /* APB1HLPENR */
    { 0x0CC, 0x017F7800 }, /* APB2LPENR */
    { 0x0D0, 0x0030F9E2 }, /* APB3LPENR */
    { 0x0F4, 0x0C000000 }, /* RSR */
};

static void stm32h5_rcc_reset_hold(Object *obj, ResetType type)
{
    Stm32h5RccState *s = STM32H5_RCC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    for (size_t i = 0; i < ARRAY_SIZE(stm32h5_rcc_reset_values); i++) {
        s->regs[stm32h5_rcc_reset_values[i].offset / 4] =
            stm32h5_rcc_reset_values[i].value;
    }
}

static uint64_t stm32h5_rcc_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32h5RccState *s = opaque;

    return s->regs[addr / 4];
}

static void stm32h5_rcc_write(void *opaque, hwaddr addr, uint64_t val64,
                              unsigned size)
{
    Stm32h5RccState *s = opaque;
    uint32_t value = val64;

    switch (addr) {
    case A_CR:
        for (size_t i = 0; i < ARRAY_SIZE(cr_on_bits); i++) {
            const uint32_t on = 1u << cr_on_bits[i];
            if (value & on) {
                value |= on << 1;
            } else {
                value &= ~(on << 1);
            }
        }
        value |= CR_HSIDIVF;
        break;
    case A_CFGR1:
        value = (value & ~(CFGR1_SW << CFGR1_SWS_SHIFT)) |
                ((value & CFGR1_SW) << CFGR1_SWS_SHIFT);
        break;
    default:
        break;
    }
    s->regs[addr / 4] = value;
}

static const MemoryRegionOps stm32h5_rcc_ops = {
    .read = stm32h5_rcc_read,
    .write = stm32h5_rcc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void stm32h5_rcc_init(Object *obj)
{
    Stm32h5RccState *s = STM32H5_RCC(obj);

    memory_region_init_io(&s->mmio, obj, &stm32h5_rcc_ops, s,
                          TYPE_STM32H5_RCC, STM32H5_RCC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const VMStateDescription vmstate_stm32h5_rcc = {
    .name = TYPE_STM32H5_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Stm32h5RccState, STM32H5_RCC_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32h5_rcc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = stm32h5_rcc_reset_hold;
    dc->vmsd = &vmstate_stm32h5_rcc;
}

static const TypeInfo stm32h5_rcc_types[] = {
    {
        .name          = TYPE_STM32H5_RCC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h5RccState),
        .instance_init = stm32h5_rcc_init,
        .class_init    = stm32h5_rcc_class_init,
    },
};

DEFINE_TYPES(stm32h5_rcc_types)
