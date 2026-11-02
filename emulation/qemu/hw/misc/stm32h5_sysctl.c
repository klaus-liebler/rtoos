/*
 * STM32H5 power control (voltage scaling), flash interface (ACR) and DTS
 * temperature sensor models
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ST RM0481, register layout and reset values from the STM32H563
 * SVD (v1.8).
 *
 * Both are register files with the hardware's reset values and the status
 * firmware polls while raising the clock:
 *   - PWR_VOSCR (0x10) / PWR_VOSSR (0x14, reset 0x08): a new voltage scale is
 *     ready at once (VOSRDY = ACTVOSRDY = 1, ACTVOS mirrors VOS).
 *   - FLASH_ACR (0x00, reset 0x13): a new latency is active at once (reads back
 *     as written). All other FLASH registers read 0 (not busy, no errors).
 * Neither changes emulated timing.
 *
 * DTS, as measured on the FCU (SWD register dumps): TS1_START = 1 measures
 * continuously - TS1_RDY reads 0, DR follows the temperature; TS1_START = 0:
 * TS1_RDY = 1 (when enabled), DR keeps the last count (0 before the first
 * measurement). Fixed temperature: the FCU's sensor at about 28.6 degC
 * (FM 716.625 kHz) counted with PCLK1 = 125 MHz; factory values
 * T0VALR1 = 0x1C1B, RAMPVALR = 0x7CB of that chip. Must match Renode's
 * stm32h563.repl.
 */

#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/misc/stm32h5_sysctl.h"

#define PWR_VOSCR 0x10
#define PWR_VOSSR 0x14
#define PWR_VOSSR_RESET 0x00000008u
#define VOSSR_VOSRDY    (1u << 3)
#define VOSSR_ACTVOSRDY (1u << 13)
#define VOSSR_ACTVOS_SHIFT 14
#define VOSCR_VOS_SHIFT 4

#define FLASH_ACR 0x00
#define FLASH_ACR_RESET 0x00000013u

#define DTS_CFGR1    0x00
#define DTS_T0VALR1  0x08
#define DTS_RAMPVALR 0x10
#define DTS_DR       0x1C
#define DTS_SR       0x20
#define DTS_CFGR1_TS1_EN    (1u << 0)
#define DTS_CFGR1_TS1_START (1u << 4)
#define DTS_SR_TS1_RDY      (1u << 15)
#define DTS_FCU_T0VALR1  0x00001C1Bu
#define DTS_FCU_RAMPVALR 0x000007CBu
#define DTS_PCLK_HZ      125000000u
#define DTS_FM_HZ        716625u

static void stm32h5_pwr_reset_hold(Object *obj, ResetType type)
{
    Stm32h5PwrState *s = STM32H5_PWR(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[PWR_VOSSR / 4] = PWR_VOSSR_RESET;
}

static uint64_t stm32h5_pwr_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32h5PwrState *s = opaque;

    return s->regs[addr / 4];
}

static void stm32h5_pwr_write(void *opaque, hwaddr addr, uint64_t val64,
                              unsigned size)
{
    Stm32h5PwrState *s = opaque;
    uint32_t value = val64;

    switch (addr) {
    case PWR_VOSSR:
        return; /* read-only */
    case PWR_VOSCR:
        s->regs[PWR_VOSSR / 4] = VOSSR_VOSRDY | VOSSR_ACTVOSRDY |
            (((value >> VOSCR_VOS_SHIFT) & 3u) << VOSSR_ACTVOS_SHIFT);
        break;
    default:
        break;
    }
    s->regs[addr / 4] = value;
}

static void stm32h5_flash_reset_hold(Object *obj, ResetType type)
{
    Stm32h5FlashState *s = STM32H5_FLASH(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[FLASH_ACR / 4] = FLASH_ACR_RESET;
}

static uint64_t stm32h5_flash_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32h5FlashState *s = opaque;

    return addr == FLASH_ACR ? s->regs[addr / 4] : 0;
}

static void stm32h5_flash_write(void *opaque, hwaddr addr, uint64_t val64,
                                unsigned size)
{
    Stm32h5FlashState *s = opaque;

    s->regs[addr / 4] = val64;
}

static bool stm32h5_dts_measuring(const Stm32h5DtsState *s)
{
    const uint32_t both = DTS_CFGR1_TS1_EN | DTS_CFGR1_TS1_START;

    return (s->regs[DTS_CFGR1 / 4] & both) == both;
}

static void stm32h5_dts_reset_hold(Object *obj, ResetType type)
{
    Stm32h5DtsState *s = STM32H5_DTS(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[DTS_T0VALR1 / 4] = DTS_FCU_T0VALR1;
    s->regs[DTS_RAMPVALR / 4] = DTS_FCU_RAMPVALR;
}

static uint64_t stm32h5_dts_read(void *opaque, hwaddr addr, unsigned size)
{
    Stm32h5DtsState *s = opaque;

    if (addr == DTS_SR) {
        uint32_t cfgr1 = s->regs[DTS_CFGR1 / 4];

        return (cfgr1 & DTS_CFGR1_TS1_EN) && !stm32h5_dts_measuring(s)
            ? DTS_SR_TS1_RDY : 0;
    }
    return s->regs[addr / 4];
}

static void stm32h5_dts_write(void *opaque, hwaddr addr, uint64_t val64,
                              unsigned size)
{
    Stm32h5DtsState *s = opaque;

    switch (addr) {
    case DTS_T0VALR1:
    case DTS_RAMPVALR:
    case DTS_DR:
    case DTS_SR:
        return; /* read-only */
    default:
        break;
    }
    s->regs[addr / 4] = val64;
    if (stm32h5_dts_measuring(s)) {
        uint32_t smp = (s->regs[DTS_CFGR1 / 4] >> 16) & 0xFu;

        s->regs[DTS_DR / 4] =
            (uint32_t)((uint64_t)DTS_PCLK_HZ * (smp ? smp : 1) / DTS_FM_HZ);
    }
}

#define STM32H5_REG32_OPS(read_fn, write_fn) {          \
        .read = read_fn,                                \
        .write = write_fn,                              \
        .endianness = DEVICE_LITTLE_ENDIAN,             \
        .valid = {                                      \
            .min_access_size = 4,                       \
            .max_access_size = 4,                       \
        },                                              \
        .impl = {                                       \
            .min_access_size = 4,                       \
            .max_access_size = 4,                       \
        },                                              \
    }

static const MemoryRegionOps stm32h5_pwr_ops =
    STM32H5_REG32_OPS(stm32h5_pwr_read, stm32h5_pwr_write);
static const MemoryRegionOps stm32h5_flash_ops =
    STM32H5_REG32_OPS(stm32h5_flash_read, stm32h5_flash_write);
static const MemoryRegionOps stm32h5_dts_ops =
    STM32H5_REG32_OPS(stm32h5_dts_read, stm32h5_dts_write);

static void stm32h5_pwr_init(Object *obj)
{
    Stm32h5PwrState *s = STM32H5_PWR(obj);

    memory_region_init_io(&s->mmio, obj, &stm32h5_pwr_ops, s,
                          TYPE_STM32H5_PWR, STM32H5_SYSCTL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void stm32h5_flash_init(Object *obj)
{
    Stm32h5FlashState *s = STM32H5_FLASH(obj);

    memory_region_init_io(&s->mmio, obj, &stm32h5_flash_ops, s,
                          TYPE_STM32H5_FLASH, STM32H5_SYSCTL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void stm32h5_dts_init(Object *obj)
{
    Stm32h5DtsState *s = STM32H5_DTS(obj);

    memory_region_init_io(&s->mmio, obj, &stm32h5_dts_ops, s,
                          TYPE_STM32H5_DTS, STM32H5_SYSCTL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const VMStateDescription vmstate_stm32h5_dts = {
    .name = TYPE_STM32H5_DTS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Stm32h5DtsState, STM32H5_SYSCTL_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32h5_dts_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = stm32h5_dts_reset_hold;
    dc->vmsd = &vmstate_stm32h5_dts;
}

static const VMStateDescription vmstate_stm32h5_pwr = {
    .name = TYPE_STM32H5_PWR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Stm32h5PwrState, STM32H5_SYSCTL_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_stm32h5_flash = {
    .name = TYPE_STM32H5_FLASH,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Stm32h5FlashState, STM32H5_SYSCTL_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32h5_pwr_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = stm32h5_pwr_reset_hold;
    dc->vmsd = &vmstate_stm32h5_pwr;
}

static void stm32h5_flash_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = stm32h5_flash_reset_hold;
    dc->vmsd = &vmstate_stm32h5_flash;
}

static const TypeInfo stm32h5_sysctl_types[] = {
    {
        .name          = TYPE_STM32H5_PWR,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h5PwrState),
        .instance_init = stm32h5_pwr_init,
        .class_init    = stm32h5_pwr_class_init,
    },
    {
        .name          = TYPE_STM32H5_FLASH,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h5FlashState),
        .instance_init = stm32h5_flash_init,
        .class_init    = stm32h5_flash_class_init,
    },
    {
        .name          = TYPE_STM32H5_DTS,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Stm32h5DtsState),
        .instance_init = stm32h5_dts_init,
        .class_init    = stm32h5_dts_class_init,
    },
};

DEFINE_TYPES(stm32h5_sysctl_types)
