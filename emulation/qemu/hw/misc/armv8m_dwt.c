/*
 * ARMv8-M Data Watchpoint and Trace unit: cycle counter only
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ARMv8-M Architecture Reference Manual, DWT_CTRL / DWT_CYCCNT.
 *
 * QEMU's ARMv7-M container leaves the DWT range RAZ/WI, so firmware that times
 * code with DWT_CYCCNT reads 0. This model implements:
 *   - DWT_CTRL (0x000): CYCCNTENA (bit 0); NUMCOMP = 0, no comparators, the
 *     NOxxx feature bits read 0 except NOTRCPKT/NOEXTTRIG/NOPRFCNT (no trace,
 *     no external triggers, no profiling counters).
 *   - DWT_CYCCNT (0x004): counts at the core clock while CYCCNTENA is set,
 *     derived from QEMU_CLOCK_VIRTUAL (consistent with SysTick, not
 *     cycle-accurate).
 * DEMCR.TRCENA (in the SCS) is not modelled: the counter runs on CYCCNTENA alone.
 * All other registers are RAZ/WI.
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "migration/vmstate.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/armv8m_dwt.h"

#define A_CTRL   0x000
#define A_CYCCNT 0x004

#define CTRL_CYCCNTENA  (1u << 0)
#define CTRL_NOPRFCNT   (1u << 24)
#define CTRL_NOEXTTRIG  (1u << 26)
#define CTRL_NOTRCPKT   (1u << 27)
#define CTRL_RO_BITS    (CTRL_NOPRFCNT | CTRL_NOEXTTRIG | CTRL_NOTRCPKT)

static uint32_t armv8m_dwt_cyccnt(Armv8mDwtState *s)
{
    if (!(s->ctrl & CTRL_CYCCNTENA)) {
        return s->cyccnt;
    }
    const int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->start_ns;
    return s->cyccnt +
        (uint32_t)muldiv64(elapsed, clock_get_hz(s->clk), NANOSECONDS_PER_SECOND);
}

static uint64_t armv8m_dwt_read(void *opaque, hwaddr addr, unsigned size)
{
    Armv8mDwtState *s = opaque;

    switch (addr) {
    case A_CTRL:
        return s->ctrl | CTRL_RO_BITS;
    case A_CYCCNT:
        return armv8m_dwt_cyccnt(s);
    default:
        return 0;
    }
}

static void armv8m_dwt_write(void *opaque, hwaddr addr, uint64_t val64,
                             unsigned size)
{
    Armv8mDwtState *s = opaque;
    const uint32_t value = val64;

    switch (addr) {
    case A_CTRL:
        s->cyccnt = armv8m_dwt_cyccnt(s);
        s->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->ctrl = value & CTRL_CYCCNTENA;
        break;
    case A_CYCCNT:
        s->cyccnt = value;
        s->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps armv8m_dwt_ops = {
    .read = armv8m_dwt_read,
    .write = armv8m_dwt_write,
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

static void armv8m_dwt_reset_hold(Object *obj, ResetType type)
{
    Armv8mDwtState *s = ARMV8M_DWT(obj);

    s->ctrl = 0;
    s->cyccnt = 0;
    s->start_ns = 0;
}

static void armv8m_dwt_init(Object *obj)
{
    Armv8mDwtState *s = ARMV8M_DWT(obj);

    memory_region_init_io(&s->mmio, obj, &armv8m_dwt_ops, s, TYPE_ARMV8M_DWT,
                          ARMV8M_DWT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", NULL, NULL, 0);
}

static const VMStateDescription vmstate_armv8m_dwt = {
    .name = TYPE_ARMV8M_DWT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, Armv8mDwtState),
        VMSTATE_UINT32(ctrl, Armv8mDwtState),
        VMSTATE_UINT32(cyccnt, Armv8mDwtState),
        VMSTATE_INT64(start_ns, Armv8mDwtState),
        VMSTATE_END_OF_LIST()
    }
};

static void armv8m_dwt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = armv8m_dwt_reset_hold;
    dc->vmsd = &vmstate_armv8m_dwt;
}

static const TypeInfo armv8m_dwt_types[] = {
    {
        .name          = TYPE_ARMV8M_DWT,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Armv8mDwtState),
        .instance_init = armv8m_dwt_init,
        .class_init    = armv8m_dwt_class_init,
    },
};

DEFINE_TYPES(armv8m_dwt_types)
