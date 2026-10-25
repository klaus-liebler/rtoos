/*
 * ARMv8-M Data Watchpoint and Trace unit: cycle counter only
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ARMV8M_DWT_H
#define HW_MISC_ARMV8M_DWT_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ARMV8M_DWT "armv8m-dwt"
OBJECT_DECLARE_SIMPLE_TYPE(Armv8mDwtState, ARMV8M_DWT)

#define ARMV8M_DWT_BASE 0xE0001000
#define ARMV8M_DWT_SIZE 0x1000

struct Armv8mDwtState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    Clock *clk;          /* core clock: CYCCNT rate */
    uint32_t ctrl;
    uint32_t cyccnt;     /* value at start_ns (while counting) */
    int64_t start_ns;
};

#endif /* HW_MISC_ARMV8M_DWT_H */
