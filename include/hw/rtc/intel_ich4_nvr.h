/*
 * Intel ICH4 NVR
 *
 * Copyright (c) 2003-2004 Fabrice Bellard
 * Copyright (c) 2023 Tiseno100
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HW_INTEL_ICH4_NVR_H
#define HW_INTEL_ICH4_NVR_H

#include "qapi/qapi-types-machine.h"
#include "qemu/queue.h"
#include "qemu/timer.h"
#include "hw/isa/isa.h"
#include "qom/object.h"

#define TYPE_INTEL_ICH4_NVR "intel_ich4_nvr"
OBJECT_DECLARE_SIMPLE_TYPE(Intel_ICH4_NVR_State, INTEL_ICH4_NVR)

struct Intel_ICH4_NVR_State {
    ISADevice parent_obj;

    MemoryRegion io;
    MemoryRegion coalesced_io;
    uint8_t cmos_data[256]; /* 2 Banks (128 Byte Standard + 128 Byte Extended) */
    uint8_t cmos_index;
    uint8_t isairq;
    uint16_t io_base;
    int32_t base_year;
    uint64_t base_rtc;
    uint64_t last_update;
    int64_t offset;
    qemu_irq irq;
    int it_shift;
    /* periodic timer */
    QEMUTimer *periodic_timer;
    int64_t next_periodic_time;
    /* update-ended timer */
    QEMUTimer *update_timer;
    uint64_t next_alarm_time;
    uint16_t irq_reinject_on_ack_count;
    uint32_t irq_coalesced;
    uint32_t period;
    QEMUTimer *coalesced_timer;
    Notifier clock_reset_notifier;
    LostTickPolicy lost_tick_policy;
    Notifier suspend_notifier;
    QLIST_ENTRY(Intel_ICH4_NVR_State) link;
};

#define RTC_ISA_IRQ 8

Intel_ICH4_NVR_State *intel_ich4_nvr_init(ISABus *bus, int base_year, qemu_irq intercept_irq);
void intel_ich4_nvr_write_cmos(Intel_ICH4_NVR_State *s, int addr, int val);
int intel_ich4_nvr_read_cmos(Intel_ICH4_NVR_State *s, int addr);
void qmp_rtc_reset_reinjection_new(Error **errp);

#endif /* HW_INTEL_ICH4_NVR_H */
