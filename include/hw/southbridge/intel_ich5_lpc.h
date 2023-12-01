/*
 * Intel ICH5 LPC
 *
 * Copyright (c) 2006 Fabrice Bellard
 * Copyright (c) 2023 Tiseno100
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_INTEL_ICH5_LPC_H
#define HW_INTEL_ICH5_LPC_H

#include "hw/acpi/intel_ich5_acpi.h"
#include "hw/pci/pci_device.h"
#include "hw/rtc/intel_ich5_nvr.h"

struct Intel_ICH5_LPC_State {
    PCIDevice dev;

    uint64_t pic_level;

    qemu_irq lpc_irqs_in[24]; /* i8259 + IOAPIC */

    /* This member isn't used. Just for save/load compatibility */
    int32_t pci_irq_levels_vmstate[8];

    /* NVR */
    Intel_ICH5_NVR_State rtc;

    /* Reset Control Register contents */
    uint8_t rcr;

    /* IO memory region for Reset Control Register (ICH4_RCR_IOPORT) */
    MemoryRegion rcr_mem;

    /* The ACPI device so we can remap it's SCI & ACPI */
    Intel_ICH5_ACPI_State *acpi;
};
typedef struct Intel_ICH5_LPC_State Intel_ICH5_LPC_State;

/* Meant to link the ACPI device formed on pc_init1 with the LPC device so we can remap ACPI I/O */
void intel_ich5_link_acpi(Intel_ICH5_LPC_State *lpc, Intel_ICH5_ACPI_State *acpi);

#define TYPE_INTEL_ICH5_LPC "intel-ich5-lpc"
DECLARE_INSTANCE_CHECKER(Intel_ICH5_LPC_State, INTEL_ICH5_LPC, TYPE_INTEL_ICH5_LPC)

#define TYPE_INTEL_ICH5 "intel-ich5"

#endif
