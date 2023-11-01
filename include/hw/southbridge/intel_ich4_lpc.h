/*
 * Intel® 82801DB I/O Controller Hub 4 LPC Bridge
 *
 * Copyright (c) 2006 Fabrice Bellard
 * Copyright (c) 2018 Hervé Poussineau
 * Copyright (c) 2023 Tiseno100
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#ifndef HW_SOUTHBRIDGE_ICH4_H
#define HW_SOUTHBRIDGE_ICH4_H

#include "hw/acpi/intel_ich4_acpi.h"
#include "hw/pci/pci_device.h"
#include "hw/rtc/intel_ich4_nvr.h"

/*
 * Reset Control Register: PCI-accessible ISA-Compatible Register at address
 * 0xcf9, provided by the PCI/ISA bridge (ICH4 PCI function 0, 8086:7000).
 */
#define ICH4_RCR_IOPORT 0xcf9

#define ICH4_NUM_PIC_IRQS       16      /* i8259 * 2 */
#define ICH4_NUM_PIRQS          8ULL    /* ICH4 is capable to store 8 PIRQ's. PIRQ[A-H] */

struct ICH4State {
    PCIDevice dev;

    uint64_t pic_levels;

    qemu_irq *pic;

    /* This member isn't used. Just for save/load compatibility */
    int32_t pci_irq_levels_vmstate[ICH4_NUM_PIRQS];

    /* NVR */
    Intel_ICH4_NVR_State rtc;

    /* Reset Control Register contents */
    uint8_t rcr;

    /* IO memory region for Reset Control Register (ICH4_RCR_IOPORT) */
    MemoryRegion rcr_mem;

    Intel_ICH4_ACPI_State *acpi;
};
typedef struct ICH4State ICH4State;

/* Meant to link the ACPI device formed on pc_init1 with the LPC device so we can remap ACPI I/O */
void intel_ich4_link_acpi(ICH4State *lpc, Intel_ICH4_ACPI_State *acpi);

#define TYPE_ICH4_PCI_DEVICE "intel-ich4-lpc"
DECLARE_INSTANCE_CHECKER(ICH4State, ICH4_PCI_DEVICE, TYPE_ICH4_PCI_DEVICE)

#define TYPE_ICH4_DEVICE "intel-ich4"

#endif
