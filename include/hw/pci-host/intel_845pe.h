/*
 * Intel® 845GE/845PE Chipset
 *
 * Copyright (c) 2006 Fabrice Bellard
 * Copyright (c) 2023 Tiseno100
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#ifndef HW_PCI_INTEL_845PE_H
#define HW_PCI_INTEL_845PE_H

#include "hw/pci/pci_device.h"
#include "hw/pci-host/pam.h"
#include "qom/object.h"

#define INTEL_845PE_HOST_PROP_PCI_TYPE "pci-type"

#define TYPE_INTEL_845PE_HOST_BRIDGE "intel_845pe-pcihost"
#define TYPE_INTEL_845PE_PCI_DEVICE "intel_845pe"

OBJECT_DECLARE_SIMPLE_TYPE(Intel_845PE_PCI_State, INTEL_845PE_PCI_DEVICE)

struct Intel_845PE_PCI_State {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    /* PAM "Shadow RAM" */
    PAMMemoryRegion pam_regions[PAM_REGIONS_COUNT];

    /* PCI IRQ Table Region */
    MemoryRegion pci_irq_table;

    /* SMRAM */
    MemoryRegion smram_region, high_smram_region;
    MemoryRegion cpu_smram, smram, low_smram, high_smram;
};

/* Not yet */
//#define TYPE_IGD_PASSTHROUGH_I440FX_PCI_DEVICE "igd-passthrough-i440FX"

#endif
