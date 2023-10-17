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

#ifndef HW_PCI_845PE_H
#define HW_PCI_845PE_H

#include "hw/pci/pci_device.h"
#include "hw/pci-host/pam.h"
#include "qom/object.h"

#define I845PE_HOST_PROP_PCI_TYPE "pci-type"

#define TYPE_INTEL_845PE_HOST_BRIDGE "intel_845pe-pcihost"
#define TYPE_INTEL_845PE_PCI_DEVICE "intel_845pe"

OBJECT_DECLARE_SIMPLE_TYPE(PCI845PEState, INTEL_845PE_PCI_DEVICE)

struct PCI845PEState {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    PAMMemoryRegion pam_regions[PAM_REGIONS_COUNT];
    MemoryRegion smram_region;
    MemoryRegion smram, low_smram, high_smram;

    MemoryRegion global_smram;
};

/* Not yet */
//#define TYPE_IGD_PASSTHROUGH_I440FX_PCI_DEVICE "igd-passthrough-i440FX"

#endif
