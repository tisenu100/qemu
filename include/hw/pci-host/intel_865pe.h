/*
 * Intel 865P/865PE Memory Controller Hub
 *
 * Copyright (c) 2006 Fabrice Bellard
 * Copyright (c) 2023 Tiseno100
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#ifndef HW_PCI_INTEL_865PE_H
#define HW_PCI_INTEL_865PE_H

#include "hw/pci/pci_device.h"
#include "hw/pci-host/pam.h"
#include "qom/object.h"

#define INTEL_865PE_HOST_PROP_PCI_TYPE "pci-type"

#define TYPE_INTEL_865PE_HOST_BRIDGE "intel_865pe-pcihost"
#define TYPE_INTEL_865PE_PCI_DEVICE "intel_865pe"

OBJECT_DECLARE_SIMPLE_TYPE(Intel_865PE_PCI_State, INTEL_865PE_PCI_DEVICE)

struct Intel_865PE_PCI_State {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    /* PAM "Shadow RAM" */
    PAMMemoryRegion pam_regions[PAM_REGIONS_COUNT];

    /* SMRAM */
    MemoryRegion smram_region, high_smram_region;
    MemoryRegion cpu_smram, smram, low_smram, high_smram;
};

/* Not yet */
//#define TYPE_IGD_PASSTHROUGH_I440FX_PCI_DEVICE "igd-passthrough-i440FX"

#endif
