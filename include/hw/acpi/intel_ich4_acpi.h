/*
 * Intel ICH4 based ACPI Implementation based on Qemu's PIIX4 PM structure
 *
 * Copyright (c) 2006 Fabrice Bellard
 * Copyright (c) 2023 Tiseno100
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#ifndef HW_ACPI_PIIX4_H
#define HW_ACPI_PIIX4_H

#include "hw/pci/pci_device.h"
#include "hw/acpi/acpi.h"
#include "hw/acpi/cpu_hotplug.h"
#include "hw/acpi/memory_hotplug.h"
#include "hw/acpi/pcihp.h"
#include "hw/i2c/pm_smbus.h"
#include "hw/isa/apm.h"

#define TYPE_INTEL_ICH4_ACPI "intel-ich4-acpi"
OBJECT_DECLARE_SIMPLE_TYPE(Intel_ICH4_ACPI_State, INTEL_ICH4_ACPI)

struct Intel_ICH4_ACPI_State {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    /* ACPI I/O Base */
    MemoryRegion io;
    uint32_t io_base;

    /* GPE */
    MemoryRegion io_gpe;
    ACPIREGS ar;

    /* Good old APM ports. B2h/B3h */
    APMState apm;

    /* PIIX Compatible SMBus */
    PMSMBus smb;
    uint32_t smb_io_base;

    /* SMI/SCI IRQ Handling */
    qemu_irq irq;
    qemu_irq smi_irq;
    bool smm_enabled;
    bool smm_compat;
    Notifier machine_ready;
    Notifier powerdown_notifier;

    /* Qemu stuff */
    AcpiPciHpState acpi_pci_hotplug;
    bool not_migrate_acpi_index;

    uint8_t disable_s3;
    uint8_t disable_s4;
    uint8_t s4_val;

    bool cpu_hotplug_legacy;
    AcpiCpuHotplug gpe_cpu;
    CPUHotplugState cpuhp_state;

    MemHotplugState acpi_memory_hotplug;
};

#endif
