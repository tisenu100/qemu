/*
 * Intel ICH5 based ACPI Implementation based on Qemus PIIX4 PM structure
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

#ifndef HW_INTEL_ICH5_ACPI_H
#define HW_INTEL_ICH5_ACPI_H

#include "hw/pci/pci_device.h"
#include "hw/acpi/acpi.h"
#include "hw/i2c/pm_smbus.h"
#include "hw/isa/apm.h"

#define TYPE_INTEL_ICH5_ACPI "intel-ich5-acpi"
OBJECT_DECLARE_SIMPLE_TYPE(Intel_ICH5_ACPI_State, INTEL_ICH5_ACPI)

struct Intel_ICH5_ACPI_State {
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

    /* SMBus */
    PMSMBus smb;
    uint16_t smb_io_base;

    /* SMI/SCI IRQ Handling */
    qemu_irq irq;
    qemu_irq smi_irq;

    /* SMI Controller */
    MemoryRegion smi_io;
    bool smi_lock;
    uint8_t smi_w[4];
    uint8_t smi_s[4];

    /* SMI Trap Handler */
    ISABus *isa_bus; /* To forward ISA read/writes */
    MemoryRegion smi_trap_io;
    uint8_t smi_trap[2];

    MemoryRegion kbc_trap; /* Port 60-64h Trap */

    /* SCI */
    uint16_t sci_irq;
    int sci_level;

    /* SMI/SCI Qemu Stuff */
    bool smm_enabled;
    bool smm_compat;
    bool not_migrate_acpi_index;
    Notifier machine_ready;
    Notifier powerdown_notifier;

    uint8_t disable_s3;
    uint8_t disable_s4;
    uint8_t s4_val;
};
typedef struct Intel_ICH5_ACPI_State Intel_ICH5_ACPI_State;

void intel_ich5_acpi_mount_kbc_trap(Intel_ICH5_ACPI_State *acpi, ISABus *bus);

#endif
