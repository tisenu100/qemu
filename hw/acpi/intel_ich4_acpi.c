/*
 * Intel ICH4 based ACPI/SMBus Implementation based on Qemu's PIIX4 PM structure
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

/*
    Note: PIIX4 has an entire seperate PM device of it's own. Since ICH0 ACPI is handled by the LPC Device while SMBus has it's own PCI Controller.
    We recycle the PIIX4 PM Controller for the SMBus while keeping the PM intact
*/

#include "qemu/osdep.h"
#include "qemu/qemu-print.h"
#include "hw/irq.h"
#include "hw/isa/apm.h"
#include "hw/i2c/pm_smbus.h"
#include "hw/pci/pci.h"
#include "hw/qdev-properties.h"
#include "hw/acpi/acpi.h"
#include "hw/acpi/pcihp.h"
#include "sysemu/runstate.h"
#include "sysemu/sysemu.h"
#include "sysemu/xen.h"
#include "qapi/error.h"
#include "qemu/range.h"
#include "hw/acpi/cpu.h"
#include "hw/mem/pc-dimm.h"
#include "hw/mem/nvdimm.h"
#include "hw/acpi/acpi_dev_interface.h"
#include "migration/vmstate.h"
#include "hw/core/cpu.h"
#include "qom/object.h"

#include "hw/acpi/intel_ich4_acpi.h"

#define GPE_BASE 0xafe0
#define GPE_LEN 4

struct pci_status {
    uint32_t up; /* deprecated, maintained for migration compatibility */
    uint32_t down;
};

#define ACPI_ENABLE 0xf1
#define ACPI_DISABLE 0xf0

static void pm_tmr_timer(ACPIREGS *ar)
{
    Intel_ICH4_ACPI_State *s = container_of(ar, Intel_ICH4_ACPI_State, ar);
    acpi_update_sci(&s->ar, s->irq);
}


/* Global SMI Trigger. Qemu likes it being an IRQ */
static void intel_ich4_provoke_smi(Intel_ICH4_ACPI_State *s, uint16_t status_set)
{
    if (s->smi_irq) {
        qemu_printf("Intel ICH4 ACPI: We have provoked an SMI 0x%04x!\n", status_set);
        qemu_irq_raise(s->smi_irq);
    }
}

/* APMC SMI control */
static void apm_ctrl_changed(uint32_t val, void *arg)
{
    Intel_ICH4_ACPI_State *s = arg;

    acpi_pm1_cnt_update(&s->ar, val == ACPI_ENABLE, val == ACPI_DISABLE);
    if (val == ACPI_ENABLE || val == ACPI_DISABLE) {
        return;
    }

    /* When SMI handlers are ready */
    intel_ich4_provoke_smi(s, 0);
}

static void piix4_pm_init(Object *obj)
{
    Intel_ICH4_ACPI_State *s = INTEL_ICH4_ACPI(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->smi_irq, "smi-irq", 1);
}

static uint64_t gpe_readb(void *opaque, hwaddr addr, unsigned width)
{
    Intel_ICH4_ACPI_State *s = opaque;
    uint32_t val = acpi_gpe_ioport_readb(&s->ar, addr);

    return val;
}

static void gpe_writeb(void *opaque, hwaddr addr, uint64_t val,
                       unsigned width)
{
    Intel_ICH4_ACPI_State *s = opaque;

    acpi_gpe_ioport_writeb(&s->ar, addr, val);
    acpi_update_sci(&s->ar, s->irq);
}

static const MemoryRegionOps intel_ich4_acpi_gpe_ops = {
    .read = gpe_readb,
    .write = gpe_writeb,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void intel_ich4_send_gpe(AcpiDeviceIf *adev, AcpiEventStatusBits ev)
{
    Intel_ICH4_ACPI_State *s = INTEL_ICH4_ACPI(adev);

    acpi_send_gpe_event(&s->ar, s->irq, ev);
}

static void intel_ich4_smbus(int msb, int lsb, int en, Intel_ICH4_ACPI_State *s)
{
    s->smb_io_base = ((msb << 8) | lsb) & 0xffc0;

    if(!!en)
        qemu_printf("Intel ICH4 SMBus: SMBus enabled on address 0x%04x\n", s->smb_io_base);
    else
        qemu_printf("Intel ICH4 SMBus: SMBus is disabled\n");

    memory_region_transaction_begin();
    memory_region_set_enabled(&s->smb.io, !!en);
    memory_region_set_address(&s->smb.io, s->smb_io_base);
    memory_region_transaction_commit();
}

static void intel_ich4_smbus_features(int val, Intel_ICH4_ACPI_State *s) {

    /*
        Page 455 of the Intel ICH4 datasheet

        Bit 2: I2C_EN
        0 = Disabled
        1 = The ICH4 is enabled to communicate with I2C devices. This will change the formatting of some
        commands
    */
    bool i2c_enable = !!(val & 4);

    s->smb.i2c_enable = i2c_enable;
    qemu_printf("Intel ICH4 SMBus: I2C Capabilities were %s!\n", i2c_enable ? "enabled" : "disabled");
}

static void intel_ich4_smbus_write_config(PCIDevice *dev, uint32_t address, uint32_t val, int len)
{
    Intel_ICH4_ACPI_State *ich4_acpi = INTEL_ICH4_ACPI(dev);

    for(int i = 0; i < len; i++){
        int ro_only = 0;

        uint8_t new_val = (val >> (i * 8)) & 0xff;

        switch(address + i){
            case 0x04:
                new_val = new_val & 0x01;
            break;

            case 0x07:
                new_val &= new_val & 0x08;
                new_val |= 0x02;
            break;

            case 0x20:
                new_val = new_val & 0xe0;
                new_val |= 1;
            break;

            case 0x2c:
            case 0x2d:
            case 0x2e:
            case 0x2f:
                if(dev->config[address + i])
                    ro_only = 1;
            break;

            case 0x40:
                new_val = new_val & 0x07;
            break;

            case 0x21:
            case 0x3c:
            break;

            default:
                ro_only = 1;
            break;
        }

        if(!ro_only) {
            pci_default_write_config(dev, address, val, len);
            dev->config[address + i] = new_val;
            qemu_printf("Intel ICH4 SMBus: dev->regs[0x%02x] = %02x\n", address + i, new_val);
        }
    }

    switch(address){
        case 0x04:
        case 0x20:
        case 0x21:
            intel_ich4_smbus(dev->config[0x21], dev->config[0x20], dev->config[0x04] & 1, ich4_acpi);
        break;

        case 0x40:
            intel_ich4_smbus_features(dev->config[0x40], ich4_acpi);
        break;
    }
}

static int vmstate_acpi_post_load(void *opaque, int version_id)
{
//    Intel_ICH4_ACPI_State *s = opaque;

//    pm_io_space_update(s);
//    intel_ich4_smbus(s);
    return 0;
}

#define VMSTATE_GPE_ARRAY(_field, _state)                            \
 {                                                                   \
     .name       = (stringify(_field)),                              \
     .version_id = 0,                                                \
     .info       = &vmstate_info_uint16,                             \
     .size       = sizeof(uint16_t),                                 \
     .flags      = VMS_SINGLE | VMS_POINTER,                         \
     .offset     = vmstate_offset_pointer(_state, _field, uint8_t),  \
 }

static const VMStateDescription vmstate_gpe = {
    .name = "gpe",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_GPE_ARRAY(sts, ACPIGPE),
        VMSTATE_GPE_ARRAY(en, ACPIGPE),
        VMSTATE_END_OF_LIST()
    }
};

static bool intel_ich4_acpi_vmstate_need_smbus(void *opaque, int version_id)
{
    return pm_smbus_vmstate_needed();
}


/* qemu-kvm 1.2 uses version 3 but advertised as 2
 * To support incoming qemu-kvm 1.2 migration, change version_id
 * and minimum_version_id to 2 below (which breaks migration from
 * qemu 1.2).
 *
 */
static const VMStateDescription vmstate_acpi = {
    .name = "intel-ich4-acpi",
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = vmstate_acpi_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, Intel_ICH4_ACPI_State),
        VMSTATE_UINT16(ar.pm1.evt.sts, Intel_ICH4_ACPI_State),
        VMSTATE_UINT16(ar.pm1.evt.en, Intel_ICH4_ACPI_State),
        VMSTATE_UINT16(ar.pm1.cnt.cnt, Intel_ICH4_ACPI_State),
        VMSTATE_STRUCT(apm, Intel_ICH4_ACPI_State, 0, vmstate_apm, APMState),
        VMSTATE_STRUCT_TEST(smb, Intel_ICH4_ACPI_State, intel_ich4_acpi_vmstate_need_smbus, 3,
                            pmsmb_vmstate, PMSMBus),
        VMSTATE_TIMER_PTR(ar.tmr.timer, Intel_ICH4_ACPI_State),
        VMSTATE_INT64(ar.tmr.overflow_time, Intel_ICH4_ACPI_State),
        VMSTATE_STRUCT(ar.gpe, Intel_ICH4_ACPI_State, 2, vmstate_gpe, ACPIGPE),
        VMSTATE_END_OF_LIST()
    }
};

static void intel_ich4_acpi_reset(DeviceState *dev)
{
    Intel_ICH4_ACPI_State *s = INTEL_ICH4_ACPI(dev);

    acpi_pm1_evt_reset(&s->ar);
    acpi_pm1_cnt_reset(&s->ar);
    acpi_pm_tmr_reset(&s->ar);
    acpi_gpe_reset(&s->ar);
    acpi_update_sci(&s->ar, s->irq);
}

static void intel_ich4_acpi_powerdown_req(Notifier *n, void *opaque)
{
    Intel_ICH4_ACPI_State *s = container_of(n, Intel_ICH4_ACPI_State, powerdown_notifier);

    assert(s != NULL);
    acpi_pm1_evt_power_down(&s->ar);
}

static void intel_ich4_acpi_machine_ready(Notifier *n, void *opaque)
{
//    Intel_ICH4_ACPI_State *s = container_of(n, Intel_ICH4_ACPI_State, machine_ready);
//    PCIDevice *d = PCI_DEVICE(s);
//    MemoryRegion *io_as = pci_address_space_io(d);
//    uint8_t *pci_conf;

/*
    pci_conf = d->config;
    pci_conf[0x5f] = 0x10 |
        (memory_region_present(io_as, 0x378) ? 0x80 : 0);
    pci_conf[0x63] = 0x60;
    pci_conf[0x67] = (memory_region_present(io_as, 0x3f8) ? 0x08 : 0) |
        (memory_region_present(io_as, 0x2f8) ? 0x90 : 0);
*/
}

static void intel_ich4_acpi_add_properties(Intel_ICH4_ACPI_State *s)
{
    static const uint8_t acpi_enable_cmd = ACPI_ENABLE;
    static const uint8_t acpi_disable_cmd = ACPI_DISABLE;
    static const uint32_t gpe0_blk = GPE_BASE;
    static const uint32_t gpe0_blk_len = GPE_LEN;
    static const uint16_t sci_int = 9;

    object_property_add_uint8_ptr(OBJECT(s), ACPI_PM_PROP_ACPI_ENABLE_CMD, &acpi_enable_cmd, OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(OBJECT(s), ACPI_PM_PROP_ACPI_DISABLE_CMD, &acpi_disable_cmd, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(OBJECT(s), ACPI_PM_PROP_GPE0_BLK, &gpe0_blk, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(OBJECT(s), ACPI_PM_PROP_GPE0_BLK_LEN, &gpe0_blk_len, OBJ_PROP_FLAG_READ);
    object_property_add_uint16_ptr(OBJECT(s), ACPI_PM_PROP_SCI_INT, &sci_int, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(OBJECT(s), ACPI_PM_PROP_PM_IO_BASE, &s->io_base, OBJ_PROP_FLAG_READ);
}

static void intel_ich4_acpi_realize(PCIDevice *dev, Error **errp)
{
    Intel_ICH4_ACPI_State *s = INTEL_ICH4_ACPI(dev);

    /* APM */
    qemu_printf("Intel ICH4 ACPI: Starting APM\n");
    apm_init(dev, &s->apm, apm_ctrl_changed, s);

    /* SMBus */
    pm_smbus_init(DEVICE(dev), &s->smb, true);
    memory_region_set_enabled(&s->smb.io, false);
    memory_region_add_subregion(pci_address_space_io(dev), s->smb_io_base, &s->smb.io);
    qemu_printf("Intel ICH4 SMBus: SMBus is up!\n");

    /* ACPI */
    memory_region_init(&s->io, OBJECT(s), "intel-ich4-acpi", 64);
    memory_region_set_enabled(&s->io, false);
    memory_region_add_subregion(pci_address_space_io(dev), s->io_base, &s->io);
    qemu_printf("Intel ICH4 ACPI: ACPI is up!\n");

    /* GPE */
    memory_region_init_io(&s->io_gpe, OBJECT(s), &intel_ich4_acpi_gpe_ops, s, "acpi-gpe0", GPE_LEN);
    memory_region_add_subregion(pci_address_space_io(dev), GPE_BASE, &s->io_gpe);

    /* ACPI Events & Timers */
    acpi_pm_tmr_init(&s->ar, pm_tmr_timer, &s->io);
    acpi_pm1_evt_init(&s->ar, pm_tmr_timer, &s->io);
    acpi_pm1_cnt_init(&s->ar, &s->io, s->disable_s3, s->disable_s4, s->s4_val,
                      !s->smm_compat && !s->smm_enabled);
    acpi_gpe_init(&s->ar, GPE_LEN);

    /* Notifiers for Qemu */
    s->powerdown_notifier.notify = intel_ich4_acpi_powerdown_req;
    qemu_register_powerdown_notifier(&s->powerdown_notifier);

    s->machine_ready.notify = intel_ich4_acpi_machine_ready;
    qemu_add_machine_init_done_notifier(&s->machine_ready);

    intel_ich4_acpi_add_properties(s);
}

static Property intel_ich4_acpi_properties[] = {
    DEFINE_PROP_UINT32("smb_io_base", Intel_ICH4_ACPI_State, smb_io_base, 0),
    DEFINE_PROP_UINT8(ACPI_PM_PROP_S3_DISABLED, Intel_ICH4_ACPI_State, disable_s3, 0),
    DEFINE_PROP_UINT8(ACPI_PM_PROP_S4_DISABLED, Intel_ICH4_ACPI_State, disable_s4, 0),
    DEFINE_PROP_UINT8(ACPI_PM_PROP_S4_VAL, Intel_ICH4_ACPI_State, s4_val, 2),
    DEFINE_PROP_BOOL("smm-compat", Intel_ICH4_ACPI_State, smm_compat, false),
    DEFINE_PROP_BOOL("smm-enabled", Intel_ICH4_ACPI_State, smm_enabled, false),
    DEFINE_PROP_BOOL("x-not-migrate-acpi-index", Intel_ICH4_ACPI_State, not_migrate_acpi_index, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void piix4_pm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    AcpiDeviceIfClass *adevc = ACPI_DEVICE_IF_CLASS(klass);

    /* Common */
    k->realize = intel_ich4_acpi_realize;
    dc->reset = intel_ich4_acpi_reset;
    dc->desc = "Intel ICH4 Compatible ACPI & SMBus";
    dc->vmsd = &vmstate_acpi;

    /* SMBus PCI Device */
    k->config_write = intel_ich4_smbus_write_config;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_82371AB_3;
    k->class_id = PCI_CLASS_SERIAL_SMBUS;

    /* Properties */
    device_class_set_props(dc, intel_ich4_acpi_properties);
    dc->user_creatable = false;
    adevc->send_event = intel_ich4_send_gpe;
}

static const TypeInfo piix4_pm_info = {
    .name          = TYPE_INTEL_ICH4_ACPI,
    .parent        = TYPE_PCI_DEVICE,
    .instance_init  = piix4_pm_init,
    .instance_size = sizeof(Intel_ICH4_ACPI_State),
    .class_init    = piix4_pm_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_ACPI_DEVICE_IF },
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    }
};

static void piix4_pm_register_types(void)
{
    type_register_static(&piix4_pm_info);
}

type_init(piix4_pm_register_types)
