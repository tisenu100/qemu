/*
 * Intel ICH4 based ACPI/SMBus Implementation based on Qemus PIIX4 PM structure
 *
 * Copyright (c) 2006 Fabrice Bellard
 * Copyright (c) 2023 Tiseno100
 *
 * ACPI core.c parts
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
    Note: PIIX4 has an entire seperate PM device of it's own. Since ICH0 ACPI is handled by the LPC while SMBus has it's own PCI Controller.
    We recycle the PIIX4 PM Controller for the SMBus while keeping the PM is linked to the LPC
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
#include "hw/acpi/acpi_dev_interface.h"
#include "migration/vmstate.h"
#include "hw/core/cpu.h"
#include "qom/object.h"
#include "qapi/error.h"
#include "qapi/opts-visitor.h"
#include "qapi/qapi-events-run-state.h"
#include "qapi/qapi-visit-acpi.h"

#include "hw/acpi/intel_ich4_acpi.h"

#define ACPI_ENABLE 0xf1
#define ACPI_DISABLE 0xf0

struct pci_status {
    uint32_t up; /* deprecated, maintained for migration compatibility */
    uint32_t down;
};

/* Global SMI Trigger */
static void intel_ich4_provoke_smi(Intel_ICH4_ACPI_State *s)
{
    if (s->smi_irq) { /* The SMI Interrupt is set the the pc_init code */
        qemu_printf("Intel ICH4 ACPI: An SMI interrupt was provoked!\n");
        qemu_irq_raise(s->smi_irq);
    }
}

/* PM Timer. When it times out provoke an SCI or SMI Instead */
static void pm_tmr_timer(ACPIREGS *ar)
{
    Intel_ICH4_ACPI_State *s = container_of(ar, Intel_ICH4_ACPI_State, ar);

    if((ar->pm1.evt.en & 1) && (ar->pm1.cnt.cnt & 1))
        acpi_update_sci(&s->ar, s->irq);
//    else if(ar->pm1.evt.en & 1)
//        intel_ich4_provoke_smi(s);
}

/* APM Control */
static void apm_ctrl_changed(uint32_t val, void *arg)
{
    Intel_ICH4_ACPI_State *s = arg;

    if(s->smi_w[0] & 0x20) { /* APMC SMI. If B3h is written provoke it */
        qemu_printf("Intel ICH4 ACPI: APMC has requested an SMI\n");
        s->smi_s[0] |= 0x20;
        intel_ich4_provoke_smi(s);
    }
}

/* GPE */
static uint64_t gpe_readb(void *opaque, hwaddr addr, unsigned width)
{
    Intel_ICH4_ACPI_State *s = opaque;
    uint32_t val = acpi_gpe_ioport_readb(&s->ar, addr);

    return val;
}

static void gpe_writeb(void *opaque, hwaddr addr, uint64_t val, unsigned width)
{
    Intel_ICH4_ACPI_State *s = opaque;

    int address = addr & 3;

    for(int i = 0; i < (int)width; i++)
        acpi_gpe_ioport_writeb(&s->ar, address + i, val >> (i * 8));

    /* We speculate something is expected for the BIOS to program GPE so we provoke an SCI immediately */
    acpi_update_sci(&s->ar, s->irq);

    qemu_printf("Intel ICH4 ACPI: GPE was updated 0x%04x\n", *s->ar.gpe.en);
}

static void intel_ich4_send_gpe(AcpiDeviceIf *adev, AcpiEventStatusBits ev)
{
    Intel_ICH4_ACPI_State *s = INTEL_ICH4_ACPI(adev);

    acpi_send_gpe_event(&s->ar, s->irq, ev);
}

static const MemoryRegionOps gpe_ops = {
    .read = gpe_readb,
    .write = gpe_writeb,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

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

/* PM1 Control. Qemu has it's own but we do it ourselves instead */
static uint64_t pm1_cnt_read(void *opaque, hwaddr addr, unsigned width)
{
    ACPIREGS *ar = opaque;
    return (ar->pm1.cnt.cnt >> ((int)addr & 1) * 8) & 0xff;
}

static void pm1_cnt_write(void *opaque, hwaddr addr, uint64_t val, unsigned width)
{
    Intel_ICH4_ACPI_State *s = INTEL_ICH4_ACPI(opaque);
    ACPIREGS *ar = opaque;

    ar->pm1.cnt.cnt = (val << ((addr == 1) * 8)) | ar->pm1.cnt.cnt; /* Avoid zeroing out the LSB bits when we write on the MSB ones and vice versa */

    qemu_printf("Intel ICH4 ACPI: PM Control update 0x%04x!\n", (int)ar->pm1.cnt.cnt);

    if(ar->pm1.cnt.cnt & 0x2000) { /* Bit 13: Sleep Enable */
        if(s->smi_w[0] & 0x10) { /* SLEEP_ENABLE SMI */
            s->smi_s[0] |= 0x10;
            qemu_printf("Intel ICH4 ACPI: Ignoring Sleep Status as an SMI request was gived\n");
            intel_ich4_provoke_smi(s);
        }
        else {
            /*
                Intel ICH4 Sleep Tables

                0: Working
                1: STPCLK#
                2: --
                3: --
                4: --
                5: Suspend to RAM
                6: Suspend to Disk
                7: Soft Off
            */
            switch((ar->pm1.cnt.cnt >> 10) & 7) {
                case 5:
                    qemu_printf("Intel ICH4 ACPI: Suspending to RAM\n");
                    qemu_system_suspend_request();
                break;

                case 6:
                    qemu_printf("Intel ICH4 ACPI: Suspending to Disk\n");
                    qapi_event_send_suspend_disk();
                    qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
                break;

                case 7:
                    qemu_printf("Intel ICH4 ACPI: We are powering down\n");
                    qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
                break;
            }
        }
    }

    if(!!(ar->pm1.cnt.cnt & 0x0004) && !!(s->smi_w[0] & 0x04)) {
        s->smi_s[0] |= 0x04;
        qemu_printf("Intel ICH4 ACPI: Global Release bit was set with BIOS_RLS. Requesting SMI interrupt\n");
        intel_ich4_provoke_smi(s);
    }
}

static const MemoryRegionOps pm1_cnt_ops = {
    .read = pm1_cnt_read,
    .write = pm1_cnt_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* Intel ICH4 Compatible SMI Handlers */
static void smi_handler_reset(Intel_ICH4_ACPI_State *s)
{
    memset(s->smi_w, 0, sizeof(s->smi_w));
    memset(s->smi_s, 0, sizeof(s->smi_s));

    qemu_printf("Intel ICH4 ACPI: SMI Handler has been reset\n");
}

static void smi_handler_writeb(void *opaque, hwaddr addr, uint64_t val, unsigned width)
{
    Intel_ICH4_ACPI_State *s = opaque;

    /*
        Page 363 of the Intel ICH4 Datasheet.
        Refer to the page above for more details:

        There are various methods to provoke a System Mangament Interrupt(SMI) to the Intel ICH4.
        These are the methods though we probably bother about.

        PMBASE + 30h

        18: INTEL_USB2_EN EHCI SMI(Probs not needed)
        17: LEGACY_USB2_EN Legacy EHCI SMI(Probs not needed)
         7: BIOS Release Allows SCI Interrupts
         5: APMC_EN An SMI will be provoked on every write to the APM Port (B3h)
         4: SLP_SMI_EN Trigger an SMI instead of changing Sleep Modes
         3: LEGACY_USB_EN Legacy USB Functionality triggers SMI
         2: BIOS_EN Global ACPI Register triggers SMI
         0: GBL_SMI_EN The Global Switch
    */

    int address = addr & 7;

    if(address < 4)
        s->smi_w[address] = val;
    else
        s->smi_s[address - 4] &= val;

    qemu_printf("Intel ICH4 ACPI: SMI Handler has been updated 0x%02x%02x%02x%02x\n", s->smi_w[3], s->smi_w[2], s->smi_w[1], s->smi_w[0]);
}

static uint64_t smi_handler_readb(void *opaque, hwaddr addr, unsigned width)
{
    Intel_ICH4_ACPI_State *s = opaque;
    int address = addr & 7;

    if(address < 4)
        return s->smi_w[address] & 0xff;
    else
        return s->smi_s[address - 4] & 0xff;
}

static const MemoryRegionOps intel_ich4_acpi_smi_handler_ops = {
    .read = smi_handler_readb,
    .write = smi_handler_writeb,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* Intel ICH4 SMBus PCI Device */
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

static int intel_ich4_acpi_vmstate_post_load(void *opaque, int version_id)
{
    //Intel_ICH4_ACPI_State *s = opaque;

    /* PM IO too but for LPC */
    //intel_ich4_smbus(dev->config[0x21], dev->config[0x20], dev->config[0x04] & 1, ich4_acpi);
    return 0;
}

static bool intel_ich4_acpi_vmstate_need_smbus(void *opaque, int version_id)
{
    return pm_smbus_vmstate_needed();
}

static const VMStateDescription vmstate_acpi = {
    .name = "intel-ich4-acpi",
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = intel_ich4_acpi_vmstate_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, Intel_ICH4_ACPI_State),
        VMSTATE_UINT16(ar.pm1.evt.sts, Intel_ICH4_ACPI_State),
        VMSTATE_UINT16(ar.pm1.evt.en, Intel_ICH4_ACPI_State),
        VMSTATE_UINT16(ar.pm1.cnt.cnt, Intel_ICH4_ACPI_State),
        VMSTATE_STRUCT(apm, Intel_ICH4_ACPI_State, 0, vmstate_apm, APMState),
        VMSTATE_STRUCT_TEST(smb, Intel_ICH4_ACPI_State, intel_ich4_acpi_vmstate_need_smbus, 3, pmsmb_vmstate, PMSMBus),
        VMSTATE_TIMER_PTR(ar.tmr.timer, Intel_ICH4_ACPI_State),
        VMSTATE_INT64(ar.tmr.overflow_time, Intel_ICH4_ACPI_State),
        VMSTATE_STRUCT(ar.gpe, Intel_ICH4_ACPI_State, 2, vmstate_gpe, ACPIGPE),
        VMSTATE_END_OF_LIST()
    }
};

static void intel_ich4_acpi_reset(DeviceState *dev)
{
    Intel_ICH4_ACPI_State *s = INTEL_ICH4_ACPI(dev);
    PCIDevice *d = PCI_DEVICE(dev);
//    intel_ich4_smbus(dev->config[0x21], dev->config[0x20], dev->config[0x04] & 1, s);

    acpi_pm1_cnt_reset(&s->ar); /* We use stock reset although we use a modified PM1ACNT */
    acpi_pm1_evt_reset(&s->ar);

    acpi_pm_tmr_reset(&s->ar);
    acpi_gpe_reset(&s->ar);
    acpi_update_sci(&s->ar, s->irq);

    smi_handler_reset(s);

    d->config[0x06] = 0x80;
    d->config[0x07] = 0x02;
    d->config[0x20] = 0x01;
    d->config[0x3d] = 0x02;
}

static void intel_ich4_acpi_powerdown_req(Notifier *n, void *opaque)
{
    Intel_ICH4_ACPI_State *s = container_of(n, Intel_ICH4_ACPI_State, powerdown_notifier);

    assert(s != NULL);
    acpi_pm1_evt_power_down(&s->ar);
}

static void intel_ich4_acpi_add_properties(Intel_ICH4_ACPI_State *s)
{
    static const uint8_t acpi_enable_cmd = ACPI_ENABLE;
    static const uint8_t acpi_disable_cmd = ACPI_DISABLE;
    static const uint32_t gpe0_blk = 0x2c;
    static const uint32_t gpe0_blk_len = 4;
    static const uint16_t sci_int = 9;

    object_property_add_uint8_ptr(OBJECT(s), ACPI_PM_PROP_ACPI_ENABLE_CMD, &acpi_enable_cmd, OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(OBJECT(s), ACPI_PM_PROP_ACPI_DISABLE_CMD, &acpi_disable_cmd, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(OBJECT(s), ACPI_PM_PROP_GPE0_BLK, &gpe0_blk, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(OBJECT(s), ACPI_PM_PROP_GPE0_BLK_LEN, &gpe0_blk_len, OBJ_PROP_FLAG_READ);
    object_property_add_uint16_ptr(OBJECT(s), ACPI_PM_PROP_SCI_INT, &sci_int, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(OBJECT(s), ACPI_PM_PROP_PM_IO_BASE, &s->io_base, OBJ_PROP_FLAG_READ);
}

static void intel_ich4_acpi_init(Object *obj)
{
    Intel_ICH4_ACPI_State *s = INTEL_ICH4_ACPI(obj);

    /* SCI Interrupt */
    qdev_init_gpio_out(DEVICE(obj), &s->irq, 1);

    /* SMI Interrupt */
    qdev_init_gpio_out_named(DEVICE(obj), &s->smi_irq, "smi-irq", 1);
}

static void intel_ich4_acpi_realize(PCIDevice *dev, Error **errp)
{
    Intel_ICH4_ACPI_State *s = INTEL_ICH4_ACPI(dev);

    qemu_printf("Intel ICH4 SMBus: I got realized\n");
    qemu_printf("Intel ICH4 ACPI: I got realized\n");

    /* SMBus */
    pm_smbus_init(DEVICE(dev), &s->smb, true);
    memory_region_set_enabled(&s->smb.io, false);
    memory_region_add_subregion(pci_address_space_io(dev), s->smb_io_base, &s->smb.io);
    qemu_printf("Intel ICH4 SMBus: SMBus is up!\n");

    /* ACPI */
    memory_region_init(&s->io, OBJECT(s), "intel-ich4-acpi", 0x7f);
    memory_region_set_enabled(&s->io, false);
    memory_region_add_subregion(pci_address_space_io(dev), s->io_base, &s->io);
    qemu_printf("Intel ICH4 ACPI: ACPI is starting!\n");

    /* APM */
    apm_init(dev, &s->apm, apm_ctrl_changed, s);
    qemu_printf("Intel ICH4 ACPI: APM\n");

    /* Event Handler */
    acpi_pm1_evt_init(&s->ar, pm_tmr_timer, &s->io);
    qemu_printf("Intel ICH4 ACPI: ACPI PM Event Handler\n");

    /* Control */
    qemu_register_wakeup_notifier(&s->ar.wakeup);
    qemu_register_wakeup_support();
    s->ar.wakeup.notify = acpi_notify_wakeup;
    memory_region_init_io(&s->ar.pm1.cnt.io, memory_region_owner(&s->io), &pm1_cnt_ops, s, "acpi_pm1_cnt", 2);
    memory_region_add_subregion(&s->io, 0x04, &s->ar.pm1.cnt.io);
    qemu_printf("Intel ICH4 ACPI: ACPI PM Control\n");

    /* Timer */
    acpi_pm_tmr_init(&s->ar, pm_tmr_timer, &s->io);
    qemu_printf("Intel ICH4 ACPI: ACPI Timer\n");

    /* General Purpose Events */
    memory_region_init_io(&s->io_gpe, memory_region_owner(&s->io), &gpe_ops, s, "acpi_gpe", 4);
    memory_region_add_subregion(&s->io, 0x2c, &s->io_gpe);
    acpi_gpe_init(&s->ar, 4);
    qemu_printf("Intel ICH4 ACPI: ACPI GPE\n");

    /* SMI Handler */
    memory_region_init_io(&s->smi_io, memory_region_owner(&s->io), &intel_ich4_acpi_smi_handler_ops, s, "acpi_smi", 8);
    memory_region_add_subregion(&s->io, 0x30, &s->smi_io);
    qemu_printf("Intel ICH4 ACPI: SMI Handler\n");

    qemu_printf("Intel ICH4 ACPI: ACPI has started!\n");

    /* Notifiers for Qemu */
    s->powerdown_notifier.notify = intel_ich4_acpi_powerdown_req;
    qemu_register_powerdown_notifier(&s->powerdown_notifier);

    intel_ich4_acpi_add_properties(s);
}

static Property intel_ich4_acpi_properties[] = {
    DEFINE_PROP_UINT32("smb_io_base", Intel_ICH4_ACPI_State, smb_io_base, 0),
    DEFINE_PROP_UINT8(ACPI_PM_PROP_S4_VAL, Intel_ICH4_ACPI_State, s4_val, 2),
    DEFINE_PROP_BOOL("smm-compat", Intel_ICH4_ACPI_State, smm_compat, true),
    DEFINE_PROP_BOOL("smm-enabled", Intel_ICH4_ACPI_State, smm_enabled, true),
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
    k->device_id = PCI_DEVICE_ID_INTEL_ICH4_SMBUS;
    k->class_id = PCI_CLASS_SERIAL_SMBUS;

    /* Properties */
    device_class_set_props(dc, intel_ich4_acpi_properties);
    dc->user_creatable = false;
    adevc->send_event = intel_ich4_send_gpe;
}

static const TypeInfo piix4_pm_info = {
    .name          = TYPE_INTEL_ICH4_ACPI,
    .parent        = TYPE_PCI_DEVICE,
    .instance_init  = intel_ich4_acpi_init,
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
