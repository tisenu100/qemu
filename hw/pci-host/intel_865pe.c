/*
 * Intel 865P/865PE Memory Controller Hub
 *
 * Copyright (c) 2006 Fabrice Bellard
 * Copyright (c) 2023 Tiseno100
 * 
 * Portions of q35.c
 * Copyright (c) 2009, 2010, 2011
 *               Isaku Yamahata <yamahata at valinux co jp>
 *               VA Linux Systems Japan K.K.
 * Copyright (C) 2012 Jason Baron <jbaron@redhat.com>
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

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/qemu-print.h"
#include "qemu/range.h"
#include "hw/i386/pc.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "qom/object.h"

#include "hw/pci-host/intel_865pe.h"

OBJECT_DECLARE_SIMPLE_TYPE(Intel_865PE_State, INTEL_865PE_HOST_BRIDGE)

struct Intel_865PE_State {
    PCIHostState parent_obj;

    MemoryRegion *system_memory;
    MemoryRegion *io_memory;
    MemoryRegion *pci_address_space;
    MemoryRegion *ram_memory;
    Range pci_hole;
    uint64_t below_4g_mem_size;
    uint64_t above_4g_mem_size;
    uint64_t pci_hole64_size;
    bool pci_hole64_fix;

    char *pci_type;
};

static void intel_865pe_realize(PCIDevice *dev, Error **errp)
{
    qemu_printf("Intel 865PE MCH: I got realized!\n");

    /* Qemu does this on the 440FX. Declare we got no IOMMU support */
    if (object_property_get_bool(qdev_get_machine(), "iommu", NULL))
        warn_report("Intel 865PE: IOMMU is not supported!");
}

static void intel_865pe_smram(Intel_865PE_PCI_State *d)
{
    PCIDevice *pd = PCI_DEVICE(d);

    /*
        Per the Intel 865PE datasheet

        Bit 6: When D_OPEN=1 and D_LCK=0, the SMM space SDRAM
               is made visible even when SMM decode is not active. This is intended to help BIOS initialize SMM
               space. Software should ensure that D_OPEN=1 and D_CLS=1 are not set at the same time.

        Bit 3: If set to a 1, Compatible SMRAM functions are
               enabled, providing 128 KB of SDRAM accessible at the A0000h address while in SMM (ADS# with
               SMM decode). To enable Extended SMRAM function this bit has be set to 1.


        We just implement whatever Qemu needs according to the 440FX. The BIOS use the low SMRAM anyways.
    */

    memory_region_transaction_begin();

    if(pd->config[0x9d] & 0x08) {
        if(pd->config[0x9d] & 0x40){
            qemu_printf("Intel 865PE: SMRAM Region is open\n");
            memory_region_set_enabled(&d->smram_region, false);
            memory_region_set_enabled(&d->high_smram_region, !(pd->config[0x9e] & 0x80));
            memory_region_set_enabled(&d->low_smram, true);
            memory_region_set_enabled(&d->high_smram, pd->config[0x9e] & 0x80);
        }
        else if(pd->config[0x9d] & 0x20) {
            qemu_printf("Intel 865PE: SMRAM Region is closed\n");
            memory_region_set_enabled(&d->smram_region, true);
            memory_region_set_enabled(&d->low_smram, false);
        }
        else {
            qemu_printf("Intel 865PE: SMRAM Region is enabled\n");
            memory_region_set_enabled(&d->smram_region, true);
            memory_region_set_enabled(&d->high_smram_region, true);
            memory_region_set_enabled(&d->low_smram, true);
            memory_region_set_enabled(&d->high_smram, pd->config[0x9e] & 0x80);
        }
    } else {
        qemu_printf("Intel 865PE: SMRAM Region is disabled\n");
        memory_region_set_enabled(&d->low_smram, false);
        memory_region_set_enabled(&d->high_smram, false);
        memory_region_set_enabled(&d->smram_region, true);
        memory_region_set_enabled(&d->high_smram_region, true);
    }

    memory_region_transaction_commit();

}

static void intel_865pe_pam(Intel_865PE_PCI_State *d)
{
    PCIDevice *pd = PCI_DEVICE(d);

    memory_region_transaction_begin();

    for (int i = 0; i < ARRAY_SIZE(d->pam_regions); i++)
        pam_update(&d->pam_regions[i], i, pd->config[0x90 + DIV_ROUND_UP(i, 2)]);
    
    memory_region_transaction_commit();
}

static void intel_865pe_write_config(PCIDevice *dev, uint32_t address, uint32_t val, int len)
{
    Intel_865PE_PCI_State *d = INTEL_865PE_PCI_DEVICE(dev);
    pci_default_write_config(dev, address, val, len);

    for(int i = 0; i < len; i++){
        int ro_only = 0;

        uint8_t new_val = (val >> (i * 8)) & 0xff;

        switch(address + i){
            case 0x05:
                new_val = new_val & 0x01;
            break;

            case 0x07:
                new_val &= ~(new_val & 0x03);
            break;

            case 0x12:
                new_val = new_val & 0xfc;
            break;

            case 0x2c:
            case 0x2d:
            case 0x2e:
            case 0x2f:
                if(dev->config[address + i] != 0) /* After Subsystem ID is programmed, it cannot be reprogrammed again */
                    ro_only = 1;
            break;

            case 0x51:
                new_val = new_val & 2;
            break;

            case 0x53: /* Device not present bit. No clearance to what it does */
                new_val = new_val & 1;
            break;

            case 0x60:
                new_val = new_val & 0x18;
            break;

            case 0x97:
                new_val = new_val & 0x80;
            break;

            case 0x9d:
                if(dev->config[0x9d] & 0x10) /* D_LCK bit. If set don't program SMRAM */
                    ro_only = 1;
                else
                    new_val = (new_val & 0x78) | 4;
            break;

            case 0x9e:
                if(dev->config[0x9d] & 0x10) /* D_LCK bit. If set don't program SMRAM */
                    ro_only = 1;
                else
                    new_val = (new_val & 0x87) | 0x38;
            break;

            case 0xa8:
                new_val = new_val & 0x17;
            break;

            case 0xa9:
                new_val = new_val & 0x1e;
                new_val |= 0x01;
            break;

            case 0xb0:
                new_val = new_val & 0x81;
            break;

            case 0xb4:
                new_val = new_val & 0x3f;
            break;

            case 0xb9:
                new_val = new_val & 0xf0;
            break;

            case 0xbc:
                new_val = new_val & 0xf1;
            break;

            case 0xbd:
                new_val = new_val & 0xf1;
            break;

            case 0xc4:
                new_val = new_val & 0xf1;
            break;

            case 0xc6: /* We enforce 200Mhz with FSB 800Mhz */
                new_val = (new_val & 0x20) | 0x02;
            break;

            case 0xc7:
                new_val = new_val & 0xec;
            break;

            case 0xc8:
                new_val &= ~(new_val & 0x3c);
            break;

            case 0xc9:
                new_val &= ~(new_val & 0x03);
            break;

            case 0xca:
                new_val = new_val & 0x7e;
            break;

            case 0xcb:
                new_val = new_val & 0x02;
            break;

            case 0x13:
            case 0x90:
            case 0x91:
            case 0x92:
            case 0x93:
            case 0x94:
            case 0x95:
            case 0x96:
            case 0xba:
            case 0xbb:
            case 0xc5:
            case 0xde:
            case 0xdf:
            break;

            default:
                ro_only = 1;
            break;
        }

        if(!ro_only) {
            dev->config[address + i] = new_val;

            if(((address + i) < 0x90) || ((address + i) > 0x96)) /* Don't log PAM Writes */
                qemu_printf("Intel 865PE MCH: dev->regs[0x%02x] = %02x\n", address + i, new_val);
        }

    }


    /* Intel 865PE functionality */
    switch(address){
        case 0x90: /* PAM "Shadow RAM" */
        case 0x91:
        case 0x92:
        case 0x93:
        case 0x94:
        case 0x95:
        case 0x96:
            intel_865pe_pam(d);
        break;

        case 0x9d: /* SMRAM */
        case 0x9e:
            intel_865pe_smram(d);
        break;
    }
}

static int intel_865pe_post_load(void *opaque, int version_id)
{
    Intel_865PE_PCI_State *dev = opaque;

    intel_865pe_pam(dev);
    intel_865pe_smram(dev);
    return 0;
}

static const VMStateDescription vmstate_intel_865pe = {
    .name = "Intel 865PE",
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = intel_865pe_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, Intel_865PE_PCI_State),
        /* Used to be smm_enabled, which was basically always zero because
         * SeaBIOS hardly uses SMM.  SMRAM is now handled by CPU code.
         */
        VMSTATE_UNUSED(1),
        VMSTATE_END_OF_LIST()
    }
};

static void intel_865pe_pcihost_get_pci_hole_start(Object *obj, Visitor *v,
                                              const char *name, void *opaque,
                                              Error **errp)
{
    Intel_865PE_State *s = INTEL_865PE_HOST_BRIDGE(obj);
    uint64_t val64;
    uint32_t value;

    val64 = range_is_empty(&s->pci_hole) ? 0 : range_lob(&s->pci_hole);
    value = val64;
    assert(value == val64);
    visit_type_uint32(v, name, &value, errp);
}

static void intel_865pe_pcihost_get_pci_hole_end(Object *obj, Visitor *v,
                                            const char *name, void *opaque,
                                            Error **errp)
{
    Intel_865PE_State *s = INTEL_865PE_HOST_BRIDGE(obj);
    uint64_t val64;
    uint32_t value;

    val64 = range_is_empty(&s->pci_hole) ? 0 : range_upb(&s->pci_hole) + 1;
    value = val64;
    assert(value == val64);
    visit_type_uint32(v, name, &value, errp);
}

/*
 * The 64bit PCI hole start is set by the Guest firmware
 * as the address of the first 64bit PCI MEM resource.
 * If no PCI device has resources on the 64bit area,
 * the 64bit PCI hole will start after "over 4G RAM" and the
 * reserved space for memory hotplug if any.
 */
static uint64_t intel_865pe_pcihost_get_pci_hole64_start_value(Object *obj)
{
    PCIHostState *h = PCI_HOST_BRIDGE(obj);
    Intel_865PE_State *s = INTEL_865PE_HOST_BRIDGE(obj);
    Range w64;
    uint64_t value;

    pci_bus_get_w64_range(h->bus, &w64);
    value = range_is_empty(&w64) ? 0 : range_lob(&w64);
    if (!value && s->pci_hole64_fix) {
        value = pc_pci_hole64_start();
    }
    return value;
}

static void intel_865pe_pcihost_get_pci_hole64_start(Object *obj, Visitor *v,
                                                const char *name,
                                                void *opaque, Error **errp)
{
    uint64_t hole64_start = intel_865pe_pcihost_get_pci_hole64_start_value(obj);

    visit_type_uint64(v, name, &hole64_start, errp);
}

/*
 * The 64bit PCI hole end is set by the Guest firmware
 * as the address of the last 64bit PCI MEM resource.
 * Then it is expanded to the PCI_HOST_PROP_PCI_HOLE64_SIZE
 * that can be configured by the user.
 */
static void intel_865pe_pcihost_get_pci_hole64_end(Object *obj, Visitor *v,
                                              const char *name, void *opaque,
                                              Error **errp)
{
    PCIHostState *h = PCI_HOST_BRIDGE(obj);
    Intel_865PE_State *s = INTEL_865PE_HOST_BRIDGE(obj);
    uint64_t hole64_start = intel_865pe_pcihost_get_pci_hole64_start_value(obj);
    Range w64;
    uint64_t value, hole64_end;

    pci_bus_get_w64_range(h->bus, &w64);
    value = range_is_empty(&w64) ? 0 : range_upb(&w64) + 1;
    hole64_end = ROUND_UP(hole64_start + s->pci_hole64_size, 1ULL << 30);
    if (s->pci_hole64_fix && value < hole64_end) {
        value = hole64_end;
    }
    visit_type_uint64(v, name, &value, errp);
}

static void intel_865pe_pcihost_initfn(Object *obj)
{
    Intel_865PE_State *s = INTEL_865PE_HOST_BRIDGE(obj);
    PCIHostState *phb = PCI_HOST_BRIDGE(obj);

    memory_region_init_io(&phb->conf_mem, obj, &pci_host_conf_le_ops, phb, "pci-conf-idx", 4);
    memory_region_init_io(&phb->data_mem, obj, &pci_host_data_le_ops, phb, "pci-conf-data", 4);

    /* Entire Memory Block */
    object_property_add_link(obj, PCI_HOST_PROP_RAM_MEM, TYPE_MEMORY_REGION, (Object **) &s->ram_memory, qdev_prop_allow_set_link_before_realize, 0);

    /* PCI Memory Range */
    object_property_add_link(obj, PCI_HOST_PROP_PCI_MEM, TYPE_MEMORY_REGION, (Object **) &s->pci_address_space, qdev_prop_allow_set_link_before_realize, 0);

    /* System Memory Space */
    object_property_add_link(obj, PCI_HOST_PROP_SYSTEM_MEM, TYPE_MEMORY_REGION, (Object **) &s->system_memory, qdev_prop_allow_set_link_before_realize, 0);

    /* I/O Memory */
    object_property_add_link(obj, PCI_HOST_PROP_IO_MEM, TYPE_MEMORY_REGION, (Object **) &s->io_memory, qdev_prop_allow_set_link_before_realize, 0);
}

static void intel_865pe_reset(DeviceState *s)
{
    PCIDevice *dev = PCI_DEVICE(s);
    Intel_865PE_PCI_State *d = INTEL_865PE_PCI_DEVICE(s);

    dev->config[0x04] = 0x06;
    dev->config[0x06] = 0x90;
    dev->config[0x10] = 0x08;
    dev->config[0x2c] = 0x00;
    dev->config[0x2d] = 0x00;
    dev->config[0x2e] = 0x00;
    dev->config[0x2f] = 0x00;
    dev->config[0x34] = 0xe4;
    dev->config[0x9d] = 0x02;
    dev->config[0x9e] = 0x38;
    dev->config[0xa0] = 0x02;
    dev->config[0xa2] = 0x30;
    dev->config[0xa4] = 0x17;
    dev->config[0xa5] = 0x02;
    dev->config[0xa7] = 0x1f;
    dev->config[0xa9] = 0x01; /* AGP Enabled */
    dev->config[0xbc] = 0x10;
    dev->config[0xbd] = 0x10;
    dev->config[0xc5] = 0x04;
    dev->config[0xc6] = 0x02;
    dev->config[0xe4] = 0x09;
    dev->config[0xe5] = 0xa0;
    dev->config[0xe6] = 0x04;
    dev->config[0xe7] = 0xf1;

    intel_865pe_pam(d);
    intel_865pe_smram(d);
}

static void intel_865pe_pcihost_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    Intel_865PE_State *s = INTEL_865PE_HOST_BRIDGE(dev);
    PCIHostState *phb = PCI_HOST_BRIDGE(dev);

    PCIBus *pci_bus = pci_root_bus_new(dev, NULL, s->pci_address_space, s->io_memory, 0, TYPE_PCI_BUS);
    phb->bus = pci_bus;

    PCIDevice *d = pci_create_simple(pci_bus, PCI_DEVFN(0, 0), s->pci_type);
    Intel_865PE_PCI_State *f = INTEL_865PE_PCI_DEVICE(d);

    /* Intel 865PE speaks on registers Cf8h & Cfch */
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    memory_region_add_subregion(s->io_memory, 0xcf8, &phb->conf_mem);
    sysbus_init_ioports(sbd, 0xcf8, 4);

    memory_region_add_subregion(s->io_memory, 0xcfc, &phb->data_mem);
    sysbus_init_ioports(sbd, 0xcfc, 4);

    /* Qemu sets Port CF8h(Config Address Registers) as coalesced pio */
    memory_region_set_flush_coalesced(&phb->data_mem);
    memory_region_add_coalescing(&phb->conf_mem, 0, 4);

    /* Setup the addressing of the I/O APIC */
    range_set_bounds(&s->pci_hole, s->below_4g_mem_size, IO_APIC_DEFAULT_ADDRESS - 1);

    /* Setup PCI memory mapping */
    pc_pci_as_mapping_init(s->system_memory, s->pci_address_space);

    /* If the SMRAM region is disabled then allow it to be seen and accessed */
    memory_region_init_alias(&f->smram_region, OBJECT(f), "smram-region", s->pci_address_space, 0xa0000, 0x20000);
    memory_region_add_subregion_overlap(s->system_memory, 0xa0000, &f->smram_region, 1);
    memory_region_set_enabled(&f->smram_region, true);
    memory_region_init_alias(&f->high_smram_region, OBJECT(f), "high-smram-region", s->ram_memory, 0xfeda0000, 0x20000);
    memory_region_add_subregion_overlap(s->system_memory, 0xfeda0000, &f->high_smram_region, 1);
    memory_region_set_enabled(&f->high_smram_region, true);

    /* SMRAM as seen by SMM CPUs */
    memory_region_init(&f->smram, OBJECT(f), "smram", 4 * GiB);
    memory_region_set_enabled(&f->smram, true);

    /* CPU SMM 30000-4FFFF section */
    memory_region_init_alias(&f->cpu_smram, OBJECT(f), "cpu-smm", s->ram_memory, 0x30000, 0x20000);
    memory_region_set_enabled(&f->cpu_smram, true);
    memory_region_add_subregion(&f->smram, 0x30000, &f->cpu_smram);

    /* Low SMRAM A0000-BFFFF section */
    memory_region_init_alias(&f->low_smram, OBJECT(f), "smram-low", s->ram_memory, 0xa0000, 0x20000);
    memory_region_set_enabled(&f->low_smram, true);
    memory_region_add_subregion(&f->smram, 0xa0000, &f->low_smram);

    /* High SMRAM FEDA0000-FEDBFFFF mirror */
    memory_region_init_alias(&f->high_smram, OBJECT(f), "smram-high", s->ram_memory, 0xfeda0000, 0x20000);
    memory_region_set_enabled(&f->high_smram, true);
    memory_region_add_subregion(&f->smram, 0xfeda0000, &f->high_smram);

    object_property_add_const_link(qdev_get_machine(), "smram", OBJECT(&f->smram));

    /* Start the PAM. This is just Shadow RAM. Qemu has it's own PAM implementation. We just follow behind as Intel 865PE is no different. */
    init_pam(&f->pam_regions[0], OBJECT(f), s->ram_memory, s->system_memory, s->pci_address_space, PAM_BIOS_BASE, PAM_BIOS_SIZE);
    for (unsigned i = 0; i < ARRAY_SIZE(f->pam_regions) - 1; ++i) {
        init_pam(&f->pam_regions[i + 1], OBJECT(f), s->ram_memory,
                 s->system_memory, s->pci_address_space,
                 PAM_EXPAN_BASE + i * PAM_EXPAN_SIZE, PAM_EXPAN_SIZE);
    }

    intel_865pe_pam(f);
    intel_865pe_smram(f);
}

static void intel_865pe_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = intel_865pe_realize;
    k->config_write = intel_865pe_write_config;
    k->config_read = pci_default_read_config;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_865PE_MCH;
    k->revision = 0x02;
    k->class_id = PCI_CLASS_BRIDGE_HOST;
    dc->desc = "Intel 865PE";
    dc->reset = intel_865pe_reset;
    dc->vmsd = &vmstate_intel_865pe;
    dc->user_creatable = false;
    dc->hotpluggable   = false;
}

static const TypeInfo intel_865pe_info = {
    .name          = TYPE_INTEL_865PE_PCI_DEVICE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Intel_865PE_PCI_State),
    .class_init    = intel_865pe_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static const char *intel_865pe_pcihost_root_bus_path(PCIHostState *host_bridge, PCIBus *rootbus)
{
    return "0000:00";
}

static Property intel_865pe_props[] = {
    DEFINE_PROP_SIZE(PCI_HOST_PROP_PCI_HOLE64_SIZE, Intel_865PE_State,
                     pci_hole64_size, 1ULL << 31),
    DEFINE_PROP_SIZE(PCI_HOST_BELOW_4G_MEM_SIZE, Intel_865PE_State,
                     below_4g_mem_size, 0),
    DEFINE_PROP_SIZE(PCI_HOST_ABOVE_4G_MEM_SIZE, Intel_865PE_State,
                     above_4g_mem_size, 0),
    DEFINE_PROP_BOOL("x-pci-hole64-fix", Intel_865PE_State, pci_hole64_fix, true),
    DEFINE_PROP_STRING( INTEL_865PE_HOST_PROP_PCI_TYPE, Intel_865PE_State, pci_type),
    DEFINE_PROP_END_OF_LIST(),
};

static void intel_865pe_pcihost_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIHostBridgeClass *hc = PCI_HOST_BRIDGE_CLASS(klass);

    hc->root_bus_path = intel_865pe_pcihost_root_bus_path;
    dc->realize = intel_865pe_pcihost_realize;
    dc->fw_name = "pci";
    device_class_set_props(dc, intel_865pe_props);
    /* Reason: needs to be wired up by pc_init1 */
    dc->user_creatable = false;

    object_class_property_add(klass, PCI_HOST_PROP_PCI_HOLE_START, "uint32",
                              intel_865pe_pcihost_get_pci_hole_start,
                              NULL, NULL, NULL);

    object_class_property_add(klass, PCI_HOST_PROP_PCI_HOLE_END, "uint32",
                              intel_865pe_pcihost_get_pci_hole_end,
                              NULL, NULL, NULL);

    object_class_property_add(klass, PCI_HOST_PROP_PCI_HOLE64_START, "uint64",
                              intel_865pe_pcihost_get_pci_hole64_start,
                              NULL, NULL, NULL);

    object_class_property_add(klass, PCI_HOST_PROP_PCI_HOLE64_END, "uint64",
                              intel_865pe_pcihost_get_pci_hole64_end,
                              NULL, NULL, NULL);
}

static const TypeInfo intel_865pe_pcihost_info = {
    .name          = TYPE_INTEL_865PE_HOST_BRIDGE,
    .parent        = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(Intel_865PE_State),
    .instance_init = intel_865pe_pcihost_initfn,
    .class_init    = intel_865pe_pcihost_class_init,
};

static void intel_865pe_register_types(void)
{
    type_register_static(&intel_865pe_info);
    type_register_static(&intel_865pe_pcihost_info);
}

type_init(intel_865pe_register_types)
