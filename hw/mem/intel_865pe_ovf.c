/*
 * Intel 865PE Overflow
 *
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

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/qemu-print.h"
#include "qemu/range.h"
#include "hw/i386/pc.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bus.h"
#include "hw/pci/pci_ids.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "qom/object.h"

#include "hw/mem/intel_865pe_ovf.h"

static uint64_t dram_read(void *opaque, hwaddr addr, unsigned width)
{
    Intel_865PE_OVF_State *d = opaque;
    addr = addr & 0xff;

    return d->regs[(int)addr];
}

static void dram_write(void *opaque, hwaddr addr, uint64_t val, unsigned width)
{
    Intel_865PE_OVF_State *d = opaque;
    addr = addr & 0xff;
    qemu_printf("Intel 865PE DRAM: Writing 0x%02x on 0x%02x\n", (int)val, (int)addr);
    d->regs[(int)addr] = val;
}

static const MemoryRegionOps dram_ops = {
    .read = dram_read,
    .write = dram_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void intel_865pe_dram_remap(PCIDevice *dev)
{
    Intel_865PE_OVF_State *d = INTEL_865PE_OVF(dev);
    d->dram_io_base = (dev->config[0x13] << 24) | (dev->config[0x12] << 16) | (dev->config[0x11] << 8);
    bool enabled = !!(dev->config[0x04] & 0x02) && (d->dram_io_base != 0);

    memory_region_transaction_begin();
    memory_region_set_enabled(&d->dram, true);
    memory_region_set_address(&d->dram, d->dram_io_base);
    memory_region_transaction_commit();

    if(enabled)
        qemu_printf("Intel 865PE DRAM: Remapped to position 0x%08x\n", d->dram_io_base);
}

static void intel_865pe_ovf_write_config(PCIDevice *dev, uint32_t address, uint32_t val, int len)
{
    for(int i = 0; i < len; i++){
        int ro_only = 0;

        uint8_t new_val = (val >> (i * 8)) & 0xff;

        switch(address + i){
            case 0x04:
                new_val = new_val & 0x03;
            break;

            case 0x11:
                new_val = new_val & 0xf0;
            break;

            case 0x12:
            case 0x13:
            break;

            default:
                ro_only = 1;
            break;
        }

        if(!ro_only) {
            pci_default_write_config(dev, address, val, len);
            dev->config[address + i] = new_val;
            qemu_printf("Intel 865PE OVF: dev->regs[0x%02x] = %02x\n", address + i, new_val);
        }
    }

    switch(address)
    {
        case 0x04:
        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13:
            intel_865pe_dram_remap(dev);
        break;
    }
}

static void intel_865pe_ovf_reset(DeviceState *dev)
{
    PCIDevice *d = PCI_DEVICE(dev);
    intel_865pe_dram_remap(d);
}

static void intel_865pe_ovf_realize(PCIDevice *dev, Error **errp)
{
    Intel_865PE_OVF_State *d = INTEL_865PE_OVF(dev);

    /* DRAM Handler (MMR) */
    memory_region_init_io(&d->dram, OBJECT(d), &dram_ops, d, "intel-865pe-ovf", 0x10000);
    memory_region_set_enabled(&d->dram, false);
    memory_region_add_subregion(pci_address_space(dev), d->dram_io_base, &d->dram);
}

static void intel_865pe_ovf_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = intel_865pe_ovf_realize;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_865PE_OVF;
    k->revision = 0x02;
    k->class_id = PCI_CLASS_SYSTEM_OTHER;
    k->config_write = intel_865pe_ovf_write_config;
    k->config_read = pci_default_read_config;
    dc->reset = intel_865pe_ovf_reset;
    dc->vmsd = &vmstate_pci_device;
    dc->user_creatable = false;
}

static const TypeInfo intel_865pe_ovf_info = {
    .name          = TYPE_INTEL_865PE_OVF,
    .parent        = TYPE_PCI_DEVICE,
    .class_init    = intel_865pe_ovf_class_init,
    .instance_size = sizeof(Intel_865PE_OVF_State),
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void intel_865pe_ovf_register_types(void)
{
    type_register_static(&intel_865pe_ovf_info);
}

type_init(intel_865pe_ovf_register_types)
