/*
 * Intel ICH5 IDE Controller
 *
 * Copyright (c) 2003 Fabrice Bellard
 * Copyright (c) 2006 Openedhand Ltd.
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
 *
 */

#include "qemu/osdep.h"
#include "qemu/qemu-print.h"
#include "qapi/error.h"
#include "hw/pci/pci.h"
#include "hw/ide/pci.h"
#include "trace.h"

#include "hw/ide/piix.h"

static uint64_t bmdma_read(void *opaque, hwaddr addr, unsigned size)
{
    BMDMAState *bm = opaque;
    uint32_t val;

    if (size != 1) {
        return ((uint64_t)1 << (size * 8)) - 1;
    }

    switch(addr & 3) {
    case 0:
        val = bm->cmd;
        break;
    case 2:
        val = bm->status;
        break;
    default:
        val = 0xff;
        break;
    }

    trace_bmdma_read(addr, val);
    return val;
}

static void bmdma_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    BMDMAState *bm = opaque;

    if (size != 1) {
        return;
    }

    trace_bmdma_write(addr, val);

    switch(addr & 3) {
    case 0:
        bmdma_cmd_writeb(bm, val);
        break;
    case 2:
        bmdma_status_writeb(bm, val);
        break;
    }
}

static const MemoryRegionOps piix_bmdma_ops = {
    .read = bmdma_read,
    .write = bmdma_write,
};

static void bmdma_setup_bar(PCIIDEState *d)
{
    memory_region_init(&d->bmdma_bar, OBJECT(d), "piix-bmdma-container", 16);
    for(int i = 0; i < 2; i++) {
        BMDMAState *bm = &d->bmdma[i];

        /* Bus Mastering */
        memory_region_init_io(&bm->extra_io, OBJECT(d), &piix_bmdma_ops, bm,"piix-bmdma", 4);
        memory_region_add_subregion(&d->bmdma_bar, i * 8, &bm->extra_io);
        memory_region_init_io(&bm->addr_ioport, OBJECT(d),&bmdma_addr_ioport_ops, bm, "bmdma", 4);
        memory_region_add_subregion(&d->bmdma_bar, i * 8 + 4, &bm->addr_ioport);
    }
}

static void intel_ich5_ide_reset(DeviceState *s)
{
    PCIIDEState *d = PCI_IDE(s);
    PCIDevice *dev = PCI_DEVICE(d);

    dev->config[0x06] = 0x80;
    dev->config[0x07] = 0x02;
    dev->config[0x09] = 0x8a;
    dev->config[0x10] = 0x01;
    dev->config[0x14] = 0x01;
    dev->config[0x18] = 0x01;
    dev->config[0x1c] = 0x01;
    dev->config[0x20] = 0x01;
    dev->config[0x3d] = 0x01;

    ide_bus_reset(&d->bus[0]);
    ide_bus_reset(&d->bus[1]);

    pci_ide_update_mode(d);
}

static void intel_ich5_ide_write(PCIDevice *dev, uint32_t address, uint32_t val, int len)
{
    PCIIDEState *s = PCI_IDE(dev);

    pci_default_write_config(dev, address, val, len);

    for(int i = 0; i < len; i++){
        int ro_only = 0;

        uint8_t new_val = (val >> (i * 8)) & 0xff;

        switch(address + i){
            case 0x04:
                new_val = new_val & 0x07;
            break;

            case 0x05:
                new_val = new_val & 0x04;
            break;

            case 0x07:
                new_val &= ~(new_val & 0x28);
            break;

            case 0x09:
                new_val = new_val & 0x1f;
                new_val |= 0x8a;
            break;

            case 0x10:
            case 0x18:
                new_val = new_val & 0xf8;
                new_val |= 0x01;
            break;

            case 0x14:
            case 0x1c:
                new_val = new_val & 0xfc;
                new_val |= 0x01;
            break;

            case 0x20:
                new_val = new_val & 0xf0;
                new_val |= 0x01;
            break;

            case 0x2c:
            case 0x2d:
            case 0x2e:
            case 0x2f:
                if(dev->config[address + i] != 0) /* After Subsystem ID is programmed, it cannot be reprogrammed again */
                    ro_only = 1;
            break;

            case 0x41:
            case 0x43:
                new_val = new_val & 0xf3;
            break;

            case 0x48:
                new_val = new_val & 0x0f;
            break;

            case 0x4a:
            case 0x4b:
                new_val = new_val & 0x33;
            break;

            case 0x55:
                new_val = new_val & 0xf0;
            break;

            case 0x56:
                new_val = new_val & 0x0f;
            break;

            case 0x11:
            case 0x15:
            case 0x19:
            case 0x1d:
            case 0x3c:
            case 0x40:
            case 0x42:
            case 0x44:
            case 0x54:
                break;

            default:
                ro_only = 1;
            break;
        }

        if(!ro_only) {
            dev->config[address + i] = new_val;
            qemu_printf("Intel ICH5 IDE: dev->regs[0x%02x] = %02x\n", address + i, new_val);
        }
    }

    if(address == 0x09) /* Whenever we update states. Call in the PCI IDE Handler */
            pci_ide_update_mode(s);   
}

static void intel_ich5_ide_realize(PCIDevice *dev, Error **errp)
{
    PCIIDEState *d = PCI_IDE(dev);
    DeviceState *s = DEVICE(dev);

    /* Primary Drives */

    memory_region_init_io(&d->data_bar[0], OBJECT(d), &pci_ide_data_le_ops, &d->bus[0], "intel-ich5-ide-primary-control", 8);
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &d->data_bar[0]);

    memory_region_init_io(&d->cmd_bar[0], OBJECT(d), &pci_ide_cmd_le_ops, &d->bus[0], "intel-ich5-ide-command-control", 4);
    pci_register_bar(dev, 1, PCI_BASE_ADDRESS_SPACE_IO, &d->cmd_bar[0]);

    ide_bus_init(&d->bus[0], sizeof(d->bus[0]), s, 0, 2);
    ide_bus_init_output_irq(&d->bus[0], isa_get_irq(NULL, 14));
    bmdma_init(&d->bus[0], &d->bmdma[0], d);
    ide_bus_register_restart_cb(&d->bus[0]);

    /* Secondary Drives */
    memory_region_init_io(&d->data_bar[1], OBJECT(d), &pci_ide_data_le_ops, &d->bus[1], "intel-ich5-ide-slave-control", 8);
    pci_register_bar(dev, 2, PCI_BASE_ADDRESS_SPACE_IO, &d->data_bar[1]);

    memory_region_init_io(&d->cmd_bar[1], OBJECT(d), &pci_ide_cmd_le_ops, &d->bus[1], "intel-ich5-ide-slave-control", 4);
    pci_register_bar(dev, 3, PCI_BASE_ADDRESS_SPACE_IO, &d->cmd_bar[1]);

    ide_bus_init(&d->bus[1], sizeof(d->bus[1]), s, 1, 2);
    ide_bus_init_output_irq(&d->bus[1], isa_get_irq(NULL, 15));
    bmdma_init(&d->bus[1], &d->bmdma[1], d);
    ide_bus_register_restart_cb(&d->bus[1]);

    /* Bus Mastering */
    bmdma_setup_bar(d);
    pci_register_bar(dev, 4, PCI_BASE_ADDRESS_SPACE_IO, &d->bmdma_bar);
}

static void pci_piix_ide_exitfn(PCIDevice *dev)
{
    PCIIDEState *d = PCI_IDE(dev);

    for (int i = 0; i < 2; i++) {
        memory_region_del_subregion(&d->bmdma_bar, &d->bmdma[i].extra_io);
        memory_region_del_subregion(&d->bmdma_bar, &d->bmdma[i].addr_ioport);
    }
}

static void intel_ich5_ide_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    dc->reset = intel_ich5_ide_reset;
    dc->vmsd = &vmstate_ide_pci;
    k->config_write = intel_ich5_ide_write;
    k->config_read = pci_default_read_config;
    k->realize = intel_ich5_ide_realize;
    k->exit = pci_piix_ide_exitfn;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_ICH5_IDE;
    k->class_id = PCI_CLASS_STORAGE_IDE;
    k->revision = 0x02;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->hotpluggable = false;
    dc->user_creatable = false;
}

static const TypeInfo intel_ich5_ide_info = {
    .name          = TYPE_INTEL_ICH5_IDE,
    .parent        = TYPE_PCI_IDE,
    .class_init    = intel_ich5_ide_class_init,
};

static void intel_ich5_ide_register_types(void)
{
    type_register_static(&intel_ich5_ide_info);
}

type_init(intel_ich5_ide_register_types)
