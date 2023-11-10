/*
 * Intel ICH4 Hub to PCI Bridge
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
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_bus.h"
#include "qemu/module.h"
#include "hw/pci-bridge/intel_ich4_hub.h"

/*
 * Stock Qemu PCI Bridge Implementation
 */

static void intel_ich4_hub_realize(PCIDevice *dev, Error **errp)
{
    Intel_ICH4_Hub_State *br = INTEL_ICH4_HUB(dev);

    pci_bridge_initfn(dev, TYPE_PCI_BUS);

    pci_bridge_update_mappings(PCI_BRIDGE(br));
}

static void intel_ich4_hub_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = intel_ich4_hub_realize;
    k->exit = pci_bridge_exitfn;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_ICH4_HUB;
    k->revision = 0x02;
    k->config_write = pci_bridge_write_config;
    k->config_read = pci_default_read_config;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    dc->reset = pci_bridge_reset;
    dc->vmsd = &vmstate_pci_device;
    dc->user_creatable = false;
}

static const TypeInfo intel_ich4_hub_info = {
    .name          = TYPE_INTEL_ICH4_HUB,
    .parent        = TYPE_PCI_BRIDGE,
    .class_init    = intel_ich4_hub_class_init,
    .instance_size = sizeof(Intel_ICH4_Hub_State),
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void intel_ich4_hub_register_types(void)
{
    type_register_static(&intel_ich4_hub_info);
}

type_init(intel_ich4_hub_register_types)
