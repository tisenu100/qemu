/*
 * Intel ICH4 LPC
 *
 * Copyright (c) 2006 Fabrice Bellard
 * Copyright (c) 2023 Tiseno100
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
#include "qemu/range.h"
#include "qapi/error.h"
#include "hw/dma/i8257.h"
#include "hw/southbridge/ich4_lpc.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/isa/isa.h"
#include "sysemu/runstate.h"
#include "migration/vmstate.h"
#include "qemu/qemu-print.h"

int intel_ich4_irq_table(int irq);

int intel_ich4_irq_table(int irq)
{   
    switch(irq)
    {
        case 0x03:
        case 0x04:
        case 0x05:
        case 0x06:
        case 0x07:
        case 0x09:
        case 0x0a:
        case 0x0b:
        case 0x0c:
        case 0x0e:
        case 0x0f:
            return irq;
        
        default: /*Invalid IRQ */
            return 0;
    }
}

static PCIINTxRoute route_intx_pin_to_irq(void *opaque, int pin)
{
    ICH4State *ich4 = opaque;
    int irq = intel_ich4_irq_table(ich4->dev.config[((pin > 3) ? 0x64 : 0x60) + pin] & 0x0f);
    PCIINTxRoute route;

    route.mode = !!irq ? PCI_INTX_ENABLED : PCI_INTX_DISABLED;
    route.irq = !!irq ? irq : -1;

    return route;
}

static void intel_ich4_pirq(uint32_t val, ICH4State *ich4)
{
    PCIBus *bus = pci_get_bus(&ich4->dev);
    PCIDevice *dev = &ich4->dev;

    val -= (val > 0x64) ? 0x64 : 0x60; /* Current PIRQ pin */
    int level = pci_bus_get_irq_level(bus, val); /* PIRQ Level */
    int pic_irq = dev->config[0x60 + val] & 0x0f; /* Pick up the IRQ from the register */
    uint64_t mask; /* Qemu magic */

    ich4->pic_levels = 0;

    mask = 1ULL << ((pic_irq * 8ULL) + val);
    ich4->pic_levels &= ~mask;
    ich4->pic_levels |= mask * !!level;

    qemu_set_irq(ich4->pic[pic_irq], !!(ich4->pic_levels & (((1ULL << 8ULL) - 1) << (pic_irq * 8ULL))));
}

static void intel_ich4_set_irq(void *opaque, int pirq, int level)
{
    ICH4State *ich4 = opaque;
    intel_ich4_pirq(pirq, ich4);
}

static void intel_ich4_write_config(PCIDevice *dev, uint32_t address, uint32_t val, int len)
{
    qemu_printf("Intel ICH4: dev->pci_conf[%02x] = 0x%02x\n", address, val);

    pci_default_write_config(dev, address, val, len);
    if (ranges_overlap(address, len, ICH4_PIRQCA, 4)) {
        ICH4State *ich4 = ICH4_PCI_DEVICE(dev);

        pci_bus_fire_intx_routing_notifier(pci_get_bus(&ich4->dev));

        /* Qemu has some way to do IRQ updates 
           Let's figure

           Per page 299 on Intel ICH4 datasheet
           0x60 PIRQA#
           0x61 PIRQB#
           0x62 PIRQC#
           0x63 PIRQD#
           0x68 PIRQE#
           0x69 PIRQF#
           0x6a PIRQG#
           0x6b PIRQH#
        */
        intel_ich4_pirq(address, ich4);
    }
}

static void ich4_reset(DeviceState *dev)
{
    ICH4State *d = ICH4_PCI_DEVICE(dev);

    d->pic_levels = 0;
    d->rcr = 0;
}

static int ich4_post_load(void *opaque, int version_id)
{
    ICH4State *ich4 = opaque;
    int pirq;

    /*
     * Because the i8259 has not been deserialized yet, qemu_irq_raise
     * might bring the system to a different state than the saved one;
     * for example, the interrupt could be masked but the i8259 would
     * not know that yet and would trigger an interrupt in the CPU.
     *
     * Here, we update irq levels without raising the interrupt.
     * Interrupt state will be deserialized separately through the i8259.
     */
    ich4->pic_levels = 0;
    for (pirq = 0; pirq < 8ULL; pirq++) {
        intel_ich4_pirq(pirq, ich4);
    }
    return 0;
}

static int ich4_pre_save(void *opaque)
{
    ICH4State *ich4 = opaque;

    for (int i = 0; i < ARRAY_SIZE(ich4->pci_irq_levels_vmstate); i++) {
        ich4->pci_irq_levels_vmstate[i] = pci_bus_get_irq_level(pci_get_bus(&ich4->dev), i);
    }

    return 0;
}

static bool piix_rcr_needed(void *opaque)
{
    ICH4State *ich4 = opaque;

    return (ich4->rcr != 0);
}

static const VMStateDescription vmstate_ich4_rcr = {
    .name = "PIIX Compatible Reset Control",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = piix_rcr_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(rcr, ICH4State),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ich4 = {
    .name = "Intel ICH4 LPC",
    .version_id = 3,
    .minimum_version_id = 2,
    .post_load = ich4_post_load,
    .pre_save = ich4_pre_save,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, ICH4State),
        VMSTATE_INT32_ARRAY_V(pci_irq_levels_vmstate, ICH4State,
                              ICH4_NUM_PIRQS, 3),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription*[]) {&vmstate_ich4_rcr, NULL}
};


static void rcr_write(void *opaque, hwaddr addr, uint64_t val, unsigned len)
{
    ICH4State *d = opaque;

    if (val & 4) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    }
    d->rcr = val & 2; /* keep System Reset type only */
}

static uint64_t rcr_read(void *opaque, hwaddr addr, unsigned len)
{
    ICH4State *d = opaque;

    return d->rcr;
}

static const MemoryRegionOps rcr_ops = {
    .read = rcr_read,
    .write = rcr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void pci_ich4_realize(PCIDevice *dev, Error **errp)
{
    ICH4State *d = ICH4_PCI_DEVICE(dev);
    ISABus *isa_bus;

    qemu_printf("Intel ICH4 LPC: I got realized!\n");

    /* Form the LPC Bus */
    qemu_printf("Intel ICH4 LPC: LPC bus ready\n");
    isa_bus = isa_bus_new(DEVICE(d), pci_address_space(dev), pci_address_space_io(dev), errp);
    if (!isa_bus) return;

    /* PIIX Compatible Reset Port */
    memory_region_init_io(&d->rcr_mem, OBJECT(dev), &rcr_ops, d, "piix-compatible-reset-control", 1);
    memory_region_add_subregion_overlap(pci_address_space_io(dev), 0xcf9, &d->rcr_mem, 1);
    qemu_printf("Intel ICH4 LPC: PIIX Compatible reset control is mounted at port 0xcf9h\n");

    /* Prepare the DMA controller */
    i8257_dma_init(isa_bus, 1);
    qemu_printf("Intel ICH4 LPC: DMA Controller is up\n");

    /* MC146818 Compatible NVR */
    if (!qdev_realize(DEVICE(&d->rtc), BUS(isa_bus), errp))
        return;
    qemu_printf("Intel ICH4 LPC: NVR has been sanitized\n");
}

static void pci_ich4_init(Object *obj)
{
    ICH4State *d = ICH4_PCI_DEVICE(obj);

    /* MC146818 Compatible NVR/CMOS */
    qemu_printf("Intel ICH4 LPC: Mounting the NVR up\n");
    object_initialize_child(obj, "rtc", &d->rtc, TYPE_MC146818_RTC);
}

static void pci_ich4_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->config_write = intel_ich4_write_config;
    k->config_read = pci_default_read_config;
    dc->reset       = ich4_reset;
    dc->desc        = "Intel ICH4 LPC Bridge";
    dc->vmsd        = &vmstate_ich4;
    dc->hotpluggable   = false;
    k->vendor_id    = PCI_VENDOR_ID_INTEL;
    k->device_id    = PCI_DEVICE_ID_INTEL_ICH4_LPC;
    k->class_id     = PCI_CLASS_BRIDGE_ISA;
    dc->user_creatable = false;
}

static const TypeInfo ich4_pci_type_info = {
    .name = TYPE_ICH4_PCI_DEVICE,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(ICH4State),
    .instance_init = pci_ich4_init,
    .abstract = true,
    .class_init = pci_ich4_class_init,
    .interfaces = (InterfaceInfo[]) {{ INTERFACE_CONVENTIONAL_PCI_DEVICE }, { }, },
};

static void ich4_realize(PCIDevice *dev, Error **errp)
{
    ERRP_GUARD();
    ICH4State *ich4 = ICH4_PCI_DEVICE(dev);
    PCIBus *pci_bus = pci_get_bus(dev);

    pci_ich4_realize(dev, errp);
    if (*errp) {
        return;
    }

    pci_bus_irqs(pci_bus, intel_ich4_set_irq, ich4, 8ULL);
    pci_bus_set_route_irq_fn(pci_bus, route_intx_pin_to_irq);
}

static void ich4_class_init(ObjectClass *klass, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = ich4_realize;
}

static const TypeInfo ich4_info = {
    .name          = TYPE_ICH4_DEVICE,
    .parent        = TYPE_ICH4_PCI_DEVICE,
    .class_init    = ich4_class_init,
};

static void ich4_register_types(void)
{
    type_register_static(&ich4_pci_type_info);
    type_register_static(&ich4_info);
}

type_init(ich4_register_types)
