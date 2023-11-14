/*
 * Intel ICH4 LPC
 *
 * Copyright (c) 2006 Fabrice Bellard
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
#include "qemu/range.h"
#include "qapi/error.h"
#include "hw/dma/i8257.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/isa/isa.h"
#include "sysemu/runstate.h"
#include "migration/vmstate.h"
#include "qemu/qemu-print.h"

#include "hw/acpi/intel_ich4_acpi.h"
#include "hw/southbridge/intel_ich4_lpc.h"

void intel_ich4_link_acpi(ICH4State *lpc, Intel_ICH4_ACPI_State *acpi)
{
    lpc->acpi = acpi;
    qemu_printf("Intel ICH4 LPC: ACPI has been linked\n");
}

static void intel_ich4_acpi(int msb, int lsb, int en, Intel_ICH4_ACPI_State *acpi)
{
    acpi->io_base = ((msb << 8) | lsb) & 0xffc0;

    if(en)
        qemu_printf("Intel ICH4 LPC: ACPI base updated to 0x%04x\n", acpi->io_base);
    else
        qemu_printf("Intel ICH4 LPC: ACPI is disabled\n");

    memory_region_transaction_begin();
    memory_region_set_enabled(&acpi->io, !!en);
    memory_region_set_address(&acpi->io, acpi->io_base);
    memory_region_transaction_commit();
}

/*
static void intel_ich4_acpi_irq(int val, Intel_ICH4_ACPI_State *acpi)
{

}
*/

static void intel_ich4_gpio(int msb, int lsb, int en)
{
    int io_base = (msb << 8) | lsb;

    if(en)
        qemu_printf("Intel ICH4 LPC: GPIO base updated to 0x%04x\n", io_base);
    else
        qemu_printf("Intel ICH4 LPC: GPIO is disabled\n");
}

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
        
        default: /* Invalid IRQ */
            return 0;
    }
}

static PCIINTxRoute route_intx_pin_to_irq(void *opaque, int pin)
{
    ICH4State *ich4 = opaque;
    int irq = 0;
    
    if(!(ich4->dev.config[((pin > 3) ? 0x64 : 0x60) + pin] & 0x80))
        irq = intel_ich4_irq_table(ich4->dev.config[((pin > 3) ? 0x64 : 0x60) + pin] & 0x0f);
    else
        irq = 16 + pin;

    PCIINTxRoute route;

    route.mode = !!irq ? PCI_INTX_ENABLED : PCI_INTX_DISABLED;
    route.irq = !!irq ? irq : -1;

    return route;
}

static void intel_ich4_pirq(ICH4State *ich4)
{
    PCIBus *bus = pci_get_bus(&ich4->dev);
    PCIDevice *dev = &ich4->dev;

    qemu_printf("Intel ICH4 LPC: We provoked in PIRQ update!\n");

    for(int i = 0; i < 8; i++){
        int level = pci_bus_get_irq_level(bus, i); /* PIRQ Level */
        int pic_irq = intel_ich4_irq_table(dev->config[0x60 + ((i > 3) ? i + 4 : i)] & 0x0f); /* Pick up the IRQ from the table */
        int enabled = !(dev->config[0x60 + ((i > 3) ? i + 4 : i)] & 0x80) && (pic_irq != 0);
        uint64_t mask; /* Qemu magic */

        ich4->pic_levels = 0;

        if(!!enabled) {
//            qemu_printf("Intel ICH4 LPC: PIRQ%c has the IRQ of %d\n", 'A' + i, pic_irq & 0x0f);
            mask = 1ULL << ((pic_irq * 8ULL) + i);
            ich4->pic_levels &= ~mask;
            ich4->pic_levels |= mask * !!level;

            qemu_set_irq(ich4->pic[pic_irq], !!(ich4->pic_levels & (((1ULL << 8ULL) - 1) << (pic_irq * 8ULL))));
        }
        else {
//            qemu_printf("Intel ICH4 LPC: PIRQ%c is handled by APIC or disabled (APIC: %d)\n", 'A' + i, 16 + i);
            mask = 1ULL << (((16 + i) * 8ULL) + i);
            ich4->pic_levels &= ~mask;
            ich4->pic_levels |= mask * !!level;

            qemu_set_irq(ich4->pic[pic_irq], !!(ich4->pic_levels & (((1ULL << 8ULL) - 1) << ((16 + i) * 8ULL))));
        }
    }
}

static void intel_ich4_set_irq(void *opaque, int pirq, int level)
{
    ICH4State *ich4 = opaque;
    intel_ich4_pirq(ich4);
}

static void intel_ich4_nvr_reset(ICH4State *s)
{
    s->rtc.u128e = 0;
    s->rtc.l128lock = 0;
    s->rtc.u128lock = 0;

    qemu_printf("Intel ICH4 NVR: Upper Bank Configuration has been reset\n");
}

static void intel_ich4_nvr(int u128e, int l128lock, int u128lock, ICH4State *s)
{
    s->rtc.u128e = !!u128e;

    qemu_printf("Intel ICH4 NVR: Upper 128-byte range %s!\n", u128e ? "enabled" : "disabled");

    if(!s->rtc.l128lock) /* Per Intel ICH4 datasheet, once you lock the 38-3F range, you can't unlock unless you reset. */
        s->rtc.l128lock = !!l128lock;
    
    if(!s->rtc.u128lock)
        s->rtc.u128lock = !!u128lock;
}

static void intel_ich4_write_config(PCIDevice *dev, uint32_t address, uint32_t val, int len)
{
    ICH4State *ich4 = ICH4_PCI_DEVICE(dev);

    for(int i = 0; i < len; i++){
        int ro_only = 0;

        uint8_t new_val = (val >> (i * 8)) & 0xff;

        switch(address + i){

            case 0x40:
                new_val = new_val & 0x80;
            break;

            case 0x44:
                new_val = new_val & 0x17;
            break;

            case 0x4e:
                if(!!(dev->config[address + i] & 2))
                    ro_only = 1;
                else
                    new_val = new_val & 0x03;
            break;

            case 0x54:
                new_val = new_val & 0x0f;
            break;

            case 0x58:
                new_val = new_val & 0xc0;
            break;

            case 0x5c:
                new_val = new_val & 0x10;
            break;

            case 0x60:
            case 0x61:
            case 0x62:
            case 0x63:
            case 0x68:
            case 0x69:
            case 0x6a:
            case 0x6b:
                new_val = new_val & 0x8f;
            break;

            case 0x88:
                new_val = new_val & 0x06;
            break;

            case 0x8a:
                new_val &= 0x06;
            break;

            case 0x91:
                new_val = new_val & 0xfc;
            break;

            case 0xa0:
                new_val = new_val & 0x33;
            break;

            case 0xa2:
                new_val &= new_val & 0x7b;
            break;

            case 0xa4:
                new_val = new_val & 0xc8;
            break;

            case 0xa8:
                new_val = new_val & 0x3f;
            break;

            case 0xc0:
                new_val = new_val & 0xf0;
            break;

            case 0xd0:
                new_val = new_val & 0xc7;
            break;

            case 0xd1:
                new_val = new_val & 0x39;
            break;

            case 0xd2:
                new_val = new_val & 0x20;
            break;

            case 0xd3:
                new_val = new_val & 0x03;
            break;

            case 0xd4:
                new_val &= new_val & 0x02;
            break;

            case 0xd5:
                new_val &= new_val & 0x3f;
            break;

            case 0xd8:
                new_val &= new_val & 0x1c;
            break;

            case 0xe0:
                new_val &= new_val & 0x77;
            break;

            case 0xe1:
                new_val &= new_val & 0x13;
            break;

            case 0xe2:
                new_val &= new_val & 0x3b;
            break;

            case 0xe4:
                new_val &= new_val & 0x81;
            break;

            case 0xe7:
                new_val &= new_val & 0x3f;
            break;

            case 0xec:
                new_val &= new_val & 0xf1;
            break;

            case 0xf0:
                new_val &= new_val & 0x0f;
            break;

            case 0xf2:
                new_val &= new_val & 0x6b;
            break;

            case 0xf3:
                new_val &= new_val & 0xc7;
            break;

            case 0x41:
            case 0x59:
            case 0x64:
            case 0x90:
            case 0xb8:
            case 0xb9:
            case 0xba:
            case 0xbb:
            case 0xc4:
            case 0xc5:
            case 0xc6:
            case 0xc7:
            case 0xc8:
            case 0xc9:
            case 0xca:
            case 0xcb:
            case 0xcc:
            case 0xcd:
            case 0xe3:
            case 0xe5:
            case 0xe6:
            case 0xe8:
            case 0xe9:
            case 0xea:
            case 0xeb:
            case 0xed:
            case 0xee:
            case 0xef:
            break;

            default:
                ro_only = 1;
            break;
        }

        if(!ro_only) {
            pci_default_write_config(dev, address, new_val, len);
            dev->config[address + i] = new_val;
            qemu_printf("Intel ICH4 LPC: dev->regs[0x%02x] = %02x\n", address + i, new_val);
        }

    }

    switch(address){
        case 0x40:
        case 0x41:
        case 0x44:
            intel_ich4_acpi(dev->config[0x41], dev->config[0x40] & 0x80, dev->config[0x44] & 0x10, ich4->acpi);
        break;

        case 0x58:
        case 0x59:
        case 0x5c:
            intel_ich4_gpio(dev->config[0x59], dev->config[0x58] & 0xc0, dev->config[0x5c] & 0x10);
        break;

        case 0x60:
        case 0x61:
        case 0x62:
        case 0x63:
        case 0x68:
        case 0x69:
        case 0x6a:
        case 0x6b:
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
            intel_ich4_pirq(ich4);
        break;

        case 0xd8:
            intel_ich4_nvr(dev->config[0xd8] & 0x04, dev->config[0xd8] & 0x08, dev->config[0xd8] & 0x10, ich4);
        break;
    }

}

static void intel_ich4_lpc_reset(DeviceState *s)
{
    ICH4State *d = ICH4_PCI_DEVICE(s);
    PCIDevice *dev = PCI_DEVICE(d);

    dev->config[0x40] = 0x01;
    dev->config[0x58] = 0x01;
    dev->config[0x60] = 0x80;
    dev->config[0x61] = 0x80;
    dev->config[0x62] = 0x80;
    dev->config[0x63] = 0x80;
    dev->config[0x68] = 0x80;
    dev->config[0x69] = 0x80;
    dev->config[0x6a] = 0x80;
    dev->config[0x6b] = 0x80;
    dev->config[0xa8] = 0x0d;
    dev->config[0xe3] = 0xff;
    dev->config[0xe8] = 0x00;
    dev->config[0xe9] = 0x33;
    dev->config[0xea] = 0x22;
    dev->config[0xeb] = 0x11;
    dev->config[0xee] = 0x78;
    dev->config[0xef] = 0x56;

    d->pic_levels = 0;
    d->rcr = 0;

    intel_ich4_acpi(0, 0, 0, d->acpi);
    intel_ich4_gpio(0, 0, 0);
    intel_ich4_nvr_reset(d);
}

static int ich4_post_load(void *opaque, int version_id)
{
    ICH4State *ich4 = opaque;

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
    intel_ich4_pirq(ich4);
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
        VMSTATE_INT32_ARRAY_V(pci_irq_levels_vmstate, ICH4State, 8ULL, 3),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription*[]) {&vmstate_ich4_rcr, NULL}
};

static void rcr_write(void *opaque, hwaddr addr, uint64_t val, unsigned len)
{
    ICH4State *d = opaque;

    if (val & 4) {
        qemu_printf("PIIX Reset Control: We are resetting!\n");
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

static void intel_ich4_realize(PCIDevice *dev, Error **errp)
{
    ICH4State *d = ICH4_PCI_DEVICE(dev);
    ISABus *isa_bus;

    qemu_printf("Intel ICH4 LPC: I got realized!\n");

    /* Form the LPC Bus */
    qemu_printf("Intel ICH4 LPC: LPC bus ready\n");
    isa_bus = isa_bus_new(DEVICE(d), pci_address_space(dev), pci_address_space_io(dev), errp);
    if (!isa_bus) {
        qemu_printf("Intel ICH4 LPC: Failed to mount the LPC bus!\n");
        return;
    }

    /* PIIX Compatible Reset Port */
    memory_region_init_io(&d->rcr_mem, OBJECT(dev), &rcr_ops, d, "piix-compatible-reset-control", 1);
    memory_region_add_subregion_overlap(pci_address_space_io(dev), 0xcf9, &d->rcr_mem, 1);
    qemu_printf("Intel ICH4 LPC: PIIX Compatible reset control is mounted at port 0xcf9h\n");

    /* Register ISA IRQs */
    isa_bus_register_input_irqs(isa_bus, d->isa_irqs_in);

    /* Prepare the DMA controller */
    i8257_dma_init(isa_bus, 0);
    qemu_printf("Intel ICH4 LPC: DMA Controller is up\n");

    /* NVR */
    qdev_prop_set_int32(DEVICE(&d->rtc), "base_year", 2000);
    if (!qdev_realize(DEVICE(&d->rtc), BUS(isa_bus), errp))
        return;
    uint32_t irq = object_property_get_uint(OBJECT(&d->rtc), "irq", &error_fatal);
    isa_connect_gpio_out(ISA_DEVICE(&d->rtc), 0, irq);

    qemu_printf("Intel ICH4 LPC: NVR has been sanitized\n");
}

static void intel_ich4_init(Object *obj)
{
    ICH4State *d = ICH4_PCI_DEVICE(obj);

    /* NVR */
    qemu_printf("Intel ICH4 LPC: Mounting the NVR up\n");
    object_initialize_child(obj, "rtc", &d->rtc, TYPE_INTEL_ICH4_NVR);
}

static void intel_ich4_lpc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->vendor_id    = PCI_VENDOR_ID_INTEL;
    k->device_id    = PCI_DEVICE_ID_INTEL_ICH4_LPC;
    k->class_id     = PCI_CLASS_BRIDGE_ISA;
    k->config_write = intel_ich4_write_config;
    k->config_read = pci_default_read_config;
    k->revision = 0x02;
    dc->reset       = intel_ich4_lpc_reset;
    dc->desc        = "Intel ICH4 LPC Bridge";
    dc->vmsd        = &vmstate_ich4;
    dc->hotpluggable   = false;
    dc->user_creatable = false;
}

static const TypeInfo ich4_pci_type_info = {
    .name = TYPE_ICH4_PCI_DEVICE,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(ICH4State),
    .instance_init = intel_ich4_init,
    .abstract = true,
    .class_init = intel_ich4_lpc_class_init,
    .interfaces = (InterfaceInfo[]) {{ INTERFACE_CONVENTIONAL_PCI_DEVICE }, { }, },
};

static void intel_ich4_lpc_realize(PCIDevice *dev, Error **errp)
{
    ERRP_GUARD();
    ICH4State *ich4 = ICH4_PCI_DEVICE(dev);
    PCIBus *pci_bus = pci_get_bus(dev);

    intel_ich4_realize(dev, errp);
    if (*errp) return;

    pci_bus_irqs(pci_bus, intel_ich4_set_irq, ich4, 8ULL);
    pci_bus_set_route_irq_fn(pci_bus, route_intx_pin_to_irq);
}

static void intel_ich4_lpc_init(ObjectClass *klass, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = intel_ich4_lpc_realize;
}

static const TypeInfo ich4_info = {
    .name          = TYPE_ICH4_DEVICE,
    .parent        = TYPE_ICH4_PCI_DEVICE,
    .class_init    = intel_ich4_lpc_init,
};

static void intel_ich4_register_types(void)
{
    type_register_static(&ich4_pci_type_info);
    type_register_static(&ich4_info);
}

type_init(intel_ich4_register_types)
