/*
 * Intel ICH5 LPC
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

#include "hw/acpi/intel_ich5_acpi.h"
#include "hw/southbridge/intel_ich5_lpc.h"

static void intel_ich5_acpi_sci_update(void *opaque, int irq_num, int level)
{
    Intel_ICH5_LPC_State *s = opaque;
    qemu_set_irq(s->lpc_irqs_in[s->acpi->sci_irq], level);
}

void intel_ich5_link_acpi(Intel_ICH5_LPC_State *lpc, Intel_ICH5_ACPI_State *acpi)
{
    lpc->acpi = acpi;

    /* Prepare the SCI IRQ when ready */
    lpc->acpi->irq = qemu_allocate_irq(intel_ich5_acpi_sci_update, lpc, 0);

    qemu_printf("Intel ICH5 LPC: ACPI has been linked\n");
}

static void intel_ich5_acpi(int msb, int lsb, int en, Intel_ICH5_ACPI_State *acpi)
{
    acpi->io_base = ((msb << 8) | lsb) & 0xffc0;

    if(!!en)
        qemu_printf("Intel ICH5 LPC: ACPI base updated to 0x%04x\n", acpi->io_base);
    else
        qemu_printf("Intel ICH5 LPC: ACPI is disabled\n");

    memory_region_transaction_begin();
    memory_region_set_enabled(&acpi->io, !!en);
    memory_region_set_address(&acpi->io, acpi->io_base);
    memory_region_transaction_commit();
}

static int intel_ich5_sci_table(Intel_ICH5_LPC_State *lpc, int num)
{
    PCIDevice *dev = PCI_DEVICE(lpc);
   
    switch(num)
    {
        case 0x01:
        case 0x02:
            return 10 + (num - 1);

        case 0x04:
        case 0x05:
        case 0x06:
        case 0x07:
            if(dev->config[0xd1] & 1)
                return 20 + (num - 4);
            else
                return 9;

        default: /* Default but also fallback */
            return 9;
    }
}

static void intel_ich5_acpi_irq(int val, Intel_ICH5_LPC_State *lpc)
{
    Intel_ICH5_ACPI_State *acpi = lpc->acpi;
    PCIDevice *dev = PCI_DEVICE(lpc);

    qemu_set_irq(acpi->irq, 0); /* Dispatch the SCI IRQ so we can update it */
    acpi->sci_irq = intel_ich5_sci_table(lpc, dev->config[0x44] & 7);
    qemu_set_irq(acpi->irq, 1); /* Now update it */

    qemu_printf("Intel ICH5 LPC: SCI IRQ was updated to IRQ: %d\n", acpi->sci_irq);
}

static void intel_ich5_gpio(int msb, int lsb, int en)
{
    int io_base = (msb << 8) | lsb;

    if(en)
        qemu_printf("Intel ICH5 LPC: GPIO base updated to 0x%04x\n", io_base);
    else
        qemu_printf("Intel ICH5 LPC: GPIO is disabled\n");
}

static int intel_ich5_irq_table(int irq)
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
    Intel_ICH5_LPC_State *d = opaque;
    int irq = intel_ich5_irq_table(d->dev.config[((pin > 3) ? 0x64 : 0x60) + pin] & 0x0f);
    
    if((d->dev.config[((pin > 3) ? 0x64 : 0x60) + pin] & 0x80) || (irq == 0))
        irq = 16 + pin;

    PCIINTxRoute route;

    route.mode = PCI_INTX_ENABLED;
    route.irq = irq;

    return route;
}

static void intel_ich5_pirq(int pirq, Intel_ICH5_LPC_State *d) /* The PIRQ Router */
{
    PCIDevice *dev = &d->dev;

        int pic_irq = intel_ich5_irq_table(dev->config[0x60 + ((pirq > 3) ? pirq + 4 : pirq)] & 0x0f); /* Pick up the IRQ from the table */
        int enabled = !(dev->config[0x60 + ((pirq > 3) ? pirq + 4 : pirq)] & 0x80) && (pic_irq != 0); /* Check if the the PIRQ is asserted & if the IRQ value is valid on the table */

        d->pic_level = 0;

        if(!enabled)
            pic_irq = 16 + pirq;

        qemu_printf("Intel ICH5 LPC: PIRQ %c raised on IRQ %d\n", 'A' + pirq, pic_irq);

        d->pic_level |= pci_bus_get_irq_level(pci_get_bus(dev), pirq);
        qemu_set_irq(d->lpc_irqs_in[pic_irq], d->pic_level);
}

static void intel_ich5_lpc_raise_pirq(void *opaque, int pirq, int level)
{
    Intel_ICH5_LPC_State *d = opaque;
    intel_ich5_pirq(pirq, d);
}

static void intel_ich5_nvr_reset(Intel_ICH5_LPC_State *s)
{
    s->rtc.u128e = 0;
    s->rtc.l128lock = 0;
    s->rtc.u128lock = 0;
}

static void intel_ich5_nvr(int u128e, int l128lock, int u128lock, Intel_ICH5_LPC_State *s)
{
    s->rtc.u128e = !!u128e;

    qemu_printf("Intel ICH5 NVR: Upper 128-byte range %s!\n", u128e ? "enabled" : "disabled");

    if(!s->rtc.l128lock) /* Per Intel ICH5 datasheet, once you lock the 38-3F range, you can't unlock unless you reset. */
        s->rtc.l128lock = !!l128lock;
    
    if(!s->rtc.u128lock)
        s->rtc.u128lock = !!u128lock;
}

static void intel_ich5_write_config(PCIDevice *dev, uint32_t address, uint32_t val, int len)
{
    Intel_ICH5_LPC_State *d = INTEL_ICH5_LPC(dev);
    pci_default_write_config(dev, address, val, len);

    for(int i = 0; i < len; i++){
        int ro_only = 0;
        int pme_reg = 0;
        uint8_t new_val = (val >> (i * 8)) & 0xff;

        switch(address + i){
            case 0x04:
                new_val = new_val & 0x40;
            break;

            case 0x05:
                new_val = new_val & 0x01;
            break;

            case 0x07:
                new_val &= ~(new_val & 0xf9);
                new_val |= 0x02;
            break;

            case 0x40:
                new_val = (new_val & 0x80) | 1;
                pme_reg = 1;
            break;

            case 0x41:
                pme_reg = 1;
            break;

            case 0x44:
                new_val = new_val & 0x17;
                pme_reg = 1;
            break;

            case 0x4e:
                new_val = new_val & 0x03;
            break;

            case 0x54:
                new_val = new_val & 0x0f;
            break;

            case 0x58:
                new_val = (new_val & 0x80) | 1;
            break;

            case 0x5c:
                new_val = new_val & 0x10;
            break;

            case 0x60:
            case 0x61:
            case 0x62:
            case 0x63:
                new_val = new_val & 0x8f;
            break;

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
                new_val &= ~(new_val & 0x06);
            break;

            case 0x91:
                new_val = new_val & 0xfc;
            break;

            case 0xa0:
                new_val = new_val & 0x3f;
                if(dev->config[0xa0] & 0x10) /* SMI_LOCK */
                    new_val |= 0x10;

                pme_reg = 1;
            break;

            case 0xa2:
                new_val &= ~(new_val & 0x9f); /* DRAM Initialization (Bit 7) is not allowed to be set cause the BIOS may think the DRAM initialization was interrupted. */
                pme_reg = 1;
            break;

            case 0xa4:
                new_val = new_val & 0xfc;
                new_val &= ~(new_val & 3);
                pme_reg = 1;
            break;

            case 0xa8:
                new_val = new_val & 0x3f;
                pme_reg = 1;
            break;

            case 0xad:
                new_val = new_val & 0x03;
                pme_reg = 1;
            break;

            case 0xb8:
            case 0xb9:
            case 0xba:
            case 0xbb:
                pme_reg = 1;
            break;

            case 0xc0:
                new_val = new_val & 0xf0;
                pme_reg = 1;
            break;

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
                pme_reg = 1;
            break;

            case 0xd0:
                new_val = new_val & 0xc7;
            break;

            case 0xd1:
                new_val = new_val & 0xbb;
                if(dev->config[0xd1] & 0x02)
                    new_val |= 0x02;
            break;

            case 0xd2:
                new_val = new_val & 0x0f;
            break;

            case 0xd3:
                new_val = new_val & 0x03;
            break;

            case 0xd4:
                new_val = new_val & 0x02;
            break;

            case 0xd5:
                new_val = new_val & 0x2f;
            break;

            case 0xd8:
                new_val = new_val & 0x1c;

                if(dev->config[0xd8] & 0x10) /* Unless a reset is held, the NVR lock bits if written, cannot be changed */
                    new_val |= 0x10;
                if(dev->config[0xd8] & 0x08)
                    new_val |= 0x08;
            break;

            case 0xe0:
                new_val = new_val & 0x77;
            break;

            case 0xe1:
                new_val = new_val & 0x13;
            break;

            case 0xe4:
                new_val = new_val & 0x01;
            break;

            case 0xe6:
                new_val = new_val & 0x0f;
            break;

            case 0xe7:
                new_val = new_val & 0x2f;
            break;

            case 0xec:
                new_val = new_val & 0xf1;
            break;

            case 0xf0:
                new_val = new_val & 0x0f;
            break;

            case 0xf2:
                new_val = new_val & 0x6f;
            break;

            case 0xf3:
                new_val = new_val & 0xcf;
            break;

            case 0x59:
            case 0x64:
            case 0x90:
            case 0xe3:
            case 0xe5:
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
            dev->config[address + i] = new_val;

            if(pme_reg)
                d->pme_conf[address + i] = new_val;

            qemu_printf("Intel ICH5 LPC: dev->regs[0x%02x] = %02x\n", address + i, new_val);
        }

    }

    switch(address){
        case 0x40:
        case 0x41:
        case 0x44:
            intel_ich5_acpi(dev->config[0x41], dev->config[0x40] & 0x80, dev->config[0x44] & 0x10, d->acpi);
            intel_ich5_acpi_irq(dev->config[0x44], d);
        break;

        case 0x58:
        case 0x59:
        case 0x5c:
            intel_ich5_gpio(dev->config[0x59], dev->config[0x58] & 0xc0, dev->config[0x5c] & 0x10);
        break;

        case 0x60:
        case 0x61:
        case 0x62:
        case 0x63:
        case 0x68:
        case 0x69:
        case 0x6a:
        case 0x6b:
            /*
               Per Intel ICH5 datasheet. The PIRQs are categorized as this:
               0x60 PIRQA#
               0x61 PIRQB#
               0x62 PIRQC#
               0x63 PIRQD#
               0x68 PIRQE#
               0x69 PIRQF#
               0x6a PIRQG#
               0x6b PIRQH#
            */
            pci_bus_fire_intx_routing_notifier(pci_get_bus(&d->dev));
        break;

        case 0xd8:
            intel_ich5_nvr(dev->config[0xd8] & 0x04, dev->config[0xd8] & 0x08, dev->config[0xd8] & 0x10, d);
        break;
    }

}

/*
static uint32_t intel_ich5_read_config(PCIDevice *dev, uint32_t address, int len)
{
    qemu_printf("Intel ICH5 LPC: dev->regs[0x%02x] (%x)\n", (int)address, (int)dev->config[address]);
    return pci_default_read_config(dev, address, len);
}
*/

static void intel_ich5_lpc_reset(DeviceState *s)
{
    Intel_ICH5_LPC_State *d = INTEL_ICH5_LPC(s);
    PCIDevice *dev = PCI_DEVICE(d);

    for(int i = 0x40; i < 0xf3; i++)
        dev->config[i] = 0x00;

    dev->config[0x04] = 0x0f;
    dev->config[0x06] = 0x80;
    dev->config[0x07] = 0x02;
    dev->config[0x58] = 0x01;
    dev->config[0x60] = 0x80;
    dev->config[0x61] = 0x80;
    dev->config[0x62] = 0x80;
    dev->config[0x63] = 0x80;
    dev->config[0x64] = 0x10;
    dev->config[0x68] = 0x80;
    dev->config[0x69] = 0x80;
    dev->config[0x6a] = 0x80;
    dev->config[0x6b] = 0x80;
    dev->config[0xa8] = 0x0d;
    dev->config[0xd0] = 0x04;
    dev->config[0xe3] = 0xff;
    dev->config[0xe8] = 0x00;
    dev->config[0xe9] = 0x33;
    dev->config[0xea] = 0x22;
    dev->config[0xeb] = 0x11;
    dev->config[0xee] = 0x78;
    dev->config[0xef] = 0x56;
    dev->config[0xf0] = 0x02;

    dev->config[0x40] = d->pme_conf[0x40];
    dev->config[0x41] = d->pme_conf[0x41];
    dev->config[0x44] = d->pme_conf[0x44];

    /* PME registers remain as they were even after reset */
    for(int i = 0; i < 4; i++)
        dev->config[0xb8 + i] = d->pme_conf[0xb8 + i];

    for(int i = 0; i < 16; i++){
        dev->config[0xa0 + i] = d->pme_conf[0xa0 + i];
        dev->config[0xc0 + i] = d->pme_conf[0xc0 + i];
    }

    d->rcr = 0;

    intel_ich5_acpi(dev->config[0x41], dev->config[0x40] & 0x80, dev->config[0x44] & 0x10, d->acpi);
    intel_ich5_acpi_irq(dev->config[0x44], d);
    intel_ich5_gpio(0, 0, 0);
    intel_ich5_nvr_reset(d);
}

static int intel_ich5_lpc_pre_save(void *opaque)
{
    Intel_ICH5_LPC_State *dev = opaque;

    for (int i = 0; i < ARRAY_SIZE(dev->pci_irq_levels_vmstate); i++) {
        dev->pci_irq_levels_vmstate[i] = pci_bus_get_irq_level(pci_get_bus(&dev->dev), i);
    }

    return 0;
}

static bool piix_rcr_needed(void *opaque)
{
    Intel_ICH5_LPC_State *d = opaque;

    return (d->rcr != 0);
}

static const VMStateDescription vmstate_piix_rcr = {
    .name = "PIIX Compatible Reset Control",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = piix_rcr_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(rcr, Intel_ICH5_LPC_State),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_intel_ich5_lpc = {
    .name = "Intel ICH5 LPC",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = intel_ich5_lpc_pre_save,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, Intel_ICH5_LPC_State),
        VMSTATE_INT32_ARRAY_V(pci_irq_levels_vmstate, Intel_ICH5_LPC_State, 8ULL, 3),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription*[]) {&vmstate_piix_rcr, NULL}
};

static void rcr_write(void *opaque, hwaddr addr, uint64_t val, unsigned len)
{
    Intel_ICH5_LPC_State *d = opaque;

    if (val & 4) {
        qemu_printf("Reset Control: We are resetting!\n");

        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);

        return;
    }
    d->rcr = val & 2; /* Keep System Reset type only */
}

static uint64_t rcr_read(void *opaque, hwaddr addr, unsigned len)
{
    Intel_ICH5_LPC_State *d = opaque;

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

static void intel_ich5_realize(PCIDevice *dev, Error **errp)
{
    Intel_ICH5_LPC_State *d = INTEL_ICH5_LPC(dev);
    ISABus *isa_bus;

    /* Form the LPC Bus */
    isa_bus = isa_bus_new(DEVICE(d), pci_address_space(dev), pci_address_space_io(dev), errp);
    if (!isa_bus) {
        qemu_printf("Intel ICH5 LPC: Failed to mount the LPC bus!\n");
        return;
    }

    /* PIIX Compatible Reset Port */
    memory_region_init_io(&d->rcr_mem, OBJECT(dev), &rcr_ops, d, "piix-compatible-reset-control", 1);
    memory_region_add_subregion_overlap(pci_address_space_io(dev), 0xcf9, &d->rcr_mem, 1);

    /* Connect the IRQs */
    isa_bus_register_input_irqs(isa_bus, d->lpc_irqs_in);

    /* Prepare the DMA controller */
    i8257_dma_init(isa_bus, 0);

    /* NVR */
    qdev_prop_set_int32(DEVICE(&d->rtc), "base_year", 2000);
    if (!qdev_realize(DEVICE(&d->rtc), BUS(isa_bus), errp))
        return;
    uint32_t irq = object_property_get_uint(OBJECT(&d->rtc), "irq", &error_fatal);
    isa_connect_gpio_out(ISA_DEVICE(&d->rtc), 0, irq);

    /* PCI PME Registers */
        memset(d->pme_conf, 0, sizeof(d->pme_conf));
        dev->config[0x40] = 0x01;
        dev->config[0xa8] = 0x0d;
}

static void intel_ich5_lpc_init(Object *obj)
{
    Intel_ICH5_LPC_State *d = INTEL_ICH5_LPC(obj);

    /* Register ISA IRQs */
    qdev_init_gpio_out_named(DEVICE(obj), d->lpc_irqs_in, "lpc-irqs", 24);

    /* NVR */
    qemu_printf("Intel ICH5 LPC: Mounting the NVR up\n");
    object_initialize_child(obj, "rtc", &d->rtc, TYPE_INTEL_ICH5_NVR);
}

static void intel_ich5_lpc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->vendor_id    = PCI_VENDOR_ID_INTEL;
    k->device_id    = PCI_DEVICE_ID_INTEL_ICH5_LPC;
    k->class_id     = PCI_CLASS_BRIDGE_ISA;
    k->config_write = intel_ich5_write_config;
    k->config_read = pci_default_read_config;
    k->revision = 0x02;
    dc->reset       = intel_ich5_lpc_reset;
    dc->desc        = "Intel ICH5 LPC Bridge";
    dc->vmsd        = &vmstate_intel_ich5_lpc;
    dc->hotpluggable   = false;
    dc->user_creatable = false;
}

static const TypeInfo intel_ich5_lpc_type_info = {
    .name = TYPE_INTEL_ICH5_LPC,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Intel_ICH5_LPC_State),
    .instance_init = intel_ich5_lpc_init,
    .abstract = true,
    .class_init = intel_ich5_lpc_class_init,
    .interfaces = (InterfaceInfo[]) {{ INTERFACE_CONVENTIONAL_PCI_DEVICE }, { }, },
};

static void intel_ich5_lpc_realize(PCIDevice *dev, Error **errp)
{
    ERRP_GUARD();
    Intel_ICH5_LPC_State *d = INTEL_ICH5_LPC(dev);
    PCIBus *pci_bus = pci_get_bus(dev);

    intel_ich5_realize(dev, errp);
    if (*errp) return;

    pci_bus_irqs(pci_bus, intel_ich5_lpc_raise_pirq, d, 8);
    pci_bus_set_route_irq_fn(pci_bus, route_intx_pin_to_irq);
}

static void intel_ich5_lpc_base_init(ObjectClass *klass, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = intel_ich5_lpc_realize;
}

static const TypeInfo intel_ich5_lpc_info = {
    .name          = TYPE_INTEL_ICH5,
    .parent        = TYPE_INTEL_ICH5_LPC,
    .class_init    = intel_ich5_lpc_base_init,
};

static void intel_ich5_register_types(void)
{
    type_register_static(&intel_ich5_lpc_type_info);
    type_register_static(&intel_ich5_lpc_info);
}

type_init(intel_ich5_register_types)
