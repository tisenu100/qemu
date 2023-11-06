/*
 * Winbond W83627HF LPC Super I/O
 *
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

#include "qemu/osdep.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "trace.h"

#include "qemu/qemu-print.h"
#include "hw/isa/superio.h"
#include "hw/block/fdc.h"
#include "hw/block/fdc-internal.h"
#include "hw/isa/winbond_w83627hf.h"

static void winbond_reset_actual(WinbondState *d); /* Allocate here so we can reset per CR00 Bit 1 request */

static void winbond_ldn_update(int val, WinbondState *d)
{
    /*
        According to the Winbond W83627HF datasheet. The programmable devices are:

        Logical Device:
        0 FDC
        1 LPT
        2 UART A
        3 UART B
        5 AMIKEY Compatible KBC
        6 CIR
        7 Gameport
        8 Watch Dog Timer
        9 VSB
        A ACPI
        B Hardware Monitor
    */
    d->ldn = val;

    if(d->ldn > 0) 
        qemu_printf("Winbond W83627HF: LDN was updated to %d\n", d->ldn);
}

void winbond_link_fdc(WinbondState *sio, FDCtrl fdc)
{
    sio->fdc = fdc;
    qemu_printf("Winbond W83627HF: FDC has been linked!\n");
}

static void winbond_remap_fdc(WinbondState *s)
{
    s->fdc_io_base = (s->ldn_regs[0][0x60] << 8) | s->ldn_regs[0][0x61];
    FDCtrl fdc = s->fdc;

    memory_region_transaction_begin();
    memory_region_set_enabled(&fdc.iomem, s->ldn_regs[0][0x30] & 1);
    memory_region_set_address(&fdc.iomem, s->fdc_io_base);
    memory_region_transaction_commit();

    if(!(s->ldn_regs[0][0x30] & 1))
        qemu_printf("Winbond W83627HF: Floppy Disk Controller has been disabled!\n");
    else
        qemu_printf("Winbond W83627HF: Floppy Disk Controller address has been updated to 0x%04x\n", s->fdc_io_base);
}

static void winbond_remap_rw(WinbondState *s)
{
//    FDCtrl *fdc = s->fdc;

//    fdc->ro = !!(d->ldn_regs[0][0xf1] & 0x02);

    if(s->ldn_regs[0][0xf2] & 2)
        qemu_printf("Winbond W83627HF: Floppy Disk Controller is enforced read only!\n");
    else
        qemu_printf("Winbond W83627HF: Floppy Disk Controller is writable\n");

}

static void winbond_fdc_dma(WinbondState *s)
{
//    FDCtrl fdc = s->fdc;

    if(s->ldn_regs[0][0x74] < 4) {
//        fdc.dma_chann = s->ldn_regs[0][0x74];
        qemu_printf("Winbond W83627HF: Floppy Disk Controller DMA has been updated to %d\n", s->ldn_regs[0][0x74]);
    }
    else {
//        fdc.dma_chann = -1;
        qemu_printf("Winbond W83627HF: Floppy Disk Controller DMA has been disabled or invalidated\n");
    }
}

static void winbond_writeb(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    WinbondState *d = opaque;

    if(!(addr & 1)) { /* Index Register */
        d->index = val;

        if(val == 0x87) {
            d->extended_function_mode = 1;
            qemu_printf("Winbond W83627HF: Extended Function Mode was enabled\n");
        }
        else if(val == 0xaa){
            d->extended_function_mode = 0;
            qemu_printf("Winbond W83627HF: Extended Function Mode was disabled\n");
        }
    } else { /* Data */
        uint8_t new_val = val;

        /* Regular Registers */
        switch(d->index) {
            case 0x00:
                d->regs[d->index] = new_val & 0x01;

                if(new_val & 1)
                    winbond_reset_actual(d);
            break;

            case 0x07:
                d->regs[d->index] = new_val;
                d->ldn = new_val;
                winbond_ldn_update(new_val, d);
            break;

            case 0x23:
                d->regs[d->index] = new_val & 0x01;
            break;

            case 0x24:
                d->regs[d->index] = new_val & 0xc5;
            break;

            case 0x25:
                d->regs[d->index] = new_val & 0x39;
            break;

            case 0x26:
                d->regs[d->index] = new_val & 0xef;
            break;

            case 0x28:
                d->regs[d->index] = new_val & 0x07;
            break;

            case 0x29:
                d->regs[d->index] = new_val & 0xfc;
            break;

            case 0x22:
            case 0x2a:
            case 0x2b:
            case 0x2e: /* 2Eh & 2Fh are just test registers */
            case 0x2f:
                d->regs[d->index] = new_val;
            break;
        }

        if(d->index >= 0x30){ /* Program the logical devices */
            switch(d->ldn){
                case 0: /* FDC */
                    switch(d->index){
                        case 0x30:
                            d->ldn_regs[d->ldn][d->index] = new_val & 0x01;
                            winbond_remap_fdc(d);
                        break;

                        case 0x60:
                        case 0x61:
                            d->ldn_regs[d->ldn][d->index] = new_val;
                            winbond_remap_fdc(d);
                        break;

                        case 0x70:
                            d->ldn_regs[d->ldn][d->index] = new_val & 0x0f;
                        break;

                        case 0x74:
                            d->ldn_regs[d->ldn][d->index] = new_val & 0x07;
                            winbond_fdc_dma(d);
                        break;

                        case 0xf4:
                            d->ldn_regs[d->ldn][d->index] = new_val & 0x5b;
                        break;

                        case 0xf0:
                        case 0xf1:
                            d->ldn_regs[d->ldn][d->index] = new_val;
                            winbond_remap_rw(d);
                        break;

                        case 0xf2:
                        case 0xf5:
                            d->ldn_regs[d->ldn][d->index] = new_val;
                        break;
                    }
                break;
            }
        }
    }
}

static uint64_t winbond_readb(void *opaque, hwaddr addr, unsigned size)
{
    WinbondState *d = opaque;

    if(!(addr & 1)) /* If we are on Index Register we return the Index value */
        return d->index;
    else if(d->index < 0x30) /* We are not on Index, we return the standard Winbond bits if lower than 0x30 */
        return d->regs[d->index];
    else if(d->ldn < 12) /* We are above standard bits, we return the Logical Device bits */
        return d->ldn_regs[d->ldn][d->index]; 
    else { /* We are none of these, invalidate and return 0xff */
        qemu_printf("Winbond W83627HF: Invalid logical device reading %d\n", d->ldn);
        return 0xffffffffffffffffULL;
    }
}

static const MemoryRegionOps winbond_ops = {
    .read = winbond_readb,
    .write = winbond_writeb,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

const VMStateDescription vmstate_winbond = {
    .name = "Winbond W83627HF",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(ldn, WinbondState),
        VMSTATE_BOOL(extended_function_mode, WinbondState),
        VMSTATE_UINT8_ARRAY(regs, WinbondState, 0x30),
        VMSTATE_UINT8_2DARRAY(ldn_regs, WinbondState, 12, 0xff),
        VMSTATE_UINT16(fdc_io_base, WinbondState),
        VMSTATE_END_OF_LIST()
    }
};

static void winbond_reset_actual(WinbondState *d)
{
    qemu_printf("Winbond W83627HF: We are resetting!\n");

    /* Clear up all the registers */
    d->extended_function_mode = 0;
    d->ldn = 0;
    memset(d->regs, 0x00, sizeof(d->regs));
    memset(d->ldn_regs, 0x00, sizeof(d->ldn_regs));

    d->regs[0x20] = 0x52; /* Device ID */
    d->regs[0x21] = 0x41; /* Revision ID. Whatever ASUS P4PE2-X reports us */
    d->regs[0x22] = 0xff;
    d->regs[0x24] = 0x80;
    d->regs[0x2a] = 0x7c;
    d->regs[0x2b] = 0xc0;

    /* Floppy */
    d->ldn_regs[0][0x30] = 0x01;
    d->ldn_regs[0][0x60] = 0x30;
    d->ldn_regs[0][0x61] = 0xf0;
    d->ldn_regs[0][0x70] = 0x06;
    d->ldn_regs[0][0x74] = 0x04;
    d->ldn_regs[0][0xf0] = 0x0e;
    d->ldn_regs[0][0xf2] = 0xff;

    winbond_remap_fdc(d);
    winbond_remap_rw(d);
    winbond_fdc_dma(d);
}

static void winbond_reset(DeviceState *d)
{
    WinbondState *s = WINBOND_W83627HF(d);

    /* For compatibility reasons we have a dynamic reset function */
    winbond_reset_actual(s);
}

static void winbond_realize(DeviceState *dev, Error **errp)
{
    WinbondState *s = WINBOND_W83627HF(dev);
    qemu_printf("Winbond W83627HF: I got realized!\n");

    /* Winbond W83627HF also supports to be used in 4E-4Fh sections. The Intel ICH4 datasheet covers this too! */
    isa_register_ioport(ISA_DEVICE(dev), &s->io, 0x2e);

    winbond_reset_actual(s);
}

static void winbond_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = winbond_realize;
    dc->reset = winbond_reset;
    dc->user_creatable = false;
    dc->vmsd = &vmstate_winbond;
}

static void winbond_initfn(Object *obj)
{
    WinbondState *s = WINBOND_W83627HF(obj);
    memory_region_init_io(&s->io, obj, &winbond_ops, s, "winbond-w83627hf", 2);
}

static const TypeInfo winbond_type_info = {
    .name          = TYPE_WINBOND_W83627HF,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(WinbondState),
    .instance_init = winbond_initfn,
    .class_init    = winbond_class_init,
};

static void winbond_register_types(void)
{
    type_register_static(&winbond_type_info);
}

type_init(winbond_register_types)
