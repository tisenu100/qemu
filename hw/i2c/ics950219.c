/*
 * ICS950219 Programmable Timing Hub for P4
 *
 * Copyright (c) 2023 Tiseno100
 *
 * smbus_eeprom.c portions
 * Copyright (c) 2007 Arastra, Inc.
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
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/smbus_slave.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/i2c/ics950219.h"
#include "qom/object.h"

#define TYPE_ICS950219 "ics950219"

OBJECT_DECLARE_SIMPLE_TYPE(ICS950219_State, ICS950219)

struct ICS950219_State {
    SMBusDevice smbusdev; /* The ICS device itself on the Bus. Say Hi! */

    /* Flags */
    uint8_t n; /* n bytes to read & write */
    uint8_t loc; /* location */
    uint8_t counter; /* counter */
    uint8_t read_counter; /* read counter */
    uint8_t state;
    uint8_t read_state;

    /* Registers */
    uint8_t regs[24]; /* 24 Bytes */
};

static uint8_t ics950219_read(SMBusDevice *dev)
{
    ICS950219_State *d = ICS950219(dev);
    uint8_t val = 0xff;
    uint8_t addr;
    switch(d->read_state) {
        case 0:
            d->state = 0;
            val = d->regs[0x08];
            qemu_printf("ICS: Data Byte Count was recieved\n");
            d->read_state++;
        break;

        case 1:
            addr = d->loc + d->read_counter - 1;
            if(addr > 0x17)
                val = 0xff;
            else
                val = d->regs[addr];

            d->read_counter++;

            if(d->read_counter == d->regs[0x08]){
                d->read_counter = 0;
                d->read_state = 0;
                qemu_printf("ICS: Transmission finished!\n");
            }
    }

    return val;
}

static int ics950219_write(SMBusDevice *dev, uint8_t *buf, uint8_t len)
{
    ICS950219_State *d = ICS950219(dev);
    for(int i = 0; i < len; i++) {
        switch(d->state) {
            case 0: /* Beggining Byte Location N. Where the RW will start at */
                d->loc = buf[i];
                qemu_printf("ICS: Base Register Address set 0x%02x\n", d->loc);

                d->state++;
            break;

            case 1: /* Byte Size R/W N. How many Bits will by RW'd */
                d->n = buf[i];
                d->counter = 0; /* Prepare the counter */
                qemu_printf("ICS: Size %d bytes\n", d->n);

                if(d->n == 0) /* Sanity check in case 0 bytes are given */
                    d->state = 0;
                else
                    d->state++;
            break;

            case 2: /* Write procedure */
                uint8_t addr = d->loc + d->counter; /* Location Given + Counted */

                if(addr > 0x17) /* In case a board uses a incompatible ICS chip it may write the unknown registers */
                    qemu_printf("ICS: Buffer Overflow. Address 0x%02x is invalid. Are you using another clock chip?\n", addr);
                else {
                    d->regs[addr] = buf[i]; /* Normally there are reserved bits & ro bits. We speculate BIOS awareness on that */
                }

                d->n--;
                d->counter++;

                if(d->n == 0) {
                    d->state = 0;
                    qemu_printf("ICS: Transmission finished!\n");
                }
            break;
        }
    }

    return 0;
}

static const VMStateDescription vmstate_ics950219 = {
    .name = "ics950219",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = 0,
    .fields      = (VMStateField[]) {
        VMSTATE_SMBUS_DEVICE(smbusdev, ICS950219_State),
        VMSTATE_UINT8_ARRAY(regs, ICS950219_State, 24),
        VMSTATE_UINT8(n, ICS950219_State),
        VMSTATE_UINT8(loc, ICS950219_State),
        VMSTATE_UINT8(counter, ICS950219_State),
        VMSTATE_UINT8(state, ICS950219_State),
        VMSTATE_UINT8(read_state, ICS950219_State),
        VMSTATE_END_OF_LIST()
    }
};

static void ics950219_reset(DeviceState *dev)
{
    ICS950219_State *d = ICS950219(dev);
    d->state = 0;
    d->n = 0;
    d->loc = 0;
    d->counter = 0;
    d->read_counter = 0;
    d->read_state = 0;

    memset(d->regs, 0, sizeof(d->regs));
    d->regs[0x01] = 0xe0;
    d->regs[0x02] = 0xff;
    d->regs[0x03] = 0xe7;
    d->regs[0x04] = 0x3f;
    d->regs[0x06] = 0x01;
    d->regs[0x07] = 0x28; /* Revision ID */
    d->regs[0x08] = 0x0f; /* How many bytes will be read back */
    d->regs[0x09] = 0x08;
    d->regs[0x0a] = 0x08;
    d->regs[0x12] = 0x44;
    d->regs[0x13] = 0x44;
    d->regs[0x14] = 0x88;
    d->regs[0x15] = 0xaa;
    d->regs[0x16] = 0xaa;
    d->regs[0x17] = 0x2a;
}

static void ics950219_realize(DeviceState *dev, Error **errp)
{
    /* Just Reset */
    ics950219_reset(dev);
}

static void ics950219_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SMBusDeviceClass *sc = SMBUS_DEVICE_CLASS(klass);

    dc->realize = ics950219_realize;
    dc->reset = ics950219_reset;
    sc->receive_byte = ics950219_read;
    sc->write_data = ics950219_write;
    dc->vmsd = &vmstate_ics950219;
    dc->user_creatable = false;
}

static const TypeInfo ics950219_info = {
    .name          = TYPE_ICS950219,
    .parent        = TYPE_SMBUS_DEVICE,
    .instance_size = sizeof(ICS950219_State),
    .class_init    = ics950219_class_initfn,
};

static void ics950219_register_types(void)
{
    type_register_static(&ics950219_info);
}

type_init(ics950219_register_types)

void ics950219_init(I2CBus *smbus)
{
    DeviceState *dev;

    dev = qdev_new(TYPE_ICS950219);
    qdev_prop_set_uint8(dev, "address", 0x69); /* Not a valid way to put it. Just most boards mounted it at address 0x69 */
    qdev_realize_and_unref(dev, (BusState *)smbus, &error_fatal);
}