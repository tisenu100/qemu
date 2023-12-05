/*
 * Winbond Clock Generator For Intel P4
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
#include "hw/i2c/winbond_w83194.h"
#include "qom/object.h"

#define TYPE_WINBOND_W83194 "winbond_w83194"

OBJECT_DECLARE_SIMPLE_TYPE(WINBOND_W83194_State, WINBOND_W83194)

struct WINBOND_W83194_State {
    SMBusDevice smbusdev; /* The ICS device itself on the Bus. Say Hi! */

    /* Flags */
    uint8_t state; /* Status */
    uint8_t command_code; /* Command Code */
    uint8_t n; /* Byte Count */
    uint8_t block_state; /* Counter for Block R/W */
    uint8_t reg_no; /* Register Number for Byte R/W */

    /* Registers */
    uint8_t regs[22]; /* 24 Bytes */
};

static uint8_t winbond_w83194_read(SMBusDevice *dev)
{
    WINBOND_W83194_State *d = WINBOND_W83194(dev);
    uint8_t val = 0xff;

    switch(d->state)
    {
        case 1:
            if(d->command_code == 0x00) {
                d->state = 4;
                d->block_state = 0;
                qemu_printf("Winbond W83194: %d bytes will be written\n", d->n);
                val = d->n; /* First Return the N Blocks that'll be read */
            }
            else {
                val = d->regs[d->reg_no]; /* Don't update Register Number. BIOS may want to read the register first then write to it. */
                qemu_printf("Winbond W83194: Byte read complete!\n");
                d->state = 0;
            }
        break;

        case 4:
            if(d->block_state < 22) /* Prevent Overflow */
                    val = d->regs[d->block_state];
            else
                    val = 0xff;

            qemu_printf("Winbond W83194: Writing 0x%02x to Register %d\n", val, d->block_state);

            d->block_state++;

            if(d->block_state == d->n) {
                qemu_printf("Winbond W83194: Block read complete!\n");
                d->state = 0;
            }
        break;

        default:
            qemu_printf("Winbond W83194: Invalid Read State\n");
            d->state = 0;
        break; 
    }

    return val;
}

static int winbond_w83194_write(SMBusDevice *dev, uint8_t *buf, uint8_t len)
{
    WINBOND_W83194_State *d = WINBOND_W83194(dev);

    for(int i = 0; i < len; i++)
    {
        switch(d->state)
        {
            case 0:
                d->command_code = buf[i] & 0x7f;
                qemu_printf("Winbond W83194: Command code 0x%02x given\n", d->command_code);

                if(d->command_code == 0x00) /* Block R/W */
                    d->state++;
                else
                    d->state = 3;
            break;

            case 1:
                d->n = buf[i] & 0x7f;
                qemu_printf("Winbond W83194: %d bytes will be written\n", d->n);
                d->block_state = 0;
                d->state++;
            break;

            case 2:
                if(d->block_state < 22) /* Prevent Overflow */
                    d->regs[d->block_state] = buf[i];

                qemu_printf("Winbond W83194: Writing 0x%02x to Register %d\n", buf[i], d->block_state);

                d->block_state++;
                if(d->block_state == d->n) {
                    qemu_printf("Winbond W83194: Block transmission complete!\n");
                    d->state = 0;
                }
            break;

            case 3:
                d->regs[d->reg_no % 22] = buf[i];
                qemu_printf("Winbond W83194: Writing 0x%02x to Register %d\n", buf[i], d->reg_no);

                d->reg_no++;
                qemu_printf("Winbond W83194: Byte transmission complete!\n");
            break;

            default:
                qemu_printf("Winbond W83194: Invalid Write State\n");
                d->state = 0;
            break; 
        }
    }

    return 0; /* Return acknowledgment no matter what */
}

static const VMStateDescription vmstate_winbond_w83194 = {
    .name = "winbond_w83194",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = 0,
    .fields      = (VMStateField[]) {
        VMSTATE_SMBUS_DEVICE(smbusdev, WINBOND_W83194_State),
        VMSTATE_UINT8_ARRAY(regs, WINBOND_W83194_State, 22),
        VMSTATE_UINT8(command_code, WINBOND_W83194_State),
        VMSTATE_UINT8(n, WINBOND_W83194_State),
        VMSTATE_UINT8(state, WINBOND_W83194_State),
        VMSTATE_UINT8(block_state, WINBOND_W83194_State),
        VMSTATE_END_OF_LIST()
    }
};

static void winbond_w83194_reset(DeviceState *dev)
{
    WINBOND_W83194_State *d = WINBOND_W83194(dev);
    d->command_code = 0;
    d->n = 0;
    d->state = 0;
    d->block_state = 0;
    d->reg_no = 0;

    memset(d->regs, 0, sizeof(d->regs)); /* Using Decimal values as used by the datasheet for better understading */
    d->regs[0] = 0x10;
    d->regs[1] = 0xe3;
    d->regs[2] = 0xff;
    d->regs[3] = 0xef;
    d->regs[4] = 0xfc;
    d->regs[6] = 0x08;
    d->regs[7] = 0x40;
    d->regs[8] = 0x8a;
    d->regs[9] = 0xce;
    d->regs[10] = 0x13;
    d->regs[11] = 0x2f;
    d->regs[12] = 0xc6;
    d->regs[13] = 0x0f;
    d->regs[14] = 0x27;
    d->regs[15] = 0xce;
    d->regs[16] = 0x24;
    d->regs[19] = 0x0a;
    d->regs[20] = 0x47;
    d->regs[21] = 0x47;
}

static void winbond_w83194_realize(DeviceState *dev, Error **errp)
{
    /* Just Reset */
    winbond_w83194_reset(dev);
}

static void winbond_w83194_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SMBusDeviceClass *sc = SMBUS_DEVICE_CLASS(klass);

    dc->realize = winbond_w83194_realize;
    dc->reset = winbond_w83194_reset;
    sc->receive_byte = winbond_w83194_read;
    sc->write_data = winbond_w83194_write;
    dc->vmsd = &vmstate_winbond_w83194;
    dc->user_creatable = false;
}

static const TypeInfo winbond_w83194_info = {
    .name          = TYPE_WINBOND_W83194,
    .parent        = TYPE_SMBUS_DEVICE,
    .instance_size = sizeof(WINBOND_W83194_State),
    .class_init    = winbond_w83194_class_initfn,
};

static void winbond_w83194_register_types(void)
{
    type_register_static(&winbond_w83194_info);
}

type_init(winbond_w83194_register_types)

void winbond_w83194_init(I2CBus *smbus)
{
    DeviceState *dev;

    dev = qdev_new(TYPE_WINBOND_W83194);
    qdev_prop_set_uint8(dev, "address", 0x69); /* Not a valid way to put it. Just most boards mounted it in address 0x69 */
    qdev_realize_and_unref(dev, (BusState *)smbus, &error_fatal);
}