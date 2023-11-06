#ifndef WINBOND_H
#define WINBOND_H

#include "sysemu/sysemu.h"
#include "hw/isa/isa.h"
#include "qom/object.h"
#include "exec/memory.h"
#include "hw/block/fdc-internal.h"

#define TYPE_WINBOND_W83627HF "winbond-w83627hf"
OBJECT_DECLARE_SIMPLE_TYPE(WinbondState, WINBOND_W83627HF)

typedef struct WinbondState {
    /*< private >*/
    ISADevice parent_obj;
    /*< public >*/

    MemoryRegion io;

    uint8_t index; /* The index function */
    uint8_t regs[0x30]; /* Standard Winbond W83627HF Register set 00h-2Fh */
    bool extended_function_mode; /* Flag to allow programming of devices per LDN. Refer to the Winbond W83627HF datasheet. Write 87h to port 0x2e to raise, then 0xaah to lower */
    uint8_t ldn; /* LDN: Logical Device Number. Basically program multiple devices with the same register set. Refer to the Winbond W83627HF datasheet for more */
    uint8_t ldn_regs[12][0xff]; /* LDN Registers. Refer to the Winbond W83627HF datasheet for more */

    /* Floppy Disk Controller */
    uint16_t fdc_io_base;
    FDCtrl fdc;
} WinbondState;

/* Links Qemu's FDC to the Winbond */
void winbond_link_fdc(WinbondState *sio, FDCtrl fdc);

#endif /* WINBOND_H */
