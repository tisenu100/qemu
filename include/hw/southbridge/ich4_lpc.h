/*
 * Intel® 82801DB I/O Controller Hub 4 LPC Bridge
 *
 * Copyright (c) 2006 Fabrice Bellard
 * Copyright (c) 2018 Hervé Poussineau
 * Copyright (c) 2023 Tiseno100
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#ifndef HW_SOUTHBRIDGE_ICH4_H
#define HW_SOUTHBRIDGE_ICH4_H

#include "hw/pci/pci_device.h"
#include "hw/rtc/mc146818rtc.h"

/* PIRQRC[A:D]: PIRQx Route Control Registers */
#define ICH4_PIRQCA 0x60
#define ICH4_PIRQCB 0x61
#define ICH4_PIRQCC 0x62
#define ICH4_PIRQCD 0x63

/*
 * Reset Control Register: PCI-accessible ISA-Compatible Register at address
 * 0xcf9, provided by the PCI/ISA bridge (ICH4 PCI function 0, 8086:7000).
 */
#define ICH4_RCR_IOPORT 0xcf9

#define ICH4_NUM_PIC_IRQS       16      /* i8259 * 2 */
#define ICH4_NUM_PIRQS          8ULL    /* ICH4 is capable to store 8 PIRQ's. PIRQ[A-H] */

struct ICH4State {
    PCIDevice dev;

    uint64_t pic_levels;

    qemu_irq *pic;

    /* This member isn't used. Just for save/load compatibility */
    int32_t pci_irq_levels_vmstate[ICH4_NUM_PIRQS];

    MC146818RtcState rtc;

    /* Reset Control Register contents */
    uint8_t rcr;

    /* IO memory region for Reset Control Register (ICH4_RCR_IOPORT) */
    MemoryRegion rcr_mem;
};
typedef struct ICH4State ICH4State;

#define TYPE_ICH4_PCI_DEVICE "intel-ich4-lpc"
DECLARE_INSTANCE_CHECKER(ICH4State, ICH4_PCI_DEVICE,
                         TYPE_ICH4_PCI_DEVICE)

#define TYPE_ICH4_DEVICE "intel-ich4"

#endif
