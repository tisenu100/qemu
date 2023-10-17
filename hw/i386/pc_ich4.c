/*
 * QEMU ICH4 Based System Emulation
 *
 * Copyright (c) 2003-2004 Fabrice Bellard
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
#include CONFIG_DEVICES

#include "qemu/units.h"
#include "hw/char/parallel-isa.h"
#include "hw/dma/i8257.h"
#include "hw/loader.h"
#include "hw/i386/x86.h"
#include "hw/i386/pc.h"
#include "hw/i386/apic.h"
#include "hw/rtc/mc146818rtc.h"
#include "hw/southbridge/piix.h"
#include "hw/display/ramfb.h"
#include "hw/firmware/smbios.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_ids.h"
#include "hw/usb.h"
#include "net/net.h"
#include "hw/ide/isa.h"
#include "hw/ide/pci.h"
#include "hw/ide/piix.h"
#include "hw/irq.h"
#include "sysemu/kvm.h"
#include "hw/i386/kvm/clock.h"
#include "hw/sysbus.h"
#include "hw/i2c/smbus_eeprom.h"
#include "exec/memory.h"
#include "hw/acpi/acpi.h"
#include "hw/acpi/piix4.h"
#include "hw/usb/hcd-uhci.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "migration/global_state.h"
#include "migration/misc.h"
#include "sysemu/numa.h"
#include "hw/hyperv/vmbus-bridge.h"
#include "hw/i386/acpi-build.h"
#include "kvm/kvm-cpu.h"
#include "target/i386/cpu.h"

#include "hw/pci-host/intel_845pe.h"
#include "hw/southbridge/ich4_lpc.h"
#include "qemu/qemu-print.h"

#define MAX_IDE_BUS 2

/*
 * Return the global irq number corresponding to a given device irq
 * pin. We could also use the bus number to have a more precise mapping.
 */
static int pc_pci_slot_get_pirq(PCIDevice *pci_dev, int pci_intx)
{
    int slot_addend;
    slot_addend = PCI_SLOT(PCI_DEVFN(0, 0)) - 1;
    return (pci_intx + slot_addend) & 3;
}

static void pc_init1(MachineState *machine, const char *host_type, const char *pci_type)
{
    qemu_printf(" --- Intel ICH4 Start --- \n");

    PCMachineState *pcms = PC_MACHINE(machine);
    PCMachineClass *pcmc = PC_MACHINE_GET_CLASS(pcms);
    X86MachineState *x86ms = X86_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *system_io = get_system_io();
    qemu_irq smi_irq;
    GSIState *gsi_state;
    ISADevice *rtc_state;
    MemoryRegion *ram_memory;
    MemoryRegion *pci_memory = NULL;
    MemoryRegion *rom_memory = system_memory;
    ram_addr_t lowmem;
    uint64_t hole64_size = 0;

    qemu_printf("PC: Loading Memory...\n");
    /*
     * Calculate ram split, for memory below and above 4G.  It's a bit
     * complicated for backward compatibility reasons ...
     *
     *  - Traditional split is 3.5G (lowmem = 0xe0000000).  This is the
     *    default value for max_ram_below_4g now.
     *
     *  - Then, to gigabyte align the memory, we move the split to 3G
     *    (lowmem = 0xc0000000).  But only in case we have to split in
     *    the first place, i.e. ram_size is larger than (traditional)
     *    lowmem.  And for new machine types (gigabyte_align = true)
     *    only, for live migration compatibility reasons.
     *
     *  - Next the max-ram-below-4g option was added, which allowed to
     *    reduce lowmem to a smaller value, to allow a larger PCI I/O
     *    window below 4G.  qemu doesn't enforce gigabyte alignment here,
     *    but prints a warning.
     *
     *  - Finally max-ram-below-4g got updated to also allow raising lowmem,
     *    so legacy non-PAE guests can get as much memory as possible in
     *    the 32bit address space below 4G.
     *
     *
     * Examples:
     *    qemu -M pc-1.7 -m 4G    (old default)    -> 3584M low,  512M high
     *    qemu -M pc -m 4G        (new default)    -> 3072M low, 1024M high
     *    qemu -M pc,max-ram-below-4g=2G -m 4G     -> 2048M low, 2048M high
     *    qemu -M pc,max-ram-below-4g=4G -m 3968M  -> 3968M low (=4G-128M)
     */

    ram_memory = machine->ram;
    if (!pcms->max_ram_below_4g) {
        pcms->max_ram_below_4g = 0xe0000000; /* default: 3.5G */
    }
    lowmem = pcms->max_ram_below_4g;
    if (machine->ram_size >= pcms->max_ram_below_4g) {
        if (pcmc->gigabyte_align) {
            if (lowmem > 0xc0000000) {
                lowmem = 0xc0000000;
            }
            if (lowmem & (1 * GiB - 1)) {
                warn_report("Large machine and max_ram_below_4g (%" PRIu64 ") not a multiple of 1G; possible bad performance.", pcms->max_ram_below_4g);
            }
        }
    }

    if (machine->ram_size >= lowmem) {
        x86ms->above_4g_mem_size = machine->ram_size - lowmem;
        x86ms->below_4g_mem_size = lowmem;
    } else {
        x86ms->above_4g_mem_size = 0;
        x86ms->below_4g_mem_size = machine->ram_size;
    }

    qemu_printf("PC: Loading TCG/KVM...\n");
    pc_machine_init_sgx_epc(pcms);
    x86_cpus_init(x86ms, pcmc->default_cpu_version);

    if (kvm_enabled() && pcmc->kvmclock_enabled) {
        kvmclock_create(pcmc->kvmclock_create_always);
    }

    /* Initialize the PCI bus */
    qemu_printf("PC: Loading PCI bus...\n");
    Object *phb;

    pci_memory = g_new(MemoryRegion, 1);
    memory_region_init(pci_memory, NULL, "pci", UINT64_MAX);
    rom_memory = pci_memory;

    phb = OBJECT(qdev_new(host_type));

    qemu_printf("PC: Loading Intel 845PE MCH...\n");

    object_property_add_child(OBJECT(machine), "intel_845pe", phb); /* Intel 845 PE */

    /* Memory Mappings. Conduct Page 102 of the Intel 845PE datasheet */
    object_property_set_link(phb, PCI_HOST_PROP_RAM_MEM, OBJECT(ram_memory), &error_fatal); /* Entire Memory */

    object_property_set_link(phb, PCI_HOST_PROP_PCI_MEM, OBJECT(pci_memory), &error_fatal); /* PCI Memory Range*/

    object_property_set_link(phb, PCI_HOST_PROP_SYSTEM_MEM, OBJECT(system_memory), &error_fatal); /* System Memory Space */

    object_property_set_link(phb, PCI_HOST_PROP_IO_MEM, OBJECT(system_io), &error_fatal); /* I/O Memory Range */

    object_property_set_uint(phb, PCI_HOST_BELOW_4G_MEM_SIZE, x86ms->below_4g_mem_size, &error_fatal); /* Memory */

    object_property_set_uint(phb, PCI_HOST_ABOVE_4G_MEM_SIZE, x86ms->above_4g_mem_size, &error_fatal); /* Extended Memory */

    object_property_set_str(phb, I845PE_HOST_PROP_PCI_TYPE, pci_type, &error_fatal); /* Set the Component type of the 845PE */
    sysbus_realize_and_unref(SYS_BUS_DEVICE(phb), &error_fatal);

    PCIBus *pci_bus = PCI_BUS(qdev_get_child_bus(DEVICE(phb), "pci.0")); /* Create Bus 0 */
    pci_bus_map_irqs(pci_bus, pc_pci_slot_get_pirq); /* Assign PIRQ's to the slots */
    pcms->bus = pci_bus;

    hole64_size = object_property_get_uint(phb, PCI_HOST_PROP_PCI_HOLE64_SIZE, &error_abort); /* PCI 64-bit Hole Size */

    pc_guest_info_init(pcms);

    /* Initialize the Memory */
    qemu_printf("PC: Linking Memory with the MCH...\n");
    pc_memory_init(pcms, system_memory, rom_memory, hole64_size);

    /* IRQ Interrupts */
    qemu_printf("PC: Recieving Interrupts...\n");
    gsi_state = pc_gsi_create(&x86ms->gsi, pcmc->pci_enabled);

    /* Initialize the ICH4 */
    qemu_printf("PC: Loading Intel ICH4 LPC...\n");
    ICH4State *ich4;
    PCIDevice *pci_dev;

    pci_dev = pci_create_simple_multifunction(pci_bus, PCI_DEVFN(0x1f, 0), TYPE_ICH4_DEVICE); /* Assign the LPC Bridge as a PCI device */

    ich4 = ICH4_PCI_DEVICE(pci_dev); /* Intel ICH4 LPC Bridge */
    ich4->pic = x86ms->gsi;

    /* Mount to the LPC BUS */
    qemu_printf("PC: Mount Intel ICH4 LPC to the proper LPC Bus\n");
    ISABus *isa_bus = ISA_BUS(qdev_get_child_bus(DEVICE(ich4), "isa.0"));

    rtc_state = ISA_DEVICE(object_resolve_path_component(OBJECT(pci_dev), "rtc"));

    isa_bus_register_input_irqs(isa_bus, x86ms->gsi);

    /* Initialize the PIC */
    qemu_printf("PC: Loading i8259 Compatible PIC\n");
    if (x86ms->pic == ON_OFF_AUTO_ON || x86ms->pic == ON_OFF_AUTO_AUTO) {
        pc_i8259_create(isa_bus, gsi_state->i8259_irq);
    }

    /* Setup APIC Interrupts */
    ioapic_init_gsi(gsi_state, "intel_845pe");

    /* That's more of a Qemu related feature related to their FPU code. Pretty much raises IRQ13 on a TCG FPU exception */
    if (tcg_enabled()) {
        x86_register_ferr_irq(x86ms->gsi[13]);
    }

    pc_vga_init(isa_bus, pci_bus);

    assert(pcms->vmport != ON_OFF_AUTO__MAX);
    if (pcms->vmport == ON_OFF_AUTO_AUTO) {
        pcms->vmport = ON_OFF_AUTO_ON;
    }

    /* init basic PC hardware */
    pc_basic_device_init(pcms, isa_bus, x86ms->gsi, rtc_state, true, 0x4);


    /* IDE Compatible Drives */
//    PCIDevice *ide = pci_create_simple(pci_bus, PCI_DEVFN(0x07, 0x01), TYPE_PIIX3_IDE);
//    BusState *idebus[MAX_IDE_BUS];
//    pci_ide_create_devs(ide);
//    idebus[0] = qdev_get_child_bus(&ide->qdev, "ide.0");
//    idebus[1] = qdev_get_child_bus(&ide->qdev, "ide.1");

    pc_cmos_init(pcms, NULL, NULL, rtc_state);


    /* Create UHCI Compatible Controllers */
//    PCIDevice *uhci[3];
//    for(int i = 0; i < 3; i++)
//        pci_create_simple_multifunction(pci_bus, PCI_DEVFN(0x29, (i * 2)), TYPE_PIIX3_USB_UHCI);


    /* ACPI */
    if (pcmc->pci_enabled && x86_machine_is_acpi_enabled(X86_MACHINE(pcms))) {
        PCIDevice *piix4_pm;

        /* We expect a prebaked ACPI Table from the BIOS */
        pcms->acpi_build_enabled = 0;

        /* SMI & SCI */
        smi_irq = qemu_allocate_irq(pc_acpi_smi_interrupt, first_cpu, 0);

        /* Create a PIIX4 Compatible ACPI device */
        piix4_pm = pci_new(PCI_DEVFN(0x1f, 9), TYPE_PIIX4_PM);

        /* Set the PIIX4 Compatible SMBus I/O Base to 0xb100. This has to be removed and be allowed by the SMBus Controller to set the I/O Base */
        qdev_prop_set_uint32(DEVICE(piix4_pm), "smb_io_base", 0xb100);

        /* Checks if SMM exists?? We do a Pentium 4 machine which is mandatory. */
        qdev_prop_set_bit(DEVICE(piix4_pm), "smm-enabled", x86_machine_is_smm_enabled(x86ms));

        /* Probably to provoke an initialization */
        pci_realize_and_unref(piix4_pm, pci_bus, &error_fatal);

        /* Set the ACPI IRQ pin to 9 like PIIX4 design. Normally we got to allow the ACPI to remap from the MCH */
        qdev_connect_gpio_out(DEVICE(piix4_pm), 0, x86ms->gsi[9]);

        /* SMI IRQ */
        qdev_connect_gpio_out_named(DEVICE(piix4_pm), "smi-irq", 0, smi_irq);

    /* SMBus */
        /* Initialize the SMBus*/
        pcms->smbus = I2C_BUS(qdev_get_child_bus(DEVICE(piix4_pm), "i2c"));

        /* Intel 845PE utilizes DDR Memory */
        uint8_t *spd = spd_data_generate(DDR, machine->ram_size);

        /* Initialize the SMBus EEPROM data. Mostly Serial Presence Detection used by modern BIOS to determine RAM */
        smbus_eeprom_init(pcms->smbus, 8, spd, 0);
        

        object_property_add_link(OBJECT(machine), PC_MACHINE_ACPI_DEVICE_PROP, TYPE_HOTPLUG_HANDLER, (Object **)&x86ms->acpi_dev, object_property_allow_set_link, OBJ_PROP_LINK_STRONG);
        object_property_set_link(OBJECT(machine), PC_MACHINE_ACPI_DEVICE_PROP, OBJECT(piix4_pm), &error_abort);
    }
}

#define DEFINE_ICH4_MACHINE(suffix, name, compatfn, optionfn) \
    static void pc_init_##suffix(MachineState *machine) \
    { \
        void (*compat)(MachineState *m) = (compatfn); \
        if (compat) { \
            compat(machine); \
        } \
        pc_init1(machine, TYPE_INTEL_845PE_HOST_BRIDGE, \
                 TYPE_INTEL_845PE_PCI_DEVICE); \
    } \
    DEFINE_PC_MACHINE(suffix, name, pc_init_##suffix, optionfn)

static void pc_ich4_machine_options(MachineClass *m)
{
    PCMachineClass *pcmc = PC_MACHINE_CLASS(m);
    pcmc->pci_root_uid = 0;
    pcmc->default_cpu_version = 1;
    pcmc->pci_enabled = 1;

    m->family = "pc-ich4";
    m->desc = "ICH4 Based Computer";
    m->default_display = "std";
    m->no_parallel = !module_object_class_by_name(TYPE_ISA_PARALLEL);
    machine_class_allow_dynamic_sysbus_dev(m, TYPE_RAMFB_DEVICE);
    machine_class_allow_dynamic_sysbus_dev(m, TYPE_VMBUS_BRIDGE);
}

static void pc_ich4_8_2_machine_options(MachineClass *m)
{
    pc_ich4_machine_options(m);
    m->alias = "pc-ich4";
    m->is_default = true;
}

DEFINE_ICH4_MACHINE(v8_2, "pc-ich4-8.2", NULL,
                      pc_ich4_8_2_machine_options);
