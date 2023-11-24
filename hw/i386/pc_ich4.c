/*
 * Intel ICH4 Baseboard
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

/*

    Our Components are:
    
    Northbridge: Intel 845PE Brookdale
    Southbridge: Intel ICH4 Desktop
    Super I/O:   Winbond W83627HF

*/

#include "qemu/osdep.h"
#include CONFIG_DEVICES

#include "qemu/units.h"
#include "hw/char/parallel-isa.h"
#include "hw/dma/i8257.h"
#include "hw/loader.h"
#include "hw/i386/x86.h"
#include "hw/i386/apic.h"
#include "hw/display/ramfb.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_ids.h"
#include "hw/usb.h"
#include "hw/irq.h"
#include "sysemu/kvm.h"
#include "target/i386/kvm/kvm_i386.h"
#include "hw/i386/kvm/clock.h"
#include "hw/sysbus.h"
#include "hw/i2c/smbus_eeprom.h"
#include "exec/memory.h"
#include "hw/acpi/acpi.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "migration/global_state.h"
#include "migration/misc.h"
#include "sysemu/numa.h"
#include "hw/hyperv/vmbus-bridge.h"
#include "hw/i386/acpi-build.h"
#include "kvm/kvm-cpu.h"
#include "target/i386/cpu.h"

#include "hw/i386/pc.h"
#include "hw/pci-host/intel_845pe.h"
#include "hw/pci-bridge/intel_845pe_agp.h"
#include "hw/southbridge/intel_ich4_lpc.h"
#include "hw/pci-bridge/intel_ich4_hub.h"
#include "hw/rtc/intel_ich4_nvr.h"
#include "hw/isa/winbond_w83627hf.h"
#include "hw/block/fdc.h"
#include "hw/block/fdc-internal.h"
#include "hw/char/parallel.h"
#include "hw/char/parallel-isa.h"
#include "hw/char/serial.h"
#include "hw/ide/isa.h"
#include "hw/ide/pci.h"
#include "hw/ide/piix.h"
#include "hw/usb/hcd-uhci.h"
#include "hw/acpi/intel_ich4_acpi.h"
#include "qemu/qemu-print.h"

/*
 * Return the global irq number corresponding to a given device irq
 * pin. We could also use the bus number to have a more precise mapping.
 */
static int pc_pci_slot_get_pirq(PCIDevice *pci_dev, int pci_intx)
{
    int slot_addend;
    slot_addend = PCI_SLOT(pci_dev->devfn) - 1;
    return (pci_intx + slot_addend) & 7;
}

static void pc_init1(MachineState *machine)
{
    qemu_printf(" --- Intel ICH4 Start --- \n");
    PCMachineState *pcms = PC_MACHINE(machine);
    PCMachineClass *pcmc = PC_MACHINE_GET_CLASS(pcms);
    X86MachineState *x86ms = X86_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *system_io = get_system_io();
    MemoryRegion *ram_memory;
    MemoryRegion *pci_memory = NULL;
    MemoryRegion *rom_memory = system_memory;
    ram_addr_t lowmem;
    uint64_t hole64_size = 0;

    qemu_printf("PC: Loading Memory...\n");
    ram_memory = machine->ram;
    if (!pcms->max_ram_below_4g) {
        pcms->max_ram_below_4g = 4 * GiB;
    }
    lowmem = pcms->max_ram_below_4g;
    if (machine->ram_size >= pcms->max_ram_below_4g) {
        if (pcmc->gigabyte_align) {
            if (lowmem > 0xc0000000) {
                lowmem = 0xc0000000;
            }
            if (lowmem & (1 * GiB - 1)) {
                warn_report("Qemu: Large machine and max_ram_below_4g (%" PRIu64 ") not a multiple of 1G; possible bad performance.", pcms->max_ram_below_4g);
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

    /* Get the CPU up */
    qemu_printf("PC: Loading TCG/KVM...\n");
    x86_cpus_init(x86ms, pcmc->default_cpu_version);

    /* KVM */
    if (kvm_enabled() && pcmc->kvmclock_enabled) {
        qemu_printf("PC: KVM Clock Detected...\n");
        kvmclock_create(pcmc->kvmclock_create_always);
    }

    /* Disable Qemu's FW_CFG */
    x86ms->fw_cfg = 0;

    /* Initialize the PCI bus */
    qemu_printf("PC: Loading PCI bus...\n");
    Object *phb = OBJECT(qdev_new(TYPE_INTEL_845PE_HOST_BRIDGE)); /* The PCI Object */

    pci_memory = g_new(MemoryRegion, 1);
    memory_region_init(pci_memory, NULL, "pci", UINT64_MAX);
    rom_memory = pci_memory; /* Copy ROM Memory Contents to PCI Memory */

    qemu_printf("PC: Loading Intel 845PE MCH...\n");
    object_property_add_child(OBJECT(machine), "intel_845pe", phb); /* Intel 845PE */

    /* Memory Mappings. Conduct Page 102 of the Intel 845PE datasheet */
    object_property_set_link(phb, PCI_HOST_PROP_RAM_MEM, OBJECT(ram_memory), &error_fatal); /* Entire Memory */

    object_property_set_link(phb, PCI_HOST_PROP_PCI_MEM, OBJECT(pci_memory), &error_fatal); /* PCI Memory Range*/

    object_property_set_link(phb, PCI_HOST_PROP_SYSTEM_MEM, OBJECT(system_memory), &error_fatal); /* System Memory Space */

    object_property_set_link(phb, PCI_HOST_PROP_IO_MEM, OBJECT(system_io), &error_fatal); /* I/O Memory Range */

    object_property_set_uint(phb, PCI_HOST_BELOW_4G_MEM_SIZE, x86ms->below_4g_mem_size, &error_fatal); /* Memory below 4GB range */

    object_property_set_uint(phb, PCI_HOST_ABOVE_4G_MEM_SIZE, x86ms->above_4g_mem_size, &error_fatal); /* Extended above 4GB range */

    object_property_set_str(phb, INTEL_845PE_HOST_PROP_PCI_TYPE, TYPE_INTEL_845PE_PCI_DEVICE, &error_fatal); /* Set the Component type of the Intel 845PE */

    sysbus_realize_and_unref(SYS_BUS_DEVICE(phb), &error_fatal);

    PCIBus *pci_bus = PCI_BUS(qdev_get_child_bus(DEVICE(phb), "pci.0")); /* Create Bus 0 */
    pci_bus_map_irqs(pci_bus, pc_pci_slot_get_pirq); /* Assign PIRQ's to the slots */
    pcms->bus = pci_bus;

    hole64_size = object_property_get_uint(phb, PCI_HOST_PROP_PCI_HOLE64_SIZE, &error_abort); /* PCI 64-bit Hole Size */

    PCIDevice *intel_845pe_agp = pci_create_simple(pci_bus, PCI_DEVFN(0x01, 0), TYPE_INTEL_845PE_AGP);
    PCIBridge *agp_bridge = PCI_BRIDGE(intel_845pe_agp);
    pci_bridge_map_irq(agp_bridge, "pci.1", pc_pci_slot_get_pirq);

    /* Qemu's Guest Info Picker */
    pc_guest_info_init(pcms);

    /* Initialize the Memory */
    qemu_printf("PC: Linking Memory with the MCH...\n");
    pc_memory_init(pcms, system_memory, rom_memory, hole64_size);

    /* IRQ Interrupts */
    qemu_printf("PC: Receiving Interrupts...\n");
    GSIState *gsi_state = pc_gsi_create(&x86ms->gsi, pcmc->pci_enabled);

    /* Initialize the ICH4 */
    qemu_printf("PC: Starting Intel ICH4...\n");
    ICH4State *lpc;

    qemu_printf("PC: Starting Intel ICH4 LPC...\n");
    PCIDevice *intel_ich4_lpc = pci_create_simple_multifunction(pci_bus, PCI_DEVFN(0x1f, 0), TYPE_ICH4_DEVICE); /* Assign the LPC Bridge as a PCI device */

    lpc = ICH4_PCI_DEVICE(intel_ich4_lpc); /* Intel ICH4 LPC Bridge */
    lpc->pic = x86ms->gsi;

    /* Mount to the LPC BUS */
    qemu_printf("PC: Mount the Intel ICH4 LPC to the proper LPC Bus\n");
    ISABus *isa_bus = ISA_BUS(qdev_get_child_bus(DEVICE(lpc), "isa.0"));

    /* Mount the RTC */
    ISADevice *rtc_state = ISA_DEVICE(object_resolve_path_component(OBJECT(intel_ich4_lpc), "rtc"));

    /* Set up LPC Interrupts */
    isa_bus_register_input_irqs(isa_bus, x86ms->gsi);

    /* Setup APIC Interrupts */
    ioapic_init_gsi(gsi_state, "intel_845pe");

    /* That's more of a Qemu related feature related to their FPU code. Pretty much raises IRQ13 on a TCG FPU exception */
    if (tcg_enabled()) {
        x86_register_ferr_irq(x86ms->gsi[13]);
    }
 
    assert(pcms->vmport != ON_OFF_AUTO__MAX);
    if (pcms->vmport == ON_OFF_AUTO_AUTO) {
        pcms->vmport = ON_OFF_AUTO_ON;
    }

    /* Initialize the Hub bridge */
    PCIDevice *intel_ich4_hub = pci_create_simple(pci_bus, PCI_DEVFN(0x30, 0), TYPE_INTEL_ICH4_HUB);
    PCIBridge *hub_bridge = PCI_BRIDGE(intel_ich4_hub);
    pci_bridge_map_irq(hub_bridge, "pci.2", pc_pci_slot_get_pirq);

    /* Now that we got the basics up. Let's load our basic components */
    qemu_printf("PC: Loading Glue Components...\n");
    pcms->hpet_enabled = true; /* Intel ICH4 comes with HPET */
    pc_basic_device_init(pcms, isa_bus, x86ms->gsi, rtc_state, false, 0x08); /* VLSI Logic */
    object_property_add_link(OBJECT(machine), "rtc_state", TYPE_ISA_DEVICE, (Object **)&x86ms->rtc, object_property_allow_set_link, OBJ_PROP_LINK_STRONG); /* NVR */
    object_property_set_link(OBJECT(machine), "rtc_state", OBJECT(rtc_state), &error_abort);
    pc_i8259_create(isa_bus, gsi_state->i8259_irq); /* PIC Controller */

    /*
        Winbond W83827HF LPC Super I/O. Used by most Pentium 4 boards.
        Some used the ITE 87xx series of LPC Super I/Os too!
        OEMs tended to use the SMSC FDC87xxxx.
    */
    qemu_printf("PC: Loading Winbond W83627HF...");
    ISADevice *winbond = isa_new(TYPE_WINBOND_W83627HF); /* Winbond W83827HF */
    WinbondState *winbond_mount = WINBOND_W83627HF(winbond); /* Meant for mounting */
    isa_realize_and_unref(winbond, isa_bus, &error_fatal); /* Mount it to the LPC bus */

    /* Form the FDC and then bind it to the Winbond to program it */
    ISADevice *fdc = isa_new(TYPE_ISA_FDC);
    winbond_mount->fd = fdc;
    DriveInfo *fd[MAX_FD];

    for (int i = 0; i < MAX_FD; i++) {
        fd[i] = drive_get(IF_FLOPPY, 0, i);
    }

    isa_realize_and_unref(fdc, isa_bus, &error_fatal);
    isa_fdc_init_drives(fdc, fd);
    FDCtrl fdd = isa_fdc_get_controller(fdc);

    winbond_link_fdc(winbond_mount, fdd); /* Mount the FDC to the Winbond */

    /* Form the LPT and then bind it to the Winbond to program it */
    ISADevice *lpt = isa_new(TYPE_ISA_PARALLEL);
    DeviceState *lpt_dev = DEVICE(lpt);
    qdev_prop_set_uint32(lpt_dev, "index", 0);
    qdev_prop_set_chr(lpt_dev, "chardev", parallel_hds[0]);

    ParallelState parallel_state = parallel_get_state(lpt); /* Unlike Serial & FDC we don't really need to mount the state from inside the code. What a gimmick :b */

    isa_realize_and_unref(lpt, isa_bus, &error_fatal);
    winbond_mount->parallel = lpt;
    winbond_link_lpt(winbond_mount, parallel_state); /* Mount the LPT to the Winbond */

    /* Form the UARTs and then bind them to the Winbond to program them */
    ISADevice *uart[2];
    DeviceState *uart_dev[2];
    SerialState uart_state[2];

    for(int i = 0; i < 1; i++){ /* Two NSC 16550 Compatible Serial Handlers */
        uart[i] = isa_new(TYPE_ISA_SERIAL);
        uart_dev[i] = DEVICE(uart[i]);
        qdev_prop_set_uint32(uart_dev[i], "index", i);
        qdev_prop_set_chr(uart_dev[i], "chardev", serial_hd(i));
        isa_realize_and_unref(uart[i], isa_bus, &error_fatal);

        winbond_mount->serial[i] = uart[i];

        uart_state[i] = serial_isa_get_state(uart[i]);
        winbond_link_uart(winbond_mount, uart_state[i], i); /* Mount the UARTs to the Winbond */
    }

    /* IDE Compatible Drives */
    qemu_printf("PC: Loading IDE...\n");
    PCIDevice *intel_ich4_ide = pci_create_simple(pci_bus, PCI_DEVFN(0x1f, 0x01), TYPE_INTEL_ICH4_IDE);
    pci_ide_create_devs(intel_ich4_ide);

    /* Create UHCI Compatible Controllers */
    /* These are Qemu Standard Devices. Intel ICH4 EHCI is even implemented by the Qemu team themselves. */
    qemu_printf("PC: Loading USB...\n");
    pci_create_simple_multifunction(pci_bus, PCI_DEVFN(0x1d, 0), TYPE_INTEL_ICH4_UHCI(0));
    pci_create_simple(pci_bus, PCI_DEVFN(0x1d, 2), TYPE_INTEL_ICH4_UHCI(2));
    pci_create_simple(pci_bus, PCI_DEVFN(0x1d, 4), TYPE_INTEL_ICH4_UHCI(4));
    pci_create_simple(pci_bus, PCI_DEVFN(0x1d, 7), "intel-ich4-ehci");

    /* ACPI */
    qemu_printf("PC: Loading ACPI...\n");
    PCIDevice *intel_ich4_acpi = pci_new(PCI_DEVFN(0x1f, 3), TYPE_INTEL_ICH4_ACPI);;

    /* We expect a prebaked ACPI Table from the BIOS */
    pcms->acpi_build_enabled = 0;

    /* Create a Intel ICH4 Compatible ACPI device */
    Intel_ICH4_ACPI_State *acpi = INTEL_ICH4_ACPI(intel_ich4_acpi);

    /* Checks if SMM exists. We do a Pentium 4 machine which makes SMM presence mandatory. */
    qdev_prop_set_bit(DEVICE(intel_ich4_acpi), "smm-enabled", kvm_enabled() ? kvm_has_smm() : 1);

    /* Probably to provoke an initialization */
    pci_realize_and_unref(intel_ich4_acpi, pci_bus, &error_fatal);

    /* Set the ACPI IRQ pin to 9 like PIIX4 design. Normally we got to allow the ACPI to remap from the MCH */
    qdev_connect_gpio_out(DEVICE(intel_ich4_acpi), 0, x86ms->gsi[9]);

    /* SMI Trigger */
    qemu_irq smi_irq = qemu_allocate_irq(pc_acpi_smi_interrupt, first_cpu, 0);
    qdev_connect_gpio_out_named(DEVICE(intel_ich4_acpi), "smi-irq", 0, smi_irq);

    /* SMBus */
    /* Initialize the SMBus*/
    pcms->smbus = I2C_BUS(qdev_get_child_bus(DEVICE(intel_ich4_acpi), "i2c"));

    /* Intel 845PE utilizes DDR Memory */
    uint8_t *spd[4];
    int modules;

    /* Initialize the Serial Presence Detect data. Mostly Serial Presence Detection used by modern BIOS to determine RAM */
    if((machine->ram_size >> 20) > 1024)
        qemu_printf("PC: WARNING! Placing memory beyond 1GB. Make sure the target motherboard supports more than 2 slots else you may face unexpected behavior!\n");

    switch(machine->ram_size >> 20) {
        case 2048: /* Generate 4 512MB Memory Modules */
            modules = 4;
            spd[0] = spd_data_generate_real(512);
            spd[1] = spd_data_generate_real(512);
            spd[2] = spd_data_generate_real(512);
            spd[3] = spd_data_generate_real(512);
        break;

        case 1536: /* Generate 3 512MB Memory Modules */
            modules = 3;
            spd[0] = spd_data_generate_real(512);
            spd[1] = spd_data_generate_real(512);
            spd[2] = spd_data_generate_real(512);
        break;

        case 1024: /* Generate 2 512MB Memory Modules */
            modules = 2;
            spd[0] = spd_data_generate_real(512);
            spd[1] = spd_data_generate_real(512);
        break;

        case 512: /* Generate 2 256MB Memory Modules */
            modules = 2;
            spd[0] = spd_data_generate_real(256);
            spd[1] = spd_data_generate_real(256);
        break;

        case 256: /* Generate 2 128MB Memory Modules */
            modules = 2;
            spd[0] = spd_data_generate_real(128);
            spd[1] = spd_data_generate_real(128);
        break;

        case 128: /* Generate 1 128MB Memory Module */
            modules = 1;
            spd[0] = spd_data_generate_real(128);
        break;

        default:
            modules = 0;
            qemu_printf("PC: We don't have an SPD profile for such a RAM size yet. Fallback on Qemu's SPD\n");
            
        break;
    }

    if(modules == 0)
        spd_data_generate(DDR, machine->ram_size);
    else
        for(int i = 0; i < modules; i++)
            smbus_eeprom_init_one(pcms->smbus, 0x50 + i, spd[i]);  

    /* Link ACPIState with the LPC so we can remap it's I/O base */
    intel_ich4_link_acpi(lpc, acpi);

    qemu_printf("PC: Entering...\n");
}

#define DEFINE_INTEL_ICH4_MACHINE(suffix, name, compatfn, optionfn) \
    static void pc_init_##suffix(MachineState *machine) \
    { \
        void (*compat)(MachineState *m) = (compatfn); \
        if (compat) { \
            compat(machine); \
        } \
        pc_init1(machine); \
    } \
    DEFINE_PC_MACHINE(suffix, name, pc_init_##suffix, optionfn)

static void pc_ich4_machine_options(MachineClass *m)
{
    PCMachineClass *pcmc = PC_MACHINE_CLASS(m);
    pcmc->pci_root_uid = 0;
    pcmc->default_cpu_version = 1;
    pcmc->pci_enabled = 1;
    m->family = "pc-ich4";
    m->desc = "Intel ICH4 Based Machine";
    m->default_display = "std";
    m->no_parallel = false;
    machine_class_allow_dynamic_sysbus_dev(m, TYPE_RAMFB_DEVICE);
    machine_class_allow_dynamic_sysbus_dev(m, TYPE_VMBUS_BRIDGE);
}

static void pc_ich4_8_2_machine_options(MachineClass *m)
{
    pc_ich4_machine_options(m);
    m->alias = "pc-ich4";
    m->is_default = true;
}

DEFINE_INTEL_ICH4_MACHINE(v8_2, "pc-ich4-8.2", NULL, pc_ich4_8_2_machine_options);
