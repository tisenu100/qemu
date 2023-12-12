/*
 * Intel ICH5 Baseboard
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
    
    Northbridge: Intel 865PE Springdale
    Southbridge: Intel ICH5 Desktop
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
#include "hw/pci-host/intel_865pe.h"
#include "hw/pci-bridge/intel_865pe_agp.h"
#include "hw/pci-bridge/intel_865pe_csa.h"
#include "hw/mem/intel_865pe_ovf.h"
#include "hw/southbridge/intel_ich5_lpc.h"
#include "hw/pci-bridge/intel_ich5_hub.h"
#include "hw/rtc/intel_ich5_nvr.h"
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
#include "hw/audio/intel_ich5_ac97.h"
#include "hw/acpi/intel_ich5_acpi.h"
#include "hw/i2c/ics950219.h"
#include "hw/i2c/winbond_w83194.h"
#include "qemu/qemu-print.h"

static int lpc_pirq(PCIDevice *pci_dev, int pci_intx)
{
    uint16_t pirq_table;

    switch(PCI_SLOT(pci_dev->devfn))
    {
        case 0x01:
            pirq_table = 0x7210;
        break;

        case 0x1d:
            pirq_table = 0x7230;
        break;

        case 0x1e:
            pirq_table = 0x3210;
        break;

        case 0x1f:
            pirq_table = 0x3012;
        break;

        default:
            qemu_printf("LPC PIRQ: Invalid Slot Assignment %d\n", PCI_SLOT(pci_dev->devfn));
            pirq_table = 0x3210;
        break;
    }

    pirq_table = (pirq_table >> (4 * pci_intx)) & 7;

    return pirq_table;
}

static int hub_pirq(PCIDevice *pci_dev, int pci_intx)
{
    uint16_t pirq_table;
    switch(PCI_SLOT(pci_dev->devfn))
    {
        case 0x00:
            pirq_table = 0x7654;
        break;

        case 0x04:
            pirq_table = 0x3210;
        break;

        case 0x05:
            pirq_table = 0x7653;
        break;

        case 0x07:
            pirq_table = 0x5376;
        break;

        case 0x08:
            pirq_table = 0x7654;
        break;

        case 0x09:
            pirq_table = 0x6537;
        break;

        case 0x0a:
            pirq_table = 0x3765;
        break;

        default:
            qemu_printf("HUB PIRQ: Invalid Slot Assignment %d\n", PCI_SLOT(pci_dev->devfn));
            pirq_table = 0x3210;
        break;
    }

    pirq_table = (pirq_table >> (4 * pci_intx)) & 7;

    return pirq_table;
}

static void pc_init1(MachineState *machine)
{
    qemu_printf(" --- Intel ICH5 Baseboard Start --- \n");
    PCMachineState *pcms = PC_MACHINE(machine);
    PCMachineClass *pcmc = PC_MACHINE_GET_CLASS(pcms);
    X86MachineState *x86ms = X86_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *system_io = get_system_io();
    MemoryRegion *ram_memory;
    MemoryRegion *pci_memory = NULL;
    MemoryRegion *rom_memory = system_memory;

    /* Initialize the Memory */
    qemu_printf("PC: Loading Memory...\n");
    ram_memory = machine->ram;
    if (!pcms->max_ram_below_4g) {
        pcms->max_ram_below_4g = 4 * GiB;
    }
    ram_addr_t lowmem = pcms->max_ram_below_4g; /* Memory below the 4GB range */
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
    Object *phb = OBJECT(qdev_new(TYPE_INTEL_865PE_HOST_BRIDGE)); /* The PCI Object */

    pci_memory = g_new(MemoryRegion, 1);
    memory_region_init(pci_memory, NULL, "pci", UINT64_MAX);
    rom_memory = pci_memory; /* Copy ROM Memory Contents to PCI Memory */

    qemu_printf("PC: Loading Intel 865PE MCH...\n");
    object_property_add_child(OBJECT(machine), TYPE_INTEL_865PE_PCI_DEVICE, phb); /* Intel 865PE */

    /* Memory Mappings. Conduct Page 102 of the Intel 865PE datasheet */
    object_property_set_link(phb, PCI_HOST_PROP_RAM_MEM, OBJECT(ram_memory), &error_fatal); /* Entire Memory */

    object_property_set_link(phb, PCI_HOST_PROP_PCI_MEM, OBJECT(pci_memory), &error_fatal); /* PCI Memory Range*/

    object_property_set_link(phb, PCI_HOST_PROP_SYSTEM_MEM, OBJECT(system_memory), &error_fatal); /* System Memory Space */

    object_property_set_link(phb, PCI_HOST_PROP_IO_MEM, OBJECT(system_io), &error_fatal); /* I/O Memory Range */

    object_property_set_uint(phb, PCI_HOST_BELOW_4G_MEM_SIZE, x86ms->below_4g_mem_size, &error_fatal); /* Memory below 4GB range */

    object_property_set_uint(phb, PCI_HOST_ABOVE_4G_MEM_SIZE, x86ms->above_4g_mem_size, &error_fatal); /* Extended above 4GB range */

    object_property_set_str(phb, INTEL_865PE_HOST_PROP_PCI_TYPE, TYPE_INTEL_865PE_PCI_DEVICE, &error_fatal); /* Set the Component type of the Intel 865PE */

    sysbus_realize_and_unref(SYS_BUS_DEVICE(phb), &error_fatal);

    PCIBus *pci_bus = PCI_BUS(qdev_get_child_bus(DEVICE(phb), "pci.0")); /* Get Bus 0 */
    pci_bus_map_irqs(pci_bus, lpc_pirq); /* Assign PIRQ's to the slots */
    pcms->bus = pci_bus;

    uint64_t hole64_size = object_property_get_uint(phb, PCI_HOST_PROP_PCI_HOLE64_SIZE, &error_abort); /* PCI 64-bit Hole Size */

    /* Overflow/DRAM Handler */
    pci_create_simple(pci_bus, PCI_DEVFN(0x06, 0), TYPE_INTEL_865PE_OVF);

    /* Qemu's Guest Info Picker */
    pc_guest_info_init(pcms);

    /* Initialize the Memory */
    qemu_printf("PC: Linking Memory with the MCH...\n");
    pc_memory_init(pcms, system_memory, rom_memory, hole64_size);

    /* IRQ Interrupts */
    qemu_printf("PC: Receiving Interrupts...\n");
    GSIState *gsi_state = pc_gsi_create(&x86ms->gsi, 1);

    /* Initialize the Intel ICH5 */
    qemu_printf("PC: Starting Intel ICH5...\n");
    Intel_ICH5_LPC_State *lpc;

    qemu_printf("PC: Starting Intel ICH5 LPC...\n");
    PCIDevice *intel_ich5_lpc = pci_new_multifunction(PCI_DEVFN(0x1f,0), TYPE_INTEL_ICH5); /* Intel ICH5 LPC Bridge */
    lpc = INTEL_ICH5_LPC(intel_ich5_lpc);

    for (int i = 0; i < 24; i++){  /* GSI (PIC + IOAPIC) Interrupts */
        qdev_connect_gpio_out_named(DEVICE(intel_ich5_lpc), "lpc-irqs", i, x86ms->gsi[i]);
    }
    pci_realize_and_unref(intel_ich5_lpc, pci_bus, &error_fatal); 

    /* Mount to the LPC BUS */
    qemu_printf("PC: Mount the Intel ICH5 LPC to the proper LPC Bus\n");
    ISABus *isa_bus = ISA_BUS(qdev_get_child_bus(DEVICE(lpc), "isa.0"));

    /* Mount the RTC */
    ISADevice *rtc_state = ISA_DEVICE(object_resolve_path_component(OBJECT(intel_ich5_lpc), "rtc"));

    /* Shove the I/O APIC addresses on the PCI Memory Address */
    ioapic_init_gsi(gsi_state, TYPE_INTEL_865PE_PCI_DEVICE);

    /* That's more of a Qemu related feature related to their FPU code. Pretty much raises IRQ13 on a TCG FPU exception */
    if (tcg_enabled()) {
        x86_register_ferr_irq(x86ms->gsi[13]);
    }
 
    assert(pcms->vmport != ON_OFF_AUTO__MAX);
    if (pcms->vmport == ON_OFF_AUTO_AUTO) {
        pcms->vmport = ON_OFF_AUTO_ON;
    }

    /* Now that we have the MCH & the Hub. Unleash the Bridges */
    /* Note: Bus resides are according to the board we target  */

    /* AGP Bridge residing on Bus 1 */
    PCIDevice *intel_865pe_agp = pci_new(PCI_DEVFN(0x01, 0), TYPE_INTEL_865PE_AGP);
    PCIBridge *agp_bridge = PCI_BRIDGE(intel_865pe_agp);
    pci_bridge_map_irq(agp_bridge, "pci.1", lpc_pirq); /* We don't have a dedicated AGP router considering we don't actually use AGP at all */
    pci_realize_and_unref(intel_865pe_agp, pci_bus, &error_fatal);

    /* CSA Bridge residing on Bus 2 */
    PCIDevice *intel_865pe_csa = pci_new(PCI_DEVFN(0x03, 0), TYPE_INTEL_865PE_CSA);
    PCIBridge *csa_bridge = PCI_BRIDGE(intel_865pe_csa);
    pci_bridge_map_irq(csa_bridge, "pci.2", lpc_pirq);
    pci_realize_and_unref(intel_865pe_csa, pci_bus, &error_fatal);

    /* Hub Bridge residing on Bus 3 */
    PCIDevice *intel_ich5_hub = pci_new(PCI_DEVFN(0x1e, 0), TYPE_INTEL_ICH5_HUB);
    PCIBridge *hub_bridge = PCI_BRIDGE(intel_ich5_hub);
    pci_bridge_map_irq(hub_bridge, "pci.3", hub_pirq);
    pci_realize_and_unref(intel_ich5_hub, pci_bus, &error_fatal);

    /* Now that we got the basics up. Let's load our basic components */
    qemu_printf("PC: Loading Glue Components...\n");
    pcms->hpet_enabled = true; /* Intel ICH5 comes with HPET */
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
    isa_realize_and_unref(winbond, isa_bus, &error_fatal); /* Mount it to the LPC bus */

    /* SATA Dummy Controller */
    /* Keep it disabled for now */
//    pci_create_simple(pci_bus, PCI_DEVFN(0x1f, 0x02), TYPE_INTEL_ICH5_SATA);

    /* IDE Compatible Drives */
    qemu_printf("PC: Loading IDE...\n");
    PCIDevice *intel_ich5_ide = pci_create_simple(pci_bus, PCI_DEVFN(0x1f, 0x01), TYPE_INTEL_ICH5_IDE);
    PCIIDEState *ide = PCI_IDE(intel_ich5_ide);
    pci_ide_create_devs(intel_ich5_ide);
    qdev_connect_gpio_out_named(DEVICE(ide), "isa-irq", 0, x86ms->gsi[14]);
    qdev_connect_gpio_out_named(DEVICE(ide), "isa-irq", 1, x86ms->gsi[15]);

    /* Create UHCI Compatible Controllers */
    /* These are Qemu Standard Devices    */
    qemu_printf("PC: Loading USB...\n");
    PCIDevice *intel_ich5_uhci[4];
    intel_ich5_uhci[0] = pci_create_simple_multifunction(pci_bus, PCI_DEVFN(0x1d, 0), TYPE_INTEL_ICH5_UHCI(0));
    intel_ich5_uhci[1] = pci_create_simple(pci_bus, PCI_DEVFN(0x1d, 1), TYPE_INTEL_ICH5_UHCI(1));
    intel_ich5_uhci[2] = pci_create_simple(pci_bus, PCI_DEVFN(0x1d, 2), TYPE_INTEL_ICH5_UHCI(2));
    intel_ich5_uhci[3] = pci_create_simple(pci_bus, PCI_DEVFN(0x1d, 3), TYPE_INTEL_ICH5_UHCI(3));
    pci_create_simple(pci_bus, PCI_DEVFN(0x1d, 7), "intel-ich5-ehci");

    UHCIState *uhci[4];
    for(int i = 0; i < 4; i++)
        uhci[i] = UHCI(intel_ich5_uhci[i]);
    
    /* ACPI */
    qemu_printf("PC: Loading ACPI...\n");
    PCIDevice *intel_ich5_acpi = pci_new(PCI_DEVFN(0x1f, 3), TYPE_INTEL_ICH5_ACPI);
    Intel_ICH5_ACPI_State *acpi = INTEL_ICH5_ACPI(intel_ich5_acpi);

    /* We expect a prebaked ACPI Table from the BIOS */
    pcms->acpi_build_enabled = 0;

    /* Checks if SMM exists. We do a Pentium 4 machine which makes SMM presence mandatory. */
    qdev_prop_set_bit(DEVICE(intel_ich5_acpi), "smm-enabled", kvm_enabled() ? kvm_has_smm() : 1);

    /* Probably to provoke an initialization */
    pci_realize_and_unref(intel_ich5_acpi, pci_bus, &error_fatal);

    /* SMI Trigger */
    qemu_irq smi_irq = qemu_allocate_irq(pc_acpi_smi_interrupt, first_cpu, 0);
    qdev_connect_gpio_out_named(DEVICE(intel_ich5_acpi), "smi-irq", 0, smi_irq);

    for(int i = 0; i < 4; i++)
        qdev_connect_gpio_out_named(DEVICE(uhci[i]), "smi-irq", 0, smi_irq);

    /* SMBus */
    /* Initialize the SMBus*/
    pcms->smbus = I2C_BUS(qdev_get_child_bus(DEVICE(intel_ich5_acpi), "i2c"));

    /* Intel 865PE utilizes DDR Memory */
    uint8_t *spd[4];
    int modules;

    /* Intel ICH5 Compatible AC'97 */
    pci_create_simple(pci_bus, PCI_DEVFN(0x1f, 5), TYPE_INTEL_ICH5_AC97);

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

    if(modules == 0) {
        spd[0] = spd_data_generate(DDR, machine->ram_size);
        smbus_eeprom_init_one(pcms->smbus, 0x50, spd[0]);
    }
    else
        for(int i = 0; i < modules; i++)
            smbus_eeprom_init_one(pcms->smbus, 0x50 + i, spd[i]);

    /* ICS Clock Chip */
//    winbond_w83194_init(pcms->smbus);
    ics950219_init(pcms->smbus);

    /* Link ACPIState with the LPC so we can remap it's I/O base */
    intel_ich5_link_acpi(lpc, acpi);
    intel_ich5_acpi_mount_kbc_trap(acpi, isa_bus);

    qemu_printf(" --- Intel ICH5 Baseboard Finish --- \n");
}

#define DEFINE_INTEL_ICH5_MACHINE(suffix, name, compatfn, optionfn) \
    static void pc_init_##suffix(MachineState *machine) \
    { \
        void (*compat)(MachineState *m) = (compatfn); \
        if (compat) { \
            compat(machine); \
        } \
        pc_init1(machine); \
    } \
    DEFINE_PC_MACHINE(suffix, name, pc_init_##suffix, optionfn)

static void pc_ich5_machine_options(MachineClass *m)
{
    PCMachineClass *pcmc = PC_MACHINE_CLASS(m);
    pcmc->pci_root_uid = 0;
    pcmc->default_cpu_version = 1;
    pcmc->pci_enabled = 1;
    m->family = "intel-ich5-baseboard";
    m->desc = "Intel ICH5 Baseboard";
    m->default_display = "std";
    m->no_parallel = false;
    machine_class_allow_dynamic_sysbus_dev(m, TYPE_RAMFB_DEVICE);
    machine_class_allow_dynamic_sysbus_dev(m, TYPE_VMBUS_BRIDGE);
}

static void pc_ich5_8_2_machine_options(MachineClass *m)
{
    pc_ich5_machine_options(m);
    m->alias = "intel-ich5-baseboard";
    m->is_default = true;
}

DEFINE_INTEL_ICH5_MACHINE(v8_2, "intel-ich5-baseboard-8.2", NULL, pc_ich5_8_2_machine_options);
