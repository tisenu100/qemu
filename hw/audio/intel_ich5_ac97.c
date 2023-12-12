/*
 * Intel ICH5 AC97
 *
 * Copyright (C) 2006 InnoTek Systemberatung GmbH
 * Copyright (C) 2023 Tiseno100
 * 
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation,
 * in version 2 as it comes in the "COPYING" file of the VirtualBox OSE
 * distribution. VirtualBox OSE is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY of any kind.
 *
 * If you received this file as part of a commercial VirtualBox
 * distribution, then only the terms of your commercial VirtualBox
 * license agreement apply instead of the previous paragraph.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/audio/soundhw.h"
#include "audio/audio.h"
#include "hw/irq.h"
#include "hw/pci/pci_device.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/qemu-print.h"
#include "sysemu/dma.h"
#include "qom/object.h"
#include "intel_ich5_ac97.h"

#define SOFT_VOLUME
#define SR_FIFOE 16             /* rwc */
#define SR_BCIS  8              /* rwc */
#define SR_LVBCI 4              /* rwc */
#define SR_CELV  2              /* ro */
#define SR_DCH   1              /* ro */
#define SR_VALID_MASK ((1 << 5) - 1)
#define SR_WCLEAR_MASK (SR_FIFOE | SR_BCIS | SR_LVBCI)
#define SR_RO_MASK (SR_DCH | SR_CELV)
#define SR_INT_MASK (SR_FIFOE | SR_BCIS | SR_LVBCI)

#define CR_IOCE  16             /* rw */
#define CR_FEIE  8              /* rw */
#define CR_LVBIE 4              /* rw */
#define CR_RR    2              /* rw */
#define CR_RPBM  1              /* rw */
#define CR_VALID_MASK ((1 << 5) - 1)
#define CR_DONT_CLEAR_MASK (CR_IOCE | CR_FEIE | CR_LVBIE)

#define GC_WR    4              /* rw */
#define GC_CR    2              /* rw */
#define GC_VALID_MASK ((1 << 6) - 1)

#define GS_MD3   (1 << 17)      /* rw */
#define GS_AD3   (1 << 16)      /* rw */
#define GS_RCS   (1 << 15)      /* rwc */
#define GS_B3S12 (1 << 14)      /* ro */
#define GS_B2S12 (1 << 13)      /* ro */
#define GS_B1S12 (1 << 12)      /* ro */
#define GS_S1R1  (1 << 11)      /* rwc */
#define GS_S0R1  (1 << 10)      /* rwc */
#define GS_S1CR  (1 << 9)       /* ro */
#define GS_S0CR  (1 << 8)       /* ro */
#define GS_MINT  (1 << 7)       /* ro */
#define GS_POINT (1 << 6)       /* ro */
#define GS_PIINT (1 << 5)       /* ro */
#define GS_RSRVD ((1 << 4) | (1 << 3))
#define GS_MOINT (1 << 2)       /* ro */
#define GS_MIINT (1 << 1)       /* ro */
#define GS_GSCI  1              /* rwc */
#define GS_RO_MASK (GS_B3S12 | \
                    GS_B2S12 | \
                    GS_B1S12 | \
                    GS_S1CR  | \
                    GS_S0CR  | \
                    GS_MINT  | \
                    GS_POINT | \
                    GS_PIINT | \
                    GS_RSRVD | \
                    GS_MOINT | \
                    GS_MIINT)
#define GS_VALID_MASK ((1 << 18) - 1)
#define GS_WCLEAR_MASK (GS_RCS | GS_S1R1 | GS_S0R1 | GS_GSCI)

#define BD_IOC (1 << 31)
#define BD_BUP (1 << 30)

#define TYPE_INTEL_ICH5_AC97 "intel-ich5-ac97"
OBJECT_DECLARE_SIMPLE_TYPE(Intel_ICH5_AC97_State, INTEL_ICH5_AC97)

#define REC_MASK 7
enum {
    REC_MIC = 0,
    REC_CD,
    REC_VIDEO,
    REC_AUX,
    REC_LINE_IN,
    REC_STEREO_MIX,
    REC_MONO_MIX,
    REC_PHONE
};

typedef struct BD {
    uint32_t addr;
    uint32_t ctl_len;
} BD;

typedef struct Intel_ICH5_AC97_BM_State {
    uint32_t bdbar;             /* rw 0 */
    uint8_t civ;                /* ro 0 */
    uint8_t lvi;                /* rw 0 */
    uint16_t sr;                /* rw 1 */
    uint16_t picb;              /* ro 0 */
    uint8_t piv;                /* ro 0 */
    uint8_t cr;                 /* rw 0 */
    unsigned int bd_valid;
    BD bd;
} Intel_ICH5_AC97_BM_State;

struct Intel_ICH5_AC97_State {
    PCIDevice dev;
    QEMUSoundCard card;
    uint32_t glob_cnt;
    uint32_t glob_sta;
    uint32_t cas;
    uint32_t last_samp;
    Intel_ICH5_AC97_BM_State bm_regs[3];
    uint8_t mixer_data[256];
    SWVoiceIn *voice_pi;
    SWVoiceIn *voice_p2;
    SWVoiceOut *voice_po;
    SWVoiceOut *voice_spdif;
    SWVoiceIn *voice_mc;
    SWVoiceIn *voice_mc2;
    int invalid_freq[3];
    uint8_t silence[128];
    int bup_flag;
    MemoryRegion io_nam;
    MemoryRegion io_nabm;
    MemoryRegion io_mm;
    MemoryRegion io_mb;
};

enum {
    BUP_SET = 1,
    BUP_LAST = 2
};

#define MKREGS(prefix, start)                   \
enum {                                          \
    prefix ## _BDBAR = start,                   \
    prefix ## _CIV = start + 4,                 \
    prefix ## _LVI = start + 5,                 \
    prefix ## _SR = start + 6,                  \
    prefix ## _PICB = start + 8,                \
    prefix ## _PIV = start + 10,                \
    prefix ## _CR = start + 11                  \
}

enum {
    PI_INDEX = 0,
    PO_INDEX,
    MC_INDEX,
    CODEC_SEMAPHORE, /* Unused */
    MC2_INDEX,
    PI2_INDEX,
    SPDIF_INDEX,
    LAST_INDEX
};

MKREGS(PI, PI_INDEX * 16);
MKREGS(PO, PO_INDEX * 16);
MKREGS(MC, MC_INDEX * 16);
MKREGS(MC2, MC2_INDEX * 16);
MKREGS(PI2, PI2_INDEX * 16);
MKREGS(SPDIF, SPDIF_INDEX * 16);

enum {
    GLOB_CNT = 0x2c,
    GLOB_STA = 0x30,
    CAS      = 0x34
};

#define GET_BM(index) (((index) >> 4) & 3)

static void po_callback(void *opaque, int free);
static void pi_callback(void *opaque, int avail);
static void mc_callback(void *opaque, int avail);

static void fetch_bd(Intel_ICH5_AC97_State *s, Intel_ICH5_AC97_BM_State *r)
{
    uint8_t b[8];

    pci_dma_read(&s->dev, r->bdbar + r->civ * 8, b, 8);
    r->bd_valid = 1;
    r->bd.addr = le32_to_cpu(*(uint32_t *) &b[0]) & ~3;
    r->bd.ctl_len = le32_to_cpu(*(uint32_t *) &b[4]);
    r->picb = r->bd.ctl_len & 0xffff;
}

static void update_sr(Intel_ICH5_AC97_State *s, Intel_ICH5_AC97_BM_State *r, uint32_t new_sr)
{
    int event = 0;
    int level = 0;
    uint32_t new_mask = new_sr & SR_INT_MASK;
    uint32_t old_mask = r->sr & SR_INT_MASK;
    uint32_t masks[] = {GS_PIINT, GS_POINT, GS_MINT};

    if (new_mask ^ old_mask) {
        /** @todo is IRQ deasserted when only one of status bits is cleared? */
        if (!new_mask) {
            event = 1;
            level = 0;
        } else {
            if ((new_mask & SR_LVBCI) && (r->cr & CR_LVBIE)) {
                event = 1;
                level = 1;
            }
            if ((new_mask & SR_BCIS) && (r->cr & CR_IOCE)) {
                event = 1;
                level = 1;
            }
        }
    }

    r->sr = new_sr;

    if (!event) {
        return;
    }

    if (level) {
        s->glob_sta |= masks[r - s->bm_regs];
        pci_irq_assert(&s->dev);
    } else {
        s->glob_sta &= ~masks[r - s->bm_regs];
        pci_irq_deassert(&s->dev);
    }
}

static void voice_set_active(Intel_ICH5_AC97_State *s, int bm_index, int on)
{
    switch (bm_index) {
    case PI_INDEX:
        AUD_set_active_in(s->voice_pi, on);
        break;

    case PO_INDEX:
        AUD_set_active_out(s->voice_po, on);
        break;

    case MC_INDEX:
        AUD_set_active_in(s->voice_mc, on);
        break;

    default:
        break;
    }
}

static void reset_bm_regs(Intel_ICH5_AC97_State *s, Intel_ICH5_AC97_BM_State *r)
{
    r->bdbar = 0;
    r->civ = 0;
    r->lvi = 0;
    /** todo do we need to do that? */
    update_sr(s, r, SR_DCH);
    r->picb = 0;
    r->piv = 0;
    r->cr = r->cr & CR_DONT_CLEAR_MASK;
    r->bd_valid = 0;

    voice_set_active(s, r - s->bm_regs, 0);
    memset(s->silence, 0, sizeof(s->silence));
}

static void mixer_store(Intel_ICH5_AC97_State *s, uint32_t i, uint16_t v)
{
    if (i + 2 > sizeof(s->mixer_data)) {
        return;
    }

    s->mixer_data[i + 0] = v & 0xff;
    s->mixer_data[i + 1] = v >> 8;
}

static uint16_t mixer_load(Intel_ICH5_AC97_State *s, uint32_t i)
{
    uint16_t val = 0xffff;

    if (i + 2 > sizeof(s->mixer_data)) {
    } else {
        val = s->mixer_data[i + 0] | (s->mixer_data[i + 1] << 8);
    }

    return val;
}

static void open_voice(Intel_ICH5_AC97_State *s, int index, int freq)
{
    struct audsettings as;

    as.freq = freq;
    as.nchannels = 2;
    as.fmt = AUDIO_FORMAT_S16;
    as.endianness = 0;

    if (freq > 0) {
        s->invalid_freq[index] = 0;
        switch (index) {
        case PI_INDEX:
            s->voice_pi = AUD_open_in(&s->card, s->voice_pi, "intel-ich5-ac97.pi", s, pi_callback, &as);
            break;

        case PO_INDEX:
            s->voice_po = AUD_open_out(&s->card, s->voice_po, "intel-ich5-ac97.po", s, po_callback, &as);
            break;

        case MC_INDEX:
            s->voice_mc = AUD_open_in(&s->card, s->voice_mc, "intel-ich5-ac97.mc", s, mc_callback, &as);
            break;
        }
    } else {
        s->invalid_freq[index] = freq;
        switch (index) {
        case PI_INDEX:
            AUD_close_in(&s->card, s->voice_pi);
            s->voice_pi = NULL;
            break;

        case PO_INDEX:
            AUD_close_out(&s->card, s->voice_po);
            s->voice_po = NULL;
            break;

        case MC_INDEX:
            AUD_close_in(&s->card, s->voice_mc);
            s->voice_mc = NULL;
            break;
        }
    }
}

static void reset_voices(Intel_ICH5_AC97_State *s, uint8_t active[LAST_INDEX])
{
    uint16_t freq;

    freq = mixer_load(s, AC97_PCM_LR_ADC_Rate);
    open_voice(s, PI_INDEX, freq);
    AUD_set_active_in(s->voice_pi, active[PI_INDEX]);

    freq = mixer_load(s, AC97_PCM_Front_DAC_Rate);
    open_voice(s, PO_INDEX, freq);
    AUD_set_active_out(s->voice_po, active[PO_INDEX]);

    freq = mixer_load(s, AC97_MIC_ADC_Rate);
    open_voice(s, MC_INDEX, freq);
    AUD_set_active_in(s->voice_mc, active[MC_INDEX]);
}

static void get_volume(uint16_t vol, uint16_t mask, int inverse,
                       int *mute, uint8_t *lvol, uint8_t *rvol)
{
    *mute = (vol >> MUTE_SHIFT) & 1;
    *rvol = (255 * (vol & mask)) / mask;
    *lvol = (255 * ((vol >> 8) & mask)) / mask;

    if (inverse) {
        *rvol = 255 - *rvol;
        *lvol = 255 - *lvol;
    }
}

static void update_combined_volume_out(Intel_ICH5_AC97_State *s)
{
    uint8_t lvol, rvol, plvol, prvol;
    int mute, pmute;

    get_volume(mixer_load(s, AC97_Master_Volume_Mute), 0x1f, 0, &mute, &lvol, &rvol);
    get_volume(mixer_load(s, AC97_PCM_Out_Volume_Mute), 0x1f, 1, &pmute, &plvol, &prvol);

    mute = mute | pmute;
    lvol = (lvol * plvol) / 255;
    rvol = (rvol * prvol) / 255;

    AUD_set_volume_out(s->voice_po, mute, lvol, rvol);
}

static void update_volume_in(Intel_ICH5_AC97_State *s)
{
    uint8_t lvol, rvol;
    int mute;

    get_volume(mixer_load(s, AC97_Record_Gain_Mute), 0x1f, 1, &mute, &lvol, &rvol);

    AUD_set_volume_in(s->voice_pi, mute, lvol, rvol);
}

static void set_volume(Intel_ICH5_AC97_State *s, int index, uint32_t val)
{
    switch (index) {
    case AC97_Master_Volume_Mute:
        val &= 0x9f1f;
        mixer_store(s, index, val);
        update_combined_volume_out(s);
        break;
    case AC97_PCM_Out_Volume_Mute:
        val &= 0x9f1f;
        mixer_store(s, index, val);
        update_combined_volume_out(s);
        break;
    case AC97_Record_Gain_Mute:
        val &= 0x8f0f;
        mixer_store(s, index, val);
        update_volume_in(s);
        break;
    }
}

static void record_select(Intel_ICH5_AC97_State *s, uint32_t val)
{
    uint8_t rs = val & REC_MASK;
    uint8_t ls = (val >> 8) & REC_MASK;
    mixer_store(s, AC97_Record_Select, rs | (ls << 8));
}

static void mixer_reset(Intel_ICH5_AC97_State *s)
{
    uint8_t active[LAST_INDEX];

    memset(s->mixer_data, 0, sizeof(s->mixer_data));
    memset(active, 0, sizeof(active));
    mixer_store(s, AC97_Headphone_Volume_Mute, 0x8000);
    mixer_store(s, AC97_Master_Volume_Mono_Mute, 0x8000);
    mixer_store(s, AC97_PC_BEEP_Volume_Mute, 0x8000);
    mixer_store(s, AC97_Phone_Volume_Mute, 0x8008);
    mixer_store(s, AC97_Mic_Volume_Mute, 0x8008);
    mixer_store(s, AC97_Line_In_Volume_Mute, 0x8808);
    mixer_store(s, AC97_CD_Volume_Mute, 0x8808);
    mixer_store(s, AC97_Video_Volume_Mute, 0x8000);
    mixer_store(s, AC97_Aux_Volume_Mute, 0x8808);
    mixer_store(s, AC97_Powerdown_Ctrl_Stat, 0x000f);
    mixer_store(s, AC97_PCM_Front_DAC_Rate, 0xbb80);
    mixer_store(s, AC97_PCM_Surround_DAC_Rate, 0xbb80);
    mixer_store(s, AC97_PCM_LFE_DAC_Rate, 0xbb80);
    mixer_store(s, AC97_PCM_LR_ADC_Rate, 0xbb80);
    mixer_store(s, AC97_MIC_ADC_Rate, 0xbb80);
    mixer_store(s, AC97_6Ch_Vol_C_LFE_Mute, 0x8080);
    mixer_store(s, AC97_6Ch_Vol_L_R_Surround_Mute, 0x8080);
    mixer_store(s, AC97_SPDIF_Control, 0x2000);
    mixer_store(s, AC97_SPDIF_DAC_Volume, 0x0808);
    mixer_store(s, AC97_Sense_Function_Information, 0x02f1);
    mixer_store(s, AC97_Sense_Detail, 0x0500);
    mixer_store(s, AC97_Extension_Control, 0x60a0);

    mixer_store(s, AC97_Vendor_ID1, 0x414c); /* Use the Realtek ALC665 Configuration */
    mixer_store(s, AC97_Vendor_ID2, 0x4760);
    mixer_store(s, AC97_Extended_Audio_ID, 0x09c4);
    mixer_store(s, AC97_Extended_Audio_Ctrl_Stat, 0x05f0);

    record_select(s, 0);
    set_volume(s, AC97_Master_Volume_Mute, 0x8000);
    set_volume(s, AC97_PCM_Out_Volume_Mute, 0x8808);
    set_volume(s, AC97_Record_Gain_Mute, 0x8808);

    reset_voices(s, active);
}

/**
 * Native audio mixer
 * I/O Reads
 */
static uint32_t nam_readb(void *opaque, uint32_t addr)
{
    Intel_ICH5_AC97_State *s = opaque;
    s->cas = 0;
    return ~0U;
}

static uint32_t nam_readw(void *opaque, uint32_t addr)
{
    Intel_ICH5_AC97_State *s = opaque;
    s->cas = 0;
    return mixer_load(s, addr);
}

static uint32_t nam_readl(void *opaque, uint32_t addr)
{
    Intel_ICH5_AC97_State *s = opaque;
    s->cas = 0;
    return ~0U;
}

/**
 * Native audio mixer
 * I/O Writes
 */
static void nam_writeb(void *opaque, uint32_t addr, uint32_t val)
{
    Intel_ICH5_AC97_State *s = opaque;
    s->cas = 0;
}

static void nam_writew(void *opaque, uint32_t addr, uint32_t val)
{
    Intel_ICH5_AC97_State *s = opaque;

    /*
    
        Our Codec is Realtek. Mainly we target the functionality of the Realtek ALC665

    */

    s->cas = 0; /* Clear Semaphore to indicate the codec was accessed */

    switch (addr) {
    /* Reset/Power Handlers */
    case AC97_Reset: /* Reset. Any writes will basically reset the register */
        mixer_reset(s);
        break;
    case AC97_Powerdown_Ctrl_Stat:
        val &= ~0x800f;
        val |= mixer_load(s, addr) & 0xf;
        mixer_store(s, addr, val);
        break;

    /* Volume Handlers. What Qemu really needs of */
    case AC97_Master_Volume_Mute: /* Master */
    case AC97_PCM_Out_Volume_Mute: /* PCM Out */
    case AC97_Record_Gain_Mute: /* Mic In */
        set_volume(s, addr, val);
        break;
    case AC97_Record_Select:
        record_select(s, val);
        break;

    /* Extended Audio Controls */
    case AC97_Extended_Audio_Ctrl_Stat:
        mixer_store(s, AC97_PCM_Front_DAC_Rate, 0xbb80); /* At ALC665 datasheet. It's reported that the Front DAC can do up to 96Khz! */
        mixer_store(s, AC97_PCM_LR_ADC_Rate,    0xbb80);
        mixer_store(s, AC97_MIC_ADC_Rate, 0xbb80);
        open_voice(s, PI_INDEX, 48000);
        open_voice(s, PO_INDEX, 48000);
        open_voice(s, MC_INDEX, 48000);
        mixer_store(s, AC97_Extended_Audio_Ctrl_Stat, val);
        break;

    /* Audio Interrupt Handler */
    case AC97_Audio_Int_and_Paging:
        val &= ~0x8000; /* Global IRQ Handler */
        break;

    case AC97_PCM_Front_DAC_Rate:
        open_voice(s, PO_INDEX, val);
        break;
    case AC97_MIC_ADC_Rate:
        open_voice(s, MC_INDEX, val);
        break;
    case AC97_PCM_LR_ADC_Rate:
        open_voice(s, PI_INDEX, val);
        break;

    /* """Sense""" */
    case AC97_Sense_Function_Select:
        mixer_store(s, AC97_Sense_Detail, val);

        /* Now the real speculated deal */
        s->mixer_data[0x68] |= 0x10; /* Report that the sensed data were handled */
        break;

    case AC97_Sense_Function_Information:
        val &= ~0x0010; /* Clear the Status Bit */
        mixer_store(s, AC97_Sense_Function_Information, val);
        break;

    /* These Functions are not emulated or route to read only bits */
    case AC97_Headphone_Volume_Mute:
    case AC97_Master_Volume_Mono_Mute:
    case AC97_Master_Tone_RL:
    case AC97_PC_BEEP_Volume_Mute:
    case AC97_Phone_Volume_Mute:
    case AC97_Mic_Volume_Mute:
    case AC97_Line_In_Volume_Mute:
    case AC97_CD_Volume_Mute:
    case AC97_Video_Volume_Mute:
    case AC97_Aux_Volume_Mute:
    case AC97_Record_Gain_Mic_Mute:
    case AC97_General_Purpose:
    case AC97_3D_Control:
    case AC97_SPDIF_DAC_Volume:
    case AC97_Vendor_ID1:
    case AC97_Vendor_ID2:
    case AC97_6Ch_Vol_C_LFE_Mute:
        /* Ignore */
        break;
    default: /* Whatever value we don't know or care tb written at */
        mixer_store(s, addr, val);
        break;
    }
}

static void nam_writel(void *opaque, uint32_t addr, uint32_t val)
{
    Intel_ICH5_AC97_State *s = opaque;
    s->cas = 0;
}

/**
 * Native audio bus master
 * I/O Reads
 */
static uint32_t nabm_readb(void *opaque, uint32_t addr)
{
    Intel_ICH5_AC97_State *s = opaque;
    Intel_ICH5_AC97_BM_State *r = NULL;
    uint32_t val = ~0U;

    switch (addr) {
    case CAS:
        val = s->cas;
        s->cas = 1;
        break;
    case PI_CIV:
    case PO_CIV:
    case MC_CIV:
    case MC2_CIV:
    case PI2_CIV:
    case SPDIF_CIV:
        r = &s->bm_regs[GET_BM(addr)];
        val = r->civ;
        break;
    case PI_LVI:
    case PO_LVI:
    case MC_LVI:
    case MC2_LVI:
    case PI2_LVI:
    case SPDIF_LVI:
        r = &s->bm_regs[GET_BM(addr)];
        val = r->lvi;
        break;
    case PI_PIV:
    case PO_PIV:
    case MC_PIV:
    case MC2_PIV:
    case PI2_PIV:
    case SPDIF_PIV:
        r = &s->bm_regs[GET_BM(addr)];
        val = r->piv;
        break;
    case PI_CR:
    case PO_CR:
    case MC_CR:
    case MC2_CR:
    case PI2_CR:
    case SPDIF_CR:
        r = &s->bm_regs[GET_BM(addr)];
        val = r->cr;
        break;
    case PI_SR:
    case PO_SR:
    case MC_SR:
    case MC2_SR:
    case PI2_SR:
    case SPDIF_SR:
        r = &s->bm_regs[GET_BM(addr)];
        val = r->sr & 0xff;
        break;
    default:
        break;
    }
    return val;
}

static uint32_t nabm_readw(void *opaque, uint32_t addr)
{
    Intel_ICH5_AC97_State *s = opaque;
    Intel_ICH5_AC97_BM_State *r = NULL;
    uint32_t val = ~0U;

    switch (addr) {
    case PI_SR:
    case PO_SR:
    case MC_SR:
    case MC2_SR:
    case PI2_SR:
    case SPDIF_SR:
        r = &s->bm_regs[GET_BM(addr)];
        val = r->sr;
        break;
    case PI_PICB:
    case PO_PICB:
    case MC_PICB:
    case MC2_PICB:
    case PI2_PICB:
    case SPDIF_PICB:
        r = &s->bm_regs[GET_BM(addr)];
        val = r->picb;
        break;
    }
    return val;
}

static uint32_t nabm_readl(void *opaque, uint32_t addr)
{
    Intel_ICH5_AC97_State *s = opaque;
    Intel_ICH5_AC97_BM_State *r = NULL;
    uint32_t val = ~0U;

    switch (addr) {
    case PI_BDBAR:
    case PO_BDBAR:
    case MC_BDBAR:
    case MC2_BDBAR:
    case PI2_BDBAR:
    case SPDIF_BDBAR:
        r = &s->bm_regs[GET_BM(addr)];
        val = r->bdbar;
        break;
    case PI_CIV:
    case PO_CIV:
    case MC_CIV:
    case MC2_CIV:
    case PI2_CIV:
    case SPDIF_CIV:
        r = &s->bm_regs[GET_BM(addr)];
        val = r->civ | (r->lvi << 8) | (r->sr << 16);
        break;
    case PI_PICB:
    case PO_PICB:
    case MC_PICB:
    case MC2_PICB:
    case PI2_PICB:
    case SPDIF_PICB:
        r = &s->bm_regs[GET_BM(addr)];
        val = r->picb | (r->piv << 16) | (r->cr << 24);
        break;
    case GLOB_CNT:
        val = s->glob_cnt;
        break;
    case GLOB_STA:
        val = s->glob_sta | GS_S0CR;
        break;
    default:
        break;
    }
    return val;
}

/**
 * Native audio bus master
 * I/O Writes
 */
static void nabm_writeb(void *opaque, uint32_t addr, uint32_t val)
{
    Intel_ICH5_AC97_State *s = opaque;
    Intel_ICH5_AC97_BM_State *r = NULL;

    switch (addr) {
    /* Last Valid Indexes. Rest are of the same order */
    case PI_LVI: /* PCM In */
    case PO_LVI: /* PCM Out */
    case MC_LVI: /* Mic In  */
    case MC2_LVI: /* Mic2 In */
    case PI2_LVI: /* PCM2 In */
    case SPDIF_LVI: /* SPDIF */
        r = &s->bm_regs[GET_BM(addr)];
        if ((r->cr & CR_RPBM) && (r->sr & SR_DCH)) {
            r->sr &= ~(SR_DCH | SR_CELV);
            r->civ = r->piv;
            r->piv = (r->piv + 1) % 32;
            fetch_bd(s, r);
        }
        r->lvi = val % 32;
        break;

    /* Control */
    case PI_CR:
    case PO_CR:
    case MC_CR:
    case MC2_CR:
    case PI2_CR:
    case SPDIF_CR:
        r = &s->bm_regs[GET_BM(addr)];
        if (val & CR_RR) {
            reset_bm_regs(s, r);
        } else {
            r->cr = val & CR_VALID_MASK;
            if (!(r->cr & CR_RPBM)) {
                voice_set_active(s, r - s->bm_regs, 0);
                r->sr |= SR_DCH;
            } else {
                r->civ = r->piv;
                r->piv = (r->piv + 1) % 32;
                fetch_bd(s, r);
                r->sr &= ~SR_DCH;
                voice_set_active(s, r - s->bm_regs, 1);
            }
        }
        break;

    /* Status */
    case PI_SR:
    case PO_SR:
    case MC_SR:
    case MC2_SR:
    case PI2_SR:
    case SPDIF_SR:
        r = &s->bm_regs[GET_BM(addr)];
        r->sr |= val & ~(SR_RO_MASK | SR_WCLEAR_MASK);
        update_sr(s, r, r->sr & ~(val & SR_WCLEAR_MASK));
        break;
    default:
        break;
    }
}

static void nabm_writew(void *opaque, uint32_t addr, uint32_t val)
{
    Intel_ICH5_AC97_State *s = opaque;
    Intel_ICH5_AC97_BM_State *r = NULL;

    switch (addr) {
    case PI_SR:
    case PO_SR:
    case MC_SR:
    case MC2_SR:
    case PI2_CR:
    case SPDIF_CR:
        r = &s->bm_regs[GET_BM(addr)];
        r->sr |= val & ~(SR_RO_MASK | SR_WCLEAR_MASK);
        update_sr(s, r, r->sr & ~(val & SR_WCLEAR_MASK));
        break;
    default:
        break;
    }
}

static void nabm_writel(void *opaque, uint32_t addr, uint32_t val)
{
    Intel_ICH5_AC97_State *s = opaque;
    Intel_ICH5_AC97_BM_State *r = NULL;

    switch (addr) {
    /* Bus Master Address */
    case PI_BDBAR:
    case PO_BDBAR:
    case MC_BDBAR:
    case MC2_BDBAR:
    case PI2_BDBAR:
    case SPDIF_BDBAR:
        r = &s->bm_regs[GET_BM(addr)];
        r->bdbar = val & ~3;
        break;
    case GLOB_CNT:
        /* TODO: Handle WR or CR being set (warm/cold reset requests) */
        if (!(val & (GC_WR | GC_CR))) {
            s->glob_cnt = val & GC_VALID_MASK;
        }
        break;

    /* Global Status */
    case GLOB_STA:
        s->glob_sta &= ~(val & GS_WCLEAR_MASK);
        s->glob_sta |= (val & ~(GS_WCLEAR_MASK | GS_RO_MASK)) & GS_VALID_MASK;
        break;
    default:
        break;
    }
}

static int write_audio(Intel_ICH5_AC97_State *s, Intel_ICH5_AC97_BM_State *r,
                       int max, int *stop)
{
    uint8_t tmpbuf[4096];
    uint32_t addr = r->bd.addr;
    uint32_t temp = r->picb << 1;
    uint32_t written = 0;
    int to_copy = 0;
    temp = MIN(temp, max);

    if (!temp) {
        *stop = 1;
        return 0;
    }

    while (temp) {
        int copied;
        to_copy = MIN(temp, sizeof(tmpbuf));
        pci_dma_read(&s->dev, addr, tmpbuf, to_copy);
        copied = AUD_write(s->voice_po, tmpbuf, to_copy);
        if (!copied) {
            *stop = 1;
            break;
        }
        temp -= copied;
        addr += copied;
        written += copied;
    }

    if (!temp) {
        if (to_copy < 4) {
            s->last_samp = 0;
        } else {
            s->last_samp = *(uint32_t *)&tmpbuf[to_copy - 4];
        }
    }

    r->bd.addr = addr;
    return written;
}

static void write_bup(Intel_ICH5_AC97_State *s, int elapsed)
{
    if (!(s->bup_flag & BUP_SET)) {
        if (s->bup_flag & BUP_LAST) {
            int i;
            uint8_t *p = s->silence;
            for (i = 0; i < sizeof(s->silence) / 4; i++, p += 4) {
                *(uint32_t *) p = s->last_samp;
            }
        } else {
            memset(s->silence, 0, sizeof(s->silence));
        }
        s->bup_flag |= BUP_SET;
    }

    while (elapsed) {
        int temp = MIN(elapsed, sizeof(s->silence));
        while (temp) {
            int copied = AUD_write(s->voice_po, s->silence, temp);
            if (!copied) {
                return;
            }
            temp -= copied;
            elapsed -= copied;
        }
    }
}

static int read_audio(Intel_ICH5_AC97_State *s, Intel_ICH5_AC97_BM_State *r,
                      int max, int *stop)
{
    uint8_t tmpbuf[4096];
    uint32_t addr = r->bd.addr;
    uint32_t temp = r->picb << 1;
    uint32_t nread = 0;
    int to_copy = 0;
    SWVoiceIn *voice = (r - s->bm_regs) == MC_INDEX ? s->voice_mc : s->voice_pi;

    temp = MIN(temp, max);

    if (!temp) {
        *stop = 1;
        return 0;
    }

    while (temp) {
        int acquired;
        to_copy = MIN(temp, sizeof(tmpbuf));
        acquired = AUD_read(voice, tmpbuf, to_copy);
        if (!acquired) {
            *stop = 1;
            break;
        }
        pci_dma_write(&s->dev, addr, tmpbuf, acquired);
        temp -= acquired;
        addr += acquired;
        nread += acquired;
    }

    r->bd.addr = addr;
    return nread;
}

static void transfer_audio(Intel_ICH5_AC97_State *s, int index, int elapsed)
{
    Intel_ICH5_AC97_BM_State *r = &s->bm_regs[index];
    int stop = 0;

    if (s->invalid_freq[index]) {
        return;
    }

    if (r->sr & SR_DCH) {
        if (r->cr & CR_RPBM) {
            switch (index) {
            case PO_INDEX:
                write_bup(s, elapsed);
                break;
            }
        }
        return;
    }

    while ((elapsed >> 1) && !stop) {
        int temp;

        if (!r->bd_valid) {
            fetch_bd(s, r);
        }

        if (!r->picb) {
            if (r->civ == r->lvi) {
                r->sr |= SR_DCH; /* CELV? */
                s->bup_flag = 0;
                break;
            }
            r->sr &= ~SR_CELV;
            r->civ = r->piv;
            r->piv = (r->piv + 1) % 32;
            fetch_bd(s, r);
            return;
        }

        switch (index) {
        case PO_INDEX:
            temp = write_audio(s, r, elapsed, &stop);
            elapsed -= temp;
            r->picb -= (temp >> 1);
            break;

        case PI_INDEX:
        case MC_INDEX:
            temp = read_audio(s, r, elapsed, &stop);
            elapsed -= temp;
            r->picb -= (temp >> 1);
            break;
        }

        if (!r->picb) {
            uint32_t new_sr = r->sr & ~SR_CELV;

            if (r->bd.ctl_len & BD_IOC) {
                new_sr |= SR_BCIS;
            }

            if (r->civ == r->lvi) {
                new_sr |= SR_LVBCI | SR_DCH | SR_CELV;
                stop = 1;
                s->bup_flag = (r->bd.ctl_len & BD_BUP) ? BUP_LAST : 0;
            } else {
                r->civ = r->piv;
                r->piv = (r->piv + 1) % 32;
                fetch_bd(s, r);
            }

            update_sr(s, r, new_sr);
        }
    }
}

static void pi_callback(void *opaque, int avail)
{
    transfer_audio(opaque, PI_INDEX, avail);
}

static void mc_callback(void *opaque, int avail)
{
    transfer_audio(opaque, MC_INDEX, avail);
}

static void po_callback(void *opaque, int free)
{
    transfer_audio(opaque, PO_INDEX, free);
}

static const VMStateDescription vmstate_intel_ich5_ac97_bm_regs = {
    .name = "intel_ich5_ac97_bm_regs",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(bdbar, Intel_ICH5_AC97_BM_State),
        VMSTATE_UINT8(civ, Intel_ICH5_AC97_BM_State),
        VMSTATE_UINT8(lvi, Intel_ICH5_AC97_BM_State),
        VMSTATE_UINT16(sr, Intel_ICH5_AC97_BM_State),
        VMSTATE_UINT16(picb, Intel_ICH5_AC97_BM_State),
        VMSTATE_UINT8(piv, Intel_ICH5_AC97_BM_State),
        VMSTATE_UINT8(cr, Intel_ICH5_AC97_BM_State),
        VMSTATE_UINT32(bd_valid, Intel_ICH5_AC97_BM_State),
        VMSTATE_UINT32(bd.addr, Intel_ICH5_AC97_BM_State),
        VMSTATE_UINT32(bd.ctl_len, Intel_ICH5_AC97_BM_State),
        VMSTATE_END_OF_LIST()
    }
};

static int intel_ich5_ac97_post_load(void *opaque, int version_id)
{
    uint8_t active[LAST_INDEX];
    Intel_ICH5_AC97_State *s = opaque;

    record_select(s, mixer_load(s, AC97_Record_Select));
    set_volume(s, AC97_Master_Volume_Mute, mixer_load(s, AC97_Master_Volume_Mute));
    set_volume(s, AC97_PCM_Out_Volume_Mute, mixer_load(s, AC97_PCM_Out_Volume_Mute));
    set_volume(s, AC97_Record_Gain_Mute, mixer_load(s, AC97_Record_Gain_Mute));

    active[PI_INDEX] = !!(s->bm_regs[PI_INDEX].cr & CR_RPBM);
    active[PO_INDEX] = !!(s->bm_regs[PO_INDEX].cr & CR_RPBM);
    active[MC_INDEX] = !!(s->bm_regs[MC_INDEX].cr & CR_RPBM);
    reset_voices(s, active);

    s->bup_flag = 0;
    s->last_samp = 0;
    return 0;
}

static const VMStateDescription vmstate_intel_ich5_ac97 = {
    .name = "intel_ich5_ac97",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = intel_ich5_ac97_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, Intel_ICH5_AC97_State),
        VMSTATE_UINT32(glob_cnt, Intel_ICH5_AC97_State),
        VMSTATE_UINT32(glob_sta, Intel_ICH5_AC97_State),
        VMSTATE_UINT32(cas, Intel_ICH5_AC97_State),
        VMSTATE_STRUCT_ARRAY(bm_regs, Intel_ICH5_AC97_State, 3, 1,
                             vmstate_intel_ich5_ac97_bm_regs, Intel_ICH5_AC97_BM_State),
        VMSTATE_BUFFER(mixer_data, Intel_ICH5_AC97_State),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t nam_read(void *opaque, hwaddr addr, unsigned size)
{
    if ((addr / size) > 512) {
        return -1;
    }
    uint16_t val = 0;
    switch (size) {
    case 1:
        return nam_readb(opaque, addr);
    case 2:
        val = nam_readw(opaque, addr);
        qemu_printf("Reading NAM 0x%02x, len %d 0x%04x\n", (int)addr, (int)size, (uint16_t)val);
        return val;
    case 4:
        return nam_readl(opaque, addr);
    default:
        return -1;
    }
}

static void nam_write(void *opaque, hwaddr addr, uint64_t val,
                      unsigned size)
{
    if ((addr / size) > 512) {
        return;
    }
    qemu_printf("Writing NAM 0x%02x, len %d 0x%04x\n", (int)addr, (int)size, (int)val);
    switch (size) {
    case 1:
        nam_writeb(opaque, addr, val);
        break;
    case 2:
        nam_writew(opaque, addr, val);
        break;
    case 4:
        nam_writel(opaque, addr, val);
        break;
    }
}

static const MemoryRegionOps ac97_io_nam_ops = {
    .read = nam_read,
    .write = nam_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint64_t nabm_read(void *opaque, hwaddr addr, unsigned size)
{
    if ((addr / size) > 256) {
        return -1;
    }
//    qemu_printf("Reading NABM 0x%02x, len %d\n", (int)addr, (int)size);
    switch (size) {
    case 1:
        return nabm_readb(opaque, addr);
    case 2:
        return nabm_readw(opaque, addr);
    case 4:
        return nabm_readl(opaque, addr);
    default:
        return -1;
    }
}

static void nabm_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    if ((addr / size) > 256) {
        return;
    }
//    qemu_printf("Writing NABM 0x%02x, len %d\n", (int)addr, (int)size);
    switch (size) {
    case 1:
        nabm_writeb(opaque, addr, val);
        break;
    case 2:
        nabm_writew(opaque, addr, val);
        break;
    case 4:
        nabm_writel(opaque, addr, val);
        break;
    }
}


static const MemoryRegionOps ac97_io_nabm_ops = {
    .read = nabm_read,
    .write = nabm_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};


static void intel_ich5_ac97_write(PCIDevice *dev, uint32_t address, uint32_t val, int len)
{
    pci_default_write_config(dev, address, val, len);

    for(int i = 0; i < len; i++){
        int ro_only = 0;

        uint8_t new_val = (val >> (i * 8)) & 0xff;

        switch(address + i)
        {
            case 0x04:
                new_val = new_val & 0x07;
            break;

            case 0x05:
                new_val = new_val & 0x04;
            break;

            case 0x07:
                new_val &= ~(new_val & 0x40);
                new_val |= 0x02;
            break;

            case 0x14:
                new_val = new_val & 0xc0;
            break;

            case 0x2c:
            case 0x2d:
            case 0x2e:
            case 0x2f:
                if(dev->config[address + i] != 0) /* After Subsystem ID is programmed, it cannot be reprogrammed again */
                    ro_only = 1;
            break;

            case 0x40:
                new_val = new_val & 0x0f;
            break;

            case 0x41:
                new_val = new_val & 0x01;
            break;

            case 0x54:
                new_val = new_val & 0x03;
            break;

            case 0x55:
                new_val &= ~(new_val & 0x80);
            break;

            case 0x11:
            case 0x15:
            case 0x19:
            case 0x1a:
            case 0x1b:
            case 0x1d:
            case 0x1e:
            case 0x1f:
            case 0x3c:
            break;

            default:
                ro_only = 1;
            break;
        }

        if(!ro_only) {
            dev->config[address + i] = new_val;
        }
    }

    switch(address)
    {
        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13:
            if(dev->config[0x04] & 1)
                qemu_printf("Intel ICH5 AC97: NAMBAR was updated to 0x%04x\n", (dev->config[0x13] << 24) | (dev->config[0x12] << 16) | (dev->config[0x11] << 8) | (dev->config[0x10] & 0xc0));
        break;

        case 0x14:
        case 0x15:
        case 0x16:
        case 0x17:
            if(dev->config[0x04] & 1)
                qemu_printf("Intel ICH5 AC97: NABMBAR was updated to 0x%04x\n", (dev->config[0x17] << 24) | (dev->config[0x16] << 16) | (dev->config[0x15] << 8) | (dev->config[0x14] & 0xc0));
        break;

        case 0x18:
        case 0x19:
        case 0x1a:
        case 0x1b:
            if(dev->config[0x04] & 2)
                qemu_printf("Intel ICH5 AC97: MMBAR was updated to 0x%04x\n", (dev->config[0x1b] << 24) | (dev->config[0x1a] << 16) | (dev->config[0x19] << 8) | (dev->config[0x18] & 0xc0));
        break;

        case 0x1c:
        case 0x1d:
        case 0x1e:
        case 0x1f:
            if(dev->config[0x04] & 2)
                qemu_printf("Intel ICH5 AC97: MBBAR was updated to 0x%04x\n", (dev->config[0x1f] << 24) | (dev->config[0x1e] << 16) | (dev->config[0x1d] << 8) | (dev->config[0x1c] & 0xc0));
        break;
    }
}

static void intel_ich5_ac97_reset(DeviceState *dev)
{
    Intel_ICH5_AC97_State *s = INTEL_ICH5_AC97(dev);

    reset_bm_regs(s, &s->bm_regs[0]);
    reset_bm_regs(s, &s->bm_regs[1]);
    reset_bm_regs(s, &s->bm_regs[2]);
    mixer_reset(s);
}

static void intel_ich5_ac97_realize(PCIDevice *dev, Error **errp)
{
    Intel_ICH5_AC97_State *s = INTEL_ICH5_AC97(dev);

    if (!AUD_register_card("intel-ich5-ac97", &s->card, errp)) { /* Form the sound device */
        return;
    }

    dev->config[0x06] = 0x80;
    dev->config[0x07] = 0x02;
    dev->config[0x10] = 0x01;
    dev->config[0x14] = 0x01;
    dev->config[0x18] = 0x01;
    dev->config[0x1c] = 0x01;
    dev->config[0x34] = 0x50;
    dev->config[0x3d] = 0x02;
    dev->config[0x40] = 0x09;
    dev->config[0x50] = 0x01;
    dev->config[0x52] = 0xc2;
    dev->config[0x53] = 0xc9;

    /* Native Audio Mixer */
    memory_region_init_io(&s->io_nam, OBJECT(s), &ac97_io_nam_ops, s, "nam", 256);
    pci_register_bar(&s->dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->io_nam);

    /* Native Audio Bus Mastering */
    memory_region_init_io(&s->io_nabm, OBJECT(s), &ac97_io_nabm_ops, s, "nabm", 64);
    pci_register_bar(&s->dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->io_nabm);

    /* Mixer Base */
    memory_region_init_io(&s->io_mm, OBJECT(s), &ac97_io_nam_ops, s, "mm", 512);
    pci_register_bar(&s->dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->io_mm);

    /* Bus Master Base */
    memory_region_init_io(&s->io_mb, OBJECT(s), &ac97_io_nabm_ops, s, "mb", 256);
    pci_register_bar(&s->dev, 3, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->io_mb);

    intel_ich5_ac97_reset(DEVICE(s));
}

static void intel_ich5_ac97_exit(PCIDevice *dev)
{
    Intel_ICH5_AC97_State *s = INTEL_ICH5_AC97(dev);

    AUD_close_in(&s->card, s->voice_pi);
    AUD_close_out(&s->card, s->voice_po);
    AUD_close_in(&s->card, s->voice_mc);
    AUD_remove_card(&s->card);
}

static Property intel_ich5_ac97_properties[] = {
    DEFINE_AUDIO_PROPERTIES(Intel_ICH5_AC97_State, card),
    DEFINE_PROP_END_OF_LIST(),
};

static void intel_ich5_ac97_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = intel_ich5_ac97_realize;
    k->exit = intel_ich5_ac97_exit;
    k->config_write = intel_ich5_ac97_write;
    k->config_read = pci_default_read_config;
    k->vendor_id = PCI_VENDOR_ID_INTEL;
    k->device_id = PCI_DEVICE_ID_INTEL_ICH5_AC97;
    k->revision = 0x02;
    k->class_id = PCI_CLASS_MULTIMEDIA_AUDIO;
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
    dc->desc = "Intel ICH5 AC97";
    dc->vmsd = &vmstate_intel_ich5_ac97;
    device_class_set_props(dc, intel_ich5_ac97_properties);
    dc->reset = intel_ich5_ac97_reset;
    dc->user_creatable = false; /* Intel ICH5 AC'97 is onboard */
}

static const TypeInfo intel_ich5_ac97_info = {
    .name          = TYPE_INTEL_ICH5_AC97,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Intel_ICH5_AC97_State),
    .class_init    = intel_ich5_ac97_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void intel_ich5_ac97_register_types(void)
{
    type_register_static(&intel_ich5_ac97_info);
    deprecated_register_soundhw("intel-ich5-ac97", "Intel ICH5 AC97", 0, TYPE_INTEL_ICH5_AC97);
}

type_init(intel_ich5_ac97_register_types)
