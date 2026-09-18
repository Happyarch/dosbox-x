/*
 *  Copyright (C) 2026  The DOSBox Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/* Pro Audio Spectrum 16 emulation (stage 2: register file + MV508 mixer).
 *
 * Knowledge sources: the authors of 86Box's snd_pas16.c and the Linux
 * kernel OSS PAS16 documentation. Hardware behavior is re-expressed here
 * in this fork's house style; no code is copied from those sources.
 *
 * Stage 2 models the base-relative register file and the MV508 mixer
 * state. There is no sample flow yet: DMA/IRQ/PIT wiring, OPL routing
 * and MPU routing arrive in stage 3, so every side effect that needs
 * those APIs is stored as state now and marked STAGE3 below.
 */

#include <string.h>
#include "dosbox.h"
#include "inout.h"
#include "logging.h"
#include "mixer.h"
#include "pic.h"
#include "setup.h"
#include "control.h"

/* Stage 2: MV508 attenuation curves, re-expressed as plain data.
 * Behavior per 86Box snd_pas16.c mixer tables (2 dB steps for channel
 * volumes, 1 dB steps for master); values are the hardware dB curve. */
static const double pas_vol_2db[32] = {
	25.0,    32.0,    41.0,    51.0,    65.0,    82.0,   103.0,   130.0,
	164.0,   206.0,   260.0,   327.0,   412.0,   519.0,   653.0,   822.0,
	1036.0,  1304.0,  1641.0,  2067.0,  2602.0,  3276.0,  4125.0,  5192.0,
	6537.0,  8230.0, 10362.0, 13044.0, 16422.0, 20674.0, 26027.0, 32767.0
};

static const double pas_master_1db[64] = {
	18.0,    25.0,    29.0,    32.0,    36.0,    41.0,    46.0,    51.0,
	58.0,    65.0,    73.0,    82.0,    92.0,   103.0,   116.0,   130.0,
	146.0,   164.0,   184.0,   206.0,   231.0,   260.0,   292.0,   327.0,
	367.0,   412.0,   462.0,   519.0,   582.0,   653.0,   733.0,   822.0,
	923.0,  1036.0,  1162.0,  1304.0,  1463.0,  1641.0,  1842.0,  2067.0,
	2319.0,  2602.0,  2920.0,  3276.0,  3676.0,  4125.0,  4628.0,  5192.0,
	5826.0,  6537.0,  7335.0,  8230.0,  9234.0, 10362.0, 11626.0, 13044.0,
	14636.0, 16422.0, 18426.0, 20674.0, 23197.0, 26027.0, 29204.0, 32767.0
};

/* Stage 2: register bit meanings, re-expressed from the PAS16 behavior
 * in 86Box snd_pas16.c (IRQ/PCM/filter bit positions). */
#define PAS_IRQ_SAMP	0x04
#define PAS_IRQ_PCM	0x08
#define PAS_IRQ_MIDI	0x10

#define PAS_PCM_MONO	0x20
#define PAS_PCM_ENABLE	0x40
#define PAS_PCM_DMA	0x80

#define PAS_FILT_MUTE	0x20

/* Stage 2: MV508 index/data protocol bits at base+0x0403, re-expressed
 * from 86Box snd_pas16.c PAS16 mixer handling. */
#define PAS_MIX_ADDR	0x80
#define PAS_MIX_CHAN	0x60
#define PAS_MIX_LEFT	0x20
#define PAS_MIX_RIGHT	0x40
#define PAS_MIX_SELMIX	0x10
/* NOTE: PAS_MIX_INBANK shares its value with PAS_MIX_LEFT by hardware
 * design; the two apply to different operands (data byte vs index). */
#define PAS_MIX_INBANK	0x20

/* Stage 2: DMA select map from 86Box snd_pas16.c, re-expressed. */
static const Bitu pas_dma_map[8] = {4, 1, 2, 3, 0, 5, 6, 7};

static struct {
	bool enabled;
	Bitu basePort;
	Bitu irq;
	Bitu dma;
	Bitu rate;
	/* Stage 2 register file (PAS16 read/write behavior per 86Box
	 * snd_pas16.c; DMA/IRQ/PIT effects deferred to stage 3). */
	uint8_t mixerCtrl;	/* B88: mixer-control latch */
	uint8_t irqStatus;	/* B89: pending IRQs, clear-on-write-1 */
	uint8_t irqMask;		/* B8B: enabled IRQs (low 5 bits) */
	uint8_t filterCtrl;	/* B8A: mute + filter select */
	bool filterOn;		/* low 5 bits hit a known cutoff */
	Bitu filterCutoff;	/* selected cutoff in Hz, 0 when off */
	uint8_t pcmCtrl;		/* F8A: mono/stereo + enable bits */
	uint8_t stereoHalf;	/* stereo L/R toggle (DMA engine, stage 3) */
	bool dmaFlip;		/* 8-bit sample flip-flop (DMA engine, stage 3) */
	uint8_t waitStates;	/* BC00 */
	uint8_t prescale;		/* BC02: PIT prescaler (stage 3 clock) */
	uint8_t sysConf[4];	/* 8000-8003: system config nibbles */
	uint8_t ioConf[4];	/* F000-F003: IO/DMA/IRQ config */
	uint8_t compat;		/* F400: SB/MPU compat enables (side effects stage 3) */
	uint8_t compatBase;	/* F401: SB compat base nibbles */
	uint8_t sbIrqDma;		/* F802: SB IRQ/DMA select (side effects stage 3) */
	uint8_t midiCtrl;		/* 1401/1403: UART control */
	uint8_t midiStat;		/* 1800: UART status */
	uint8_t midiFifo;		/* 1801: FIFO status */
	uint8_t midiData;		/* 1402/1802: UART data byte */
	bool midiUartIn;
	bool midiUartOut;
	uint8_t midiQueue[256];	/* receive queue, empty until stage 3 MIDI in */
	uint8_t midiR;
	uint8_t midiW;
	/* Stage 2 MV508 mixer (recalc/reset behavior per 86Box snd_pas16.c). */
	uint8_t mixIndex;		/* 0403 index latch */
	uint8_t mixRegs[3][128];	/* bank 0: channel vols, bank 1: input mix, bank 2: master/tone */
	double voiceGainL[8];
	double voiceGainR[8];
	double masterL;
	double masterR;
	uint8_t bass;		/* 4-bit tone, 6 is flat */
	uint8_t treble;
	MixerChannel *chan;
} pas;

/* Stage 2: recompute MV508 gains from the register image.
 * Behavior per 86Box snd_pas16.c mv508 recalc; re-expressed. */
static void PAS_MixerRecalc(void) {
	for (Bitu s = 0; s < 8; s++) {
		pas.voiceGainL[s] = pas_vol_2db[pas.mixRegs[0][0x30 + s] & 0x1f] / 32767.0;
		pas.voiceGainR[s] = pas_vol_2db[pas.mixRegs[0][0x50 + s] & 0x1f] / 32767.0;
	}
	pas.masterL = pas_master_1db[pas.mixRegs[2][0x21] & 0x3f] / 32767.0;
	pas.masterR = pas_master_1db[pas.mixRegs[2][0x41] & 0x3f] / 32767.0;
	pas.bass = pas.mixRegs[2][0x23] & 0x0f;
	pas.treble = pas.mixRegs[2][0x24] & 0x0f;
	if (pas.chan != NULL)
		pas.chan->SetVolume((float)pas.masterL, (float)pas.masterR);
}

/* Stage 2: MV508 power-on defaults.
 * Behavior per 86Box snd_pas16.c MV508 reset (Linux-driver values). */
static void PAS_MixerReset(void) {
	static const uint8_t defaults[8] = {0x18, 0x1f, 0x17, 0x17, 0x17, 0x17, 0x0f, 0x17};
	for (Bitu s = 0; s < 8; s++) {
		pas.mixRegs[0][0x30 + s] = defaults[s];
		pas.mixRegs[0][0x50 + s] = defaults[s];
	}
	pas.mixRegs[2][0x21] = 0x1f;
	pas.mixRegs[2][0x41] = 0x1f;
	pas.mixRegs[2][0x23] = 0x06;
	pas.mixRegs[2][0x24] = 0x06;
	PAS_MixerRecalc();
}

/* Stage 2: PCM engine state reset (no PIC/DMA calls yet; stage 3).
 * Behavior per 86Box snd_pas16.c PCM reset; re-expressed. */
static void PAS_ResetPCM(void) {
	pas.pcmCtrl = 0x00;
	pas.stereoHalf = 0;
	pas.irqStatus &= 0xd7;
	/* STAGE3: clear the guest IRQ line if nothing is pending. */
}

/* Stage 2: register-file reset, minus PIT/PIC/IO effects (stage 3).
 * Behavior per 86Box snd_pas16.c register reset; re-expressed. */
static void PAS_ResetRegs(void) {
	/* STAGE3: clear the guest IRQ line here. */
	pas.sysConf[0] &= 0xfd;
	pas.sysConf[1] = 0x00;
	pas.sysConf[2] = 0x00;
	pas.prescale = 0x00;
	/* STAGE3: restore the PIT constant and gate counters 0/1 off. */
	pas.filterCtrl = 0x00;
	pas.filterOn = false;
	pas.filterCutoff = 0;
	PAS_ResetPCM();
	pas.dmaFlip = false;
	pas.irqMask = 0x00;
	pas.irqStatus = 0x00;
	PAS_MixerReset();
}

static int PAS_IrqConvert(Bitu val) {
	/* Stage 2: nibble-to-IRQ map per 86Box snd_pas16.c; re-expressed. */
	int irq = (int)(val & 0x0f);
	if (irq == 0) return -1;
	if (irq <= 6) return irq + 1;
	if (irq < 0x0b) return irq + 3;
	return irq + 4;
}

/* Stage 2: MV508 index/data write at base+0x0403.
 * Behavior per 86Box snd_pas16.c PAS16 0x0403 handling; re-expressed. */
static void PAS_MixerWrite(Bitu val) {
	if (val & PAS_MIX_ADDR) {
		pas.mixIndex = (uint8_t)(val & ~PAS_MIX_ADDR);
		return;
	}
	Bitu bank;
	Bitu mask;
	if (pas.mixIndex & PAS_MIX_SELMIX) {
		bank = (val & PAS_MIX_INBANK) ? 1 : 0;
		mask = 0x1f;
	} else {
		bank = 2;
		mask = 0x3f;
	}
	if (pas.mixIndex & PAS_MIX_CHAN)
		pas.mixRegs[bank][pas.mixIndex & 0x7f] = (uint8_t)(val & mask);
	else {
		pas.mixRegs[bank][(pas.mixIndex | PAS_MIX_LEFT) & 0x7f] = (uint8_t)(val & mask);
		pas.mixRegs[bank][(pas.mixIndex | PAS_MIX_RIGHT) & 0x7f] = (uint8_t)(val & mask);
	}
	PAS_MixerRecalc();
}

static Bitu pas_read(Bitu port, Bitu iolen) {
	(void)iolen;//UNUSED
	Bitu off = port - pas.basePort;
	uint8_t ret = 0xff;
	switch (off) {
	case 0x0400: case 0x0401: case 0x0402: case 0x0403:
		/* Stage 2: MV508 has no readable registers; behavior per
		 * 86Box snd_pas16.c (no 0x04xx read case). */
		break;
	case 0x0800:
		/* Stage 2: B88 mixer-control latch; behavior per 86Box
		 * snd_pas16.c PAS16 read path; re-expressed. */
		ret = pas.mixerCtrl;
		break;
	case 0x0801:
		/* Stage 2: B89 IRQ status, bit 5 masked; behavior per
		 * 86Box snd_pas16.c; re-expressed. */
		ret = pas.irqStatus & 0xdf;
		break;
	case 0x0802:
		ret = pas.filterCtrl;
		break;
	case 0x0803:
		/* Stage 2: B8B IRQ mask with read-only board-ID bit set;
		 * behavior per 86Box snd_pas16.c; re-expressed. */
		ret = pas.irqMask | 0x20;
		break;
	case 0x0c00: case 0x0c01:
		/* Stage 2: F88/F89 PCM data has no holding register;
		 * behavior per 86Box snd_pas16.c (no read case). */
		break;
	case 0x0c02:
		ret = pas.pcmCtrl;
		break;
	case 0x1401: case 0x1403:
		ret = pas.midiCtrl;
		break;
	case 0x1402: case 0x1802:
		/* Stage 2: UART data read; behavior per 86Box snd_pas16.c
		 * (probe echo + empty queue); re-expressed. No MIDI flow yet. */
		ret = 0;
		if (pas.midiUartIn) {
			if ((pas.midiData == 0xaa) && (pas.midiCtrl & 0x04))
				ret = pas.midiData;
			else {
				ret = pas.midiQueue[pas.midiR];
				if (pas.midiR != pas.midiW) {
					pas.midiR++;
					pas.midiR &= 0xff;
				}
			}
			pas.midiStat &= ~0x04;
			/* STAGE3: re-evaluate the MIDI IRQ here. */
		}
		break;
	case 0x1800:
		ret = pas.midiStat;
		break;
	case 0x1801:
		ret = pas.midiFifo;
		break;
	case 0x2401:
		/* Stage 2: board revision reads 0; behavior per 86Box
		 * snd_pas16.c code path (its header names 2789 for the old
		 * PAS map instead); code followed, see report. */
		ret = 0x00;
		break;
	case 0x8000: case 0x8001: case 0x8002: case 0x8003:
		ret = pas.sysConf[off - 0x8000];
		break;
	case 0xbc00:
		ret = pas.waitStates;
		break;
	case 0xbc02:
		ret = pas.prescale;
		break;
	case 0xec03:
		/* Stage 2: PAS16 operation mode (stereo FM + 16-bit +
		 * CD-ROM select bits); behavior per 86Box snd_pas16.c. */
		ret = 0x0f;
		break;
	case 0xf000: case 0xf001: case 0xf002: case 0xf003:
		ret = pas.ioConf[off - 0xf000];
		break;
	case 0xf400:
		/* Stage 2: compat enables; live SB/MPU status bits arrive
		 * with the stage 3 SB/MPU wiring. */
		ret = pas.compat & 0xf3;
		break;
	case 0xf401:
		ret = pas.compatBase;
		break;
	case 0xf802:
		ret = pas.sbIrqDma;
		break;
	case 0xfc00:
		/* Stage 2: board model for PAS16; behavior per 86Box
		 * snd_pas16.c code path (its header names FF88 for the old
		 * PAS map instead); code followed, see report. */
		ret = 0x0c;
		break;
	case 0xfc03:
		/* Stage 2: AT bus, XT/AT timing + PAS16 flag; behavior
		 * per 86Box snd_pas16.c; re-expressed. */
		ret = 0x31;
		break;
	default:
		/* Unmapped offsets in the decoded windows read back 0xFF,
		 * matching both the oracle default and this fork's open-bus
		 * convention for uninstalled ports. */
		break;
	}
	return ret;
}

static void pas_write(Bitu port, Bitu val, Bitu iolen) {
	(void)iolen;//UNUSED
	Bitu off = port - pas.basePort;
	uint8_t v = (uint8_t)val;
	switch (off) {
	case 0x0400: case 0x0401: case 0x0402:
		/* Stage 2: ignored; behavior per 86Box snd_pas16.c. */
		break;
	case 0x0403:
		PAS_MixerWrite(v);
		break;
	case 0x0800:
		/* Stage 2: B88 on PAS16 latches and resets the PCM engine
		 * only when bit 0 is clear; bit-0-set writes are ignored.
		 * Behavior per 86Box snd_pas16.c; re-expressed. The LMC
		 * serial-mixer machine is Plus-only and not modeled here. */
		if (!(v & 0x01)) {
			pas.mixerCtrl = v;
			PAS_ResetPCM();
		}
		break;
	case 0x0801:
		/* Stage 2: B89 clear-on-write-1; behavior per 86Box
		 * snd_pas16.c; re-expressed (PIC clear is stage 3). */
		pas.irqStatus &= ~v;
		break;
	case 0x0802: {
		/* Stage 2: B8A filter control; behavior per 86Box snd_pas16.c
		 * PAS16 path; re-expressed. PIT gate effects and FIR
		 * coefficient rebuild are stage 3; mute/IRQ edge notes below. */
		/* STAGE3: gate PIT counters (bit 7 -> ctr 1, bit 6 -> ctr 0). */
		pas.stereoHalf = 0;
		pas.dmaFlip = false;
		/* NOTE: the rising-mute IRQ clear in the oracle lives on the
		 * old-PAS path only; the PAS16 path stores the byte as-is. */
		pas.filterCtrl = v;
		pas.filterOn = false;
		pas.filterCutoff = 0;
		switch (v & 0x1f) {
		case 0x01: pas.filterOn = true; pas.filterCutoff = 17897; break;
		case 0x02: pas.filterOn = true; pas.filterCutoff = 15909; break;
		case 0x04: pas.filterOn = true; pas.filterCutoff = 2982; break;
		case 0x09: pas.filterOn = true; pas.filterCutoff = 11931; break;
		case 0x11: pas.filterOn = true; pas.filterCutoff = 8948; break;
		case 0x19: pas.filterOn = true; pas.filterCutoff = 5965; break;
		default: break;
		}
		break;
	}
	case 0x0803:
		/* Stage 2: B8B IRQ mask; behavior per 86Box snd_pas16.c;
		 * re-expressed (PIC clear is stage 3). */
		pas.irqMask = v & 0x1f;
		pas.irqStatus &= ((v & 0x1f) | 0xe0);
		break;
	case 0x0c00: case 0x0c01:
		/* Stage 2: F88/F89 writes carry no state; behavior per
		 * 86Box snd_pas16.c (flush only, no sample path yet). */
		break;
	case 0x0c02:
		/* Stage 2: F8A PCM control with enable-edge reset; behavior
		 * per 86Box snd_pas16.c code bit positions (0x20/0x40/0x80);
		 * re-expressed. See report on the header/code mismatch. */
		if ((v & PAS_PCM_ENABLE) && !(pas.pcmCtrl & PAS_PCM_ENABLE)) {
			pas.stereoHalf = 0;
			pas.irqStatus &= 0xd7;
			pas.dmaFlip = false;
		}
		pas.pcmCtrl = v;
		break;
	case 0x1401: case 0x1403:
		/* Stage 2: UART control + mode flags; behavior per 86Box
		 * snd_pas16.c; re-expressed (MIDI routing is stage 3). */
		pas.midiCtrl = v;
		if ((v & 0x60) == 0x60) {
			pas.midiUartOut = false;
			pas.midiUartIn = false;
		} else if ((v & 0x1c) == 0x04)
			pas.midiUartIn = true;
		else
			pas.midiUartOut = true;
		/* STAGE3: re-evaluate the MIDI IRQ here. */
		break;
	case 0x1402: case 0x1802:
		/* Stage 2: UART data store; transmit is stage 3. */
		pas.midiData = v;
		break;
	case 0x1800:
		pas.midiStat = v;
		/* STAGE3: re-evaluate the MIDI IRQ here. */
		break;
	case 0x1801:
		pas.midiFifo = v;
		break;
	case 0x8000:
		/* Stage 2: system config 1 with reset-on-rise of the top
		 * bits; behavior per 86Box snd_pas16.c; re-expressed
		 * (PIT/IO re-hookup is stage 3, base is conf-fixed). */
		if ((v & 0xc0) && !(pas.sysConf[0] & 0xc0)) {
			PAS_ResetRegs();
			pas.sysConf[0] = 0x00;
		} else
			pas.sysConf[0] = v;
		/* STAGE3: update the PIT clock select here. */
		break;
	case 0x8001:
		pas.sysConf[1] = v;
		break;
	case 0x8002:
		pas.sysConf[2] = v;
		/* STAGE3: update the PIT clock select here. */
		break;
	case 0x8003:
		pas.sysConf[3] = v;
		/* STAGE3: SCSI IRQ handling lives outside this stage. */
		break;
	case 0xbc00:
		pas.waitStates = v;
		break;
	case 0xbc02:
		pas.prescale = v;
		/* STAGE3: update the PIT clock select here. */
		break;
	case 0xf000:
		/* Stage 2: joystick-enable bit stored; gameport remap is
		 * out of this stage. */
		pas.ioConf[0] = v;
		break;
	case 0xf001:
		pas.ioConf[1] = v;
		pas.dma = pas_dma_map[v & 0x07];
		/* STAGE3: update the PIT clock select here. */
		break;
	case 0xf002:
		/* Stage 2: PAS IRQ select stored via the nibble map; the
		 * SCSI high nibble is out of scope (no SCSI side). */
		pas.ioConf[2] = v;
		/* STAGE3: clear the old guest IRQ line here. */
		pas.irq = (Bitu)PAS_IrqConvert(v);
		break;
	case 0xf003:
		pas.ioConf[3] = v;
		break;
	case 0xf400:
		/* Stage 2: compat enables stored masked; SB/MPU address
		 * effects arrive with the stage 3 wiring. */
		pas.compat = v & 0xf3;
		break;
	case 0xf401:
		pas.compatBase = v;
		/* STAGE3: derive the SB/MPU base addresses here. */
		break;
	case 0xf802:
		/* Stage 2: SB IRQ/DMA select stored; effect is stage 3. */
		pas.sbIrqDma = v;
		break;
	default:
		/* Unmapped offsets ignore writes, matching the oracle
		 * default and this fork's uninstalled-port convention. */
		break;
	}
}

/* Stage 2: silent mixer callback; the sample pump arrives in stage 3. */
static void PAS_Callback(Bitu len) {
	if (!len || pas.chan == NULL) return;
	pas.chan->AddSilence();
}

class PAS: public Module_base {
public:
	IO_ReadHandleObject ReadHandler[16];
	IO_WriteHandleObject WriteHandler[16];
	MixerObject MixerChan;
	PAS(Section* configuration):Module_base(configuration) {
		Section_prop * section=static_cast<Section_prop *>(configuration);
		if(!section->Get_bool("pas")||control->opt_silent) return;
		pas.enabled = true;
		pas.basePort = (unsigned int)section->Get_hex("pasbase");
		pas.irq = (unsigned int)section->Get_int("pasirq");
		pas.dma = (unsigned int)section->Get_int("pasdma");
		pas.rate = (unsigned int)section->Get_int("pasrate");

		LOG_MSG("PAS:Initializing Pro Audio Spectrum 16 emulation...");
		memset(pas.mixRegs, 0, sizeof(pas.mixRegs));
		pas.mixIndex = 0;
		pas.chan = NULL;
		pas.midiR = 0;
		pas.midiW = 0;
		PAS_ResetRegs();
		pas.mixerCtrl = 0x00;
		pas.midiCtrl = 0x00;
		pas.midiStat = 0x00;
		pas.midiFifo = 0x00;
		pas.midiData = 0x00;
		pas.midiUartIn = false;
		pas.midiUartOut = false;
		pas.compat = 0x00;
		pas.compatBase = 0x00;
		pas.sbIrqDma = 0x00;
		pas.waitStates = 0x00;

		Bitu h = 0;
		ReadHandler[h].Install(pas.basePort + 0x0400, pas_read, IO_MB, 4); h++;
		WriteHandler[0].Install(pas.basePort + 0x0400, pas_write, IO_MB, 4);
		ReadHandler[h].Install(pas.basePort + 0x0800, pas_read, IO_MB, 4); h++;
		WriteHandler[1].Install(pas.basePort + 0x0800, pas_write, IO_MB, 4);
		ReadHandler[h].Install(pas.basePort + 0x0c00, pas_read, IO_MB, 4); h++;
		WriteHandler[2].Install(pas.basePort + 0x0c00, pas_write, IO_MB, 4);
		ReadHandler[h].Install(pas.basePort + 0x1400, pas_read, IO_MB, 4); h++;
		WriteHandler[3].Install(pas.basePort + 0x1400, pas_write, IO_MB, 4);
		ReadHandler[h].Install(pas.basePort + 0x1800, pas_read, IO_MB, 4); h++;
		WriteHandler[4].Install(pas.basePort + 0x1800, pas_write, IO_MB, 4);
		ReadHandler[h].Install(pas.basePort + 0x2401, pas_read, IO_MB); h++;
		WriteHandler[5].Install(pas.basePort + 0x2401, pas_write, IO_MB);
		ReadHandler[h].Install(pas.basePort + 0x8000, pas_read, IO_MB, 4); h++;
		WriteHandler[6].Install(pas.basePort + 0x8000, pas_write, IO_MB, 4);
		ReadHandler[h].Install(pas.basePort + 0xbc00, pas_read, IO_MB); h++;
		WriteHandler[7].Install(pas.basePort + 0xbc00, pas_write, IO_MB);
		ReadHandler[h].Install(pas.basePort + 0xbc02, pas_read, IO_MB); h++;
		WriteHandler[8].Install(pas.basePort + 0xbc02, pas_write, IO_MB);
		ReadHandler[h].Install(pas.basePort + 0xec03, pas_read, IO_MB); h++;
		WriteHandler[9].Install(pas.basePort + 0xec03, pas_write, IO_MB);
		ReadHandler[h].Install(pas.basePort + 0xf000, pas_read, IO_MB, 4); h++;
		WriteHandler[10].Install(pas.basePort + 0xf000, pas_write, IO_MB, 4);
		ReadHandler[h].Install(pas.basePort + 0xf400, pas_read, IO_MB, 2); h++;
		WriteHandler[11].Install(pas.basePort + 0xf400, pas_write, IO_MB, 2);
		ReadHandler[h].Install(pas.basePort + 0xf802, pas_read, IO_MB); h++;
		WriteHandler[12].Install(pas.basePort + 0xf802, pas_write, IO_MB);
		ReadHandler[h].Install(pas.basePort + 0xfc00, pas_read, IO_MB); h++;
		WriteHandler[13].Install(pas.basePort + 0xfc00, pas_write, IO_MB);
		ReadHandler[h].Install(pas.basePort + 0xfc03, pas_read, IO_MB); h++;
		WriteHandler[14].Install(pas.basePort + 0xfc03, pas_write, IO_MB);
		/* NOTE: base+0x1000 (1388-138B PIT rate/count) is deliberately
		 * not installed here; the oracle delegates that window to its
		 * PIT device and stage 3 attaches this fork's PIT there.
		 * OPL aliases (base+0x0000) stay with the fork's live OPL
		 * until the stage 3 OPL routing lands. */

		pas.chan = MixerChan.Install(&PAS_Callback, pas.rate, "PAS");
		pas.chan->Enable(false);

		LOG_MSG("PAS:... finished.");
	}
};

static PAS* test = NULL;

static void PAS_ShutDown(Section* sec){
    (void)sec;//UNUSED
    if (test != NULL) {
        delete test;
        test = NULL;
    }
}

void PAS_OnReset(Section *sec) {
    (void)sec;//UNUSED
	if (test == NULL && !IS_PC98_ARCH) {
		LOG(LOG_MISC,LOG_DEBUG)("Allocating PAS emulation");
		test = new PAS(control->GetSection("pas"));
	}
}

void PAS_Init(Section* sec) {
    (void)sec;//UNUSED
	LOG(LOG_MISC,LOG_DEBUG)("Initializing PAS emulation");

	AddExitFunction(AddExitFunctionFuncPair(PAS_ShutDown),true);
	AddVMEventFunction(VM_EVENT_RESET,AddVMEventFunctionFuncPair(PAS_OnReset));
}
