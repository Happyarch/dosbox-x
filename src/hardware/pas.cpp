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

/* Pro Audio Spectrum 16 emulation (stage 4: static bring-up).
 *
 * Knowledge sources: the authors of 86Box's snd_pas16.c and the Linux
 * kernel OSS PAS16 documentation. Hardware behavior is re-expressed here
 * in this fork's house style; no code is copied from those sources.
 *
 * Stage 2 modeled the base-relative register file and the MV508 mixer
 * state. Stage 3 wired the side effects: the guest IRQ line via the
 * fork PIC_* calls, sample flow via the fork DMA channel reads driven
 * by a re-expressed 1388-138B rate/count window, and the card-side OPL
 * and MPU routing decisions. Stage 4 (static) instantiates the card
 * from sdlmain beside INNOVA_Init and validates the bring-up sequence
 * against the oracle without executing the emulator. Every behavioral
 * block is re-derived; points that need cross-file APIs or hardware
 * measurement stay marked STAGE4 below.
 */

#include <string.h>
#include "dosbox.h"
#include "dma.h"
#include "inout.h"
#include "logging.h"
#include "mixer.h"
#include "pic.h"
#include "setup.h"
#include "control.h"

/* Stage 3: host MIDI transmit, following the house pattern of a local
 * forward declaration (as in the fork MPU handler). */
void MIDI_RawOutByte(uint8_t data);

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

/* Stage 4: card PIT clock for the 1388-138B rate/count window,
 * re-expressed from the hardware note the oracle carries in its own
 * header (card clocked at 1193180 Hz). The prescaler register divides
 * that clock when native mode and a nonzero prescaler select it. The
 * exact scaled curve is undetermined from the available sources: the
 * oracle delegates it to its own PIT module outside the reference
 * file, and the kernel doc carries no clock data, so the divide model
 * stands pending hardware measurement. */
static const double PAS_PIT_CLOCK = 1193180.0;

/* Stage 3: sample-pump chunking, following the house precedent of
 * pumping several samples per PIC event instead of one event per
 * sample. 5 ms events cover rates up to about 51 kHz. */
#define PAS_PUMP_MAX 256
#define PAS_PUMP_INTERVAL_MS 5.0

static struct {
	bool enabled;
	Bitu basePort;
	/* Stage 3: signed IRQ line. Nibble 0 programs no line, which the
	 * oracle models as -1; stage 2 stored the converter output into an
	 * unsigned and wrapped, so every PIC call must have guarded it.
	 * Holding -1 for none resolves that flag. */
	int irq;
	Bitu dma;
	Bitu rate;
	/* Stage 3 register file (PAS16 behavior re-expressed; DMA/IRQ/PIT
	 * side effects wired below). */
	uint8_t mixerCtrl;	/* B88: mixer-control latch */
	uint8_t irqStatus;	/* B89: pending IRQs, clear-on-write-1 */
	uint8_t irqMask;		/* B8B: enabled IRQs (low 5 bits) */
	uint8_t filterCtrl;	/* B8A: mute + filter select */
	bool filterOn;		/* low 5 bits hit a known cutoff */
	Bitu filterCutoff;	/* selected cutoff in Hz, 0 when off */
	uint8_t pcmCtrl;		/* F8A: mono/stereo + enable bits */
	uint8_t stereoHalf;	/* stereo L/R toggle (DMA engine) */
	bool dmaFlip;		/* 8-bit sample flip-flop (DMA engine) */
	uint8_t waitStates;	/* BC00 */
	uint8_t prescale;		/* BC02: PIT prescaler (stage 3 clock) */
	uint8_t sysConf[4];	/* 8000-8003: system config nibbles */
	uint8_t ioConf[4];	/* F000-F003: IO/DMA/IRQ config */
	uint8_t compat;		/* F400: SB/MPU compat enables (state only) */
	uint8_t compatBase;	/* F401: SB/MPU compat base nibbles */
	uint8_t sbIrqDma;		/* F802: SB IRQ/DMA select (stored, no SB side) */
	uint8_t midiCtrl;		/* 1401/1403: UART control */
	uint8_t midiStat;		/* 1800: UART status */
	uint8_t midiFifo;		/* 1801: FIFO status */
	uint8_t midiData;		/* 1402/1802: UART data byte */
	bool midiUartIn;
	bool midiUartOut;
	uint8_t midiQueue[256];	/* receive queue (host MIDI-in is STAGE4) */
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
	/* Stage 3: DMA/IRQ/PIT plumbing state, re-expressed from the
	 * oracle PCM/DMA engine behavior. */
	DmaChannel *dmachan;	/* attached fork channel, NULL when none */
	bool dmaMasked;		/* guest masked the channel or TC fired */
	bool dmaTC;		/* terminal count seen: reads go dry */
	uint16_t dmaHold;	/* half-word hold for 8-bit on wide channel */
	bool sampDry;		/* current sample instant found no data */
	int16_t pcmL;		/* held outputs (compat stereo keeps a side) */
	int16_t pcmR;
	uint16_t pitCount[2];	/* 1388/1389 divisor image, counters 0/1 */
	uint8_t pitMode[2];	/* 138B control mode field per counter */
	uint8_t pitAccess[2];	/* 138B access field per counter */
	uint8_t pitNeed[2];	/* divisor byte assembly: 0 wants LSB */
	bool pitSet[2];		/* divisor fully programmed at least once */
	bool pitGate[2];	/* B8A bit 6 -> counter 0, bit 7 -> counter 1 */
	bool pitReadHi[2];	/* read interleave tracker */
	int32_t ctr1pos;	/* counter-1 cascade countdown (PCM IRQ) */
	double pitClock;	/* effective card clock after prescaler */
	bool pumpOn;		/* sample pump event is armed */
	Bitu sbBase;		/* F401-derived SB compat base (state only) */
	Bitu mpuBase;		/* F401-derived MPU compat base (state only) */
	bool sbOn;		/* F400 SB compat enable (state only) */
	bool mpuOn;		/* F400 MPU compat enable (state only) */
} pas;

static void PAS_PumpUpdate(void);
static void PAS_PumpTick(Bitu val);

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

/* Stage 3: guest IRQ line, counter-piece of the oracle raise/clear
 * pair. The line follows the pending low-5 status bits; every entry
 * point that changes status or mask re-evaluates. The no-line case
 * (nibble 0, irq -1) never touches the fork PIC. Pending low-5 bits
 * are always mask-enabled here: the mask write strips disabled bits
 * and every raise checks the mask first, matching the oracle order. */
static void PAS_EvalIRQ(void) {
	if (pas.irq == -1) return;
	if (pas.irqStatus & 0x1f)
		PIC_ActivateIRQ((Bitu)pas.irq);
	else
		PIC_DeActivateIRQ((Bitu)pas.irq);
}

static void PAS_RaiseIRQ(Bitu bits) {
	pas.irqStatus |= (uint8_t)bits;
	if (pas.irq == -1) return;
	if (pas.irqStatus & pas.irqMask & (uint8_t)bits)
		PIC_ActivateIRQ((Bitu)pas.irq);
}

/* Stage 3: MIDI IRQ condition, re-expressed from the oracle MIDI
 * update helper. UART-out requests with status bits 3/4, UART-in with
 * status bit 2; the bit itself clears only via B89/B8B/reset. */
static void PAS_UpdateMidiIRQ(void) {
	if (pas.midiUartOut && (pas.midiStat & 0x18))
		PAS_RaiseIRQ(PAS_IRQ_MIDI);
	else if (pas.midiUartIn && (pas.midiStat & 0x04))
		PAS_RaiseIRQ(PAS_IRQ_MIDI);
	else
		PAS_EvalIRQ();
}

/* Stage 4: effective card clock, re-expressed from the oracle clock
 * select (native-mode bit at 8000 plus a nonzero prescaler picks the
 * divided clock). The exact scaled curve stays undetermined (see the
 * PAS_PIT_CLOCK note); the divide model stands. */
static void PAS_UpdateClock(void) {
	pas.pitClock = PAS_PIT_CLOCK;
	if ((pas.sysConf[0] & 0x02) && pas.prescale)
		pas.pitClock = PAS_PIT_CLOCK / (double)pas.prescale;
	PAS_PumpUpdate();
}

/* Stage 3: programmed sample rate from the counter-0 divisor. An
 * 8254 divisor of 0 means 65536; unprogrammed means silent. */
static double PAS_SampleRate(void) {
	if (!pas.pitSet[0]) return 0.0;
	Bitu div = pas.pitCount[0] ? pas.pitCount[0] : 65536;
	return pas.pitClock / (double)div;
}

/* Stage 3: DMA channel attach, following the house pattern (release
 * the old channel callback, take the new channel, register for
 * mask/unmask/terminal-count). Map value 4 is the cascade slot, which
 * carries no channel. Registration replays the current mask state
 * into the callback, so dmaMasked is fresh afterwards. */
static void PAS_DMA_Callback(DmaChannel *chan, DMAEvent event);

static void PAS_AttachDMA(void) {
	if (pas.dmachan != NULL) {
		pas.dmachan->Register_Callback(NULL);
		pas.dmachan = NULL;
	}
	pas.dmaMasked = true;
	pas.dmaTC = false;
	if (pas.dma == 4) {
		PAS_PumpUpdate();
		return;
	}
	DmaChannel *ch = GetDMAChannel((uint8_t)pas.dma);
	if (ch == NULL) {
		PAS_PumpUpdate();
		return;
	}
	pas.dmachan = ch;
	pas.dmachan->Register_Callback(PAS_DMA_Callback);
	PAS_PumpUpdate();
}

static void PAS_DMA_Callback(DmaChannel *chan, DMAEvent event) {
	if (chan != pas.dmachan) return;
	if (event == DMA_MASKED)
		pas.dmaMasked = true;
	else if (event == DMA_UNMASKED) {
		pas.dmaMasked = false;
		pas.dmaTC = false;
	} else if (event == DMA_REACHED_TC)
		pas.dmaTC = true;
	PAS_PumpUpdate();
}

/* Stage 4: pump arming. PIO mode (PCM on but DMA enable clear) stays
 * silent by design: the oracle stores no sample on F88/F89 writes
 * (buffer-flush no-ops), so there is no PIO path to model. */
static bool PAS_PumpWanted(void) {
	if (!(pas.pcmCtrl & PAS_PCM_ENABLE)) return false;
	if (!(pas.pcmCtrl & PAS_PCM_DMA)) return false;
	if (!pas.pitGate[0]) return false;
	if (!pas.pitSet[0]) return false;
	if (pas.dmachan == NULL) return false;
	if (pas.dmaMasked || pas.dmaTC) return false;
	return true;
}

static void PAS_PumpUpdate(void) {
	PIC_RemoveEvents(PAS_PumpTick);
	pas.pumpOn = PAS_PumpWanted();
	if (pas.chan != NULL)
		pas.chan->Enable(pas.pumpOn);
	if (pas.pumpOn)
		PIC_AddEvent(PAS_PumpTick, PAS_PUMP_INTERVAL_MS);
}

/* Stage 3: DMA sample readers, re-expressed from the oracle
 * word/half-word walk. Narrow channels (0-3) move bytes, wide ones
 * (5-7) move words, and an 8-bit sample on a wide channel consumes
 * half a word per tick through the flip-flop. A short fork read means
 * the channel ran dry: flag it, and the tick renders silence with no
 * IRQ, as the oracle zero-fill path does. Width and sign handling
 * follow the 8001 system-config bits: bit 2 selects 16-bit, bit 3
 * masks 12-bit, bit 4 inverts the MSB. */
static uint8_t PAS_ReadByte(void) {
	uint8_t b = 0;
	if (pas.dmachan->Read(1, &b) != 1) {
		pas.sampDry = true;
		return 0;
	}
	return b;
}

static uint16_t PAS_ReadSamp8(void) {
	uint16_t w;
	if (pas.dma >= 5) {
		if (!pas.dmaFlip) {
			uint8_t b[2] = {0, 0};
			if (pas.dmachan->Read(2, b) != 2) {
				pas.sampDry = true;
				return 0;
			}
			pas.dmaHold = (uint16_t)(b[0] | ((uint16_t)b[1] << 8));
		}
		w = pas.dmaFlip ? (pas.dmaHold >> 8) : (pas.dmaHold & 0xff);
		pas.dmaFlip = !pas.dmaFlip;
	} else {
		w = PAS_ReadByte();
	}
	return (uint16_t)((w ^ 0x80) << 8);
}

static uint16_t PAS_ReadSamp16(int *clocks) {
	uint16_t w;
	if (pas.dma >= 5) {
		uint8_t b[2] = {0, 0};
		if (pas.dmachan->Read(2, b) != 2) {
			pas.sampDry = true;
			return 0;
		}
		w = (uint16_t)(b[0] | ((uint16_t)b[1] << 8));
		*clocks = 1;
	} else {
		uint8_t lo = PAS_ReadByte();
		uint8_t hi = PAS_ReadByte();
		w = (uint16_t)(lo | ((uint16_t)hi << 8));
		*clocks = 2;
	}
	if (pas.sysConf[1] & 0x08)
		w &= 0xfff0;
	if (pas.sysConf[1] & 0x10)
		w ^= 0x8000;
	return w;
}

/* Stage 3: one sample instant, re-expressed from the oracle counter-0
 * tick. Mono fans one read to both sides; native stereo (8000 bit 1)
 * reads a fresh pair; compat stereo alternates halves and mirrors the
 * half into status bit 5. The counter-1 cascade follows the oracle
 * word-clock rule: 8-bit instants do not advance it, 16-bit instants
 * advance it per word consumed, and reaching zero reloads and raises
 * the PCM IRQ. The sample IRQ goes out once per instant. Both IRQs
 * stay subject to the counter-1 rate mode the drivers program. */
static void PAS_Timer0Step(int16_t *out) {
	pas.sampDry = false;
	bool wide16 = (pas.sysConf[1] & 0x04) != 0;
	int clocks = 0;
	if (pas.pcmCtrl & PAS_PCM_MONO) {
		uint16_t t = wide16 ? PAS_ReadSamp16(&clocks) : PAS_ReadSamp8();
		pas.pcmL = pas.pcmR = (int16_t)t;
	} else if (pas.sysConf[0] & 0x02) {
		uint16_t l = wide16 ? PAS_ReadSamp16(&clocks) : PAS_ReadSamp8();
		int more = 0;
		uint16_t r = wide16 ? PAS_ReadSamp16(&more) : PAS_ReadSamp8();
		clocks += more;
		pas.pcmL = (int16_t)l;
		pas.pcmR = (int16_t)r;
	} else {
		uint16_t t = wide16 ? PAS_ReadSamp16(&clocks) : PAS_ReadSamp8();
		if (pas.stereoHalf)
			pas.pcmR = (int16_t)t;
		else
			pas.pcmL = (int16_t)t;
		pas.stereoHalf ^= 1;
		pas.irqStatus = (uint8_t)((pas.irqStatus & 0xdf) | (pas.stereoHalf << 5));
	}
	if (pas.sampDry) {
		out[0] = out[1] = 0;
		return;
	}
	out[0] = pas.pcmL;
	out[1] = pas.pcmR;
	if (clocks > 0 && pas.pitGate[1] && pas.pitSet[1]) {
		pas.ctr1pos -= clocks;
		while (pas.ctr1pos <= 0) {
			pas.ctr1pos += pas.pitCount[1] ? pas.pitCount[1] : 65536;
			if (pas.pitMode[1] & 0x02)
				PAS_RaiseIRQ(PAS_IRQ_PCM);
		}
	}
	if (pas.pitMode[1] & 0x02)
		PAS_RaiseIRQ(PAS_IRQ_SAMP);
}

static void PAS_PumpTick(Bitu /*val*/) {
	if (!pas.pumpOn || pas.chan == NULL) return;
	double rate = PAS_SampleRate();
	Bitu n = (Bitu)(rate * (PAS_PUMP_INTERVAL_MS / 1000.0) + 0.5);
	if (n < 1) n = 1;
	if (n > PAS_PUMP_MAX) n = PAS_PUMP_MAX;
	static int16_t buf[PAS_PUMP_MAX * 2];
	for (Bitu i = 0; i < n; i++)
		PAS_Timer0Step(&buf[i * 2]);
	pas.chan->AddSamples_s16(n, buf);
	if (pas.pumpOn)
		PIC_AddEvent(PAS_PumpTick, PAS_PUMP_INTERVAL_MS);
}

/* Stage 3: PCM engine reset, re-expressed from the oracle PCM
 * reset. Clearing the enable stops the pump; the line follows. */
static void PAS_ResetPCM(void) {
	pas.pcmCtrl = 0x00;
	pas.stereoHalf = 0;
	pas.irqStatus &= 0xd7;
	PAS_EvalIRQ();
	PAS_PumpUpdate();
}

/* Stage 3: register-file reset, re-expressed from the oracle common
 * reset. Gates drop, the clock returns to standard, and the pump and
 * line follow. Counter divisors are NOT cleared: the guest reprograms
 * them, and the set flags keep a stale rate from restarting the pump
 * only until the gates reopen it. */
static void PAS_ResetRegs(void) {
	pas.sysConf[0] &= 0xfd;
	pas.sysConf[1] = 0x00;
	pas.sysConf[2] = 0x00;
	pas.prescale = 0x00;
	pas.pitClock = PAS_PIT_CLOCK;
	pas.pitGate[0] = pas.pitGate[1] = false;
	pas.filterCtrl = 0x00;
	pas.filterOn = false;
	pas.filterCutoff = 0;
	PAS_ResetPCM();
	pas.dmaFlip = false;
	pas.irqMask = 0x00;
	pas.irqStatus = 0x00;
	PAS_MixerReset();
	PAS_EvalIRQ();
	PAS_PumpUpdate();
}

static int PAS_IrqConvert(Bitu val) {
	/* Stage 3: nibble-to-IRQ map, re-expressed from the oracle
	 * converter. Nibble 0 means no line (-1); the result is stored
	 * in the signed pas.irq, which resolves the stage-2 wrap flag
	 * (the -1 used to land in an unsigned). */
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

/* Stage 3: 1388-138B rate/count window, re-expressed from the
 * oracle PIT attachment. The fork has no per-card PIT device to
 * attach, so the window is modeled here as an 8254-style counter pair:
 * 138B takes control words (counter select, access, mode), 1388/1389
 * assemble the counter-0/1 divisors LSB-then-MSB, and 138A (counter 2)
 * is stored nowhere. Reads return the stored divisor image LSB/MSB in
 * turn; the control port reads back open-bus. A completed divisor or
 * gate change re-arms the sample pump. Counter modes default to the
 * rate-generator shape the drivers program until the guest writes
 * control. */
static Bitu pas_pit_read(Bitu port, Bitu iolen) {
	(void)iolen;//UNUSED
	Bitu off = port - (pas.basePort + 0x1000);
	if (off > 3) return 0xff;
	if (off == 3 || off == 2) return 0xff;
	uint16_t v = pas.pitCount[off];
	if (!pas.pitReadHi[off]) {
		pas.pitReadHi[off] = true;
		return v & 0xff;
	}
	pas.pitReadHi[off] = false;
	return (v >> 8) & 0xff;
}

static void pas_pit_write(Bitu port, Bitu val, Bitu iolen) {
	(void)iolen;//UNUSED
	Bitu off = port - (pas.basePort + 0x1000);
	if (off > 3) return;
	uint8_t v = (uint8_t)val;
	if (off == 3) {
		Bitu c = (v >> 6) & 3;
		if (c > 1) return;
		Bitu access = (v >> 4) & 3;
		if (access == 0) return;
		pas.pitMode[c] = (uint8_t)((v >> 1) & 7);
		pas.pitAccess[c] = (uint8_t)access;
		pas.pitNeed[c] = (access == 2) ? 1 : 0;
		pas.pitReadHi[c] = false;
		return;
	}
	if (off == 2) return;
	Bitu c = off;
	if (pas.pitNeed[c] == 0) {
		if (pas.pitAccess[c] == 2) {
			pas.pitCount[c] = (uint16_t)((pas.pitCount[c] & 0x00ff) | ((uint16_t)v << 8));
		} else {
			pas.pitCount[c] = (uint16_t)((pas.pitCount[c] & 0xff00) | v);
			if (pas.pitAccess[c] == 3) {
				pas.pitNeed[c] = 1;
				return;
			}
		}
	} else {
		pas.pitCount[c] = (uint16_t)((pas.pitCount[c] & 0x00ff) | ((uint16_t)v << 8));
		pas.pitNeed[c] = 0;
	}
	pas.pitSet[c] = true;
	if (c == 1)
		pas.ctr1pos = pas.pitCount[1] ? pas.pitCount[1] : 65536;
	PAS_PumpUpdate();
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
		/* Stage 3: UART data read, re-expressed from the oracle
		 * (probe echo + queue drain). The MIDI IRQ follows the
		 * drained status; filling the queue from host MIDI-in is
		 * STAGE4. */
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
			PAS_UpdateMidiIRQ();
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
		/* Stage 3: compat enables; the live SB/MPU devices keep
		 * their own ports (state-only wiring, see the write side). */
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
		/* Stage 3: B89 clear-on-write-1, re-expressed from the oracle
		 * PAS16 path. The line drops when no low-5 bit stays pending. */
		pas.irqStatus &= ~v;
		PAS_EvalIRQ();
		break;
	case 0x0802: {
		/* Stage 4: B8A filter control, re-expressed from the oracle
		 * PAS16 path. Bit 7 gates counter 1, bit 6 gates counter 0;
		 * the half-toggle and flip-flop restart with the engine. The
		 * rising-mute mask/status clear lives on the old-PAS path
		 * only, so the PAS16 path stores the byte as-is. The cutoff
		 * table matches the oracle values. Filter coefficients stay
		 * deferred (STAGE4): the oracle builds a runtime
		 * Blackman-windowed-sinc FIR per cutoff against the host
		 * rate, which is new DSP code rather than data, and the fork
		 * mixer consumes channel output directly, so no resample
		 * stage needs them yet. */
		pas.pitGate[1] = (v & 0x80) != 0;
		pas.pitGate[0] = (v & 0x40) != 0;
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
		PAS_PumpUpdate();
		break;
	}
	case 0x0803:
		/* Stage 3: B8B IRQ mask, re-expressed from the oracle PAS16
		 * path. Disabled low-5 bits are stripped from status, then
		 * the line follows what stays pending. */
		pas.irqMask = v & 0x1f;
		pas.irqStatus &= ((v & 0x1f) | 0xe0);
		PAS_EvalIRQ();
		break;
	case 0x0c00: case 0x0c01:
		/* Stage 4: F88/F89 PIO writes carry no sample path, matching
		 * the oracle (its writes flush the mixer buffer only and
		 * store nothing); the DMA engine above is the only sample
		 * source. */
		break;
	case 0x0c02:
		/* Stage 3: F8A PCM control with enable-edge reset, re-expressed
		 * from the oracle enable edge. Arming or clearing the enable
		 * re-evaluates the sample pump. */
		if ((v & PAS_PCM_ENABLE) && !(pas.pcmCtrl & PAS_PCM_ENABLE)) {
			pas.stereoHalf = 0;
			pas.irqStatus &= 0xd7;
			pas.dmaFlip = false;
		}
		pas.pcmCtrl = v;
		PAS_PumpUpdate();
		break;
	case 0x1401: case 0x1403:
		/* Stage 3: UART control + mode flags, re-expressed from the
		 * oracle MIDI path. The mode change re-evaluates the MIDI
		 * IRQ on the PAS line. */
		pas.midiCtrl = v;
		if ((v & 0x60) == 0x60) {
			pas.midiUartOut = false;
			pas.midiUartIn = false;
		} else if ((v & 0x1c) == 0x04)
			pas.midiUartIn = true;
		else
			pas.midiUartOut = true;
		PAS_UpdateMidiIRQ();
		break;
	case 0x1402: case 0x1802:
		/* Stage 3: UART data store, re-expressed from the oracle
		 * MIDI path. UART-out bytes reach the host MIDI device. */
		pas.midiData = v;
		if (pas.midiUartOut)
			MIDI_RawOutByte(v);
		break;
	case 0x1800:
		pas.midiStat = v;
		PAS_UpdateMidiIRQ();
		break;
	case 0x1801:
		pas.midiFifo = v;
		break;
	case 0x8000:
		/* Stage 3: system config 1 with reset-on-rise of the top
		 * bits; the clock select follows (base stays conf-fixed). */
		if ((v & 0xc0) && !(pas.sysConf[0] & 0xc0)) {
			PAS_ResetRegs();
			pas.sysConf[0] = 0x00;
		} else
			pas.sysConf[0] = v;
		PAS_UpdateClock();
		break;
	case 0x8001:
		pas.sysConf[1] = v;
		break;
	case 0x8002:
		pas.sysConf[2] = v;
		PAS_UpdateClock();
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
		PAS_UpdateClock();
		break;
	case 0xf000:
		/* Stage 2: joystick-enable bit stored; gameport remap is
		 * out of this stage. */
		pas.ioConf[0] = v;
		break;
	case 0xf001:
		/* Stage 3: DMA select, re-expressed from the oracle select
		 * path. The channel is re-attached (house pattern) and the
		 * clock select follows, since the width feeds it. */
		pas.ioConf[1] = v;
		pas.dma = pas_dma_map[v & 0x07];
		PAS_AttachDMA();
		PAS_UpdateClock();
		break;
	case 0xf002:
		/* Stage 3: PAS IRQ select via the nibble map; the SCSI high
		 * nibble is out of scope (no SCSI side). The old line is
		 * released first, then the new line follows pending state. */
		pas.ioConf[2] = v;
		if (pas.irq != -1)
			PIC_DeActivateIRQ((Bitu)pas.irq);
		pas.irq = PAS_IrqConvert(v);
		PAS_EvalIRQ();
		break;
	case 0xf003:
		pas.ioConf[3] = v;
		break;
	case 0xf400:
		/* Stage 4: compat enables, re-expressed from the oracle
		 * compat path (bit 1 enables the SB side, bit 0 the MPU
		 * side). The enables are state only: driving the live fork
		 * SB/MPU devices needs cross-file address hooks that do not
		 * exist yet (STAGE4, owned by those files; the oracle
		 * re-addresses its SB/MPU devices on every F400/F401
		 * write, which is the behavior the hooks must replay). */
		pas.compat = v & 0xf3;
		pas.sbOn = (v & 0x02) != 0;
		pas.mpuOn = (v & 0x01) != 0;
		break;
	case 0xf401:
		/* Stage 4: compat bases, re-expressed from the oracle base
		 * derivation (low nibble picks the 0x2x0 SB base, high
		 * nibble the 0x3x0 MPU base). Stored; live remap is the
		 * same STAGE4 item as above, replayed on every write. */
		pas.compatBase = v;
		pas.sbBase = 0x200 + (((Bitu)v & 0x0f) << 4);
		pas.mpuBase = 0x300 + ((Bitu)v & 0xf0);
		break;
	case 0xf802:
		/* Stage 3: SB IRQ/DMA select is stored only; the plan keeps
		 * the SB-DSP side out, so there is nothing to drive. */
		pas.sbIrqDma = v;
		break;
	default:
		/* Unmapped offsets ignore writes, matching the oracle
		 * default and this fork's uninstalled-port convention. */
		break;
	}
}

/* Stage 3: mixer pull callback. The sample pump pushes decoded DMA
 * bytes via AddSamples; this fills any remainder with silence so the
 * channel stays gapless while the pump is idle. */
static void PAS_Callback(Bitu len) {
	if (!len || pas.chan == NULL) return;
	pas.chan->AddSilence();
}

class PAS: public Module_base {
public:
	IO_ReadHandleObject ReadHandler[16];
	IO_WriteHandleObject WriteHandler[16];
	MixerObject MixerChan;
	~PAS() {
		PIC_RemoveEvents(PAS_PumpTick);
		pas.pumpOn = false;
		if (pas.dmachan != NULL) {
			pas.dmachan->Register_Callback(NULL);
			pas.dmachan = NULL;
		}
		if (pas.chan != NULL)
			pas.chan->Enable(false);
	}
	PAS(Section* configuration):Module_base(configuration) {
		Section_prop * section=static_cast<Section_prop *>(configuration);
		if(!section->Get_bool("pas")||control->opt_silent) return;
		pas.enabled = true;
		pas.basePort = (unsigned int)section->Get_hex("pasbase");
		pas.irq = section->Get_int("pasirq");
		pas.dma = (unsigned int)section->Get_int("pasdma");
		pas.rate = (unsigned int)section->Get_int("pasrate");

		LOG_MSG("PAS:Initializing Pro Audio Spectrum 16 emulation...");
		memset(pas.mixRegs, 0, sizeof(pas.mixRegs));
		pas.mixIndex = 0;
		pas.chan = NULL;
		pas.midiR = 0;
		pas.midiW = 0;
		pas.dmachan = NULL;
		pas.dmaMasked = true;
		pas.dmaTC = false;
		pas.dmaHold = 0;
		pas.sampDry = false;
		pas.pcmL = 0;
		pas.pcmR = 0;
		pas.pitCount[0] = pas.pitCount[1] = 0;
		pas.pitMode[0] = pas.pitMode[1] = 3;
		pas.pitAccess[0] = pas.pitAccess[1] = 3;
		pas.pitNeed[0] = pas.pitNeed[1] = 0;
		pas.pitSet[0] = pas.pitSet[1] = false;
		pas.pitGate[0] = pas.pitGate[1] = false;
		pas.pitReadHi[0] = pas.pitReadHi[1] = false;
		pas.ctr1pos = 65536;
		pas.pitClock = PAS_PIT_CLOCK;
		pas.pumpOn = false;
		pas.sbBase = 0;
		pas.mpuBase = 0;
		pas.sbOn = false;
		pas.mpuOn = false;
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
		ReadHandler[h].Install(pas.basePort + 0x1000, pas_pit_read, IO_MB, 4); h++;
		WriteHandler[15].Install(pas.basePort + 0x1000, pas_pit_write, IO_MB, 4);
		/* Stage 4 NOTE: OPL aliases (base+0x0000) are NOT installed
		 * here, by decision. The card carries an OPL3, but this fork
		 * already serves 0x388-0x38B from its shared OPL emulation,
		 * and the default PAS base is 0x388, so the alias window
		 * coincides with it (SB-style: the card owns no FM, the shared
		 * emulation does). A non-default base would need forwarder
		 * hooks into the OPL module, which has none to offer today
		 * (STAGE4, owned by that module). Likewise the MV508 FM
		 * voice gains computed above scale the PCM path only; mixing
		 * the shared OPL output through them is the same STAGE4. */

		pas.chan = MixerChan.Install(&PAS_Callback, pas.rate, "PAS");
		pas.chan->Enable(false);
		PAS_AttachDMA();

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

/* Stage 4: no-arg instantiation entry in the INNOVA_Init house
 * pattern: sdlmain calls this once; allocation happens on reset via
 * PAS_OnReset. */
void PAS_Init() {
	LOG(LOG_MISC,LOG_DEBUG)("Initializing PAS emulation");

	AddExitFunction(AddExitFunctionFuncPair(PAS_ShutDown),true);
	AddVMEventFunction(VM_EVENT_RESET,AddVMEventFunctionFuncPair(PAS_OnReset));
}
