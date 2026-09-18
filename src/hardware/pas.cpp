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

/* Pro Audio Spectrum 16 emulation skeleton (stage 1: configuration only).
 *
 * Knowledge sources: the authors of 86Box's snd_pas16.c and the Linux
 * kernel OSS PAS16 documentation. Hardware behavior is re-expressed here
 * in this fork's house style; no code is copied from those sources.
 */

#include <string.h>
#include "dosbox.h"
#include "inout.h"
#include "logging.h"
#include "mixer.h"
#include "pic.h"
#include "setup.h"
#include "control.h"

static struct {
	bool enabled;
	Bitu basePort;
	Bitu irq;
	Bitu dma;
	Bitu rate;
} pas;

class PAS: public Module_base {
public:
	PAS(Section* configuration):Module_base(configuration) {
		Section_prop * section=static_cast<Section_prop *>(configuration);
		if(!section->Get_bool("pas")||control->opt_silent) return;
		pas.enabled = true;
		pas.basePort = (unsigned int)section->Get_hex("pasbase");
		pas.irq = (unsigned int)section->Get_int("pasirq");
		pas.dma = (unsigned int)section->Get_int("pasdma");
		pas.rate = (unsigned int)section->Get_int("pasrate");

		LOG_MSG("PAS:Initializing Pro Audio Spectrum 16 emulation...");
		/* Stage 1: configuration only. No port handlers, no mixer channel
		   and no IRQ/DMA wiring yet; those arrive with the stage 2 register file. */
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
