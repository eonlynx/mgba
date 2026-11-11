/* Copyright (c) 2026 eonlynx
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba/internal/gblink.h>

#include <mgba/internal/gb/gb.h>
#include <mgba/internal/gb/io.h>
#include <mgba/internal/gb/sio.h>

// GB side of the cable.
//
// The GBA is the clock master, so when the GB is in external-clock mode
// (SC bit 0 = 0) the core does not shift anything by itself.  This driver takes
// the transfer over: it records the byte the GB wants to send, marks itself
// armed, and completes (SB + serial IRQ) only when the GBA has clocked it.
// That is exactly what the hardware does, and it is what keeps the two sides
// in lockstep.
//
// During the very first handshake the GB also probes the link as the clock
// master (SC bit 0 = 1).  In that case there is nobody else to clock it, so the
// transfer completes immediately with whatever the GBA last sent.

#define GB_GB_LINK_POLL_CYCLES 64
// Behave like an unplugged cable if the GBA never clocks (2 s at 4.19 MHz).
#define GB_GB_LINK_TIMEOUT_CYCLES (4194304u * 2u)

static void _gbGBLinkComplete(struct GBSIOGBLinkDriver* link, uint8_t received) {
	struct GBSIO* sio = link->d.p;
	struct GB* gb = sio->p;
	link->waiting = false;
	mTimingDeschedule(&gb->timing, &link->event);
	// Cancel the shift window the core may have scheduled for an internal
	// clock transfer, and finish the byte here.
	sio->remainingBits = 0;
	sio->pendingSB = 0xFF;
	gb->memory.io[GB_REG_SB] = received;
	gb->memory.io[GB_REG_SC] = GBRegisterSCClearEnable(gb->memory.io[GB_REG_SC]);
	gb->memory.io[GB_REG_IF] |= 1 << GB_IRQ_SIO;
	GBUpdateIRQs(gb);
	mLOG(GB_SIO, DEBUG, "GBLink GB done recv=%02X gb=%u gba=%u",
	     received, link->bridge->gbCount, link->bridge->gbaCount);
}

static void _gbGBLinkPoll(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	UNUSED(cyclesLate);
	struct GBSIOGBLinkDriver* link = context;
	uint8_t received;
	if (!link->waiting || !link->d.p) {
		return;
	}
	if (GBLinkBridgeGbFinish(link->bridge, link->waitForGba, &received)) {
		_gbGBLinkComplete(link, received);
		return;
	}
	link->waited += GB_GB_LINK_POLL_CYCLES;
	if (link->waited > GB_GB_LINK_TIMEOUT_CYCLES) {
		GBLinkBridgeArm(link->bridge, false);
		_gbGBLinkComplete(link, 0xFF);
		return;
	}
	mTimingSchedule(timing, &link->event, GB_GB_LINK_POLL_CYCLES);
}

static void _gbGBLinkWriteSB(struct GBSIODriver* driver, uint8_t value) {
	struct GBSIOGBLinkDriver* link = (struct GBSIOGBLinkDriver*) driver;
	link->sb = value;
	GBLinkBridgeSetGbSb(link->bridge, value);
	static unsigned logCount;
	if (logCount < 300 || (++logCount & 0x3F) == 0) {
		mLOG(GB_SIO, DEBUG, "GBLink GB sb=%02X", value);
		logCount++;
	}
}

static uint8_t _gbGBLinkWriteSC(struct GBSIODriver* driver, uint8_t value) {
	struct GBSIOGBLinkDriver* link = (struct GBSIOGBLinkDriver*) driver;
	struct GBSIO* sio = link->d.p;
	struct GB* gb = sio->p;
	bool enabled = GBRegisterSCIsEnable(value);
	static unsigned scLog;

	if (enabled) {
		if (scLog < 300 || (++scLog & 0x3F) == 0) {
			mLOG(GB_SIO, DEBUG, "GBLink GB %s sc=%02X sb=%02X",
			     GBRegisterSCIsShiftClock(value) ? "master" : "slave ", value, link->sb);
			scLog++;
		}
	}

	if (enabled && GBRegisterSCIsShiftClock(value)) {
		// The GB is acting as the clock master (handshake probe).
		mTimingDeschedule(&gb->timing, &sio->event);
		GBLinkBridgeSetGbSb(link->bridge, link->sb);
		_gbGBLinkComplete(link, GBLinkBridgeGbaLatch(link->bridge));
	} else if (enabled) {
		// The GB is the external-clock slave: arm and wait for the GBA.
		mTimingDeschedule(&gb->timing, &sio->event);
		sio->remainingBits = 0;
		GBLinkBridgeSetGbSb(link->bridge, link->sb);
		GBLinkBridgeArm(link->bridge, true);
		link->waitForGba = GBLinkBridgeGbaCount(link->bridge) + 1;
		link->waited = 0;
		link->waiting = true;
		mTimingDeschedule(&gb->timing, &link->event);
		mTimingSchedule(&gb->timing, &link->event, GB_GB_LINK_POLL_CYCLES);
	} else if (link->waiting) {
		link->waiting = false;
		GBLinkBridgeArm(link->bridge, false);
		mTimingDeschedule(&gb->timing, &link->event);
	}
	return 0;
}

void GBSIOGBLinkDriverCreate(struct GBSIOGBLinkDriver* driver, struct GBLinkBridge* bridge) {
	memset(driver, 0, sizeof(*driver));
	driver->d.writeSB = _gbGBLinkWriteSB;
	driver->d.writeSC = _gbGBLinkWriteSC;
	driver->bridge = bridge;
	driver->event.context = driver;
	driver->event.name = "GB GB Link";
	driver->event.callback = _gbGBLinkPoll;
	driver->event.priority = 0x40;
}
