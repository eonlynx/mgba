/* Copyright (c) 2026 eonlynx
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba/internal/gblink.h>

#include <mgba/internal/gb/gb.h>
#include <mgba/internal/gb/io.h>
#include <mgba/internal/gb/sio.h>

// A real link cable is synchronous: the GB clocks, and the byte it reads is
// whatever the other machine had already shifted in.  When the GB is the clock
// master, that means its byte only becomes available once the GBA has actually
// consumed the previous one.  Letting the GB core shift on its own clock makes
// it run ahead of the GBA (the GBA then answers dozens of bytes late and the
// GB reads a stale latch), which is what broke the Crystal handshake.
//
// So the driver takes the transfer over from the core: it hands the byte to
// the bridge and only completes the GB side (SB + serial IRQ) once the GBA has
// clocked it out.  No thread is ever blocked; the GB just keeps executing
// until the interrupt arrives, exactly like an external-clock slave.

#define GB_GB_LINK_POLL_CYCLES 64
// Give up if the GBA is not attached/paused: behave like an unplugged cable.
#define GB_GB_LINK_TIMEOUT_CYCLES (4194304u * 2u)

static void _gbGBLinkComplete(struct GBSIOGBLinkDriver* link, uint8_t received) {
	struct GBSIO* sio = link->d.p;
	struct GB* gb = sio->p;
	link->waiting = false;
	mTimingDeschedule(&gb->timing, &link->event);
	// The core scheduled its own 8-bit shift when SC was written; cancel it.
	sio->remainingBits = 0;
	sio->pendingSB = 0xFF;
	gb->memory.io[GB_REG_SB] = received;
	gb->memory.io[GB_REG_SC] = GBRegisterSCClearEnable(gb->memory.io[GB_REG_SC]);
	gb->memory.io[GB_REG_IF] |= 1 << GB_IRQ_SIO;
	GBUpdateIRQs(gb);
}

static void _gbGBLinkPoll(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	UNUSED(cyclesLate);
	struct GBSIOGBLinkDriver* link = context;
	if (!link->waiting || !link->d.p) {
		return;
	}
	uint32_t gba;
	uint8_t latch;
	MutexLock(&link->bridge->mutex);
	gba = link->bridge->gbaCount;
	latch = link->bridge->gbaLatch;
	MutexUnlock(&link->bridge->mutex);
	link->waited += GB_GB_LINK_POLL_CYCLES;
	if (gba != link->waitForGba) {
		_gbGBLinkComplete(link, latch);
		return;
	}
	if (link->waited > GB_GB_LINK_TIMEOUT_CYCLES) {
		_gbGBLinkComplete(link, 0xFF);
		return;
	}
	mTimingSchedule(timing, &link->event, GB_GB_LINK_POLL_CYCLES);
}

static void _gbGBLinkWriteSB(struct GBSIODriver* driver, uint8_t value) {
	struct GBSIOGBLinkDriver* link = (struct GBSIOGBLinkDriver*) driver;
	link->sb = value;
}

static uint8_t _gbGBLinkWriteSC(struct GBSIODriver* driver, uint8_t value) {
	struct GBSIOGBLinkDriver* link = (struct GBSIOGBLinkDriver*) driver;
	struct GBSIO* sio = link->d.p;
	struct GB* gb = sio->p;
	bool enabled = GBRegisterSCIsEnable(value);
	uint32_t now = mTimingCurrentTime(&gb->timing);
	if (enabled && !link->enabled && (uint32_t) (now - link->lastCycle) > GB_LINK_GB_IDLE_GAP) {
		// A new link session: nothing can be pending in lockstep mode, but
		// make sure a previous attempt left nothing behind.
		GBLinkBridgeFlush(link->bridge);
	}
	link->lastCycle = now;
	link->enabled = enabled;
	if (enabled && GBRegisterSCIsShiftClock(value)) {
		uint32_t before;
		// Take over: the core must not shift on its own, the byte is clocked
		// by the GBA.
		mTimingDeschedule(&gb->timing, &sio->event);
		sio->remainingBits = 0;
		MutexLock(&link->bridge->mutex);
		before = link->bridge->gbaCount;
		MutexUnlock(&link->bridge->mutex);
		static unsigned logCount;
		if (logCount < 256 || (++logCount & 0x3F) == 0) {
			mLOG(GB_SIO, DEBUG, "GBLink GB gb=%u gba=%u sb=%02X",
			     link->bridge->gbCount, link->bridge->gbaCount, link->sb);
			logCount++;
		}
		GBLinkBridgeExchange(link->bridge, link->sb, false);
		link->waitForGba = before + 1;
		link->waited = 0;
		link->waiting = true;
		mTimingDeschedule(&gb->timing, &link->event);
		mTimingSchedule(&gb->timing, &link->event, GB_GB_LINK_POLL_CYCLES);
	} else if (link->waiting) {
		link->waiting = false;
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
