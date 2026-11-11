/* Copyright (c) 2026 eonlynx
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba/internal/gblink.h>

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>
#include <mgba/internal/gba/sio.h>

// The GB is the clock master: it has already shifted the byte in when it
// pushes it, so the GBA must complete almost immediately.  Any artificial
// delay here is added on top of the GB's own ~1 ms per byte and doubles the
// byte time, which makes the GBA fall behind during the fast party exchange
// (Crystal clocks up to ~1024 bytes/s there) and never catch up.
#define GBA_GB_LINK_CYCLES 256
#define GBA_GB_LINK_POLL_CYCLES 1024

static void _gbaGBLinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate);
static void _gbaGBLinkReset(struct GBASIODriver* driver);
static void _gbaGBLinkDeinit(struct GBASIODriver* driver);
static bool _gbaGBLinkHandlesMode(struct GBASIODriver* driver, enum GBASIOMode mode);
static int _gbaGBLinkConnectedDevices(struct GBASIODriver* driver);
static int _gbaGBLinkDeviceId(struct GBASIODriver* driver);
static bool _gbaGBLinkStart(struct GBASIODriver* driver);

void GBASIOGBLinkDriverCreate(struct GBASIOGBLinkDriver* driver, struct GBLinkBridge* bridge) {
	memset(driver, 0, sizeof(*driver));
	driver->d.deinit = _gbaGBLinkDeinit;
	driver->d.reset = _gbaGBLinkReset;
	driver->d.handlesMode = _gbaGBLinkHandlesMode;
	driver->d.connectedDevices = _gbaGBLinkConnectedDevices;
	driver->d.deviceId = _gbaGBLinkDeviceId;
	driver->d.start = _gbaGBLinkStart;
	driver->bridge = bridge;
	driver->event.context = driver;
	driver->event.name = "GBA GB Link";
	driver->event.callback = _gbaGBLinkEvent;
	driver->event.priority = 0x80;
}

static void _gbaGBLinkDeschedule(struct GBASIOGBLinkDriver* driver) {
	if (driver->d.p && driver->d.p->p) {
		mTimingDeschedule(&driver->d.p->p->timing, &driver->event);
	}
}

static void _gbaGBLinkReset(struct GBASIODriver* d) {
	_gbaGBLinkDeschedule((struct GBASIOGBLinkDriver*) d);
}

static void _gbaGBLinkDeinit(struct GBASIODriver* d) {
	_gbaGBLinkDeschedule((struct GBASIOGBLinkDriver*) d);
}

static bool _gbaGBLinkHandlesMode(struct GBASIODriver* d, enum GBASIOMode mode) {
	UNUSED(d);
	return mode == GBA_SIO_NORMAL_8;
}

static int _gbaGBLinkConnectedDevices(struct GBASIODriver* d) {
	UNUSED(d);
	return 1;
}

static int _gbaGBLinkDeviceId(struct GBASIODriver* d) {
	UNUSED(d);
	return 0;
}

static bool _gbaGBLinkStart(struct GBASIODriver* d) {
	struct GBASIOGBLinkDriver* driver = (struct GBASIOGBLinkDriver*) d;
	struct GBA* gba = d->p->p;
	uint32_t now = mTimingCurrentTime(&gba->timing);
	if (!driver->started || (uint32_t) (now - driver->lastTransfer) > GB_LINK_SESSION_GAP) {
		// The GB keeps clocking while the GBA is not listening (the player is
		// still walking to the cable club counter, for example).  Those bytes
		// belong to a previous session: drop them, or the GBA would start the
		// exchange thousands of bytes behind.
		GBLinkBridgeFlush(driver->bridge);
	}
	driver->started = true;
	driver->lastTransfer = now;
	driver->starts++;
	if (!mTimingIsScheduled(&gba->timing, &driver->event)) {
		mTimingSchedule(&gba->timing, &driver->event, GBA_GB_LINK_CYCLES);
	}
	// Completion is handled by the driver when the GB clocks the byte.
	return false;
}

static void _gbaGBLinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	UNUSED(cyclesLate);
	struct GBASIOGBLinkDriver* driver = context;
	struct GBASIO* sio = driver->d.p;
	if (!sio || !driver->started) {
		return;
	}
	driver->lastTransfer = mTimingCurrentTime(timing);
	driver->polls++;
	// Do not depend on the game re-arming the transfer: while the GB keeps
	// clocking (it is the master), every byte it sends must be consumed.  If
	// the game disables interrupts for a moment the SIO would otherwise stop
	// for seconds, the GB would read the stale latch and time out.
	if (!GBLinkBridgeHasGbByte(driver->bridge)) {
		mTimingSchedule(timing, &driver->event, GBA_GB_LINK_POLL_CYCLES);
		return;
	}
	uint8_t out = sio->p->memory.io[GBA_REG(SIODATA8)];
	uint8_t in = GBLinkBridgeExchange(driver->bridge, out, true);
	static unsigned logCount;
	if (logCount < 256 || (++logCount & 0x3F) == 0) {
		mLOG(GBA_SIO, DEBUG, "GBLink GBA gba=%u gb=%u out=%02X in=%02X q=%u st=%u p=%u",
		     driver->bridge->gbaCount, driver->bridge->gbCount, out, in,
		     (driver->bridge->gbToGbaHead + GB_LINK_QUEUE_SIZE - driver->bridge->gbToGbaTail) % GB_LINK_QUEUE_SIZE,
		     driver->starts, driver->polls);
		logCount++;
	}
	GBASIONormal8FinishTransfer(sio, in, 0);
	mTimingSchedule(timing, &driver->event, GBA_GB_LINK_POLL_CYCLES);
}
