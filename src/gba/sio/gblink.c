/* Copyright (c) 2026 eonlynx
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba/internal/gblink.h>

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>
#include <mgba/internal/gba/sio.h>

// One byte at 8192 bps is ~976 us, i.e. 16384 GBA cycles.  The GB clock
// master runs at the same byte rate, so the queues stay balanced.
#define GBA_GB_LINK_CYCLES 16384

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
	if (!mTimingIsScheduled(&gba->timing, &driver->event)) {
		mTimingSchedule(&gba->timing, &driver->event, GBA_GB_LINK_CYCLES);
	}
	// Completion is handled by the driver when the GB clocks the byte.
	return false;
}

static void _gbaGBLinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	UNUSED(timing);
	struct GBASIOGBLinkDriver* driver = context;
	struct GBASIO* sio = driver->d.p;
	if (!sio || !GBASIONormalIsStart(sio->siocnt)) {
		return;
	}
	uint8_t out = sio->p->memory.io[GBA_REG(SIODATA8)];
	uint8_t in = GBLinkBridgeExchange(driver->bridge, out, true);
	GBASIONormal8FinishTransfer(sio, in, cyclesLate);
}
