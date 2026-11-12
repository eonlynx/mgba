/* Copyright (c) 2026 eonlynx
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba/internal/gblink.h>

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>
#include <mgba/internal/gba/sio.h>

// GBA side of the cable.  The game drives normal 8-bit SIO with the internal
// clock, so every transfer it starts has to be completed here.  The byte it
// sends is latched for the GB, and the byte it receives is whatever the GB
// wrote to SB and armed.
//
// 8 bits at the GBA's normal-mode rate is well under 1 ms; the reference
// implementations pace themselves externally (VCOUNT/timer), so completing a
// few hundred cycles after the start is both accurate and safe.
#define GBA_GB_LINK_CYCLES 512
#define GBA_GB_LINK_POLL_CYCLES 1024

static void _gbaGBLinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate);
static void _gbaGBLinkReset(struct GBASIODriver* driver);
static void _gbaGBLinkDeinit(struct GBASIODriver* driver);
static bool _gbaGBLinkHandlesMode(struct GBASIODriver* driver, enum GBASIOMode mode);
static int _gbaGBLinkConnectedDevices(struct GBASIODriver* driver);
static int _gbaGBLinkDeviceId(struct GBASIODriver* driver);
static bool _gbaGBLinkStart(struct GBASIODriver* driver);
static uint16_t _gbaGBLinkWriteSIOCNT(struct GBASIODriver* driver, uint16_t value);

void GBASIOGBLinkDriverCreate(struct GBASIOGBLinkDriver* driver, struct GBLinkBridge* bridge) {
	memset(driver, 0, sizeof(*driver));
	driver->d.deinit = _gbaGBLinkDeinit;
	driver->d.reset = _gbaGBLinkReset;
	driver->d.handlesMode = _gbaGBLinkHandlesMode;
	driver->d.connectedDevices = _gbaGBLinkConnectedDevices;
	driver->d.deviceId = _gbaGBLinkDeviceId;
	driver->d.start = _gbaGBLinkStart;
	driver->d.writeSIOCNT = _gbaGBLinkWriteSIOCNT;
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
	struct GBASIOGBLinkDriver* driver = (struct GBASIOGBLinkDriver*) d;
	_gbaGBLinkDeschedule(driver);
	driver->started = false;
}

static void _gbaGBLinkDeinit(struct GBASIODriver* d) {
	struct GBASIOGBLinkDriver* driver = (struct GBASIOGBLinkDriver*) d;
	_gbaGBLinkDeschedule(driver);
	driver->started = false;
}

static bool _gbaGBLinkHandlesMode(struct GBASIODriver* d, enum GBASIOMode mode) {
	UNUSED(d);
	return mode == GBA_SIO_NORMAL_8;
}

// SI (bit 2) reflects whether the GB currently holds a byte for us.  The game
// reads it before starting a transfer, like Gen3-to-GenX's slave-ready check.
static uint16_t _gbaGBLinkWriteSIOCNT(struct GBASIODriver* d, uint16_t value) {
	struct GBASIOGBLinkDriver* driver = (struct GBASIOGBLinkDriver*) d;
	if (GBLinkBridgeHasGbSb(driver->bridge)) {
		value &= ~0x0004;
	} else {
		value |= 0x0004;
	}
	return value;
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
	driver->started = true;
	driver->polls = 0;
	if (!mTimingIsScheduled(&gba->timing, &driver->event)) {
		mTimingSchedule(&gba->timing, &driver->event, GBA_GB_LINK_CYCLES);
	}
	// Completion is handled by the driver.
	return false;
}

static void _gbaGBLinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	UNUSED(timing);
	UNUSED(cyclesLate);
	struct GBASIOGBLinkDriver* driver = context;
	struct GBASIO* sio = driver->d.p;
	if (!sio || !driver->started) {
		return;
	}
	// The game gave up on this transfer (its timeout expired): stop polling.
	if (!GBASIONormalIsStart(sio->siocnt)) {
		return;
	}
	driver->polls++;

	// Always finish the transfer, like the real cable: the master does not
	// stall waiting for the slave to arm.  If the GB has a byte armed it is
	// consumed; otherwise the transfer reads SERIAL_NO_DATA_BYTE (0xFE), the
	// protocol's "nothing to send" value, instead of repeating stale data.
	uint8_t out = sio->p->memory.io[GBA_REG(SIODATA8)];
	uint8_t in = 0xFF;
	GBLinkBridgeGbaExchange(driver->bridge, out, &in);

	static unsigned logCount;
	if (logCount < 256 || (++logCount & 0x3F) == 0) {
		mLOG(GBA_SIO, DEBUG, "GBLink GBA gba=%u gb=%u out=%02X in=%02X",
		     GBLinkBridgeGbaCount(driver->bridge), GBLinkBridgeGbCount(driver->bridge), out, in);
		logCount++;
	}

	GBASIONormal8FinishTransfer(sio, in, 0);
}
