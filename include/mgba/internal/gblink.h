/* Copyright (c) 2026 eonlynx
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef M_GBLINK_H
#define M_GBLINK_H

#include <mgba-util/common.h>

#include <string.h>

CXX_GUARD_START

#include <mgba-util/threading.h>
#include <mgba/core/timing.h>
#include <mgba/gba/interface.h>
#include <mgba/gb/interface.h>

// GBA <-> GB link bridge.
//
// This is a passive cable, not a protocol: the GBA is the clock master (its
// software drives normal 8-bit SIO with the internal clock) and the GB is the
// external-clock slave.  A transfer only happens when the GBA starts one, so
// the GB cannot run ahead, which is what makes this work between two emulator
// cores running in separate threads.
//
// Only two things are shared: the byte the GB wrote to SB (plus whether it is
// armed and waiting for a clock) and the byte the GBA clocked out last.

struct GBLinkBridge {
	uint8_t gbaLatch;   // last byte clocked out by the GBA
	uint8_t gbaData;    // byte clocked out by the last *armed* transfer
	uint8_t gbSb;       // last byte the GB wrote to SB
	bool gbArmed;       // the GB is armed and waiting for the GBA to clock it
	uint32_t gbaCount;  // completed GBA transfers
	uint32_t gbCount;   // completed GB transfers
	Mutex mutex;
};

struct GBASIOGBLinkDriver {
	struct GBASIODriver d;
	struct GBLinkBridge* bridge;
	struct mTimingEvent event;
	bool started;
};

struct GBSIOGBLinkDriver {
	struct GBSIODriver d;
	struct GBLinkBridge* bridge;
	struct mTimingEvent event;
	uint8_t sb;
	uint32_t waitForGba;
	uint32_t waited;
	bool waiting;
};

static inline void GBLinkBridgeInit(struct GBLinkBridge* bridge) {
	memset(bridge, 0, sizeof(*bridge));
	bridge->gbaLatch = 0xFF;
	bridge->gbaData = 0xFF;
	bridge->gbSb = 0xFF;
	MutexInit(&bridge->mutex);
}

static inline void GBLinkBridgeDeinit(struct GBLinkBridge* bridge) {
	MutexDeinit(&bridge->mutex);
}

// Drops any half-finished transfer when a link session ends.
static inline void GBLinkBridgeReset(struct GBLinkBridge* bridge) {
	MutexLock(&bridge->mutex);
	bridge->gbaLatch = 0xFF;
	bridge->gbaData = 0xFF;
	bridge->gbSb = 0xFF;
	bridge->gbArmed = false;
	bridge->gbaCount = 0;
	bridge->gbCount = 0;
	MutexUnlock(&bridge->mutex);
}

static inline void GBLinkBridgeSetGbSb(struct GBLinkBridge* bridge, uint8_t sb) {
	MutexLock(&bridge->mutex);
	bridge->gbSb = sb;
	MutexUnlock(&bridge->mutex);
}

static inline void GBLinkBridgeArm(struct GBLinkBridge* bridge, bool armed) {
	MutexLock(&bridge->mutex);
	bridge->gbArmed = armed;
	MutexUnlock(&bridge->mutex);
}

// Publishes the GB's byte and arms the transfer atomically, returning the
// number of GBA clocks seen so far.  The GB then waits for gbaCount + 1, so a
// clock cannot slip between "publish the byte" and "start waiting for it".
static inline uint32_t GBLinkBridgeArmWithCount(struct GBLinkBridge* bridge, uint8_t sb) {
	uint32_t count;
	MutexLock(&bridge->mutex);
	bridge->gbSb = sb;
	bridge->gbArmed = true;
	count = bridge->gbaCount;
	MutexUnlock(&bridge->mutex);
	return count;
}

static inline uint32_t GBLinkBridgeGbaCount(struct GBLinkBridge* bridge) {
	uint32_t count;
	MutexLock(&bridge->mutex);
	count = bridge->gbaCount;
	MutexUnlock(&bridge->mutex);
	return count;
}

static inline uint32_t GBLinkBridgeGbCount(struct GBLinkBridge* bridge) {
	uint32_t count;
	MutexLock(&bridge->mutex);
	count = bridge->gbCount;
	MutexUnlock(&bridge->mutex);
	return count;
}

static inline uint8_t GBLinkBridgeGbaLatch(struct GBLinkBridge* bridge) {
	uint8_t byte;
	MutexLock(&bridge->mutex);
	byte = bridge->gbaLatch;
	MutexUnlock(&bridge->mutex);
	return byte;
}

static inline bool GBLinkBridgeHasGbSb(struct GBLinkBridge* bridge) {
	bool has;
	MutexLock(&bridge->mutex);
	has = bridge->gbArmed;
	MutexUnlock(&bridge->mutex);
	return has;
}

static inline uint8_t GBLinkBridgeGbSb(struct GBLinkBridge* bridge) {
	uint8_t sb;
	MutexLock(&bridge->mutex);
	sb = bridge->gbSb;
	MutexUnlock(&bridge->mutex);
	return sb;
}

// GBA side: one master transfer finished.  The master never stalls, exactly
// like the real cable; a transfer that finds nothing armed still completes and
// reads SERIAL_NO_DATA_BYTE (0xFE), which the protocol ignores.
//
// Only a transfer that actually clocked an armed GB byte counts as a completed
// GB transfer, which keeps the two streams one byte to one byte.  That
// transfer's outgoing byte is stored in gbaData, because by the time the GB
// notices the clock the GBA may already have sent later (unarmed) bytes, and
// gbaLatch would no longer be the byte for this transfer.
static inline void GBLinkBridgeGbaExchange(struct GBLinkBridge* bridge, uint8_t out, uint8_t* in) {
	MutexLock(&bridge->mutex);
	bridge->gbaLatch = out;
	if (bridge->gbArmed) {
		bridge->gbaData = out;
		*in = bridge->gbSb;
		bridge->gbArmed = false;
		++bridge->gbaCount;
	} else {
		*in = 0xFE;
	}
	MutexUnlock(&bridge->mutex);
}

// GB side: complete the armed transfer once the GBA has clocked it.
static inline bool GBLinkBridgeGbFinish(struct GBLinkBridge* bridge, uint32_t waitFor, uint8_t* in) {
	bool ok;
	MutexLock(&bridge->mutex);
	ok = (int32_t) (bridge->gbaCount - waitFor) >= 0;
	if (ok) {
		*in = bridge->gbaData;
		++bridge->gbCount;
	}
	MutexUnlock(&bridge->mutex);
	return ok;
}

void GBASIOGBLinkDriverCreate(struct GBASIOGBLinkDriver* driver, struct GBLinkBridge* bridge);
void GBSIOGBLinkDriverCreate(struct GBSIOGBLinkDriver* driver, struct GBLinkBridge* bridge);

CXX_GUARD_END

#endif
