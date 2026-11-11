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
// The GB is the clock master and the GBA is the normal-mode 8-bit SIO slave,
// exactly like a DMG/GB/GBC link cable.  Each side pushes the byte it sends
// and pops the byte the other side sent, through a pair of queues.  The GBA
// driver paces itself at the GB link rate (~1 KiB/s), so both sides stay in
// step without needing cycle-level lockstep between the two cores.

#define GB_LINK_QUEUE_SIZE 8192

// The GB keeps clocking bytes while the GBA is not listening (for example,
// while the player is still walking to the cable club counter).  Anything the
// GB sent before the GBA opened the link belongs to a previous "session" and
// must be discarded, otherwise the GBA starts the exchange thousands of bytes
// behind.  A session is considered new when the GBA has not transferred
// anything for a few frames.
#define GB_LINK_SESSION_GAP (4 * 280896) // 4 GBA frames
// One GB frame (4.194304 MHz).  The GB toggles SC off/on for every byte, so
// anything longer than a frame between an enable and the previous activity is
// a genuinely new link session.
#define GB_LINK_GB_IDLE_GAP 70224

struct GBLinkBridge {
	// The GB clocks one byte at a time and reads whatever the GBA currently
	// has latched in its shift register, so the GBA -> GB direction is a
	// single slot, not a queue.  With one push and one pop per exchange a
	// queue never drains: any backlog becomes a permanent delay and the GB
	// keeps reading bytes from dozens of exchanges ago.
	uint8_t gbaLatch;
	// The GB -> GBA direction stays a queue: the GBA only consumes a byte when
	// it completes a transfer, and it can lag behind while it catches up.
	uint8_t gbToGba[GB_LINK_QUEUE_SIZE];
	uint32_t gbToGbaHead;
	uint32_t gbToGbaTail;
	uint32_t gbaCount;
	uint32_t gbCount;
	Mutex mutex;
};

struct GBASIOGBLinkDriver {
	struct GBASIODriver d;
	struct GBLinkBridge* bridge;
	struct mTimingEvent event;
	uint32_t lastTransfer;
	uint32_t starts;
	uint32_t polls;
	bool started;
};

struct GBSIOGBLinkDriver {
	struct GBSIODriver d;
	struct GBLinkBridge* bridge;
	struct mTimingEvent event;
	uint8_t sb;
	uint32_t waitForGba;
	uint32_t waited;
	uint32_t lastCycle;
	bool waiting;
	bool enabled;
};

static inline void GBLinkBridgeInit(struct GBLinkBridge* bridge) {
	memset(bridge, 0, sizeof(*bridge));
	// Until the GBA detects the peer it idles on the connection byte.
	bridge->gbaLatch = 0x02;
	MutexInit(&bridge->mutex);
}

static inline void GBLinkBridgeDeinit(struct GBLinkBridge* bridge) {
	MutexDeinit(&bridge->mutex);
}

static inline void GBLinkBridgeFlush(struct GBLinkBridge* bridge) {
	MutexLock(&bridge->mutex);
	bridge->gbaLatch = 0x02;
	bridge->gbToGbaHead = 0;
	bridge->gbToGbaTail = 0;
	MutexUnlock(&bridge->mutex);
}

static inline bool GBLinkBridgeHasGbByte(struct GBLinkBridge* bridge) {
	bool has;
	MutexLock(&bridge->mutex);
	has = bridge->gbToGbaTail != bridge->gbToGbaHead;
	MutexUnlock(&bridge->mutex);
	return has;
}

static inline uint8_t GBLinkBridgeExchange(struct GBLinkBridge* bridge, uint8_t out, bool fromGba) {
	uint8_t in;
	MutexLock(&bridge->mutex);
	if (fromGba) {
		bridge->gbaCount++;
		// This is what the GB will clock out from now on.
		bridge->gbaLatch = out;
		in = 0xFF;
		if (bridge->gbToGbaTail != bridge->gbToGbaHead) {
			in = bridge->gbToGba[bridge->gbToGbaTail];
			bridge->gbToGbaTail = (bridge->gbToGbaTail + 1) % GB_LINK_QUEUE_SIZE;
		}
	} else {
		bridge->gbCount++;
		uint32_t next = (bridge->gbToGbaHead + 1) % GB_LINK_QUEUE_SIZE;
		if (next != bridge->gbToGbaTail) {
			bridge->gbToGba[bridge->gbToGbaHead] = out;
			bridge->gbToGbaHead = next;
		}
		in = bridge->gbaLatch;
	}
	MutexUnlock(&bridge->mutex);
	return in;
}

void GBASIOGBLinkDriverCreate(struct GBASIOGBLinkDriver* driver, struct GBLinkBridge* bridge);
void GBSIOGBLinkDriverCreate(struct GBSIOGBLinkDriver* driver, struct GBLinkBridge* bridge);

CXX_GUARD_END

#endif
