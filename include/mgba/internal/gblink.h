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

struct GBLinkBridge {
	uint8_t gbaToGb[GB_LINK_QUEUE_SIZE];
	uint32_t gbaToGbHead;
	uint32_t gbaToGbTail;
	uint8_t gbToGba[GB_LINK_QUEUE_SIZE];
	uint32_t gbToGbaHead;
	uint32_t gbToGbaTail;
	Mutex mutex;
};

struct GBASIOGBLinkDriver {
	struct GBASIODriver d;
	struct GBLinkBridge* bridge;
	struct mTimingEvent event;
};

struct GBSIOGBLinkDriver {
	struct GBSIODriver d;
	struct GBLinkBridge* bridge;
	uint8_t sb;
};

static inline void GBLinkBridgeInit(struct GBLinkBridge* bridge) {
	memset(bridge, 0, sizeof(*bridge));
	MutexInit(&bridge->mutex);
}

static inline void GBLinkBridgeDeinit(struct GBLinkBridge* bridge) {
	MutexDeinit(&bridge->mutex);
}

static inline uint8_t GBLinkBridgeExchange(struct GBLinkBridge* bridge, uint8_t out, bool fromGba) {
	uint8_t in = 0xFF;
	uint32_t next;
	MutexLock(&bridge->mutex);
	if (fromGba) {
		next = (bridge->gbaToGbHead + 1) % GB_LINK_QUEUE_SIZE;
		if (next != bridge->gbaToGbTail) {
			bridge->gbaToGb[bridge->gbaToGbHead] = out;
			bridge->gbaToGbHead = next;
		}
		if (bridge->gbToGbaTail != bridge->gbToGbaHead) {
			in = bridge->gbToGba[bridge->gbToGbaTail];
			bridge->gbToGbaTail = (bridge->gbToGbaTail + 1) % GB_LINK_QUEUE_SIZE;
		}
	} else {
		next = (bridge->gbToGbaHead + 1) % GB_LINK_QUEUE_SIZE;
		if (next != bridge->gbToGbaTail) {
			bridge->gbToGba[bridge->gbToGbaHead] = out;
			bridge->gbToGbaHead = next;
		}
		if (bridge->gbaToGbTail != bridge->gbaToGbHead) {
			in = bridge->gbaToGb[bridge->gbaToGbTail];
			bridge->gbaToGbTail = (bridge->gbaToGbTail + 1) % GB_LINK_QUEUE_SIZE;
		}
	}
	MutexUnlock(&bridge->mutex);
	return in;
}

void GBASIOGBLinkDriverCreate(struct GBASIOGBLinkDriver* driver, struct GBLinkBridge* bridge);
void GBSIOGBLinkDriverCreate(struct GBSIOGBLinkDriver* driver, struct GBLinkBridge* bridge);

CXX_GUARD_END

#endif
