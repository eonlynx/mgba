/* Copyright (c) 2026 eonlynx
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba/internal/gblink.h>

#include <mgba/internal/gb/gb.h>
#include <mgba/internal/gb/io.h>
#include <mgba/internal/gb/sio.h>

static void _gbGBLinkWriteSB(struct GBSIODriver* driver, uint8_t value) {
	struct GBSIOGBLinkDriver* link = (struct GBSIOGBLinkDriver*) driver;
	link->sb = value;
}

static uint8_t _gbGBLinkWriteSC(struct GBSIODriver* driver, uint8_t value) {
	struct GBSIOGBLinkDriver* link = (struct GBSIOGBLinkDriver*) driver;
	if (GBRegisterSCIsEnable(value) && GBRegisterSCIsShiftClock(value)) {
		// The GB is the clock master: hand our outgoing byte to the GBA and
		// take whatever the GBA was waiting to send.
		link->d.p->pendingSB = GBLinkBridgeExchange(link->bridge, link->sb, false);
	}
	return 0;
}

void GBSIOGBLinkDriverCreate(struct GBSIOGBLinkDriver* driver, struct GBLinkBridge* bridge) {
	memset(driver, 0, sizeof(*driver));
	driver->d.writeSB = _gbGBLinkWriteSB;
	driver->d.writeSC = _gbGBLinkWriteSC;
	driver->bridge = bridge;
}
