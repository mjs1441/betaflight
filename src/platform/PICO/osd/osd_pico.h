/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "osd/osd.h"
#include "pg/vcd.h"

// TODO ***  export function clearscreen instead of osdCharBuffer
#define OSD_CHAR_BUFFER_LENGTH 480
extern uint8_t osdCharBuffer[OSD_CHAR_BUFFER_LENGTH];

int osdPioCountHSyncs(void);
void osdPioDetectStart(void);
void osdPioStartNTSC(void);
void osdPioStartPAL(void);

void osdPioWriteChar(uint8_t x, uint8_t y, uint8_t c);
void osdPioWrite(uint8_t x, uint8_t y, const char *text);
bool osdPioBufferAvailable(void);
bool osdPioDrawScreenUntil(uint32_t limit_micros);
int osdPioRowsCount(void);
bool osdPioRenderItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY);

// testing, probably don't need
// bool osdPioInitDevice( const struct vcdProfile_s *vcdProfile);
void osdPioEnableDevice(void);
void osdPioDisableDevice(void);
