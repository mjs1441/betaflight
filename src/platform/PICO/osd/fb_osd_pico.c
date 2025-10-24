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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_FB_OSD

#include "drivers/fb_osd_impl.h"
#include "drivers/time.h"
#include "osd_pico.h"

// void    fbOsdHardwareReset(void);
// void    fbOsdPreinit(const struct fbOsdConfig_s *fbOsdConfig);
// void    fbOsdInvert(bool invert);
// void    fbOsdBrightness(uint8_t black, uint8_t white);
// bool    fbOsdBuffersSynced(void);

// *** TODO merge osd_pico.c into fb_osd_pico.c (probably - might tease out some lower level stuff, pio-related)
// osd_pico -> osd_pio, DMA, IRQ or so

static bool inNTSCrange(int n)
{
    return n >= 253 && n <= 255;
}

static bool inPALrange(int n)
{
    return n >= 304 && n <= 306;
}

fbOsdInitStatus_e fbOsdInit(const struct fbOsdConfig_s *fbOsdConfig, const struct vcdProfile_s *vcdProfile)
{
    UNUSED(fbOsdConfig);
    UNUSED(vcdProfile); // TODO
    static bool first = true;
    static int count;

    const int repeatTarget = 100;
    static int repeatCount;
    static int lastHSyncs = -1;
    static uint32_t lastMicros;

    count++;

    if (first) {
        osdPioDetectStart();
        first = false;
        lastMicros = micros();
        return FB_OSD_INIT_INITIALISING;
    }

    UNUSED(lastMicros);
#if 0
    uint32_t now = micros();
    if (cmpTimeUs(now, lastMicros) < 41000) {
        // Reading involves stopping the PIO program
        // Allow time to recover from previous read and have fresh VSync cycle to get a good next read.
        return FB_OSD_INIT_INITIALISING;
    }
#endif
    
    int hSyncs = osdPioCountHSyncs();
    if (!hSyncs) {
        // Invalid - maybe tried to read again too quickly.
        return FB_OSD_INIT_INITIALISING;
    }

#if 1
    if (hSyncs != lastHSyncs) {
        // bprintf("OSD %d detected %d hSyncs", count, hSyncs);
    }
#endif

    // While composite source is warming up, might expect to see hSyncs increasing over a period of seconds
    // from zero to a stable number.
    
    if (lastHSyncs == hSyncs) {
        repeatCount++;
        if (0 == (repeatCount % 5)) {
            bprintf("repeat %d of %d", repeatCount, hSyncs);
        }
    } else {
        repeatCount = 0;
    }
    
    if (repeatCount == repeatTarget) {
        if (inNTSCrange(hSyncs)) {
            bprintf("OSD %d repeat %d of %d (NTSC)", count, repeatCount, hSyncs);
            osdPioStartNTSC();
            return FB_OSD_INIT_OK;
        } else if (inPALrange(hSyncs)) {
            bprintf("OSD %d repeat %d of %d (PAL)", count, repeatCount, hSyncs);
            osdPioStartPAL();
            return FB_OSD_INIT_OK;
        }
    }
    
    lastHSyncs = hSyncs;
    return FB_OSD_INIT_INITIALISING;
}

bool fbOsdReInitIfRequired(bool forceStallCheck)
{
    UNUSED(forceStallCheck);
    return false;
}

// Return true if screen still being transferred
#define DRAWSCREEN_TIME_LIMIT_US 14
bool fbOsdDrawScreen(void)
{
    // Spend no more than DRAWSCREEN_TIME_LIMIT_US on each iteration, will keep
    // on calling in here until we return false for "all done".
    return osdPioDrawScreenUntil(micros() + DRAWSCREEN_TIME_LIMIT_US);
//    return osdDrawScreenUntil(micros() - 1);
}

bool fbOsdWriteFontCharacter(uint8_t char_address, const uint8_t *font_data)
{
    // future: might store fonts in flash...
    UNUSED(char_address);
    UNUSED(font_data);
    return false;
}

    
uint8_t fbOsdGetRowsCount(void)
{
    // *** TODO support NTSC and return appropriate answer here
    return VIDEO_LINES_PAL;
}

void fbOsdWrite(uint8_t x, uint8_t y, uint8_t attr, const char *text)
{
    // *** TODO implement some attr behaviours
    UNUSED(attr);
    osdPioWrite(x, y, text);
}

void fbOsdWriteChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    // *** TODO implement some attr behaviours
    UNUSED(attr);
    osdPioWriteChar(x, y, c);
}

void fbOsdClearScreen(void)
{
    memset(osdCharBuffer, 0x20, OSD_CHAR_BUFFER_LENGTH);
}

void fbOsdRefreshAll(void)
{
    fbOsdReInitIfRequired(true);
    while (fbOsdDrawScreen()) ; // Call draw function until transfer is completed.
}

bool fbOsdBufferInUse(void)
{
    return !osdPioBufferAvailable();
}

bool fbOsdLayerSupported(displayPortLayer_e layer)
{
    return layer == DISPLAYPORT_LAYER_FOREGROUND;
}

bool fbOsdLayerSelect(displayPortLayer_e layer)
{
    return fbOsdLayerSupported(layer); // no action
}

bool fbOsdLayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    UNUSED(destLayer);
    UNUSED(sourceLayer);
    return false;
}

void fbOsdSetBackgroundType(displayPortBackground_e backgroundType)
{
    // Not currently supporting different background types.
    UNUSED(backgroundType);
}

#endif // USE_FB_OSD
