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
    const int ntscHsyncsMin = 253;
    const int ntscHsyncsMax = 254;
    return n >= ntscHsyncsMin - 1 && n <= ntscHsyncsMax + 1;
}

static bool inPALrange(int n)
{
    const int palHsyncs = 305;
    return n >= palHsyncs - 1 && n <= palHsyncs + 1;
}

fbOsdInitStatus_e fbOsdInit(const struct fbOsdConfig_s *fbOsdConfig, const struct vcdProfile_s *vcdProfile)
{
    UNUSED(fbOsdConfig);
    UNUSED(vcdProfile); // TODO
    static bool first = true;

    const int repeatTarget = 15;
    static int repeatCount;
    static int lastHSyncs = -1;
    static int lastRange; // 1 = NTSC range, -1 = PAL range, 0 = neither


    if (first) {
        osdPioDetectStart();
        first = false;
        return FB_OSD_INIT_INITIALISING;
    }
    
    int hSyncs = osdPioCountHSyncs();
    
#ifdef PICO_TRACE
    static int count;
    count++;
    if (false) { // ((count % 100017 == 0) || (hSyncs && hSyncs != lastHSyncs)) {
        bprintf("OSD %d detected %d hSyncs", count, hSyncs);
    }
#endif

    if (!hSyncs) {
        // Invalid - maybe tried to read again too quickly.
        return FB_OSD_INIT_INITIALISING;
    }

#ifdef PICO_TRACE    
    if (count % 7 == 0 && hSyncs != lastHSyncs) {
        bprintf("OSD %d detected %d hSyncs", count, hSyncs);
    }
#endif

    // While composite source is warming up, might expect to see hSyncs increasing over a period of seconds
    // from zero to a stable number.
    int range = inNTSCrange(hSyncs) ? 1 : inPALrange(hSyncs) ? -1 : 0;
    if (lastRange == range) {
        repeatCount++;
        if (0 == (repeatCount % 4)) {
            bprintf("OSD repeat %d of %d (%s)", repeatCount, hSyncs, range == 1 ? "NTSC" : range == -1 ? "PAL" : "neither PAL nor NTSC");
        }
    } else {
        repeatCount = 0;
    }
    
    if (repeatCount == repeatTarget) {
        if (range == 1) {
            bprintf("OSD seen %d of %d (NTSC)", repeatCount, hSyncs);
            osdPioStartNTSC();
            return FB_OSD_INIT_OK;
        } else if (range == -1) {
            bprintf("OSD seen %d of %d (PAL)", repeatCount, hSyncs);
            osdPioStartPAL();
            return FB_OSD_INIT_OK;
        }
    }
    
    lastHSyncs = hSyncs;
    lastRange = range;
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
