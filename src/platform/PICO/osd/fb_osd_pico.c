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
#include "osd_pico.h"

// void    fbOsdHardwareReset(void);
// void    fbOsdPreinit(const struct fbOsdConfig_s *fbOsdConfig);
// void    fbOsdInvert(bool invert);
// void    fbOsdBrightness(uint8_t black, uint8_t white);
// bool    fbOsdBuffersSynced(void);

// *** TODO merge osd_pico.c into fb_osd_pico.c (probably - might tease out some lower level stuff, pio-related)
// osd_pico -> osd_pio, DMA, IRQ or so

fbOsdInitStatus_e fbOsdInit(const struct fbOsdConfig_s *fbOsdConfig, const struct vcdProfile_s *vcdProfile)
{
    UNUSED(fbOsdConfig);
    UNUSED(vcdProfile);
    return FB_OSD_INIT_NOT_FOUND;
}

bool fbOsdReInitIfRequired(bool forceStallCheck)
{
    UNUSED(forceStallCheck);
    return false;
}

// Return true if screen still being transferred
bool fbOsdDrawScreen(void)
{
    // static time, spend no more than... 10?us per iteration
    return true;
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
    return osdBuffer1Safe();
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

bool fbOsdIsDeviceDetected(void)
{
    // *** TODO
    return true;
}

void fbOsdSetBackgroundType(displayPortBackground_e backgroundType)
{
    // Not currently supporting different background types.
    UNUSED(backgroundType);
}

#endif // USE_FB_OSD
