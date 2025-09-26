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

#include "platform.h"

#include "drivers/io.h"
#include "hardware/irq.h"
#include "hardware/pio.h"

#ifdef USE_OSD_SD

// each char 12 x 18 pixels
#define PICO_OSD_CHAR_WIDTH  12
#define PICO_OSD_CHAR_HEIGHT 18

// chars OSD_SD_ROWS x OSD_SD_COLS (30 x 16)
// 360 / 8 = 45 x 288
#define PICO_OSD_BUF_WIDTH   (OSD_SD_COLS * PICO_OSD_CHAR_WIDTH / 8)
#define PICO_OSD_BUF_HEIGHT  (OSD_SD_ROWS * PICO_OSD_CHAR_HEIGHT)
#define PICO_OSD_BUF_LENGTH  (PICO_OSD_BUF_WIDTH * PICO_OSD_BUF_HEIGHT)

static uint8_t monoBuffer[PICO_OSD_BUF_LENGTH];




#else // USE_OSD_SD
// no OSD SD

#endif // USE_OSD_SD
