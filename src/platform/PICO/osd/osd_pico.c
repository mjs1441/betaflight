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
#include "drivers/io_impl.h"

#ifdef USE_OSD_SD

#include "osd/osd.h"

#include "hardware/irq.h"
#include "hardware/pio.h"

#include "osd_tx.pio.h"

// each char 12 x 18 pixels
#define PICO_OSD_CHAR_WIDTH  12
#define PICO_OSD_CHAR_HEIGHT 18

// chars OSD_SD_ROWS x OSD_SD_COLS (30 x 16)
// 360 / 8 = 45 x 288
#define PICO_OSD_BUF_WIDTH   (OSD_SD_COLS * PICO_OSD_CHAR_WIDTH / 8)
#define PICO_OSD_BUF_HEIGHT  (OSD_SD_ROWS * PICO_OSD_CHAR_HEIGHT)
#define PICO_OSD_BUF_LENGTH  (PICO_OSD_BUF_WIDTH * PICO_OSD_BUF_HEIGHT)

static const PIO osdPio = PIO_INSTANCE(PIO_OSD_INDEX);
static int osd_tx_offset;
static int osd_en_gpio;
static int osd_tx_sm;

// 360 x 288
static uint8_t monoBuffer[PICO_OSD_BUF_LENGTH];

void osd_test_init(void)
{
    bprintf("osd_test_init");
    for (int i=0; i<PICO_OSD_BUF_LENGTH; ++i) {
        int y = i / PICO_OSD_BUF_WIDTH;
        int x = (i % PICO_OSD_BUF_WIDTH) * 8; // approx. pixels
        int dd = (x-180)*(x-180)+(y-144)*(y-144);
        monoBuffer[i] = dd < 15000 ? 0xff : 0;
    }

    osd_en_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_EN_PIN));
    osd_tx_offset = pio_add_program(osdPio, &osd_tx_program);
    osd_tx_sm = pio_claim_unused_sm(osdPio, false);
    if (osd_tx_sm < 0) {
        bprintf("*** pico osd tx failed to claim state machine");
        return;
    }

    // osd_tx_program_init(osdPio, osd_tx, osd_tx_offset, osd_en_gpio);
    pio_sm_config config = osd_tx_program_get_default_config(osd_tx_offset); // default config with wrap set
    pio_sm_set_consecutive_pindirs(osdPio, osd_tx_sm, osd_en_gpio, 1, true /* output */);
    sm_config_set_out_pins(&config, osd_en_gpio, 1);
    //static void sm_config_set_out_shift (pio_sm_config * c, bool shift_right, bool autopull, uint pull_threshold) [inline], [static]
    sm_config_set_out_shift(&config, true, false, 32);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
    int pioclock = (int)1e6; // TODO
    float div = (float)SystemCoreClock / pioclock;
    sm_config_set_clkdiv(&config, div);
    pio_sm_init(osdPio, osd_tx_sm, osd_tx_offset, &config);
}

/*
  #define OSD_W_PIN            PA32
  #define OSD_EN_PIN           PA33
  // #define OSD_SYNC_PIN
*/

void osd_test(void)
{
    osd_test_init();
    pio_sm_set_enabled(osdPio, osd_tx_sm, true);
}


#else // USE_OSD_SD
// no OSD SD

#endif // USE_OSD_SD
