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

#if !(defined OSD_W_PIN && defined OSD_EN_PIN && defined OSD_SYNC_PIN)
#error This PICO OSD requires OSD_W_PIN, OSD_EN_PIN and OSD_SYNC_PIN to be defined
#endif

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
static int osd_w_gpio;
static int osd_sync_gpio;
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
    osd__gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_W_PIN));
    if (osd_en_gpio != osd_w_gpio + 1) {
        bprintf("*** OSD_EN_GPIO must be next pin up from OSD_W_GPIO (%d vs %d)", osd_en_gpio, osd_w_gpio);
    }

    osd_sync_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_SYNC_PIN));
    if (osd_sync_gpio != osd_en_gpio + 1) {
        // might relax this... wait GPIO vs wait PINS if single SM, or just separate SMs
        bprintf("*** OSD_SYNC_GPIO must be next pin up from OSD_EN_GPIO (%d vs %d)", osd_sync_gpio, osd_en_gpio);
    }

    bprintf("osd_w gpio %d, osd_en gpio %d, osd_sync gpio %d", osd_w_gpio, osd_en_gpio, osd_sync_gpio);
    // TODO PIO BASE

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
  
#define OSD_W_PIN            PA16
#define OSD_EN_PIN           PA17
#define OSD_SYNC_PIN         PA18

*/

bool timer_callback(repeating_timer_t *rt)
{
    uint8_t *buffer = (uint8_t *)rt->user_data;
    bprintf("buffer = %p, monoBuffer = %p", buffer, monoBuffer);
        
//    pio_sm_set_enabled(osdPio, osd_tx_sm, true);
    //return false;
    return true;
}


static repeating_timer_t rtdata;

void osd_test(void)
{
    osd_test_init();
//     int32_t delay_ms = 20;
    int32_t delay_ms = 1520;
    bprintf("adding timer");
    if (!add_repeating_timer_ms(delay_ms, timer_callback, &monoBuffer[0], &rtdata)) {
        bprintf("*** failed to add timer ***");
    }

    while (true) {
        sleep_ms(1);
    }
}


#else // USE_OSD_SD
// no OSD SD

#endif // USE_OSD_SD
