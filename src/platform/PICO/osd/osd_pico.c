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

#ifdef USE_OSD_SD

#include <string.h>

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/time.h"

#include "drivers/system.h"


#if !(defined OSD_W_PIN && defined OSD_EN_PIN && defined OSD_SYNC_PIN)
#error This PICO OSD requires OSD_W_PIN, OSD_EN_PIN and OSD_SYNC_PIN to be defined
#endif

#include "osd/osd.h"

#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "osd_tx.pio.h"

// each char 12 x 18 pixels
#define PICO_OSD_CHAR_WIDTH  12
#define PICO_OSD_CHAR_HEIGHT 18

// chars OSD_SD_ROWS x OSD_SD_COLS (30 x 16)
// 360 / 8 = 45 x 288
// 2 bits per pixel
#define PICO_OSD_BPP         2
//#define PICO_OSD_BUF_WIDTH   (OSD_SD_COLS * PICO_OSD_CHAR_WIDTH / 8)
#define ROUND_WORD(x)        (4 * (((x) + 3)/4))
//#define PICO_OSD_BUF_WIDTH   ROUND_WORD(OSD_SD_COLS * PICO_OSD_CHAR_WIDTH * PICO_OSD_BPP / 8)
#define PICO_OSD_BUF_WIDTH   ROUND_WORD(360 * PICO_OSD_BPP / 8)
#define PICO_OSD_BUF_LINEWORDS (PICO_OSD_BUF_WIDTH/4)
////#define PICO_OSD_BUF_HEIGHT  (OSD_SD_ROWS * PICO_OSD_CHAR_HEIGHT)
#define PICO_OSD_BUF_HEIGHT  256
#define PICO_OSD_BUF_LENGTH  (PICO_OSD_BUF_WIDTH * PICO_OSD_BUF_HEIGHT)
#define PICO_OSD_BUF_WORDS   (PICO_OSD_BUF_LENGTH / 4)

static const PIO osdPio = PIO_INSTANCE(PIO_OSD_INDEX);
static const uint osdPioIrq = PIO_IRQ_NUM(osdPio, 0);
static int osd_tx_offset;
static int osd_en_gpio;
static int osd_w_gpio;
static int osd_sync_gpio;
static int osd_tx_sm;

// 360 x 288 x 2 bits per pixel
static uint8_t osdBuffer[PICO_OSD_BUF_LENGTH];
static uint8_t osdBuffer2[PICO_OSD_BUF_LENGTH];

static int osd_dma_channel;

 void plot(int x, int y, int c)
{
    (void)osdBuffer2;
    // c =  0 -> transparent (no overlay)   W=any EN=0
    // c =  1 -> black                      W=0   EN=1
    // c =  2 -> white                      W=1   EN=1
    uint8_t * pbyte = osdBuffer + PICO_OSD_BUF_WIDTH * y;
    pbyte += (int)(x/4); // 4 pixels per byte
    static uint8_t masks[4] = {0b00000011, 0b00001100, 0b00110000, 0b11000000};
    static uint8_t  cols[4] = {0b00000000, 0b10101010, 0b11111111, 0b00000000};
    uint8_t mask = masks[x%4];
    uint8_t col = cols[c];
    *pbyte = ((*pbyte) &(~mask)) | (mask&col);
}

static void vsync_callback(void);

void osd_test_init(void)
{
    bprintf("osd_test_init");
    bprintf("pbw %d, pbh %d, bpl %d", PICO_OSD_BUF_WIDTH, PICO_OSD_BUF_HEIGHT, PICO_OSD_BUF_LENGTH);
    for (int i=0; i<PICO_OSD_BUF_LENGTH; ++i) {
        int y = i / PICO_OSD_BUF_WIDTH;
        int x = (i % PICO_OSD_BUF_WIDTH) * 4; // approx. pixels
        int dd = (x-184)*(x-184)+(y-128)*(y-128);
//        monoBuffer[i] = dd < 15000 ? 0xff : 0;
        osdBuffer[i] = dd < 15000 ? (dd < 3720 ? 0b10101010 : 0xff) : 0;
        osdBuffer[i] = 0xff; // dd < 15000 ? (dd < 3720 ? 0b10101010 : 0xff) : 0;
        osdBuffer[i] = 0;
//        (void)dd;
    }


    
#if 0
    for (int i=0; i<368; ++i) {
        for (int j=0; j<256; ++j) {
            plot(i,j, ((j%100)<10 ? (j%2)+1 : 0)); // (int)((13*j + i/37))%4);
        }
    }

    
#endif
    
#if 1
    // this pattern particularly hard for small old screen
    for (int i=0; i<360; ++i) {
        plot(i, 256-1, 1);
        plot(i, 256-2, 1);
        plot(i, 256-3, 1);
        plot(i, 256-3, 1);
        plot(i, 256-4, 2);
        plot(i, 256-5, 2);
        plot(i, 0, 1);
        plot(i, 1, 1);
        plot(i, 2, 1);
        plot(i, 3, 2);
        plot(i, 3, 2);
        plot(i, 4, 2);
    }

    for (int i=0; i<256; ++i) { 
        plot(0, i, 1);
        plot(1, i, 1);
        plot(2, i, 2);
        plot(3, i, 2);
        plot(359, i, 2);
        plot(358, i, 2);
        plot(357, i, 1);
        plot(356, i, 1);
        plot(360, i, 2);
        plot(361, i, 2);
        plot(362, i, 2);
        plot(363, i, 2);
        plot(364, i, 2);
        plot(365, i, 2);
        plot(366, i, 2);
        plot(367, i, 2);
    }
#endif
    
    osd_en_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_EN_PIN));
    osd_w_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_W_PIN));
    if (osd_en_gpio != osd_w_gpio + 1) {
        bprintf("*** OSD_EN_GPIO must be next pin up from OSD_W_GPIO (%d vs %d)", osd_en_gpio, osd_w_gpio);
    }

    osd_sync_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_SYNC_PIN));
    if (osd_sync_gpio != osd_en_gpio + 1) {
        // might relax this... wait GPIO vs wait PINS if single SM, or just separate SMs
        bprintf("*** OSD_SYNC_GPIO must be next pin up from OSD_EN_GPIO (%d vs %d)", osd_sync_gpio, osd_en_gpio);
    }

    bprintf("osd_w gpio %d, osd_en gpio %d, osd_sync gpio %d", osd_w_gpio, osd_en_gpio, osd_sync_gpio);
    // *** TODO PIO BASE

    osd_tx_offset = pio_add_program(osdPio, &osd_tx_program);
    osd_tx_sm = pio_claim_unused_sm(osdPio, false);
    if (osd_tx_sm < 0) {
        bprintf("*** pico osd tx failed to claim state machine");
        return;
    }

    // set up for outputs from PIO
    pio_gpio_init(osdPio, osd_w_gpio);
    pio_gpio_init(osdPio, osd_en_gpio);

    // [00:37:44.679969 0.002269] osd_w gpio 16, osd_en gpio 17, osd_sync gpio 18
    pio_sm_config config = osd_tx_program_get_default_config(osd_tx_offset); // default config with wrap set
    pio_sm_set_consecutive_pindirs(osdPio, osd_tx_sm, osd_w_gpio, 2, true /* output */);
    pio_sm_set_consecutive_pindirs(osdPio, osd_tx_sm, osd_sync_gpio, 1, false /* input */);
    sm_config_set_in_pin_base(&config, osd_sync_gpio); // in PIN set SYNC (for WAIT)
    sm_config_set_in_pin_count(&config, 1);
    sm_config_set_jmp_pin(&config, osd_sync_gpio);     // jmp PIN is SYNC
    sm_config_set_set_pins(&config, osd_w_gpio, 2);    // set PIN set W, EN
    sm_config_set_out_pins(&config, osd_w_gpio, 2);    // out PIN set W, EN

    // * TODO auto pull for OSR, or not

    sm_config_set_out_shift(&config, true, false, 32); // no autopull
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);

    int pioclock = (int)75e6; // TODO
    float div = (float)SystemCoreClock / pioclock;
    bprintf("pio clock div = %f", (double)div);
    sm_config_set_clkdiv(&config, div);
    pio_sm_init(osdPio, osd_tx_sm, osd_tx_offset, &config);

    /*
      must arrange ISR to contain 359 = h pixels - 1
 stop (or not started yet), clear fifos
 send 359 to SM (put in TX fifo)
 pio_sm_exec_wait_blocking(pio, sm, [pull])
 pio_sm_exec_wait_blocking(pio, sm, [mov isr, osr])
    */

#if 0
    // prepare value for horiz pixel loop
//    pio_sm_put(osdPio, osd_tx_sm, 359);
    pio_sm_put(osdPio, osd_tx_sm, 255); // see how square...
//    pio_sm_put(osdPio, osd_tx_sm, 344);
#else
    // prepare value for vert pixel loop
    pio_sm_put(osdPio, osd_tx_sm, 255);
#endif
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_pull(false, false));
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_mov(pio_isr, pio_osr));

    pio_set_irq0_source_enabled(osdPio, pis_interrupt0, true); // enable state machine IRQ 0 => system irq PIO_thisone_IRQ_0
    irq_set_exclusive_handler(osdPioIrq, vsync_callback);
    irq_set_enabled(osdPioIrq, true);


    /*
    BF DMA thing, with ownership

// --- DMA Configuration ---
    const dmaIdentifier_e dma_id = dmaGetFreeIdentifier();
    if (dma_id == DMA_NONE || !dmaAllocate(dma_id, OWNER_LED_STRIP, 0)) {
        return false;
    }
    dma_chan = DMA_IDENTIFIER_TO_CHANNEL(dma_id);
    */

    osd_dma_channel = dma_claim_unused_channel(false);
    if (!osd_dma_channel) {
        bprintf("**** failed to claim dma channel for osd pico");
        return;
    }
    
    dma_channel_config c = dma_channel_get_default_config(osd_dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(osdPio, osd_tx_sm, true));

    dma_channel_configure(
        osd_dma_channel,
        &c,
        &osdPio->txf[osd_tx_sm],  // Write address (PIO TX FIFO)
        osdBuffer2,                // Read address
        PICO_OSD_BUF_WORDS,        // Number of transfers
        false                      // Don't start immediately
    );


    /*
    if dma has handler
// disable the channel on IRQ0
dma_channel_set_irq0_enabled(channel, false);
// abort the channel
dma_channel_abort(channel);
// clear the spurious IRQ (if there was one)
dma_channel_acknowledge_irq0(channel);
// re-enable the channel on IRQ0
dma_channel_set_irq0_enabled(channel, true);

otherwise just dma_channel_abort
    */
}

static void vsync_callback(void)
{
    // 50 per second (PAL)
    static int c=0;
    // Need to clear the IRQ flag state from the PIO.
    // This just writes a 1 to a register, doesn't mess with SM execution    
    pio_interrupt_clear(osdPio, 0);

    // * stop any dma in progress
    // * clear pio tx fifo
    // * flip buffer (or alternate buffers)
    // * start dma

    dma_channel_abort(osd_dma_channel);
    pio_sm_clear_fifos(osdPio, osd_tx_sm);
    memcpy(osdBuffer2, osdBuffer, PICO_OSD_BUF_LENGTH);
    dma_channel_start(osd_dma_channel);

    if (++c % 250 == 0) {
        bprintf("%d vsync_callback",c);
    }
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
    bprintf("buffer = %p, osdBuffer = %p", buffer, osdBuffer);
        
//    pio_sm_set_enabled(osdPio, osd_tx_sm, true);
    //return false;
    return true;
}


static repeating_timer_t rtdata;

static void enable(void)
{
    pio_sm_set_enabled(osdPio, osd_tx_sm, true);
    pio_gpio_init(osdPio, osd_w_gpio);
    pio_gpio_init(osdPio, osd_en_gpio);
//    gpio_set_pulls(osd_w_gpio, true, false);
}

static void disable(void)
{
    pio_sm_set_enabled(osdPio, osd_tx_sm, false);
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_set(pio_pins, 0));
    gpio_init(osd_w_gpio);
    gpio_init(osd_en_gpio);
//    gpio_set_pulls(osd_w_gpio, true, false);
}

void osd_test(void)
{
    osd_test_init();
//     int32_t delay_ms = 20;
//    int32_t delay_ms = 1520;
    int32_t delay_ms = 51520;
    
    bprintf("adding timer");
    if (!add_repeating_timer_ms(delay_ms, timer_callback, &osdBuffer[0], &rtdata)) {
        bprintf("*** failed to add timer ***");
    }

    int pc;
    int pca[50];
    int hist[32];
    for (int i=0; i<32; ++i) hist[i] = 0;
    bprintf("SM offset is %d", osd_tx_offset);
    while (true) {
        pc = pio_sm_get_pc(osdPio, osd_tx_sm); bprintf("A pc = %d less offset = %d", pc, pc - osd_tx_offset);
        delay(893);
//        delay(13893);
        delay(13);
        pc = pio_sm_get_pc(osdPio, osd_tx_sm); bprintf("B pc = %d less offset = %d", pc, pc - osd_tx_offset);

        bprintf("      ENABLE");
        enable();
#if 0
        int cc = 0;
        int cj = 0;
        while (1) {
            for (int j=0; j<PICO_OSD_BUF_HEIGHT; ++j) {
                int jj = PICO_OSD_BUF_WIDTH * j;
                for (int i=0; i<PICO_OSD_BUF_LINEWORDS; ++i) {
                    uint32_t w = *(uint32_t*)(&osdBuffer[4*i + jj]);
                    pio_sm_put_blocking(osdPio, osd_tx_sm, w);
                    cj++;
                }
            }
            cc += 1;
            if (cc%200 == 0) { bprintf("cc %d (%d words)", cc, cj); }
        }
#endif

#if 1
        (void)hist;
        (void)pca;
#else
        while (1) {
            uint32_t x = getCycleCounter();
            if ((x%13) == 1 || (x % 17) == 7) {
                for (int i=0; i<10000000; ++i) {
                    hist[osdPio->sm[osd_tx_sm].addr]++;
                }
                bprintf("-----");
                for (int i=0; i<32-osd_tx_offset; ++i) {
                    bprintf("%d: %d", i, hist[i+osd_tx_offset]);
                }
                bprintf("-----");
            }
        }
        
        for (int i=0; i<50; ++i) {
            pca[i] = pio_sm_get_pc(osdPio, osd_tx_sm);
        }
        for (int i=0; i<50; ++i) {
            bprintf("pc = %d", pca[i] - osd_tx_offset);
        }
        bprintf(".");
        delay(7);
        while (pio_sm_get_pc(osdPio, osd_tx_sm) < osd_tx_offset + 17) {
            ;
        }
        for (int i=0; i<50; ++i) {
            pca[i] = pio_sm_get_pc(osdPio, osd_tx_sm);
        }
        for (int i=0; i<50; ++i) {
            bprintf("pc = %d", pca[i] - osd_tx_offset);
        }
        
#endif
            
        pc = pio_sm_get_pc(osdPio, osd_tx_sm); bprintf("C pc = %d less offset = %d", pc, pc - osd_tx_offset);
        delay(4997);
        pc = pio_sm_get_pc(osdPio, osd_tx_sm); bprintf("D pc = %d less offset = %d", pc, pc - osd_tx_offset);

        bprintf("      DISABLE");
        disable();
        pc = pio_sm_get_pc(osdPio, osd_tx_sm); bprintf("E pc = %d less offset = %d", pc, pc - osd_tx_offset);
    }
}


#else // USE_OSD_SD
// no OSD SD

#endif // USE_OSD_SD
