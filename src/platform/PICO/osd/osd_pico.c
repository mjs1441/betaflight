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

//#ifdef USE_OSD_SD
#ifdef TEST_PIO_OSD

#include <string.h>
#include <stdlib.h>

#include "common/printf.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "flight/imu.h"

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
/*
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
*/

// 23 -> 23*4*4 = 368 pixels -> 30.67 chars
// 288 for PAL field
// _BUF_ in bytes
#define PICO_OSD_LINE_WORDS 23
#define PICO_OSD_BUF_WIDTH (PICO_OSD_LINE_WORDS*4)
#define PICO_OSD_BUF_HEIGHT 288
//#define PICO_OSD_BUF_HEIGHT 270
//#define PICO_OSD_BUF_HEIGHT 256
//#define PICO_OSD_BUF_HEIGHT 272
//#define PICO_OSD_BUF_HEIGHT 266
#define PICO_OSD_BUF_LENGTH  (PICO_OSD_BUF_WIDTH * PICO_OSD_BUF_HEIGHT)
#define PICO_OSD_BUF_WORDS   (PICO_OSD_BUF_LENGTH / 4)

static const PIO osdPio = PIO_INSTANCE(PIO_OSD_INDEX);
static const uint osdPioIrq = PIO_IRQ_NUM(osdPio, 0);
static const int fb_nx = PICO_OSD_BUF_WIDTH * 4;
static const int fb_ny = PICO_OSD_BUF_HEIGHT;

static int osd_tx_offset;
static int osd_en_gpio;
static int osd_w_gpio;
static int osd_sync_gpio;
static int osd_tx_sm;

// 360 x 288 x 2 bits per pixel
// **** TODO uint32_t aligned
// TODO faster memcpy
// currently building with no-builtin-memcpy
// and gcc13.3 with nanolib -> just does byte copy (even when known aligned)
// (also gcc14.3)
__attribute__((aligned(4))) static uint32_t osdBuffer1W[PICO_OSD_BUF_LENGTH/4];
__attribute__((aligned(4))) static uint32_t osdBuffer2W[PICO_OSD_BUF_LENGTH/4];
static uint8_t* osdBuffer1 = (uint8_t *)osdBuffer1W;
static uint8_t* osdBuffer2 = (uint8_t *)osdBuffer2W;

static const uint32_t zero;
static int dma_chan_buf1_to_buf2;
static int dma_chan_zero_to_buf1;
static int dma_chan_buf2_to_fifo;

static volatile bool in_safe_zone;
static volatile uint32_t safe_zone_period;
static volatile uint32_t sza;
static volatile uint32_t szb;
static volatile uint32_t szc;
static volatile uint32_t szd;
static volatile uint32_t sze;

static const int charsPerLine = 30;
static const int charLines = 16; // enough for VIDEO_LINES_PAL = 16 and VIDEO_LINES_NTSC = 13
static const int numChars = charsPerLine * charLines;

//static uint8_t charBuffer[numChars];
static uint8_t charBuffer[480];

void osdPioWriteChar(uint8_t x, uint8_t y, uint8_t c);
void osdPioWrite(uint8_t x, uint8_t y, const char *text);

int64_t safe_zone_callback(alarm_id_t id, void * user_data)
{
    static int cc;
    UNUSED(id);
    UNUSED(user_data);
    in_safe_zone = false;
    szd = getCycleCounter();
    if (++cc == 99999991) {
        bprintf("\nsz %d %d %d %d  %d\n",sza,szb,szc,szd,sze);
    }
    return 0; // don't automatically reschedule
}

bool osdBuffer1Safe(void)
{
    // TODO *** add back in dma check
    return in_safe_zone;
//    return in_safe_zone && !dma_channel_is_busy(dma_chan_buf1_to_buf2) && !dma_channel_is_busy(dma_chan_zero_to_buf1);
//    return // !dma_channel_is_busy(dma_chan_buf1_to_buf2) &&
//        !dma_channel_is_busy(dma_chan_zero_to_buf1);
//    return  !dma_channel_is_busy(dma_chan_buf1_to_buf2);
}

void testUpdate(void);

void plot(int x, int y, int c)
{
    static int badcount = 10;
    // c =  0 -> transparent (no overlay)   W=any EN=0
    // c =  1 -> black                      W=0   EN=1
    // c =  2 -> white                      W=1   EN=1
    if (x<0 || y<0 || x>=fb_nx || y>=fb_ny) {
        if (badcount-- > 0) {
            bprintf("*** out of range plot %d, %d, %d",x,y,c);
        }
        return;
    }

    uint8_t * pbyte = osdBuffer1 + PICO_OSD_BUF_WIDTH * y;
    pbyte += (int)(x/4); // 4 pixels per byte
    if (pbyte<osdBuffer1 || pbyte>=osdBuffer1 + PICO_OSD_BUF_LENGTH) {
        bprintf("huh %p (%p) %d, %d, %d",pbyte,osdBuffer1, x,y,c);
    }
    static uint8_t masks[4] = {0b00000011, 0b00001100, 0b00110000, 0b11000000};
    static uint8_t  cols[4] = {0b00000000, 0b10101010, 0b11111111, 0b00000000};
    uint8_t mask = masks[x%4];
    uint8_t col = cols[c];
    *pbyte = ((*pbyte) &(~mask)) | (mask&col);
}

static void vsync_callback(void);

static void plotBorder(void)
{    
    for (int i=0; i<fb_nx; ++i) {
        if (i < fb_nx/6) {plot(i, i, 2); plot(i, fb_ny-i-1, 2);}
        if (i > 5*fb_nx/6) {plot(i, (fb_nx-i-1), 2); plot(i, fb_ny-(fb_nx-i-1)-1, 2);}
        plot(i,0,1); plot(i,fb_ny-1,1);
        plot(i,1,2); plot(i,fb_ny-2,2);
    }
    for (int i=0; i<fb_ny; ++i) {
        plot(0,i,1); plot(fb_nx-1,i,1);
        plot(1,i,2); plot(fb_nx-2,i,2);
    }
    for (int i=200; i<270; ++i) {
        plot(i,i,2);
        plot(i-1,i,1);
    }
}

void osd_test_init(void)
{
    safe_zone_period = 18000;
    safe_zone_period = 10000; // half of PAL 20000us, disallow TRANSFER (render to osdBuffer1) during final 10000 or so
//    safe_zone_period = 16723; // half of PAL 20000us, disallow TRANSFER (render to osdBuffer1) during final 10000 or so
//    safe_zone_period = 16000; // half of PAL 20000us, disallow TRANSFER (render to osdBuffer1) during final 10000 or so
//    safe_zone_period = 1234; // half of PAL 20000us, disallow TRANSFER (render to osdBuffer1) during final 10000 or so
//    safe_zone_period = 220100;
    in_safe_zone = true;

    bprintf("osd_test_init");
    bprintf("pbw %d, pbh %d, bpl %d", PICO_OSD_BUF_WIDTH, PICO_OSD_BUF_HEIGHT, PICO_OSD_BUF_LENGTH);
    bprintf("osdBuffer1: %p osdBuffer2: %p", osdBuffer1, osdBuffer2);
    bprintf("nx %d, ny %d", fb_nx, fb_ny);
    for (int i=0; i<PICO_OSD_BUF_LENGTH; ++i) {
//        int y = i / PICO_OSD_BUF_WIDTH;
//        int x = (i % PICO_OSD_BUF_WIDTH) * 4; // approx. pixels
//        int dd = (x-184)*(x-184)+(y-128)*(y-128);
//        monoBuffer[i] = dd < 15000 ? 0xff : 0;
//        osdBuffer1[i] = dd < 15000 ? (dd < 3720 ? 0b10101010 : 0xff) : 0;
//        osdBuffer1[i] = 0xff; // dd < 15000 ? (dd < 3720 ? 0b10101010 : 0xff) : 0;
        osdBuffer1[i] = 0;
//        (void)dd;
    }

    for (int i=0; i<numChars; ++i) {
        charBuffer[i] = 0;
    }
    
#if 0
    for (int i=0; i<fb_nx; ++i) {
        for (int j=0; j<fb_ny; ++j) {
            plot(i,j, ((j%100)<10 ? (j%2)+1 : 0)); // (int)((13*j + i/37))%4);
        }
    }

    
#endif
    
#if 1
    plotBorder();

#elif 1
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
    gpio_put(osd_w_gpio, false);
    gpio_put(osd_en_gpio, false);
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

//    int pioclock = (int)75e6; // TODO
//    int pioclock = (int)75e6 * 1.01; // TODO
    int pioclock = (int)75e6 * 1.01; // TODO acceptable "slack"? clock should be accurate to ~ 1.00003 ?
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
//    pio_sm_put(osdPio, osd_tx_sm, 255);
//    pio_sm_put(osdPio, osd_tx_sm, 287);
    pio_sm_put(osdPio, osd_tx_sm, PICO_OSD_BUF_HEIGHT - 1);
#endif
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_pull(false, false));
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_mov(pio_isr, pio_osr));

    pio_set_irq0_source_enabled(osdPio, pis_interrupt0, true); // enable state machine IRQ 0 => system irq PIO_thisone_IRQ_0
    irq_set_exclusive_handler(osdPioIrq, vsync_callback);
    irq_set_enabled(osdPioIrq, true);


    // TODO *** consistent dma_claim vs dmaAllocate in PICO, probably follow SPI example

    dma_chan_buf1_to_buf2 = dma_claim_unused_channel(false);
    if (-1 == dma_chan_buf1_to_buf2) {
        bprintf("**** failed to claim dma channel (buf1 to buf2) for osd pico");
        return;
    }

    dma_chan_zero_to_buf1 = dma_claim_unused_channel(false);
    if (-1 == dma_chan_zero_to_buf1) {
        bprintf("**** failed to claim dma channel (zero to buf1) for osd pico");
        dma_channel_unclaim(dma_chan_buf1_to_buf2);
        return;
    }

    dma_chan_buf2_to_fifo = dma_claim_unused_channel(false);
    if (-1 == dma_chan_buf2_to_fifo) {
        bprintf("**** failed to claim dma channel (buf2 to fifo) for osd pico");
        dma_channel_unclaim(dma_chan_zero_to_buf1);
        dma_channel_unclaim(dma_chan_buf1_to_buf2);
        return;
    }

    dma_channel_config c = dma_channel_get_default_config(dma_chan_buf2_to_fifo);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(osdPio, osd_tx_sm, true));

    dma_channel_configure(
        dma_chan_buf2_to_fifo,
        &c,
        &osdPio->txf[osd_tx_sm],  // Write address (fixed PIO TX FIFO)
        NULL,                     // Read address (reset each time)
        PICO_OSD_BUF_WORDS,       // Number of transfers
        false                     // Don't start immediately
    );

    c = dma_channel_get_default_config(dma_chan_zero_to_buf1);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_chain_to(&c, dma_chan_buf2_to_fifo); // DMA to PIO fifo starts immediately on completion of clearing buf1

    dma_channel_configure(
        dma_chan_zero_to_buf1,
        &c,
        NULL,                     // Write address (reset each time)
        &zero,                    // Read address (fixed)
        PICO_OSD_BUF_WORDS,       // Number of transfers
        false                     // Don't start immediately
    );

    c = dma_channel_get_default_config(dma_chan_buf1_to_buf2);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    channel_config_set_chain_to(&c, dma_chan_zero_to_buf1); // DMA to clear buf1 starts immediately on completion of buffer flip

    dma_channel_configure(
        dma_chan_buf1_to_buf2,
        &c,
        NULL,                     // Write address (reset each time)
        NULL,                     // Read address (reset each time)
        PICO_OSD_BUF_WORDS,       // Number of transfers
        false                     // Don't start immediately
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

volatile int ouccount;
volatile int oucunsafe;

//#define unsafetestloop
#ifdef unsafetestloop
volatile int oucunsafe2;
volatile int oucunsafe3;
#endif
//static volatile int vdelay;
static volatile int vsyncflag;

static void vsync_callback(void)
{
    vsyncflag = 1;
    // 50 per second (PAL)
    static int c=0;
    // Need to clear the IRQ flag state from the PIO.
    // This just writes a 1 to a register, doesn't mess with SM execution    
//    pio_interrupt_clear(osdPio, 0);

    // start new dma as soon as possible, or at any rate before doing significant update work

    // * stop any dma in progress
    // * flip buffer (or alternate buffers)
    // * reset the read address and transfer count on the channel
    // * start dma
    // * do any work to update the buffer
    // * clear pio tx fifo

//#define testdmaabort

    static int business;
    static int busybuf;

    sza=getCycleCounter();

    if (dma_channel_is_busy(dma_chan_buf1_to_buf2) || dma_channel_is_busy(dma_chan_zero_to_buf1)) {
        // Unexpected, PIO shouldn't get back to vsync IRQ unless dma buf2->fifo has started
        // unless sync signals are rather mixed up (detected vsync pulse but no hsync pulses on any line)
        busybuf++;
        dma_channel_abort(dma_chan_zero_to_buf1);
        dma_channel_abort(dma_chan_buf1_to_buf2);
    }

    // Ensure that DMA (buf2 to PIO FIFO) is not in progress, and that the PIO FIFO is empty.
    // (Can happen if some lines were skipped due to not detecting hsync pulse)
    if (dma_channel_is_busy(dma_chan_buf2_to_fifo)) {
        ++business;
        dma_channel_abort(dma_chan_buf2_to_fifo);
    }

    pio_sm_clear_fifos(osdPio, osd_tx_sm);

    // Reset the incrementing addresses
    dma_channel_set_read_addr(dma_chan_buf2_to_fifo, osdBuffer2, false);
    dma_channel_set_read_addr(dma_chan_buf1_to_buf2, osdBuffer1, false);
    dma_channel_set_write_addr(dma_chan_buf1_to_buf2, osdBuffer2, false);
    dma_channel_set_write_addr(dma_chan_zero_to_buf1, osdBuffer1, false);
    
    // Start DMA flipping osdBuffer1 to osdBuffer2
    // chains to DMA for zero->osdBuffer1 (clears screen buffer)
    // chains to DMA for buf2 -> screen

// testing dma speed
//    if (c==123) {
//         uint32_t dc1 = getCycleCounter();
//         dma_channel_start(dma_chan_buf1_to_buf2);
//         uint32_t dc2 = getCycleCounter();
//         while (dma_channel_is_busy(dma_chan_buf1_to_buf2)) ;
//         uint32_t dc3 = getCycleCounter();
//         bprintf("* dc3-dc1 %d dc3-dc2 %d buf1 %p werc %p buf2 %p", dc3-dc1, dc3-dc2, osdBuffer1, &werc[0], osdBuffer2);
//     }

    dma_channel_start(dma_chan_buf1_to_buf2);

    // probably best clear at end, just in case there are re-trigger issues if cleared earlier...
    pio_interrupt_clear(osdPio, 0);

    // static alarm_id_t add_alarm_in_us (uint64_t us, alarm_callback_t callback, void * user_data, bool fire_if_past)
    // typedef int64_t(* alarm_callback_t)(alarm_id_t id, void *user_data)
    szb = getCycleCounter();

    static alarm_id_t aid = -1 ;
    if (aid != -1) {
        cancel_alarm(aid);
    }
    aid = add_alarm_in_us(safe_zone_period, safe_zone_callback, 0, true);
    in_safe_zone = true;
    
    // the rest is just debug and testing.

    ++c;

#if 0
    static int32_t maxcc;
    static int32_t cca;
    static int ccc;
//    int nav = 250;
    int nav = 99999250;
    uint32_t m1 = getCycleCounter();
    // testUpdate();
    int32_t dd = getCycleCounter() - m1;
    if (dd>maxcc) maxcc = dd;
    cca += dd;
    if (++ccc == nav) {
        ccc=0;
        bprintf("(%d %d busy %d busybuf %d) ave us per update: %.1f, max %.1f",
                cca, maxcc, business, busybuf,
                ((double)cca)/nav/150, ((double)maxcc)/150);
        cca = 0;
        maxcc = 0;
    }
#endif
    
    static uint32_t vmax = 0;
    static uint32_t szo;
    uint32_t q;
    static uint32_t qtot;
    uint32_t szn = getCycleCounter();
    if (c>20) {
        q = szn-szo;
        vmax = q > vmax ? q : vmax;
        qtot += q;
    }
    szo = szn;
    
    if (c % 250 == 0) {
        bprintf("%d vsync_callback busy %d %d",c, business, busybuf);
        bprintf("max time between callbacks: %d, last: %d, ave: %.1f",vmax/150, q/150, (double)(((float)qtot)/c/150));
        vmax = 0;
#ifdef unsafetestloop
        bprintf("ouccount %d of which unsafe %d %d %d (%.3f %.3f %.3f of 20000)", ouccount,
                oucunsafe, oucunsafe2, oucunsafe3,
                (double)(((float)oucunsafe)*20000.0f/ouccount),
                (double)(((float)oucunsafe2)*20000.0f/ouccount),
                (double)(((float)oucunsafe3)*20000.0f/ouccount)
               );
#else
        bprintf("ouccount %d of which unsafe %d ~ %d of 20000 ~ %.3f cf %d (%d)", ouccount, oucunsafe, (int)((float)oucunsafe * 20000.0f / (float)ouccount), ((double)oucunsafe)/ouccount, 20000 - safe_zone_period, (int)((float)oucunsafe * 20000.0f / (float)ouccount) - (20000 - safe_zone_period));
#endif
    }

    szc=getCycleCounter();
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
#ifdef PICO_TRACE
    uint8_t *buffer = (uint8_t *)rt->user_data;
    bprintf("buffer = %p, osdBuffer1 = %p", buffer, osdBuffer1);
#else
    UNUSED(rt);
#endif
        
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

#ifdef oldtests
static void disable(void)
{
    pio_sm_set_enabled(osdPio, osd_tx_sm, false);
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_set(pio_pins, 0));
    gpio_init(osd_w_gpio);
    gpio_init(osd_en_gpio);
//    gpio_set_pulls(osd_w_gpio, true, false);
}
#endif

void osd_test(void)
{
    osd_test_init();

    (void)rtdata;
/*
//     int32_t delay_ms = 20;
//    int32_t delay_ms = 1520;
    int32_t delay_ms = 51520;
    
  bprintf("adding timer");
    if (!add_repeating_timer_ms(delay_ms, timer_callback, &osdBuffer1[0], &rtdata)) {
        bprintf("*** failed to add timer ***");
    }
*/
    
#if 1
    bprintf("OSD PIO Enable");
    enable();
#else
    // debug PIO, histogram etc.
    
    int disp = 0;
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
                    uint32_t w = *(uint32_t*)(&osdBuffer1[4*i + jj]);
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
        (void)disp;
        return;
#elif 1
        (void)hist;
        (void)pca;

        if (disp == 0) {
            // moving blocks
            for (int i=0; i<15; ++i) {
                delay(250);
                for (int x=0; x<368; ++x) {
                    int xx = (3*(x+i))>>7;
                    for (int y=0; y<256; ++y) {
                        int yy = (3*y)>>7;
                        plot(x,y,((int)(xx+yy+i))%4);
                    }
                }
            }
        } else if (disp == 1) {
            for (int i=0; i<760; ++i) {
                for (int x=0; x<368; ++x) {
                    int rx = 32 + ((int)((i*3)/5)) % 304;
                    for (int y=0; y<256; ++y) {
                        int ry = 32 + ((int)(((i+123)*5)/7)) % 192;
                        int d = (x-rx)*(x-rx) + (y-ry)*(y-ry);
                        plot(x,y, d<1000 ? d<780 ? d<300 ? 0 : 2 : 1 : 0);
                    }
                }
            }
        }

        disp = (disp + 1)%2;
        
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
        delay(1997);
        pc = pio_sm_get_pc(osdPio, osd_tx_sm); bprintf("D pc = %d less offset = %d", pc, pc - osd_tx_offset);

        bprintf("      DISABLE");
        disable();
        pc = pio_sm_get_pc(osdPio, osd_tx_sm); bprintf("E pc = %d less offset = %d", pc, pc - osd_tx_offset);
    }
#endif
}


#ifdef TEST_PIO_OSD
void testOSDtaskOffPidLoop(void)
{
#if 1
    return;
#else
    testUpdate();
#endif
}

#endif

// osd_elements artificalhorizon attitude.values.*
// also see sensors/gyro/gyro.ADCf, but note gyro ~ rad/sec, accel ~ rad/sec^2
// flight/imu.c -> "euler angles" (sic) (pitch, roll, yaw)
// ./telemetry/crsf.c:    sbufWriteU16BigEndian(dst, decidegrees2Radians10000(attitude.values.roll));

void osdUpdateCallback(uint32_t t_us)
{
#if 1
    ouccount++;
    if (!osdBuffer1Safe()) {
        oucunsafe++;
        testUpdate();
    } else {
        testUpdate();//        oucunsafe++;
    }
    UNUSED(t_us);
    
#elif 1
    UNUSED(t_us);
    ouccount++;
    oucunsafe += !osdBuffer1Safe();
    testUpdate(); // delayMicroseconds(755);
#elif defined unsafetestloop
    UNUSED(t_us);
    while (true) {
        ouccount++;
        oucunsafe  += dma_channel_is_busy(dma_chan_buf1_to_buf2);
        oucunsafe2 += dma_channel_is_busy(dma_chan_zero_to_buf1);
        oucunsafe3 += dma_channel_is_busy(dma_chan_buf2_to_fifo);
//        zero++;
    }
#else
    static char oucbuf[30];
    ouccount++;
    sze = getCycleCounter();
//    if (vsyncflag) {
//        vsyncflag=0;
//        delayMicroseconds(vdelay);
//        vdelay = 18000 + (vdelay+1) % 2000;
//    }
//    osdPrintFloat(oucbuf, 0x64, ((float)t_us)/10000, "", 3, false, 0x6c);
//    osdPioWrite(2,0,"fish"); UNUSED(oucbuf); UNUSED(t_us);
//    osdPioWrite(2,0,oucbuf);
    if (osdBuffer1Safe()) {
        testUpdate();
    } else {
        oucunsafe++;
    }
    //  osdPioWrite(2,0,"fish");
    UNUSED(oucbuf); UNUSED(t_us);
#endif
}

void testUpdate(void)
{
//#define clearscreen
//#define testcard
#define textpaint
//#define blockpaint
#define ahpaint
//#define testsprintf
    // none:       0.0
    // blockpaint 39.4
    // ahpaint   216.7
    // clearscreen (bzero) 1104 (loop bytes) 1104 (loop words) 1104 (memset) 1104
    // clearscreen (__aeabi_memset) 99.6
    // testcard 3302.2 [includes memset clear]
    // textpaint blank: 25.7 ~4lines: 160
    static int parity;
    parity = 1-parity;

#ifdef testsprintf
    static char tsbuf[32];
#if 0
    int snprintf(char *str, size_t size, const char *format, ...);
    int sprintf(char *str, const char *format, ...);
//    for (int i=0; i<100; ++i) {
    for (float i=0.123f; i<100.0f; ++i) {
//        snprintf(tsbuf, 30, "look %d here %d so",i, i); // 622.5
//        sprintf(tsbuf, "look %d here %d so",i, i); // 621.8
        sprintf(tsbuf, "A%.1fB",(double)i); // ...
        sprintf(tsbuf, "A%.2fB",(double)i); // 929.6
    }
#else
//    for (int i=0; i<100; ++i) {
    for (float i=0.123f; i<100.0f; ++i) {
//        tfp_sprintf(tsbuf, "look %d here %d so",i, i); // 429.8
        int osdPrintFloat(char *buffer, char leadingSymbol, float value, char *formatString, unsigned decimalPlaces, bool round, char trailingSymbol);
        osdPrintFloat(tsbuf, 'A', i, "", 1, false, 'B'); // ...
        osdPrintFloat(tsbuf, 'A', i, "", 2, false, 'B'); // 1166.8
    }
#endif
    
#endif
#ifdef clearscreen
//    for (int i=0; i<PICO_OSD_BUF_LENGTH; ++i) {
//        osdBuffer1[i] = 0;
//    }
    {
        void *__aeabi_memset(void *s, size_t n, int c); // maybe , size_t n);    
//        __aeabi_memset(osdBuffer1, PICO_OSD_BUF_LENGTH, 0xff);
        __aeabi_memset(osdBuffer1, PICO_OSD_BUF_LENGTH, 0);

//        uint32_t *p = (uint32_t *)osdBuffer1;
//        uint32_t *p = osdBuffer1W;
//        for (int i=0; i<PICO_OSD_BUF_LENGTH/4; ++i) {
//            *p++ = 0;
//        }
//        memset(osdBuffer1, 0, PICO_OSD_BUF_LENGTH/4);
    }
//    bzero(osdBuffer1, PICO_OSD_BUF_LENGTH);
#endif
    
#ifdef testcard
//    memset(osdBuffer1, 0b10101010, PICO_OSD_BUF_LENGTH); // black background
#ifndef clearscreen
    memset(osdBuffer1, 0, PICO_OSD_BUF_LENGTH); // transparent background
#endif
    for (int i=0; i<fb_nx; ++i) {
        plot(i, 0, 2);
        plot(i, fb_ny/2-1, 2);
        plot(i, fb_ny-1, 2);
    }
    for (int i=0; i<fb_ny; ++i) {
        plot(0, i, 2);
        plot(fb_nx/2-1, i, 2);
        plot(fb_nx-1, i, 2);
    }
    int xx2 = fb_nx/2;
    int yy2 = fb_ny/2;
    for (int k=2; k<10; ++k) {
        int q = fb_nx/2/k;
        int r = fb_ny/2/k;
        for (int j=0; j<16; ++j) {
            plot(xx2-q,j,2);
            plot(xx2+q,j,2);
            plot(xx2-q,fb_ny-1-j,2);
            plot(xx2+q,fb_ny-1-j,2);
            plot(j,yy2-r,2);
            plot(j,yy2+r,2);
            plot(fb_nx-1-j,yy2-r,2);
            plot(fb_nx-1-j,yy2+r,2);
        }
        for (int i=xx2-q; i<xx2+q; ++i) {
            plot(i,k-1,2);
            plot(i,fb_ny - k,2);
        }
        for (int i=yy2-r; i<yy2+r; ++i) {
            plot(k-1,i,2);
            plot(fb_nx-k,i,2);
        }
    }
    // white diagonals to corners and centres of sides
    for (int i=0; i<64; ++i) {
        plot(i, i, 2);
        plot(i, fb_ny/2 - 1 -i, 2);
        plot(i, fb_ny/2 + i, 2);
        plot(i, fb_ny - 1 - i, 2);
        plot(fb_nx -1 -i, i, 2);
        plot(fb_nx -1 -i, fb_ny/2 - 1 -i, 2);
        plot(fb_nx -1 -i, fb_ny/2 + i, 2);
        plot(fb_nx -1 -i, fb_ny - 1 - i, 2);
        plot(fb_nx/2 -1 -i, i, 2);
        plot(fb_nx/2  +i, i, 2);
        plot(fb_nx/2 -1 -i, fb_ny -1 -i, 2);
        plot(fb_nx/2  +i, fb_ny -1 -i, 2);
        
    }

#elif 0
    if (0 == (millis() % 5000) ) { parity = 1 - parity; }

//    if (parity) {
    if (1) {
//    memset(osdBuffer1, 0b10101010, PICO_OSD_BUF_LENGTH/2);
//    memset(osdBuffer1 + PICO_OSD_BUF_LENGTH/2, 0xff, PICO_OSD_BUF_LENGTH/2);
    memset(osdBuffer1, 0xff, PICO_OSD_BUF_LENGTH/2);
    memset(osdBuffer1 + PICO_OSD_BUF_LENGTH/2, 0b10101010, PICO_OSD_BUF_LENGTH/2);

    // initially with set x,22 in pio
    // not with i=1... nor with j=1...
    // not with i<20,j<20 but with i<25,j<25
    // i<24, j<24 looks double bad, but maybe correct on one field, bad on the other (squares about 1/4 from the right)
    // from v offsets, I think the bad first square is late by 3/4 line
#if 1
//    int iii=25; int jjj = 25;
    int iii=45; int jjj = 45;
//    int iii=20; int jjj = 20;
//    int iii=5; int jjj = 5;
    for (int i=0; i<iii; ++i) {
        for (int j=0; j<jjj; ++j) {

            // how long is 24 pixels horizontally? we are running at 9 cpp so 24*9/150us = 1.44us
            // maybe monitor thinks it's front porch?

            // wait a minute, we're not allowed to write in first half of first line of even frame nor 2nd half of last line of odd frame
            // so let's enforce that
            // either with colour black (level black) or with not enable
            // try both
            
// *** 
// Individually, these two lines are fine, and we see black square in top left or top right
// but together, we get wobbly out of sync, black squares appear about 1/4 way across the row (and not stable)
            // [ was probably just total time... ]
            plot(i,j,1);
            plot(fb_nx-1-i,j,1);
// ***


//            plot(i,fb_ny-1,2);
//            plot(fb_nx-1-i,fb_ny-1,2);
        }
    }

    // enforce (overkill full lines, both fields)
#if 1
    for (int i=0; i<fb_nx; ++i) {
        for (int j=0; j<4; ++j) {
//        plot(i,0,0);
//        plot(i,1,0);
//        plot(i,2,0);
//        plot(i,fb_ny-1,0);
//            plot(i,j,(i/32)%3);
//            plot(i,fb_ny-1-j,(i/32)%3);
            plot(i,j,(i/32)%3);
//            plot(i,j+4,(i/32)%3);
            plot(i,fb_ny-1-j,0);
        }
    }
#endif    
#endif
    
    } else {
        memset(osdBuffer1, 0, PICO_OSD_BUF_LENGTH);
    }
#if 0
  int s = millis()/4234;
    for (int i=0; i<PICO_OSD_BUF_LENGTH; ++i) {
        osdBuffer1[i] = (0b1010101) * ((s >> 10)&3);
        if (i%577 == 234)
            s = s*13+29;
    }
    memset(osdBuffer1, 0b10101010 /*0xff*/, PICO_OSD_BUF_LENGTH/2);
    memset(osdBuffer1 + PICO_OSD_BUF_LENGTH/2, 0xff /* 0b10101010*/, PICO_OSD_BUF_LENGTH/2);
#endif


#endif // testcard
    
#ifdef textpaint
    extern const uint8_t fontData[18*3*256];

    const int hoffs = 0; //4; // 0..7
    const int pxpc = 12;
    const int bxpc = pxpc / 4; // 4 pixels per byte
    const int pypc = 18;
    const int bpc  = bxpc * pypc;
    const int fbbpl = fb_nx / 4; // bytes per line = pixels per line / pixels per byte3

    for (int i=0; i<numChars; ++i) {
        uint8_t c = charBuffer[i];
        if (!c) {
            continue;
        }

        // paint chars to buffer here
        // 1 char = 12 pixels = 3 bytes. 4 chars = 48 pixels = 12 bytes = 3 words
        int x = i % charsPerLine;
        int y = i / charsPerLine; // or loop x,y
        uint8_t * bufp = osdBuffer1 + hoffs + fbbpl * y * pypc + x * bxpc; // pointer to topleft of char dest on osdBuffer1
        // TODO bufp without multiply, loop x,y etc.

        const uint8_t * fontp = &fontData[c*bpc]; // 3 bytes per 12 pixel char line, 18 lines
        // rp2350 don't have to worry about cache, all 1-clock sram

//        bprintf("painting char '%c' (0x%02x) at %d, %d from %p (cf %p) to %p (cf %p)",
//                c, c, x, y, fontp, fontData, bufp, osdBuffer1);
        for (int j=0; j<18; ++j) {
            for (int b=0; b<3; ++b) {
                *bufp++ = *fontp++;
            }
            bufp += fbbpl - 3; // new line, back 3 bytes
        }
    }


#endif

    //bzero(osdBuffer1, PICO_OSD_BUF_LENGTH);
    //plotBorder();
#ifdef blockpaint
    {
        static const int nxx = fb_nx - 64;
        static const float xp = ((float)(nxx))/4000000;
        uint32_t ctime = micros();
        int x = 27 + ((int)(ctime*xp)) % nxx;
        int y = 32;
        for (int i=0; i<10; ++i) {
            for (int j=0; j<10; ++j) {
                plot(x+i, y+j, (j==0 || i==0) ? 1 : 2);
            }
        }
    }
#endif

#ifdef ahpaint
    //bzero(osdBuffer1, PICO_OSD_BUF_LENGTH);
    //plotBorder();

    /*
      osd_ah_max_pit = 20
osd_ah_max_rol = 40
osd_ah_invert = OFF
    */

    // cf. osd_elements.c osdElementArtificialHorizon
    // Get pitch and roll limits [and values] in tenths of degrees
    const int maxPitch = osdConfig()->ahMaxPitch * 10;
    const int maxRoll = osdConfig()->ahMaxRoll * 10;
    const int ahSign = osdConfig()->ahInvert ? -1 : 1;
    const int rollAngle = constrain(attitude.values.roll * ahSign, -maxRoll, maxRoll);
    int pitchAngle = constrain(attitude.values.pitch * ahSign, -maxPitch, maxPitch);

    static bool didPitchCalc;
    static float pitchMult;
    static float rollMult;
    static const float pitchMaxOffset = fb_ny*0.2f; // fb_ny*0.1f;
    static const float rollMaxOffset = fb_ny*0.2f;
    static const int hcx = fb_nx / 2;
    static const int hcy = fb_ny * 0.68f;
    static const int hhwid = 8 * (int)(fb_nx * 0.2f / 8);
    static const float oohhwid = 1.0f / hhwid;

    if (!didPitchCalc) {
        pitchMult = pitchMaxOffset / maxPitch;
        rollMult = rollMaxOffset / maxRoll;
    }

    int im = fb_ny*0.75f;
    int imm = im/10;
    int yy = fb_ny*0.9f;
    for (int i=0; i<im; ++i) {
        int dd = hhwid*1.15f;
        if (i==0 || i==im-1) {
            for (int j=-4; j<=4; ++j) {
                plot(hcx-dd+j, yy, 2);
                plot(hcx+dd+j, yy, 2);
            }
        } else if (i%imm == 0) {
            for (int j=-2; j<=2; ++j) {
                plot(hcx-dd+j, yy, 2);
                plot(hcx+dd+j, yy, 2);
            }
        }
        if (i%8 == 2) {
            plot(hcx-dd, yy, 2);
            plot(1 + hcx-dd, yy, 1);
            plot(hcx+dd, yy, 2);
            plot(1 + hcx+dd, yy, 1);
        }
        yy--;
    }
    
    int ypitchoffset = 0;
    int yrollmax;; // ypitchoffset -+ yrollmax across hwid

    // Convert pitchAngle to y compensation value
    // (maxPitch / 25) divisor matches previous settings of fixed divisor of 8 and fixed max AHI pitch angle of 20.0 degrees

    if (maxPitch > 0) { // <-- where did that come from?
        ypitchoffset = pitchAngle * pitchMult; // small angles, pitchAngle roughly proportional to pitch delta (in pixels)
    }

    yrollmax = rollAngle * rollMult;

    int xi = hcx - hhwid;
    float yf = hcy + ypitchoffset - yrollmax;
    float yDelta = yrollmax * oohhwid;
    // int hmod = hhwid/4;
    
    for (int i=0; i< 2*hhwid+1; ++i) {
        plot(xi, yf, 2);
        plot(xi, yf+1, 1);
        if (i == 0 || i == 2*hhwid) { //  || (i%hmod == 0)) {
            plot(xi, yf-1, 2);
            plot(xi, yf+1, 2);
            plot(xi, yf-2, 2);
            plot(xi, yf+2, 2);
        }
        yf += yDelta;
        xi++;
    }
   

#endif // ahpaint

#if 0
    static const int usPerRun = 50000;
    static const int nxx = fb_nx - 64;
    static const float xp = ((float)(nxx))/4000000;
    static uint32_t ttime;
    if (!ttime) {
        ttime = micros();
    }
    uint32_t ctime = micros();
    int32_t dtime = (int32_t)(ctime - ttime);
    if (dtime > usPerRun) {
        bzero(osdBuffer1, PICO_OSD_BUF_LENGTH);
        plotBorder();
        int x = 27 + ((int)(ctime*xp)) % nxx;
        (void)xp;
        int y = 32;
        for (int i=0; i<10; ++i) {
            for (int j=0; j<10; ++j) {
                plot(x+i, y+j, (j==0 || i==0) ? 1 : 2);
            }
        }
        ttime = ctime;            
    }
#endif
}

void osdPioWriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    if (x < charsPerLine && y < charLines) {
        charBuffer[y*charsPerLine + x] = c;
    }
}

void osdPioWrite(uint8_t x, uint8_t y, const char *text)
{
    if (y < charLines) {
        uint8_t *p = charBuffer + y * charsPerLine;
        int i=0;
        while (text[i] && x < charsPerLine) {
            p[x++] = text[i++];
        }
    }
}

#else // USE_OSD_SD

// no OSD SD

// if required
void osdPioWriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    UNUSED(x);
    UNUSED(y);
    UNUSED(c);
}

#endif // USE_OSD_SD
