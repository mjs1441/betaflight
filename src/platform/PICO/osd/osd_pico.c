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
//#ifdef TEST_PIO_OSD
#ifdef USE_FB_OSD

#if !(defined OSD_W_PIN && defined OSD_EN_PIN && defined OSD_SYNC_PIN)
#error This PICO OSD requires OSD_W_PIN, OSD_EN_PIN and OSD_SYNC_PIN to be defined
#endif

#include <string.h>
#include <stdlib.h>

#include "common/printf.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/osd.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "fc/rc_controls.h"
#include "flight/imu.h"
#include "osd/osd.h"
#include "pg/vcd.h"
#include "rx/rx.h"

// pico sdk
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

// local
#include "osd_pico.h"
#include "osd_tx.pio.h"
#include "font_betaflight.h"

// each char 12 x 18 pixels
#define PICO_OSD_CHAR_WIDTH  12
#define PICO_OSD_CHAR_HEIGHT 18

// chars OSD_SD_ROWS x OSD_SD_COLS (30 x 16)
// 360 / 8 = 45 x 288
// 2 bits per pixel

// 23 -> 23*4*4 = 368 pixels -> 30.67 chars
// 288 for PAL field
// PIO hard coded to 23 words of pixel data per line (=> 368 pixels)
#define PICO_OSD_LINE_WORDS 23
#define PICO_OSD_BUF_WIDTH (PICO_OSD_LINE_WORDS*4)

#define PICO_OSD_BUF_HEIGHT_NTSC (PICO_OSD_CHAR_HEIGHT * VIDEO_LINES_NTSC)
#define PICO_OSD_BUF_HEIGHT_PAL (PICO_OSD_CHAR_HEIGHT * VIDEO_LINES_PAL)
#define PICO_OSD_BUF_HEIGHT_MAX PICO_OSD_BUF_HEIGHT_PAL
#define PICO_OSD_BUF_LENGTH (PICO_OSD_BUF_WIDTH * PICO_OSD_BUF_HEIGHT_MAX)

// 18*13 = 234, 18*16 = 288
STATIC_ASSERT(PICO_OSD_BUF_HEIGHT_NTSC == 234, pico_ntsc_lines_failed);
STATIC_ASSERT(PICO_OSD_BUF_HEIGHT_PAL == 288, pico_pal_lines_failed);

#define PICO_OSD_DISPLAY_WORDS_NTSC (PICO_OSD_LINE_WORDS * PICO_OSD_BUF_HEIGHT_NTSC)
#define PICO_OSD_DISPLAY_WORDS_PAL  (PICO_OSD_LINE_WORDS * PICO_OSD_BUF_HEIGHT_PAL)

static const PIO osdPio = PIO_INSTANCE(PIO_OSD_INDEX);
static const uint osdPioIrq = PIO_IRQ_NUM(osdPio, 0);

static const int fb_nx = PICO_OSD_BUF_WIDTH * 4;
static const int charsPerLine = 30;

static const int charWidth = PICO_OSD_CHAR_WIDTH;
static const int charHeight = PICO_OSD_CHAR_HEIGHT;
static const int charHalfWidth = charWidth / 2;
static const int charHalfHeight = charHeight / 2;

// PAL / NTSC, require initialisation.
static int fb_ny;
static int charLines = VIDEO_LINES_PAL; // Variable, default to 16 (PAL)
static int numChars;

// PIO program offset and state machine.
static int osd_tx_offset;
static int osd_tx_sm;

// GPIOs for white/black, write enable, sync-detect.
static int osd_w_gpio;
static int osd_en_gpio;
static int osd_sync_gpio;
static int osdPioBase;

__attribute__((aligned(4))) static uint32_t osdBuffer1W[PICO_OSD_BUF_LENGTH/4];
__attribute__((aligned(4))) static uint32_t osdBuffer2W[PICO_OSD_BUF_LENGTH/4];
static uint8_t* osdBufferA = (uint8_t *)osdBuffer1W;
static uint8_t* osdBufferB = (uint8_t *)osdBuffer2W;

static const uint32_t zero;
//static const uint32_t zero = 0xaaaaaaaa;
//static const uint32_t zero = 0x22222222;
//static const uint32_t zero = 0x88888888;
//static const uint32_t zero = 0xf2f2f2f2;

static int dma_chan_zero_to_bufA;
static int dma_chan_bufB_to_fifo;

// buffer update control (avoid tearing etc.)
static volatile bool in_safe_zone;
static volatile uint32_t safe_zone_period;
static volatile bool transferredSinceVsync;

// trace / debugging
static volatile uint32_t sza;
static volatile uint32_t szb;
static volatile uint32_t szc;
static volatile uint32_t szd;
static volatile uint32_t sze;
static volatile int tus;
static volatile int tusr;
static volatile uint32_t maxcycles;
static volatile int nisz;
static volatile int dmb;
static volatile uint32_t maxAHI;


// 30 * 16 = 480
uint8_t osdCharBuffer[OSD_SD_COLS * OSD_SD_ROWS];

void osdPioWriteChar(uint8_t x, uint8_t y, uint8_t c);
void osdPioWrite(uint8_t x, uint8_t y, const char *text);

static void init_gpios(void)
{
    static bool did;
    if (!did) {
        // Insist on osd_w_gpio -> osd_en_gpio -> osd_sync_gpio being consecutive.
        osd_w_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_W_PIN));
        osd_en_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_EN_PIN));
        if (osd_en_gpio != osd_w_gpio + 1) {
            bprintf("*** OSD_EN_GPIO must be next pin up from OSD_W_GPIO (%d vs %d)", osd_en_gpio, osd_w_gpio);
        }
        
        osd_sync_gpio = IO_GPIOPinIdxByTag(IO_TAG(OSD_SYNC_PIN));
        if (osd_sync_gpio != osd_en_gpio + 1) {
            // might relax this... wait GPIO vs wait PINS if single SM, or just separate SMs
            bprintf("*** OSD_SYNC_GPIO must be next pin up from OSD_EN_GPIO (%d vs %d)", osd_sync_gpio, osd_en_gpio);
        }

        osdPioBase = osd_sync_gpio < 32 ? 0 : 16; // Need the higher range if the highest gpio is not in the low range 0..31.
        bprintf("osd_w gpio %d, osd_en gpio %d, osd_sync gpio %d osdPioBase %d", osd_w_gpio, osd_en_gpio, osd_sync_gpio, osdPioBase);
        did = true;
    }
}

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

bool osdPioBufferAvailable(void)
{
#if 1
    if (transferredSinceVsync) {
        return false;
    }

    if (!in_safe_zone) {
        nisz++;
        return false;
    }

    if (dma_channel_is_busy(dma_chan_zero_to_bufA)) {
        dmb++;
        return false;
    }

    return true;
#else
    return !transferredSinceVsync && in_safe_zone && !dma_channel_is_busy(dma_chan_zero_to_bufA);
#endif
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

    uint8_t * pByte = osdBufferA + PICO_OSD_BUF_WIDTH * y;
    pByte += (int)(x/4); // 4 pixels per byte
#if 0
    if (pByte<osdBufferA || pByte>=osdBufferA + PICO_OSD_BUF_LENGTH) {
        bprintf("huh %p (%p) %d, %d, %d",pByte,osdBufferA, x,y,c);
    }
#endif
    static uint8_t masks[4] = {0b00000011, 0b00001100, 0b00110000, 0b11000000};
    static uint8_t  cols[4] = {0b00000000, 0b10101010, 0b11111111, 0b00000000};
    uint8_t mask = masks[x%4];
    uint8_t col = cols[c];
    *pByte = ((*pByte) &(~mask)) | (mask&col);
}

void hLine(int x, int y, int count, int col)
{
    for (int i=0; i<count; ++i) {
        plot(x++, y, col);
    }
}

void dhLine(int x, int y, int count)
{
    for (int i=x; i < x + count; i++) {
        plot(i, y, 2);
        plot(i, y+1, 1);
    }
}

void dvLine(int x, int y, int count)
{
    for (int i=y; i < y + count; i++) {
        plot(x, i, 2);
        plot(x+1, i, 1);
    }
}

void plotBlob(int x, int y)
{
    plot(x-2, y-2, 2);
    plot(x-1, y-2, 2);
    plot(x, y-2, 2);
    plot(x+1, y-2, 2);
    plot(x+2, y-2, 2);
    plot(x-2, y-1, 2);
    plot(x-2, y, 2);
    plot(x-2, y+1, 2);
    plot(x-2, y+2, 2);
    plot(x-1, y+2, 2);
    plot(x, y+2, 2);
    plot(x+1, y+2, 2);
    plot(x+2, y+2, 2);
    plot(x+2, y+1, 2);
    plot(x+2, y, 2);
    plot(x+2, y-1, 2);

    plot(x-1, y-1, 1);
    plot(x-1, y, 1);
    plot(x-1, y+1, 1);
    plot(x, y+1, 1);
    plot(x+1, y+1, 1);
    plot(x+1, y, 1);
    plot(x+1, y-1, 1);
    plot(x, y-1, 1);
}
    
// WARNING iter line functions are designed to be called iteratively, but only from one source at a time.

typedef struct {
    int count;
    int maxCount;
    float delta;
    bool shallow;
    int ic;
    float fc;
} iterLineData_t;

static void iterLineDataInit(iterLineData_t *data, int x1, int y1, int x2, int y2)
{
    data->count = 0;
    int dx = x2 - x1;
    int dy = y2 - y1;
    bool shallow = ABS(dx) > ABS(dy);
    data->shallow = shallow;
    if (shallow) {
        data->delta = (float)dy / dx;
        if (x1 < x2) {
            data->ic = x1;
            data->fc = (float)y1;
            data->maxCount = x2 - x1 + 1;
        } else {
            data->ic = x2;
            data->fc = (float)y2;
            data->maxCount = x1 - x2 + 1;
        }
    } else {
        data->delta = dy == 0 ? 0.0f : (float)dx / dy; // cope with case of a single point.
        if (y1 < y2) {
            data->fc = (float)x1;
            data->ic = y1;
            data->maxCount = y2 - y1 + 1;
        } else {
            data->fc = (float)x2;
            data->ic = y2;
            data->maxCount = y1 - y2 + 1;
        }
    }
}

// iterLineData shared amongst all of the iter...Line functions.
static iterLineData_t iterLineData;

static void iterLineInit(int x1, int y1, int x2, int y2)
{
    iterLineDataInit(&iterLineData, x1, y1, x2, y2);
}

static bool iterDLineNext(void)
{
    if (iterLineData.count >= iterLineData.maxCount) {
        return true; // all done.
    }

    if (iterLineData.shallow) {
        plot(iterLineData.ic, iterLineData.fc, 2);
        plot(iterLineData.ic, iterLineData.fc + 1, 1);
        iterLineData.ic++;
        iterLineData.fc += iterLineData.delta;
    } else {
        plot(iterLineData.fc, iterLineData.ic, 2);
        plot(iterLineData.fc + 1, iterLineData.ic, 1);
        iterLineData.ic++;
        iterLineData.fc += iterLineData.delta;
    }

    iterLineData.count++;
    return false;
}

static bool iterDashedDLineNext(void)
{
    if (iterLineData.count >= iterLineData.maxCount) {
        return true; // all done.
    }

    if ((iterLineData.count % 16) < 9) {
        if (iterLineData.shallow) {
            plot(iterLineData.ic, iterLineData.fc, 2);
            plot(iterLineData.ic, iterLineData.fc + 1, 1);
        }
        else {
            plot(iterLineData.fc, iterLineData.ic, 2);
            plot(iterLineData.fc + 1, iterLineData.ic, 1);
        }
    }

    iterLineData.ic++;
    iterLineData.fc += iterLineData.delta;
    iterLineData.count++;
    return false;
}


static void vsync_callback(void);

#if 0
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
}
#endif
    
#if 0
    for (int i=0; i<fb_nx; ++i) {
        for (int j=0; j<fb_ny; ++j) {
            plot(i,j, ((j%100)<10 ? (j%2)+1 : 0)); // (int)((13*j + i/37))%4);
        }
    }

    
#endif
    
#if 0
    // plotBorder();
    UNUSED(plotBorder);

#elif 0
    UNUSED(plotBorder);
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

static void osd_init_device(bool isPAL, int displayLines, int transferWords)
{
//    safe_zone_period = 18000;
    safe_zone_period = 16000;
//     safe_zone_period = 12000; // half of PAL 20000us, disallow TRANSFER (render to osdBufferA) during final 10000 or so
    in_safe_zone = true;

    bprintf("OSD osd_init_device lines %d words %d", displayLines, transferWords);
    bprintf("pbw %d, pbh %d, bpl %d", PICO_OSD_BUF_WIDTH, PICO_OSD_BUF_HEIGHT_MAX, PICO_OSD_BUF_LENGTH);
    bprintf("osdBuffer1: %p osdBuffer2: %p", osdBuffer1W, osdBuffer2W);
    bprintf("nx %d, ny %d", fb_nx, fb_ny);
    for (int i=0; i<PICO_OSD_BUF_LENGTH; ++i) {
//        int y = i / PICO_OSD_BUF_WIDTH;
//        int x = (i % PICO_OSD_BUF_WIDTH) * 4; // approx. pixels
//        int dd = (x-184)*(x-184)+(y-128)*(y-128);
//        monoBuffer[i] = dd < 15000 ? 0xff : 0;
//        osdBuffer1[i] = dd < 15000 ? (dd < 3720 ? 0b10101010 : 0xff) : 0;
//        osdBuffer1[i] = 0xff; // dd < 15000 ? (dd < 3720 ? 0b10101010 : 0xff) : 0;
        osdBufferA[i] = 0;
    }

    for (int i=0; i<numChars; ++i) {
        osdCharBuffer[i] = 0;
    }

    init_gpios();

    pio_set_gpio_base(osdPio, osdPioBase);
    osd_tx_offset = pio_add_program(osdPio, isPAL ? &osd_tx_pal_program : &osd_tx_ntsc_program);
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

    // default config with wrap set    
    pio_sm_config config = isPAL ? osd_tx_pal_program_get_default_config(osd_tx_offset)
        : osd_tx_ntsc_program_get_default_config(osd_tx_offset);

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

    int pioclock = (int)75e6;
    float div = (float)SystemCoreClock / pioclock;
    bprintf("OSD device clock div = %f", (double)div);
    sm_config_set_clkdiv(&config, div);
    pio_sm_init(osdPio, osd_tx_sm, osd_tx_offset, &config);

    // prepare value for vert pixel loop
    pio_sm_put(osdPio, osd_tx_sm, displayLines - 1);
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_pull(false, false));
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_mov(pio_isr, pio_osr));

    pio_set_irq0_source_enabled(osdPio, pis_interrupt0, true); // enable state machine IRQ 0 => system irq PIO_thisone_IRQ_0
    irq_set_exclusive_handler(osdPioIrq, vsync_callback);
    irq_set_enabled(osdPioIrq, true);

    // TODO *** consistent dma_claim vs dmaAllocate in PICO, probably follow SPI example

    dma_chan_zero_to_bufA = dma_claim_unused_channel(false);
    if (-1 == dma_chan_zero_to_bufA) {
        bprintf("**** failed to claim dma channel (zero to bufA) for osd pico");
        return;
    }

    dma_chan_bufB_to_fifo = dma_claim_unused_channel(false);
    if (-1 == dma_chan_bufB_to_fifo) {
        bprintf("**** failed to claim dma channel (buf2 to fifo) for osd pico");
        dma_channel_unclaim(dma_chan_zero_to_bufA);
        return;
    }

    dma_channel_config c = dma_channel_get_default_config(dma_chan_bufB_to_fifo);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(osdPio, osd_tx_sm, true));

    dma_channel_configure(
        dma_chan_bufB_to_fifo,
        &c,
        &osdPio->txf[osd_tx_sm],  // Write address (fixed PIO TX FIFO)
        NULL,                     // Read address (reset each time)
        transferWords,           // Number of transfers
        false                     // Don't start immediately
    );

    c = dma_channel_get_default_config(dma_chan_zero_to_bufA);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_chain_to(&c, dma_chan_bufB_to_fifo); // DMA to PIO fifo starts immediately on completion of clearing buf1

    dma_channel_configure(
        dma_chan_zero_to_bufA,
        &c,
        NULL,                     // Write address (reset each time)
        &zero,                    // Read address (fixed)
        transferWords,           // Number of transfers
        false                     // Don't start immediately
    );

    /*
      dma channel abort, workaround for erratum
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

int osdPioRowsCount(void)
{
    return charLines;
}

static void clearCountProgram(void)
{
    pio_sm_set_enabled(osdPio, osd_tx_sm, false);
    pio_remove_program_and_unclaim_sm(&osd_count_sync_program, osdPio, osd_tx_sm, osd_tx_offset);
}

void osdPioStartNTSC(void)
{
    fb_ny = PICO_OSD_BUF_HEIGHT_NTSC;
    charLines = VIDEO_LINES_NTSC;
    numChars = charsPerLine * charLines;
    bprintf("OSD set NTSC buf height %d char lines %d numChars %d", fb_ny, charLines, numChars);
    clearCountProgram();
    osd_init_device(false, PICO_OSD_BUF_HEIGHT_NTSC, PICO_OSD_DISPLAY_WORDS_NTSC);
    osdPioEnableDevice();
}

void osdPioStartPAL(void)
{
    fb_ny = PICO_OSD_BUF_HEIGHT_PAL;
    charLines = VIDEO_LINES_PAL;
    numChars = charsPerLine * charLines;
    bprintf("OSD set PAL buf height %d char lines %d numChars %d", fb_ny, charLines, numChars);
    clearCountProgram();
    osd_init_device(true, PICO_OSD_BUF_HEIGHT_PAL, PICO_OSD_DISPLAY_WORDS_PAL);
    osdPioEnableDevice();
}

static const int initLines = 1000;

void osdPioDetectStart(void)
{
    init_gpios();
    pio_set_gpio_base(osdPio, osdPioBase);
    osd_tx_offset = pio_add_program(osdPio, &osd_count_sync_program);
    osd_tx_sm = pio_claim_unused_sm(osdPio, false);
    pio_sm_config config = osd_count_sync_program_get_default_config(osd_tx_offset);

    pio_sm_set_consecutive_pindirs(osdPio, osd_tx_sm, osd_sync_gpio, 1, false /* input */);
    sm_config_set_in_pin_base(&config, osd_sync_gpio); // in PIN set SYNC (for WAIT)
    sm_config_set_in_pin_count(&config, 1);
    sm_config_set_jmp_pin(&config, osd_sync_gpio);     // jmp PIN is SYNC

    sm_config_set_out_shift(&config, true, false, 32); // no autopull
    sm_config_set_in_shift(&config, true, false, 32); // no autopush

    int pioclock = (int)75e6;
    float div = (float)SystemCoreClock / pioclock;
    bprintf("OSD Detect pio clock div = %f", (double)div);
    sm_config_set_clkdiv(&config, div);

    pio_sm_init(osdPio, osd_tx_sm, osd_tx_offset, &config);
    // Prepare OSR with the initial hsync count for the decrementing loop.
    pio_sm_put(osdPio, osd_tx_sm, initLines);
    // this now in PIO code: pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_pull(false, false));

    // Start counting...
    pio_sm_set_enabled(osdPio, osd_tx_sm, true);
}

int osdPioCountHSyncs(void)
{
    // Non-blocking, understand return of 0 as invalid / not ready.
    int hsyncs = 0;
    int pc = pio_sm_get_pc(osdPio, osd_tx_sm);
    if (pc - osd_tx_offset == osd_count_sync_offset_ready) {
        // Program has reached "pull block". Extract from ISR, then restart by sending to TX fifo.
        pio_sm_clear_fifos(osdPio, osd_tx_sm);
        pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_push(false, false));
        hsyncs = initLines - pio_sm_get(osdPio, osd_tx_sm);
        pio_sm_put(osdPio, osd_tx_sm, initLines);
    }

    return hsyncs;
}

volatile int ouccount;
volatile int oucunsafe;

//#define unsafetestloop
#ifdef unsafetestloop
volatile int oucunsafe2;
volatile int oucunsafe3;
#endif
//static volatile int vdelay;

static void vsync_callback(void)
{
    static int fieldOddEven;
    fieldOddEven = fieldOddEven ^ 0x1;  // odd or even field (we can't tell which is which), alternate 0, 1
    uint8_t * tptr = osdBufferA;
    osdBufferA = osdBufferB;
    osdBufferB = tptr;

    transferredSinceVsync = 0;
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

    if (dma_channel_is_busy(dma_chan_zero_to_bufA)) {
        // Unexpected, PIO shouldn't get back to vsync IRQ unless dma buf2->fifo has started
        // unless sync signals are rather mixed up (detected vsync pulse but no hsync pulses on any line)
        busybuf++;
        dma_channel_abort(dma_chan_zero_to_bufA);
    }

    // Ensure that DMA (buf2 to PIO FIFO) is not in progress, and that the PIO FIFO is empty.
    // (Can happen if some lines were skipped due to not detecting hsync pulse)
    if (dma_channel_is_busy(dma_chan_bufB_to_fifo)) {
        ++business;
        dma_channel_abort(dma_chan_bufB_to_fifo);
    }

    pio_sm_clear_fifos(osdPio, osd_tx_sm);

    // Reset the incrementing addresses
    dma_channel_set_read_addr(dma_chan_bufB_to_fifo, osdBufferB, false);
    dma_channel_set_write_addr(dma_chan_zero_to_bufA, osdBufferA, false);
    
    // Start DMA for zero->osdBufferA (clears screen buffer)
    // chains to DMA for osdBufferB -> screen

// testing dma speed
//    if (c==123) {
//         uint32_t dc1 = getCycleCounter();
//         dma_channel_start(dma_chan_buf1_to_buf2);
//         uint32_t dc2 = getCycleCounter();
//         while (dma_channel_is_busy(dma_chan_buf1_to_buf2)) ;
//         uint32_t dc3 = getCycleCounter();
//         bprintf("* dc3-dc1 %d dc3-dc2 %d buf1 %p werc %p buf2 %p", dc3-dc1, dc3-dc2, osdBuffer1, &werc[0], osdBuffer2);
//     }

    dma_channel_start(dma_chan_zero_to_bufA);

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
    int cm = c % 250;
    if (c>20) {
        q = szn-szo;
        if (cm != 1) {
            vmax = q > vmax ? q : vmax;
        }
        qtot += q;
    }

    static uint32_t n_to_c;
    if (c % 250 == 0) {
//        bprintf("%d vsync_callback busy %d %d (previous tainted n to c %d)",c, business, busybuf, n_to_c);
        bprintf("%d vsync_callback busy %d %d nisz %d dmb %d (previous tainted n to c %d)",
                c, business, busybuf, nisz, dmb, n_to_c);
        nisz = 0; dmb = 0;
        // NB ave wraps quickly (~1000 vsyncs)
//        bprintf("max time between callbacks: %d, last: %d, ave: %.1f",vmax/150, q/150, (double)(((float)qtot)/c/150));
//        bprintf("tus %d, tusr %d, ave %.1f calls per VS, %.1f rds per VS, %.1f calls/rd",
//                tus, tusr,
//                (double)tus/250, (double)tusr/250, (double)tus/tusr);
//        bprintf("max (per rd) us per call (ave over rds) %.1f, for which painted (ave over rds) %.1f",
//                (double)maxcycles/150.0/tusr, (double)paintedmaxcycles/tusr);
        bprintf("max (per rd) us per call (max over %d rds) %d", tusr, maxcycles/150);
//        bprintf("max ah cache cycles %d", maxAHI);
        maxcycles = 0;
        tus = 0; tusr = 0;
        vmax = 0;
#if 0
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
#endif
        n_to_c = getCycleCounter() - szn;
        UNUSED(n_to_c);
    }

    szo = szn;
    
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
    bprintf("buffer = %p, osdBufferA = %p", buffer, osdBufferA);
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

// maybe don't need...
static void disable(void)
{
    pio_sm_set_enabled(osdPio, osd_tx_sm, false);
    pio_sm_exec_wait_blocking(osdPio, osd_tx_sm, pio_encode_set(pio_pins, 0));
    gpio_init(osd_w_gpio);
    gpio_init(osd_en_gpio);
//    gpio_set_pulls(osd_w_gpio, true, false);
}

void osdPioEnableDevice(void) {
    enable();
}

void osdPioDisableDevice(void) {
    disable();
}

void osd_test(void)
{
    osd_init_device(true, PICO_OSD_BUF_HEIGHT_PAL, PICO_OSD_DISPLAY_WORDS_PAL);

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

void plotTestCard(void)
{
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
}

// osd_elements artificalhorizon attitude.values.*
// also see sensors/gyro/gyro.ADCf, but note gyro ~ rad/sec, accel ~ rad/sec^2
// flight/imu.c -> "euler angles" (sic) (pitch, roll, yaw)
// ./telemetry/crsf.c:    sbufWriteU16BigEndian(dst, decidegrees2Radians10000(attitude.values.roll));

void osdUpdateCallback(uint32_t t_us)
{
    bprintf("\n*** not in use *** \n");
#if 0
    ouccount++;
    if (!osdPioBufferAvailable()) {
        oucunsafe++;
        testUpdate();
    } else {
        // put equivalent delay here if you want to measure proportions from counters
    }
    UNUSED(t_us);
#elif defined unsafetestloop
    UNUSED(t_us);
    while (true) {
        ouccount++;
        oucunsafe2 += dma_channel_is_busy(dma_chan_zero_to_bufA);
        oucunsafe3 += dma_channel_is_busy(dma_chan_bufB_to_fifo);
//        zero++;
    }
#else
    static char oucbuf[30];
    ouccount++;
    sze = getCycleCounter();
    osdPrintFloat(oucbuf, 0x64, ((float)t_us)/10000, "", 3, false, 0x6c);
    osdPioWrite(2,0,oucbuf);
    if (osdPioBufferAvailable()) {
        testUpdate();
    } else {
        oucunsafe++;
    }
#endif
}

typedef struct {
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t yMid;
} info_sidebars_t;

static info_sidebars_t infoSidebars;
static bool calculatedSidebars;
static bool cachedSidebars;

// cf. osd_element.c implementation osdBackgroundHorizonSidebars
#define AH_SIDEBAR_WIDTH_POS 7
#define AH_SIDEBAR_HEIGHT_POS 3
static void cacheSidebarsInfo(uint8_t x, uint8_t y)
{
    // Cache the top left cornder and right edge in buffer coords
    // given the centre in char coords.
    // Sidebars are static (background), unchanging until reboot,
    // so only calculate once.

    if (!calculatedSidebars) {
        infoSidebars.x1 = (x - AH_SIDEBAR_WIDTH_POS) * charWidth + charHalfWidth;
        infoSidebars.y1 = (y - AH_SIDEBAR_HEIGHT_POS) * charHeight; // not  + charHalfHeight because sub 0.5char*charHeight
        infoSidebars.yMid = y * charHeight + charHalfHeight;
        infoSidebars.x2 = (x + AH_SIDEBAR_WIDTH_POS) * charWidth + charHalfWidth;;
//        infoSidebars.y2 = (y + AH_SIDEBAR_HEIGHT_POS) * charHeight;
        calculatedSidebars = true;
    }

    cachedSidebars = true;
}

typedef struct {
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
    bool outOfRange;
} info_ah_t;

static info_ah_t infoArtificialHorizon;
static bool cachedAH;
#include <math.h>

// cf. osd_element.c implementation osdElementArtificialHorizon
#define AH_SYMBOL_COUNT 9
static void cacheArtificialHorizonInfo(uint8_t x, uint8_t y)
{
    // takes about 5us (every 20ms)
//    uint32_t c1 = getCycleCounter();
    // Adjust to central y value of character-based AH element.
    y += (AH_SYMBOL_COUNT - 1) / 2;

    // Get pitch and roll limits in tenths of degrees
    const int ahSign = osdConfig()->ahInvert ? -1 : 1;
    const int maxPitch = osdConfig()->ahMaxPitch * 10;
    // roll is uncontrained now. // const int maxRoll = osdConfig()->ahMaxRoll * 10;
    // const int rollAngle = constrain(attitude.values.roll * ahSign, -maxRoll, maxRoll);
    const int rollAngle = attitude.values.roll * ahSign;
    int pitchAngleUnconstrained = attitude.values.pitch * ahSign;
    int pitchAngle = constrain(pitchAngleUnconstrained, -maxPitch, maxPitch);

    infoArtificialHorizon.outOfRange = pitchAngle != pitchAngleUnconstrained;

    static const int barScale = (AH_SIDEBAR_WIDTH_POS - 2) * charWidth; // The AH bar should fit nicely between the Sidebars.
    const int displacementScale = (fb_ny - 32) / 2; // going to fit maxPitch to screen (vertically).
    const float d2r = 3.14159265f * 2 / 360 / 10; // Extra scale factor of 10 for 10th of degree -> radian.
    float tp = tanf(pitchAngle * d2r);
    float cr = cosf(rollAngle * d2r);
    float sr = sinf(rollAngle * d2r);
    float tscale = tp * displacementScale / tanf(maxPitch * d2r);
    int xc = x * charWidth + charHalfWidth - tscale * sr;
    int yc = y * charHeight + charHalfHeight + tscale * cr;
    infoArtificialHorizon.x1 = xc + barScale * cr;
    infoArtificialHorizon.y1 = yc + barScale * sr;
    infoArtificialHorizon.x2 = xc - barScale * cr;
    infoArtificialHorizon.y2 = yc - barScale * sr;

    if (tusr == -234) {
        bprintf("OSD ah pitch %d roll %d tp %f cr %f sr %f tscale %f xc %d yc %d x1y1 %d %d x2y2 %d %d",
                pitchAngle, rollAngle, (double)tp, (double)cr, (double)sr, (double)tscale, xc, yc,
                infoArtificialHorizon.x1, infoArtificialHorizon.y1, infoArtificialHorizon.x2, infoArtificialHorizon.y2);
    }
    cachedAH = true;
//    uint32_t cd = getCycleCounter() - c1;
//    maxAHI = cd > maxAHI ? cd : maxAHI;
}

typedef struct {
    uint16_t xLeft;
    uint16_t yTop;
    uint16_t xStick;
    uint16_t yStick;
} info_stick_t;

static info_stick_t infoStickLeft;
static info_stick_t infoStickRight;
static uint8_t cachedStickLeft;
static uint8_t cachedStickRight;


typedef struct radioControls_s {
    uint8_t left_vertical;
    uint8_t left_horizontal;
    uint8_t right_vertical;
    uint8_t right_horizontal;
} radioControls_t;

static const radioControls_t radioModes[4] = {
    { PITCH,    YAW,    THROTTLE,   ROLL }, // Mode 1
    { THROTTLE, YAW,    PITCH,      ROLL }, // Mode 2
    { PITCH,    ROLL,   THROTTLE,   YAW  }, // Mode 3
    { THROTTLE, ROLL,   PITCH,      YAW  }, // Mode 4
};

// Stick overlay size
#define OSD_STICK_OVERLAY_WIDTH 7
#define OSD_STICK_OVERLAY_HEIGHT 5

static const int stickWidth = charWidth * OSD_STICK_OVERLAY_WIDTH;
static const int stickHeight = charHeight * OSD_STICK_OVERLAY_HEIGHT;

void cacheStickInfo(info_stick_t *infoPtr, uint8_t x, uint8_t y, rc_alias_e vert, rc_alias_e horiz)
{
    infoPtr->xLeft = charWidth * x;
    infoPtr->yTop = charHeight * y;
#if 1
    UNUSED(vert);
    UNUSED(horiz);
    float tr = micros()*(6.283f/1000000.0f / 3);
    infoPtr->xStick = (uint16_t)(infoPtr->xLeft + stickWidth/2 * (1 + cosf(tr)));
    infoPtr->yStick = (uint16_t)(infoPtr->yTop + stickHeight/2 * (1 + sinf(tr)));
#else
    
    const float cursorX = constrainf(rcData[horiz], PWM_RANGE_MIN, PWM_RANGE_MAX);
    const float cursorY = constrainf(rcData[vert], PWM_RANGE_MIN, PWM_RANGE_MAX);


    infoPtr->xStick = (uint16_t)scaleRangef(cursorX, PWM_RANGE_MIN, PWM_RANGE_MAX, infoPtr->xLeft, infoPtr->xLeft + stickWidth);

    // note y inverted, cf. osd_elements.c
    infoPtr->yStick = (uint16_t)scaleRangef(cursorY, PWM_RANGE_MIN, PWM_RANGE_MAX, infoPtr->yTop + stickHeight, infoPtr->yTop);
#endif
}


void cacheStickLeftInfo(uint8_t x, uint8_t y)
{
    rc_alias_e vertical_channel = radioModes[osdConfig()->overlay_radio_mode-1].left_vertical;
    rc_alias_e horizontal_channel = radioModes[osdConfig()->overlay_radio_mode-1].left_horizontal;
    cacheStickInfo(&infoStickLeft, x, y, vertical_channel, horizontal_channel);
    cachedStickLeft = 1; // Ready to render background.
}

void cacheStickRightInfo(uint8_t x, uint8_t y)
{
    rc_alias_e vertical_channel = radioModes[osdConfig()->overlay_radio_mode-1].right_vertical;
    rc_alias_e horizontal_channel = radioModes[osdConfig()->overlay_radio_mode-1].right_horizontal;
    cacheStickInfo(&infoStickRight, x, y, vertical_channel, horizontal_channel);
    cachedStickRight = 1; // Ready to render background.
}

bool osdPioDrawItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY)
{
    // Cache information for rendering an osd item later on.
    switch (item) {
    case OSD_HORIZON_SIDEBARS:
        cacheSidebarsInfo(elemPosX, elemPosY);
//         cachedSidebars = false;return false;
        return true;
    case OSD_ARTIFICIAL_HORIZON:
//#define testnoahhere
#ifdef testnoahhere
        UNUSED(cacheArtificialHorizonInfo);
        return false;
#else
        cacheArtificialHorizonInfo(elemPosX, elemPosY);
        return true;
#endif

    case OSD_STICK_OVERLAY_LEFT:
        cacheStickLeftInfo(elemPosX, elemPosY);
        return true;

    case OSD_STICK_OVERLAY_RIGHT:
        cacheStickRightInfo(elemPosX, elemPosY);
        return true;
        
    default:
        // Not handled here
        return false;
    }
}

static bool renderSidebarsUntil(uint32_t limit_micros)
{
    static int count = -1;
    static const int maxCount = (2*AH_SIDEBAR_HEIGHT_POS + 1) * charHeight + 1;

    if (!cachedSidebars) {
        return true; // Nothing to do here.
    }

    int x1 = infoSidebars.x1;
    int x2 = infoSidebars.x2;

    if (count < 0) {
        // Render the central indicators. 
        int yMid = infoSidebars.yMid;
        for (int i=1; i<6; ++i) {
            plot(x1 + 16 - i, yMid + i, 2);
            plot(x1 + 16 - i, yMid + i - 1, 1);
            plot(x1 + 16 - i, yMid - i, 2);
            plot(x1 + 16 - i, yMid - i - 1, 1);
            plot(x2 - 16 + i, yMid + i, 2);
            plot(x2 - 16 + i, yMid + i - 1, 1);
            plot(x2 - 16 + i, yMid - i, 2);
            plot(x2 - 16 + i, yMid - i - 1, 1);
        }

        count++;
    }

    int y = count + infoSidebars.y1;
    while (micros() < limit_micros && count < maxCount) {
        // bprintf("y = %d, x1=%d, x2=%d, y1 = %d", y,x1,x2, y1);
        // This is borderline for wanting to break down further (not to exceed limit_micros of around 20us by too much)
        if (count % 16 == 0) {
            dhLine(x1-2, y, 5);
            dhLine(x2-2, y, 5);
        } else if (count % 8 == 0) {
            dhLine(x1-5, y, 11);
            dhLine(x2-5, y, 11);
        }

        y++;
        count++;
    }

    if (count == maxCount) {
        count = -1; // Restart with central indicators
        cachedSidebars = false;
        return true; // All done with Sidebars.
    }

    return false;
}

static bool renderAHUntil(uint32_t limit_micros)
{
    static bool first = true;

    if (!cachedAH) {
        return true; // Nothing to do here.
    }

    if (first) {
        iterLineInit(infoArtificialHorizon.x1, infoArtificialHorizon.y1, infoArtificialHorizon.x2, infoArtificialHorizon.y2);
        first = false;
    }

    bool done = false;
    bool oor = infoArtificialHorizon.outOfRange;
    while (micros() < limit_micros && !done) {
        done = oor ? iterDashedDLineNext() :  iterDLineNext();
    }

    if (done) {
        // All done.
        first = true;
        cachedAH = false;
        return true; // Done with AH for this round.
    }

    return false;
}

bool renderCharsUntil(uint32_t limit_micros)
{

    // Saved state.
    // Position: currentChar (and cached currentY, currentX, currentPtr).
    static int currentChar;
    static int currentY;
    static int currentX;
    static uint8_t *currentPtr;

    // ** BEWARE ** charsPerLine doesn't correspond with pixels or bytes per line,
    // because we have some spare: 368 pixels not 360 for alignment reasons

    // REM *** if we use hoffs or equivalent, do same in plot and other drawing routines
    const int hoffs = 0; //0..2 (using 90 of 92 bytes)
    const int pxpc = PICO_OSD_CHAR_WIDTH;
    const int bxpc = pxpc / 4; // 4 pixels per byte -> 3 bytes to go across by 1 char
    const int pypc = PICO_OSD_CHAR_HEIGHT;
    const int bpc  = bxpc * pypc;
    const int fbbpl = fb_nx / 4; // bytes per line = pixels per line / pixels per byte3
    const int fbbpNextLine = pypc * fbbpl - charsPerLine * bxpc; // byte increment from  (top left of) last char of line to first of next line.

    if (0 == currentChar) {
        currentY = 0;
        currentX = 0;
        currentPtr = osdBufferA + hoffs;
    }

    tus++;
    while (currentY < charLines) {
        // currentPtr is pointer to topleft of char dest on osdBufferA
        while (cmpTimeUs(limit_micros, micros()) > 0 && currentX < charsPerLine) {
            uint8_t c = osdCharBuffer[currentChar++];
            // Buffer is always cleared after vsync before we start updating it. So, we can
            // ignore empty characters.
            // *** TODO check char 0 and char 32 (spc) are always transparent
//            if (currentY >= 14 && currentY <= 15) c = 0x17; // <-- bad with PiB output and PAL
//            if (currentY >= 14 && currentY <= 15) c = 0x9d; // not a problem
            if (c!=0 && c!=0x20) {
                // 1 char = 12 pixels = 3 bytes. 4 chars = 48 pixels = 12 bytes = 3 words
                const uint8_t * fontp = &fontData[c * bpc]; // 3 bytes per 12 pixel char line, 18 lines
                uint8_t * bufPtr = currentPtr;
                for (int j=0; j<pypc; ++j) {
                    // write out loop of bxpc (bytes per char = 3)
                    *bufPtr++ = *fontp++;
                    *bufPtr++ = *fontp++;
                    *bufPtr++ = *fontp++;
                    bufPtr += fbbpl - 3; // new line, back 3 bytes
                }
            }

            currentPtr += bxpc;
            currentX++;
        }

        if (currentX < charsPerLine) {
            break;
        }

        currentX = 0;
        currentY++;
        currentPtr += fbbpNextLine;
    }

    if (currentChar == numChars) { // equivalently currentY == charLines
        // Reached the end, reset.
        currentChar = 0;
        return true;
    }

    return false;
}

#if 0
        int bs = 252; int bx = 18; // bad
//        int bs = 262; int bx = 8; // ok
//        int bs = 252; int bx = 8; // ok
        for (int badline = bs; badline < bs + bx; badline+=1) {
            uint32_t *ptr = (uint32_t *)(osdBufferA + (fbbpl * badline));
            for (int x=0; x<92/4 /*92*/; ++x) {
//                *ptr++ = 0xf5555f55; // 0xff; // 55;
//                *ptr++ = 0x55555f55; // 0xff; // 55;
                *ptr++ = 0x55555555; // 0xff; // 55;
            }
        }
#elif 0
//        int bs = 252; int bx = 18; // bad - but maybe just timing
//        int bs = 262; int bx = 8; // ok
//        int bs = 252; int bx = 8; // ok
        int bs = 260; int bx = 3; // ok - fixes frame on row 14
        for (int badline = bs; badline < bs + bx; badline+=1) {
            uint8_t *ptr = (osdBufferA + (fbbpl * badline));
//            for (int x=0; x<92 /*92*/; ++x) {
//            for (int x=24; x<64; ++x) {
//            for (int x=24; x<28; ++x) { // <-- doesn't clear problem
            for (int x=44; x<48; ++x) { // clears the problem
//            for (int x=20; x<60 /*92*/; ++x) {
//                *ptr++ = 0xf5555f55; // 0xff; // 55;
//                *ptr++ = 0x55555f55; // 0xff; // 55;
//                *ptr++ = 0xff; // 
//                *ptr++ = 0x55;
//                *ptr++ = (x < 32 || x > 70) ? 0x55 : 0xff;
//                *ptr++ = (x < 32 || x > 70) ? 0xff : 0x55;
                ptr[x] = 0xff;
//                ptr++;
            }
        }
#endif

bool renderSticksBackgroundUntil(uint32_t limit_micros)
{
    // cachedStickLeft, cachedStickRight, "state" are the state.
    static int state;

    while (micros() < limit_micros && cachedStickLeft == 1) {
        int xMid = infoStickLeft.xLeft + stickWidth / 2;
        int yMid = infoStickLeft.yTop + stickHeight / 2;
        if (state == 0) {
            dhLine(infoStickLeft.xLeft, yMid, stickWidth);
            state++;
        } else {
            dvLine(xMid, infoStickLeft.yTop, stickHeight);
            state = 0;
            cachedStickLeft = 2; // next render stick position.
        }
    }

    while (micros() < limit_micros && cachedStickRight == 1) {
        int xMid = infoStickRight.xLeft + stickWidth / 2;
        int yMid = infoStickRight.yTop + stickHeight / 2;
        if (state == 0) {
            dhLine(infoStickRight.xLeft, yMid, stickWidth);
            state++;
        } else {
            dvLine(xMid, infoStickRight.yTop, stickHeight);
            state = 0;
            cachedStickRight = 2; // next render stick position.
        }
    }

    return (cachedStickLeft != 1 && cachedStickRight != 1);
}

    
bool renderSticksForegroundUntil(uint32_t limit_micros)
{
    while (micros() < limit_micros && cachedStickLeft == 2) {
        plotBlob(infoStickLeft.xStick, infoStickLeft.yStick);
        cachedStickLeft = 0;
    }

    while (micros() < limit_micros && cachedStickRight == 2) {
        plotBlob(infoStickRight.xStick, infoStickRight.yStick);
        cachedStickRight = 0;
    }

    return (cachedStickLeft == 0 && cachedStickRight == 0);
}

    
// Update screen buffer (paint characters etc to buffer), up until a time limit.
// Store state so that we can resume.
// Return false when complete (no more to do).
bool osdPioDrawScreenUntil(uint32_t limit_micros)
{
#if 0
    UNUSED(limit_micros);
    plotTestCard();
    return false;
#endif

    static uint32_t maxcyclesthisround;

    uint32_t c1 = getCycleCounter();

    bool complete =
        renderSticksBackgroundUntil(limit_micros) &&
        renderSidebarsUntil(limit_micros) &&
        renderAHUntil(limit_micros) &&
        renderCharsUntil(limit_micros) &&
        renderSticksForegroundUntil(limit_micros);

    uint32_t cd = getCycleCounter() - c1;
    if (cd > maxcyclesthisround) {
        maxcyclesthisround = cd;
    }

    if (complete) {
        // accumulate for averaging: maxcycles += maxcyclesthisround;
        maxcycles = maxcyclesthisround;
        tusr++;
        maxcyclesthisround = 0;

        transferredSinceVsync = true;
        return false; // Nothing more to draw.
    }

    return true;
}

void testUpdate(void)
{
    tus++;
//#define clearscreen
//#define testcard
#define textpaint
//#define blockpaint
//#define ahpaint
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
//        osdBufferA[i] = 0;
//    }
    {
        void *__aeabi_memset(void *s, size_t n, int c); // maybe , size_t n);    
//        __aeabi_memset(osdBufferA, PICO_OSD_BUF_LENGTH, 0xff);
        __aeabi_memset(osdBufferA, PICO_OSD_BUF_LENGTH, 0);

//        uint32_t *p = (uint32_t *)osdBufferA;
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
    memset(osdBufferA, 0, PICO_OSD_BUF_LENGTH); // transparent background
#endif

    plotTestCard();

#elif 0
    if (0 == (millis() % 5000) ) { parity = 1 - parity; }

//    if (parity) {
    if (1) {
//    memset(osdBuffer1, 0b10101010, PICO_OSD_BUF_LENGTH/2);
//    memset(osdBuffer1 + PICO_OSD_BUF_LENGTH/2, 0xff, PICO_OSD_BUF_LENGTH/2);
    memset(osdBufferA, 0xff, PICO_OSD_BUF_LENGTH/2);
    memset(osdBufferA + PICO_OSD_BUF_LENGTH/2, 0b10101010, PICO_OSD_BUF_LENGTH/2);

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
        memset(osdBufferA, 0, PICO_OSD_BUF_LENGTH);
    }
#if 0
  int s = millis()/4234;
    for (int i=0; i<PICO_OSD_BUF_LENGTH; ++i) {
        osdBufferA[i] = (0b1010101) * ((s >> 10)&3);
        if (i%577 == 234)
            s = s*13+29;
    }
    memset(osdBufferA, 0b10101010 /*0xff*/, PICO_OSD_BUF_LENGTH/2);
    memset(osdBufferA + PICO_OSD_BUF_LENGTH/2, 0xff /* 0b10101010*/, PICO_OSD_BUF_LENGTH/2);
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

//    for (int i=0; i<10; ++i) {
    for (int i=0; i<numChars; ++i) {
        uint8_t c = osdCharBuffer[i];
        if (!c) {
            continue;
        }

        // paint chars to buffer here
        // 1 char = 12 pixels = 3 bytes. 4 chars = 48 pixels = 12 bytes = 3 words
        int x = i % charsPerLine;
        int y = i / charsPerLine; // or loop x,y
        uint8_t * bufp = osdBufferA + hoffs + fbbpl * y * pypc + x * bxpc; // pointer to topleft of char dest on osdBufferA
        // TODO bufp without multiply, loop x,y etc.

        const uint8_t * fontp = &fontData[c*bpc]; // 3 bytes per 12 pixel char line, 18 lines
        // rp2350 don't have to worry about cache, all 1-clock sram

//        bprintf("painting char '%c' (0x%02x) at %d, %d from %p (cf %p) to %p (cf %p)",
//                c, c, x, y, fontp, fontData, bufp, osdBufferA);
        for (int j=0; j<18; ++j) {
            for (int b=0; b<3; ++b) {
                *bufp++ = *fontp++;
            }
            bufp += fbbpl - 3; // new line, back 3 bytes
        }
    }


#endif

    //bzero(osdBufferA, PICO_OSD_BUF_LENGTH);
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
    //bzero(osdBufferA, PICO_OSD_BUF_LENGTH);
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
        bzero(osdBufferA, PICO_OSD_BUF_LENGTH);
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
        osdCharBuffer[y*charsPerLine + x] = c;
    }
}

void osdPioWrite(uint8_t x, uint8_t y, const char *text)
{
    if (y < charLines) {
        uint8_t *p = osdCharBuffer + y * charsPerLine;
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
