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

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"

#include "drivers/system.h"
#include "drivers/io.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart_impl.h"

#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/uart.h"

#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

// The PIO block for software UARTs UART2, UART3
static const PIO uartPio = PIO_INSTANCE(UART_PIO_INDEX);
static int txProgram_offset = -1;
static int rxProgram_offset = -1;

typedef struct pioDetails_s {
    irq_num_t irqn;
    int rxPin;
    int txPin;
    uint16_t sm_rx;
    uint16_t sm_tx;
} pioDetails_t;

// Store for details, assuming possible UART2, UART3
static pioDetails_t uartPioDetails[2];
#define UART_PIO_DETAILS_IDX(id) (id - SERIAL_PORT_UART2)
#define UART_PIO_DETAILS_PTR(id) (&uartPioDetails[UART_PIO_DETAILS_IDX(id)])

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART0
    {
        .identifier = SERIAL_PORT_UART0,
        .reg = uart0,
        .rxPins = {
            { DEFIO_TAG_E(PA1) },
            { DEFIO_TAG_E(PA3) },
            { DEFIO_TAG_E(PA13) },
            { DEFIO_TAG_E(PA15) },
            { DEFIO_TAG_E(PA17) },
            { DEFIO_TAG_E(PA19) },
            { DEFIO_TAG_E(PA29) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA31) },
            { DEFIO_TAG_E(PA33) },
            { DEFIO_TAG_E(PA35) },
            { DEFIO_TAG_E(PA45) },
            { DEFIO_TAG_E(PA47) },
#endif
        },
        .txPins = {
            { DEFIO_TAG_E(PA0) },
            { DEFIO_TAG_E(PA2) },
            { DEFIO_TAG_E(PA12) },
            { DEFIO_TAG_E(PA14) },
            { DEFIO_TAG_E(PA16) },
            { DEFIO_TAG_E(PA18) },
            { DEFIO_TAG_E(PA28) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA30) },
            { DEFIO_TAG_E(PA32) },
            { DEFIO_TAG_E(PA34) },
            { DEFIO_TAG_E(PA44) },
            { DEFIO_TAG_E(PA46) },
#endif
        },
        .irqn = UART0_IRQ,
        .txBuffer = uart0TxBuffer,
        .rxBuffer = uart0RxBuffer,
        .txBufferSize = sizeof(uart0TxBuffer),
        .rxBufferSize = sizeof(uart0RxBuffer),
    },
#endif

#ifdef USE_UART1
    {
        .identifier = SERIAL_PORT_UART1,
        .reg = uart1,
        .rxPins = {
            { DEFIO_TAG_E(PA5) },
            { DEFIO_TAG_E(PA7) },
            { DEFIO_TAG_E(PA9) },
            { DEFIO_TAG_E(PA11) },
            { DEFIO_TAG_E(PA21) },
            { DEFIO_TAG_E(PA23) },
            { DEFIO_TAG_E(PA25) },
            { DEFIO_TAG_E(PA27) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA37) },
            { DEFIO_TAG_E(PA39) },
            { DEFIO_TAG_E(PA41) },
            { DEFIO_TAG_E(PA43) },
#endif
        },
        .txPins = {
            { DEFIO_TAG_E(PA4) },
            { DEFIO_TAG_E(PA6) },
            { DEFIO_TAG_E(PA8) },
            { DEFIO_TAG_E(PA10) },
            { DEFIO_TAG_E(PA20) },
            { DEFIO_TAG_E(PA22) },
            { DEFIO_TAG_E(PA24) },
            { DEFIO_TAG_E(PA26) },
#ifdef RP2350B
            { DEFIO_TAG_E(PA36) },
            { DEFIO_TAG_E(PA38) },
            { DEFIO_TAG_E(PA40) },
            { DEFIO_TAG_E(PA42) },
#endif
        },
        .irqn = UART1_IRQ,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif

    // PIO-based UARTs. For now, hardwired to UARTs 2,3,4,5 on PIO number UART_PIO_INDEX.
#ifdef USE_UART2
    {
        .identifier = SERIAL_PORT_UART2,
        .reg = (USART_TypeDef *)uartPio,
        .irqn = PIO_IRQ_NUM(uartPio, 0),
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif

#ifdef USE_UART3
    {
        .identifier = SERIAL_PORT_UART3,
        .reg = (USART_TypeDef *)uartPio,
        .irqn = PIO_IRQ_NUM(uartPio, 1), // TODO is there benefit in UART2 and UART3 going through distinct IRQs?
        // makes coding easier - don't need to keep track of assigned statemachine since will only be
        // one sm generating interrupts [for RX not empty] on each IRQ?
        // eventually TX by DMA, but that's a DREQ (we don't have to deal with a PIO interrupt) [TBC]
        .txBuffer = uart3TxBuffer,
        .rxBuffer = uart3RxBuffer,
        .txBufferSize = sizeof(uart3TxBuffer),
        .rxBufferSize = sizeof(uart3RxBuffer),
    },
#endif
};

#define PIO_IRQ_INDEX(irqn) (irqn == PIO_IRQ_NUM(uartPio, 0) ? 0 : 1)

bool isHardwareUART(serialPortIdentifier_e identifier)
{
    return identifier == SERIAL_PORT_UART0 || identifier == SERIAL_PORT_UART1;
}

uartPinDef_t makePinDef(ioTag_t tag)
{
    uartPinDef_t ret = { .pin = tag };
    return ret;
}

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
#if UART_TRAIT_PINSWAP
    STATIC_ASSERT(false, pico_no_UART_pinswap);
#endif
    bprintf("pico uartPinConfigure");

    int pinIndexMin = 48;
    int pinIndexMax = -1;
    int pioBase = 0;
    for (const uartHardware_t* hardware = uartHardware; hardware < ARRAYEND(uartHardware); hardware++) {
        const serialPortIdentifier_e identifier = hardware->identifier;
        uartDevice_t* uartdev = uartDeviceFromIdentifier(identifier);
        const int resourceIndex = serialResourceIndex(identifier);
        if (uartdev == NULL || resourceIndex < 0) {
            // malformed uartHardware
            bprintf("* pico uartPinConfigure %p malformed, uartdev %p, resourceIndex %d",hardware, uartdev, resourceIndex);
            continue;
        }
        const ioTag_t cfgRx = pSerialPinConfig->ioTagRx[resourceIndex];
        const ioTag_t cfgTx = pSerialPinConfig->ioTagTx[resourceIndex];
        bprintf("pico uartPinConfigure hw = %p dev = %p,  tags rx 0x%x, tx 0x%x", hardware, uartdev, cfgRx, cfgTx);
        if (!cfgRx && !cfgTx) {
            continue;
        }

        if (isHardwareUART(identifier)) {
            for (unsigned pindex = 0; pindex < UARTHARDWARE_MAX_PINS; pindex++) {
                if (cfgRx && cfgRx == hardware->rxPins[pindex].pin) {
                    uartdev->rx = hardware->rxPins[pindex];
                }

                if (cfgTx && cfgTx == hardware->txPins[pindex].pin) {
                    uartdev->tx = hardware->txPins[pindex];
                }
            }

        } else {
            // software UART by PIO
            // On a single PIO block, we are restricted either to pins 0-31 or pins 16-47.
            pinIndexMin = cfgRx && (DEFIO_TAG_PIN(cfgRx) < pinIndexMin) ? DEFIO_TAG_PIN(cfgRx) : pinIndexMin;
            pinIndexMax = cfgRx && (DEFIO_TAG_PIN(cfgRx) > pinIndexMax) ? DEFIO_TAG_PIN(cfgRx) : pinIndexMax;
            pinIndexMin = cfgTx && (DEFIO_TAG_PIN(cfgTx) < pinIndexMin) ? DEFIO_TAG_PIN(cfgTx) : pinIndexMin;
            pinIndexMax = cfgTx && (DEFIO_TAG_PIN(cfgTx) > pinIndexMax) ? DEFIO_TAG_PIN(cfgTx) : pinIndexMax;
            if (pinIndexMax >= 32) {
                if (pinIndexMin < 16) {
                    bprintf("* Not configuring UART%d (PIO can't span pins min %d max %d)",
                            uartDeviceIdxFromIdentifier(identifier), pinIndexMin, pinIndexMax);
                    continue;
                } else {
                    pioBase = 16;
                }
            }

            if (cfgRx) {
                uartdev->rx = makePinDef(cfgRx);
            }

            if (cfgTx) {
                uartdev->tx = makePinDef(cfgTx);
            }
        }

        if (uartdev->rx.pin || uartdev->tx.pin ) {
            uartdev->hardware = hardware;
        } else {
            bprintf("\n ** unexpected no rx.pin or tx.pin even though cfgRx or cfgTx");
        }
    }

    bprintf("pico uartPinConfigure pio%d pin min, max = %d, %d; setting gpio base to %d", PIO_NUM(uartPio), pinIndexMin, pinIndexMax, pioBase);

    // The GPIO base must be set before adding the program.
    pio_set_gpio_base(uartPio, pioBase);
}

bool ensurePioProgram(PIO pio, const pio_program_t *program, bool isTx)
{
    if (isTx) {
        if (txProgram_offset < 0) {
            txProgram_offset = pio_add_program(pio, program);
        }

        return txProgram_offset >= 0;
   } else {
        if (rxProgram_offset < 0) {
            rxProgram_offset = pio_add_program(pio, program);
        }

        return rxProgram_offset >= 0;
    }
}

#if 0
// poll until we have emptied rx fifo (if we can)
static void poll_rx(PIO pio, uint sm)
{
    // 8-bit read from the uppermost byte of the FIFO, as data is left-justified
    io_rw_8 *rxfifo_shift = (io_rw_8*)&pio->rxf[sm] + 3;
    int cc=0;
    char c=0;
    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        cc++;
        c = (char) *rxfifo_shift;
    }
    bprintf("rx polled %d, last character 0x%x",cc, c);

    bool isempty = pio_sm_is_rx_fifo_empty(pio, sm);
    bprintf("rx fifo for sm %d is_empty=%d", sm, isempty);
}
#endif

#ifdef PICO_TRACE
#include "drivers/time.h"
#endif
void uartIrqHandler(uartPort_t *s)
{
#ifdef IRQMEASURESOME
    static int uic;
    static timeUs_t tt;
    static timeUs_t tl;
    if (!uic) {
        tt = micros();
        tl = tt;
    }
    uic++;
    if ((uic % 1000) == 0) {
        timeUs_t tnow = micros();
        int td = tnow - tl;
        int ttot = tnow - tt;
        tl = tnow;
        bprintf("uartIrq total %d, last %.1fHz overall %.1f Hz", uic, 1000000000.0/td, (1000000.0 * uic)/ttot);
    }
#endif
////    bprintf("uartIrqHandler");

    uart_inst_t *uartInstance = UART_INST(s->USARTx);
    if ((uart_get_hw(uartInstance)->imsc & (UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS)) != 0) {
        //bprintf("uartIrqHandler RX");
        while (uart_is_readable(uartInstance)) {
            const uint8_t ch = uart_getc(uartInstance);
            //bprintf("uartIrqHandler RX %x", ch);
            if (s->port.rxCallback) {
                s->port.rxCallback(ch, s->port.rxCallbackData);
            } else {
                bprintf("RX %x -> buffer",ch);
                s->port.rxBuffer[s->port.rxBufferHead] = ch;
                s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
            }
        }
    }

    if ((uart_get_hw(uartInstance)->imsc & UART_UARTIMSC_TXIM_BITS) != 0) {

//        int c = s->port.txBufferHead - s->port.txBufferTail;
        while (uart_is_writable(uartInstance)) {
            if (s->port.txBufferTail != s->port.txBufferHead) {
                ///bprintf("uartIrqHandler TX put %x", s->port.txBuffer[s->port.txBufferTail]);
                uart_putc(uartInstance, s->port.txBuffer[s->port.txBufferTail]);
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            } else {
                // TODO check, RX enabled based on mode?
                //bprintf("uart done put %d, disabling tx interrupt",c);
                uart_set_irqs_enabled(uartInstance, s->port.mode & MODE_RX, false);
                break;
            }
        }
    }
}

static void on_uart0(void)
{
    uartIrqHandler(&uartDevice[UARTDEV_0].port);
}

static void on_uart1(void)
{
    uartIrqHandler(&uartDevice[UARTDEV_1].port);
}

bool serialUART_hardware(uint32_t baudRate, portMode_e mode, portOptions_e options,
                        const uartHardware_t *hardware, serialPortIdentifier_e identifier, IO_t txIO, IO_t rxIO)
{
    UNUSED(options); // TODO ?
    UNUSED(mode); // TODO ?

    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier); // rx is always +1

    if (txIO) {
        IOInit(txIO, ownerTxRx, ownerIndex);
        uint32_t txPin = IO_Pin(txIO);
        bprintf("gpio set function UART on tx pin %d", txPin);
        gpio_set_function(txPin, GPIO_FUNC_UART);
    }

    if (rxIO) {
        IOInit(rxIO, ownerTxRx + 1, ownerIndex);
        uint32_t rxPin = IO_Pin(rxIO);
        gpio_set_function(rxPin, GPIO_FUNC_UART);
        bprintf("gpio set function UART on rx pin %d", rxPin);
        gpio_set_pulls(rxPin, true, false); // Pull up
    }

    uart_inst_t *uartInstance = UART_INST(hardware->reg);
    bprintf("serialUART uart init %p baudrate %d", uartInstance, baudRate);
    uart_init(uartInstance, baudRate);

    // TODO implement - use options here...
    uart_set_hw_flow(uartInstance, false, false);
    uart_set_format(uartInstance, 8, 1, UART_PARITY_NONE);

// TODO want fifos?
////    uart_set_fifo_enabled(uart, false);
    uart_set_fifo_enabled(uartInstance, true);

    bprintf("\n ** going to set exclusive handler and enable for irqn %d cf. calculated for uartPio IRQ0: %d",
            hardware->irqn, pio_get_irq_num(uartPio, 0));
    irq_set_exclusive_handler(hardware->irqn, hardware->irqn == UART0_IRQ ? on_uart0 : on_uart1);
    irq_set_enabled(hardware->irqn, true);

    // Don't enable any uart irq yet, wait until a call to uartReconfigure...
    // (with current code in serial_uart.c, this prevents irq callback before rxCallback has been set)
    // TODO review serial_uart.c uartOpen()
    return true;
}

static void uartPioIrqHandler(uartPort_t *s, pioDetails_t *pioDetailsPtr)
{
    UNUSED(s);
    UNUSED(pioDetailsPtr);
//    bprintf("**** TODO");
    uint sm_rx = pioDetailsPtr->sm_rx;

    // 8-bit read from the uppermost byte of the FIFO, as data is left-justified
    io_rw_8 *rxfifo_shift = (io_rw_8*)&uartPio->rxf[sm_rx] + 3;
    serialReceiveCallbackPtr rxCallback = s->port.rxCallback;
    if (rxCallback) {
        void *rxCallbackData = s->port.rxCallbackData;
        while (!pio_sm_is_rx_fifo_empty(uartPio, sm_rx)) {
            const uint8_t ch = (uint8_t)*rxfifo_shift;
            rxCallback(ch, rxCallbackData);
        }
    } else {
        volatile uint8_t *rxBuffer = s->port.rxBuffer;
        uint32_t rxBufferSize = s->port.rxBufferSize;
        while (!pio_sm_is_rx_fifo_empty(uartPio, sm_rx)) {
            const uint8_t ch = (uint8_t)*rxfifo_shift;
            rxBuffer[s->port.rxBufferHead] = ch;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % rxBufferSize;
        }
    }

#if 0
todo tx
    if ((uart_get_hw(uartInstance)->imsc & UART_UARTIMSC_TXIM_BITS) != 0) {

//        int c = s->port.txBufferHead - s->port.txBufferTail;
        while (uart_is_writable(uartInstance)) {
            if (s->port.txBufferTail != s->port.txBufferHead) {
                ///bprintf("uartIrqHandler TX put %x", s->port.txBuffer[s->port.txBufferTail]);
                uart_putc(uartInstance, s->port.txBuffer[s->port.txBufferTail]);
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            } else {
                // TODO check, RX enabled based on mode?
                //bprintf("uart done put %d, disabling tx interrupt",c);
                uart_set_irqs_enabled(uartInstance, s->port.mode & MODE_RX, false);
                break;
            }
        }
    }
#endif
}

static void on_uart2(void)
{
///    bprintf("\n\n on_uart2");
    uartPioIrqHandler(&uartDevice[UARTDEV_2].port, UART_PIO_DETAILS_PTR(SERIAL_PORT_UART2));
}

static void on_uart3(void)
{
///    bprintf("\n\n\n\non_uart3");
    uartPioIrqHandler(&uartDevice[UARTDEV_2].port, UART_PIO_DETAILS_PTR(SERIAL_PORT_UART2));
}

bool serialUART_pio(uint32_t baudRate, portMode_e mode, portOptions_e options,
                    const uartHardware_t *hardware, serialPortIdentifier_e identifier, IO_t txIO, IO_t rxIO)
{
    UNUSED(options); // TODO ?
    UNUSED(mode); // TODO ?

    const int ownerIndex = serialOwnerIndex(identifier);
    const resourceOwner_e ownerTxRx = serialOwnerTxRx(identifier); // rx is always +1
    pioDetails_t *uartPioDetailsPtr = UART_PIO_DETAILS_PTR(identifier);
    uartPioDetailsPtr->irqn = hardware->irqn;

    if (txIO) {
        IOInit(txIO, ownerTxRx, ownerIndex);
        uint32_t txPin = IO_Pin(txIO);
        bprintf("set up PIO for UART on tx pin %d", txPin);
        if (!ensurePioProgram(uartPio, &uart_tx_program, true /* isTx */)) {
            bprintf("pico serialUART_pio tx failed to add program to pio");
            return false;
        }

        const int pio_sm_tx = pio_claim_unused_sm(uartPio, false);
        if (pio_sm_tx < 0) {
            bprintf("pico serialUART_pio tx failed to claim state machine");
            return false;
        }

        uartPioDetailsPtr->txPin = txPin;
        uartPioDetailsPtr->sm_tx = pio_sm_tx;

        // Arrange GPIO, assign PIO SM pins, FIFO, clock, enable SM
        uart_tx_program_init(uartPio, pio_sm_tx, txProgram_offset, txPin, baudRate);
    }

    if (rxIO) {
        IOInit(rxIO, ownerTxRx + 1, ownerIndex);
        uint32_t rxPin = IO_Pin(rxIO);
        bprintf("set up PIO for UART on rx pin %d", rxPin);
        if (!ensurePioProgram(uartPio, &uart_rx_program, false /* isTx */)) {
            bprintf("pico serialUART_pio rx failed to add program to pio");
            return false;
        }

        const int pio_sm_rx = pio_claim_unused_sm(uartPio, false);
        if (pio_sm_rx < 0) {
            bprintf("pico serialUART_pio rx failed to claim state machine");
            return false;
        }

        uartPioDetailsPtr->rxPin = rxPin;
        uartPioDetailsPtr->sm_rx = pio_sm_rx;

        // Arrange GPIO including pullup for RX, assign PIO SM pins, FIFO, clock, enable SM
        bprintf("\nserial_uart pio init RX program on pin %d, sm %d", rxPin, pio_sm_rx);
        uart_rx_program_init(uartPio, pio_sm_rx, rxProgram_offset, rxPin, baudRate);
    }

    bprintf("serialUART.. pio/uart (requested) baudrate %d", baudRate);

    // TODO implement - use options here...
//    uart_set_hw_flow(uartInstance, false, false);
//    uart_set_format(uartInstance, 8, 1, UART_PARITY_NONE);

// TODO want fifos?
////    uart_set_fifo_enabled(uart, false);
//    uart_set_fifo_enabled(uartInstance, true);

    bprintf("id %d, going to set exclusive handler for irqn %d, hope noone else is looking at it...", hardware->identifier, hardware->irqn);
    irq_set_exclusive_handler(hardware->irqn, hardware->identifier == SERIAL_PORT_UART2 ? on_uart2 : on_uart3); // TODO more general
    irq_set_enabled(hardware->irqn, true);

    // Don't enable pio irq yet, wait until a call to uartReconfigure...
    // (with current code in serial_uart.c, this prevents irq callback before rxCallback has been set)
    return true;
}

uartPort_t *serialUART(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    bprintf("\nserialUART");
    uartPort_t *s = &uartdev->port;
    const uartHardware_t *hardware = uartdev->hardware;

    IO_t txIO = IOGetByTag(uartdev->tx.pin);
    IO_t rxIO = IOGetByTag(uartdev->rx.pin);

    if (!txIO && !rxIO) {
        bprintf("serialUART no pins mapped for device %p", s->USARTx);
        return NULL;
    }

    const serialPortIdentifier_e identifier = s->port.identifier;
    // SERIAL_PORT_UART0, 1, 2, 3, ...

    bool uartInitialised;
    if (isHardwareUART(identifier)) {
        uartInitialised = serialUART_hardware(baudRate, mode, options,
                                              hardware, identifier, txIO, rxIO);
    } else {
        uartInitialised = serialUART_pio(baudRate, mode, options,
                                              hardware, identifier, txIO, rxIO);
    }

    if (!uartInitialised) {
        bprintf("* Failed to initialised uart device %p, id %d", hardware->reg, identifier);
        return NULL;
    }

    s->port.vTable = uartVTable;
    s->port.baudRate = baudRate; // TODO set by caller?
    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;

    s->USARTx = hardware->reg;
    bprintf("====== setting USARTx to reg == %p", s->USARTx);
    return s;
}

// called from platform-specific uartReconfigure
void uartConfigureExternalPinInversion(uartPort_t *uartPort)
{
#if !defined(USE_INVERTER)
    UNUSED(uartPort);
#else
    const bool inverted = uartPort->port.options & SERIAL_INVERTED;
    // TODO support INVERTER, not using enableInverter(= pin based)
    enableInverter(uartPort->port.identifier, inverted);
#endif
}

void uartEnableTxInterrupt(uartPort_t *uartPort)
{
    // testing only!
    const serialPortIdentifier_e identifier = uartPort->port.identifier;
/////////    pioDetails_t *uartPioDetailsPtr = UART_PIO_DETAILS_PTR(identifier);
////////    uint sm_rx = uartPioDetailsPtr->sm_rx;
/////////    poll_rx(uartPio, sm_rx);

    if (isHardwareUART(identifier)) {
//    bprintf("uartEnableTxInterrupt");
        if (uartPort->port.txBufferTail == uartPort->port.txBufferHead) {
            return;
        }

        // uart0TxBuffer has size 1024
        
#if 0
        bprintf("uartEnableTxInterrupt %d (head:0x%x, tail:0x%x)",
                uartPort->port.txBufferHead - uartPort->port.txBufferTail,
                uartPort->port.txBufferHead,
                uartPort->port.txBufferTail);
        bprintf("going to set interrupts for uart %p", uartPort->USARTx);
#endif

        // TODO Check: rx mask based on mode rather than RX interrupt pending?
        //    uart_set_irqs_enabled(s->USARTx, uart_get_hw(s->USARTx)->imsc & UART_UARTIMSC_RXIM_BITS, true);
        uart_set_irqs_enabled(UART_INST(uartPort->USARTx), uartPort->port.mode & MODE_RX, true);
    } else {
        bprintf("***** enableTxInterrupt REM TODO PIO");
    }
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    UNUSED(s);
    //TODO: Implement
}
#endif

void uartReconfigure_hardware(uartPort_t *s)
{
    uart_inst_t *uartInstance = UART_INST(s->USARTx);
    bprintf("uartReconfigure for port %p with USARTX %p", s, uartInstance);
    int achievedBaudrate = uart_init(uartInstance, s->port.baudRate);
#ifdef PICO_TRACE
    bprintf("uartReconfigure h/w %p, requested baudRate %d, achieving %d", uartInstance, s->port.baudRate, achievedBaudrate);
#else
    UNUSED(achievedBaudrate);
#endif
    uart_set_format(uartInstance, 8, 1, UART_PARITY_NONE);

    // TODO fifo or not to fifo?
    //    uart_set_fifo_enabled(s->USARTx, false);
    uart_set_fifo_enabled(uartInstance, true);
    uartConfigureExternalPinInversion(s);
    uart_set_hw_flow(uartInstance, false, false);

// TODO would like to verify rx pin has been setup?
//    if ((s->mode & MODE_RX) && rxIO) {
    if (s->port.mode & MODE_RX) {
        bprintf("serialUART setting RX irq");
        uart_set_irqs_enabled(uartInstance, true, false);
    }

    bprintf("uartReconfigure note port.mode = 0x%x", s->port.mode);
    // TODO should we care about MODE_TX ?

}


void uartReconfigure_pio(uartPort_t *s)
{
    bprintf("uartReconfigure for port %p with PIO %p", s, uartPio);
    const serialPortIdentifier_e identifier = s->port.identifier;
    pioDetails_t *uartPioDetailsPtr = UART_PIO_DETAILS_PTR(identifier);
    uint sm_tx = uartPioDetailsPtr->sm_tx;
    uint sm_rx = uartPioDetailsPtr->sm_rx;

    uart_tx_program_init(uartPio, sm_tx, txProgram_offset, uartPioDetailsPtr->txPin, s->port.baudRate);
    uart_rx_program_init(uartPio, sm_rx, rxProgram_offset, uartPioDetailsPtr->rxPin, s->port.baudRate);

    // TODO PIO format currently restricted to 8n1
    // uart_set_format(uartInstance, 8, 1, UART_PARITY_NONE);
    // no hw flow
    // uart_set_hw_flow(uartInstance, false, false);
    uartConfigureExternalPinInversion(s);

    if (s->port.mode & MODE_RX) {
        irq_num_t irqn = uartPioDetailsPtr->irqn;
        int irqn_index = PIO_IRQ_INDEX(irqn);
        pio_interrupt_source_t irqSource = pio_get_rx_fifo_not_empty_interrupt_source(sm_rx);
        bprintf("\n pis_sm0_rx_fifo_not_empty = %d, sm_rx = %d, pgrneis(sm_rx) = %d",
                pis_sm0_rx_fifo_not_empty, sm_rx, irqSource);
        bprintf("\n ** serialUART reconfigure PIO RX [sm %d] enable source %d on irqn %d index %d", sm_rx, irqSource, irqn, irqn_index);
//        pio_set_irqn_source_enabled(uartPio, irqn, irqSource, true);
        pio_set_irqn_source_enabled(uartPio, irqn_index, irqSource, true);

//        bprintf("\n ** reenabling irq %d just for good measure", irqn);
//        irq_set_enabled(irqn, true);
    }

    bprintf("uartReconfigure note port.mode = 0x%x", s->port.mode);
    // TODO should we care about MODE_TX ?

#if 0
    uint32_t uartFr = uart_get_hw(uartInstance)->fr;
    bprintf("uartReconfigure flag register 0x%x",uartFr);
    bprintf("uartReconfigure extra call to on_uart1");
    on_uart1();
    bprintf("put some in...");
    uart_putc(uartInstance, 'A');
    uart_putc(uartInstance, 'B');
    uart_putc(uartInstance, 'C');
    uartFr = uart_get_hw(uartInstance)->fr;
    bprintf("uartReconfigure flag register 0x%x",uartFr);
    bprintf("wait a mo");
    extern void delayMicroseconds(uint32_t);
    delayMicroseconds(123456);
    uartFr = uart_get_hw(uartInstance)->fr;
    bprintf("uartReconfigure flag register 0x%x",uartFr);
    bprintf("uartReconfigure extra special call to on_uart1");
    on_uart1();
#endif
}

void uartReconfigure(uartPort_t *s)
{
    const serialPortIdentifier_e identifier = s->port.identifier;
    if (isHardwareUART(identifier)) {
        uartReconfigure_hardware(s);
    } else {
        uartReconfigure_pio(s);
    }
}

#endif /* USE_UART */
