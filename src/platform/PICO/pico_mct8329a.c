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

#ifdef PICO_ESC_MCT8329A

#include <strings.h>
#include "drivers/time.h"

#define TESTREGS

static uint8_t muxAddr = MCT8329A_MUX_ADDR;
static uint8_t muxResetPin = MCT8329A_MUX_RESET_GPIO;
static i2c_inst_t *muxi2c = I2C_INSTANCE(MCT8329A_MUX_I2C_INDEX);
static uint8_t MCTi2cLocation = MCT8329A_MCT_ADDR; // i2c location of MCT8329A

static uint8_t testReadRegs[] = {
    0x80, 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0x8E,
    0x90, 0x92, 0x94, 0x96, 0x98, 0x9A, 0x9C, 0x9E,
    0xA0, 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC, 0xAE
};

static const int numTestRegs = sizeof(testReadRegs) / sizeof(testReadRegs[0]);

void i2cMuxEnableDevice(int device)
{
    uint8_t buf = 1 << device;
    int wrote = i2c_write_blocking(muxi2c, muxAddr, &buf, 1, false);
    bprintf("mux wrote %d bytes, value 0x%02x (device %d)", wrote, buf, device);
    int readBack = i2c_read_blocking(muxi2c, muxAddr, &buf, 1, false);
    bprintf("read back %d bytes, value 0x%02x", readBack, buf);
#ifndef PICO_TRACE
    UNUSED(wrote);
    UNUSED(readBack);
#endif
}

bool readMCTRegister32(uint8_t i2cLocation, uint32_t mctAddress, uint32_t *result)
{
    // Read in 32-bit words, no CRC
    uint8_t control_word[3] = {0x90, (mctAddress&0x00000F00)>>8, mctAddress&0x000000FF};

    // bprintf("try read register %x on i2c location %x", mctAddress, i2cLocation);
    *result = 0;
//    uint32_t c1 = getCycleCounter();
    int res = i2c_write_blocking(muxi2c, i2cLocation, control_word, 3, true /*no stop*/);
    if (res != 3) {
        bprintf("i2c_write_blocking for control word returned %d", res);
        return false;
    }

//    uint32_t c2 = getCycleCounter();
//    dwait(); // assuming 100kHz
//    delayI2C();

    // previous no stop -> will do a restart here, sending the address before reading
    res = i2c_read_blocking(muxi2c, i2cLocation, (uint8_t *)result, 4, false /*stop*/);
//    uint32_t c3 = getCycleCounter();

//    bprintf("control word write: %.1f us, data read: %.1f us, total %.1f us",
//            ((int32_t)(c2-c1))/150.0, ((int32_t)(c3-c2))/150.0, ((int32_t)(c3-c1))/150.0);
    if (res != 4) {
        bprintf("i2c_read_blocking returned %d with result %08x", res, *result);
    }

    return res == 4;
}

bool writeMCTRegister32(uint8_t i2cLocation, uint32_t mctAddress, uint32_t data)
{
    // Write in 32-bit words, no CRC
    uint8_t control_word[3] = {0x10, (mctAddress&0x00000F00)>>8, mctAddress&0x000000FF};

    // we could set up a buffer with control_word followed by data, but this (use burst mode) is more convenient
    // and we have the option of inserting a delay between control word and data
//    uint32_t c1 = getCycleCounter();
    int res = i2c_write_burst_blocking(muxi2c, i2cLocation, control_word, 3);
    if (res != 3) {
        bprintf("i2c_write_burst_blocking for control word returned %d", res);
        return false;
    }
//    uint32_t c2 = getCycleCounter();
//    delayI2C();

    // uint32_t data is stored lo-endian, which is what is required for a sequence of uint8_t here.
    res = i2c_write_blocking(muxi2c, i2cLocation, (uint8_t *)(&data), 4, false /*stop*/);
//    uint32_t c3 = getCycleCounter();
//    bprintf("control word write: %.1f us, data write: %.1f us, total %.1f us",
//            ((int32_t)(c2-c1))/150.0, ((int32_t)(c3-c2))/150.0, ((int32_t)(c3-c1))/150.0);
     if (res != 4) {
        bprintf("i2c_write_blocking for data word returned %d", res);
        return false;
    }

    return true;
}

#if 0
static void i2cHardTestRW(unsigned long mct_addr, uint32_t val1, uint32_t val2)
{
    uint32_t result;
    bool res;
    res = readMCTRegister32(MCTi2cLocation, mct_addr, &result);
#ifndef PICO_TRACE
    UNUSED(res);
#endif
    bprintf("[result %d] MCT register %x (i2c location %x), contents read as %08x", res, mct_addr, MCTi2cLocation, result);
    res = writeMCTRegister32(MCTi2cLocation, mct_addr, val2);
    bprintf("[result %d] MCT register %x (i2c location %x), wrote %08x", res, mct_addr, MCTi2cLocation, val2);
    res = readMCTRegister32(MCTi2cLocation, mct_addr, &result);
    bprintf("[result %d] MCT register %x (i2c location %x), contents read as %08x", res, mct_addr, MCTi2cLocation, result);
    res = writeMCTRegister32(MCTi2cLocation, mct_addr, val1);
    bprintf("[result %d] MCT register %x (i2c location %x), wrote %08x", res, mct_addr, MCTi2cLocation, val1);
    res = readMCTRegister32(MCTi2cLocation, mct_addr, &result);
    bprintf("[result %d] MCT register %x (i2c location %x), contents read as %08x", res, mct_addr, MCTi2cLocation, result);
}
#endif

static void i2cHardTestRead(unsigned long mct_addr)
{
    uint32_t result;
    bool res;
#ifndef PICO_TRACE
    UNUSED(res);
#endif
    res = readMCTRegister32(MCTi2cLocation, mct_addr, &result);
    bprintf("[%s] %x: %08x", res ? "success" : "fail   ", mct_addr, result);
}

static void i2cMuxReset(bool enable)
{
    bprintf("reset mux (%s)", enable ? "enable" : "disable");
    gpio_put(muxResetPin, 0);
    delayMicroseconds(1); // Required to hold low for at least 9ns, so 1us is plenty
    if (enable)
        gpio_put(muxResetPin, 1);
}

static bool mctWriteReg(uint32_t mctAddress, uint32_t data)
{
    bool result = writeMCTRegister32(MCTi2cLocation, mctAddress, data);
    uint32_t buf;
    result = result && readMCTRegister32(MCTi2cLocation, mctAddress, &buf) && buf == data;
    return result;
}
static bool mctSetRegs(int device)
{
    bprintf("[hw] Setting registers for device %d", device);
    if (device<0 || device>3) {
        bprintf("device out of range (0..3)");
        return false;
    }

    i2cMuxEnableDevice(device);
    bool result = true;
    int numRegs = 0; int succeeded = 0;
#ifdef MCT8329A_ISD_CONFIG
    result = result && mctWriteReg(0x80, MCT8329A_ISD_CONFIG);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_MOTOR_STARTUP1
    result = result && mctWriteReg(0x82, MCT8329A_MOTOR_STARTUP1);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_MOTOR_STARTUP2
    result = result && mctWriteReg(0x84, MCT8329A_MOTOR_STARTUP2);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_CLOSED_LOOP1
    result = result && mctWriteReg(0x86, MCT8329A_CLOSED_LOOP1);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_CLOSED_LOOP2
    result = result && mctWriteReg(0x88, MCT8329A_CLOSED_LOOP2);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_CLOSED_LOOP3
    result = result && mctWriteReg(0x8A, MCT8329A_CLOSED_LOOP3);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_CLOSED_LOOP4
    result = result && mctWriteReg(0x8C, MCT8329A_CLOSED_LOOP4);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_CONST_SPEED
    result = result && mctWriteReg(0x8E, MCT8329A_CONST_SPEED);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_CONST_PWR
    result = result && mctWriteReg(0x90, MCT8329A_CONST_PWR);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_FAULT_CONFIG1
    result = result && mctWriteReg(0x92, MCT8329A_FAULT_CONFIG1);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_FAULT_CONFIG2
    result = result && mctWriteReg(0x94, MCT8329A_FAULT_CONFIG2);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_150_DEG_TWO_PH_PROFILE
    result = result && mctWriteReg(0x96, MCT8329A_150_DEG_TWO_PH_PROFILE);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_150_DEG_THREE_PH_PROFILE
    result = result && mctWriteReg(0x98, MCT8329A_150_DEG_THREE_PH_PROFILE);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_REF_PROFILES1
    result = result && mctWriteReg(0x9A, MCT8329A_REF_PROFILES1);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_REF_PROFILES2
    result = result && mctWriteReg(0x9C, MCT8329A_REF_PROFILES2);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_REF_PROFILES3
    result = result && mctWriteReg(0x9E, MCT8329A_REF_PROFILES3);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_REF_PROFILES4
    result = result && mctWriteReg(0xA0, MCT8329A_REF_PROFILES4);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_REF_PROFILES5
    result = result && mctWriteReg(0xA2, MCT8329A_REF_PROFILES5);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_REF_PROFILES6
    result = result && mctWriteReg(0xA4, MCT8329A_REF_PROFILES6);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_PIN_CONFIG1
    result = result && mctWriteReg(0xA6, MCT8329A_PIN_CONFIG1);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_PIN_CONFIG2
    result = result && mctWriteReg(0xA8, MCT8329A_PIN_CONFIG2);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_DEVICE_CONFIG
    result = result && mctWriteReg(0xAA, MCT8329A_DEVICE_CONFIG);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_GD_CONFIG1
    result = result && mctWriteReg(0xAC, MCT8329A_GD_CONFIG1);
    numRegs++; succeeded += result;
#endif
#ifdef MCT8329A_GD_CONFIG2
    result = result && mctWriteReg(0xAE, MCT8329A_GD_CONFIG2);
    numRegs++; succeeded += result;
#endif
    bprintf("device %d, set %d of %d regs", device, succeeded, numRegs);
    return result;
}

struct { uint8_t reg; const char *name; } mctLookup[] =
{
    {0x80, "ISD_CONFIG"},
    {0x82, "MOTOR_STARTUP1"},
    {0x84, "MOTOR_STARTUP2"},
    {0x86, "CLOSED_LOOP1"},
    {0x88, "CLOSED_LOOP2"},
    {0x8A, "CLOSED_LOOP3"},
    {0x8C, "CLOSED_LOOP4"},
    {0x8E, "CONST_SPEED"},
    {0x90, "CONST_PWR"},
    {0x92, "FAULT_CONFIG1"},
    {0x94, "FAULT_CONFIG2"},
    {0x96, "150_DEG_TWO_PH_PROFILE"},
    {0x98, "150_DEG_THREE_PH_PROFILE"},
    {0x9A, "REF_PROFILES1"},
    {0x9C, "REF_PROFILES2"},
    {0x9E, "REF_PROFILES3"},
    {0xA0, "REF_PROFILES4"},
    {0xA2, "REF_PROFILES5"},
    {0xA4, "REF_PROFILES6"},
    {0xA6, "PIN_CONFIG1"},
    {0xA8, "PIN_CONFIG2"},
    {0xAA, "DEVICE_CONFIG"},
    {0xAC, "GD_CONFIG1"},
    {0xAE, "GD_CONFIG2"},
};

const int numMCTregs = sizeof(mctLookup) / sizeof(mctLookup[0]);

bool mctReadRegByName(int device, const char *name, uint32_t *result)
{
    bool success = false;
    bool found = false;
    i2cMuxEnableDevice(device);
    for (int i=0; i<numMCTregs; ++i) {
        if (!strcasecmp(mctLookup[i].name, name)) {
            found = true;
            success = readMCTRegister32(MCTi2cLocation, mctLookup[i].reg, result);
            break;
        }
    }

    if (!found) {
        bprintf("mctReadRegByName unknown name '%s'", name);
    }

    return success;
}

bool mctWriteRegByName(int device, const char *name, uint32_t data)
{
    bool success = false;
    bool found = false;
    i2cMuxEnableDevice(device);
    for (int i=0; i<numMCTregs; ++i) {
        if (!strcasecmp(mctLookup[i].name, name)) {
            found = true;
            success = writeMCTRegister32(MCTi2cLocation, mctLookup[i].reg, data);
        }
    }

    if (!found) {
        bprintf("mctWriteRegByName unknown name '%s'", name);
    }

    return success;
}

void pico_esc_mct8329a_init(bool isDshotProtocol)
{
    // At end of motorDevInit, if successful
    UNUSED(isDshotProtocol); // dshot or pwm type
    bprintf("pico_esc_mct8329a_init %d", isDshotProtocol);

    irq_set_enabled(MCT8329A_MUX_I2C_INDEX == 0 ? I2C0_IRQ : I2C1_IRQ, false);

    i2cMuxReset(true);
    for (int motorDevice = 0; motorDevice < 4; ++motorDevice) {
        mctSetRegs(motorDevice);
        for (int i=0; i<numTestRegs; ++i) {
            i2cHardTestRead(testReadRegs[i]);
        }
    }

    irq_set_enabled(MCT8329A_MUX_I2C_INDEX == 0 ? I2C0_IRQ : I2C1_IRQ, true);
}

#endif
