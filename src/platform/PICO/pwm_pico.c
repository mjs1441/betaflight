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
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_PWM_OUTPUT

#include "build/debug.h"
#include "build/debug_pin.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_impl.h"
#include "drivers/motor_types.h"

#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#ifdef PICO_ESC_MCT8329A
#include "pico_mct8329a.h"
#endif

typedef struct picoPwmMotors_s {
    uint16_t slice;
    uint16_t channel;
} picoPwmMotors_t;

static picoPwmMotors_t picoPwmMotors[MAX_SUPPORTED_MOTORS];
static bool useContinuousUpdate = false;
static uint16_t wrap;
static uint32_t clock_divide;

void pwmShutdownPulsesForAllMotors(void)
{
    for (int index = 0; index < pwmMotorCount; index++) {
        pwm_set_chan_level(picoPwmMotors[index].slice, picoPwmMotors[index].channel, 0);
    }
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors();
}

static void pwmWriteStandard(uint8_t index, float value)
{
    uint16_t dutyLevel = lrintf(MAX(0, (value + pwmMotors[index].pulseOffset) * pwmMotors[index].pulseScale));
//    bprintf("pwmWriteStandard %d val %.1f on pwm %d,%d -> %d of %d so %.1f", index, (double)value, picoPwmMotors[index].slice, picoPwmMotors[index].channel,
//            dutyLevel, wrap, ((double)dutyLevel)/wrap );
    pwm_set_chan_level(picoPwmMotors[index].slice, picoPwmMotors[index].channel, dutyLevel);
}

static void pwmCompleteMotorUpdate(void)
{
    if (useContinuousUpdate) {
        return;
    }

    // TODO (maybe) support other than MOTOR_PROTOCOL_PWM (which has useContinuousUpdate true)
    for (int index = 0; index < pwmMotorCount; index++) {
        pwm_set_chan_level(picoPwmMotors[index].slice, picoPwmMotors[index].channel, 0);
    }
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static void pwmMotorPostInit(void)
{
#ifdef PICO_ESC_MCT8329A
    pico_esc_mct8329a_init(false);
#endif
}

static motorVTable_t motorPwmVTable = {
    .postInit = pwmMotorPostInit,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .convertExternalToMotor = pwmConvertFromExternal,
    .convertMotorToExternal = pwmConvertToExternal,
    .write = pwmWriteStandard,
    .decodeTelemetry = NULL,
    .updateComplete = pwmCompleteMotorUpdate,
    .requestTelemetry = NULL,
    .isMotorIdle = NULL,
};

bool motorPwmDevInit_PWM(unsigned motorIndex, const motorDevConfig_t *motorConfig, pwm_config *pwmConfig)
{
#ifdef MOTOR_FC_PWM_RATE
    // Override pwm rate for FC->ESC communication, e.g. to 16kHz for some ESCs -> divide = 1, wrap is 0x23c3 at clock = 150MHz
    const unsigned pwmRateHz = MOTOR_FC_PWM_RATE;
    UNUSED(motorConfig);
#else
    const unsigned pwmRateHz = motorConfig->motorPwmRate;
#endif

    // f = sysclk / div / wrap
    // Max clock divide is just under 256.
    if (0 == wrap) {
        clock_divide =  (SystemCoreClock / pwmRateHz + 0xfffe) / 0xffff; // round up
        if (clock_divide > 255) {
            clock_divide  = 255;
            wrap = 0xffff;
        } else {
            wrap = SystemCoreClock / pwmRateHz / clock_divide;
        }

        bprintf("req freq of pwm period %d -> divide = %d, wrap = 0x%x, incr freq = %.1f",
                pwmRateHz, clock_divide, wrap, ((double)SystemCoreClock)/clock_divide);
    }

    pwm_config_set_clkdiv_int(pwmConfig, clock_divide);
    pwm_config_set_wrap(pwmConfig, wrap);

    // As per comment in mixer_init.c, all PWM motor scaling is done to standard PWM range of 1000-2000,
    // so lowest possible is 1000 (offset to be subtracted), and max range is also 1000.
    pwmMotors[motorIndex].pulseScale = wrap / 1000.0f;
    pwmMotors[motorIndex].pulseOffset = - 1000.0f;

    return true;
}

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t idlePulse)
{
    UNUSED(idlePulse);

    if (!device) {
        return false;
    }

    pwmMotorCount = device->count;

    memset(pwmMotors, 0, sizeof(pwmMotors));

    if (!device || !motorConfig) {
        return false;
    }

    pwmMotorCount = device->count;
    device->vTable = &motorPwmVTable;

    useContinuousUpdate = motorConfig->useContinuousUpdate;

    float sMin = 0;
    float sLen = 0;
    switch (motorConfig->motorProtocol) {
    default:
    case MOTOR_PROTOCOL_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case MOTOR_PROTOCOL_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case MOTOR_PROTOCOL_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    case MOTOR_PROTOCOL_BRUSHED:
        sMin = 0;
        useContinuousUpdate = true;
        break;
    case MOTOR_PROTOCOL_PWM:
        useContinuousUpdate = true;
        break;
    }

    for (unsigned motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < pwmMotorCount; motorIndex++) {

        const unsigned reorderedMotorIndex = motorConfig->motorOutputReordering[motorIndex];
        const ioTag_t tag = motorConfig->ioTags[reorderedMotorIndex];

        pwmMotors[motorIndex].io = IOGetByTag(tag);
        uint8_t pin = IO_PINBYTAG(tag);

        const uint16_t slice = pwm_gpio_to_slice_num(pin);
        const uint16_t channel = pwm_gpio_to_channel(pin);

        IOInit(pwmMotors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));

        picoPwmMotors[motorIndex].slice = slice;
        picoPwmMotors[motorIndex].channel = channel;

        pwm_config config = pwm_get_default_config();
        
        if (motorConfig->motorProtocol == MOTOR_PROTOCOL_PWM) {
            motorPwmDevInit_PWM(motorIndex, motorConfig, &config);
        } else {
            // TODO pwm protocols other than MOTOR_PROTOCOL_PWM not yet tested / supported.

            /* standard PWM outputs */
            // margin of safety is 4 periods when not continuous
            const unsigned pwmRateHz = useContinuousUpdate ? motorConfig->motorPwmRate : ceilf(1 / ((sMin + sLen) * 4));

            /*
              PWM Frequency = clock / (interval * (wrap + 1))
              
              Wrap is when the counter resets to zero.
              Interval (divider) will determine the resolution.
            */
            const uint32_t clock = clock_get_hz(clk_sys); // PICO timer clock for PWM is the CPU clock.
            
            /* used to find the desired timer frequency for max resolution */
            const unsigned prescaler = ceilf(clock / pwmRateHz); /* rounding up */
            const uint32_t hz = clock / prescaler;
            const unsigned period = useContinuousUpdate ? hz / pwmRateHz : 0xffff;

            pwmMotors[motorIndex].pulseScale = ((motorConfig->motorProtocol == MOTOR_PROTOCOL_BRUSHED) ? period : (sLen * hz)) / 1000.0f;
            pwmMotors[motorIndex].pulseOffset = (sMin * hz) - (pwmMotors[motorIndex].pulseScale * 1000);

            const uint8_t interval = (uint8_t)(clock / period);
            const uint8_t fraction = (uint8_t)(((clock / period) - interval) * (0x01 << 4));
            pwm_config_set_clkdiv_int_frac(&config, interval, fraction);
            pwm_config_set_wrap(&config, period);
        }

        gpio_set_function(pin, GPIO_FUNC_PWM);
        pwm_set_chan_level(slice, channel, 0);
        pwm_init(slice, &config, true);

        /*
            if brushed then it is the entire length of the period.
            TODO: this can be moved back to periodMin and periodLen
            once mixer outputs a 0..1 float value.
        */
        pwmMotors[motorIndex].enabled = true;
    }

    return true;
}

#endif // USE_PWM_OUTPUT
