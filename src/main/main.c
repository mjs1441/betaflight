/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "fc/init.h"

#include "scheduler/scheduler.h"

void run(void);

#include <string.h>

int main(int argc, char * argv[])
{
#ifdef USE_MAIN_ARGS
    targetParseArgs(argc, argv);
#else
    UNUSED(argc);
    UNUSED(argv);
#endif
    init();

#if 0
    uint8_t buffer[10];
    memset(&buffer[1], 0, 4);
    memset(&buffer[2], 0, 4);
    bprintf("%d , buffer1 %d, buffer2 %d\n", buffer, &buffer[1], &buffer[2]);
#endif

#if 0
    bprintf("SCB CCR starts of as %08x", SCB->CCR);
//    SCB->CCR |= ( SCB_CCR_DIV_0_TRP_Msk | SCB_CCR_UNALIGN_TRP_Msk | SCB_CCR_BFHFNMIGN_Msk);
//    bprintf("SCB CCR now set to %08x", SCB->CCR);
    
    static char buf1[128];
    static char buf2[128];
    for (int i=0; i<100; ++i) {
        buf2[i] = i;
    }

    char * p = (&buf2[0] + 3);
    char * q = &buf1[0];
    p = p - ((int)p % 4);
    for (int i=0; i<4; ++i) {
        for (int j=0; j<4; ++j) {
            bprintf("look from %p to %p", p, q);
            memcpy(q, p, 11);
            q++;
        }
        p++;
    }

    bprintf("didit");
#endif
    
    run();

    return 0;
}

void FAST_CODE run(void)
{
    while (true) {
        scheduler();
#if defined(RUN_LOOP_DELAY_US) && RUN_LOOP_DELAY_US > 0
        delayMicroseconds_real(RUN_LOOP_DELAY_US);
#endif
    }
}
