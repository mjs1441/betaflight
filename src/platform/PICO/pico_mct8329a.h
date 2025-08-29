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

typedef struct mctLookup_s { uint8_t reg; const char *name; } mctLookup_t;
extern mctLookup_t mctLookup[];
extern const int numMCTregs;
extern const int numMCTdevices;

void pico_esc_mct8329a_init(bool isDshotProtocol);
bool mctReadRegByName(int device, const char *name, uint32_t *result);
bool mctWriteRegByName(int device, const char *name, uint32_t data);
bool mctWriteRegByAddress(int device, uint32_t reg, uint32_t data);
