/* This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/** library for 1-wire protocol as slave (API)
 *  @file onewire_slave.h
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2017
 *  @note peripherals used: timer @ref onewire_slave_timer, GPIO @ref onewire_slave_gpio
 *  @note overdrive mode is not supported
 */
#pragma once

extern volatile uint8_t byte;

extern volatile bool rom_cmd_received;

extern volatile bool read_rom_byte;

extern volatile bool mem_cmd_received;

void mitm_setup(void);

