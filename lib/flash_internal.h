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
/** library to read/write internal flash (API)
 *  @file flash_internal.h
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2016-2017
 *  @note peripherals used: none
 */
#pragma once

/** read data from internal flash
 *  @param[in] address start address of the data to read
 *  @param[out] buffer where to store the read data
 *  @param[in] size how much data to read, in bytes
 *  @return if read succeeded
 */
bool flash_internal_read(uint32_t address, uint8_t *buffer, size_t size);
/** write data to internal flash
 *  @param[in] address start address where to write data to
 *  @param[in] buffer data to be written
 *  @param[in] size how much data to write, in bytes
 *  @return if write succeeded
 */
bool flash_internal_write(uint32_t address, uint8_t *buffer, size_t size);
