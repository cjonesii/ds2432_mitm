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
/** library for USB CDC ACM communication (API)
 *  @file usb_cdcacm.h
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2016
 */
#pragma once

/** transmit and receive buffer sizes */
#define CDCACM_BUFFER 64
/** how many bytes available in the received buffer since last read */
extern volatile uint8_t usb_cdcacm_received;

/** setup USB CDC ACM peripheral */
void usb_cdcacm_setup(void);
/** get character received over USB (blocking)
 *  @return character received over USB
 *  @note blocks until character is received over USB when received buffer is empty
 */
char usb_cdcacm_getchar(void);
/** send character over USB (non-blocking)
 *  @param[in] c character to send
 *  @note blocks if transmit buffer is full, else puts in buffer and returns
 */ 
void usb_cdcacm_putchar(char c);
