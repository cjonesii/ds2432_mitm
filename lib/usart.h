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
/** library for USART communication (API)
 *  @file usart.h
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2016
 *  @note peripherals used: USART @ref usart
 */
#pragma once

/** transmit and receive buffer sizes */
#define USART_BUFFER 128
/** how many bytes available in the received buffer since last read */
extern volatile bool usart_received;

/** setup USART peripheral */
void usart_setup(void);
/** send character over USART (blocking)
 *  @param[in] c character to send
 *  @note blocks until character transmission started */
void usart_putchar_blocking(char c);
/** ensure all data has been transmitted (blocking)
 *  @note block until all data has been transmitted
 */
void usart_flush(void);
/** get character received over USART (blocking)
 *  @return character received over USART
 *  @note blocks until character is received over USART when received buffer is empty
 */
char usart_getchar(void);
/** send character over USART (non-blocking)
 *  @param[in] c character to send
 *  @note blocks if transmit buffer is full, else puts in buffer and returns
 */
void usart_putchar_nonblocking(char c);
