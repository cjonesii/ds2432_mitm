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
/** library for USART communication (code)
 *  @file usart.c
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2016
 *  @note peripherals used: USART @ref usart
 */

/* standard libraries */
#include <stdint.h> // standard integer types
#include <stdio.h> // standard I/O facilities
#include <stdlib.h> // general utilities

/* STM32 (including CM3) libraries */
#include <libopencm3/stm32/rcc.h> // real-time control clock library
#include <libopencm3/stm32/gpio.h> // general purpose input output library
#include <libopencm3/stm32/usart.h> // universal synchronous asynchronous receiver transmitter library
#include <libopencm3/cm3/nvic.h> // interrupt handler
#include <libopencmsis/core_cm3.h> // Cortex M3 utilities

#include "usart.h" // USART header and definitions
#include "global.h" // common methods

/** @defgroup usart USART peripheral used for UART communication
 *  @{
 */
#define USART_ID 1 /**< USART peripheral */
/** @} */

#define USART_BAUDRATE 1500000 /**< serial baudrate, in bits per second (with 8N1 8 bits, no parity bit, 1 stop bit settings) */

/* input and output ring buffer, indexes, and available memory */
static uint8_t rx_buffer[USART_BUFFER] = {0}; /**< ring buffer for received data */
static volatile uint8_t rx_i = 0; /**< current position of read received data */
static volatile uint8_t rx_used = 0; /**< how much data has been received and not red */
static uint8_t tx_buffer[USART_BUFFER] = {0}; /**< ring buffer for data to transmit */
static volatile uint8_t tx_i = 0; /**< current position of transmitted data */
static volatile uint8_t tx_used = 0; /**< how much data needs to be transmitted */

volatile bool usart_received = false;

void usart_setup(void)
{
	/* enable USART I/O peripheral */
	rcc_periph_clock_enable(USART_PORT_RCC(USART_ID)); // enable clock for USART port peripheral
	rcc_periph_clock_enable(USART_RCC(USART_ID)); // enable clock for USART peripheral
	rcc_periph_clock_enable(RCC_AFIO); // enable pin alternate function (USART)
	gpio_set_mode(USART_PORT(USART_ID), GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USART_PIN_TX(USART_ID)); // setup GPIO pin USART transmit
	gpio_set_mode(USART_PORT(USART_ID), GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, USART_PIN_RX(USART_ID)); // setup GPIO pin USART receive
	gpio_set(USART_PORT(USART_ID), USART_PIN_RX(USART_ID)); // pull up to avoid noise when not connected

	/* setup USART parameters */
	usart_set_baudrate(USART(USART_ID), USART_BAUDRATE);
	usart_set_databits(USART(USART_ID), 8);
	usart_set_stopbits(USART(USART_ID), USART_STOPBITS_1);
	usart_set_mode(USART(USART_ID), USART_MODE_TX_RX);
	usart_set_parity(USART(USART_ID), USART_PARITY_NONE);
	usart_set_flow_control(USART(USART_ID), USART_FLOWCONTROL_NONE);

	nvic_enable_irq(USART_IRQ(USART_ID)); // enable the USART interrupt
	usart_enable_rx_interrupt(USART(USART_ID)); // enable receive interrupt
	usart_enable(USART(USART_ID)); // enable USART

	/* reset buffer states */
	tx_i = 0;
	tx_used = 0;
	rx_i = 0;
	rx_used = 0;
	usart_received = false;
}

void usart_putchar_blocking(char c)
{
	usart_flush(); // empty buffer first
	usart_send_blocking(USART(USART_ID), c); // send character
}

void usart_flush(void)
{
	while (tx_used) { // idle until buffer is empty
		__WFI(); // sleep until interrupt
	}
	usart_wait_send_ready(USART(USART_ID)); // wait until transmit register is empty (transmission might not be complete)
}

char usart_getchar(void)
{
	while (!rx_used) { // idle until data is available
		__WFI(); // sleep until interrupt
	}
	char to_return = rx_buffer[rx_i]; // get the next available character
	usart_disable_rx_interrupt(USART(USART_ID)); // disable receive interrupt to prevent index corruption
	rx_i = (rx_i+1)%LENGTH(rx_buffer); // update used buffer
	rx_used--; // update used buffer
	usart_received = (rx_used!=0); // update available data
	usart_enable_rx_interrupt(USART(USART_ID)); // enable receive interrupt
	return to_return;
}

void usart_putchar_nonblocking(char c)
{
	while (tx_used>=LENGTH(tx_buffer)) { // idle until buffer has some space
		usart_enable_tx_interrupt(USART(USART_ID)); // enable transmit interrupt
		__WFI(); // sleep until something happened
	}
	usart_disable_tx_interrupt(USART(USART_ID)); // disable transmit interrupt to prevent index corruption
	tx_buffer[(tx_i+tx_used)%LENGTH(tx_buffer)] = c; // put character in buffer
	tx_used++; // update used buffer
	usart_enable_tx_interrupt(USART(USART_ID)); // enable transmit interrupt
}

/** USART interrupt service routine called when data has been transmitted or received */
void USART_ISR(USART_ID)(void)
{
	if (usart_get_flag(USART(USART_ID), USART_SR_TXE)) { // data has been transmitted
		if (!tx_used) { // no data in the buffer to transmit
			usart_disable_tx_interrupt(USART(USART_ID)); // disable transmit interrupt
		} else {
			usart_send(USART(USART_ID),tx_buffer[tx_i]); // put data in transmit register
			tx_i = (tx_i+1)%LENGTH(rx_buffer); // update location on buffer
			tx_used--; // update used size
		}
	}
	if (usart_get_flag(USART(USART_ID), USART_SR_RXNE)) { // data has been received
		// only save data if there is space in the buffer
		while (rx_used>=LENGTH(rx_buffer)) { // if buffer is full
			rx_i = (rx_i+1)%LENGTH(rx_buffer); // drop oldest data
			rx_used--; // update used buffer information
		}
		rx_buffer[(rx_i+rx_used)%LENGTH(rx_buffer)] = usart_recv(USART(USART_ID)); // put character in buffer
		rx_used++; // update used buffer
		usart_received = (rx_used!=0); // update available data
	}
}
