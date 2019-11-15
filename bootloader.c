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
/** USB DFU bootloader
 *  @file bootloader.c
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2017
 */
/* standard libraries */
#include <stdint.h> // standard integer types
#include <stdbool.h> // boolean types

/* STM32 (including CM3) libraries */
#include <libopencm3/cm3/scb.h> // vector table definition
#include <libopencm3/stm32/rcc.h> // clock utilities
#include <libopencm3/stm32/gpio.h> // GPIO utilities

/* own libraries */
#include "global.h" // board definitions
#include "usb_dfu.h" // USB DFU utilities

/** bootloader entry point */
void main(void);
void main(void)
{
	// check of DFU mode is forced
	bool dfu_force = false; // to remember if DFU mode is forced
	// check if a soft boot has been used
	if (0==(RCC_CSR&0xfc000000)) { // no reset flag present -> this was a soft reset using csr_reset_core(), very probably to start the DFU mode
		dfu_force = true;
	} else { // check if the force DFU mode input is set
		// disable SWJ pin to use as GPIO
#if (GPIO(B)==GPIO(DFU_FORCE_PORT)) && (GPIO(4)==GPIO(DFU_FORCE_PIN))
		gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST, 0);
#elif ((GPIO(B)==GPIO(DFU_FORCE_PORT)) && (GPIO(3)==GPIO(DFU_FORCE_PIN))) || ((GPIO(A)==GPIO(DFU_FORCE_PORT)) && (GPIO(15)==GPIO(DFU_FORCE_PIN)))
		gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);
#elif ((GPIO(A)==GPIO(DFU_FORCE_PORT)) && (GPIO(14)==GPIO(DFU_FORCE_PIN))) || ((GPIO(A)==GPIO(DFU_FORCE_PORT)) && (GPIO(13)==GPIO(DFU_FORCE_PIN)))
		gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, 0);
#endif
		rcc_periph_clock_enable(RCC_GPIO(DFU_FORCE_PORT)); // enable clock for GPIO domain
		gpio_set_mode(GPIO(DFU_FORCE_PORT), GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO(DFU_FORCE_PIN)); // set GPIO to input
		// pull on the opposite of the expected value
		if (DFU_FORCE_VALUE) {
			gpio_clear(GPIO(DFU_FORCE_PORT), GPIO(DFU_FORCE_PIN)); // pull down to be able to detect when tied to high
		} else {
			gpio_set(GPIO(DFU_FORCE_PORT), GPIO(DFU_FORCE_PIN)); // pull up to be able to detect when tied to low
		}
		if ((!DFU_FORCE_VALUE && 0==gpio_get(GPIO(DFU_FORCE_PORT), GPIO(DFU_FORCE_PIN))) || (DFU_FORCE_VALUE && 0!=gpio_get(GPIO(DFU_FORCE_PORT), GPIO(DFU_FORCE_PIN)))) { // check if output is set to the value to force DFU mode
			dfu_force = true; // DFU mode forced
		}
	}

	// start application if valid
	/* the application starts with the vector table
	 * the first entry in the vector table is the initial stack pointer (SP) address
	 * the stack will be placed in RAM
	 * on STM32F1xx SRAM begins at 0x2000 0000, and on STM32F103x8 there is 20KB of RAM (0x5000).
	 * since the stack grown "downwards" it should start at the end of the RAM: 0x2000 5000
	 * if the SP is not in this range (e.g. flash has been erased) there is no valid application
	 * the second entry in the vector table is the reset address, corresponding to the application start
	 */
	volatile uint32_t* application = &__application_beginning; // get the value of the application address symbol (use a register instead on the stack since the stack pointer will be changed)
	if (!dfu_force && (((*application)&0xFFFE0000)==0x20000000)) { // application at address seems valid
		SCB_VTOR = (volatile uint32_t)(application); // set vector table to application vector table (store at the beginning of the application)
		__asm__ volatile ("MSR msp,%0" : :"r"(*application)); // set stack pointer to address provided in the beginning of the application (loaded into a register first)
		(*(void(**)(void))(application + 1))(); // start application (by jumping to the reset function which address is stored as second entry of the vector table)
	}

	rcc_clock_setup_in_hse_8mhz_out_72mhz(); // start main clock
	board_setup(); // setup board to control LED
	led_on(); // indicate bootloader started
	usb_dfu_setup(); // setup USB DFU for firmware upload
	usb_dfu_start(); // run DFU mode
}
