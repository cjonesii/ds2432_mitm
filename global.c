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
/** global definitions and methods (code)
 *  @file global.c
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2016-2017
 */
/* standard libraries */
#include <stdint.h> // standard integer types
#include <stdlib.h> // general utilities

/* STM32 (including CM3) libraries */
#include <libopencmsis/core_cm3.h> // Cortex M3 utilities
#include <libopencm3/cm3/nvic.h> // interrupt handler
#include <libopencm3/cm3/systick.h> // SysTick library
#include <libopencm3/stm32/rcc.h> // real-time control clock library
#include <libopencm3/stm32/gpio.h> // general purpose input output library
#include <libopencm3/stm32/timer.h> // timer library
#include <libopencm3/stm32/exti.h> // external interrupt defines

#include "global.h" // common methods
#include "string.h" // memory utilities

volatile bool button_flag = false;
volatile uint32_t sleep_duration = 0; /**< sleep duration count down (in SysTick interrupts) */

char* b2s(uint64_t binary, uint8_t rjust)
{
	static char string[64+1] = {0}; // the string representation to return
	uint8_t bit = LENGTH(string)-1; // the index of the bit to print
	string[bit--] = '\0'; // terminate string

	while (binary) {
		if (binary & 1) {
			string[bit--] = '1';
		} else {
			string[bit--] = '0';
		}
		binary >>= 1;
	}

	while (64-bit-1<rjust && bit>0) {
		string[bit--] = '0';
	}

	return string;
}

/** switch on board LED */
void led_on(void)
{
#if defined(SYSTEM_BOARD) || defined(BLUE_PILL) || defined(CORE_BOARD)
	gpio_clear(GPIO(LED_PORT), GPIO(LED_PIN));
#elif defined(MAPLE_MINI)
	gpio_set(GPIO(LED_PORT), GPIO(LED_PIN));
#endif
}
/** switch off board LED */
void led_off(void)
{
#if defined(SYSTEM_BOARD) || defined(BLUE_PILL) || defined(CORE_BOARD)
	gpio_set(GPIO(LED_PORT), GPIO(LED_PIN));
#elif defined(MAPLE_MINI)
	gpio_clear(GPIO(LED_PORT), GPIO(LED_PIN));
#endif
}
/** toggle board LED */
void led_toggle(void)
{
	gpio_toggle(GPIO(LED_PORT), GPIO(LED_PIN));
}

void sleep_us(uint32_t duration)
{
	systick_counter_disable(); // disable SysTick to reconfigure it
	systick_clear(); // reset SysTick
	systick_set_frequency(1000000,rcc_ahb_frequency); // set SysTick frequency to microseconds
	systick_interrupt_enable(); // enable interrupt to count duration
	sleep_duration = duration; // save sleep duration for count down
	systick_counter_enable(); // start counting
	while (sleep_duration) { // wait for count down to complete
		__WFI(); // go to sleep	
	}
}

void sleep_ms(uint32_t duration)
{
	systick_counter_disable(); // disable SysTick to reconfigure it
	systick_clear(); // reset SysTick
	systick_set_frequency(1000,rcc_ahb_frequency); // set SysTick frequency to milliseconds
	systick_interrupt_enable(); // enable interrupt to count duration
	sleep_duration = duration; // save sleep duration for count down
	systick_counter_enable(); // start counting
	while (sleep_duration) { // wait for count down to complete
		__WFI(); // go to sleep	
	}
}

/** SysTick interrupt handler */
void sys_tick_handler(void)
{
	if (sleep_duration) {
		sleep_duration--; // decrement duration
	}
	if (0==sleep_duration) { // sleep complete
		systick_counter_disable(); // stop systick
		systick_interrupt_disable(); // stop interrupting
	}
}

void board_setup(void)
{
	// setup LED
	rcc_periph_clock_enable(RCC_GPIO(LED_PORT)); // enable clock for LED
	gpio_set_mode(GPIO(LED_PORT), GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO(LED_PIN)); // set LED pin to 'output push-pull'
	led_off(); // switch off LED per default

	// setup button 
    /* Will remove button isr due to conflict with master pin */
#if defined(BUTTON_PORT) && defined(BUTTON_PIN)
	rcc_periph_clock_enable(RCC_GPIO(BUTTON_PORT)); // enable clock for button
	gpio_set_mode(GPIO(BUTTON_PORT), GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO(BUTTON_PIN)); // set button pin to input
	rcc_periph_clock_enable(RCC_AFIO); // enable alternate function clock for external interrupt
	exti_select_source(EXTI(BUTTON_PIN), GPIO(BUTTON_PORT)); // mask external interrupt of this pin only for this port
#if defined(MAPLE_MINI)
	gpio_clear(GPIO(BUTTON_PORT), GPIO(BUTTON_PIN)); // pull down to be able to detect button push (go high)
	exti_set_trigger(EXTI(BUTTON_PIN), EXTI_TRIGGER_RISING); // trigger when button is pressed
#elif defined(CORE_BOARD)
	gpio_set(GPIO(BUTTON_PORT), GPIO(BUTTON_PIN)); // pull up to be able to detect button push (go low)
	exti_set_trigger(EXTI(BUTTON_PIN), EXTI_TRIGGER_FALLING); // trigger when button is pressed
#endif
	exti_enable_request(EXTI(BUTTON_PIN)); // enable external interrupt
	nvic_enable_irq(NVIC_EXTI_IRQ(BUTTON_PIN)); // enable interrupt
#endif
}

#if defined(BUTTON_PIN)
/** interrupt service routine called when button is pressed */
void EXTI_ISR(BUTTON_PIN)(void)
{
	exti_reset_request(EXTI(BUTTON_PIN)); // reset interrupt
	button_flag = true; // perform button action
}
#endif
