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
/** STM32F1 Maxim DS2432 (1k-Bit Protected 1-Wire EEPROM with SHA-1 Engine) implementation
 *  @file application.c
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2016-2017
 */

/* standard libraries */
#include <stdint.h> // standard integer types
#include <stdlib.h> // standard utilities
#include <string.h> // string utilities
#include <time.h> // date/time utilities

/* STM32 (including CM3) libraries */
#include <libopencmsis/core_cm3.h> // Cortex M3 utilities
#include <libopencm3/cm3/scb.h> // vector table definition
#include <libopencm3/cm3/nvic.h> // interrupt utilities
#include <libopencm3/stm32/gpio.h> // general purpose input output library
#include <libopencm3/stm32/rcc.h> // real-time control clock library
#include <libopencm3/stm32/exti.h> // external interrupt utilities
#include <libopencm3/stm32/rtc.h> // real time clock utilities
#include <libopencm3/stm32/iwdg.h> // independent watchdog utilities
#include <libopencm3/stm32/dbgmcu.h> // debug utilities
#include <libopencm3/stm32/flash.h> // flash utilities

/* own libraries */
#include "global.h" // board definitions
#include "print.h" // printing utilities
#include "usart.h" // USART utilities
#include "usb_cdcacm.h" // USB CDC ACM utilities
#include "mitm.h"

#define WATCHDOG_PERIOD 10000 /**< watchdog period in ms */

volatile bool rtc_internal_tick_flag = false;

time_t time_rtc = 0;
struct tm* time_tm;

size_t putc(char c)
{
	size_t length = 0;
	static char newline = 0;
	if (0==c) {
		length = 0;
	} else if ('\r' == c || '\n' == c) {
		if (0==newline || c==newline) {
			usart_putchar_nonblocking('\r'); // send CR over USART
			usb_cdcacm_putchar('\r'); // send CR over USB
			usart_putchar_nonblocking('\n'); // send LF over USART
			usb_cdcacm_putchar('\n'); // send LF over USB
			length += 2; // remember we printed 2 characters
			newline = c; // remember on which character we sent the newline
		} else {
			length = 0; // the \r or \n of \n\r or \r\n has already been printed
		}
	} else {
		usart_putchar_nonblocking(c); // send byte over USART
		usb_cdcacm_putchar(c); // send byte over USB
		newline = 0; // clear new line
		length++; // remember we printed 1 character
	}
	return length; // return number of characters printed   
}

/** user input command */
static char command[32] = {0};
/** user input command index */
uint8_t command_i = 0;

/** process user command
 *  @param[in] str user command string (\0 ended)
 */
static void process_command(char* str)
{
	// split command
	const char* delimiter = " ";
	char* word = strtok(str,delimiter);
	if (!word) {
		goto error;
	}
	// parse command
	if (0==strcmp(word,"h") || 0==strcmp(word,"help") || 0==strcmp(word,"?")) {
		printf("available commands:\n");
		printf("led [on|off|toggle]\n");
	} else if (0==strcmp(word,"l") || 0==strcmp(word,"led")) {
		word = strtok(NULL,delimiter);
		if (!word) {	
			printf("LED is ");
			if (gpio_get(GPIO(LED_PORT), GPIO(LED_PIN))) {
				printf("on\n");
			} else {
				printf("off\n");
			}
		} else if (0==strcmp(word,"on")) {
			led_on(); // switch LED on
			printf("LED switched on\n"); // notify user		
		} else if (0==strcmp(word,"off")) {
			led_off(); // switch LED off
			printf("LED switched off\n"); // notify user
		} else if (0==strcmp(word,"toggle")) {
			led_toggle(); // toggle LED
			printf("LED toggled\n"); // notify user
		} else {
			goto error;
		}
	} else if (0==strcmp(word,"time")) {
		word = strtok(NULL,delimiter);
		if (!word) {
			time_rtc = rtc_get_counter_val(); // get time from internal RTC
			time_tm = localtime(&time_rtc); // convert time
			printf("time: %02d:%02d:%02d\n", time_tm->tm_hour, time_tm->tm_min, time_tm->tm_sec);
		} else if (strlen(word)!=8 || word[0]<'0' || word[0]>'2' || word[1]<'0' || word[1]>'9' || word[3]<'0' || word[3]>'5' || word[4]<'0' || word[4]>'9' || word[6]<'0' || word[6]>'5' || word[7]<'0' || word[7]>'9') { // time format is incorrect
				goto error;
		} else {
			time_rtc = rtc_get_counter_val(); // get time from internal RTC
			time_tm = localtime(&time_rtc); // convert time
			time_tm->tm_hour = (word[0]-'0')*10+(word[1]-'0')*1; // set hours
			time_tm->tm_min = (word[3]-'0')*10+(word[4]-'0')*1; // set minutes
			time_tm->tm_sec = (word[6]-'0')*10+(word[7]-'0')*1; // set seconds
			time_rtc = mktime(time_tm); // get back seconds
			rtc_set_counter_val(time_rtc); // save time to internal RTC
			printf("time set\n");
		}
	} else if (0==strcmp(word,"date")) {
		word = strtok(NULL,delimiter);
		if (!word) {
			time_rtc = rtc_get_counter_val(); // get time from internal RTC
			time_tm = localtime(&time_rtc); // convert time
			printf("date: %d-%02d-%02d\n", 1900+time_tm->tm_year, time_tm->tm_mon+1, time_tm->tm_mday);
		} else if (strlen(word)!=10 || word[0]!='2' || word[1]!='0' || word[2]<'0' || word[2]>'9' || word[3]<'0' || word[3]>'9' || word[5]<'0' || word[5]>'1' || word[6]<'0' || word[6]>'9' || word[8]<'0' || word[8]>'3' || word[9]<'0' || word[9]>'9') {
				goto error;
		} else {
			time_rtc = rtc_get_counter_val(); // get time from internal RTC
			time_tm = localtime(&time_rtc); // convert time
			time_tm->tm_year = ((word[0]-'0')*1000+(word[1]-'0')*100+(word[2]-'0')*10+(word[3]-'0')*1)-1900; // set year
			time_tm->tm_mon = (word[5]-'0')*10+(word[6]-'0')*1-1; // set month
			time_tm->tm_mday = (word[8]-'0')*10+(word[9]-'0')*1; // set day
			time_rtc = mktime(time_tm); // get back seconds
			rtc_set_counter_val(time_rtc); // save time to internal RTC
			printf("date set\n");
		}
	} else {
		goto error;
	}

	return; // command successfully processed
error:
	printf("command not recognized. enter help to list commands\n");
	return;
}

/** program entry point
 *  this is the firmware function started by the micro-controller
 */
void main(void);
void main(void)
{	
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

#if DEBUG
	// enable functionalities for easier debug
	DBGMCU_CR |= DBGMCU_CR_IWDG_STOP; // stop independent watchdog counter when code is halted
	DBGMCU_CR |= DBGMCU_CR_WWDG_STOP; // stop window watchdog counter when code is halted
	DBGMCU_CR |= DBGMCU_CR_STANDBY; // allow debug also in standby mode (keep digital part and clock powered)
	DBGMCU_CR |= DBGMCU_CR_STOP; // allow debug also in stop mode (keep clock powered)
	DBGMCU_CR |= DBGMCU_CR_SLEEP; // allow debug also in sleep mode (keep clock powered)
#else
	// setup watchdog to reset in case we get stuck (i.e. when an error occurred)
	iwdg_set_period_ms(WATCHDOG_PERIOD); // set independent watchdog period
	iwdg_start(); // start independent watchdog
#endif

	board_setup(); // setup board
    usart_setup(); /* setup USART (for printing)*/
	usb_cdcacm_setup(); // setup USB CDC ACM (for printing)

#if !(DEBUG)
	// show watchdog information
	printf("watchdog set to (%.2fs)\n",WATCHDOG_PERIOD/1000.0);
	if (FLASH_OBR&FLASH_OBR_OPTERR) {
		printf("option bytes not set in flash: software watchdog used (not started at reset)\n");
	} else if (FLASH_OBR&FLASH_OBR_WDG_SW) {
		printf("software watchdog used (not started at reset)\n");
	} else {
		printf("hardware watchdog used (started at reset)\n");
	}
#endif

	// setup RTC
	printf("setup internal RTC: ");
	rtc_auto_awake(RCC_LSE, 32768-1); 
    /* ensure internal RTC is on, uses the 32.678 kHz LSE, and the prescale is 
     * set to our tick speed, else update backup registers accordingly (power 
     * off the micro-controller for the change to take effect)
     */
	rtc_interrupt_enable(RTC_SEC); // enable RTC interrupt on "seconds"
	nvic_enable_irq(NVIC_RTC_IRQ); // allow the RTC to interrupt
	printf("OK\n");

	time_rtc= rtc_get_counter_val(); // get time from internal RTC
	time_tm = localtime(&time_rtc); // convert time
	printf("date: %d-%02d-%02d %02d:%02d:%02d\n", 1900+time_tm->tm_year, time_tm->tm_mon+1, time_tm->tm_mday, time_tm->tm_hour, time_tm->tm_min, time_tm->tm_sec);

	printf("Setup 1-Wire bus");
    mitm_setup();
	printf("OK\n");

	// main loop
	printf("command input: ready\n");
	bool action = false; // if an action has been performed don't go to sleep
	button_flag = false; // reset button flag
	char ch = '\0'; // to store received character
	bool char_flag = false; // a new character has been received

	while (true) { // infinite loop
		iwdg_reset(); // kick the dog

		while (usart_received) { 
			action = true; 
			led_toggle(); 
			ch = usart_getchar(); 
			char_flag = true; 
		}

		while (usb_cdcacm_received) { // data received over USB
			action = true; // action has been performed
			led_toggle(); // toggle LED
			ch = usb_cdcacm_getchar(); // store receive character
			char_flag = true; // notify character has been received
		}

		while (char_flag) { // user data received
			char_flag = false; // reset flag
			action = true; // action has been performed
			printf("%c",ch); // echo receive character
			if (ch=='\r' || ch=='\n') { // end of command received
				if (command_i>0) { // there is a command to process
					command[command_i] = 0; // end string
					command_i = 0; // prepare for next command
					process_command(command); // process user command
				}
			} else { // user command input
				command[command_i] = ch; // save command input
				if (command_i<LENGTH(command)-2) { // verify if there is place to save next character
					command_i++; // save next character
				}
			}
		}

		while (button_flag) { // user pressed button
			action = true; // action has been performed
			led_toggle(); // toggle LED
			printf("button pressed\n");
			button_flag = false; // reset flag
		}

		while (rtc_internal_tick_flag) { // the internal RTC ticked
			rtc_internal_tick_flag = false; // reset flag
			action = true; // action has been performed
#if !defined(BLUE_PILL) // on the blue pill the LED is close to the 32.768 kHz oscillator and heavily influences it
			//led_toggle(); // toggle LED (good to indicate if main function is stuck)
#endif
			time_rtc = rtc_get_counter_val(); // get time from internal RTC (seconds since Unix Epoch)
			time_tm = localtime(&time_rtc); // get time in tm format from Epoch (time zones are not handled for non-POSIX environments)
			if (0==time_tm->tm_sec) { // new minute
				printf("time: %02d:%02d:%02d\n", time_tm->tm_hour, time_tm->tm_min, time_tm->tm_sec);
			}
		}

        while (rom_cmd_received) {
                                switch(byte) {
                            case 0x33: /* Read ROM */
                                f120_state = F120_ROM_CMD_DONE;
                                break;
                            case 0x55: /* Match ROM */
                                f120_state = F120_ROMCMD_MATCHROM;
                                break;
                            case 0xf0: /* Search ROM */
                                f120_state = F120_ROM_SEARCH;
                                break;
                            case 0xcc: /* Skip ROM */
                                f120_state = F120_MEM_CMD;
                                break;
                            case 0xa5: /* Resume Command */
                                f120_state = F120_ROM_RESUME_CMD;
                                break;
                            case 0x3c: /* OD Skip ROM */
                                f120_state = F120_ROM_OD_SKIP;
                                break;
                            case 0x69: /* OD Match ROM */
                                f120_state = F120_ROM_OD_MATCH;
                                break;
                            default:
                                break;
                        }
}
		if (action) { // go to sleep if nothing had to be done, else recheck for activity
			action = false;
		} else {
			__WFI(); // go to sleep
		}
	} // main loop
}

/** @brief interrupt service routine called when tick passed on RTC */
void rtc_isr(void)
{
	rtc_clear_flag(RTC_SEC); // clear flag
	rtc_internal_tick_flag = true; // notify to show new time
}
