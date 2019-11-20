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
/** library for 1-wire protocol as master (code)
 *  @file onewire_slave.c
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2017
 *  @note peripherals used: GPIO and timer @ref onewire_slave_timer, GPIO @ref onewire_slave_gpio
 *  @note overdrive mode is not supported
 *  @implements 1-Wire protocol description from Book of iButton Standards
 */

/** Carlos Jones <cjonesii@yahoo.com> 
 *  converted to mitm code
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <libopencmsis/core_cm3.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>

#include <global.h>
#include <mitm.h>

#define F120_TIMER   2
#define F120_PORT    A
#define F120_PIN     4

#define DS2432_TIMER 3
#define DS2432_PORT  B
#define DS2432_PIN   10

static volatile enum {
	F120_IDLE,
	F120_RESET,
	F120_WAIT_PRESENCE,
	F120_DS2432_PRESENT,
	F120_ROM_CMD,
	F120_MEM_CMD,
	F120_ROMCMD_READROM,
	F120_ROMCMD_MATCHROM,
	F120_ROM_SEARCH,
	F120_ROM_RESUME_CMD,
	F120_ROM_OD_SKIP,
	F120_ROM_OD_MATCH,
    F120_MAX
} f120_state = F120_IDLE;

static volatile enum {
    DS2432_IDLE,
	DS2432_WAIT_PRESENCE,
	DS2432_DS2432_PRESENT,
	DS2432_ROM_CMD,
	DS2432_ROMCMD_READROM,
	DS2432_ROMCMD_MATCHROM,
    DS2432_MAX
} ds2432_state = DS2432_IDLE;

/* static bool bypass_mode = true;
 */ 
static bool f120_rc_flag = false;
/*static uint8_t bit = 0;*/
static uint8_t bit_ctr = 0;
static uint8_t byte = 0;
static uint8_t byte_ctr = 0;
/*static uint8_t rom_cmd[8] = {0};*/
static bool rx_mode = false;

void mitm_setup(void) {
    /* F120 Master */
	rcc_periph_clock_enable(RCC_TIM(F120_TIMER));
	timer_reset(TIM(F120_TIMER));
	timer_set_mode(TIM(F120_TIMER), TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM(F120_TIMER), 1-1);
	timer_set_period(TIM(F120_TIMER), 33259);
	timer_set_oc_mode(TIM(F120_TIMER), TIM_OC1, TIM_OCM_FROZEN);
	timer_set_oc_value(TIM(F120_TIMER), TIM_OC1, 1151);
	timer_set_oc_mode(TIM(F120_TIMER), TIM_OC2, TIM_OCM_FROZEN);
	timer_set_oc_value(TIM(F120_TIMER), TIM_OC2, 2289);
	timer_set_oc_mode(TIM(F120_TIMER), TIM_OC1, TIM_OCM_FROZEN);
	timer_set_oc_value(TIM(F120_TIMER), TIM_OC1, 6497);
	timer_clear_flag(TIM(F120_TIMER), TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);
	timer_update_on_overflow(TIM(F120_TIMER));
	timer_enable_irq(TIM(F120_TIMER), TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM_IRQ(F120_TIMER));

	rcc_periph_clock_enable(RCC_GPIO(F120_PORT));
	gpio_set(GPIO(F120_PORT), GPIO(F120_PIN));
	gpio_set_mode(GPIO(F120_PORT), GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO(F120_PIN));

	rcc_periph_clock_enable(RCC_AFIO);
	exti_select_source(EXTI(F120_PIN), GPIO(F120_PORT));
	exti_set_trigger(EXTI(F120_PIN), EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI(F120_PIN));
	nvic_enable_irq(NVIC_EXTI_IRQ(F120_PIN));

    gpio_set(GPIO(DS2432_PORT),GPIO(DS2432_PIN));
    gpio_set_mode(GPIO(DS2432_PORT), GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO(DS2432_PIN));

	exti_select_source(EXTI(DS2432_PIN), GPIO(DS2432_PORT));
	exti_set_trigger(EXTI(DS2432_PIN), EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI(DS2432_PIN));
	nvic_enable_irq(NVIC_EXTI_IRQ(DS2432_PIN));
    f120_state = F120_IDLE;
    ds2432_state = DS2432_IDLE
}

void EXTI_ISR(F120_PIN)(void) {
    exti_reset_request(EXTI(F120_PIN));

    if (gpio_get(GPIO(F120_PORT), GPIO(F120_PIN))) { /* F120 GPIO Pulled High */
        switch (f120_state) {
            case F120_RESET:
                /* This must be triggered by F120 (HW triggered) */
                ds2432_state = DS2432_WAIT_PRESENCE:
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_DS2432_PRESENT:
                f120_state = F120_ROM_CMD;
                break;
            case F120_ROM_CMD:
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_ROMCMD_READROM:
                /* This must be triggered by F120 (HW triggered) */
                if (rx_mode == false) {
                    bit_ctr++;
                    byte_ctr = bit_ctr / 8;

                    if (bit_ctr == 64) {
                        f120_state = F120_MEM_CMD;
                        byte_ctr = 0;
                    }
                }
                break;
            case F120_ROMCMD_MATCHROM:
                /* Master TX bits to match ROM */
                /* Relay signal to DS2432 */
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                bit_ctr++;
                if (bit_ctr == 64) {
                    f120_rc_flag = true;
                    f120_state = F120_MEM_CMD;
                }
                break;
            default:
				break;
        }
    } else { /* low */
		timer_disable_counter(TIM(F120_TIMER)); // stop timer for reconfiguration
		timer_set_counter(TIM(F120_TIMER), 0); // reset timer counter
		timer_disable_irq(TIM(F120_TIMER), TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE); // disable all timers
        switch (f120_state) {
            case F120_IDLE:
                gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_WAIT_PRESENCE:
                ds2432_state = DS2432_DS2432_PRESENT;
                break;
            case F120_ROM_CMD:
                timer_enable_irq(TIM(F120_TIMER), TIM_DIER_CC2IE);
                ds2432_state = DS2432_ROM_CMD;
                gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_ROMCMD_READROM:
                /* F120 will always initiate the Read Time Slot */
                /* rx_mode must be false if init by F120 (HW triggered) */
                if (rx_mode == false) {
                    rx_mode = true;
                    gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                }
                /* Do nothing if init by SW */
                /* DS2432 TX:
                 *   Chip Family Code 1byte
                 *   Serial Number    6bytes
                 *   CRC of the above 1byte 
                 * No need to sniff the 64bits sent back by DS2432
                 */
                break;
            case F120_ROMCMD_MATCHROM:
                /* Master TX bits to match ROM */
                /* Relay signal to DS2432 */
                gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_ROM_SEARCH:
                /* Relay signal to DS2432 */
                break;
            case F120_MEM_CMD:
                /* If DS2432 doesn't respond, the reset pulse will trigger */
                break;
            default:
                break;
        }
		timer_clear_flag(TIM(F120_TIMER), TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF); // clear all flags
		timer_enable_counter(TIM(F120_TIMER)); // start timer to measure the configured timeouts
    }
}

void EXTI_ISR(DS2432_PIN)(void) {
    exti_reset_request(EXTI(DS2432_PIN));

    if (gpio_get(GPIO(DS2432_PORT), GPIO(DS2432_PIN))) { /* high */
        switch (ds2432_state) {
            case DS2432_DS2432_PRESENT:
                f120_state = F120_DS2432_PRESENT;
                gpio_set(GPIO(F120_PORT), GPIO(F120_PIN));
                break;
            case DS2432_ROMCMD_READROM:
                if (rx_mode == true) {
                    rx_mode = false;
                    gpio_set(GPIO(F120_PORT), GPIO(F120_PIN));
                }
                break;
            default:
                break;
        }
    } else { /* DS2432 GPIO Pulled Low */
        switch (ds2432_state) {
            case DS2432_WAIT_PRESENCE:
                f120_state = F120_WAIT_PRESENCE;
                gpio_clear(GPIO(F120_PORT), GPIO(F120_PIN));
            case DS2432_ROMCMD_READROM:
            /* In Read ROM (Master RX Mode), the initial signal (low) from Master ilicits
             * a response from DS2432 (if the bit to be transmitted is zero), however,
             * since the master GPIO signal is emulated by software, it will remain low even 
             * after the DS2432 stopped pulling it. To overcome this, a flag is set to determine
             * whether the signal is from DS2432 or from the master.
             * The signal from DS2432 will be relayed to master while the signal from master
             * will set the signal back to high
             */
                if (rx_mode == true) {
                    rx_mode = false;
                    gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                } else {
                    rx_mode = true;
                    gpio_clear(GPIO(F120_PORT), GPIO(F120_PIN));
                }
                break;
            default:
                break;
        }
    }
}

void TIM_ISR(F120_TIMER)(void) {
    if (timer_interrupt_source(TIM(F120_TIMER), TIM_SR_UIF)) {
        if (gpio_get(GPIO(F120_PORT), GPIO(F120_PIN)) == 0) {
            f120_state = F120_RESET;
            f120_rc_flag = false;
            rx_mode = false;
        }
        timer_disable_counter(TIM(F120_TIMER));
        timer_disable_irq(TIM(F120_TIMER), TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE);
        timer_clear_flag(TIM(F120_TIMER), TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);
    }

    if (timer_interrupt_source(TIM(F120_TIMER), TIM_SR_CC2IF)) {
        switch (f120_state) {
            case F120_ROM_CMD:
                bit_ctr++;
                if (gpio_get(GPIO(F120_PORT), GPIO(F120_PIN))) {
                    byte = byte | (1 << bit_ctr);
                }
                if (bit_ctr == 8) {
                    bit_ctr = 0;
                    f120_state = F120_ROM_CMD_DONE;
                }
                break;
            default:
                break;
        }
        timer_clear_flag(TIM(F120_TIMER), TIM_SR_CC2IF);
    }
}

/* Logs */
/* 20191107 - Done initialization and reset */
