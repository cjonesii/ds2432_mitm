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
	F120_ROM_MATCH,
	F120_ROM_SEARCH,
	F120_ROM_RESUME_CMD,
	F120_ROM_OD_SKIP,
	F120_ROM_OD_MATCH,
    F120_MAX
} f120_state = F120_IDLE;

static volatile enum {
	DS2432_IDLE,
	DS2432_RESET,
	DS2432_WAIT_PRESENCE,
	DS2432_DS2432_PRESENT,
	DS2432_ROM_CMD,
	DS2432_ROMCMD_READROM,
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

    /* DS2432 Slave*/
    rcc_periph_clock_enable(RCC_TIM(DS2432_TIMER));
	timer_reset(TIM(DS2432_TIMER));
    timer_set_mode(TIM(DS2432_TIMER), TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM(DS2432_TIMER), 1-1);
	timer_set_period(TIM(F120_TIMER), 33259);
	timer_set_oc_mode(TIM(F120_TIMER), TIM_OC1, TIM_OCM_FROZEN);
	timer_set_oc_value(TIM(F120_TIMER), TIM_OC1, 16*(rcc_ahb_frequency/1000000)-1); /*1151);*/
	timer_set_oc_mode(TIM(F120_TIMER), TIM_OC2, TIM_OCM_FROZEN);
	timer_set_oc_value(TIM(F120_TIMER), TIM_OC2, 45*(rcc_ahb_frequency/1000000)-1-350); /*2289);*/
	timer_set_oc_mode(TIM(F120_TIMER), TIM_OC1, TIM_OCM_FROZEN);
	timer_set_oc_value(TIM(F120_TIMER), TIM_OC1, 6497);
	timer_clear_flag(TIM(DS2432_TIMER), TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);
	timer_update_on_overflow(TIM(F120_TIMER));
	timer_enable_irq(TIM(DS2432_TIMER), TIM_DIER_UIE);
	nvic_enable_irq(NVIC_TIM_IRQ(DS2432_TIMER));

    gpio_set(GPIO(DS2432_PORT),GPIO(DS2432_PIN));
    gpio_set_mode(GPIO(DS2432_PORT), GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO(DS2432_PIN));

	exti_select_source(EXTI(DS2432_PIN), GPIO(DS2432_PORT));
	exti_set_trigger(EXTI(DS2432_PIN), EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI(DS2432_PIN));
	nvic_enable_irq(NVIC_EXTI_IRQ(DS2432_PIN));
}

void EXTI_ISR(F120_PIN)(void) {
    exti_reset_request(EXTI(F120_PIN));

    if (gpio_get(GPIO(F120_PORT), GPIO(F120_PIN))) { /* high */
        switch (f120_state) {
            case F120_RESET:
                /* This must be triggered by F120 (HW triggered) */
                f120_state = F120_WAIT_PRESENCE;
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_DS2432_PRESENT:
                /* This should be from DS2432 GPIO (SW triggered) */
                /* F120 is in TX Mode */
                f120_state = F120_ROM_CMD;
                break;
            case F120_ROM_CMD:
                /* This must be triggered by F120 (HW triggered) */
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_ROMCMD_READROM:
                /* This must be triggered by F120 (HW triggered) */
                bit_ctr++;
                byte_ctr = bit_ctr / 8;

                if (bit_ctr == 64) {
                    f120_state = F120_MEM_CMD;
                    byte_ctr = 0;
                }
                break;
            case F120_ROM_MATCH:
                /* Master TX bits to match ROM */
                /* Relay signal to DS2432 */
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                bit_ctr++;
                if (bit_ctr == 64) {
                    f120_rc_flag = true;
                    f120_state = F120_MEM_CMD;
                }
            default:
				/* F120_IDLE 
				 * F120_WAIT_PRESENCE
				 * f120_state = F120_IDLE;
                 */
				break;
        }
    } else { /* low */
		timer_disable_counter(TIM(F120_TIMER)); // stop timer for reconfiguration
		timer_set_counter(TIM(F120_TIMER), 0); // reset timer counter
		timer_disable_irq(TIM(F120_TIMER), TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE); // disable all timers
        switch (f120_state) {
            case F120_IDLE:
                /* This must be triggered by F120 (HW triggered) */
                f120_state = F120_RESET;
                gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_WAIT_PRESENCE:
                /* This should be from DS2432 GPIO (SW triggered) */
                f120_state = F120_DS2432_PRESENT;
                break;
            case F120_ROM_CMD:
                /* This must be triggered by F120 (HW triggered) */
                /* F120 is in TX Mode */
                /* Trigger the timer to sniff what bit is sent */
                timer_enable_irq(TIM(F120_TIMER), TIM_DIER_CC2IE);
                gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_ROMCMD_READROM:
                /* F120 will always initiate the Read Time Slot */
                /* This must be triggered by F120 (HW triggered) */
                gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                /* DS2432 TX:
                 *   Chip Family Code 1byte
                 *   Serial Number    6bytes
                 *   CRC of the above 1byte 
                 * No need to sniff the 64bits sent back by DS2432
                 */
                break;
            case F120_ROM_MATCH:
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

void TIM_ISR(F120_TIMER)(void) {
    if (timer_interrupt_source(TIM(F120_TIMER), TIM_SR_UIF)) {
        if (gpio_get(GPIO(F120_PORT), GPIO(F120_PIN)) == 0) {
            f120_state = F120_RESET;
        }
        timer_disable_counter(TIM(F120_TIMER));
        timer_disable_irq(TIM(F120_TIMER), TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE);
        timer_clear_flag(TIM(F120_TIMER), TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);
    }
    if (timer_interrupt_source(TIM(F120_TIMER), TIM_SR_CC2IF)) {
        switch (f120_state) {
            case F120_ROM_CMD:
                if (gpio_get(GPIO(F120_PORT), GPIO(F120_PIN))) {
                    byte = byte | (1 << bit_ctr);
                    bit_ctr++;
                    if (bit_ctr == 8) {
                        bit_ctr = 0;
                        switch(byte) {
                            case 0x33: /* Read ROM */
                                f120_state = F120_ROMCMD_READROM;
                                ds2432_state = DS2432_ROMCMD_READROM;
                                break;
                            case 0x55: /* Match ROM */
                                f120_state = F120_ROM_MATCH;
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
                }
                break;
            default:
                break;
        }
        timer_clear_flag(TIM(F120_TIMER), TIM_SR_CC2IF);
    }

    /*if (f120_state == F120_IDLE) {
        if (!(gpio_get(GPIO(F120_PORT), GPIO(F120_PIN)))) {
            f120_state = F120_RESET;
        }
    }*/

    /*if (f120_state == F120_ROM_MATCH) {
        if (!(gpio_get(GPIO(F120_PORT), GPIO(F120_PIN)))) {
            f120_state = F120_RESET;
        }
    }*/

    /*if (f120_state == F120_ROMCMD_READROM) {
        if (!(gpio_get(GPIO(F120_PORT), GPIO(F120_PIN)))) {
            gpio_set(GPIO(F120_PORT), GPIO(F120_PIN));
        }
    }*/
}

void EXTI_ISR(DS2432_PIN)(void) {
    exti_reset_request(EXTI(DS2432_PIN));

    if (gpio_get(GPIO(DS2432_PORT), GPIO(DS2432_PIN))) { /* high */
        switch (ds2432_state) {
            case DS2432_RESET:
                /* This should be from F120 GPIO (SW triggered) */
                ds2432_state = DS2432_WAIT_PRESENCE;
                break;
            case DS2432_DS2432_PRESENT:
                /* This must be triggered by DS2432 chip (HW triggered)*/
                ds2432_state = DS2432_ROM_CMD;
                gpio_set(GPIO(F120_PORT), GPIO(F120_PIN));
                break;
            case DS2432_ROM_CMD:
                /* This should be from F120 GPIO (SW triggered) */
                /* F120 is in TX Mode, just chill */
                /*break;*/
            default:
                break;
        }
    } else { /* low */
		timer_disable_counter(TIM(DS2432_TIMER)); // stop timer for reconfiguration
		timer_set_counter(TIM(DS2432_TIMER), 0); // reset timer counter
		timer_disable_irq(TIM(DS2432_TIMER), TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE); // disable all timers
        switch (ds2432_state) {
            case DS2432_IDLE:
                /* This should be from F120 GPIO (SW triggered) */
                ds2432_state = DS2432_RESET;
                break;
            case DS2432_WAIT_PRESENCE:
                /* This must be triggered by DS2432 chip (HW triggered) */
                ds2432_state = DS2432_DS2432_PRESENT;
                gpio_clear(GPIO(F120_PORT), GPIO(F120_PIN));
            case DS2432_ROMCMD_READROM:
                /* This must be triggered by DS2432 chip (HW triggered) */
                /* F120 is in RX Mode */
                gpio_clear(GPIO(F120_PORT), GPIO(F120_PIN));
                break;
            case DS2432_ROM_CMD:
            default:
                break;
        }
		timer_clear_flag(TIM(DS2432_TIMER), TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF); // clear all flags
		timer_enable_counter(TIM(DS2432_TIMER)); // start timer to measure the configured timeouts
    }
}


/* Logs */
/* 20191107 - Done initialization and reset */
