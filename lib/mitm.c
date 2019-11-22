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

#define DS2432_PORT  B
#define DS2432_PIN   10

static volatile enum {
	F120_IDLE,
	F120_RESET,
	F120_WAIT_PRESENCE,
	F120_DS2432_PRESENT,
	F120_ROM_CMD,
    F120_ROM_CMD_DONE,
	F120_MEM_CMD,
	F120_MEM_CMD_DONE,
	F120_ROMCMD_READROM,
    F120_ROMCMD_READROM_DONE,
	F120_ROMCMD_MATCHROM,
	F120_ROMCMD_SEARCHROM,
	F120_ROM_RESUME_CMD,
	F120_ROM_OD_SKIP,
	F120_ROM_OD_MATCH,
    F120_MEMCMD_WRITESCRATCHPAD,
    F120_MEMCMD_READSCRATCHPAD,
    F120_MEMCMD_LOAD1STSECRET,
    F120_MEMCMD_COMPUTENXTSECRET,
    F120_MEMCMD_COPYSCRATCHPAD,
    F120_MEMCMD_READAUTHPAGE,
    F120_MEMCMD_READMEM,
    F120_MAX
} f120_state = F120_IDLE;

static volatile enum {
    DS2432_IDLE,
	DS2432_WAIT_PRESENCE,
	DS2432_DS2432_PRESENT,
    DS2432_ROM_CMD_DONE,
    DS2432_MEM_CMD_DONE,
	DS2432_ROMCMD_READROM,
	DS2432_ROMCMD_MATCHROM,
    DS2432_MAX
} ds2432_state = DS2432_IDLE;

/* static bool bypass_mode = true;
 */ 
static volatile bool f120_rc_flag = false;
/*static uint8_t bit = 0;*/
static volatile uint8_t bit_ctr = 0;
volatile uint8_t byte;
/* static uint8_t byte_ctr = 0; */
/*static uint8_t rom_cmd[8] = {0};*/
static volatile bool rx_init_flag = false;
volatile bool rom_cmd_received;
volatile bool read_rom_byte;
volatile bool mem_cmd_received;

void mitm_setup(void) {
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
    ds2432_state = DS2432_IDLE;
    rom_cmd_received = false;
    read_rom_byte = false;
    mem_cmd_received = false;
    rx_init_flag = false;
    byte = 0;
    bit_ctr = 0;
}

void EXTI_ISR(F120_PIN)(void) {
    exti_reset_request(EXTI(F120_PIN));

    if (gpio_get(GPIO(F120_PORT), GPIO(F120_PIN))) { /* F120 GPIO Pulled High */
        switch (f120_state) {
            case F120_RESET:
                ds2432_state = DS2432_WAIT_PRESENCE;
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_DS2432_PRESENT:
                f120_state = F120_ROM_CMD;
                break;
            case F120_ROM_CMD:
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_ROM_CMD_DONE:
                ds2432_state = DS2432_ROM_CMD_DONE;
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_ROMCMD_READROM_DONE:
                f120_state = F120_MEM_CMD;
                break;
            case F120_MEM_CMD:
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_MEM_CMD_DONE:
                ds2432_state = DS2432_MEM_CMD_DONE;
                gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
            case F120_ROMCMD_MATCHROM:
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
    } else { /* F120 GPIO Pulled Low */
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
                gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_ROMCMD_READROM:
                /* F120 will always initiate the Read Time Slot */
                if (rx_init_flag == false) {
                    rx_init_flag = true;
                    ds2432_state = DS2432_ROMCMD_READROM;
                    timer_enable_irq(TIM(F120_TIMER), TIM_DIER_CC2IE);
                    gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                }
                /* DS2432 TX:
                 *   Chip Family Code 1byte
                 *   Serial Number    6bytes
                 *   CRC of the above 1byte 
                 * No need to sniff the 8bytes (64bits) sent back by DS2432
                 */
                break;
            case F120_MEM_CMD:
                timer_enable_irq(TIM(F120_TIMER), TIM_DIER_CC2IE);
                gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_ROMCMD_MATCHROM:
                /* Master TX bits to match ROM */
                /* Relay signal to DS2432 */
                gpio_clear(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                break;
            case F120_ROMCMD_SEARCHROM:
                /* Relay signal to DS2432 */
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
            case DS2432_ROM_CMD_DONE: /* change state at last 0 to 1 transition if last bit is 0 */
                switch(byte) {
                    case 0x33: /* Read ROM */
                        f120_state = F120_ROMCMD_READROM;
                        break;
                    case 0x55: /* Match ROM */
                        f120_state = F120_ROMCMD_MATCHROM;
                        break;
                    case 0xf0: /* Search ROM */
                        f120_state = F120_ROMCMD_SEARCHROM;
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
                rom_cmd_received = true;
                break;
            case DS2432_ROMCMD_READROM:
                if (rx_init_flag == true) {
                    rx_init_flag = false;
                    gpio_set(GPIO(F120_PORT), GPIO(F120_PIN));
                }
                break;
            case DS2432_MEM_CMD_DONE:
                switch(byte) {
                    case 0x33: /* Read ROM */
                        f120_state = F120_ROMCMD_READROM;
                        break;
                    case 0x55: /* Match ROM */
                        f120_state = F120_ROMCMD_MATCHROM;
                        break;
                    case 0xf0: /* Search ROM */
                        f120_state = F120_ROMCMD_SEARCHROM;
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
                mem_cmd_received = true;
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
    /* In Read ROM (Master RX Mode / DS2432 TX Mode), the initial signal (low)
     * from Master ilicits a response from DS2432 to keep the signal to a low
     * state if the bit to be transmitted is zero, however, since the master
     * GPIO signal is emulated by software, the signal tends to remain low even
     * after the DS2432 stopped driving it. To overcome this, a flag is set to
     * determine whether the signal came from DS2432 or from the master.
     * The signal from DS2432 will be relayed to master while the signal from
     * master will set the signal back to high.
     */
                if (rx_init_flag == true) {
                    rx_init_flag = false;
                    gpio_set(GPIO(DS2432_PORT), GPIO(DS2432_PIN));
                } else {
                    rx_init_flag = true;
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
            rx_init_flag = false;
            byte = 0;
            bit_ctr = 0;
        }
        timer_disable_counter(TIM(F120_TIMER));
        timer_disable_irq(TIM(F120_TIMER), TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE);
        timer_clear_flag(TIM(F120_TIMER), TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);
    }

    if (timer_interrupt_source(TIM(F120_TIMER), TIM_SR_CC2IF)) {
        uint16_t gpio_val;
        switch (f120_state) {
            case F120_ROM_CMD:
                gpio_val = gpio_get(GPIO(F120_PORT), GPIO(F120_PIN));
                byte = byte | (gpio_val << bit_ctr);
                bit_ctr++;

                if (bit_ctr == 8) {
                    bit_ctr = 0;
                    if (gpio_val) { /* Change state immediately if last bit is 1 */
                        switch(byte) {
                            case 0x33: /* Read ROM */
                                f120_state = F120_ROMCMD_READROM;
                                break;
                            case 0x55: /* Match ROM */
                                f120_state = F120_ROMCMD_MATCHROM;
                                break;
                            case 0xf0: /* Search ROM */
                                f120_state = F120_ROMCMD_SEARCHROM;
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
                        rom_cmd_received = true;
                    } else {
                        f120_state = F120_ROM_CMD_DONE;
                    }
                }
                break;
            case F120_ROMCMD_READROM:
                gpio_val = gpio_get(GPIO(F120_PORT), GPIO(F120_PIN));
                byte = byte | (gpio_val << bit_ctr);
                bit_ctr++;
                if ((bit_ctr % 8) == 0) {
                    read_rom_byte = true;
                }
                if (bit_ctr == 64) {
                    bit_ctr = 0;
                    if (gpio_val) { /* Change state immediately if last bit is 1 */
                        f120_state = F120_MEM_CMD;
                    } else {
                        f120_state = F120_ROMCMD_READROM_DONE;
                    }
                }
                break;
            case F120_MEM_CMD:
                gpio_val = gpio_get(GPIO(F120_PORT), GPIO(F120_PIN));
                byte = byte | (gpio_val << bit_ctr);
                bit_ctr++;
                if (bit_ctr == 8) {
                    bit_ctr = 0;
                    if (gpio_val) { /* Change state immediately if last bit is 1 */
                        switch(byte) {
                            case 0x0F: /* Write Scratchpad */
                                f120_state = F120_MEMCMD_WRITESCRATCHPAD;
                                break;
                            case 0xAA: /* Read Scratchpad */
                                f120_state = F120_MEMCMD_READSCRATCHPAD;
                                break;
                            case 0x5A: /* Load First Secret */
                                f120_state = F120_MEMCMD_LOAD1STSECRET;
                                break;
                            case 0x33: /* Compute Next Secret */
                                f120_state = F120_MEMCMD_COMPUTENXTSECRET;
                                break;
                            case 0x55: /* Copy Scratchpad */
                                f120_state = F120_MEMCMD_COPYSCRATCHPAD;
                                break;
                            case 0xA5: /* Read Authenticated Page */
                                f120_state = F120_MEMCMD_READAUTHPAGE;
                                break;
                            case 0xF0: /* Read Memory */
                                f120_state = F120_MEMCMD_READMEM;
                                break;
                            default:
                                break;
                        }
                        mem_cmd_received = true;
                    } else {
                        f120_state = F120_MEM_CMD_DONE;
                    }
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
