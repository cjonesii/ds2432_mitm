/** @addtogroup flash_file
 *
 */

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
 * Copyright (C) 2010 Mark Butler <mbutler@physics.otago.ac.nz>
 * Copyright (C) 2012-13 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Flash routines shared by L0/L1.  L4 has a different flash controller */

/**@{*/

#include <libopencm3/stm32/flash.h>

/*---------------------------------------------------------------------------*/
/** @brief Enable the FLASH Prefetch Buffer

This buffer is used for instruction fetches and is enabled by default after
reset.

Note carefully the restrictions under which the prefetch buffer may be
enabled or disabled. Prefetch is only available when 64-bit
access is enabled.
*/

void flash_prefetch_enable(void)
{
	FLASH_ACR |= FLASH_ACR_PRFTEN;
}

/*---------------------------------------------------------------------------*/
/** @brief Disable the FLASH Prefetch Buffer

Note carefully the restrictions under which the prefetch buffer may be
set to disabled. See the reference and programming manuals for details.
*/

void flash_prefetch_disable(void)
{
	FLASH_ACR &= ~FLASH_ACR_PRFTEN;
}

/*---------------------------------------------------------------------------*/
/** @brief Set the Number of Wait States

Used to match the system clock to the FLASH memory access time. See the
programming manual for more information on clock speed and voltage ranges. The
latency must be changed to the appropriate value <b>before</b> any increase in
clock speed, or <b>after</b> any decrease in clock speed. A latency setting of
zero only applies if 64-bit mode is not used.

@param[in] ws values from @ref flash_latency.
*/

void flash_set_ws(uint32_t ws)
{
	uint32_t reg32;

	reg32 = FLASH_ACR;
	reg32 &= ~(1 << 0);
	reg32 |= ws;
	FLASH_ACR = reg32;
}

/**
 * Unlock primary access to the flash control/erase block
 * You must call this before using any of the low level routines
 * yourself.
 * @sa flash_unlock
 */
void flash_unlock_pecr(void)
{
	FLASH_PEKEYR = FLASH_PEKEYR_PEKEY1;
	FLASH_PEKEYR = FLASH_PEKEYR_PEKEY2;
}

void flash_lock_pecr(void)
{
	FLASH_PECR |= FLASH_PECR_PELOCK;
}

/**
 * Unlock program memory itself.
 * Writes the magic sequence to unlock the program memory
 * you must have already unlocked access to this register!
 * @sa flash_unlock_pecr
 */
void flash_unlock_progmem(void)
{
	FLASH_PRGKEYR = FLASH_PRGKEYR_PRGKEY1;
	FLASH_PRGKEYR = FLASH_PRGKEYR_PRGKEY2;
}

void flash_lock_progmem(void)
{
	FLASH_PECR |= FLASH_PECR_PRGLOCK;
}

/**
 * Unlock option bytes.
 * Writes the magic sequence to unlock the option bytes,
 * you must have already unlocked access to this register!
 * @sa flash_unlock_pecr
 */
void flash_unlock_option_bytes(void)
{
	FLASH_OPTKEYR = FLASH_OPTKEYR_OPTKEY1;
	FLASH_OPTKEYR = FLASH_OPTKEYR_OPTKEY2;
}

void flash_lock_option_bytes(void)
{
	FLASH_PECR |= FLASH_PECR_OPTLOCK;
}

/** @brief Unlock all segments of flash
 *
 */
void flash_unlock(void)
{
	flash_unlock_pecr();
	flash_unlock_progmem();
	flash_unlock_option_bytes();
}

/** @brief Lock all segments of flash
 *
 */
void flash_lock(void)
{
	flash_lock_option_bytes();
	flash_lock_progmem();
	flash_lock_pecr();
}

/** @brief Write a word to eeprom
 *
 * @param address assumed to be in the eeprom space, no checking
 * @param data word to write
 */
void eeprom_program_word(uint32_t address, uint32_t data)
{
	flash_unlock_pecr();
	/* erase only if needed */
	FLASH_PECR &= ~FLASH_PECR_FTDW;
	MMIO32(address) = data;
	flash_lock_pecr();
}

/** @brief Write a block of words to eeprom
 *
 * Writes a block of words to EEPROM at the requested address, erasing if
 * necessary, and locking afterwards.  Only wordwise writing is safe for
 * writing any value
 *
 * @param[in] address must point to EEPROM space, no checking!
 * @param[in] data pointer to data to write
 * @param[in] length_in_words size of of data in WORDS!
 */
void eeprom_program_words(uint32_t address, uint32_t *data, int length_in_words)
{
	int i;
	flash_unlock_pecr();
	while (FLASH_SR & FLASH_SR_BSY);
	/* erase only if needed */
	FLASH_PECR &= ~FLASH_PECR_FTDW;
	for (i = 0; i < length_in_words; i++) {
		MMIO32(address + (i * sizeof(uint32_t))) = *(data+i);
		while (FLASH_SR & FLASH_SR_BSY);
	}
	flash_lock_pecr();
}

/**@}*/
