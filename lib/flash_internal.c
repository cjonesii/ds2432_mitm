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
/** library to read/write internal flash (code)
 *  @file flash_internal.c
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2016
 *  @note peripherals used: none
 */
/* standard libraries */
#include <stdint.h> // standard integer types
#include <stdlib.h> // general utilities

/* STM32 (including CM3) libraries */
#include <libopencm3/stm32/desig.h> // device signature utilities
#include <libopencm3/stm32/flash.h> // flash utilities

#include "flash_internal.h" // flash storage library API
#include "global.h" // global definitions

/** the flash page size (medium-density devices have 1KiB page size) */
#define PAGE_SIZE 1024

bool flash_internal_read(uint32_t address, uint8_t *buffer, size_t size)
{
	// verify it's in the flash area and do other sanity checks
	if (address<FLASH_BASE || address>(UINT32_MAX-size) || (address+size)>(FLASH_BASE+DESIG_FLASH_SIZE*1024) || buffer==NULL || size==0) {
		return false;
	}

	// copy data byte per byte (a more efficient way would be to copy words, than the remaining bytes)
	for (size_t i=0; i<size; i++) {
		buffer[i] = *((uint8_t*)address+i);
	}

	return true;
}

bool flash_internal_write(uint32_t address, uint8_t *buffer, size_t size)
{
	// verify it's in the flash area and do other sanity checks
	if (address<FLASH_BASE || address>(UINT32_MAX-size) || (address+size)>(FLASH_BASE+DESIG_FLASH_SIZE*1024) || buffer==NULL || size==0 || size%2) {
		return false;
	}

	flash_unlock(); // unlock flash to be able to write it
	while (size) { // write page by page until all data has been written
		uint32_t page_start = address-(address%PAGE_SIZE); // get start of the current page
		bool erase = false; // verify if the flash to write is erased of if we need to erase the page
		for (uint32_t flash=address; flash<(address+size) && flash<(page_start+PAGE_SIZE); flash += 2) { // go through page
			if (*(uint16_t*)(flash)!=0xffff) { // is flash not erased
				erase = true; // the erase flash
			}
		}
		if (erase) { // make copy of the page to erase and erase it
			uint8_t page_data[PAGE_SIZE]; // a copy of the complete page before the erase it
			uint16_t page_i = 0; // index for page data
			// copy page before address
			for (uint32_t flash=page_start; flash<address && flash<(page_start+PAGE_SIZE) && page_i<sizeof(page_data); flash++) {
				page_data[page_i++] = *(uint8_t*)(flash);
			}
			// copy data starting at address
			while (size>0 && page_i<sizeof(page_data)) {
				page_data[page_i++] = *buffer;
				buffer++;
				address++;
				size--;
			}
			// copy data after buffer until end of page
			while (page_i<sizeof(page_data)) {
				page_data[page_i] = *(uint8_t*)(page_start+page_i);
				page_i++;
			}
			flash_erase_page(page_start); // erase current page
			if (flash_get_status_flags()!=FLASH_SR_EOP) { // operation went wrong
				flash_lock(); // lock back flash to protect it
				return false;
			}
			for (uint16_t i=0; i<PAGE_SIZE/2; i++) { // write whole page
				flash_program_half_word(page_start+i*2, *((uint16_t*)(page_data+i*2)));
				if (flash_get_status_flags()!=FLASH_SR_EOP) { // operation went wrong
					flash_lock(); // lock back flash to protect it
					return false;
				}
				if (*((uint16_t*)(page_data+i*2))!=*((uint16_t*)(page_start+i*2))) { // verify the programmed data is right
					flash_lock(); // lock back flash to protect it
					return false;
				}
			}
		} else { // simply data until end of page
			while (size>0 && address<(page_start+PAGE_SIZE)) {
				flash_program_half_word(address, *((uint16_t*)(buffer)));
				if (flash_get_status_flags()!=FLASH_SR_EOP) { // operation went wrong
					flash_lock(); // lock back flash to protect it
					return false;
				}
				if (*((uint16_t*)address)!=*((uint16_t*)buffer)) { // verify the programmed data is right
					flash_lock(); // lock back flash to protect it
					return false;
				}
				buffer += 2;
				address += 2;
				size -= 2;
			}
		}
	}
	flash_lock(); // lock back flash to protect it

	return true;
}
