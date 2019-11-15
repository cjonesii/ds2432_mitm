/* linker script for application running on STM32F103x8 micro-controller
 * the STM32F103x8 has 64kB of flash starting at 0x0800 0000, and 20kB of RAM starting at 0x2000 0000
 * the USB DFU bootloader will take the first 8 kB of flash, followed by the application
 */

/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 8K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}
PROVIDE(__application_beginning = ORIGIN(rom) + LENGTH(rom));
PROVIDE(__application_end = __application_beginning + 56K);

/* include rest of the definitions for the STM32F1 family */
INCLUDE libopencm3_stm32f1.ld
