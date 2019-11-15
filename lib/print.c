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
/** printing utilities to replace the large printf from the standard library (code)
 *  @file print.c
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2017
 */
/* standard libraries */
#include <stdint.h> // standard integer types
#include <stdlib.h> // standard definitions
#include <stdbool.h> // boolean types
#include <stdarg.h> // variadic utilities

/* own libraries */
#include "print.h" // printing utilities

/** @defgroup print_crlf output \r\n (Carriage Return + Line Feed) for each \r, \n, \r\n, or \n\r for better terminal compatibility
 *  @{
 **/
#define CRLF true /**< if CR+LN new line should be enforced */
/** @} */

/** print character
 *  @param[out] str string to print character on (use NULL to print on user output)
 *  @param[in,out] size size of string
 *  @param[in] c character to be printed
 *  @return number of characters printed
 **/
static size_t print_char(char** str, size_t* size, char c)
{
	size_t length = 1; // remember how many characters have been printed or should have been added on string (normally just one)
	if (0==c) { // don't print string termination character
		length = 0; // remember we didn't print anything
	} else if (NULL==str || NULL==*str || NULL==size) { // character should not be saved on string
		length = putc(c); // print on user define output
	} else if (*size>1) { // // there is enough space in the string to store the character
		**str = c; // add provided character to string
		*str += 1; // go to next character on string
		*size -= 1; // remember we used one character on string
	} else if (1==*size) { // string is reaching it's end
		**str = '\0'; // add termination character to string (don't go to next character)
		*size -= 1; // remember we used one character on string
	}
	return length;
}

/** print string
 *  @param[out] str string to print string on (use NULL to print on user output)
 *  @param[in,out] size size of string
 *  @param[in] s string to be printed
 *  @return number of characters printed
 **/
static size_t print_string(char** str, size_t* size, const char* s)
{
	size_t length = 0; // number of characters printed
	while (*s) { // stop at end of string
		length += print_char(str, size, *(s++)); // print character
	}
	return length;
}

/** print unsigned number
 *  @param[out] str string to print unsigned number on (use NULL to print on user output)
 *  @param[in,out] size size of string
 *  @param[in] u unsigned number to be printed
 *  @param[in] padding number of 0's to pad
 *  @param[in] sign if sign should be printed
 *  @return number of characters printed
 **/
static size_t print_unsigned(char** str, size_t* size, uint64_t u, uint8_t padding, bool sign) {
	char number[20] = {0}; // construct the number in reverse order (20 chars are required to store UINT64_MAX)
	uint8_t digits = 0; // to count the number of digits
	size_t length = 0; // number of characters printed
	do {
		number[digits++] = '0'+(u%10); // store digit
		u /= 10; // go to next digit
	} while (u>0);
	if (digits>sizeof(number)) { // prevent buffer underflow
		return 0;
	}
	if (sign) { // print sign
		length += print_char(str, size, '+'); // we only have positive numbers
	}
	for (uint8_t zeros = digits; zeros<padding; zeros++) { // print padding 0's
		length += print_char(str, size, '0'); // print 0
	}
	for (uint8_t digit = 0; digit < digits; digit++) { // go through all digits
		length += print_char(str, size, number[digits-digit-1]); // print digit (in reverse order)
	}
	return length; // return number of characters printed
}

/** print signed number
 *  @param[out] str string to print signed number on (use NULL to print on user output)
 *  @param[in,out] size size of string
 *  @param[in] d signed number to be printed
 *  @param[in] padding number of 0's to pad
 *  @param[in] sign if sign should be printed
 *  @return number of characters printed
 **/
static size_t print_signed(char** str, size_t* size, int64_t d, uint8_t padding, bool sign) {
	size_t length = 0; // number of characters printed
	if (d<0) {
		length += print_char(str, size, '-'); // print sign
		length += print_unsigned(str, size, (uint64_t)-d, padding, false); // print number (casting because there is one more negative value then positive value)
	} else {
		length += print_unsigned(str, size, d, padding, sign); // print number
	}
	return length; // return number of characters printed
}

/** print nibble (half-byte)
 *  @param[out] str string to print nibble on (use NULL to print on user output)
 *  @param[in,out] size size of string
 *  @param[in] nibble nibble to be printed
 *  @param[in] upcase use upcase digits (A-F)
 *  @return number of characters printed
 **/
static size_t print_nibble(char** str, size_t* size, uint8_t nibble, bool upcase) {
	size_t length = 0; // number of characters printed
	nibble &= 0x0f; // ensure we only have a nibble
	if (nibble<10) {
		length += print_char(str, size, '0'+nibble);
	} else if (upcase) {
		length += print_char(str, size, 'A'+nibble-10);
	} else {
		length += print_char(str, size, 'a'+nibble-10);
	}
	return length; // return number of characters printed
}

/** print hex value
 *  @param[out] str string to print hex on (use NULL to print on user output)
 *  @param[in,out] size size of string
 *  @param[in] hex hex value to be printed
 *  @param[in] padding number of 0's to pad
 *  @param[in] prefix if 0x prefix should be printed
 *  @param[in] upcase use upcase digits (A-F)
 *  @return number of characters printed
 **/
static size_t print_hex(char** str, size_t* size, uint32_t hex, uint8_t padding, bool prefix, bool upcase) {
	size_t length = 0; // number of characters printed
	if (prefix) { // print 0x prefix
		length += print_char(str, size, '0');
		length += print_char(str, size, 'x');
	}
	uint8_t digits = 0; // number of digits to print
	// figure out number of digits to print
	if (hex>0x00ffffff) {
		digits = 8;
	} else if (hex>0x0000ffff) {
		digits = 6;
	} else if (hex>0x000000ff) {
		digits = 4;
	} else {
		digits = 2;
	}
	for (uint8_t zeros = digits; zeros<padding; zeros++) { // print padding 0's
		length += print_char(str, size, '0'); // print 0
	}
	for (uint8_t digit = 0; digit < digits; digit++) { // go through all digits
		length += print_nibble(str, size, hex>>((digits-digit-1)*4), upcase); // print nibble (in reverse order)
	}
	return length; // return number of characters printed
} 

/** print bits
 *  @param[out] str string to print bits on (use NULL to print on user output)
 *  @param[in,out] size size of string
 *  @param[in] u bits to be printed
 *  @param[in] padding number of 0's to pad
 *  @param[in] prefix if 0b prefix should be printed
 *  @return number of characters printed
 **/
static size_t print_bits(char** str, size_t* size, uint32_t u, uint8_t padding, bool prefix) {
	char bits[32] = {0}; // construct the bit string in reverse order
	uint8_t digits = 0; // to count the number of digits
	size_t length = 0; // number of characters printed
	do {
		bits[digits++] = '0'+(u&0x1); // store bit
		u >>= 1; // go to next bit
	} while (u>0);
	if (digits>sizeof(bits)) { // prevent buffer underflow
		return 0;
	}
	if (prefix) { // print prefix
		length += print_char(str, size, '0');
		length += print_char(str, size, 'b');
	}
	for (uint8_t zeros = digits; zeros<padding; zeros++) { // print padding 0's
		length += print_char(str, size, '0'); // print 0
	}
	for (uint8_t digit = 0; digit < digits; digit++) { // go through all bits
		length += print_char(str, size, bits[digits-digit-1]); // print bit (in reverse order)
	}
	return length; // return number of characters printed
}

/** print format string on string or user output
 *  @param[out] str string to print format string on, or user output if str is set to NULL (str will always be terminated with a null character '\0')
 *  @param[in,out] size size of string (writes at most size characters on str, including the termination null character '\0')
 *  @param[in] format format string to be printed
 *  @param[in] va arguments referenced by format string to be printed
 *  @return number of characters printed (a return value of size or more means that the output was truncated) 
 **/
static size_t vsnprintf(char** str, size_t* size, const char *format, va_list va)
{
	size_t length = 0; // number of characters printed
	uint8_t padding = 0; // number of padding 0's
	bool sign = false; // if sign needs to be printed
	while (*format) { // go through format string
		padding = 0; // reset padding
		sign = false; // reset sign
		if ('%'!=*format) { // check for format specifier prefix
			length += print_char(str, size, *format++); // print character (no interpretation needed)
		} else {
			format++; // go to format specifier
			if (0==*format) { // end of string detected
				goto end;
			}
			// check if sign need to be printed
			if ('+'==*format) { // sign required
				sign = true; // remember sign is required
				format++; // go to padding number
				if (0==*format) { // end of string detected
					goto end;
				}
			}	
			// check padding
			if ('0'==*format) { // padding required
				format++; // go to padding number
				if (0==*format) { // end of string detected
					goto end;
				}
				if (*format>='0' && *format<='9') {
					padding = *format-'0';
					format++; // go to format specifier
					if (0==*format) { // end of string detected
						goto end;
					}
				}
			}
			// check format specifier
			switch (*format) {
				case 'u': // for uint8_t, uint16_t, uint32_t, unsigned int, unsigned long
					length += print_unsigned(str, size, va_arg(va,uint32_t), padding, sign);
					break;
				case 'U': // for uint64_t, unsigned long long
					length += print_unsigned(str, size, va_arg(va,uint64_t), padding, sign);
					break;
				case 'd': // for int8_t, int16_t, int32_t, int, long
					length += print_signed(str, size, va_arg(va,int32_t), padding, sign);
					break;
				case 'D': // for int64_t, long long
					length += print_signed(str, size, va_arg(va,int64_t), padding, sign);
					break;
				case 'c': // for char, unsigned char
					length += print_char(str, size, (char)(va_arg(va,int))); // needs casting because the returned value is promoted
					break;
				case 'x': // for downcase hexadecimal
					length += print_hex(str, size, va_arg(va,uint32_t), padding, sign, false);
					break;
				case 'X': // for upcase hexadecimal
					length += print_hex(str, size, va_arg(va,uint32_t), padding, sign, true);
					break;
				case 'b': // for bits
					length += print_bits(str, size, va_arg(va,uint32_t), padding, sign);
					break;
				case 's': // for strings
					length += print_string(str, size, va_arg(va,char*));
					break;
				default:
					length += print_char(str, size, *format); // print character (unknown format specifier)
			}
			format++; // go to next character
		}
	}
end:
	if (NULL!=str && NULL!=*str && NULL!=size) { // when working on a string
		**str='\0'; // enforce null termination
	}
	return length; // return number of characters it should have written
}

size_t printf(const char *format, ...)
{
	size_t length = 0;
	va_list arglist;
	va_start(arglist, format);
	length = vsnprintf(NULL, NULL, format, arglist);
	va_end(arglist);
	return length;
}

size_t snprintf(char* str, size_t size, const char* format, ...)
{
	size_t length = 0;
	va_list arglist;
	va_start(arglist, format);
	length = vsnprintf(&str, &size, format, arglist);
	va_end(arglist);
	return length;
}
