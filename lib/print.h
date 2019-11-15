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
/** printing utilities to replace the large printf from the standard library (API)
 *  @note use % as format specifier prefix, followed by + to enforce sign of prefix, 0 and 0-9 for padding, and format specifier
 *  format specifier supported are: c for far, s for string, u for uint32_t, d for int32_t, U for uint64_t, D for int64_t, x for lower case hex up to uint32_t, X for upper case hex up to uint32_t, b for bits up to uint32_t
 *  @file print.h
 *  @author King Kévin <kingkevin@cuvoodoo.info>
 *  @date 2017
 */
#pragma once

/** print a single character on user output
 *  @warning this must be implemented by the user (using the desired output interface)
 *  @param[in] c character to be printed
 *  @return number of characters printed
 **/
size_t putc(char c);
/** print format string on user output
 *  @param[in] format format string to be printed
 *  @param[in] ... arguments referenced by format string to be printed
 *  @return number of characters printed
 **/
size_t printf(const char* format, ...);
/** print format string on string or user output
 *  @param[out] str string to print format string on, or user output if str is set to NULL (str will always be terminated with a null character '\0')
 *  @param[in,out] size size of string (writes at most size characters on str, including the termination null character '\0')
 *  @param[in] format format string to be printed
 *  @param[in] ... arguments referenced by format string to be printed
 *  @return number of characters printed (a return value of size or more means that the output was truncated) 
 **/
size_t snprintf(char* str, size_t size, const char* format, ...);
