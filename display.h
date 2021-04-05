#pragma once

#include "nmea.h"

#include <stdint.h>

#define kNumDigits 6
#define kNumSegments 8

/**
 * Send setup commands to the MAX7219 driver chip
 */
void max7219_init(void);

/**
 * Send a command and data byte to the MAX7219
 */
void max7219_cmd(uint8_t address, uint8_t data);

/**
 * Send complete digit/segment register configuration to the MAX72XX
 *
 * This must be called for max7219_set_digit calls to be shown on the display.
 *
 * All 8 digit (sink) registers need to be set at once as our wiring is flipped in order
 * to drive common anode displays. Each of our phsyical digits is represented by one bit
 * in each of the 8 digit registers (instead of the normal one-byte-per-digit wiring).
 */
void display_send_buffer();

/**
 * Emulate setting a digit register on the MAX72XX (mapped for common anode wiring)
 * Digit register is 1-indexed.
 */
void max7219_set_digit(uint8_t digitRegister, uint8_t segments);

/**
 * Emulate writing a digit under the MAX72XX's BCD display mode (mapped for common anode wiring)
 * Digit register is 1-indexed
 */
void display_set_digit_bcd(uint8_t digitRegister, uint8_t value);

/**
 * Send the passed time to the MAX7219 as 6 BCD digits
 */
void display_set_buffer_partial(uint8_t digitIndex, uint8_t value);

/**
 * Send the passed time to the MAX7219 as 6 BCD digits
 */
void display_set_buffer(DateTime* now);

/**
 * Set segments to display the passed string (approximately)
 */
void display_set_buffer_ascii(uint8_t digitIndex, const char* str);

/**
 * Clear the display buffer
 */
void display_clear();

/**
 * Rotate the currently writable buffer to be ready for output by display_send_buffer
 */
void display_swap_buffers();

/**
 * Clear all decimal segments and enable one in sequence
 * Each call, the enabled segment is stepped forward and loops
 */
void display_overlay_ticker();

/**
 * Display an "Er" with the passed digit
 */
void display_error_code(uint8_t code);

/**
 * Feed an ambient brightness ADC reading to update the display brightness
 */
void display_adjust_brightness(const uint16_t reading);