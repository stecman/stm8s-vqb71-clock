#include "display.h"
#include "pindefs.h"
#include "spi.h"

#include <stddef.h>

// Double buffer for segment data
static uint8_t _buffers[2][kNumSegments] = {{0}, {0}};
static uint8_t* _segmentWiseData = (uint8_t*) &_buffers[0];
static uint8_t* _sendBuffer = (uint8_t*) &_buffers[1];

// Map matching the MAX7219's BCD input mode
// @see display_set_digit_bcd
const uint8_t max7219_bcdMap[16] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
    0b01000000, // -
    0b01111001, // E
    0b01110110, // H
    0b00111000, // L
    0b01110011, // P
    0x0,        // Blank
};

// ASCII map for text display
// @see display_set_buffer_ascii
const uint8_t max7219_charMap[26] = {
    0b01011111, // a
    0b01111100, // b
    0b01011000, // c
    0b01011110, // d
    0b01111011, // e
    0b01110001, // f
    0b01101111, // g
    0b01110100, // h
    0b00010000, // i
    0b00001100, // j
    0b01110101, // k
    0b00110000, // l
    0b00010100, // m
    0b01010100, // n
    0b01011100, // o
    0b01110011, // p
    0b01100111, // q
    0b01010000, // r
    0b01101101, // s
    0b01111000, // t
    0b00011100, // u
    0b00011100, // v
    0b00010100, // w
    0b01110110, // x
    0b01101110, // y
    0b01011011, // z
};

// Map of logical digit index to rev 1.0 board wiring using a MAX7221
const uint8_t max7219_digitMap[kNumSegments] = {
    /* 0: */ 0,
    /* 1: */ 4,
    /* 2: */ 3,
    /* 3: */ 1,
    /* 4: */ 5,
    /* 5: */ 2,
};

void max7219_init(void)
{
    // Disable binary decode mode
    // We can't use this as we're using the MAX7219 to drive common-anode displays
    max7219_cmd(0x09, 0x00);

    // Set scan mode to 8x8
    // This is the "number of digits" command, but we've wired these as the segments
    max7219_cmd(0x0B, 0x7);

    // Disable test mode
    max7219_cmd(0x0F, 0);

    // Send the empty display buffer
    display_send_buffer();

    // Enable display
    max7219_cmd(0x0C, 1);
}

void max7219_cmd(uint8_t address, uint8_t data)
{
    // Prepare for generating a rising edge on the LOAD pin
    MAX72XX_PORT->ODR &= ~MAX72XX_LOAD_PIN;

    spi_send_blocking(address);
    spi_send_blocking(data);

    // Wait for the SPI transmission to complete
    while ((SPI->SR & SPI_FLAG_BSY));

    // Generate rising edge to latch command and data bytes
    MAX72XX_PORT->ODR |= MAX72XX_LOAD_PIN;
}

void max7219_set_digit(uint8_t digitRegister, uint8_t segments)
{
    // Map logical digit to actual hardware wiring
    const uint8_t mappedDigit = max7219_digitMap[digitRegister - 1];

    // Create a bitmask for the 1-indexed digit register
    const uint8_t digitMask = (1<<mappedDigit);

    // Set/clear the digit's corresponding bit in each segment byte
    for (uint8_t i = 0; i < kNumSegments; ++i) {
        if (segments & 0x01) {
            _segmentWiseData[i] |= digitMask;
        } else {
            _segmentWiseData[i] &= ~digitMask;
        }

        // Next segment
        segments >>= 1;
    }
}

void display_send_buffer()
{
    // Block interrupts during display update to avoid contention with the brightness update interrupt
    disableInterrupts();

    for (uint8_t i = 0; i < kNumSegments; ++i) {
        const uint8_t digitRegister = i + 1;
        max7219_cmd(digitRegister, _sendBuffer[i]);
    }

    enableInterrupts();
}

void display_set_digit_bcd(uint8_t digitRegister, uint8_t value)
{
    // Get segments that should be on
    uint8_t segments = max7219_bcdMap[value & 0xF];

    // Turn on the decimal point if the most-significant bit is set
    segments |= (value & 0x80);

    max7219_set_digit(digitRegister, segments);
}

void display_set_buffer_partial(uint8_t digitIndex, uint8_t value)
{
    // Manually split out tens and ones columns
    // This saves some code space over using division
    uint8_t tens = 0;

    while (value >= 10) {
        value -= 10;
        ++tens;
    }

    display_set_digit_bcd(digitIndex, tens);
    display_set_digit_bcd(digitIndex + 1, value);
}

void display_set_buffer(DateTime* now)
{
    display_set_buffer_partial(1, now->hour);
    display_set_buffer_partial(3, now->minute);
    display_set_buffer_partial(5, now->second);
}

void display_set_buffer_ascii(uint8_t digitIndex, const char* str)
{
    while (*str != NULL) {
        const char value = *str++;
        max7219_set_digit(digitIndex++, max7219_charMap[value - 'A']);
    }
}

/**
 * Set all digits on the display to a value with no illuminated segments
 */
void display_clear()
{
    for (uint8_t i = 0; i < kNumSegments; ++i) {
        _segmentWiseData[i] = 0x0;
    }
}

void display_swap_buffers()
{
    uint8_t* preparedData = _segmentWiseData;

    // Set the old output buffer as the new input buffer
    _segmentWiseData = _sendBuffer;

    // Disable interrupts while setting the output pointer
    // This is required as the LDW instruction takes 2 cycles.
    // The delay introduced for a conflicting interrupt is at worst around 0.25uS
    disableInterrupts();
    _sendBuffer = preparedData;
    enableInterrupts();
}

void display_overlay_ticker()
{
    static uint8_t waitIndicator = 0;

    // Clear segment on all digits
    _segmentWiseData[7] = 0x0;

    // Set segment on next phsyical digit
    _segmentWiseData[7] = (1<<max7219_digitMap[waitIndicator]);

    ++waitIndicator;
    if (waitIndicator == kNumDigits) {
        waitIndicator = 0;
    }
}

void display_error_code(uint8_t code)
{
    display_clear();

    // Display error code
    display_set_digit_bcd(1, 11 /* E */);
    max7219_set_digit(2, 0b01010000 /* r */);
    display_set_digit_bcd(4, code);

    display_swap_buffers();
    display_send_buffer();
}