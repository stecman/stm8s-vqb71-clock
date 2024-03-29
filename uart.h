#pragma once

#include <stm8s.h>

/**
 * Set up the UART peripheral
 * This must be called before other uart_* functions
 */
void uart_init(void);

/**
 * Return true if a byte is ready in the internal buffer
 * @see uart_read_byte to retrieve any buffered bytes
 */
bool uart_has_byte(void);

/**
 * Read one byte from the internal UART buffer
 * If no bytes are available, this blocks until the next UART receive interrupt
 */
char uart_read_byte(void);

/**
 * Transmit one byte
 * Blocks until the transmit buffer can be written
 */
void uart_send_blocking(uint8_t byte);

/**
 * Transmit an array of bytes
 * Blocks between each byte. Returns once the last byte has been sent
 */
void uart_send_stream_blocking(uint8_t* bytes, uint8_t length);

/**
 * Interrupt handler for UART1 receive
 */
INTERRUPT_HANDLER(uart1_receive_irq, ITC_IRQ_UART1_RX);