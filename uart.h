#pragma once

#include <stm8s.h>
#include <stdint.h>

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
void uart1_receive_irq(void) __interrupt(ITC_IRQ_UART1_RX);