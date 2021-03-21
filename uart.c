#include "uart.h"

#include "circbuf.h"

volatile static CircBuf _uartBuffer;

char uart_read_byte(void)
{
    // Block until a character is available
    while (circbuf_is_empty(&_uartBuffer));

    return circbuf_pop(&_uartBuffer);
}

void uart_send_blocking(uint8_t byte)
{
    // Wait for the last transmission to complete
    while ( (UART1->SR & UART1_SR_TC) == 0 );

    // Put the byte in the TX buffer
    UART1->DR = byte;
}

void uart_send_stream_blocking(uint8_t* bytes, uint8_t length)
{
    while (length > 0) {
        uart_send_blocking(*bytes);

        --length;
        ++bytes;
    }
}

void uart1_receive_irq(void) __interrupt(ITC_IRQ_UART1_RX)
{
    const uint8_t byte = ((uint8_t) UART1->DR);

    circbuf_append(&_uartBuffer, byte);
}