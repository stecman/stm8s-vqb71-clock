#pragma once

#include <stm8s.h>

inline void spi_send_blocking(uint8_t data)
{
    // Load data into TX register
    SPI->DR = data;

    // Wait for TX buffer empty flag
    while (!(SPI->SR & SPI_FLAG_TXE));
}