#pragma once

#define TEST_PIN_PORT GPIOC
#define TEST_PIN_1 (1<<3)
#define TEST_PIN_2 (1<<4)

#define LDR_PORT GPIOD
#define LDR_PIN (1<<3)

#define GPS_PORT GPIOB
#define GPS_PIN_TIMEPULSE (1<<4)
#define GPS_PIN_EXTINT (1<<5)

#define BUTTON_PORT GPIOA
#define BUTTON_PIN_TIMEZONE (1<<2)
#define BUTTON_PIN_DST (1<<3)

#define MAX72XX_PORT GPIOD
#define MAX72XX_LOAD_PIN (1<<2)

#define SPIOUT_PORT GPIOC
#define SPIOUT_CLK_PIN (1<<5)
#define SPIOUT_MOSI_PIN (1<<6)