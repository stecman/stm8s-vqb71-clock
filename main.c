#include <stm8s.h>
#include <stm8s_uart1.h>

#include "delay.h"
#include "display.h"
#include "macros.h"
#include "nmea.h"
#include "pindefs.h"
#include "uart.h"
#include "ubxgps.h"

#include <stdbool.h>
#include <stddef.h>

static DateTime _currentTime;
static volatile int8_t _timezoneOffset = 13;
static volatile bool _timeNeedsIncrement = false;
static volatile bool _hasSignal = false;

static volatile uint16_t _ambientBrightnessSample;
static volatile bool _hasBrightnessSample = false;

/**
 * Add a value to a checksum (8-Bit Fletcher Algorithm)
 * The initial checksum value must be {0,0}
 */
static inline void ubx_update_checksum(uint8_t* checksum, uint8_t value)
{
	checksum[0] += value;
	checksum[1] += checksum[0];
}

static void ubx_update_checksum_multi(uint8_t* checksum, uint8_t* data, uint16_t length)
{
	for (uint16_t i = 0; i < length; ++i) {
		ubx_update_checksum(checksum, data[i]);
	}
}

enum UbxResponse {
    kUbxNack = 0, // Matches the NACK message ID
    kUbxAck = 1,  // Matches the ACK message ID
    kUbxResponseTimeout = 0x55,
    kUbxBadResponse = 0xBD
};

static enum UbxResponse ubx_send(uint8_t msgClass, uint8_t msgId, uint8_t* data, uint16_t length)
{
    // Send packet to receiver
    {
    	uint8_t header[6] = {
    		0xB5, 0x62, // Every message starts with these sync characters
    		msgClass,
    		msgId,
    		(uint8_t) (length & 0xFF), // Payload length as little-endian (LSB first)
    		(uint8_t) (length >> 8),
    	};

    	uint8_t checksum[2] = {0, 0};

    	// Checksum includes the payload and the header minus its two fixed bytes
    	ubx_update_checksum_multi(checksum, (uint8_t*)(&header) + 2, sizeof(header) - 2);
    	ubx_update_checksum_multi(checksum, data, length);

    	// Send the message over serial
    	uart_send_stream_blocking(header, sizeof(header));
    	uart_send_stream_blocking(data, length);
    	uart_send_stream_blocking(checksum, sizeof(checksum));
    }

    // Look for receiver response
    // TODO: make this a more generic UBX packet reading routine that verifies checksum
    // TODO: handle response timeout. Currently this blocks forever if the GPS doesn't respond
    {
        const uint8_t response_header[] = {0xB5, 0x62, 0x05};

        uint8_t searchIndex = 0;
        enum UbxResponse response = kUbxBadResponse;

        // Wait for the ACK/NACK response header
        while (searchIndex < sizeof(response_header)) {
            const char byte = uart_read_byte();

            if (byte == response_header[searchIndex]) {
                ++searchIndex;
            }
        }

        // Read message ID as response
        response = uart_read_byte();

        // Discard packet length as we're not using it here
        uart_read_byte();
        uart_read_byte();

        if (uart_read_byte() == msgClass &&
            uart_read_byte() == msgId) {
            return response;
        } else {
            return kUbxBadResponse;
        }
    }
}

// Account for 39.5us delay between top-of-second and complete display update
#define kTimepulseOffsetNs 39500
const uint8_t gps_cfg_tp5_data[] = {
    UBX_VALUE_U8(0), // Timepulse selection (only one available on NEO-6M)
    UBX_VALUE_U8(0), // Reserved 0
    UBX_VALUE_U16(0), // Reserved 1
    UBX_VALUE_S16(50), // Antenna cable delay (nS)
    UBX_VALUE_S16(0), // RF group delay (readonly)
    UBX_VALUE_U32(1), // Freqency of time pulse in Hz
    UBX_VALUE_U32(1), // Freqency of time pulse in Hz when locked to GPS time
    UBX_VALUE_U32(1000), // Length of time pulse in uS
    UBX_VALUE_U32(10000), // Length of time pulse in uS when locked to GPS time
    UBX_VALUE_S32(kTimepulseOffsetNs), // User configurable timepuse delay (nS)
    UBX_VALUE_U32(0xFF), // All flags set
};

const uint8_t gps_cfg_nav5_data[] = {
    UBX_VALUE_U16(0b00111111), // Mask selecting settings to apply
    UBX_VALUE_U8(2), // "Stationary" dynamic platform model
    UBX_VALUE_U8(3), // Get either a 3D or 2D fix
    UBX_VALUE_U32(0), // fixed altitude for 2D
    UBX_VALUE_U32(0), // fixed altitude variance for 2D
    UBX_VALUE_U8(20), // minimum elevation is 20 degrees...
    UBX_VALUE_U8(180), // maximum time to perform dead reckoning in case of GPS signal loss (s)
    UBX_VALUE_U16(100), // position DoP mask is 10.0...
    UBX_VALUE_U16(100), // time DoP mask is 10.0...
    UBX_VALUE_U16(100), // position accuracy mask in meters...
    UBX_VALUE_U16(100), // time accuracy mask in meters...
    UBX_VALUE_U8(0), // static hold threshold is 0 cm/s...
    UBX_VALUE_U8(60), // dynamic GNSS timeout is 60 seconds (not used)...
    UBX_VALUE_U32(0), // Reserved
    UBX_VALUE_U32(0), // Reserved
    UBX_VALUE_U32(0), // Reserved
};

// ID byte of NMEA messages to enable
// All messages share the same class byte of 0xF0
const uint8_t gps_enableMessages[] = {
    0x03, // GSV (GNSS Satellites in View)
    0x04, // RMC (Recommended Minimum data)
};

// ID byte of NMEA messages to disable
// All messages share the same class byte of 0xF0
const uint8_t gps_disableMessages[] = {
    0x0A, // DTM (Datum Reference)
    0x09, // GBS (GNSS Satellite Fault Detection)
    0x00, // GGA (Global positioning system fix data)
    0x01, // GLL (Latitude and longitude, with time of position fix and status)
    0x40, // GPQ (Poll message)
    0x06, // GRS (GNSS Range Residuals)
    0x02, // GSA (GNSS DOP and Active Satellites)
    0x07, // GST (GNSS Pseudo Range Error Statistics)
    0x0E, // THS (True Heading and Status)
    0x41, // TXT (Text Transmission)
    0x05, // VTG (Course over ground and Ground speed)
};

void gps_set_nmea_send_rate(const uint8_t rate, const uint8_t* messageIds, const uint8_t length)
{
    // Buffer for message rate configuration
    uint8_t cfg_msg_data[3] = {
        0xF0, // Message class
        0, // Message ID placeholder
        rate, // Send rate
    };

    // Set send rate to zero to disable messages
    for (uint8_t i = 0; i < length; ++i) {
        cfg_msg_data[1] = messageIds[i];
        ubx_send(0x06, 0x01, cfg_msg_data, sizeof(cfg_msg_data));
    }
}

// Wait for GPS start-up
void gps_init()
{
    // Configure time-pulse
    ubx_send(0x06, 0x31, gps_cfg_tp5_data, sizeof(gps_cfg_tp5_data));

    // Configure stationary mode
    ubx_send(0x06, 0x24, gps_cfg_nav5_data, sizeof(gps_cfg_nav5_data));

    // Enable NMEA messages we want to use and disable others
    gps_set_nmea_send_rate(1, gps_enableMessages, sizeof(gps_enableMessages));
    gps_set_nmea_send_rate(0, gps_disableMessages, sizeof(gps_disableMessages));
}



/**
 * Update the passed time to the current timezone offset
 */
static void apply_timezone_offset(DateTime* now)
{
    // Adjust hour for timezone
    int8_t hour = now->hour;
    hour += _timezoneOffset;

    if (hour > 23) {
        hour -= 24;
    } else if (hour < 0) {
        hour += 24;
    }

    now->hour = hour;
}

static inline uint16_t read_adc_buffer()
{
    // Load ADC reading (least-significant byte must be read first)
    uint16_t result = ADC1->DRL;
    result |= (ADC1->DRH << 8);

    return result;
}

void display_adjust_brightness(const uint16_t reading)
{
    // State to obtain an average of LDR readings
    // The size of this array should  be a power of two to make division simpler
    static uint16_t averageBuffer[16];
    static uint8_t writeIndex = 0;
    static uint16_t runningTotal = 0;

    // Adjust running total with the new value
    runningTotal -= averageBuffer[writeIndex];
    runningTotal += reading;

    // Append new reading
    averageBuffer[writeIndex] = reading;
    writeIndex = (writeIndex + 1) % COUNT_OF(averageBuffer);

    const uint16_t average = runningTotal/COUNT_OF(averageBuffer);

    // Scale the 1024 ADC values to fit in the 16 brightness levels of the MAX72XX
    max7219_cmd(0x0A, average / 64);
}

void increment_time(DateTime* tim)
{
    ++tim->second;

    if (tim->second == 60) {
        tim->second = 0;
        ++tim->minute;
    }

    if (tim->minute == 60) {
        tim->minute = 0;
        ++tim->hour;
    }

    if (tim->hour == 24) {
        tim->hour = 0;
    }
}

int main()
{
    // Configure the clock for maximum speed on the 16MHz HSI oscillator
    // At startup the clock output is divided by 8
    // CLK->CKDIVR = (CLK_PRESCALER_CPUDIV8 & CLK_CKDIVR_CPUDIV);
    CLK->CKDIVR = 0x0;

    // Delay to prevent partial initialisation when programming (reset is only blipped low by the programmer)
    _delay_ms(10);

    // Configure test points as outputs
    TEST_PIN_PORT->DDR = TEST_PIN_1 | TEST_PIN_2; // Output mode
    TEST_PIN_PORT->CR1 = TEST_PIN_1 | TEST_PIN_2; // Push-pull mode
    TEST_PIN_PORT->ODR = TEST_PIN_1 | TEST_PIN_2; // Pull high

    // Interrupt outputs from GPS as inputs
    EXTI->CR1 |= 0x04; // Rising edge triggers interrupt
    GPS_PORT->DDR &= ~GPS_PIN_TIMEPULSE; // Input mode
    GPS_PORT->CR1 |= GPS_PIN_TIMEPULSE;  // Enable internal pull-up
    GPS_PORT->CR2 |= GPS_PIN_TIMEPULSE;  // Interrupt enabled

    // Buttons as input
    EXTI->CR1 |= 0x02; // Falling edge triggers interrupt
    BUTTON_PORT->DDR &= ~(BUTTON_PIN_DST | BUTTON_PIN_TIMEZONE); // Input mode
    BUTTON_PORT->CR1 |= BUTTON_PIN_DST | BUTTON_PIN_TIMEZONE; // Enable internal pull-up
    BUTTON_PORT->CR2 |= BUTTON_PIN_DST | BUTTON_PIN_TIMEZONE;  // Interrupt enabled

    // MAX7219  chip select as output
    MAX72XX_PORT->DDR |= MAX72XX_LOAD_PIN; // Output mode
    MAX72XX_PORT->CR1 |= MAX72XX_LOAD_PIN; // Push-pull mode
    MAX72XX_PORT->CR2 |= MAX72XX_LOAD_PIN; // Speed up to 10Mhz
    MAX72XX_PORT->ODR |= MAX72XX_LOAD_PIN; // Active low: initially set high

    // Set SPI output pins to high-speed mode per the datasheet:
    //
    // > When using the SPI in High-speed mode, the I/Os where SPI outputs are connected should
    // > be programmed as fast slope outputs in order to be able to reach the expected bus speed.
    //
    SPIOUT_PORT->DDR |= SPIOUT_MOSI_PIN | SPIOUT_CLK_PIN; // Output
    SPIOUT_PORT->CR2 |= SPIOUT_MOSI_PIN | SPIOUT_CLK_PIN; // High-speed

    // Enable SPI for MAX7219 display driver
    SPI_Init(SPI_FIRSTBIT_MSB,
             SPI_BAUDRATEPRESCALER_2,
             SPI_MODE_MASTER,
             SPI_CLOCKPOLARITY_LOW,
             SPI_CLOCKPHASE_1EDGE,
             SPI_DATADIRECTION_1LINE_TX,
             SPI_NSS_SOFT, 0);

    SPI_Cmd(ENABLE);

    // Enable UART for GPS comms
    UART1_Init(9600, // Baud rate
               UART1_WORDLENGTH_8D,
               UART1_STOPBITS_1,
               UART1_PARITY_NO,
               UART1_SYNCMODE_CLOCK_DISABLE,
               UART1_MODE_TXRX_ENABLE);

    UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
    UART1_Cmd(ENABLE);


    // Enable ADC for ambient light sensing
    // Conversion is triggered by timer 1's TRGO event
    ADC1->CSR = ADC1_CSR_EOCIE | // Enable interrupt at end of conversion
                ADC1_CHANNEL_4; // Convert on ADC channel 4 (pin D3)

    ADC1->CR2 = ADC1_CR2_ALIGN | // Place LSB in lower register
                ADC1_CR2_EXTTRIG; // Start conversion on external event (TIM1 TRGO event)

    ADC1->CR1 = ADC1_PRESSEL_FCPU_D18 | // ADC @ fcpu/18
                ADC1_CR1_ADON; // Power on the ADC


    // Configure TIM1 to trigger ADC conversion automatically
    const uint16_t tim1_prescaler = 16000; // Prescale the 16MHz system clock to a 1ms tick
    TIM1->PSCRH = (tim1_prescaler >> 8);
    TIM1->PSCRL = (tim1_prescaler & 0xFF);

    const uint16_t tim1_auto_reload = 69; // Number of milliseconds to count to
    TIM1->ARRH = (tim1_auto_reload >> 8);
    TIM1->ARRL = (tim1_auto_reload & 0xFF);

    const uint16_t tim1_compare_reg1 = 1; // Create a 1ms OC1REF pulse (PWM1 mode)
    TIM1->CCR1H = (tim1_compare_reg1 >> 8);
    TIM1->CCR1L = (tim1_compare_reg1 & 0xFF);

    // Use capture-compare channel 1 to trigger ADC conversions
    // This doesn't have any affect on pin outputs as TIM1_CCER1_CC1E and TIM1_BKR_MOE are not set
    TIM1->CCMR1 = TIM1_OCMODE_PWM1; // Make OC1REF high when counter is less than CCR1 and low when higher
    TIM1->EGR = TIM1_EGR_CC1G; // Enable compare register 1 event
    TIM1->CR2 = TIM1_TRGOSOURCE_OC1REF; // Enable TRGO event on compare match

    TIM1->EGR |= TIM1_EGR_UG; // Generate an update event to register new settings

    TIM1->CR1 = TIM1_CR1_CEN; // Enable the counter

    enableInterrupts();

    max7219_init();
    gps_init();
    nmea_init();

    while (true) {

        // Prepare the value to be sent at the next time pulse from the GPS if needed
        // This is intentionally a bitwise AND
        if (_hasSignal & _timeNeedsIncrement) {
            _timeNeedsIncrement = false;

            increment_time(&_currentTime);

            DateTime displayTime;
            displayTime = _currentTime;
            apply_timezone_offset(&displayTime);
            display_set_buffer(&displayTime);

            display_swap_buffers();
        }

        if (_hasBrightnessSample) {
            _hasBrightnessSample = false;
            display_adjust_brightness(_ambientBrightnessSample);
        }

        if (uart_has_byte()) {
            const GpsReadStatus status = nmea_parse(uart_read_byte());

            switch (status) {
                case kGPS_RMC_TimeUpdated: {
                    const DateTime* lastTick = nmea_get_time();

                    _currentTime = (*lastTick);
                    _timeNeedsIncrement = true;
                    _hasSignal = true;
                    break;
                }

                case kGPS_RMC_NoSignal:
                    // Walk the decimal point across the display to indicate activity
                    _hasSignal = false;
                    break;

                case kGPS_GSV_Updated:
                    if (!_hasSignal) {
                        display_set_buffer_partial(1, nmea_get_sv());
                        display_set_buffer_partial(3, nmea_get_tracking_sv());
                        display_set_buffer_partial(5, nmea_get_cno());
                        display_overlay_ticker();
                        display_swap_buffers();
                        display_send_buffer();
                    }
                    break;


                case kGPS_InvalidChecksum:
                    display_error_code(1);
                    break;

                case kGPS_BadFormat:
                    // This state is returned if the UART line isn't pulled high (ie. GPS unplugged)
                    display_error_code(2);
                    break;

                case kGPS_UnknownState:
                    // This state is returned if the UART line isn't pulled high (ie. GPS unplugged)
                    display_error_code(3);
                    break;
            }
        }
    }
}

INTERRUPT_HANDLER(gps_irq, ITC_IRQ_PORTB)
{
    if (_hasSignal) {
        display_send_buffer();
        _timeNeedsIncrement = true;
    }
}

INTERRUPT_HANDLER(button_irq, ITC_IRQ_PORTA)
{
    _delay_ms(5);

    if (BUTTON_PORT->IDR & ~BUTTON_PIN_DST) {
        if (_timezoneOffset == 13) {
            _timezoneOffset = -12;
        } else {
            _timezoneOffset++;
        }
    }
}

INTERRUPT_HANDLER(adc_irq, ITC_IRQ_ADC1)
{
    // Clear the end of conversion bit so this interrupt can fire again
    ADC1->CSR &= ~ADC1_CSR_EOC;

    _ambientBrightnessSample = read_adc_buffer();
    _hasBrightnessSample = true;
}