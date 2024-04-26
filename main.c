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

enum DisplayState {
    kDisplayState_StartupAnimation,
    kDisplayState_Time,
    kDisplayState_TimeRefresh,
    kDisplayState_TimezoneChange,
    kDisplayState_TimezoneDisplay,
    kDisplayState_Wait,
};

enum ButtonState {
    kButtonState_Unpressed,
    kButtonState_Debounce,
    kButtonState_ShortPress,
    kButtonState_LongPress,
};

struct Button {
    // Pin registers
    GPIO_TypeDef* port;
    uint8_t pinMask;

    // Current state machine mode of the button
    enum ButtonState state;

    // Next systick value that the button cares about
    uint16_t nextTick;
};

const uint8_t kButtonDebounceMs = 5;
const uint16_t kButtonLongPressMs = 1000;

// Time display state
static DateTime _currentTime;
static int8_t _timezoneOffset = 12;
static bool _timeNeedsIncrement = false;
static bool _timeNeedsDisplay = false;
static bool _hasSignal = false;

// Display state machine
static volatile enum DisplayState _displayState = kDisplayState_StartupAnimation;
static uint16_t _displayWaitTick;
static enum DisplayState _displayWaitNextState;

// Button state
static struct Button _timezoneBtn;
static struct Button _dstBtn;

// Brightness / ADC reading state
static volatile uint16_t _ambientBrightnessSample;
static volatile bool _hasBrightnessSample = false;


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

void auto_brightness_init(void)
{
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
}

void systick_init(void)
{
    // Use TIM2 to get a tick every 1.024ms
    // Only TIM1 can generate TRGO events, so We can't substitute that in here.
    // The accuracy of this isn't terribly important as it's for timing human interactions
    TIM2->PSCR = TIM2_PRESCALER_16384;
    TIM2->EGR |= TIM2_EGR_UG; // Generate an update event to register new settings
    TIM2->CR1 = TIM2_CR1_CEN; // Enable the counter
}

uint16_t systick_get(void)
{
    uint16_t value = TIM2->CNTRL;
    value |= (TIM2->CNTRH << 8);

    return value;
}


static void button_init(struct Button* btn, GPIO_TypeDef* port, uint8_t pinMask)
{
    // Configure GPIO
    port->DDR &= ~(pinMask); // Input mode
    port->CR1 |= pinMask; // Enable internal pull-up

    // Initialise struct
    btn->port = port;
    btn->pinMask = pinMask;
    btn->nextTick = 0;
    btn->state = kButtonState_Unpressed;
}

static void button_update(struct Button* btn)
{
    switch (btn->state) {
        case kButtonState_Unpressed:
            // Wait fo the button to be pressed
            if ((btn->port->IDR & btn->pinMask) == 0) {
                btn->state = kButtonState_Debounce;
                btn->nextTick = systick_get() + kButtonDebounceMs;
            }

            break;

        case kButtonState_Debounce:
            // Check button state after debounce delay
            if (systick_get() >= btn->nextTick) {
                if ((btn->port->IDR & btn->pinMask) == 0) {
                    btn->state = kButtonState_ShortPress;
                    btn->nextTick = systick_get() + kButtonLongPressMs;
                } else {
                    btn->state = kButtonState_Unpressed;
                }
            }

            break;

        case kButtonState_ShortPress:
            if (systick_get() >= btn->nextTick) {
                btn->state = kButtonState_LongPress;
            }

            // Intentional fall-through

        case kButtonState_LongPress:
            if ((btn->port->IDR & btn->pinMask) != 0) {
                btn->state = kButtonState_Debounce;
                btn->nextTick = systick_get() + kButtonDebounceMs;
            }

            break;
    }
}

static bool button_pressed(struct Button* btn)
{
    return btn->state & (kButtonState_ShortPress | kButtonState_LongPress);
}


static void wait(uint16_t milliseconds, enum DisplayState nextState)
{
    // Configure the wait
    _displayWaitTick = systick_get() + milliseconds;
    _displayWaitNextState = nextState;

    // Enter wait state
    _displayState = kDisplayState_Wait;
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

    uart_init();
    systick_init();

    // Depends on SPI
    max7219_init();
    auto_brightness_init();

    // Depends on UART
    gps_init();
    nmea_init();

    button_init(&_timezoneBtn, BUTTON_PORT, BUTTON_PIN_TIMEZONE);
    button_init(&_dstBtn, BUTTON_PORT, BUTTON_PIN_DST);

    enableInterrupts();

    while (true) {

        // Prepare the value to be sent at the next time pulse from the GPS if needed
        if (_timeNeedsIncrement) {
            _timeNeedsIncrement = false;
            _timeNeedsDisplay = true;
            increment_time(&_currentTime);
        }

        // Always update display brightness from interrupt flag
        if (_hasBrightnessSample) {
            _hasBrightnessSample = false;
            display_adjust_brightness(_ambientBrightnessSample);
        }

        // Update button state machines
        button_update(&_timezoneBtn);
        button_update(&_dstBtn);

        if (_hasBrightnessSample) {
            _hasBrightnessSample = false;
            display_adjust_brightness(_ambientBrightnessSample);
        }

        switch (_displayState) {
            case kDisplayState_StartupAnimation: {
                static uint8_t frame = 0;
                static uint16_t nextFrame = 0;
                static uint8_t data = 0;

                if (systick_get() >= nextFrame) {
                    nextFrame = systick_get() + 70;


                    data ^= (1 << (frame % kNumSegments));

                    // Set segments on all digits (1-indexed)
                    for (uint8_t i = 1; i < kNumDigits + 1; ++i) {
                        max7219_set_digit(i, data);
                    }

                    display_swap_buffers();
                    display_send_buffer();

                    ++frame;
                    if (frame == kNumSegments * 4) {
                        _displayState = kDisplayState_Time;
                    }
                }

                break;
            }

            case kDisplayState_TimeRefresh: {
                _timeNeedsDisplay = true;
                _displayState = kDisplayState_Time;

                // Intentional fall-through
            }

            case kDisplayState_Time: {
                if (_hasSignal) {
                    if (_timeNeedsDisplay) {
                        DateTime displayTime;

                        displayTime = _currentTime;
                        apply_timezone_offset(&displayTime);
                        display_set_buffer(&displayTime);
                        display_swap_buffers();
                    }
                }

                if (button_pressed(&_timezoneBtn)) {
                    _displayState = kDisplayState_TimezoneDisplay;
                }

                break;
            }

            case kDisplayState_TimezoneChange: {
                if (button_pressed(&_timezoneBtn)) {
                    _timezoneOffset++;
                    if (_timezoneOffset > 13) {
                        _timezoneOffset = -12;
                    }
                }

                // Intentional fall-through
            }

            case kDisplayState_TimezoneDisplay: {
                uint8_t bcdSign;
                uint8_t tzAbsolute;

                if (_timezoneOffset < 0) {
                    bcdSign = 0xA; // "-""
                    tzAbsolute = _timezoneOffset * -1;
                } else {
                    bcdSign = 0xE; // "P"
                    tzAbsolute = _timezoneOffset;
                }

                display_clear();
                display_set_digit_bcd(2, bcdSign);
                display_set_buffer_partial(3, tzAbsolute);
                display_swap_buffers();
                display_send_buffer();

                if (!button_pressed(&_timezoneBtn)) {
                    wait(800, kDisplayState_TimeRefresh);
                } else {
                    wait(600, kDisplayState_TimezoneChange);
                }

                break;
            }

            case kDisplayState_Wait: {
                if (systick_get() >= _displayWaitTick) {
                    _displayState = _displayWaitNextState;
                }

                break;
            }
        }

        // Don't do anything else while the display test is running
        // TODO: Allow this to run in parallel with GPS reads without messing up the display
        if (_displayState == kDisplayState_StartupAnimation) {
            continue;
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

INTERRUPT_HANDLER(adc_irq, ITC_IRQ_ADC1)
{
    // Clear the end of conversion bit so this interrupt can fire again
    ADC1->CSR &= ~ADC1_CSR_EOC;

    _ambientBrightnessSample = read_adc_buffer();
    _hasBrightnessSample = true;
}