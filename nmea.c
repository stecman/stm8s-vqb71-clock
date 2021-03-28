#include "nmea.h"
#include "macros.h"

#include <stdbool.h>
#include <string.h>

// NMEA sentences are limited to 79 characters including the start '$' and end '\r\n'
#define NMEA_STRLEN_MAX 79

// RMC: "Recommended Minimum Data"

enum GPRMCField {
    GPRMC_SentenceType = 0,
    GPRMC_Timestamp, // UTC of position fix
    GPRMC_Validity, // Data status (A=ok, V=navigation receiver warning)
    GPRMC_Latitude, // Latitude of fix
    GPRMC_Latitude_NorthSouth, // N or S
    GPRMC_Longitude, // Longitude of fix
    GPRMC_Longitude_EastWest, // E or W
    GPRMC_SpeedInKnots, // Speed over ground in knots
    GPRMC_TrueCourse, // Track made good in degrees True
    GPRMC_DateStamp, // UT date
    GPRMC_Variation, // Magnetic variation degrees (Easterly var. subtracts from true course)
    GPRMC_Variation_EastWest, // E or W
};

struct RmcData {
    DateTime output;

    // Which field in the output is currently being written to
    uint8_t outputIndex;

    // Flag to indicate the decimal portion of time is being skipped
    bool hitTimeDecimal;

    // Flag to indicate the date/time field was non-empty
    // During start-up the GPS can return blank fields while it aquires a signal
    bool sawTimeFields;
};

// GSV: "GNSS Satellites in View"

enum GPGSVField {
    GPGSV_SentenceType = 0,
    GPGSV_MessageCount, // Number of messages, total number of GPGSV messages being output
    GPGSV_MessageIndex, // Number of this message
    GPGSV_SatelliteCount, // Satellites in view

    // These fields repeat 1-4 times per GSV message
    GPGSV_SatelliteID, // Satellite ID
    GPGSV_Elevation, // Elevation, range 0..90
    GPGSV_Azimuth, // Azimuth, range 0..359
    GPGSV_CarrierNoiseDensity, // C/N0, range 0..99, null when not tracking
};

struct GsvData {
    // Multiple message tracking
    uint8_t messageCount;
    uint8_t messageIndex;

    // Number of satellites in view ("space vehicles")
    uint8_t numVisible;

    // Number of satellites the receiver is tracking
    uint8_t numTracking;

    // Sum of carrier-to-noise-density ratio for all satellites being tracked
    // This is used to calculate an average for display, which isn't technically correct
    // but should be "good enough" as a gauge of signal strength for device placement.
    uint16_t cnoSum;
};

// State machine

enum NmeaReadState {
    kSearchStart,
    kReadType,
    kReadFields,
    kChecksumVerify,
    kDone,
    kParseError = 255,
};

struct NmeaReader {
    // Character limit for parsing the current sentence
    uint8_t remaining_length;

	// Pointer to the sentence definition we've matched against
    const struct SentenceType* matchedType;

    // Checksum calculated for the current NMEA sentence
    uint8_t calculatedChecksum;

    // Buffer to store field values temporarily for parsing
    char buffer[5];
    int8_t bufIndex;

    // State for sentence matching
    enum NmeaReadState state;
    uint8_t field;

    // Sentence-specific state
    struct RmcData rmc;
    struct GsvData gsv;
};

// Sentence definitions

struct SentenceType {
    const char* identifier;

    // Process a byte for this sentence type
    void (*onByteRecv)(char byte);

    // Called at the end of a sentence when checksum has been verified
    // The return value is passed back to the caller of nmea_parse
    GpsReadStatus (*onSentenceEnd)(void);

    struct {
        uint8_t from : 4;
        uint8_t to : 4;
    } fieldLoop;
};

// Value for SentenceType.fieldLoop if there is no loop in field processing
#define NO_FIELD_LOOP {0,0}

static void _parse_rmc(char byte);
static void _parse_gsv(char byte);
static GpsReadStatus _finish_rmc(void);
static GpsReadStatus _finish_gsv(void);

const struct SentenceType sentenceImpls[] = {
    {"RMC", _parse_rmc, _finish_rmc, NO_FIELD_LOOP},
    {"GSV", _parse_gsv, _finish_gsv, {GPGSV_CarrierNoiseDensity, GPGSV_SatelliteID}},
};


static struct NmeaReader _state;

/**
 * Reset the sentence parser to be ready for the next sentence
 * This is automatically called at the end of each sentence.
 */
static void _reset_parser(void)
{
    _state.remaining_length = NMEA_STRLEN_MAX,
    _state.matchedType = NULL;
    _state.calculatedChecksum = 0x0,
    _state.bufIndex = 0,
    _state.state = kSearchStart;
    _state.field = 0;

    // Reset RMC fields
    _state.rmc.outputIndex = 0;
    _state.rmc.hitTimeDecimal = false;
    _state.rmc.sawTimeFields = false;
}

/**
 * Reset the GSV parser state
 * This parser reads across multiple messages, so handles its own parser reset
 */
static void _reset_gsv_parser(void)
{
    _state.gsv.messageCount = 0;
    _state.gsv.messageIndex = 0;
}

void nmea_init(void)
{
    _reset_parser();
    _reset_gsv_parser();
}

/**
 * Find a sentence definition that matches that passed identified
 * Returns NULL if no match is found
 */
static const struct SentenceType* findSentenceType(const char* id)
{
    for (uint8_t i = 0; i < COUNT_OF(sentenceImpls); i++) {
        if (strncmp(id, sentenceImpls[i].identifier, 3) == 0) {
            return &sentenceImpls[i];
        }
    }

    return NULL;
}

/**
 * Convert a two character hex string to a byte
 *
 * Note this assumes the input is a two character array, not a null terminated string.
 */
static inline uint8_t hex2int(char* hexPair)
{
    uint8_t output = 0;

    for (uint8_t i = 0; i < 2; ++i) {

        // Shift result to make room for next nibble
        output <<= 4;

        // Get hex character
        const uint8_t hexChar = hexPair[i];

        // Transform hex character to the 4bit equivalent number
        if (hexChar >= '0' && hexChar <= '9') {
            output |= hexChar - '0';

        } else if (hexChar >= 'A' && hexChar <= 'F') {
            output |= hexChar - 'A' + 10;

        }
    }

    return output;
}

/**
 * Convert a two character numeric string to an 8-bit number
 *
 * This is a simplified implementation that saves 86 bytes over the stdlib version
 * by assuming str contains exactly two numeric characters (zero padded if one digit).
 */
static inline uint8_t _digit_pair_to_uint(char *str)
{
    uint8_t result = 0;

    result = (str[0] - '0') * 10;
    result += str[1] - '0';

    return result;
}

/**
 * Convert a digit character into an 8-bit number
 */
static inline uint8_t _digit_to_uint(char digit)
{
    return (uint8_t) digit - '0';
}

/**
 * Collect pairs of characters and convert them to an 8-bit unsigned number once buffered
 * Returns true when &output is updated
 */
static bool _read_two_digit(char byte, uint8_t* output)
{
    _state.buffer[_state.bufIndex] = byte;
    _state.bufIndex++;

    if (_state.bufIndex == 2) {
        _state.bufIndex = 0;
        *output = _digit_pair_to_uint(_state.buffer);

        return true;
    }

    return false;
}

const DateTime* nmea_get_time(void)
{
    return &(_state.rmc.output);
}

/**
 * Get the number of satellite vehicles visible
 */
const uint8_t nmea_get_sv(void)
{
    return _state.gsv.numVisible;
}

/**
 * Get the carrier signal to noise ratio
 */
const uint8_t nmea_get_tracking_sv(void)
{
    return _state.gsv.numTracking;
}

/**
 * Get the carrier signal to noise ratio
 */
const uint8_t nmea_get_cno(void)
{
    if (_state.gsv.numTracking) {
        return _state.gsv.cnoSum / _state.gsv.numTracking;
    } else {
        return 0;
    }
}

GpsReadStatus nmea_parse(char byte)
{
    // Sanity check sentence length
    if (--(_state.remaining_length) == 0) {
        // The max string length was hit, which means the sentence was longer than allowed
        return kGPS_BadFormat;
    }

    if (byte == '\n') {
        _reset_parser();
        return kGPS_Continue;
    }

    switch (_state.state) {
        case kSearchStart: {
            // Look for start character
            if (byte == '$') {
                _state.state = kReadType;
                return kGPS_Continue;
            }

            // Not the character we're looking for
            return kGPS_Continue;
        }

        case kReadType: {
            // Include sentence type in checksum
            _state.calculatedChecksum ^= byte;

			// Collect sentence identifier
            if (_state.bufIndex < 5) {
                _state.buffer[_state.bufIndex] = byte;
                ++_state.bufIndex;

                if (_state.bufIndex < 5) {
    				return kGPS_Continue;
                }
            }

            if (_state.buffer[0] == 'G' && _state.buffer[1] == 'P') {
                const struct SentenceType* match = findSentenceType(_state.buffer + 2);
                if (match != NULL) {
                    _state.matchedType = match;
                    _state.state = kReadFields;

                    return kGPS_Continue;
                }
            }

            // Saw a '$' but the sentence type match fell through
            // Ignore everything further in this message
            _state.state = kDone;

            return kGPS_NoMatch;
        }

        case kReadFields: {
            const struct SentenceType* type = _state.matchedType;

            // Asterisk marks the end of the data and start of the checksum
            if (byte == '*') {
                _state.state = kChecksumVerify;
                return kGPS_Continue;
            }

            // Calculate checksum across sentence contents
            _state.calculatedChecksum ^= byte;

            // Fields are delimited by commas
            if (byte == ',') {
                // Follow field loop if set and matched, otherwise increment
                if (_state.field != 0 && type->fieldLoop.from == _state.field) {
                    _state.field = type->fieldLoop.to;
                } else {
                    ++_state.field;
                }

                // Reset any buffering that was in progress
                _state.bufIndex = 0;

                return kGPS_Continue;
            }

            type->onByteRecv(byte);

            return kGPS_Continue;
        }

        case kChecksumVerify: {

            // Collect checksum
            _state.buffer[_state.bufIndex] = byte;
            _state.bufIndex++;

            if (_state.bufIndex < 2) {
                return kGPS_Continue;
            }

            const uint8_t receivedChecksum = hex2int(_state.buffer);

            // Done parsing this sentence
            _state.state = kDone;

            if (receivedChecksum != _state.calculatedChecksum) {
                return kGPS_InvalidChecksum;
            }

            // Let the sentence specific parser handle end of sentence
            return _state.matchedType->onSentenceEnd();
        }

        case kDone: {
            // Ignore all further bytes until the sentence ends and the parser resets
            return kGPS_Continue;
        }

        default:
            // Entered an unrecognised state: abort
            return kGPS_UnknownState;
    }
}

static void _parse_rmc(char byte)
{
    switch (_state.field) {
        case GPRMC_Timestamp: {

            // Skip the fractional part of the timestamp field as we don't use it
            // This isn't guaranteed to be present in every message
            if (_state.rmc.hitTimeDecimal || byte == '.') {
                _state.rmc.hitTimeDecimal = true;
                return;
            }

            // INTENTIONAL FALL THROUGH TO DATESTAMP
        }

        case GPRMC_DateStamp: {
            // Pointer to the datetime value to write a value into
            uint8_t* raw = (uint8_t*) &(_state.rmc.output);
            uint8_t* dest = &(raw[_state.rmc.outputIndex]);

            if (_read_two_digit(byte, dest)) {
                ++_state.rmc.outputIndex;
                _state.rmc.sawTimeFields = true;
            }

            return;
        }

        default:
            // Skip other fields
            return;
    }
}

static GpsReadStatus _finish_rmc(void)
{
    if (_state.rmc.sawTimeFields) {
        return kGPS_RMC_TimeUpdated;
    } else {
        return kGPS_RMC_NoSignal;
    }
}

static void _parse_gsv(char byte)
{
    switch (_state.field) {
        case GPGSV_MessageCount: {
            // Initialise values for a new series of GSV messages
            if (_state.gsv.messageCount == 0) {
                _state.gsv.numTracking = 0;
                _state.gsv.cnoSum = 0;
            }

            _state.gsv.messageCount = _digit_to_uint(byte);
            return;
        }

        case GPGSV_MessageIndex: {
            _state.gsv.messageIndex = _digit_to_uint(byte);
            return;
        }

        case GPGSV_SatelliteCount: {
            _read_two_digit(byte, &(_state.gsv.numVisible));
            return;
        }

        case GPGSV_CarrierNoiseDensity: {
            uint8_t cno = 0;

            if (_read_two_digit(byte, &cno)) {
                _state.gsv.cnoSum += cno;
                _state.gsv.numTracking++;
            }

            return;
        }

        default:
            // Skip other fields
            return;
    }
}

static GpsReadStatus _finish_gsv(void)
{
    if (_state.gsv.messageCount == _state.gsv.messageIndex) {
        _reset_gsv_parser();
        return kGPS_GSV_Updated;
    } else {
        return kGPS_Continue;
    }
}