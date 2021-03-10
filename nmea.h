#pragma once

#include <stdint.h>

typedef struct DateTime {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} DateTime;

typedef enum GpsReadStatus {
    // State machine is still parsing a sentence
    kGPS_Continue = 0,

    // Partial sentence or unknown sentence type
    kGPS_NoMatch,

    // A supported sentence was parsed, but the checksum verification failed
    kGPS_InvalidChecksum,

    // The sentence had too many characters or fields and could not be parsed
    kGPS_BadFormat,

    // The parser state-machine went into an undefined state
    kGPS_UnknownState,


    // GPS date and time has been updated from sentence data
    kGPS_RMC_TimeUpdated,

    // RMC sentence matched, but it had no date/time information
    kGPS_RMC_NoSignal,


    // Satellite reception information updated from sentences
    kGPS_GSV_Updated,

} GpsReadStatus;

/**
 * Initialise the internal state of the NMEA sentence parser
 */
void nmea_init(void);

/**
 * Feed the sentence parser a byte
 */
GpsReadStatus nmea_parse(char byte);

/**
 * Get the last read date & time
 * This is only valid after a kGps_RMC_TimeUpdated parser status is returned
 */
const DateTime* nmea_get_time(void);

/**
 * Get the number of satellite vehicles visible
 */
const uint8_t nmea_get_sv(void);

/**
 * Get the number of stellite vehicles that are being tracked by the receiver
 */
const uint8_t nmea_get_tracking_sv(void);

/**
 * Get the carrier signal to noise ratio
 */
const uint8_t nmea_get_cno(void);