#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "test_nmea.h"
#include "test_utils.h"

#include "../nmea.h"

/**
 * Map of status numbers to printable names
 */
static char* statusToString[] = {
    "kGPS_Continue",
    "kGPS_NoMatch",
    "kGPS_InvalidChecksum",
    "kGPS_BadFormat",
    "kGPS_UnknownState",
    "kGPS_RMC_TimeUpdated",
    "kGPS_RMC_NoSignal",
    "kGPS_GSV_Updated",
};

void assert_equal_datetime(DateTime actual, DateTime expected)
{
    for (int i = 0; i < sizeof(DateTime); i++) {
        // Compare bytes in DateTime structs
        if ( ((uint8_t*)&actual)[i] != ((uint8_t*)&expected)[i] ) {
            munit_errorf(
                "Expected '%02d:%02d:%02d %02d/%02d/%02d'; saw '%02d:%02d:%02d %02d/%02d/%02d'",
                expected.hour,
                expected.minute,
                expected.second,
                expected.day,
                expected.month,
                expected.year,
                actual.hour,
                actual.minute,
                actual.second,
                actual.day,
                actual.month,
                actual.year
                );
        }
    }
}

void assert_equal_gps_status(GpsReadStatus actual, GpsReadStatus expected)
{
    if (actual != expected) {
        munit_errorf(
            "Expected %s; saw %s",
            statusToString[expected],
            statusToString[actual]
        );
    }
}

/**
 * Parse a sentence until the parser returns a status other than "continue"
 */
GpsReadStatus run_nmea_parse(const char* sentence)
{
    GpsReadStatus status;

    nmea_init();

    for (int i = 0; sentence[i]; i++) {
        status = nmea_parse(sentence[i]);

        //printf("%c -> %s\n", sentence[i], statusToString[status]);

        if (status != kGPS_Continue) {
            break;
        }
    }

    return status;
}

/**
 * Test parsing a valid RMC sentence
 */
void run_nmea_rmc_parse_test(const char* sentence, DateTime expectedResult)
{
    GpsReadStatus status = run_nmea_parse(sentence);
    assert_equal_gps_status(status, kGPS_RMC_TimeUpdated);

    const DateTime* result = nmea_get_time();
    assert_equal_datetime(*result, expectedResult);
}

MunitResult test_valid_rmc_sentence_1(const MunitParameter params[], void* user_data_or_fixture)
{
    const char sentence[] = "$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62\r\n";
    const DateTime expected = {
        .hour = 8,
        .minute = 18,
        .second = 36,
        .day = 13,
        .month = 9,
        .year = 98
    };

    run_nmea_rmc_parse_test(sentence, expected);

    return MUNIT_OK;
}

MunitResult test_valid_rmc_sentence_2(const MunitParameter params[], void* user_data_or_fixture)
{
    const char sentence[] = "$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70\r\n";
    const DateTime expected = {
        .hour = 22,
        .minute = 5,
        .second = 16,
        .day = 13,
        .month = 6,
        .year = 94
    };

    run_nmea_rmc_parse_test(sentence, expected);

    return MUNIT_OK;
}

// Decode valid RMC sentence with an empty time field
MunitResult test_valid_rmc_with_empty_time(const MunitParameter params[], void* user_data_or_fixture)
{
    const char sentence[] = "$GPRMC,091502.00,V,,,,,,,040219,,,N*7C\r\n";
    const DateTime expected = {
        .hour = 9,
        .minute = 15,
        .second = 2,
        .day = 4,
        .month = 2,
        .year = 19
    };

    run_nmea_rmc_parse_test(sentence, expected);

    return MUNIT_OK;
}

// Decode valid RMC sentence with an empty time field
MunitResult test_valid_gsv(const MunitParameter params[], void* user_data_or_fixture)
{
    GpsReadStatus status = run_nmea_parse(
        "$GPGSV,3,1,10,23,38,230,44,29,71,156,47,07,29,116,41,08,09,081,36*7F\r\n"
        "$GPGSV,3,2,10,10,07,189,,05,05,220,,09,34,274,42,18,25,309,44*72\r\n"
        "$GPGSV,3,3,10,26,82,187,47,28,43,056,46*77\r\n"
    );

    assert_equal_gps_status(status, kGPS_GSV_Updated);
    munit_assert_uint8(nmea_get_sv(), ==, 10);
    munit_assert_uint8(nmea_get_tracking_sv(), ==, 8);
    munit_assert_uint8(nmea_get_cno(), ==, 43);

    return MUNIT_OK;
}

// Decode a valid stream of sentences
// This is tested to ensure sequential strings are parsed individually
MunitResult test_valid_sentence_stream(const MunitParameter params[], void* user_data_or_fixture)
{
    const char sentence[] = "$GPRMC,105445.00,V,,,,,,,040219,,,N*72\r\n$GPVTG,,,,,,,,,N*30\r\n$GPGGA,105445.00,,,,,0,00,99.99,,,,,,*67\r\n";
    const DateTime expected = {
        .hour = 10,
        .minute = 54,
        .second = 45,
        .day = 4,
        .month = 2,
        .year = 19
    };

    run_nmea_rmc_parse_test(sentence, expected);

    return MUNIT_OK;
}

// Test that a message with no time data is recognised as no signal
MunitResult test_rmc_without_time(const MunitParameter params[], void* user_data_or_fixture)
{
    const char sentence[] = "$GPRMC,,V,,,,,,,,,,N*53\r\n";

    GpsReadStatus status = run_nmea_parse(sentence);
    assert_equal_gps_status(status, kGPS_RMC_NoSignal);

    return MUNIT_OK;
}

// Invalid checksum fails
MunitResult test_invalid_checksum(const MunitParameter params[], void* user_data_or_fixture)
{
    const char sentence[] = "$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*14\r\n";

    GpsReadStatus status = run_nmea_parse(sentence);
    assert_equal_gps_status(status, kGPS_InvalidChecksum);

    return MUNIT_OK;
}

// Unknown sentence is ignored (RMB)
MunitResult test_unknown_sentence_rmb(const MunitParameter params[], void* user_data_or_fixture)
{
    const char sentence[] = "$GPRMB,A,4.08,L,EGLL,EGLM,5130.02,N,00046.34,W,004.6,213.9,122.9,A*3D\r\n";

    GpsReadStatus status = run_nmea_parse(sentence);
    assert_equal_gps_status(status, kGPS_NoMatch);

    return MUNIT_OK;
}

// Unknown sentence is ignored (GSV)
MunitResult test_unknown_sentence_gga(const MunitParameter params[], void* user_data_or_fixture)
{
    const char sentence[] = "$GPGGA,161229.487,3723.2475,N,12158.3416,W,1,07,1.0,9.0,M,,,,0000*18\r\n";

    GpsReadStatus status = run_nmea_parse(sentence);
    assert_equal_gps_status(status, kGPS_NoMatch);

    return MUNIT_OK;
}

// Unknown sentence is ignored (RMA)
MunitResult test_unknown_sentence_rma(const MunitParameter params[], void* user_data_or_fixture)
{
    const char sentence[] = "$GPRMA,A,llll.ll,N,lllll.ll,W,,,ss.s,ccc,vv.v,W*hh\r\n";

    GpsReadStatus status = run_nmea_parse(sentence);
    assert_equal_gps_status(status, kGPS_NoMatch);

    return MUNIT_OK;
}

// Unexpected termination of valid looking sentence fails
MunitResult test_unexpected_sentence_termination(const MunitParameter params[], void* user_data_or_fixture)
{
    const char sentence[] = "$GPRMC,but,not,really\r\n";

    GpsReadStatus status = run_nmea_parse(sentence);
    assert_equal_gps_status(status, kGPS_BadFormat);

    return MUNIT_OK;
}

// Rejection of an endless bogus message
MunitResult test_reject_endless_message(const MunitParameter params[], void* user_data_or_fixture)
{
    GpsReadStatus status;

    nmea_init();

    int count = 0;

    while (true) {
        const GpsReadStatus status = nmea_parse('\0');

        if (status == kGPS_BadFormat) {
            return MUNIT_OK;
        }

        if (++count > 1000) {
            munit_error("Failed to reject an endless stream of bad data");
        }
    }

    // Shouldn't reach here
    return MUNIT_ERROR;
}


MunitTest tests_nmea[] = {
    TEST_CASE(test_valid_rmc_sentence_1),
    TEST_CASE(test_valid_rmc_sentence_2),
    TEST_CASE(test_valid_rmc_with_empty_time),
    TEST_CASE(test_valid_sentence_stream),
    TEST_CASE(test_rmc_without_time),
    TEST_CASE(test_valid_gsv),
    TEST_CASE(test_invalid_checksum),
    TEST_CASE(test_unknown_sentence_rmb),
    TEST_CASE(test_unknown_sentence_gga),
    TEST_CASE(test_unknown_sentence_rma),
    TEST_CASE(test_reject_endless_message),

    END_TESTS_ARRAY()
};