#pragma once

#include "munit/munit.h"

extern MunitTest tests_nmea[];

static const MunitSuite suite_nmea = {
    .prefix = "/nmea",
    .tests = tests_nmea,
    .suites = NULL,
    .iterations = 1,
    .options = MUNIT_SUITE_OPTION_NONE,
};