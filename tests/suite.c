#include "munit/munit.h"

#include "test_utils.h"
#include "test_nmea.h"

// Sub-suites
static MunitSuite suites[] = {
    suite_nmea,
    END_SUITES_ARRAY()
};

// Top-level suite
static MunitSuite top_level_suite = {
    .prefix = NULL,
    .tests = NULL,
    .suites = suites,
    .iterations = 1,
    .options = MUNIT_SUITE_OPTION_NONE,
};

int main (int argc, char* const argv[])
{
    return munit_suite_main(&top_level_suite, NULL, argc, argv);
}