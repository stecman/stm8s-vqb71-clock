#pragma once

#include <string.h>

// Helper to define a test case with just a function name
#define TEST_CASE(func) ((MunitTest){ \
        .name = "/" #func, \
        .test = &func, \
        .setup = NULL, \
        .tear_down = NULL, \
        .options = MUNIT_TEST_OPTION_NONE, \
        .parameters = NULL, \
    })

// Helper to terminate an array of test structs
#define END_TESTS_ARRAY() { NULL, NULL }

// Helper to terminate an array of suite structs
#define END_SUITES_ARRAY() { NULL, NULL, NULL, 0, MUNIT_SUITE_OPTION_NONE }