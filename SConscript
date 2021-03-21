# Default to using parallel build using all available cores
import multiprocessing
SetOption('num_jobs', multiprocessing.cpu_count())

# Device definition for the stm8s.h header
# This enables available features of the device library.
STM8_DEVICE_DEFINE = "STM8S003"

# Programmer argument for stm8flash (stlink or stlinkv2)
STM8_PROGRAMMER = "stlinkv2"

# Target device argument for stm8flash
STM8_DEVICE_PROG = "stm8s003?3"

# RAM and flash size available on the target device
# Note that SDCC doesn't appear to implement these checks for STM8 currently
FLASH_SIZE_BYTES = 8096
INTERNAL_RAM_SIZE_BYTES = 1024

# Compiled hex file target
HEX_FILE = 'main.ihx'

env_target = Environment(
    CC = 'sdcc',

    CPPDEFINES = {
        STM8_DEVICE_DEFINE: None,
    },

    CFLAGS = [
        # Building for STM8
        '-mstm8',

        # Optimisations and standard
        '--std-sdcc11',
        '--opt-code-size',
    ],

    # SDCC uses .rel instead of .o for linkable objects
    OBJSUFFIX = ".rel",

    LINKFLAGS = [
        '--verbose',
        '--out-fmt-ihx',

        # Building for STM8
        '-mstm8',

        # Available resources
        '--iram-size', INTERNAL_RAM_SIZE_BYTES,
        '--code-size', FLASH_SIZE_BYTES,
    ],

    LIBS = [
        'stm8',
    ],

    # Header search paths
    # Note that these are relative to the build directory (if SConstruct src='.'')
    CPPPATH = [
        '',
        '.',
        '../driver/inc',
    ],

    # Custom variables for use in stm8flash commands
    STM8_PROGRAMMER = STM8_PROGRAMMER,
    STM8_DEVICE_PROG = STM8_DEVICE_PROG,
)

env_target.Alias(
    'flash',
    env_target.Command(
        '_flash_phony_output',
        HEX_FILE,
        'stm8flash -c $STM8_PROGRAMMER -p $STM8_DEVICE_PROG -w $SOURCES'
    )
)

env_target.Alias(
    'size',
    env_target.Command(
        '_size_phony_output',
        HEX_FILE,
        'size $SOURCES'
    )
)

build_target = env_target.Program(
    HEX_FILE,
    [
        'main.c',
        'delay.c',
        'display.c',
        'nmea.c',
        'uart.c',
        'driver/src/stm8s_clk.c',
        'driver/src/stm8s_spi.c',
        'driver/src/stm8s_uart1.c',
    ]
)

# Only build the hex file by default (don't try to flash, etc)
env_target.Default(build_target, 'size')

# Remove the build directory when doing a clean (-c)
Clean(build_target, '../build')


###


env_testing = Environment(
    # Header search paths
    # Note that these are relative to the build directory (if SConstruct src='.'')
    CPPPATH = [
        '',
        '.',
        '../driver/inc',
    ],

    CFLAGS = [
        '-ggdb3',
        '-Og',
    ]
)

env_testing.Alias(
    'build_tests',
    env_testing.Program(
        'testsuite',
        [
            'tests/munit/munit.c',
            'tests/suite.c',
            'tests/test_nmea.c',
            'nmea.c',
        ]
    ),
)

env_testing.Alias(
    'run_tests',
    env_target.Command(
        '_run_tests_phony_output',
        'testsuite',
        './$SOURCES'
    )
)

env_testing.Alias(
    'test',
    [
        'build_tests',
        'run_tests',
    ]
)