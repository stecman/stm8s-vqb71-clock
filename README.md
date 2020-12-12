# GPS Clock (STM8S)

Firmware for an STM8SF003 microcontroller to show HH:MM:SS time on six VQB-71 segment  displays, with accurate time from a GPS module. More project write-up, schematics, etc is available on Hackaday: [VQB 71 GPS Clock](https://hackaday.io/project/170633-vqb-71-gps-clock).

The base of this project was ported to STM8S from my [1KB AVR GPS Clock](https://github.com/stecman/avr-doomclock) project.

## Setup

Grab the code:

```sh
git clone https://github.com/stecman/stm8s-vqb71-clock.git
cd stm8s-vqb71-clock

# Pull in SDCC compatible STM8S peripheral library
git submodule init
git submodule update
```

Once the toolchain below is available:

```sh
# Build
scons

# Flash through STLinkV2
scons flash
```

## Linux Toolchain

### SCons (build tool)

This should be available in your distribution's pacakage manager. It can also be installed via Python's `pip` package manager.

### SDCC (compiler)

[SDCC](http://sdcc.sourceforge.net/) may be available in your distro's package manager. I recommend installing from source to get the most recent release, as the Debian and Ubuntu repos can be a few versions behind:

```sh
# with Git
git clone https://github.com/svn2github/sdcc

# or with SVN
svn co http://svn.code.sf.net/p/sdcc/code/trunk sdcc

cd sdcc/sdcc
./configure
make
sudo make install
```

### stm8flash (flashing tool)

[stm8flash](https://github.com/vdudouyt/stm8flash) uses an STLink V1/V2 to program STM8 devices through their SWIM interface.

This needs to be compiled from source currently, which is simple:

```sh
git clone https://github.com/vdudouyt/stm8flash
cd stm8flash
make
sudo make install
```