# Makefile for avr-gcc cross-compilation and AVR firmware flashing.
#
# For simplicity, this Makefile is shared. Just choose/add a TARGET.

# Name of source file to build, minus the .c
TARGET = inverted_pendulum
# TARGET = hbridge
# TARGET = infineon_bridge
# TARGET = 4digit_7segment
# TARGET = smart_pendulum
# TARGET = i2c_lcd
# TARGET = quad_encoder
# TARGET = SN754410
# TARGET = drive_A4988
# TARGET = dcmotor_1way
# TARGET = piezo
# TARGET = voltmeter
# TARGET = vacuum_switcher
# TARGET = dual_vacuum_switcher
# TARGET = echo
# TARGET = echo_isr
# TARGET = servo_demo
# TARGET = fade
# TARGET = fade_timer0
# TARGET = pwm
# TARGET = am
# TARGET = reactime
# TARGET = ext_interrupt
# TARGET = adc_freerun
# TARGET = lightmeter
# TARGET = toggle
# TARGET = button
# TARGET = blink

# Port addresses of various arduino devices
# USBPORT = /dev/cu.usbmodem1451
USBPORT = /dev/cu.usbserial-00002014
# USBPORT = /dev/cu.usbserial-00001014
# USBPORT = /dev/cu.wchusbserial1410

# MCU, F_CPU, and BAUD:
# These can be overridden in user code with #define, but take care to place
# them before entry into any headers that refer to them.
# Note: BAUD is used for USART configuration. It has nothing to do with
# the upload rate to built-in bootloaders on arduino boards.
MCU   = atmega328p

# If using external 16 MHz crystal oscillator:
# Fuse settings are (H:05, E:DE, L:FF)
F_CPU = 16000000UL
BAUD  = 19200

# If using 328p w/ factory defaults (8 MHz internal oscillator, prescaled 1/8):
# Fuse settings are (H:07, E:D9, L:62) or (H:00, E:DF, L:62)
# F_CPU = 1000000UL
# BAUD  = 9600

# Location of USART utilities
LIBDIR = ../common

# Set default programmer to use. Others are defined below (see flash_*) and
# can be selected via command line args to make.
PROGRAMMER_TYPE = usbtiny

# extra arguments to avrdude: baud rate, chip type, -F flag, etc.
PROGRAMMER_ARGS =

# These executables must be in your PATH
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
AVRSIZE = avr-size
AVRDUDE = avrdude

##########        Compile source files into .elf                ##########
##########        Convert .elf file into .hex                   ##########
##########        You shouldn't need to edit below.             ##########

## The name of your project (without the .c)
## Or name it automatically after the enclosing directory
# TARGET = $(lastword $(subst /, ,$(CURDIR)))

# Object files: will find all .c/.h files in current directory
#  and in LIBDIR.  If you have any other (sub-)directories with code,
#  you can add them in to SOURCES below in the wildcard statement.
SOURCES=$(TARGET).c $(wildcard $(LIBDIR)/*.c)
# SOURCES=$(wildcard *.c $(LIBDIR)/*.c)
OBJECTS=$(SOURCES:.c=.o)
HEADERS=$(SOURCES:.c=.h)

## Compilation options, type man avr-gcc if you're curious.
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I. -I$(LIBDIR)
CFLAGS = -Os -g -std=gnu99 -Wall
## Use short (8-bit) data types
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
## Splits up object files per function
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS = -Wl,-Map,$(TARGET).map
## Optional, but often ends up with smaller code
LDFLAGS += -Wl,--gc-sections
## Relax shrinks code even more, but makes disassembly messy
## LDFLAGS += -Wl,--relax
## LDFLAGS += -Wl,-u,vfprintf -lprintf_flt -lm  ## for floating-point printf
## LDFLAGS += -Wl,-u,vfprintf -lprintf_min      ## for smaller printf
TARGET_ARCH = -mmcu=$(MCU)

## Explicit pattern rules:
##  To make .o files from .c files
%.o: %.c $(HEADERS) Makefile
	 $(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c -o $@ $<;

$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(TARGET_ARCH) $^ $(LDLIBS) -o $@

%.hex: %.elf
	 $(OBJCOPY) -j .text -j .data -O ihex $< $@

%.eeprom: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@

%.lst: %.elf
	$(OBJDUMP) -S $< > $@

## These targets don't have files named after them
.PHONY: all disassemble disasm eeprom size clean flash fuses

all: $(TARGET).hex

debug:
	@echo
	@echo "Source files:"   $(SOURCES)
	@echo "MCU, F_CPU, BAUD:"  $(MCU), $(F_CPU), $(BAUD)
	@echo

# Optionally create listing file from .elf
# This creates approximate assembly-language equivalent of your code.
# Useful for debugging time-sensitive bits,
# or making sure the compiler does what you want.
disassemble: $(TARGET).lst

disasm: disassemble

# Optionally show how big the resulting program is
size:  $(TARGET).elf
	$(AVRSIZE) -C --mcu=$(MCU) $(TARGET).elf

clean_target:
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).obj \
	$(TARGET).o $(TARGET).d $(TARGET).eep $(TARGET).lst \
	$(TARGET).lss $(TARGET).sym $(TARGET).map $(TARGET)~ \
	$(TARGET).eeprom

clean:
	rm -f *.elf *.hex *.obj *.o $(LIBDIR)/*.o *.d *.eep *.lst *.lss *.sym *.map *~ *.eeprom

##########------------------------------------------------------##########
##########              Programmer-specific details             ##########
##########           Flashing code to AVR using avrdude         ##########
##########------------------------------------------------------##########

flash: $(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER_TYPE) -p $(MCU) $(PROGRAMMER_ARGS) -U flash:w:$<

## An alias
program: flash

flash_eeprom: $(TARGET).eeprom
	$(AVRDUDE) -c $(PROGRAMMER_TYPE) -p $(MCU) $(PROGRAMMER_ARGS) -U eeprom:w:$<

avrdude_terminal:
	$(AVRDUDE) -c $(PROGRAMMER_TYPE) -p $(MCU) $(PROGRAMMER_ARGS) -nt

## If you've got multiple programmers that you use,
## you can define them here so that it's easy to switch.
## To invoke, use something like `make flash_arduinoISP`
flash_usbtiny: PROGRAMMER_TYPE = usbtiny
flash_usbtiny: PROGRAMMER_ARGS =  # USBTiny works with no further arguments
flash_usbtiny: flash

flash_usbasp: PROGRAMMER_TYPE = usbasp
flash_usbasp: PROGRAMMER_ARGS =  # USBasp works with no further arguments
flash_usbasp: flash

flash_arduinoISP: PROGRAMMER_TYPE = avrisp
flash_arduinoISP: PROGRAMMER_ARGS = -b 19200 -P /dev/ttyACM0
## (for windows) flash_arduinoISP: PROGRAMMER_ARGS = -b 19200 -P com5
flash_arduinoISP: flash

flash_109: PROGRAMMER_TYPE = avr109
flash_109: PROGRAMMER_ARGS = -b 9600 -P /dev/ttyUSB0
flash_109: flash

# Arduinos
uno: PROGRAMMER_TYPE = arduino
uno: F_CPU = 16000000UL
uno: BAUD = 19200
uno: PROGRAMMER_ARGS = -b 115200 -P $(USBPORT)
uno: flash

nano: PROGRAMMER_TYPE = arduino
nano: F_CPU = 16000000UL
nano: PROGRAMMER_ARGS = -b 57600 -P $(USBPORT)
nano: flash
nano: size

##########------------------------------------------------------##########
##########       Fuse settings and suitable defaults            ##########
##########------------------------------------------------------##########

## Mega 48, 88, 168, 328 default values
LFUSE = 0x62
HFUSE = 0xdf
EFUSE = 0x00

## Generic
FUSE_STRING = -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m

fuses:
	$(AVRDUDE) -c $(PROGRAMMER_TYPE) -p $(MCU) \
	           $(PROGRAMMER_ARGS) $(FUSE_STRING)
show_fuses:
	$(AVRDUDE) -c $(PROGRAMMER_TYPE) -p $(MCU) $(PROGRAMMER_ARGS) -nv

## Called with no extra definitions, sets to defaults
set_default_fuses:  FUSE_STRING = -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m
set_default_fuses:  fuses

## Set the fuse byte for full-speed mode
## Note: can also be set in firmware for modern chips
set_fast_fuse: LFUSE = 0xE2
set_fast_fuse: FUSE_STRING = -U lfuse:w:$(LFUSE):m
set_fast_fuse: fuses

## Set the EESAVE fuse byte to preserve EEPROM across flashes
set_eeprom_save_fuse: HFUSE = 0xD7
set_eeprom_save_fuse: FUSE_STRING = -U hfuse:w:$(HFUSE):m
set_eeprom_save_fuse: fuses

## Clear the EESAVE fuse byte
clear_eeprom_save_fuse: FUSE_STRING = -U hfuse:w:$(HFUSE):m
clear_eeprom_save_fuse: fuses
