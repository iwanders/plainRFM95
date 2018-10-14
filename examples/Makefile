# Always use bash.
SHELL := /bin/bash

# This makefile allows use of the Arduino command line interface and sets the sketch folder to the current folder.
# This allows a per project libraries folder.

# Path to your arduino installation
ARDUINOPATH ?= ~/Documents/Code/teensy/arduino-1.8.5/

# which board to target
# arduino:avr:uno for normal UNO.
# arduino:sam:arduino_due_x for Arduino DUE
# Arduino_STM32:STM32F1:genericSTM32F103C

# see Arduino_STM32/STM32F1/boards.txt
# see hardware/teensy/avr/boards.txt
TEENSY_TYPE ?= 31

ifeq ($(TEENSY_TYPE),31)
    CPU_SPEED = 72
    OPT = o2std
    VARIANT = mk20dx256
    USB = serial
    KEYS = en-us
    BOARD = teensy:avr:teensy31
endif

# TEENSY_TYPE=LC make
ifeq ($(TEENSY_TYPE),LC)
    CPU_SPEED = 48
    OPT = o2std
    VARIANT = mkl26z64
    USB = serial
    KEYS = en-us
    BOARD = teensy:avr:teensyLC
endif

# stm32 is the following:
#~ UPLOAD_METHOD = STLinkMethod
# Arduino_STM32:STM32F1:genericSTM32F103C:device_variant=STM32F103C8,upload_method=STLinkMethod,cpu_speed=speed_72mhz,opt=osstd


# Additional flags to pass to the build & upload.
EXTRAFLAGS = --verbose-build --preserve-temp-files

# Create path to the arduino executable.
ARDUINOEXEC = $(ARDUINOPATH)/arduino

BUILDDIR ?= /tmp/arduino_builddir_$(TEENSY_TYPE)



PREFERENCES = --pref device_variant=$(VARIANT) --pref opt=$(OPT) --pref speed=$(CPU_SPEED) --pref build.path=$(BUILDDIR) --pref usb=$(USB) --pref keys=$(KEYS)
FQBN=$(BOARD):opt=$(OPT),speed=$(CPU_SPEED),usb=$(USB),keys=$(KEYS)

FIXLIBRARIES=--pref sketchbook.path=$(shell pwd)


all:
	# This sketch .ino file is actually unused during the build, it doesn't exist...
	$(ARDUINOEXEC) $(EXTRAFLAGS) --board $(FQBN) $(PREFERENCES) $(FIXLIBRARIES) --upload main.ino

help:
	$(ARDUINOEXEC) $(EXTRAFLAGS) --board $(FQBN) $(PREFERENCES) $(FIXLIBRARIES) --list

clean:
	rm -rf /tmp/arduino_builddir

libs:
	@echo placeholder
	#$(ARDUINOEXEC) --install-library IRremote:2.0.1

env:
	@echo "alias arduino=\"$(ARDUINOEXEC)\""