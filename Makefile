# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile

ARCHITECTURE = samd
# BOARDS_TXT = /home/jbs/.arduino15/packages/arduino/hardware/samd/1.8.4/boards.txt
ARDUINO_DIR = ../arduino-1.8.11/
# ARDUINO_PACKAGE_DIR = /home/jbs/.arduino15/packages

# ARDUINO_DIR = /home/jbs/.arduino15/packages/arduino
USER_LIB_PATH := $(realpath ../libraries)
# ARDUINO_LIBS = u8g2 Wire
# ARDUINO_LIBS = U8g2_Arduino Wire Arduino-Scheduler
# ARDUINO_LIBS = U8g2_Arduino Wire  SPI
ARDUINO_LIBS = U8g2 Wire SPI Arduino_Low_Power RTCZero
# BOARD_TAG    = uno
BOARD_TAG    = nano_33_iot

# include /usr/share/arduino/Arduino.mk
include ../Arduino-Makefile/Sam.mk
