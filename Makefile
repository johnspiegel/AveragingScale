# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile

USER_LIB_PATH := $(realpath ../libraries)
# ARDUINO_LIBS = u8g2 Wire
# ARDUINO_LIBS = U8g2_Arduino Wire Arduino-Scheduler
ARDUINO_LIBS = U8g2_Arduino Wire  SPI
BOARD_TAG    = uno
include /usr/share/arduino/Arduino.mk
