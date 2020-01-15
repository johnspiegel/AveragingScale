#ifndef _BUTTONS_H_
#define _BUTTONS_H_

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif

#include <stdint.h>

void watchDigitalPin(uint8_t pin);

bool wasPressed(uint8_t pin, uint8_t wantState=LOW);

int getPhantoms();
int getBounces();

#endif // _BUTTONS_H_
