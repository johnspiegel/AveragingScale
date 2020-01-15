#include "buttons.h"
#include <avr/interrupt.h>

namespace {

constexpr int debounceDelayMs = 50;
  
struct PinStatus {
  uint8_t pin = 0;
  volatile uint8_t state = HIGH;
  volatile uint8_t nextState = HIGH;
  volatile uint8_t newEvent = false;
  volatile unsigned long nextStateMillis = 0;
  
  PinStatus *nextPinStatus = nullptr;
};

//volatile uint16_t watched_digital_pins = 0;
PinStatus *pinStatuses = nullptr; 

volatile int phantoms = 0;
volatile int bounces = 0;

}  // namespace

ISR (PCINT0_vect)
{
  const unsigned long isr_millis = millis();
  int events = 0;
  int myBounces = 0;
  for (PinStatus *ps = pinStatuses; ps != nullptr; ps = ps->nextPinStatus) {
    uint8_t state = digitalRead(ps->pin);
    // Warning: overflow after 50 days.
    if (isr_millis >= ps->nextStateMillis) {
      if (state != ps->state) {
        // Lock in a button press for 200ms.
        ps->newEvent = true;
        ps->nextStateMillis = isr_millis + debounceDelayMs;
      }
      if (state != ps->state || state != ps->nextState) {
        events++;
      }
      ps->state = ps->nextState = state;
      
    } else {
      if (state != ps->state) {
        myBounces++;
      }
      // Bounce!
      ps->nextState = state;
    }
  }
  if (events == 0 && myBounces == 0) {
    phantoms++;
  }
  bounces += myBounces;
}

void watchDigitalPin(uint8_t pin) {
  cli();
  
  pinMode(pin, INPUT_PULLUP);

  PinStatus *ps = new PinStatus();
  ps->pin = pin;
  ps->state = digitalRead(pin);
  ps->nextState = ps->state;
  ps->nextPinStatus = pinStatuses;
  pinStatuses = ps;

  // watched_digital_pins |= (1 << pin);
  
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group

  sei();
}

bool wasPressed(uint8_t pin, uint8_t wantState) {
  bool newEvent = false;
  cli();
  
  for (PinStatus *ps = pinStatuses; ps != nullptr; ps = ps->nextPinStatus) {
    if (ps->pin != pin) {
      continue;
    }

    if (ps->state == wantState) {
      newEvent |= ps->newEvent;
      ps->newEvent = false;
    }
    
    if (millis() >= ps->nextStateMillis) {
      ps->newEvent = (ps->nextState != ps->state);
      ps->state = ps->nextState;
      ps->nextStateMillis = ps->nextStateMillis + debounceDelayMs;
    }

    if (ps->state == wantState) {
      newEvent |= ps->newEvent;
      ps->newEvent = false;
    }
    break;
  }
  
  sei();
  return newEvent;
}

int getPhantoms() {
  cli();
  int r = phantoms;
  phantoms = 0;
  sei();
  return r;
}

int getBounces() {
  cli();
  int r = bounces;
  bounces = 0;
  sei();
  return r;
}
