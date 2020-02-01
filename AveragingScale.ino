/*********************************************************************
TODO:
  "Coffee Mode"
  Auto-off after a while
*********************************************************************/

#include <Wire.h>
#include <U8g2lib.h>
#include <ArduinoLowPower.h>

#include "HX711.h"
#include "buttons.h"
#include "CircularBuffer.h"

// 1.3" SH1106 DiyMall display.
#if defined(__AVR__)
U8G2_SH1106_128X64_NONAME_2_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#else
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
#endif
// 0.96" SSD1306 DiyMall display.
// U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0,  /* reset=*/ U8X8_PIN_NONE);

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 8;
const int LOADCELL_SCK_PIN = 7;

// Control Button wiring
const int POWER_BUTTON_PIN = 2;
const int ZERO_BUTTON_PIN = 3;
const int AVG_BUTTON_PIN = 9;
const int COFFEE_BUTTON_PIN = 10;

// This a calibrated value from my scale.
#if defined(__AVR__)
constexpr double GRAMS_PER_RAW = 363.79 / 135703.0;
#else
constexpr double GRAMS_PER_RAW = 0.0009570014631347681;
#endif

char* float_to_str(float f) {
  static char fbuf[16];
  // 7-wide: handle negative thousands with one decimal point:
  //   |1234567|
  //   |-2345.7|
  if (f > 99999.9) {
    f = 99999.9;
  }
  if (f < -9999.9) {
    f = -9999.9;
  }

#if defined(__AVR__)
  return dtostrf(f, 7, 1, fbuf);
#else
  float absf = abs(f);
  snprintf(fbuf, sizeof(fbuf), "%5d.%1d", int(absf), int((absf - int(absf))*10));
  if (f < 0) {
    int i = 3;
    for (i = 3; i >= 0 && fbuf[i] != ' '; i--) {}
    fbuf[i] = '-';
  }
  return fbuf;
#endif
}

class AveragingScale {
  private:
    HX711 *hx711_;  // Not owned!
    bool wiredBackwards_;
    int32_t zero_;
    double unitsPerRaw_;
    CircularBuffer<int32_t, 50> buf_;

    bool averaging_;
    bool autoAveraging_;
    int32_t averagingSum_;
    int averagingSampleCount_;
  
  public:
    AveragingScale(HX711* hx711, double unitsPerRaw=1.0, bool wiredBackwards=false) :
      hx711_(hx711),
      wiredBackwards_(wiredBackwards),
      zero_(0),
      unitsPerRaw_(unitsPerRaw),
      averaging_(false),
      autoAveraging_(false)
      {}

    int32_t readRaw() {
      int32_t v = hx711_->read();
      if (wiredBackwards_) {
        v = -v;
      }
      buf_.push(v);
      if (averaging_ || autoAveraging_) {
        averagingSum_ += v;
        averagingSampleCount_++;
      }
      return v;
    }

    struct AverageStats {
      int32_t minValue;
      int32_t maxValue;
      int32_t sum;
      int32_t sampleCount;
      int32_t average;
    };

    double getAverageUnits(byte sampleCount) {
      return rawToUnits(getRawAverage(sampleCount).average);
    }

    AverageStats getRawAverage(byte sampleCount) {
      AverageStats stats;
      if (buf_.size() == 0) {
        return stats;
      }

      stats.sum = 0;
      stats.sampleCount = 0;
      stats.minValue = INT32_MAX;
      stats.maxValue = INT32_MIN;
      for (int i = 0; i < sampleCount && i < buf_.size(); i++, stats.sampleCount++) {
        int32_t v = buf_[i];
        stats.sum += v;
        stats.minValue = min(stats.minValue, v);
        stats.maxValue = max(stats.maxValue, v);
      }
      stats.average = stats.sum / stats.sampleCount;

      return stats;
    }

    void setZero(int32_t zero) {
      Serial.print(F("ZERO! Old: "));
      Serial.print(zero_);
      Serial.print(F(" new: "));
      Serial.println(zero);
      stopAveraging();
      zero_ = zero;
    }

    void zero(byte sampleCount = 10) {
      while (buf_.size() < sampleCount) {
        readRaw();
      }
      AverageStats stats = getRawAverage(sampleCount);
      zero_ = getRawAverage(sampleCount).average;
      Serial.print(F("Zero!  zero_: "));
      Serial.print(zero_);
      Serial.print(F(" "));
      Serial.print(stats.sum);
      Serial.print(F(" "));
      Serial.print(stats.sampleCount);
      Serial.print(F(" "));
      Serial.print(buf_.size());
      Serial.println();
    }

    int32_t getRaw() const {
      if (averaging_ || autoAveraging_) {
        return averagingSum_ / averagingSampleCount_;
      } else {
        return buf_[0];
      }
    }

    int32_t get() const {
      return getRaw() - zero_;
    }

    float getUnits() const {
      return get() * unitsPerRaw_;
    }

    int32_t getInstant() const {
      return buf_[0] - zero_;
    }

    double getInstantUnits() const {
      return getInstant() * unitsPerRaw_;
    }

    double rawToUnits(long raw) const {
      return (raw - zero_) * unitsPerRaw_;
    }

    void startAveraging() {
      averaging_ = true;
      autoAveraging_ = false;
      averagingSum_ = buf_[0];
      averagingSampleCount_= 1;
    }

    void stopAveraging() {
      averaging_ = false;
      autoAveraging_ = false;
    }

    double mostStableRecentAverage() {
      double holdValue = getUnits();
      double unitsValue = holdValue;
      double minDelta = holdValue;
      for (int i = 0; i < buf_.size() && averagingSampleCount_ > 0; i++) {
        averagingSum_ -= buf_[i];
        averagingSampleCount_--;
        double newUnitsValue = getUnits();
        double delta = abs(newUnitsValue - unitsValue);
        if (delta < minDelta) {
          Serial.print(F("Maybe HOLD AVG: "));
          Serial.print(unitsValue);
          Serial.print(F(" averagingSampleCount_: "));
          Serial.println(averagingSampleCount_);
          holdValue = unitsValue;
          minDelta = delta;
        }
        unitsValue = newUnitsValue;
      }
      return holdValue;
    }

    bool isAveraging() const {
      return averaging_;
    }

    bool isAutoAveraging() const {
      return autoAveraging_;
    }

    void trackZeroDrift() {
      if (averaging_) {
        return;
      }

      const byte sampleCount = 20;  // 2s at 10 samples-per-second.
      AverageStats stats = getRawAverage(sampleCount);

      // Don't try to track drift if zero isn't stable.
      if ((stats.maxValue - stats.minValue) * unitsPerRaw_ > 1.0) {
        return;
      }

      // Don't track drift if the change is too large.
      // And don't make lots of zero adjustments for small changes either.
      float diffUnits = abs((stats.average - zero_) * unitsPerRaw_);
      if (diffUnits > 1.0 || diffUnits < 0.3) {
          return;
      }

      Serial.print(F("AutoZero!  millis: "));
      Serial.print(millis());
      Serial.print(F(" oldzero: "));
      Serial.print(zero_);
      Serial.print(F(" newzero: "));
      Serial.print(stats.average);
      Serial.print(F(" diffUnits: "));
      Serial.print(diffUnits);
      Serial.println();
      zero_ = stats.average;
    }

    bool autoAverage() {
      if (averaging_ || autoAveraging_) {
        return false;
      }

      const byte sampleCount = 5;
      AverageStats stats = getRawAverage(sampleCount);

      if (stats.maxValue > 1.05 * stats.average) {
        return false;
      }
      if (stats.minValue < 0.95 * stats.average) {
        return false;
      }
      if ((stats.average -zero_) * unitsPerRaw_ < 1.0) {
        return false;
      }

      autoAveraging_ = true;
      averagingSum_ = stats.sum;
      averagingSampleCount_= stats.sampleCount;
      Serial.print(F("Auto-Average! "));
      Serial.println(getUnits());
      return true;
    }

    bool autoAverageOff() {
      if (averaging_ || !autoAveraging_) {
        return false;
      }

      const byte sampleCount = 5;
      AverageStats stats = getRawAverage(sampleCount);

      int32_t currentAverage = getRaw();
      /*
      if (stats.maxValue < 1.1 * currentAverage &&
          stats.minValue > 0.9 * currentAverage) {
        return false;
      }
      */
      if (stats.average < 1.1 * currentAverage &&
          stats.average > 0.9 * currentAverage) {
        return false;
      }

      /*
      float diffUnits = (stats.average - zero_) * unitsPerRaw_;
      if (diffUnits > 1.0) {
        return false;
      }
      */

      Serial.println(F("Auto-Average Off! "));
      stopAveraging();
      return true;
    }
};

HX711 hx711;
AveragingScale ascale(&hx711, GRAMS_PER_RAW, /*wiredBackwards=*/false);


void wakeUp() {
  Serial.println(F("-------------------------"));
  Serial.print(F("Yawn.. waking up! millis: "));
  Serial.print(millis());
  Serial.println();
  Serial.print(F("POWER PIN: "));
  Serial.print(digitalRead(POWER_BUTTON_PIN));
  Serial.println();

  // setup();
  /*
  watchDigitalPin(POWER_BUTTON_PIN);
  display.begin();
  hx711.power_up();
  */
}

void goToSleep() {
  Serial.println(F("Yawn.. going to sleep!"));
  Serial.print(F("POWER PIN: "));
  Serial.print(digitalRead(POWER_BUTTON_PIN));
  Serial.println();

  Serial.println(F("Putting display to sleep"));
  display.setPowerSave(1);
  Serial.println(F("Sleeping the HX711"));
  hx711.power_down();

  // Wait for button to settle.
  long delayUntil = millis() + 200;
  while (millis() < delayUntil) {
    if (digitalRead(POWER_BUTTON_PIN) == LOW) {
      Serial.println(F("Zero still down!"));
      delayUntil = millis() + 200;
    }
  }
  Serial.println(F("going to sleep"));
  LowPower.attachInterruptWakeup(POWER_BUTTON_PIN, wakeUp, FALLING);
  LowPower.deepSleep();
  Serial.println(F("Yawn.. done sleeping?!"));
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("-------------------------"));

  Serial.println(F("Starting display..."));
  display.begin();
  display.setFont(u8g2_font_profont29_tr);
  display.firstPage();
  do {
    display.setCursor(0, 16);
    display.println(F("not your"));
    display.setCursor(0, 16+23);
    display.println(F(" average"));
    display.setCursor(0, 16+46);
    display.println(F("  scale"));
  } while( display.nextPage() );
  long splashStart = millis();

  Serial.println(F("Starting scale..."));
  hx711.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  ascale.zero(1);
  Serial.println(F("Scale started"));

  Serial.println(F("Watching button pins..."));
  watchDigitalPin(POWER_BUTTON_PIN);
  watchDigitalPin(ZERO_BUTTON_PIN);
  watchDigitalPin(AVG_BUTTON_PIN);
  watchDigitalPin(COFFEE_BUTTON_PIN);

  long delayTime = splashStart + 2000 - millis();
  delay(delayTime);
  Serial.print(F("Leaving splash up for: "));
  Serial.print(delayTime);
  Serial.println(F("ms"));
  Serial.println(F("Setup complete."));
  Serial.println(F("-------------------------"));
}

void displayAveraging(U8G2 *u8g2,
                      double grams,
                      bool averaging,
                      double instantGrams,
                      const CircularBuffer<double, 2> holds) {
  // Current value big at top
  u8g2->setFont(u8g2_font_profont22_tr);
  u8g2->setCursor(0, 14);
  if (ascale.isAveraging()) {
    u8g2->print("AVG");
  } else if (ascale.isAutoAveraging()) {
    u8g2->print("AUT");
  } else {
    u8g2->print("   ");
  }
  u8g2->setCursor(46, 14);
  u8g2->print(float_to_str(grams));

  // Any hold values
  u8g2->setFont(u8g2_font_profont17_tr);
  for (int holdIndex = 0; holdIndex < 2 && holdIndex < holds.size(); holdIndex++) {
    u8g2->setCursor(0, 17 * (holdIndex+2));
    u8g2->print("HLD");
    u8g2->setCursor(65, 17 * (holdIndex+2));
    u8g2->print(float_to_str(holds[holdIndex]));
  }

  // Instant value
  u8g2->setFont(u8g2_font_profont12_tr);
  u8g2->setCursor(0, 64);
  u8g2->print("Instant:");
  u8g2->setCursor(86, 64);
  u8g2->print(float_to_str(ascale.getInstantUnits()));
}

void displayCoffee(U8G2 *u8g2, double grams, long coffeeTime) {
  char buf[16];
  // Current value big at top
  u8g2->setFont(u8g2_font_profont29_tr);
  u8g2->setCursor(0, 23);
  int minutes = (coffeeTime / 1000) / 60;
  int seconds = (coffeeTime / 1000) % 60;
  snprintf(buf, sizeof(buf), "  %2d:%02ds", minutes, seconds);
  u8g2->print(buf);

  u8g2->setFont(u8g2_font_profont29_tr);
  u8g2->setCursor(0, 58);
  snprintf(buf, sizeof(buf), "%sg", float_to_str(grams));
  u8g2->print(static_cast<char*>(buf));
  /*
  u8g2->setCursor(0, 19);
  if (ascale.isAveraging()) {
    u8g2->print("AVG");
  } else if (ascale.isAutoAveraging()) {
    u8g2->print("AUT");
  } else {
    u8g2->print("   ");
  }
  */
}

void loop() {
  display.clear();
  CircularBuffer<double, 2> holds;
  bool coffeeMode = false;
  long coffeeStart = 0;

  long lastLoop = millis();
  while (true) {
    long readStart = millis();
    ascale.readRaw();
    long readDone = millis();
    /*
    Serial.print(F("Time to complete previous loop: "));
    Serial.print(readStart - lastLoop);
    Serial.print(" : ");
    Serial.print(F("  Time to readRaw: "));
    Serial.print(readDone - readStart);
    Serial.println();
    */
    lastLoop = readStart;

    double value = ascale.getUnits();
    // Don't distract with tiny fluctuations around zero.
    if (!ascale.isAveraging() && value < 0.3 && value > -0.3) {
      value = 0.0;
    }

    if (coffeeMode) {
      if (coffeeStart == 0 && value >= 1.0) {
        coffeeStart = millis();
      }
    }

    display.firstPage();
    do {
      if (coffeeMode) {
        displayCoffee(&display, value, coffeeStart ? millis() - coffeeStart : 0 );
      } else {
        displayAveraging(&display, value, ascale.isAveraging(), ascale.getInstantUnits(), holds);
      }
    } while ( display.nextPage() );

    // Serial.print(F("Value: "));
    // Serial.println(ascale.getUnits());
    // Serial.print(F("Value: "));
    // Serial.println(float_to_str(ascale.getUnits()));

    if (wasPressed(POWER_BUTTON_PIN)) {
      Serial.print(F("POWER was pressed"));
      Serial.println();
      goToSleep();
      setup();
      return;
    }

    if (wasPressed(ZERO_BUTTON_PIN)) {
      Serial.print(F("ZERO was pressed"));
      Serial.println();
      ascale.setZero(ascale.getRaw());
      // goToSleep();
    } else {
      ascale.trackZeroDrift();
    }

    if (wasPressed(AVG_BUTTON_PIN)) {
      Serial.print(F("AVG was pressed; "));
      if (!ascale.isAveraging()) {
        Serial.print(F("Averaging on."));
        ascale.startAveraging();
      } else { 
        Serial.print(F("Averaging off."));
        holds.push(value);
        ascale.stopAveraging();
      }
      Serial.println();
    } else if (ascale.isAveraging()) {

      Serial.println("averaging: ");
      Serial.println(value);
      Serial.println(ascale.getAverageUnits(10));
      Serial.println(abs(ascale.getRawAverage(10).average * GRAMS_PER_RAW));
      auto avg = ascale.getRawAverage(10);
      Serial.println(avg.average);
      Serial.println(avg.sum);
      Serial.println(avg.sampleCount);

      double recentAverage = ascale.getAverageUnits(10);
      if (value > 10.0 && abs(recentAverage) < 1.0 ||
        value <= 10.0 && abs(recentAverage / value) < 0.1) {
        double holdValue = ascale.mostStableRecentAverage();
        holds.push(holdValue);
        Serial.println("STOP averaging: ");
        ascale.stopAveraging();
      }
    }

    if (wasPressed(COFFEE_BUTTON_PIN)) {
      Serial.print(F("COFFEE was pressed"));
      Serial.println();
      if (coffeeMode) {
        Serial.print(F("Drink up!"));
        Serial.println();
        coffeeMode = false;
      } else {
        coffeeMode = true;
        coffeeStart = 0;
        Serial.print(F("Zeroing..."));
        Serial.println();
        ascale.setZero(ascale.getRaw());
        Serial.print(F("Coffee Mode on!"));
        Serial.println();
      }
    }

    if (int p = getPhantoms()) {
      Serial.print(F("Phantom buttom presses: "));
      Serial.println(p);
    }
    if (int b = getBounces()) {
      Serial.print(F("Bounces: "));
      Serial.println(b);
    }
  }
}
