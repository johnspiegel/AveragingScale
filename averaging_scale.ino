/*********************************************************************
*********************************************************************/

#include <Wire.h>
#include <U8g2lib.h>
#include "HX711.h"
#include "buttons.h"
#include "CircularBuffer.h"

// 1.3" SH1106 DiyMall display.
U8G2_SH1106_128X64_NONAME_2_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// 0.96" SSD1306 DiyMall display.
// U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0,  /* reset=*/ U8X8_PIN_NONE);

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

// Control Button wiring
const int ZERO_BUTTON_PIN = 12;
const int AVG_BUTTON_PIN = 11;

// This a calibrated value from my scale.
constexpr double GRAMS_PER_RAW = 363.79 / 135703.0;

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
  return dtostrf(f, 7, 1, fbuf);
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
    CircularBuffer<double, 2> hold_;

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

    int32_t getRaw() {
      if (averaging_ || autoAveraging_) {
        return averagingSum_ / averagingSampleCount_;
      } else {
        return buf_[0];
      }
    }

    int32_t get() {
      return getRaw() - zero_;
    }

    float getUnits() {
      return get() * unitsPerRaw_;
    }

    int32_t getInstant() {
      return buf_[0] - zero_;
    }

    double getInstantUnits() {
      return getInstant() * unitsPerRaw_;
    }

    void startAveraging() {
      averaging_ = true;
      autoAveraging_ = false;
      averagingSum_ = buf_[0];
      averagingSampleCount_= 1;
    }

    void stopAveraging() {
      if (averaging_ || autoAveraging_) {
        pushHold();
      }
      averaging_ = false;
      autoAveraging_ = false;
    }

    void pushHold() {
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
      Serial.print(F("HOLD: "));
      Serial.println(float_to_str(holdValue));
      hold_.push(holdValue);
    }

    bool isAveraging() {
      return averaging_;
    }

    bool isAutoAveraging() {
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


HX711 scale;
AveragingScale ascale(&scale, GRAMS_PER_RAW, /*wiredBackwards=*/true);


void setup() {
  delay(1000);
  Serial.begin(9600);

  Serial.println(F("-------------------------"));

  Serial.println(F("Starting display..."));


  display.begin();

  display.setFont(u8g2_font_profont22_tr);
  display.firstPage();
  do {
    display.setCursor(0, 14);
    display.println(F("Averaging "));
    display.setCursor(0, 14+22);
    display.println(F("  Scale"));
  } while( display.nextPage() );
  delay(1000);

  Serial.println(F("starting scale..."));
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  ascale.zero(1);
  Serial.println(F("scale started"));

  Serial.println(F("Watching button pins..."));
  watchDigitalPin(ZERO_BUTTON_PIN);
  watchDigitalPin(AVG_BUTTON_PIN);

  Serial.println(F("Setup complete."));
  Serial.println(F("-------------------------"));
}

void loop() {
  display.clear();

  while (true) {
    ascale.readRaw();

    display.firstPage();
    do {
      display.setFont(u8g2_font_profont22_tr);

      display.setCursor(0, 14);
      if (ascale.isAveraging()) {
        display.print("AVG");
      } else if (ascale.isAutoAveraging()) {
        display.print("AUT");
      } else {
        display.print("   ");
      }
      double value = ascale.getUnits();
      // Don't distract with tiny fluctuations around zero.
      if (!ascale.isAveraging() && value < 0.3 && value > -0.3) {
        value = 0.0;
      }
      display.setCursor(46, 14);
      display.print(float_to_str(value));

      display.setFont(u8g2_font_profont17_tr);
      for (int holdIndex = 0; holdIndex < 2 && holdIndex < ascale.hold_.size(); holdIndex++) {
        display.setCursor(0, 17 * (holdIndex+2));
        display.print("HLD");
        display.setCursor(65, 17 * (holdIndex+2));
        display.print(float_to_str(ascale.hold_[holdIndex]));
      }

      display.setFont(u8g2_font_profont12_tr);
      display.setCursor(0, 64);
      display.print("Instant:");
      display.setCursor(86, 64);
      display.print(float_to_str(ascale.getInstantUnits()));
      
    } while ( display.nextPage() );

    
    if (wasPressed(ZERO_BUTTON_PIN)) {
      Serial.print(F("ZERO was pressed"));
      Serial.println();
      ascale.setZero(ascale.getRaw());
    } else {
      ascale.trackZeroDrift();
    }

    if (wasPressed(AVG_BUTTON_PIN)) {
      Serial.print(F("AVG was pressed; averaging: "));
      Serial.println();
      if (!ascale.isAveraging()) {
        ascale.startAveraging();
      } else { 
        ascale.stopAveraging();
      }
    } else {
      ascale.autoAverage();
      ascale.autoAverageOff();
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
