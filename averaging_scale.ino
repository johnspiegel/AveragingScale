/*********************************************************************
  This is an example for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

  This example is for a 128x64 size display using I2C to communicate
  3 pins are required to interface (2 I2C and one reset)

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada  for Adafruit Industries.
  BSD license, check license.txt for more information
  All text above, and the splash screen must be included in any redistribution
*********************************************************************/


/*
TODO
// - auto-zero if last five seconds are all abs(r) < 0.1g
// - auto average if every r from last second is > .9 * average of last second and < 1.1 * avg and avg of last second >= 1.0g
- auto reset avg if every r from last second is > .9 * (1s-avg - AVG) (or for going down too?) and
// - auto "hold" avg value if every measurement in last second is < AVG-from-before-last-second (and going to zero? or < .1 AVG?)

// - track buttons pushed but nothing found high

*/

#include <Wire.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "HX711.h"
#include "buttons.h"

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;
const int TARE_BUTTON_PIN = 12;
const int AVG_BUTTON_PIN = 11;

HX711 scale;

//float FACTOR = -152700.0 / 412.0;
constexpr float FACTOR = -135703.0 / 363.79;
constexpr float GRAMS_PER_RAW = 1 / FACTOR;
long TARE = 0.0;

float to_grams(long reading) {
  float g = reading * GRAMS_PER_RAW;
  if (abs(g) < 1.0) {
    return 0.0;
  }
  return g;
}

char* float_to_str(float f) {
  static char fbuf[12];
  return dtostrf(f, 6, 1, fbuf);
}

template <typename T, int CAPACITY>
class CircularBuffer {
  private:
    T buffer_[CAPACITY];
    int size_;
    int nextInsertIndex_;

  public:
    CircularBuffer() : buffer_{0}, size_(0), nextInsertIndex_(0) {}

    void push(T value) {
      buffer_[nextInsertIndex_] = value;
      nextInsertIndex_ = (nextInsertIndex_ + 1) % CAPACITY;
      if (size_ < CAPACITY) {
        size_++;
      }
    }

    int size() {
      return size_;
    }

    // Index zero is the most-recently-pushed value,
    // index 1 is the second-most-recently-pushed value, etc.
    T operator[](int index) {
      return buffer_[(CAPACITY + nextInsertIndex_ - 1 - index) % CAPACITY];
    }
};

class AveragingScale {
  public:
    HX711 *hx711_;  // Not owned!
    bool wiredBackwards_;
    int32_t zero_;
    double unitsPerRaw_;
    CircularBuffer<int32_t, 40> buf_;
    CircularBuffer<double, 5> hold_;

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

    void zero(byte sampleCount = 20) {
      while (buf_.size() < sampleCount) {
        readRaw();
      }
      AverageStats stats = getRawAverage(sampleCount);
      zero_ = getRawAverage(sampleCount).average;
      Serial.print(F("Zero!  zero_: "));
      Serial.print(zero_);
      Serial.print(" ");
      Serial.print(stats.sum);
      Serial.print(" ");
      Serial.print(stats.sampleCount);
      Serial.print(" ");
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

    int32_t getInstantUnits() {
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
        // hold_.push(getUnits());
        pushHold();
      }
      averaging_ = false;
      autoAveraging_ = false;
      // averagingSum_ = 0;
      // averagingSampleCount_= 1;
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

    void autoZero() {
      if (averaging_) {
        return;
      }

      const byte sampleCount = 20;  // 2s at 10 samples-per-second.
      AverageStats stats = getRawAverage(sampleCount);
      /*
      Serial.print(F("sum: "));
      Serial.print(sum);
      Serial.print(F(" max: "));
      Serial.print(stats.maxValue);
      Serial.print(F(" min: "));
      Serial.print(minValue);
      Serial.print(F(" avg: "));
      Serial.print(sum / sampleCount);
      Serial.print(F(" max - min: "));
      Serial.println( ((stats.maxValue - minValue) * unitsPerRaw_ ));
      */
      if ((stats.maxValue - stats.minValue) * unitsPerRaw_ > 1.0) {
        return;
      }
      // int32_t avg = sum / sampleCount;
      /*
      if ((stats.maxValue - avg) * unitsPerRaw_ > 1.0) {
        return INT32_MIN;
      };
      if ((avg - stats.minValue) * unitsPerRaw_ > 1.0) {
        return INT32_MIN;
      };
      */

      float diffUnits = (stats.average - zero_) * unitsPerRaw_;
      /*
      Serial.print(F("WTF"));
      Serial.print(F(" avg: "));
      Serial.print(stats.average);
      Serial.print(F(" zero: "));
      Serial.print(zero_);
      Serial.print(F(" zero-avg: "));
      Serial.print(zero_ - stats.average);
      Serial.print(F(" diffUnits: "));
      Serial.print(diffUnits);
      Serial.print(F(" unitsPerRaw_: "));
      Serial.print(unitsPerRaw_);
      Serial.println();
      */
      // Serial.print(F("TARE: "));
      // Serial.println(TARE);
      // Serial.print(F("diffUnits: "));
      // Serial.println(diffUnits);
      // Serial.print(F("> "));
      // Serial.print(diffUnits > 1.0);
      // Serial.print(F("< "));
      // Serial.println(diffUnits > 1.0);
      if (diffUnits > 1.0 || abs(diffUnits) < 0.5) {
          return;
      }

      Serial.print(F("AutoZero!  millis: "));
      Serial.print(millis());
      Serial.print(F(" unitsPerRaw_: "));
      Serial.print(unitsPerRaw_);
      Serial.print(F(" diffUnits: "));
      Serial.print(diffUnits);
      Serial.print(F(" old: "));
      Serial.print(zero_);
      Serial.print(F(" new: "));
      Serial.println(stats.average);
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

      Serial.println(F("Auto-Average! "));
      autoAveraging_ = true;
      averagingSum_ = stats.sum;
      averagingSampleCount_= stats.sampleCount;
      return true;
    }

    bool autoAverageOff() {
      if (averaging_ || !autoAveraging_) {
        return false;
      }

      const byte sampleCount = 5;
      AverageStats stats = getRawAverage(sampleCount);

    /*
      Serial.print(F("sum: "));
      Serial.print(sum);
      Serial.print(F(" max: "));
      Serial.print(maxValue);
      Serial.print(F(" min: "));
      Serial.print(minValue);
      Serial.print(F(" avg: "));
      Serial.print(avg);
      Serial.print(F(" avg_grams: "));
      Serial.print(avg * GRAMS_PER_RAW);
      Serial.println();
    */

      int32_t currentAverage = getRaw();
      if (stats.maxValue < 1.1 * currentAverage &&
          stats.minValue > 0.9 * currentAverage) {
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

void setup() {
  Serial.begin(9600);
  //Serial.begin(19200);
  delay(200);

  Serial.println(F("-------------------------"));
  Serial.println(F("starting scale..."));
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  Serial.println(F("scale started"));

  TARE = scale.read_average(5);
  Serial.print(F("scale TARE: "));
  Serial.println(TARE);

  Serial.println(F("Starting display..."));
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(WHITE, BLACK);
  display.print(F("Averaging Scale"));
  display.display();

  Serial.println(F("Watching button pins..."));
  watchDigitalPin(TARE_BUTTON_PIN);
  watchDigitalPin(AVG_BUTTON_PIN);

  Serial.println(F("Setup complete."));
  Serial.println(F("-------------------------"));
  return;
}

void buttonMeasure();
void buttonMeasureNew();

void loop() {
  buttonMeasureNew();
  // buttonMeasure();
  // avgMeasure();
  // loopMeasure();
}


long autoTare(long reads[], uint8_t readsSize, uint8_t lastReadIndex) {
  const byte avgCount = 30;

  long sum = 0;
  int32_t minValue = INT32_MAX;
  int32_t maxValue = INT32_MIN;
  for (int i = 0; i < avgCount; i++) {
    byte readIndex = (lastReadIndex - i) % readsSize;
    long r = reads[readIndex];
    // Serial.print(F("index: "));
    // Serial.print(i);
    // Serial.print(F("value: "));
    // Serial.println(r);
    if (r  == 0 ) {
      return INT32_MIN;
    }
    minValue = min(minValue, r);
    maxValue = max(maxValue, r);
    sum += r;
  }
  // Serial.print(F("sum: "));
  // Serial.print(sum);
  // Serial.print(F(" max: "));
  // Serial.print(maxValue);
  // Serial.print(F(" min: "));
  // Serial.print(minValue);
  // Serial.print(F(" avg: "));
  // Serial.print(sum / avgCount);
  // Serial.print(F(" max - min: "));
  // Serial.println( ((maxValue - minValue) * -GRAMS_PER_RAW ));
  if ((maxValue - minValue) * -GRAMS_PER_RAW > 1.0) {
    return INT32_MIN;
  }
  long avg = sum / avgCount;
  if ((maxValue - avg) * -GRAMS_PER_RAW > 1.0) {
    return INT32_MIN;
  };
  if ((avg - minValue) * -GRAMS_PER_RAW > 1.0) {
    return INT32_MIN;
  };

  float diffGrams = (TARE - avg) * -GRAMS_PER_RAW;
  // Serial.print(F("TARE: "));
  // Serial.println(TARE);
  // Serial.print(F("diffGrams: "));
  // Serial.println(diffGrams);
  // Serial.print(F("> "));
  // Serial.print(diffGrams > 1.0);
  // Serial.print(F("< "));
  // Serial.println(diffGrams > 1.0);
  if (diffGrams > 1.0 || abs(diffGrams) < 0.5) {
      return INT32_MIN;
  }

  return avg;
}

bool autoAverage(long reads[], uint8_t readsSize, uint8_t lastReadIndex, long* readingSum, long* readingCount) {
  const byte avgCount = 10;

  long sum = 0;
  int32_t minValue = INT32_MAX;
  int32_t maxValue = INT32_MIN;
  for (int i = 0; i < avgCount; i++) {
    byte readIndex = (lastReadIndex - i) % readsSize;
    long r = reads[readIndex];
    // Serial.print(F("index: "));
    // Serial.print(i);
    // Serial.print(F("value: "));
    // Serial.println(r);
    if (r  == 0 ) {
      return false;
    }
    r -= TARE;
    minValue = min(minValue, r);
    maxValue = max(maxValue, r);
    sum += r;
  }
  long avg = sum / avgCount;

/*
  Serial.print(F("sum: "));
  Serial.print(sum);
  Serial.print(F(" max: "));
  Serial.print(maxValue);
  Serial.print(F(" min: "));
  Serial.print(minValue);
  Serial.print(F(" avg: "));
  Serial.print(avg);
  Serial.print(F(" avg_grams: "));
  Serial.print(avg * GRAMS_PER_RAW);
  Serial.println();
*/

  if (-maxValue < 0.95 * -avg) {
    return false;
  }
  if (-minValue > 1.05 * -avg) {
    return false;
  }
  if (avg * GRAMS_PER_RAW < 1.0) {
    return false;
  }

  Serial.println(F("Auto-Average! "));
  *readingSum = sum;
  *readingCount = avgCount;
  return true;
}

bool autoAverageOff(long reads[], uint8_t readsSize, uint8_t lastReadIndex, long currentAverage) {
  const byte avgCount = 10;

  long sum = 0;
  int32_t minValue = INT32_MAX;
  int32_t maxValue = INT32_MIN;
  for (int i = 0; i < avgCount; i++) {
    byte readIndex = (lastReadIndex - i) % readsSize;
    long r = reads[readIndex];
    // Serial.print(F("index: "));
    // Serial.print(i);
    // Serial.print(F("value: "));
    // Serial.println(r);
    if (r  == 0 ) {
      return false;
    }
    r -= TARE;
    minValue = min(minValue, r);
    maxValue = max(maxValue, r);
    sum += r;
  }
  // long avg = sum / avgCount;

/*
  Serial.print(F("sum: "));
  Serial.print(sum);
  Serial.print(F(" max: "));
  Serial.print(maxValue);
  Serial.print(F(" min: "));
  Serial.print(minValue);
  Serial.print(F(" avg: "));
  Serial.print(avg);
  Serial.print(F(" avg_grams: "));
  Serial.print(avg * GRAMS_PER_RAW);
  Serial.println();
*/

  if (-maxValue > 0.9 * -currentAverage &&
      -minValue < 1.1 * -currentAverage) {
    return false;
  }

  Serial.println(F("Auto-Average Off! "));
  return true;
}

void buttonMeasureNew() {
  AveragingScale ascale(&scale, -GRAMS_PER_RAW, /*wiredBackwards=*/true);
  ascale.zero(5);

  display.clearDisplay();

  /*
  long readingSum = 0;
  long readingCount = 0;
  bool averaging = false;
  */
  bool enableAutoAverage = true;
  /*
  float holdValue1 = 0.0;
  float holdValue2 = 0.0;

  constexpr uint8_t readsSize = 10;
  long reads[readsSize] = {0};
  byte nextReadIndex = 0; 
  */

  while (true) {
    ascale.readRaw();
    /*
    long r = -ascale.readRaw();
    reads[nextReadIndex] = r;
    uint8_t lastReadIndex = nextReadIndex;
    nextReadIndex = (nextReadIndex + 1) % readsSize;
    r -= TARE;
    
    long raw_value = 0;
    */
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 0);
    
    if (ascale.isAveraging()) {
      display.print("AVG ");
    } else {
      display.print("    ");
    }
    /*
    if (!ascale.isAveraging()) {
      // raw_value = -ascale.get();
      display.print("    ");
    } else {
      // readingSum += r;
      // readingCount++;
      // raw_value = readingSum / readingCount;
      display.print("AVG ");
    }
    */
      // raw_value = -ascale.get();
    // display.print(float_to_str(to_grams(raw_value)));
    display.print(float_to_str(ascale.getUnits()));
      /*
      Serial.print(F("getUnits() "));
      Serial.print(ascale.getUnits());
      Serial.print(F(" get(): "));
      Serial.print(ascale.get());
      Serial.print(F(" getRaw(): "));
      Serial.print(ascale.getRaw());
      Serial.println();
      */
    display.print(" ");

    for (int holdIndex = 0; holdIndex < 2 && holdIndex < ascale.hold_.size(); holdIndex++) {
      display.setTextSize(2);
      display.setTextColor(WHITE, BLACK);
      display.setCursor(0, 16 * (holdIndex+1));
      display.print("HLD ");
      display.print(float_to_str(ascale.hold_[holdIndex]));
        // Serial.print(F("HOLD: "));
        // Serial.println(float_to_str(ascale.hold_[holdIndex]));
    }
    /*
    if (holdValue1 != 0.0) {
      display.setTextSize(2);
      display.setTextColor(WHITE, BLACK);
      display.setCursor(0, 16);
      display.print("HLD ");
      display.print(float_to_str(holdValue1));
    }

    if (holdValue2 != 0.0) {
      display.setTextSize(2);
      display.setTextColor(WHITE, BLACK);
      display.setCursor(0, 32);
      display.print("HLD ");
      display.print(float_to_str(holdValue2));
    }
    */

    display.setTextSize(1);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 56);
    display.print("Instant:     ");
    display.print(float_to_str(ascale.getInstantUnits()));
    
    display.display();
    
    if (wasPressed(TARE_BUTTON_PIN)) {
      long newTare = ascale.getRaw(); // raw_value + TARE;
      Serial.print(F("TARE was pressed"));
      Serial.println();
      /*
      Serial.print(TARE);
      Serial.print(F(" new: "));
      Serial.println(newTare);
      readingSum = readingSum - readingCount *(-newTare - TARE);
      TARE = -newTare;
      averaging = false;
      enableAutoAverage = true;
      */

      // ascale.setZero(ascale.getRaw());
      ascale.setZero(newTare);

    } else {
      // long start = micros();
      ascale.autoZero();
      /*
      long tareValue = autoTare(reads, readsSize, lastReadIndex);
      if (tareValue != INT32_MIN) {
        Serial.print(F("AutoTare! millis: "));
        Serial.print(millis());
        Serial.print(F(" old: "));
        Serial.print(TARE);
        Serial.print(F(" new: "));
        Serial.println(tareValue);
        TARE = tareValue;
      }
      // long end = micros();
      // Serial.print(F("AutoTare micros: "));
      // Serial.println(end - start);
      */
    }
    if (wasPressed(AVG_BUTTON_PIN)) {
      // averaging = !averaging;
      Serial.print(F("AVG was pressed; averaging: "));
      Serial.println();
      // Serial.println(averaging);
      /*
      if (!averaging) {
        holdValue2 = holdValue1;
        holdValue1 = ascale.getUnits(); // to_grams(raw_value);
        Serial.print(F("HOLD: "));
        Serial.println(float_to_str(holdValue1));
        enableAutoAverage = true;
      } else {
        enableAutoAverage = false;
      }
      readingSum = 0;
      readingCount = 0;
      */

      if (!ascale.isAveraging()) {
        ascale.startAveraging();
      } else { 
        ascale.stopAveraging();
      }

    } else { //if (averaging && enableAutoAverage) {

      if (ascale.autoAverageOff()) {

        // averaging = false;
      }

      /*
      if (autoAverageOff(reads, readsSize, lastReadIndex, raw_value)) {
        ascale.stopAveraging();
        averaging = false;
        float avg_grams = to_grams(readingSum / readingCount);
        long newReadingSum = readingSum;
        long newReadingCount = readingCount;
        float minDelta = avg_grams;
        for (int i = 0; i < readsSize; i++) {
          byte readIndex = (lastReadIndex - i) % readsSize;
          long r = reads[readIndex];

          readingSum -= (r - TARE);
          readingCount--;
          float new_avg_grams = to_grams(readingSum / readingCount);
          float delta = abs(new_avg_grams - avg_grams);
          if (delta < minDelta) {
            Serial.print(F("Maybe HOLD AVG: "));
            Serial.print(new_avg_grams);
            Serial.print(F(" readingCount: "));
            Serial.println(readingCount);
            newReadingSum = readingSum;
            newReadingCount = readingCount;
            minDelta = delta;
          }
          avg_grams = new_avg_grams;
        }
        readingSum = newReadingSum;
        readingCount = newReadingCount;
        holdValue2 = holdValue1;
        holdValue1 = to_grams(readingSum/readingCount);
        Serial.print(F("HOLD: "));
        Serial.println(float_to_str(holdValue1));
      }
      */
    // } else { // if (!averaging && enableAutoAverage) {
      /*
      averaging = autoAverage(reads, readsSize, lastReadIndex, &readingSum, &readingCount);
      if (averaging) {
        */
      if (!ascale.autoAverage()) {
        // averaging = false;
      }
        // ascale.startAveraging();
      // }
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

void buttonMeasure() {
  display.clearDisplay();

  long readingSum = 0;
  long readingCount = 0;
  bool averaging = false;
  bool enableAutoAverage = true;
  float holdValue1 = 0.0;
  float holdValue2 = 0.0;

  constexpr uint8_t readsSize = 40;
  long reads[readsSize] = {0};
  byte nextReadIndex = 0; 

  while (true) {
    long r = scale.read();
    reads[nextReadIndex] = r;
    uint8_t lastReadIndex = nextReadIndex;
    nextReadIndex = (nextReadIndex + 1) % readsSize;
    r -= TARE;
    
    long raw_value = 0;
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 0);
    
    if (!averaging) {
      raw_value = r;
      display.print("    ");
    } else {
      readingSum += r;
      readingCount++;
      raw_value = readingSum / readingCount;
      display.print("AVG ");
    }
    display.print(float_to_str(to_grams(raw_value)));
    display.print(" ");

    if (holdValue1 != 0.0) {
      display.setTextSize(2);
      display.setTextColor(WHITE, BLACK);
      display.setCursor(0, 16);
      display.print("HLD ");
      display.print(float_to_str(holdValue1));
    }

    if (holdValue2 != 0.0) {
      display.setTextSize(2);
      display.setTextColor(WHITE, BLACK);
      display.setCursor(0, 32);
      display.print("HLD ");
      display.print(float_to_str(holdValue2));
    }

    display.setTextSize(1);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 56);
    display.print("Instant:     ");
    display.print(float_to_str(to_grams(r)));
    
    display.display();
    
    if (wasPressed(TARE_BUTTON_PIN)) {
      long newTare = raw_value + TARE;
      Serial.print(F("TARE was pressed. Old: "));
      Serial.print(TARE);
      Serial.print(F(" new: "));
      Serial.println(newTare);
      readingSum = readingSum - readingCount *(newTare - TARE);
      TARE = newTare;
      averaging = false;
      enableAutoAverage = true;

    } else {
      // long start = micros();
      long tareValue = autoTare(reads, readsSize, lastReadIndex);
      if (tareValue != INT32_MIN) {
        Serial.print(F("AutoTare! millis: "));
        Serial.print(millis());
        Serial.print(F(" old: "));
        Serial.print(TARE);
        Serial.print(F(" new: "));
        Serial.println(tareValue);
        TARE = tareValue;
      }
      // long end = micros();
      // Serial.print(F("AutoTare micros: "));
      // Serial.println(end - start);
    }
    if (wasPressed(AVG_BUTTON_PIN)) {
      averaging = !averaging;
      Serial.print(F("AVG was pressed; averaging: "));
      Serial.println(averaging);
      if (!averaging) {
        holdValue2 = holdValue1;
        holdValue1 = to_grams(raw_value);
        Serial.print(F("HOLD: "));
        Serial.println(float_to_str(holdValue1));
        enableAutoAverage = true;
      } else {
        enableAutoAverage = false;
      }
      readingSum = 0;
      readingCount = 0;
    } else if (averaging && enableAutoAverage) {
      if (autoAverageOff(reads, readsSize, lastReadIndex, raw_value)) {
        averaging = false;
        float avg_grams = to_grams(readingSum / readingCount);
        long newReadingSum = readingSum;
        long newReadingCount = readingCount;
        float minDelta = avg_grams;
        for (int i = 0; i < readsSize; i++) {
          byte readIndex = (lastReadIndex - i) % readsSize;
          long r = reads[readIndex];

          readingSum -= (r - TARE);
          readingCount--;
          float new_avg_grams = to_grams(readingSum / readingCount);
          float delta = abs(new_avg_grams - avg_grams);
          if (delta < minDelta) {
            Serial.print(F("Maybe HOLD AVG: "));
            Serial.print(new_avg_grams);
            Serial.print(F(" readingCount: "));
            Serial.println(readingCount);
            newReadingSum = readingSum;
            newReadingCount = readingCount;
            minDelta = delta;
          }
          avg_grams = new_avg_grams;
        }
        readingSum = newReadingSum;
        readingCount = newReadingCount;
        holdValue2 = holdValue1;
        holdValue1 = to_grams(readingSum/readingCount);
        Serial.print(F("HOLD: "));
        Serial.println(float_to_str(holdValue1));
      }
    } else if (!averaging && enableAutoAverage) {
      averaging = autoAverage(reads, readsSize, lastReadIndex, &readingSum, &readingCount);
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


void avgMeasure() {
  long m[10] = {0};
  byte mi = 0;

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE, BLACK);


  while (true) {
    m[mi] = 0;
    for (int sample_i = 0; sample_i < 10; sample_i++) {

      long r = scale.read();
      if (wasPressed(TARE_BUTTON_PIN)) {
        Serial.println(F("TARE was pressed"));
        TARE = r;
      }
      r -= TARE;
      if (wasPressed(AVG_BUTTON_PIN)) {
        Serial.println(F("AVG was pressed"));
        TARE = r;
      }

      display.setCursor(0, 0);
      display.print("    ");
      display.print(float_to_str(to_grams(r)));
      display.display();
      m[mi] += r;
    }
    m[mi] /= 10;
    display.setCursor(0, 16);
    display.print("1s: ");
    display.print(float_to_str(to_grams(m[mi])));

    long sum = 0;
    for (int i = 0; i < 3; i++) {
      sum += m[(10 + mi - i) % 10];
    }
    sum /= 3;
    display.setCursor(0, 32);
    display.print("3s: ");
    display.print(float_to_str(to_grams(sum)));

    sum = 0;
    for (int i = 0; i < 10; i++) {
      sum += m[(10 + mi - i) % 10];
    }
    sum /= 10;
    display.setCursor(0, 48);
    display.print("10s:");
    display.print(float_to_str(to_grams(sum)));
    display.display();

    mi = (mi + 1) % 10;
  }


}

void loopMeasure() {
  const byte NUM_LINES = 4;
  char linebuf[NUM_LINES][12] = {0};
  byte next_line = 0;

  for (byte i = 0; true; i = (i + 1) % 100, next_line = (next_line + 1) % NUM_LINES) {
    Serial.print(F("i: "));
    Serial.print(i);
    Serial.print(F("  millis: "));
    Serial.println(millis());

    long reading = (scale.read_average(50) - TARE);
    float rf = reading * GRAMS_PER_RAW;

    Serial.println(F("HX711: raw, g"));
    Serial.print(F("HX711 values: "));
    Serial.print(reading);
    Serial.print(F(", "));
    Serial.println(rf);

    char *buf = linebuf[next_line];
    char fbuf[8];
    dtostrf(rf, 6, 1, fbuf);
    snprintf(buf, sizeof(linebuf[0]), "%2d %sg", i % 100, fbuf);

    // Refresh display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    for (int l = 0; l < NUM_LINES; ++l) {
      Serial.println(linebuf[(next_line + l + 1) % NUM_LINES]);
      display.println(linebuf[(next_line + l + 1) % NUM_LINES]);
    }
    display.display();
  }
}
