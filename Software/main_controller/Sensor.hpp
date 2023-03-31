#pragma once
#include "BasicStructures.hpp"
#include "UI.hpp"

struct Sensor {
  Sensor(int pin, float calibValue):
    pin_(pin)
    {
      lastValue_ = ClippedFloat(calibValue,0,99);
      calibration_ = calibValue/measureAverage(30);
    }

  void recalibrate(float calibValue) {
      calibration_ = calibValue/measureAverage(30);
  }

  float measureAverage(int num) {
    float output = 0;
    for(int i = 0; i < num; ++i) {
      output += analogRead(pin_);
    }
    return output /(float)num;
  }

  void update() {
    lastValue_.set(measureAverage(150)*calibration_);
  }
    
  void display(EightDigitDisplay display, int pos) {
    display.writeTwoDigits((int) round(lastValue_.get()), pos);
  }

  int pin_;
  float calibration_;
  ClippedFloat lastValue_;
};
