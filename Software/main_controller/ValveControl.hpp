#pragma once
#include "BasicStructures.hpp"

//* Other fixed numbers
float VALVE_PWM_PERIOD_MS = 1000;

//! Implements proportional-integral control to adjust a quantity to a certain target value
//! This controller clips the outputs to the interval [0,1]
struct PIControl {
  double targetValue_;
  double Kp_, Ki_;
  
  unsigned long prevUpdateTime_;
  double prevError_, intError_;
  short saturation_; //the controller operates in the saturation region

  PIControl(double targetValue, double Kp, double Ki) :
    targetValue_(targetValue),
    Kp_(Kp),
    Ki_(Ki),
    prevUpdateTime_(0),
    intError_(0),
    saturation_(0){}
  
  double clipToUnitInterval(double const & x) {
    if (x < 0) {
      saturation_ = -1;
      return 0;
    }
    if (x > 1) {
      saturation_= 1;
      return 1;
    }
    saturation_= 0;
    return x;
  }

  double compute(double inputValue) {
    long currentTime = millis();
    double elapsed = (double) (currentTime - prevUpdateTime_);
    double error = targetValue_ - inputValue;
    if((saturation_ > 0 && error > 0) || (saturation_ < 0 && error < 0)) {
      //skip if saturation and error are in the same direction
    } else {
      intError_ += Ki_*error*elapsed;
    }
    intError_ = clipToUnitInterval(intError_);
    double out = clipToUnitInterval(Kp_*error + intError_);
    prevUpdateTime_ = currentTime;        
    return out;
  }  
};

struct MagneticValve {
  int pin_;
  float s_;//fraction of the pwm period to switch the valve on
  bool on_;
  MagneticValve(int pin):
    pin_(pin),
    s_(1),
    on_(false){}    

  Debouncer pwmPeriod_ = Debouncer(VALVE_PWM_PERIOD_MS);
  Debouncer onPeriod_ = Debouncer(s_*VALVE_PWM_PERIOD_MS);

  void open() {
    digitalWrite(pin_, true);
    on_ = true;
  }

  void close() {
    digitalWrite(pin_, false);
    on_ = false;
  }

  void setS(float newS) {
    if(newS >= 1 or newS <= 0){
      return;
    } 
    s_ = newS;
    onPeriod_.delay_=s_*VALVE_PWM_PERIOD_MS;  
  }
  
  void update() {
    if(!pwmPeriod_.check() &&  onPeriod_.check()) {
      if(!on_){
        return;
      } else {
        close();
        return;
      }
    }
    if(!on_){
      open();
    }
    return;
  }
};
