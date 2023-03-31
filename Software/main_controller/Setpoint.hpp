#pragma once
#include "BasicStructures.hpp"
#include "Sensor.hpp"

struct Setpoint {
  Debouncer displayDebounce_ = Debouncer(0.1);

  Setpoint(float oxy = 20.95, float he = 0.0) :
    hePercent_(ClippedFloat(he,0,99)),
    oxyPercent_(ClippedFloat(oxy,0,99)){}  

  void setHePercent(float newHe) {
    hePercent_.set(newHe);
    oxyPercent_.minimum(20.95*(1-hePercent_.get()/100.));
  }
  
  void setOxyPercent(float newOxy) {
    oxyPercent_.set(newOxy);
  }

  void computeFromSensors(Sensor oxySensor, Sensor heSensor) {
    oxyPercent_.set(oxySensor.lastValue_.get());
    hePercent_.set(100*(1-heSensor.lastValue_.get()/21)*(1-(oxySensor.lastValue_.get()-heSensor.lastValue_.get())/(100-heSensor.lastValue_.get())));
  }

  float computeHeSensorValue() {
    return 100 - 79.05*(100-oxyPercent_.get())/(100-oxyPercent_.get()-0.2095*hePercent_.get());
  }
  
  void display(EightDigitDisplay & display, int pos) {
    if(!displayDebounce_.check())return;
    //draw at position 0 or 1 [0000|1111]
    switch(pos){
      case 0:
        display.writeTwoDigits(round(oxyPercent_.get()), 1);
        display.writeTwoDigits(round(hePercent_.get()), 2);
        break;
      case 1:
        display.writeTwoDigits(round(oxyPercent_.get()), 3);
        display.writeTwoDigits(round(hePercent_.get()), 4);
        break;
      default:
        return;     
    }
  }  
  ClippedFloat hePercent_;
  ClippedFloat oxyPercent_;
};
