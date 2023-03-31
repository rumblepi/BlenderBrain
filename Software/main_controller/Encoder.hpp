#pragma once
#include "BasicStructures.hpp"
#include "UI.hpp"
#include "Setpoint.hpp"

enum GasType{
  Helium,
  Oxygen
};


struct ButtonEncoder {
  ButtonEncoder(int clkPin, int dtPin, int buttonPin):
    clk_(clkPin),
    dt_(dtPin),
    state_(0),
    fixed_(false),
    button_(DebouncedButton(buttonPin)){}

  float read(float prev) {
    bool clockVal = digitalRead(clk_);
    bool dataVal = digitalRead(dt_);
    switch (state_) {
        case 0:                         // Idle state, encoder not turning
            if (!clockVal){             // Turn clockwise and CLK goes low first
                state_ = 1;
            } else if (!dataVal) {      // Turn anticlockwise and DT goes low first
                state_ = 4;
            }
            break;
        // Clockwise rotation
        case 1:                     
            if (!dataVal) {             // Continue clockwise and DT will go low after CLK
                state_ = 2;
            } 
            break;
        case 2:
            if (clockVal) {             // Turn further and CLK will go high first
                state_ = 3;
            }
            break;
        case 3:
            if (clockVal && dataVal) {  // Both CLK and DT now high as the encoder completes one step clockwise
                state_ = 0;
                ++prev;
            }
            break;
        // Anticlockwise rotation
        case 4:                         // As for clockwise but with CLK and DT reversed
            if (!clockVal) {
                state_ = 5;
            }
            break;
        case 5:
            if (dataVal) {
                state_ = 6;
            }
            break;
        case 6:
            if (clockVal && dataVal) {
                state_ = 0;
                --prev;
            }
            break; 
    }
    return prev;
  }


  void updateSetpoint(Setpoint & sp, GasType gas) {
    float currSp;
    switch(gas) {
      case GasType::Helium:
        currSp = sp.hePercent_.get();
        currSp = read(currSp);
        sp.setHePercent(currSp);
        break;
      case GasType::Oxygen:
        currSp = sp.oxyPercent_.get();
        currSp = read(currSp);
        sp.setOxyPercent(currSp);
        break;
      default:
        return;
    }
  }

  int clk_, dt_;
  bool fixed_;
  unsigned int state_;
  DebouncedButton button_;
};