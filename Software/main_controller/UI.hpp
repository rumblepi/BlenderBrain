#pragma once
#include "LedControl.h"
#include "BasicStructures.hpp"
#include "Display.hpp"
#include "Encoder.hpp"

// Structure for the user interface
struct SingleOption {
  SingleOption() {
    dispName_ = "";
    value_ = 0;
  }
  SingleOption(char* name, int val):
    dispName_(name),
    value_(val)
    {}
  
  SingleOption(char* name, int val, int min, int max):
    dispName_(name),
    value_(val)
    {
      value_.minimax(min, max);
    }

  void show(EightDigitDisplay & display) {
    if(!displayDebounce.check()) return;
    display.clear();
    display.writeText(dispName_, 4);
    display.writeTwoDigits(value()/100, 3);
    display.writeTwoDigits(value(),4);
  }

  void value(ClippedFloat newClippedVal) {
    value_ = newClippedVal;
  }

  void value(int newVal) {
    value_.set(newVal);
  }

  int value() const {
    int intVal = static_cast<int>(value_.get());
    return intVal;    
  }
  
  char* dispName_;
  ClippedFloat value_;  
  Debouncer displayDebounce = Debouncer(50);
};

struct MixMenu {

  int activeVal_ = 0;
  const int nOptions_ = 6;

  SingleOption pPrev;
  SingleOption pPost;
  SingleOption hePrev;
  SingleOption o2Prev;
  SingleOption hePost;
  SingleOption o2Post;

  void
  setPossibleHeContent() {
    float prevFraction = pPrev.value_.get() / pPost.value_.get();
    hePost.value_.minimum(hePrev.value()*prevFraction);
    hePost.value_.maximum(100*(1-prevFraction) + hePrev.value()*prevFraction);    
  }

  void
  setPossibleO2Content() {
    float prevFraction = pPrev.value_.get() / pPost.value_.get();
    float remainingPercent = 1 - hePost.value_.get()/100.0;
    o2Post.value_.minimum((21*(1-prevFraction)+o2Prev.value()*prevFraction)*remainingPercent);
    o2Post.value_.maximum((40/remainingPercent*(1-prevFraction)+o2Prev.value()*prevFraction)*remainingPercent);
  }

  MixMenu() {
    pPrev = SingleOption("P  0", 100);
    pPost = SingleOption("P  1", 225);
    hePrev = SingleOption("HE 0", 0);
    o2Prev = SingleOption("02 0", 21);
    hePost = SingleOption("HE 1", 35);
    o2Post = SingleOption("02 1", 21);
    pPrev.value_.maximum(225);
    pPrev.value_.set(100);
    pPost.value_.maximum(225);
    pPost.value_.set(220);
    o2Post.value_.maximum(40);
  }

  void
  reset() {
    activeVal_ = 0;
  }  

  SingleOption & getActive() {
    switch(activeVal_) {  
      case 0:
        return pPrev;
      case 1:
        return hePrev;
      case 2:
        return o2Prev;
      case 3:
        pPost.value_.minimum(pPrev.value());
        return pPost;
      case 4:
        setPossibleHeContent();
        return hePost;
      case 5:
        setPossibleO2Content();
        return o2Post;
      default:
        return o2Post;
    }
  }

  bool
  next() {
    activeVal_++;
    if(activeVal_ < nOptions_) {
      return false;
    } else {
      return true;
    }
  }  

  void
  display(EightDigitDisplay & disp) {
    getActive().show(disp);
  }
  
};


struct MainMenu {
  void
  clip() {
    activeVal_ %= numEntries;
  }  

  void
  update(ButtonEncoder & enc) {
    activeVal_ = enc.read(activeVal_);
    clip();
  }

  void
  display(EightDigitDisplay & disp) {
    if(!displayDebouncer.check()) return;
    switch(activeVal_) {
      case 0:
        disp.clear();
        disp.writeText("StArt", 5);
        break;
      case 1:
        disp.clear();
        disp.writeText("SEt", 3);
        break;
    }
  }
  int
  get() {
    return activeVal_;
  }  
  Debouncer displayDebouncer = Debouncer(100);
  int activeVal_ = 0;
  int numEntries = 2;
};


struct SetMenu {

  int activeVal_ = 0;
  const int nOptions_ = 6;

  SingleOption hePWM;
  SingleOption o2PWM;
  SingleOption heKp;
  SingleOption o2Kp;
  SingleOption heKi;
  SingleOption o2Ki;

  SetMenu() {
    hePWM = SingleOption("Hint", 2500, 1000, 9999);
    o2PWM = SingleOption("Oint", 2500, 1000, 9999);
    heKp = SingleOption("HE P", 100, 1,9999);
    o2Kp = SingleOption("02 P", 100,1,9999); // divide by 10000, display with dot
    heKi = SingleOption("HE i", 10,1,9999);
    o2Ki = SingleOption("02 i", 10,1,9999);
  }

  void
  reset() {
    activeVal_ = 0;
  }  

  SingleOption & getActive() {
    switch(activeVal_) {  
      case 0:
        return hePWM;
      case 1:
        return o2PWM;
      case 2:
        return heKp;
      case 3:
        return o2Kp;
      case 4:
        return heKi;
      case 5:
        return o2Ki;
      default:
        return o2Ki;
    }
  }

  bool
  next() {
    activeVal_++;
    if(activeVal_ < nOptions_) {
      return false;
    } else {
      return true;
    }
  }  

  void
  display(EightDigitDisplay & disp) {
    getActive().show(disp);
  }
  
};
