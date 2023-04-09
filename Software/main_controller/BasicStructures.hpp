#pragma once

struct ClippedFloat {
    
  ClippedFloat(float val = 0, float min = 0, float max = 99):
    val_(val),
    min_(min),
    max_(max) {}

  //! Clip a given floating point number to a minimum and maximum value
  static float 
  clip(float number, float min, float max) {
    if (number < min) return min;
    if (number > max) return max;
    return number;  
  }

  void
  set(float val) {
    val_ = clip(val, min_, max_);
  }

  void
  minimum(float newMin) {
    if(newMin > max_) {
      return *this;
    }
    min_ = newMin;
    set(val_);
  }

  void
  maximum(float newMax) {
    if(newMax < min_) {
      return;
    }
    max_ = newMax;
    set(val_);
  }

  void
  minimax(float newMin, float newMax) {
    minimum(newMin);
    maximum(newMax);
  }

  float get() const {
    return val_;
  }

  float val_;
  float min_;
  float max_;
};


/*! The Debouncer object can be used to wait a fixed time after an event
 *  while still continuing with all tasks (i.e. without having to use a
 *  delay() method)
 */
struct Debouncer {
  Debouncer(float delay = 1.):
    delay_(delay),
    lastDebounce_(0){}

  //! returns true if the delay has elapsed since the last call
  bool check() {
    if(millis() - lastDebounce_ > delay_) {
      lastDebounce_ = millis();
      return true;
    }
    return false;
  }
  
  float delay_;
  long lastDebounce_;
};


enum ButtonState{
  Down,
  Up  
};

/*! The DebouncedButton makes use of the Debouncer class to 
 * stabilize bouncing button pushes
 */
struct DebouncedButton {
  Debouncer debounce_ = Debouncer(0.05);

  DebouncedButton(int pin, long time = 0) :
    pin_(pin),
    currentState_(ButtonState::Up)    
  {}

  //! Returns +1 if the button changed from 'Down' to 'Up', -1 if the button changed from
  //! 'Up' to 'Down' and 0 if the state did not change.
  int stateChanged() {
      if(debounce_.check()){
        ButtonState pinState = digitalRead(pin_)? ButtonState::Up : ButtonState::Down;
        if (pinState == ButtonState::Down && currentState_== ButtonState::Up) {
          currentState_ = pinState;
          return 1;
        } else if (pinState == ButtonState::Up && currentState_== ButtonState::Down) {
          currentState_ = pinState;
          return -1; 
        } else {
          return 0;
        }
      }
      return 0;
  }   

  bool pressed() {
    if (stateChanged() != 0 && currentState_ == ButtonState::Up) {
      return true;
    }
    return false;
  }

  int pin_;
  ButtonState currentState_;
};
