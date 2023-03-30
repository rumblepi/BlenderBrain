/*
 Main Control
 This file contains the main control logic uploaded to the Atmega168 for gas blending using the BlenderBrain control
 (c) Mar 28, 2023 Michael Sandbichler
*/
#include "LedControl.h"

///////////////////////////////////////////////////////////////////
// Pins etc

// Define all pins and objects according to the schematic 
// given in the 'Hardware' folder
// `* Sensors and Actuators
#define OXY_SENSOR_PIN A5
#define HE_SENSOR_PIN A4
#define HE_VALVE_PIN A0
#define OXY_VALVE_PIN A1

#define RED_LIGHT_PIN A2
#define GREEN_LIGHT_PIN A3

#define DISPLAY_DIN_PIN 9
#define DISPLAY_CS_PIN 8
#define DISPLAY_CLK_PIN 10

#define RESET_BUTTON_PIN 0

//* Encoders
#define ENCODER_CLK 6
#define ENCODER_DT 7
#define ENCODER_SW 3

//* Other fixed numbers
float VALVE_PWM_PERIOD_MS = 1000;

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

  float get() const {
    return val_;
  }

  float val_;
  float min_;
  float max_;
};

enum ErrorCode {
  //! Exit codes for different errors
  ImpossibleMix  
};

enum MixMode{
  //! For Nitrox and Heliox only one valve is on, for Trimix both valves inject gas
  Nitrox,
  Heliox,
  Trimix
};

enum GasType{
  Helium,
  Oxygen
};

enum ButtonState{
  Down,
  Up  
};

/*! The Debouncer object can be used to wait a fixed time after an event
 *  while still continuing with all tasks (i.e. without having to use a
 *  delay() method)
 */
struct Debouncer {
  Debouncer(float delay):
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
    if (stateChanged() != 0 && currentState_ == ButtonState::Down) {
      return true;
    }
    return false;
  }

  int pin_;
  ButtonState currentState_;
};

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

struct StatusLed {
  enum LedState {
    On,
    Off,
    Blinking
  };
  int pin_;
  LedState status_;
  Debouncer blinker_ = Debouncer(1000);
  Debouncer secondBlink_ = Debouncer(500);

  StatusLed(int pin):
    pin_(pin),
    status_(LedState::Off) {}
  
  void update(){
    switch(status_) {
      case LedState::On:
        digitalWrite(pin_, true);
        break;
      case LedState::Off:
        digitalWrite(pin_, false);
        break;
      case LedState::Blinking:
        if (!blinker_.check() && !secondBlink_.check()){
            digitalWrite(pin_,true);
          } else {
            digitalWrite(pin_,false);
          }
        break;
    }
  }
};


struct EightDigitDisplay {
  LedControl lc_;
  EightDigitDisplay(LedControl lc) :
    lc_(lc) {
      lc_.shutdown(0,false);
      /* Set the brightness to a medium values */
      lc_.setIntensity(0,2);
      /* and clear the display */
      lc_.clearDisplay(0);      
  }

  void clear() {
    lc_.clearDisplay(0);
  }  

  void writeNumber(int number) {
    // Writes the first eight digits of a given number to the display
    for(int i=0; i<8; ++i) {
      lc_.setDigit(0,i,(number/((int) pow(10,i))) % 10,false);
    }
  }
  
  void writeText(char* txt, int len) {
    if (len > 8) return;
    //The length of txt must not be larger than 'len' characters and will be displayed at the left
    for(int i = 0; i < len; ++i) {
      if(txt[i]=='t'){
        lc_.setLed(0,7-i,4,true);  
        lc_.setLed(0,7-i,5,true);  
        lc_.setLed(0,7-i,6,true);  
        lc_.setLed(0,7-i,7,true);       
      } else if(txt[i]=='S'){
        lc_.setDigit(0,7-i,5,false);
      } else if(txt[i]=='r'){
        lc_.setLed(0,7-i,5,true);  
        lc_.setLed(0,7-i,7,true);  
      } else {
        lc_.setChar(0,7-i,txt[i],false);
      }
    }
  }

  void writeTwoDigits(int number, int pos) {
    //write two digits at position 1,2,3 or 4 of the display |11|22|33|44|
    lc_.setDigit(0,7-2*(pos-1),(number/10) % 10,false);
    lc_.setDigit(0,6-2*(pos-1),number % 10,false);
  }

};

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
    lastValue_.set(measureAverage(20)*calibration_);
  }
    
  void display(EightDigitDisplay display, int pos) {
    display.writeTwoDigits((int) round(lastValue_.get()), pos);
  }

  int pin_;
  float calibration_;
  ClippedFloat lastValue_;
};

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

  void show(EightDigitDisplay & display) {
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

  int activeVal_ = 0;
  int numEntries = 2;
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
        return o2Prev;
      case 2:
        return hePrev;
      case 3:
        return pPost;
      case 4:
        return o2Post;
      case 5:
        return hePost;
      default:
        return hePost;
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

struct MixSetup {
  bool
  updateFromMixMenu(MixMenu settings) {
    pressureBefore = settings.pPrev.value();
    pressureAfter = settings.pPost.value();
    mixBefore = Setpoint(settings.o2Prev.value(), settings.hePrev.value());
    mixAfter = Setpoint(settings.o2Post.value(), settings.hePost.value());
    return computeRequiredMix();
  }

  bool 
  computeRequiredMix() {
    return false;
  }  

  int pressureBefore;
  int pressureAfter;
  Setpoint mixBefore;
  Setpoint mixAfter; 
  Setpoint mixRequired;  
 
};

///////////////////////////////////////////////////////////////////
// globals
//* Global variables
float oxySetPoint = 21.5;
int oldClockOxy = -1; 
bool oxyFixed = false;

float heSetPoint = 0.5;
int oldClockHe = -1;
bool heFixed = false;

float oxySensorCalib = 0;
float oxySensorValue = 20.95;
float lastOxyValue = 20.95;

ButtonEncoder encoder = ButtonEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW);
DebouncedButton reset = DebouncedButton(RESET_BUTTON_PIN);

EightDigitDisplay display = EightDigitDisplay(LedControl(DISPLAY_DIN_PIN,DISPLAY_CLK_PIN,DISPLAY_CS_PIN,1));

StatusLed redLed = StatusLed(RED_LIGHT_PIN);
StatusLed greenLed = StatusLed(GREEN_LIGHT_PIN);

Sensor oxySensor = Sensor(OXY_SENSOR_PIN, 20.95);
Sensor heSensor = Sensor(HE_SENSOR_PIN, 20.95);

PIControl oxyControl = PIControl(20.95,0.01,0.001);
PIControl heControl = PIControl(0.0,0.01,0.001);

MagneticValve oxyValve = MagneticValve(OXY_VALVE_PIN);
MagneticValve heValve = MagneticValve(HE_VALVE_PIN);

Setpoint measuredMix;

MainMenu mainMenu;
MixMenu mixMenu;
bool inMain = true;
bool mixing = false;

MixSetup mixSetup;

//////////////////////////////////////////////////////////////////
// Basic microcontroller methods

//TODO rebuild with new objects

void setup() {  
  Serial.begin(9600); 
  pinMode(OXY_SENSOR_PIN, INPUT);
  pinMode(HE_SENSOR_PIN, INPUT);

  pinMode(OXY_VALVE_PIN, OUTPUT);
  pinMode(HE_VALVE_PIN, OUTPUT);

  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  //wantedSP.oxyPercent_.minimum(20.95);
  //wantedSP.oxyPercent_.maximum(40.0);
  delay(1000);
  oxySensor.recalibrate(20.95);
  heSensor.recalibrate(20.95);
  //currentSP.displayDebounce_.delay_ = 100;
}

void loop() {
#ifdef DEBUGMODE
  debugDisplay();
  delay(1000);
#else
  if (reset.pressed()) {
    mixing = false;
    inMain = true;
    mixMenu.reset();
    oxyValve.close();
    heValve.close();
  }
  if(mixing) {
    //TODO add display and PI control of valves
    mixSetup.mixRequired.display(display, 0);
    measuredMix.computeFromSensors(oxySensor, heSensor);
    measuredMix.display(display,1);
  }
  else if (inMain) {
    mainMenu.update(encoder);
    mainMenu.display(display);
    if(encoder.button_.pressed()) {
      inMain = false;
    }
  } else {
    switch(mainMenu.activeVal_) {
      case 0://start
        mixMenu.display(display);
        mixMenu.getActive().value(encoder.read(mixMenu.getActive().value()));

        if (encoder.button_.pressed()) {
          mixing = mixMenu.next();
        }
        if (mixing) {
          mixSetup.updateFromMixMenu(mixMenu);
        }       
      break;
      case 1://set
        //TODO: make the PI controller parameters changeable
        break;
    }
  }
#endif
}
