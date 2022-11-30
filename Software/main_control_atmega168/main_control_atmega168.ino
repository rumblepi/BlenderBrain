/*
 Main Control
 This file contains the main control logic uploaded to the Atmega168 for gas blending using the BlenderBrain control
 (c) Nov 17, 2022 Michael Sandbichler
*/
#include "LedControl.h"

//#define DEBUGMODE
///////////////////////////////////////////////////////////////////
// Pins etc

// Define all pins and objects according to the schematic 
// given in the 'Hardware' folder
// `* Sensors and Actuators
#define OXY_SENSOR_PIN A5
#define HE_SENSOR_PIN A4
#define HE_VALVE_PIN A0
#define OXY_VALVE_PIN A1

#define BUTTON1_PIN 0
#define BUTTON2_PIN 1
#define RED_LIGHT_PIN A2
#define GREEN_LIGHT_PIN A3

#define DISPLAY_DIN_PIN 9
#define DISPLAY_CS_PIN 8
#define DISPLAY_CLK_PIN 10
//HCMAX7219 display(displayCs);
LedControl lc = LedControl(DISPLAY_DIN_PIN,DISPLAY_CLK_PIN,DISPLAY_CS_PIN,1); 

//* Encoders
#define OXY_ENCODER_CLK 6
#define OXY_ENCODER_DT 7
#define OXY_ENCODER_SW 3
#define HE_ENCODER_CLK 5
#define HE_ENCODER_DT 4
#define HE_ENCODER_SW 2

//* Other fixed numbers
#define VALVE_PWM_PERIOD_MS 1000

//* Global variables
float oxySetPoint = 21.5;
int oldClockOxy = -1; 
bool oxyFixed = false;

float heSetPoint = 0.5;
int oldClockHe = -1;
bool heFixed = false;

float oxySensorCalib = 0;
float oxySensorValue = 21;
float lastOxyValue = 21;

enum MachineState{
  MixSetting,
  SettingComplete,
  MixActive,
};

enum MixMode{
  Nitrox,
  Heliox,
  Trimix
};

MachineState currentState = MachineState::MixSetting;

enum ButtonState{
  Down,
  Up  
};

struct Debouncer {
  Debouncer(float delay):
    delay_(delay),
    lastDebounce_(0){}

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

struct DebouncedButton {
  Debouncer debounce_ = Debouncer(0.05);

  DebouncedButton(int pin, long time = 0) :
    pin_(pin),
    currentState_(ButtonState::Up)    
  {}

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

  int pin_;
  ButtonState currentState_;
};

DebouncedButton heEncoderButton(HE_ENCODER_SW);
DebouncedButton oxyEncoderButton(OXY_ENCODER_SW);
DebouncedButton startButton(BUTTON1_PIN);
DebouncedButton endButton(BUTTON2_PIN);

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

PIControl oxyControl(21,0.01,0.001);

//TODO: clean up the mess
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

  void writeNumber(int number) {
    // Writes the first eight digits of a given number to the display
    for(int i=0; i<8; ++i) {
      lc_.setDigit(0,i,(number/((int) pow(10,i))) % 10,false);
    }
  }
  
  void writePreText(char* txt) {
    //The length of txt must not be larger than four characters and will be displayed
    for(int i = 0; i < 4; ++i) {
      lc_.setChar(0,7-i,txt[i],false);
      if(txt[i]=='t'){
        lc_.setLed(0,7-i,4,true);  
        lc_.setLed(0,7-i,5,true);  
        lc_.setLed(0,7-i,6,true);  
        lc_.setLed(0,7-i,7,true);       
        }
      if(txt[i]=='S'){
        lc_.setDigit(0,7-i,5,false);
      }
    }
  }

  void writeTwoDigits(int number, int pos) {
    //write two digits at position 1,2,3 or 4 of the display |11|22|33|44|
    lc_.setDigit(0,7-2*(pos-1),(number/10) % 10,false);
    lc_.setDigit(0,6-2*(pos-1),number % 10,false);
  }

};

struct ButtonEncoder {
  ButtonEncoder(int clkPin, int dtPin, int buttonPin):
    clk_(clkPin),
    dt_(dtPin),
    oldClock_(-1),
    fixed_(false),
    button_(DebouncedButton(buttonPin)){}
    
  void read(float & setpoint) {
    if (!debounce.check()) return;
    int clockVal = digitalRead(clk_);
    int dataVal = digitalRead(dt_);
    if (clockVal == oldClock_ || fixed_) return;  // was a bounce. Don't count this.
    if (clockVal ^ dataVal) {
      // clockwise move
      setpoint += 0.5;
    } else {
      // counterclockwise move
      setpoint -= 0.5;
    }
    oldClock_ = clockVal;  // store clock state for debounce check.
  }

  int clk_, dt_, oldClock_;
  bool fixed_;
  Debouncer debounce = Debouncer(0.01);
  DebouncedButton button_;
};

ButtonEncoder oxyEncoder(OXY_ENCODER_CLK, OXY_ENCODER_DT, OXY_ENCODER_SW);
ButtonEncoder heEncoder(HE_ENCODER_CLK, HE_ENCODER_DT, HE_ENCODER_SW);

struct Setpoint {
  
  int hePercent;
  int oxyPercent;
};

struct Sensor {
  Sensor(int pin, float calibValue):
    pin_(pin)
    {
      lastValue_ = calibValue;
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
    lastValue_ = measureAverage(20)*calibration_;
  }
    
  void display(EightDigitDisplay display, int pos) {
    display.writeTwoDigits((int) round(lastValue_), pos);
  }

  int pin_;
  float calibration_;
  float lastValue_;
};

#ifdef DEBUGMODE
void debugDisplay() {
    float sensorVal = getAverageSensorVal(OXY_SENSOR_PIN, 20);
    //if (oxySensorValue - lastOxyValue > 1 || oxySensorValue - lastOxyValue < -1) {
    displayNumber(sensorVal); //update display
    //} 
}
#endif
//////////////////////////////////////////////////////////////////
// Basic microcontroller methods

//TODO rebuild with new objects

void setup() {  
  Serial.begin(9600); 
  pinMode(OXY_SENSOR_PIN, INPUT);
  pinMode(HE_SENSOR_PIN, INPUT);

  pinMode(OXY_VALVE_PIN, OUTPUT);
  pinMode(HE_VALVE_PIN, OUTPUT);

  oxySensorCalib = 21.0/getAverageSensorVal(OXY_SENSOR_PIN, 30);

  pinMode(HE_ENCODER_SW, INPUT_PULLUP);
  pinMode(OXY_ENCODER_SW, INPUT_PULLUP);
  
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,2);
  /* and clear the display */
  lc.clearDisplay(0);
  setHeO2Display(oxySetPoint, heSetPoint, oxyFixed, heFixed); //update display
}

void loop() {
#ifdef DEBUGMODE
  debugDisplay();
  delay(1000);
#else
  switch(currentState) {
    case MachineState::MixSetting:
        encoder_read(HE_ENCODER_CLK, HE_ENCODER_DT, heSetPoint, oldClockHe, heFixed);
        encoder_read(OXY_ENCODER_CLK, OXY_ENCODER_DT, oxySetPoint, oldClockOxy, oxyFixed);
        
      if (heEncoderButton.stateChanged() > 0) {
        heFixed = !heFixed;
        setHeO2Display(oxySetPoint, heSetPoint, oxyFixed, heFixed); //update display

      }
      if (oxyEncoderButton.stateChanged() > 0) {
        oxyFixed = !oxyFixed;
        oxyControl.targetValue_ = oxySetPoint;
        setHeO2Display(oxySetPoint, heSetPoint, oxyFixed, heFixed); //update display
      }
      if(heFixed && oxyFixed) {
        currentState = MachineState::SettingComplete;
        mixHeO2Display(oxySetPoint, heSetPoint, round(oxySensorValue), 0);
      }
      break;
    case MachineState::SettingComplete:
      readAndDisplayOxySensor();
      if (heEncoderButton.stateChanged() > 0) {
        heFixed = !heFixed;
        currentState = MachineState::MixSetting;
        setHeO2Display(oxySetPoint, heSetPoint, oxyFixed, heFixed); //update display
      }
      if (oxyEncoderButton.stateChanged() > 0) {
        oxyFixed = !oxyFixed;
        currentState = MachineState::MixSetting;
        setHeO2Display(oxySetPoint, heSetPoint, oxyFixed, heFixed); //update display
      }
      break;      
    case MachineState::MixActive:
      readAndDisplayOxySensor();
      
      break;
    default:
      Serial.println("Error");
  }
#endif
}
