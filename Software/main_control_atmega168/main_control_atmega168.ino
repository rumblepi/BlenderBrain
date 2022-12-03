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
float oxySensorValue = 20.95;
float lastOxyValue = 20.95;

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

enum GasType{
  Helium,
  Oxygen
};

MachineState currentState = MachineState::MixSetting;

enum ButtonState{
  Down,
  Up  
};

float clip(float number, float min, float max) {
  if (number < min) return min;
  if (number > max) return max;
  return number;  
}

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

EightDigitDisplay display(LedControl(DISPLAY_DIN_PIN,DISPLAY_CLK_PIN,DISPLAY_CS_PIN,1));

struct Setpoint {
  Debouncer displayDebounce_ = Debouncer(0.1);

  Setpoint(float oxy = 20.95, float he = 0.0) :
    hePercent_(he),
    oxyPercent_(oxy),
    minOxy_(0),
    maxOxy_(100.0){}  

  void setHePercent(float newHe) {
    hePercent_ = clip(newHe,0,99);
    minOxy_ = 20.95*(1-hePercent_/100.);
  }
  
  void setOxyPercent(float newOxy) {
    oxyPercent_ = clip(newOxy, minOxy_, maxOxy_);
  }
  
  void display(EightDigitDisplay & display, int pos) {
    if(!displayDebounce_.check())return;
    //draw at position 0 or 1 [0000|1111]
    switch(pos){
      case 0:
        display.writeTwoDigits(round(oxyPercent_), 1);
        display.writeTwoDigits(round(hePercent_), 2);
        break;
      case 1:
        display.writeTwoDigits(round(oxyPercent_), 3);
        display.writeTwoDigits(round(hePercent_), 4);
        break;
      default:
        return;     
    }
  }  
  float hePercent_;
  float oxyPercent_;
  float minOxy_, maxOxy_;
};

Setpoint currentPercent;
Setpoint wantedPercent;

struct ButtonEncoder {
  ButtonEncoder(int clkPin, int dtPin, int buttonPin):
    clk_(clkPin),
    dt_(dtPin),
    state_(0),
    fixed_(false),
    button_(DebouncedButton(buttonPin)){}

  void read(float & setpoint) {
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
                ++setpoint;
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
                --setpoint;
            }
            break; 
    }
  }


  void updateSetpoint(Setpoint & sp, GasType gas) {
    float currSp;
    switch(gas) {
      case GasType::Helium:
        currSp = sp.hePercent_;
        read(currSp);
        sp.setHePercent(currSp);
        break;
      case GasType::Oxygen:
        currSp = sp.oxyPercent_;
        read(currSp);
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

ButtonEncoder oxyEncoder(OXY_ENCODER_CLK, OXY_ENCODER_DT, OXY_ENCODER_SW);
ButtonEncoder heEncoder(HE_ENCODER_CLK, HE_ENCODER_DT, HE_ENCODER_SW);

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
    lastValue_ = clip(measureAverage(20)*calibration_, 0,99);
  }
    
  void display(EightDigitDisplay display, int pos) {
    display.writeTwoDigits((int) round(lastValue_), pos);
  }

  int pin_;
  float calibration_;
  float lastValue_;
};

Sensor oxySensor(OXY_SENSOR_PIN, 21.0);
Sensor heSensor(HE_SENSOR_PIN, 0.0);

#ifdef DEBUGMODE
void debugDisplay() {
    float sensorVal = oxySensor.measureAverage(20);
    //if (oxySensorValue - lastOxyValue > 1 || oxySensorValue - lastOxyValue < -1) {
    display.writeNumber(sensorVal); //update display
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

  pinMode(HE_ENCODER_SW, INPUT_PULLUP);
  pinMode(OXY_ENCODER_SW, INPUT_PULLUP);


  wantedPercent.minOxy_ = 20.95;
  wantedPercent.maxOxy_ = 40.0;
  delay(1000);
  oxySensor.recalibrate(20.95);
  currentPercent.displayDebounce_.delay_ = 100;
}

void loop() {
#ifdef DEBUGMODE
  debugDisplay();
  delay(1000);
#else
  switch(currentState) {
    case MachineState::MixSetting:
      heEncoder.updateSetpoint(wantedPercent, GasType::Helium);
      oxyEncoder.updateSetpoint(wantedPercent, GasType::Oxygen);
      oxySensor.update();
      currentPercent.oxyPercent_ = oxySensor.lastValue_;
      break;      
    case MachineState::MixActive:
      oxySensor.update();
      currentPercent.oxyPercent_ = oxySensor.lastValue_;
      break;
    default:
      break;    
  }

  wantedPercent.display(display, 0);
  currentPercent.display(display,1);
#endif
}
