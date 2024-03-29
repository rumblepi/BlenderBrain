/*
 Main Control
 This file contains the main control logic uploaded to the Atmega168 for gas blending using the BlenderBrain control
 (c) Mar 28, 2023 Michael Sandbichler
*/
///////////////////////////////////////////////////////////////////
// Pins etc
#include "BasicStructures.hpp"
#include "UI.hpp"
#include "Encoder.hpp"
#include "Sensor.hpp"
#include "Display.hpp"
#include "Setpoint.hpp"

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

struct MixSetup {
  bool
  updateFromMixMenu(MixMenu settings) {
    pressureBefore = settings.pPrev.value();
    pressureAfter = settings.pPost.value();
    mixBefore = Setpoint(settings.o2Prev.value(), settings.hePrev.value());
    mixAfter = Setpoint(settings.o2Post.value(), settings.hePost.value());
    integralControlMix.oxyPercent_.maximum(40.0);
    return computeRequiredMix();
  }

  bool 
  computeRequiredMix() {
    float prevFraction = static_cast<float>(pressureBefore) / pressureAfter;
    float requiredHe = (mixAfter.hePercent_.get() - prevFraction*mixBefore.hePercent_.get())/(1-prevFraction);
    float requiredO2 = (mixAfter.oxyPercent_.get() - prevFraction*mixBefore.oxyPercent_.get())/(1-prevFraction);
    if (requiredO2 > 40 || requiredO2 < 0 || requiredHe < 0 || requiredHe > 100) {
      return false;
    }
    mixRequired = Setpoint(requiredO2, requiredHe);
    mixEstimated = Setpoint(20.95, 0);
    updateIntegralControl();
    prevTime_ = millis();
    return true;
  }

  bool
  updateEstimatedMix(Setpoint measuredMix) {
    if(!integralDebouncer.check()) return false;
    float now = millis();
    float dt = now - prevTime_;
    mixEstimated.setOxyPercent((mixEstimated.oxyPercent_.get()*elapsedTime_ + measuredMix.oxyPercent_.get()*dt)/(elapsedTime_ + dt));
    mixEstimated.setHePercent((mixEstimated.hePercent_.get()*elapsedTime_ + measuredMix.hePercent_.get()*dt)/(elapsedTime_ + dt));
    elapsedTime_ = elapsedTime_ + dt;
    prevTime_ = now;
    return true;
  }

  void
  resetEstimatedMix() {
    mixEstimated = Setpoint(20.95, 0);
    elapsedTime_ = 0;
  }

  bool 
  updateIntegralControl() {
    integralControlMix.setOxyPercent(2*mixRequired.oxyPercent_.get() - mixEstimated.oxyPercent_.get());
    integralControlMix.setHePercent(2*mixRequired.hePercent_.get() - mixEstimated.hePercent_.get());
    return true;
  }

  int pressureBefore;
  int pressureAfter;
  Setpoint mixBefore;
  Setpoint mixAfter; 
  Setpoint mixRequired;
  Setpoint mixEstimated;
  Debouncer integralDebouncer = Debouncer(500);
  float prevTime_ = 0;
  float elapsedTime_ = 0;

  Setpoint integralControlMix;

};

///////////////////////////////////////////////////////////////////

ButtonEncoder encoder = ButtonEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW);
DebouncedButton reset = DebouncedButton(RESET_BUTTON_PIN);

EightDigitDisplay display = EightDigitDisplay(LedControl(DISPLAY_DIN_PIN,DISPLAY_CLK_PIN,DISPLAY_CS_PIN,1));

Sensor oxySensor = Sensor(OXY_SENSOR_PIN, 20.95);
Sensor heSensor = Sensor(HE_SENSOR_PIN, 20.95);

Setpoint measuredMix;

MixMenu mixMenu;
bool mixing = false;
bool goodMix = false;

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

  delay(1000);
  oxySensor.recalibrate(20.95);
  heSensor.recalibrate(20.95);
  measuredMix.displayDebounce_ = Debouncer(50);
}

void loop() {
#ifdef DEBUGMODE
  debugDisplay();
  delay(1000);
#else
  if (reset.pressed()) {
    mixing = false;
    mixMenu.reset();
    display.clear();
    for (int i = 0; i < 5; ++i) {
      display.writeText("SEnS0r  ", 8);
      delay(500);
      display.writeText("CAL1b   ", 8);
      delay(500);
    }
    oxySensor.recalibrate(20.95);
    heSensor.recalibrate(20.95);
    mixSetup.resetEstimatedMix();
  }
  if(mixing) {
    //TODO add display and PI control of valves
    if(!goodMix) {
      display.writeText("Set Err ", 8);
      mixing = false;
      mixMenu.reset();
    }
    mixSetup.integralControlMix.display(display, 0);
    oxySensor.update();
    heSensor.update();
    measuredMix.computeFromSensors(oxySensor, heSensor);
    mixSetup.updateEstimatedMix(measuredMix);
    mixSetup.updateIntegralControl();
    measuredMix.display(display,1);
  } else {
    mixMenu.display(display);
    mixMenu.getActive().value(encoder.read(mixMenu.getActive().value()));

    if (encoder.button_.pressed()) {
      mixing = mixMenu.next();
    }
    if (mixing) {
      goodMix = mixSetup.updateFromMixMenu(mixMenu);
    }       
  }
#endif
}
