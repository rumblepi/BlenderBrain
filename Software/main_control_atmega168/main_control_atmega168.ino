/*
 Main Control
 This file contains the main control logic uploaded to the Atmega168 for gas blending using the BlenderBrain control
 (c) Nov 17, 2022 Michael Sandbichler
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
#define HE_ENCODER_CLK 6
#define HE_ENCODER_DT 7
#define HE_ENCODER_SW 3
#define OXY_ENCODER_CLK 5
#define OXY_ENCODER_DT 4
#define OXY_ENCODER_SW 2

long TimeOfLastDebounce= 0;
float DelayOfDebounce = 0.01;

//* Global variables
float oxySetPoint = 22;
int oldClockOxy = -1; 
bool oxyFixed = false;

float heSetPoint = 1;
int oldClockHe = -1;
bool heFixed = false;

float oxySensorCalib = 0;
float oxySensorValue = 21;
float lastOxyValue = 21;

enum MachineState{
  MixSetting,
  MixingActive,
};

MachineState currentState = MachineState::MixSetting;

///////////////////////////////////////////////////////
// Auxiliary functions

void setHeO2Display(int o2Soll, int heSoll, int o2Is, int heIs) {
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,2);
  /* and clear the display */
  lc.clearDisplay(0);
  //todo: error checks
  lc.setDigit(0,3,(o2Soll/10) % 10,false);
  lc.setDigit(0,2,(o2Soll) % 10, false);

  lc.setDigit(0,1,(heSoll/10) % 10,false);
  lc.setDigit(0,0,(heSoll) % 10, false);

  lc.setChar(0,7,'5',false);
  lc.setChar(0,6,'E',false);
  
  //letter 't'
  lc.setLed(0,5,4,true);  
  lc.setLed(0,5,5,true);  
  lc.setLed(0,5,6,true);  
  lc.setLed(0,5,7,true); 
  
  lc.setChar(0,4,' ',false);
}

void mixHeO2Display(int o2Soll, int heSoll, int o2Is, int heIs) {
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,2);
  /* and clear the display */
  lc.clearDisplay(0);
  //todo: error checks
  lc.setDigit(0,7,(o2Soll/10) % 10,false);
  lc.setDigit(0,6,(o2Soll) % 10, false);

  lc.setDigit(0,5,(heSoll/10) % 10,false);
  lc.setDigit(0,4,(heSoll) % 10, false);

  lc.setDigit(0,3,(o2Is/10) % 10,false);
  lc.setDigit(0,2,(o2Is) % 10,false);

  lc.setDigit(0,1,(heIs/10) % 10,false);
  lc.setDigit(0,0,(heIs) % 10,false);
}

void encoder_read(int clk_pin, int dt_pin, float & setpoint, int &oldClock) {
  int clockVal = digitalRead(clk_pin);
  int dataVal = digitalRead(dt_pin);
  if (clockVal == oldClock) return;  // was a bounce. Don't count this.
  if (clockVal ^ dataVal) {
    // clockwise move
    setpoint += 0.5;
  } else {
    // counterclockwise move
    setpoint -= 0.5;
  }
  if (setpoint < 0) {
    setpoint += 100;
  }
  oldClock = clockVal;  // store clock state for debounce check.
  setHeO2Display(oxySetPoint, heSetPoint, 21, 0); //update display
}

float getAverageSensorVal(int sensor_pin, int num) {
  float output = 0;
  for(int i = 0; i < num; ++i) {
    output += analogRead(sensor_pin);
  }
  return output /(float)num;
}

//////////////////////////////////////////////////////////////////
// Basic microcontroller methods

void setup() {  
  Serial.begin(9600); 
  pinMode(OXY_SENSOR_PIN, INPUT);

  oxySensorCalib = 21.0/getAverageSensorVal(OXY_SENSOR_PIN, 30);

  pinMode(HE_ENCODER_SW, INPUT_PULLUP);
  pinMode(OXY_ENCODER_SW, INPUT_PULLUP);
  
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,2);
  /* and clear the display */
  lc.clearDisplay(0);
  setHeO2Display(oxySetPoint, heSetPoint, round(oxySensorValue), 0); //update display
}

void loop() {
  switch(currentState) {
    case MixSetting:
      if((millis() - TimeOfLastDebounce) > DelayOfDebounce){
        encoder_read(HE_ENCODER_CLK, HE_ENCODER_DT, heSetPoint, oldClockHe);
        encoder_read(OXY_ENCODER_CLK, OXY_ENCODER_DT, oxySetPoint, oldClockOxy);
        TimeOfLastDebounce = millis();
      }
      break;
    case MixingActive:
      oxySensorValue = getAverageSensorVal(OXY_SENSOR_PIN, 20)*oxySensorCalib;
      if (oxySensorValue - lastOxyValue > 1 || oxySensorValue - lastOxyValue < -1) {
        Serial.println(oxySensorValue);
        mixHeO2Display(oxySetPoint, heSetPoint, round(oxySensorValue), 0); //update display
        lastOxyValue = oxySensorValue;
      }
      break;
    default:
      Serial.println("Error");
  }


}
