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

  long TimeOfLastDebounce = 0;
  float DelayOfDebounce = 0.1;


//Encoder heEncoder(heEncoderDt, heEncoderClk);
//Encoder oxyEncoder(oxyEncoderDt, oxyEncoderClk);

//* Global variables
struct {
  float oxySetPoint = 21;
} Globals;

enum {
  MixSetting,
  MixingActive,
} MachineStates;

///////////////////////////////////////////////////////
// Auxiliary functions

void setHeO2Display(int o2Soll, int heSoll, int o2Is, int heIs) {
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,2);
  /* and clear the display */
  lc.clearDisplay(0);
  //todo: error checks
  lc.setDigit(0,7,(o2Soll/10) % 10,false);
  lc.setDigit(0,6,(o2Soll) % 10,false);

  lc.setDigit(0,5,(heSoll/10) % 10,false);
  lc.setDigit(0,4,(heSoll) % 10,false);

  lc.setDigit(0,3,(o2Is/10) % 10,false);
  lc.setDigit(0,2,(o2Is) % 10,false);

  lc.setDigit(0,1,(heIs/10) % 10,false);
  lc.setDigit(0,0,(heIs) % 10,false);
}

void check_rotary() {
          static int oldClock = -1; // Initialize to an impossible value.
          int clockVal = digitalRead(OXY_ENCODER_CLK);
          int dataVal = digitalRead(OXY_ENCODER_DT);
          if(clockVal == oldClock) return; // was a bounce. Don't count this.
          if(clockVal ^ dataVal) {
                  // clockwise move
                  Globals.oxySetPoint+=0.5;
          } else {
               // counterclockwise move
              Globals.oxySetPoint-=0.5;
           }
           if (Globals.oxySetPoint < 0) {
             Globals.oxySetPoint += 100;
           }
            oldClock = clockVal; // store clock state for debounce check.
          setHeO2Display(Globals.oxySetPoint,0,21,0); 
}

//////////////////////////////////////////////////////////////////
// Basic uc methods

void setup() {  
  Serial.begin(9600); 
  
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,2);
  /* and clear the display */
  lc.clearDisplay(0);
  setHeO2Display(21,0,21,0);
}

void loop() {
  if((millis() - TimeOfLastDebounce) > DelayOfDebounce){
    check_rotary();
    TimeOfLastDebounce = millis();
  }
}
