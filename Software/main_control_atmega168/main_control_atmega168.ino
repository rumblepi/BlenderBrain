/*
 Main Control
 This file contains the main control logic uploaded to the Atmega168 for gas blending using the BlenderBrain control
 (c) Nov 17, 2022 Michael Sandbichler
*/
#include <Encoder.h>
#include "LedControl.h"

// Define all pins and objects according to the schematic 
// given in the 'Hardware' folder
// * Sensors and Actuators
int oxySensorPin = A5;
int heSensorPin = A4;
int heValvePin = A0;
int oxyValvePin = A1;
//* UI
int button1Pin = 0;
int button2Pin = 1;
int redLightPin = A2;
int greenLightPin = A3;
//* Display
int displayDin = 9;
int displayCs = 8;
int displayClk = 10;
//HCMAX7219 display(displayCs);
LedControl lc = LedControl(displayDin,displayClk,displayCs,1); 

//* Encoders
int heEncoderClk = 6;
int heEncoderDt = 7;
int heEncoderSw = 3;
int oxyEncoderClk = 5;
int oxyEncoderDt = 4;
int oxyEncoderSw = 2;

Encoder heEncoder(heEncoderDt, heEncoderClk);
Encoder oxyEncoder(oxyEncoderDt, oxyEncoderClk);

void setup() {  
  Serial.begin(9600); 

  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,8);
  /* and clear the display */
  lc.clearDisplay(0);
}

void setHeO2Display(int o2Soll, int heSoll, int o2Is, int heIs) {
  //todo: error checks
  Serial.println((o2Soll/10)%10);
  lc.setDigit(0,7,(o2Soll/10) % 10,false);
  lc.setDigit(0,6,(o2Soll) % 10,false);
  
  lc.setDigit(0,5,(heSoll/10) % 10,false);
  lc.setDigit(0,4,(heSoll) % 10,false);

  lc.setDigit(0,3,(o2Is/10) % 10,false);
  lc.setDigit(0,2,(o2Is) % 10,false);

  lc.setDigit(0,1,(heIs/10) % 10,false);
  lc.setDigit(0,0,(heIs) % 10,false);
}

void loop() {    
  setHeO2Display(21,0,18,0);


}
