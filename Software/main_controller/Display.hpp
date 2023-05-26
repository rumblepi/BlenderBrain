#pragma once
#include "BasicStructures.hpp"

struct EightDigitDisplay {
  LedControl lc_;
  
  EightDigitDisplay(LedControl lc) :
    lc_(lc){
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
      } else if(txt[i]=='G'){
        lc_.setLed(0,7-i,4,true);  
        lc_.setLed(0,7-i,5,true);  
        lc_.setLed(0,7-i,6,true);  
        lc_.setLed(0,7-i,1,true); 
        lc_.setLed(0,7-i,3,true);   
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
