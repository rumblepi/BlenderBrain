# BlenderBrain

A controller for continuous flow blending of Nitrox and Trimix gases for diving.
The controller uses two oxygen sensors (one after Helium injection and one after Oxygen injection) and from this computes the composition of the resulting gas.
The desired mix can be adjusted by the user and is then automatically controlled via two magnetic valves. Depending on the currently measured mix, the valves are kept open longer or shorter. For this, a PI controller is implemented.

## Hardware

In the 'Hardware' folder, the KiCad files for the PCB are stored. Due to availability issues, the current configuration of the controller uses an Atmega168 microcontroller, but if available an Atmega328P can and should be used. The board requires two power supplies, one 5V to supply the microcontroller and operational amplifiers and a 24V to supply the magnetic valves as well as the indicator lights.

The oxygen sensors are assumed to be common rebreather sensors, namely ones similar to the NRC D-05R sensor range. These sensors expose three pins, of which we will use two to measure the output voltage. These galvanic cell sensors are dependent current sources outputting a given current for a certain oxygen partial pressure. The sensor also contains a small temperature compensation circuit with a few resistors and an NTC element, such that it effectively outputts a small voltage depending on the current oxygen partial pressure.
For more details see for example [http://www.advanceddivermagazine.com/articles/sensors/sensors.html].

As operational amplifiers, two TLC271 chips are used, but could be combined into a single TLC272 chip in the future. These OpAmps operate in non-inverting configuration with a gain of 101 and an additional feedback capacitor to filter high frequency noise. Note that only two of the three pins of the oxygen sensor are used.

### Programming the microcontroller via ISP

In-System-Programming can be used to update the firmware of the microcontroller. A detailed description of this procedure is given on the Arduino webpage (https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP), we store the most important points here for future reference.
We use an Arduino UNO in ISP mode to write to the Atmega microcontroller chip via the ISP pins on the board. 
Then, in the Arduino IDE the ISP sketch must be uploaded to the UNO via 'File->Examples->ArduinoISP->ArduinoISP'.

The ISP pins have a silkscreen overlay and the connections to the UNO are as follows: MOSI - 11, MISO - 12, SCK - 13, VCC - 5V, GND - GND, RESET - 10
In addition to this, it might be necessary to put a 10uF capacitor between the RESET and GND pins on the UNO.
The Atmega168 is available as processor of an Arduino Duemilanove, so in order to program it, select 'Tools->Board->Arduino Duemilanove or Dieciemila' and switch the processor to 'Atmega168' via 'Tools->Processor->Atmega168'. As we are using the UNO as an In-System-Programmer, also select 'Tools->Programmer->Arduino as ISP'. At first it might be necessary to burn a bootloader onto the chip, so select 'Tools->Burn Bootloader'. After that, the standard 'Upload'-button should do the trick.
