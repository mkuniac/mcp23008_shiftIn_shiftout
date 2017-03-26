#include <Wire.h>

#include "mcp23008.h"

//define GPIO pins
const uint8_t sda = A4; //define i2c SDA pin as A4
const uint8_t scl = A5; //define i2c SCL pin as A5
//variables for SHIFTIN
uint8_t clockPinIn = 0b00000010;//gp1
uint8_t loadPinIn =  0b00000001;//gp0
uint8_t dataPinIn =  0b00000100;//gp2
//variables for ShiftOUT
uint8_t clockPinOut = 0b00100000;//gp5
uint8_t loadPinOut =  0b00010000;//gp4
uint8_t dataPinOut =  0b00001000;//gp3
MCP mcp1(0x20); //setup instance of MCP expander

void setup() {
  // put your setup code here, to run once:
  //setup i2c pins
  pinMode (scl, INPUT_PULLUP);
  pinMode (sda, INPUT_PULLUP);

  //enable serial communication
  Serial.begin(115200);

  //setup i2c communication
  Wire.begin();


  // setup  I/O expander pins to outputs/inputs, write all GPIO to 0
  mcp1.start(B10000100, B00000000);//example use mcp.start(setupIODIR, setupGPIO)
  mcp1.shiftInSetup(clockPinIn, loadPinIn, dataPinIn); // set the clock, load, data pins for shiftIN function
  mcp1.shiftOutSetup(clockPinOut, loadPinOut, dataPinOut);//set the clock, load, data pins for shiftOUT function


  mcp1.shiftOutI2Cbegin();//This starts transmission
  mcp1.shiftOutI2C(0b00000001);
  mcp1.shiftOutI2C(0b00000001);
  mcp1.shiftOutI2C(0b00000001);
  mcp1.shiftOutI2C(0b00000001);
  mcp1.shiftOutI2Cend();//This ends transmission

}

void loop() {
  // put your main code here, to run repeatedly:

}
