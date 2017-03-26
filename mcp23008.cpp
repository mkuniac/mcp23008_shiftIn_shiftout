/*
********************************************************************
  Name    : mcp23008.cpp
  Author  : Matthew Uniac
  Date    : 23 March, 2017
  Version : 1.0
  Notes   : A library that reads and writes to a mcp23008 port expander
  		  : The library also contains functions for shiftIN and shiftOUT over i2c
  Release : public
********************************************************************
*/

#include "mcp23008.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include "mcp23008.h"


//a function to read from an i2c device and return the vlaue of a specific register as a byte
uint8_t readI2CREG(int deviceAddress, uint8_t regAddress) {
  uint8_t value = 0;
  Wire.beginTransmission(deviceAddress);//starts communication with slave device
  Wire.write(regAddress); //sets gpio register
  Wire.endTransmission();// ends communication
  Wire.requestFrom(deviceAddress, 1);// request one byte of data from expander
  value = Wire.read();
  return value; // return the value and pass it to the function call.
}

//a function to send commands to an I2C device
void writeI2CREG(int deviceAddress, uint8_t regAddress, uint8_t val) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(val);
  Wire.endTransmission();
}


// start instance with i2c address
MCP::MCP(int address){
_address = address;
}


//a function that starts the MCP23008 port expander
void MCP::start(uint8_t gpioDir, uint8_t gpioState){
  writeI2CREG(_address, IODIR, gpioDir); // Set IODIR register to which bits are input and output
  //read GPIO State.  
  currentByte = readI2CREG(_address, GPIO);
  currentByte = currentByte & gpioState; //set all bits of the GPIOstate register to 0 ie off
  writeI2CREG(_address, GPIO, currentByte); //write the GPIOstate register 
}

void MCP::shiftInSetup(uint8_t clockPinIn, uint8_t loadPinIn, uint8_t dataPinIn){
_clockPinIn = clockPinIn;
_loadPinIn = loadPinIn;
_dataPinIn = dataPinIn;
}

void MCP::shiftOutSetup(uint8_t clockPinOut, uint8_t loadPinOut, uint8_t dataPinOut){
_clockPinOut = clockPinOut;
_loadPinOut = loadPinOut;
_dataPinOut = dataPinOut;
}


//a function that communicates with 74hc165 shift register over i2c
uint8_t MCP::shiftInI2C(){ //a function to preform shiftIN over i2c
  uint8_t j;
  
  switch (_dataPinIn){ //check the dataPin value,  set j to correct value for shift below
  	case 0b00000001:
		j=0;
		break;
  	case 0b00000010:
		j=1;
		break;
  	case 0b00000100:
		j=2;
		break;
  	case 0b00001000:
		j=3;
		break;
  	case 0b00010000:
		j=4;
		break;
  	case 0b00100000:
		j=5;
		break;
  	case 0b01000000:
		j=6;
		break;	
  	case 0b10000000:
		j=7;
		break;  
	}
	
  //Set CLOCK HIGH GP1
  writeI2CREG(_address, GPIO, (currentByte = currentByte | _clockPinIn)); //expected xxxx xx1x output
  //Set LOAD HIGH GP0, Keep CLOCK HIGH
  writeI2CREG(_address, GPIO, (currentByte = currentByte | _loadPinIn | _clockPinIn)); //expected xxxx xx11 output

  //Start a loop, preform 8 times, take the bit reading
  uint8_t value = 0;
  for (int i = 7; i >= 0; i--) {

    // Set CLOCK HIGH GP1, Keep LOAD HIGH
    writeI2CREG(_address, GPIO, (currentByte = currentByte | _loadPinIn | _clockPinIn)); // expected xxxx xx11 ouput

    // Read DATA pin
    uint8_t dataRead; // input byte from IO expander used in READ calls
    dataRead = readI2CREG(_address, GPIO) >> j; //shift the bits right to drop off CLOCK and LOAD pins.
    if (dataRead <= 1) { //if input from ioexpander <=1, write each bit to the value variable
      value = value | dataRead << (7 - i); //shift the bits the number I currently is. read 1 place during each loop corresponding to dipswitch
    }

    else {
      dataRead =  dataRead << 7; //shift all bits to left, leaving just the bits assoicated with the GP2
      dataRead =  dataRead >> 7; // shift all bits to right, dropping all the extra bits off the edge
      value = value | dataRead << (7 - i); //shift the bits the number I currently is. read 1 place during each loop corresponding to dipswitch
    }

    // Set CLOCK LOW GP1
    writeI2CREG(_address, GPIO, (currentByte = currentByte ^ _clockPinIn)); // expected xxxx xx0x


  }
  // Set Load LOW - right now this is all set to 0
  writeI2CREG(_address, GPIO, (currentByte = currentByte ^ _loadPinIn)); // expected xxxx xxx0

  return value; // return the value
}


//a function that communicates with 74hc595 shift register over i2c
void MCP::shiftOutI2C(uint8_t val) {
  uint8_t shiftOutValue;
  for (int i = 7; i >= 0; i--) {
    //Shift out binary logic from Val sent with function call
    shiftOutValue = !!(val & (1 << i));

    if (val & (1 << i)) {

      //Set LATCH & CLOCK = LOW, DATA HIGH
      writeI2CREG(_address, GPIO, (currentByte = currentByte & ~(_clockPinOut|_loadPinOut) | _dataPinOut)); //expected xx00 1xxx

      //Set LATCH = LOW,  CLOCK & DATA HIGH
      writeI2CREG(_address, GPIO, (currentByte = currentByte &(~_loadPinOut)|(_clockPinOut|_dataPinOut)));  //expected xx10 1xxx
    }
    else {
      //Set LATCH, CLOCK, DATA = LOW
      writeI2CREG(_address, GPIO, (currentByte = currentByte & ~(_clockPinOut|_loadPinOut|_dataPinOut))); //expected xx00 0xxx

      //Set LATCH & DATA = LOW, CLOCK HIGH
      writeI2CREG(_address, GPIO, (currentByte = currentByte & ~(_dataPinOut|_loadPinOut)|_clockPinOut));  //expected xx1x xxxx
    }
  }
}

//function that starts data transfer to SR.  Must be called before MCP.shiftOutI2C(val)
void MCP::shiftOutI2Cbegin(){
 currentByte = readI2CREG(_address, GPIO); // read the registry
 writeI2CREG(_address, GPIO, (currentByte = currentByte & ~(_dataPinOut|_loadPinOut)|_clockPinOut)); // expected xx00 0xxx
}
    
    
//function that ENDS data transfer to SR.  Must be called AFTER MCP.shiftOutI2C(val)    	
void MCP::shiftOutI2Cend(){
 writeI2CREG(_address, GPIO, (currentByte = currentByte & ~(_clockPinOut|_dataPinOut)|_loadPinOut)); // expected xx01 0xxx
}