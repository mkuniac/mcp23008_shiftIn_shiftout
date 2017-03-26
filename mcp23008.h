/*
********************************************************************
  Name    : mcp23008.h
  Author  : Matthew Uniac
  Date    : 23 March, 2017
  Version : 1.0
  Notes   : A library that reads and writes to a mcp23008 port expander
  		  : The library also contains functions for shiftIN and shiftOUT over i2c
  Release : 
********************************************************************
*/

#ifndef mcp23008_h
#define mcp23008_h


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"

#define IODIR		0X00
#define IPOL		0x01
#define GPINTEN		0x02
#define DEFVAL		0x03
#define INTCON		0x04
#define IOCON		0x05
#define GPPU		0x06
#define INTF		0x07
#define INTCAP		0x08
#define GPIO		0x09
#define OLAT		0x0A

uint8_t readI2CREG(int deviceAddress, uint8_t regAddress); // function to write to i2c bus returns 8bit int
void writeI2CREG(int deviceAddress, uint8_t regAddress, uint8_t val); // function to read i2c bus

class MCP {
	public:
		
		MCP::MCP(int address);
    	void start(uint8_t gpioDir, uint8_t gpioState); //call to start and setup IO expander initial state
		void shiftInSetup(uint8_t clockPinIn, uint8_t loadPinIn, uint8_t dataPinIn);
		uint8_t shiftInI2C(); //a function to preform shiftIN over i2c		
		void shiftOutSetup(uint8_t clockPinOut, uint8_t loadPinOut, uint8_t dataPinOut);
		void shiftOutI2C(uint8_t val); // a function to preform shiftOUT over i2c
    	void shiftOutI2Cbegin();
    	void shiftOutI2Cend();
    	
	private:	
  		int _address; // stored device i2c address

  		//byte values 
  		uint8_t currentByte; //stored value for current byte read on any reg.

  		
  		//shiftIN byte values
  		uint8_t _clockPinIn;
		uint8_t _loadPinIn;
		uint8_t _dataPinIn;

  		//shiftOUT byte values
  		uint8_t _clockPinOut;
		uint8_t _loadPinOut;
		uint8_t _dataPinOut;
};

#endif