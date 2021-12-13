// @file :      MCP3008.cpp
// Description: A SPI library for the Microchip MCP3008 ADC written for Arduino C on ESP32
// Author:      Tyler Sovar
// Date:        11/28/2021

#include "MCP3008.h"

MCP3008::MCP3008(uint8_t dataIn, uint8_t dataOut,  uint8_t clock)
{
  _dataIn   = dataIn;
  _dataOut  = dataOut;
  _clock    = clock;
  _select   = 0;
  _channels = 8;
  _maxValue = 4095;
}

void MCP3008::begin(uint8_t select)
{
  _select = select;             // Set Chip Select Pin
  pinMode(_select, OUTPUT);     // Configure Chip Select Pin
  digitalWrite(_select, HIGH);  // Keep SPI Device in idling state (ACTIVE LOW) 

  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);  // Set SPI Settings

  // Determine SPI Interface (which pins to use)
  if (_useHSPI)      // HSPI
  {
    mySPI = new SPIClass(HSPI);
    mySPI->end();
    mySPI->begin(14, 12, 13, select);   // CLK=14 MISO=12 MOSI=13
  }
  else               // VSPI
  {
    mySPI = new SPIClass(VSPI);
    mySPI->end();
    mySPI->begin(18, 19, 23, select);   // CLK=18 MISO=19 MOSI=23
  }
}

// Set custom pins regardless of hardware interface (Software SPI)
void MCP3008::setGPIOpins(uint8_t clk, uint8_t miso, uint8_t mosi, uint8_t select)
{
  _clock   = clk;
  _dataIn  = miso;
  _dataOut = mosi;
  _select  = select;
  pinMode(_select, OUTPUT);
  digitalWrite(_select, HIGH);

  mySPI->end();  // disable SPI
  mySPI->begin(clk, miso, mosi, select);
}

// Return how many times ADC has been read
uint32_t MCP3008::count()
{
  uint32_t cnt = _count;
  _count = 0;
  return cnt;
}

// Configure SPI Speed
void MCP3008::setSPIspeed(uint32_t speed)
{
  _SPIspeed = speed;
  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
};

// Read ADC at the specified Input Channel
uint16_t MCP3008::readADC(uint8_t channel)
{
  // Make sure specified channel is valid
  if (channel >= _channels) return 0;
  if (channel < 0)          return 0;

  _count++;   // Add to counter

  // Build a 'message' to send to the ADC
  // Activate Single-Ended Mode and determine Input Channel to be read
  // BINARY MESSAGE
  //   _______________________________________________________________________
  //  |  BITS  |                                                              |
  //  |-----------------------------------------------------------------------|
  //  | BIT 7  | BIT 6  | BIT 5  | BIT 4  | BIT 3  | BIT 2  | BIT 1  | BIT 0  |
  //  |-----------------------------------------------------------------------|
  //  |-----------------------------------------------------------------------|
  //  |    0   |    0   |    0   |    0   |    0   |    0   |    0   |    1   | <--- Message #1: START BIT, 0x01
  //  |-----------------------------------------------------------------------|
  //  |SGL/DIFF|   D2   |   D1   |   D0   |    X   |    X   |    X   |    X   | <--- Message #2: Set Single/Differential, Specify Channel (0-7 in binary)
  //  |_______________________________________________________________________|
  
  bool singDiff = true;                     // Set to false if Differential Mode
  uint8_t send[3];
  send[0] = 0x01;                           // Set Starting Bit
  send[1] = singDiff << 7 | channel << 4;   // Set Single/Differential Mode, Specify Input Channel

  digitalWrite(_select, LOW);               // Activate SPI Chip (Active LOW)
  mySPI->beginTransaction(_spi_settings);   // Begin a SPI Transaction using pre-determined settings
  mySPI->transfer(send, 3);                 // Send 'message', send[] buffer is replaced with ADC reply
  mySPI->endTransaction();                  // End SPI transaction
  digitalWrite(_select, HIGH);              // De-activate SPI Chip (Inactive HIGH)

  uint16_t V_REF = 5000;    // ADC Reference Voltage
  uint16_t out_code  = (((uint16_t)(send[1] & 0x07)) << 8) | send[2];  // Determine the output code from ADC response
  uint16_t milli_volts  = ( out_code * V_REF ) / 1024;
  
  // Return ADC Input Voltage in mV
  return milli_volts;
}
