// @file :      MCP3008.h
// Description: A SPI library for the Microchip MCP3008 ADC written for Arduino C on ESP32
// Author:      Tyler Sovar
// Date:        11/28/2021

// 10-bit
// 8 CHANNELS

#include "Arduino.h"
#include "SPI.h"

  // Control Register Guide
  //   _______________________________________________________________________
  //  |-----------------------------------------------------------------------|
  //  | BIT 7  | BIT 6  | BIT 5  | BIT 4  | BIT 3  | BIT 2  | BIT 1  | BIT 0  |
  //  |-----------------------------------------------------------------------|
  //  |-----------------------------------------------------------------------|
  //  |    0   |    0   |    0   |    0   |    0   |    0   |    0   |    1   | <--- Message #1: START BIT, 0x01
  //  |-----------------------------------------------------------------------|
  //  |SGL/DIFF|   D2   |   D1   |   D0   |    X   |    X   |    X   |    X   | <--- Message #2: Set Single/Differential, Specify Channel (0-7 in binary)
  //  |_______________________________________________________________________|

class MCP3008 {
  public:
    MCP3008(uint8_t dataIn = 255, uint8_t dataOut = 255, uint8_t clock = 255);
    void     begin(uint8_t select);
    
    uint16_t readADC(uint8_t channel);

    int16_t  maxValue() { return _maxValue; };
    uint8_t  channels() { return _channels; };
  
    //       speed in Hz
    void     setSPIspeed(uint32_t speed);
    uint32_t getSPIspeed()               { return _SPIspeed; };

    // ESP32 specific
    void     selectHSPI() { _useHSPI = true;  };
    void     selectVSPI() { _useHSPI = false; };
    bool     usesHSPI()   { return _useHSPI;  };
    bool     usesVSPI()   { return !_useHSPI; };

    // to overrule ESP32 default hardware pins
    void     setGPIOpins(uint8_t clk, uint8_t miso, uint8_t mosi, uint8_t select);
    uint32_t count();
    
  protected:
    uint8_t  _dataIn;
    uint8_t  _dataOut;
    uint8_t  _clock;
    uint8_t  _select;
    bool     _hwSPI;
    uint8_t  _channels;
    int16_t  _maxValue;
    uint32_t _SPIspeed = 1000000;   // 1MHz is a safe value (datasheet); in a test 4 MHz worked.
    SPIClass    * mySPI;
    SPISettings _spi_settings;
    bool        _useHSPI = true;
    uint32_t _count;
};
