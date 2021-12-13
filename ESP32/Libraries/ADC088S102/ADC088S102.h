// @file :      ADC088S102.h
// Description: A library for the Texas Instruments ADC088S102 ADC written for Arduino C on ESP32
// Author:      Tyler Sovar
// Date:        10/28/2021

// 8-bit
// 800 kHz?
// 8 CHANNELS

#include "Arduino.h"
#include "SPI.h"

//#ifndef ADC088S102_H
//#define ADC088S102_H



// ____________________________________________________________________
//| CONTROL REGISTER BITS                                              |
//|   ---------------------------------------------------------------  |
//|  | BIT 7 | BIT 6 | BIT 5 | BIT 4 | BIT 3 | BIT 2 | BIT 1 | BIT 0 | |
//|  |-------------------------------------------------------|-------| |
//|  |   X   |   X   |  ADD2 | ADD 1 | ADD 0 |   X   |   X   |   X   | |
//|   ---------------------------------------------------------------  |
//|____________________________________________________________________|
//| INPUT CHANNEL SELECTION                                            |
//|____________________________________________________________________|
// :::::::::::::::::::
// Define Register Map

// enum reg_map{
//   const int PRE = 0b00;             // Set Bits 7 & 6 to DON'T CARE
//   const int SUF = 0b000;            // Set Bits 2, 1, & 0 to DON'T CARE
  
//   // Define IN0 - IN7. Add PRE + [ # of IN ] + SUF to make it easier to make alterations in the unlikely case of DON'T CARE error
//   const int IN0 = PRE + 0b000 + SUF;
//   const int IN1 = PRE + 0b001 + SUF;
//   const int IN2 = PRE + 0b010 + SUF;
//   const int IN3 = PRE + 0b011 + SUF;
//   const int IN4 = PRE + 0b100 + SUF;
//   const int IN5 = PRE + 0b101 + SUF;
//   const int IN6 = PRE + 0b110 + SUF;
//   const int IN7 = PRE + 0b111 + SUF;
// }

class ADC088S102 {
  public:
    ADC088S102(uint8_t dataIn = 255, uint8_t dataOut = 255, uint8_t clock = 255);
    void     begin(uint8_t select);
    
    int16_t  readADC(uint8_t channel);

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
