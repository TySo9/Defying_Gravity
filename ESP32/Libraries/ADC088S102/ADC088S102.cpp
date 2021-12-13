#include "ADC088S102.h"


ADC088S102::ADC088S102(uint8_t dataIn, uint8_t dataOut,  uint8_t clock)
{
  _dataIn   = dataIn;
  _dataOut  = dataOut;
  _clock    = clock;
  _select   = 0;
  _channels = 7;
  _maxValue = 4095;
}

void ADC088S102::begin(uint8_t select)
{
  _select = select;
  pinMode(_select, OUTPUT);
  digitalWrite(_select, HIGH);

  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);

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


void ADC088S102::setGPIOpins(uint8_t clk, uint8_t miso, uint8_t mosi, uint8_t select)
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

uint32_t ADC088S102::count()
{
  uint32_t cnt = _count;
  _count = 0;
  return cnt;
}

void ADC088S102::setSPIspeed(uint32_t speed)
{
  _SPIspeed = speed;
  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
};

int16_t ADC088S102::readADC(uint8_t channel)
{
  if (channel >= _channels) return 0;

  _count++;

  // xx[ADD2][ADD1][ADD0]xxx
  uint8_t  send = channel << 3;

  digitalWrite(_select, LOW);

  mySPI->beginTransaction(_spi_settings);
  uint8_t data = mySPI->transfer(send);
  mySPI->endTransaction();

  digitalWrite(_select, HIGH);

  uint8_t V_REF = 5;
  //return ((256 * data[1] + data[2]) & _maxValue);
  return (data * (V_REF/256));
}
