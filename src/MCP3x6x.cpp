// SPDX-License-Identifier: MIT

/**
 * @file MCP3x6x.cpp
 * @author Stefan Herold (stefan.herold@posteo.de)
 * @brief
 * @version 0.0.1
 * @date 2022-10-30
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <Arduino.h>
#include "MCP3x6x.h"

#include <wiring_private.h>

MCP3x6x::MCP3x6x(const uint16_t MCP3x6x_DEVICE_TYPE, const uint8_t pinCS, SPIClass *theSPI,
                 const uint8_t pinMOSI, const uint8_t pinMISO, const uint8_t pinCLK) {
  switch (MCP3x6x_DEVICE_TYPE) {
    case MCP3461_DEVICE_TYPE:
      _resolution_max = 16;
      _channels_max   = 2;
      break;
    case MCP3462_DEVICE_TYPE:
      _resolution_max = 16;
      _channels_max   = 4;
      break;
    case MCP3464_DEVICE_TYPE:
      _resolution_max = 16;
      _channels_max   = 8;
      break;
    case MCP3561_DEVICE_TYPE:
      _resolution_max = 24;
      _channels_max   = 2;
      break;
    case MCP3562_DEVICE_TYPE:
      _resolution_max = 24;
      _channels_max   = 4;
      break;
    case MCP3564_DEVICE_TYPE:
      _resolution_max = 24;
      _channels_max   = 8;
      break;
    default:
      _resolution_max = 16;
      _channels_max   = 8;
      #warning "bla -------- undefined MCP3x6x_DEVICE_TYPE"
      break;
  }

  //  settings.id = MCP3x6x_DEVICE_TYPE;

  _spi        = theSPI;
  _pinMISO    = pinMISO;
  _pinMOSI    = pinMOSI;
  _pinCLK     = pinCLK;
  _pinCS      = pinCS;

  _resolution = _resolution_max;
  _channel_mask |= 0xff << _channels_max;  // todo use this one
};

MCP3x6x::MCP3x6x(const uint8_t pinIRQ, const uint8_t pinMCLK, const uint16_t MCP3x6x_DEVICE_TYPE,
                 const uint8_t pinCS, SPIClass *theSPI, const uint8_t pinMOSI,
                 const uint8_t pinMISO, const uint8_t pinCLK)
    : MCP3x6x(MCP3x6x_DEVICE_TYPE, pinCS, theSPI, pinMOSI, pinMISO, pinCLK) {
  _pinIRQ  = 34; // pinIRQ; TODO: dont fix here
  _pinMCLK = pinMCLK;

  attachInterrupt(digitalPinToInterrupt(_pinIRQ), mcp_wrapper, FALLING);
}

void MCP3x6x::_reverse_array(uint8_t *array, size_t size) {
  for (size_t i = 0, e = size; i <= e / 2; i++, e--) {
    uint8_t temp = array[i];
    array[i]     = array[e - 1];
    array[e - 1] = temp;
  }
}

MCP3x6x::status_t MCP3x6x::_transfer(uint8_t *data, uint8_t addr, size_t size) {
  _spi->beginTransaction(SPISettings(MCP3x6x_SPI_SPEED, MCP3x6x_SPI_ORDER, MCP3x6x_SPI_MODE));
  noInterrupts();
  digitalWrite(_pinCS, LOW);
  _status.raw = _spi->transfer(addr);
  _spi->transfer(data, size);
  digitalWrite(_pinCS, HIGH);
  interrupts();
  _spi->endTransaction();
  return _status;
}

void MCP3x6x::writeData(const unsigned char *data, unsigned char len)
{
  digitalWrite(_pinCS,LOW); // chip Select pin low to read/write to ADC.
 
  // Write bytes
  while (len > 0)
  {
  (void)SPI.transfer(*data);
  data++;
  len--;
  }
 
  digitalWrite(_pinCS,HIGH); // chip Select pin high to stop read/write of ADC
}

bool MCP3x6x::begin(uint16_t channelmask, float vref) {
  pinMode(_pinCS, OUTPUT);
  digitalWrite(_pinCS, HIGH);
  dr2 = false;
  _spi->begin();

  _status = reset(); // MOSI 120
  
  setClockSelection(clk_sel::INTERN);          // todo make configurable by function parameter MOSI 70 32

  setDataFormat(data_format::ID_SGNEXT_DATA);  // todo make configurable by function parameter MOSI 82 48
  //  delayMicroseconds(1000);
  setIRQ() ;
  // scanmodeok
  if (channelmask != 0) {
    Serial.print("setChannelMask ");
    settings.scan.channel.raw = channelmask;  // todo apply _channel_mask
    _status                   = write(settings.scan);
  }
  
  //delayMicroseconds(1000);
  //Serial.print("setreference ");
  //Serial.print(vref);
  //setReference(vref);
  delayMicroseconds(1000);
  
  Serial.print("standby? ");
  _status = standby();       // MOSI 108
  
  // Write the initial config
  //writeData(&initialConfigData[0], 7); 

  delayMicroseconds(1000);
  return true;
}
/*bla bla*/

MCP3x6x::status_t MCP3x6x::read(Adcdata *data) {
  size_t s = 0;
  //Serial.println("read ADCdata");
  switch (_resolution_max) {
    case 16:
        //Serial.println("max res 16");

      s = settings.config3.data_format == data_format::SGN_DATA ? 2 : 4;
      break;
    case 24:
          //  Serial.println("max res 24");

      s = settings.config3.data_format == data_format::SGN_DATA ? 3 : 4;
      break;
  }
  //Serial.print("s= ");
  //Serial.println(s);

  uint8_t buffer[s];
  //Serial.print("addr: ");
  //Serial.println( MCP3x6x_CMD_SREAD | MCP3x6x_ADR_ADCDATA);
  _status = _transfer(buffer, MCP3x6x_CMD_SREAD | MCP3x6x_ADR_ADCDATA, s);
  _reverse_array(buffer, s);

/*#if MCP3x6x_DEBUG
  Serial.print("buffer: 0x");
  Serial.print(buffer[3], HEX);
  Serial.print(buffer[2], HEX);
  Serial.print(buffer[1], HEX);
  Serial.println(buffer[0], HEX);
//#endif
*/
  data->channelid = _getChannel((uint32_t &)buffer);
  data->value     = _getValue((uint32_t &)buffer);

  return _status;
}

void MCP3x6x::IRQ_handler() {
  
  //Serial.print("irq: ");
  _status.dr = true;
  dr2 = true;
  //Serial.println(_status.dr);
  
  //while (!_status.dr) {
  //_status = read(&adcdata);
  //}

  //result.raw[(uint8_t)adcdata.channelid] = adcdata.value;
/*
#if MCP3x6x_DEBUG
  Serial.print("channel: ");
  Serial.println((uint8_t)adcdata.channelid);
  Serial.print("value: ");
  Serial.println(adcdata.value);
#endif
*/
}

void MCP3x6x::lock(uint8_t key) {
  settings.lock.raw = key;
  _status           = write(settings.lock);
}

void MCP3x6x::unlock() {
  settings.lock.raw = _DEFAULT_LOCK;
  _status           = write(settings.lock);
}

void MCP3x6x::setDataFormat(data_format format) {
  settings.config3.data_format = format;
  _status                      = write(settings.config3);

  switch (format) {
    case data_format::SGN_DATA:
    case data_format::SGN_DATA_ZERO:
      _resolution--;
      break;
    case data_format::SGNEXT_DATA:
    case data_format::ID_SGNEXT_DATA:
      break;
    default:
      _resolution = -1;
      break;
  }
#if MCP3x6x_DEBUG
  Serial.print("resolution");
  Serial.println(_resolution);
#endif
}

void MCP3x6x::setConversionMode(conv_mode mode) {
  settings.config3.conv_mode = mode;
  _status                    = write(settings.config3);
}

void MCP3x6x::setAdcMode(adc_mode mode) {
  settings.config0.adc = mode;
  _status              = write(settings.config0);
}

void MCP3x6x::setClockSelection(clk_sel clk) {
  settings.config0.clk = clk;
  _status              = write(settings.config0);
}

void MCP3x6x::enableScanChannel(mux_t ch) {
  for (size_t i = 0; i < sizeof(_channelID); i++) {
    if (_channelID[i] == ch.raw) {
      bitSet(settings.scan.channel.raw, i);
#ifdef MCP3x6x_DEBUG
      Serial.println(i);
      Serial.println(ch.raw, HEX);
      Serial.println(settings.scan.channel.raw, HEX);
#endif
      break;
    }
  }
  _status = write(settings.scan);
}

void MCP3x6x::disableScanChannel(mux_t ch) {
  for (size_t i = 0; i < sizeof(_channelID); i++) {
    if (_channelID[i] == ch.raw) {
      bitClear(settings.scan.channel.raw, i);
      break;
    }
  }
  _status = write(settings.scan);
}

void MCP3x6x::setReference(float vref) {
  if (vref == 0.0) {
    vref                      = 2.4;
    settings.config0.vref_sel = 1;
    _status                   = write(settings.config0);
  }
  _reference = vref;
}

float MCP3x6x::getReference() { return _reference; }

// returns signed ADC value from raw data
int32_t MCP3x6x::_getValue(uint32_t raw) {
  switch (_resolution_max) {
    case 16:
      switch (settings.config3.data_format) {
        case (data_format::SGN_DATA_ZERO):
          return raw >> 16;
        case (data_format::SGN_DATA):
          bitWrite(raw, 31, bitRead(raw, 16));
          bitClear(raw, 16);
          return raw;
        case (data_format::SGNEXT_DATA):
        case (data_format::ID_SGNEXT_DATA):
          bitWrite(raw, 31, bitRead(raw, 17));
          return raw & 0x8000FFFF;
      };
      break;

    case 24:
      switch (settings.config3.data_format) {
        case (data_format::SGN_DATA_ZERO):
          return raw >> 8;
        case (data_format::SGN_DATA):
          bitWrite(raw, 31, bitRead(raw, 24));
          bitClear(raw, 24);
          return raw;
        case (data_format::SGNEXT_DATA):
        case (data_format::ID_SGNEXT_DATA):
          bitWrite(raw, 31, bitRead(raw, 25));
          return raw & 0x80FFFFFF;
      };
      break;
  }

  return -1;
}

uint8_t MCP3x6x::_getChannel(uint32_t raw) {
  if (settings.config3.data_format == data_format::ID_SGNEXT_DATA) {
    return ((raw >> 28) & 0x0F);
  } else {
    for (size_t i = 0; i < sizeof(_channelID); i++) {
      if (_channelID[i] == settings.mux.raw) {
        return i;
      }
    }
  }
  return -1;
}

int32_t MCP3x6x::analogRead(mux_t ch) {
  // MuxMode
  if (settings.scan.channel.raw == 0) {
//#ifdef MCP3x6x_DEBUG
  //  Serial.println("mux");
//#endif
    settings.mux = ch;
    _status      = write(settings.mux); // MOSI 90 88
    //Serial.println("write(settingsmux) status:");
    //Serial.println(_status.raw, HEX);
    //Serial.println("_status.dr status:");
    //Serial.println(_status.dr, HEX);
    dr2 = false;
    _status      = conversion();
    //Serial.println("conversion() status:");
    //Serial.println(_status.raw, HEX);
    //Serial.println("_status.dr status:");
    //Serial.println(_status.dr, HEX);

    //Serial.println("-----------------;");
    

    //delayMicroseconds(5000);
    //while (!_status.dr) {
    while (!dr2) {
      
    //while (status_dr()) {    //TODO interrupt flag
      //delayMicroseconds(1000000);
      /*
      Serial.println("_status.dr(raw) status:");
      Serial.println(_status.raw, HEX);
      Serial.println("_status.por status:");
      Serial.println(_status.por, HEX);
      Serial.println("_status.crccfg status:");
      Serial.println(_status.crccfg, HEX);
      Serial.println("_status.dr status:");
      Serial.println(_status.dr, HEX);
      Serial.println("_status.addr status:");
      Serial.println(_status.addr, HEX);
      */
     delayMicroseconds(10);
      //_status = read(&adcdata);
      //Serial.println("status;");
      //Serial.println(_status.raw, BIN);
    }
    _status = read(&adcdata);
    dr2 = false;
    return result.raw[(uint8_t)adcdata.channelid] = adcdata.value;
  }

#ifdef MCP3x6x_DEBUG
  Serial.println("scan");
#endif
  // ScanMode
  for (size_t i = 0; i < sizeof(_channelID); i++) {
    if (_channelID[i] == ch.raw) {
      _status = conversion();
      while (!_status.dr) {
        _status = read(&adcdata);
      }
      return adcdata.value;
    }
  }
  return -1;
}

int32_t MCP3x6x::analogReadDifferential(mux pinP, mux pinN) {
  settings.mux = ((uint8_t)pinP << 4) | (uint8_t)pinN;
  write(settings.mux);

  _status                                       = conversion();
  _status                                       = read(&adcdata);
  return result.raw[(uint8_t)adcdata.channelid] = adcdata.value;
}

void MCP3x6x::analogReadResolution(size_t bits) {
  if (bits <= _resolution_max) {
    _resolution = bits;
  }
}

void MCP3x6x::setResolution(size_t bits) { analogReadResolution(bits); }

void MCP3x6x::singleEndedMode() { _differential = false; }

void MCP3x6x::differentialMode() { _differential = true; }

bool MCP3x6x::isDifferential() { return _differential; }

uint32_t MCP3x6x::getMaxValue() { return pow(2, _resolution); }

bool MCP3x6x::isComplete() { return _status.dr; }

void MCP3x6x::startContinuous() {
  setConversionMode(conv_mode::CONTINUOUS);
  conversion();
}

void MCP3x6x::stopContinuous() {
  setConversionMode(conv_mode::ONESHOT_STANDBY);
  standby();
}

void MCP3x6x::startContinuousDifferential() {
  differentialMode();
  startContinuous();
}

bool MCP3x6x::isContinuous() {
  if (settings.config3.conv_mode == conv_mode::CONTINUOUS) {
    return true;
  }
  return false;
}

void MCP3x6x::setAveraging(osr rate) {
  settings.config1.osr = rate;
  _status              = write(settings.config1);
}

void MCP3x6x::setIRQ() {
  settings.irq.irq_mode = 0b01;
  settings.irq.en_fastcmd = 0b1;
  _status              = write(settings.irq);
}

int32_t MCP3x6x::analogReadContinuous(mux_t ch) {
  if (isContinuous()) {
    for (size_t i = 0; i < sizeof(_channelID); i++) {
      if (_channelID[i] == ch.raw) {
        return result.raw[(uint8_t)adcdata.channelid];
      }
    }
  }
  return -1;
}