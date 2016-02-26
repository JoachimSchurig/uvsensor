/***************************************************
 This is a library for the Si1145 UV/IR/Visible Light Sensor
 
 Designed specifically to work with the Si1145 sensor in the
 adafruit shop
 ----> https://www.adafruit.com/products/1777
 
 These sensors use I2C to communicate, 2 pins are required to
 interface
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
 
 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, all text above must be included in any redistribution
 ****************************************************/

#if defined(ARDUINO)
 #if (ARDUINO >= 100)
  #include "Arduino.h"
 #else
  #include "WProgram.h"
 #endif
 #include <Wire.h>
#else
 #include <inttypes.h>
 typedef bool boolean;
 #define USE_GPIOLIB
 #include <cppgpio.hpp>
#endif


class Adafruit_SI1145  {
public:
    Adafruit_SI1145();
    Adafruit_SI1145(boolean proximity, boolean interrupt, uint32_t microseconds);

    boolean begin(boolean proximity = true, boolean interrupt = true, uint32_t microseconds = 8000);
    boolean initialized() const { return initialized_v; }
    void reset();
    
    uint16_t readUV() const;
    uint16_t readIR() const;
    uint16_t readVisible() const;
    uint32_t readIRLux() const;
    uint32_t readVisibleLux() const;
    uint16_t readProx() const;
    
    uint16_t readVisibleRange() const;
    uint16_t readIRRange() const;
    uint16_t readVisibleSensitivity() const;
    uint16_t readIRSensitivity() const;
    
#ifndef ARDUINO
    void print_config() const;
#endif
    
private:
    uint16_t read16(uint8_t reg) const;
    uint8_t read8(uint8_t reg) const;
    void write8(uint8_t reg, uint8_t val) const;
    uint8_t readParam(uint8_t p) const;
    uint8_t writeParam(uint8_t p, uint8_t v) const;
    
    uint8_t _addr;
#ifdef USE_GPIOLIB
    GPIO::I2C m_i2c;
#endif
    boolean initialized_v;
};

