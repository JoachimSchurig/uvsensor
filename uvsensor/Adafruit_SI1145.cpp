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

/*
 * added functionality by Joachim Schurig is BSD license, too
 *
 */

#include "Adafruit_SI1145.h"

#ifdef USE_GPIOLIB
 #include <unistd.h>
 #include <cppgpio.hpp>
#endif

/* COMMANDS */
#define SI1145_PARAM_QUERY 0x80
#define SI1145_PARAM_SET 0xA0
#define SI1145_NOP 0x0
#define SI1145_RESET    0x01
#define SI1145_BUSADDR    0x02
#define SI1145_PS_FORCE    0x05
#define SI1145_ALS_FORCE    0x06
#define SI1145_PSALS_FORCE    0x07
#define SI1145_PS_PAUSE    0x09
#define SI1145_ALS_PAUSE    0x0A
#define SI1145_PSALS_PAUSE    0xB
#define SI1145_PS_AUTO    0x0D
#define SI1145_ALS_AUTO   0x0E
#define SI1145_PSALS_AUTO 0x0F
#define SI1145_GET_CAL    0x12

/* Parameters */
#define SI1145_PARAM_I2CADDR 0x00
#define SI1145_PARAM_CHLIST   0x01
#define SI1145_PARAM_CHLIST_ENUV 0x80
#define SI1145_PARAM_CHLIST_ENAUX 0x40
#define SI1145_PARAM_CHLIST_ENALSIR 0x20
#define SI1145_PARAM_CHLIST_ENALSVIS 0x10
#define SI1145_PARAM_CHLIST_ENPS1 0x01
#define SI1145_PARAM_CHLIST_ENPS2 0x02
#define SI1145_PARAM_CHLIST_ENPS3 0x04

#define SI1145_PARAM_PSLED12SEL   0x02
#define SI1145_PARAM_PSLED12SEL_PS2NONE 0x00
#define SI1145_PARAM_PSLED12SEL_PS2LED1 0x10
#define SI1145_PARAM_PSLED12SEL_PS2LED2 0x20
#define SI1145_PARAM_PSLED12SEL_PS2LED3 0x40
#define SI1145_PARAM_PSLED12SEL_PS1NONE 0x00
#define SI1145_PARAM_PSLED12SEL_PS1LED1 0x01
#define SI1145_PARAM_PSLED12SEL_PS1LED2 0x02
#define SI1145_PARAM_PSLED12SEL_PS1LED3 0x04

#define SI1145_PARAM_PSLED3SEL   0x03
#define SI1145_PARAM_PSENCODE   0x05
#define SI1145_PARAM_ALSENCODE  0x06

#define SI1145_PARAM_PS1ADCMUX   0x07
#define SI1145_PARAM_PS2ADCMUX   0x08
#define SI1145_PARAM_PS3ADCMUX   0x09
#define SI1145_PARAM_PSADCOUNTER   0x0A
#define SI1145_PARAM_PSADCGAIN 0x0B
#define SI1145_PARAM_PSADCMISC 0x0C
#define SI1145_PARAM_PSADCMISC_RANGE 0x20
#define SI1145_PARAM_PSADCMISC_PSMODE 0x04

#define SI1145_PARAM_ALSIRADCMUX   0x0E
#define SI1145_PARAM_AUXADCMUX   0x0F

#define SI1145_PARAM_ALSVISADCOUNTER   0x10
#define SI1145_PARAM_ALSVISADCGAIN 0x11
#define SI1145_PARAM_ALSVISADCMISC 0x12
#define SI1145_PARAM_ALSVISADCMISC_VISRANGE 0x20

#define SI1145_PARAM_ALSIRADCOUNTER   0x1D
#define SI1145_PARAM_ALSIRADCGAIN 0x1E
#define SI1145_PARAM_ALSIRADCMISC 0x1F
#define SI1145_PARAM_ALSIRADCMISC_RANGE 0x20

#define SI1145_PARAM_ADCCOUNTER_511CLK 0x70

#define SI1145_PARAM_ADCMUX_SMALLIR  0x00
#define SI1145_PARAM_ADCMUX_LARGEIR  0x03



/* REGISTERS */
#define SI1145_REG_PARTID  0x00
#define SI1145_REG_REVID  0x01
#define SI1145_REG_SEQID  0x02

#define SI1145_REG_INTCFG  0x03
#define SI1145_REG_INTCFG_INTOE 0x01
#define SI1145_REG_INTCFG_INTMODE 0x02

#define SI1145_REG_IRQEN  0x04
#define SI1145_REG_IRQEN_ALSEVERYSAMPLE 0x01
#define SI1145_REG_IRQEN_PS1EVERYSAMPLE 0x04
#define SI1145_REG_IRQEN_PS2EVERYSAMPLE 0x08
#define SI1145_REG_IRQEN_PS3EVERYSAMPLE 0x10


#define SI1145_REG_IRQMODE1 0x05
#define SI1145_REG_IRQMODE2 0x06

#define SI1145_REG_HWKEY  0x07
#define SI1145_REG_MEASRATE0 0x08
#define SI1145_REG_MEASRATE1  0x09
#define SI1145_REG_PSRATE  0x0A
#define SI1145_REG_PSLED21  0x0F
#define SI1145_REG_PSLED3  0x10
#define SI1145_REG_UCOEFF0  0x13
#define SI1145_REG_UCOEFF1  0x14
#define SI1145_REG_UCOEFF2  0x15
#define SI1145_REG_UCOEFF3  0x16
#define SI1145_REG_PARAMWR  0x17
#define SI1145_REG_COMMAND  0x18
#define SI1145_REG_RESPONSE  0x20
#define SI1145_REG_IRQSTAT  0x21
#define SI1145_REG_IRQSTAT_ALS  0x01

#define SI1145_REG_ALSVISDATA0 0x22
#define SI1145_REG_ALSVISDATA1 0x23
#define SI1145_REG_ALSIRDATA0 0x24
#define SI1145_REG_ALSIRDATA1 0x25
#define SI1145_REG_PS1DATA0 0x26
#define SI1145_REG_PS1DATA1 0x27
#define SI1145_REG_PS2DATA0 0x28
#define SI1145_REG_PS2DATA1 0x29
#define SI1145_REG_PS3DATA0 0x2A
#define SI1145_REG_PS3DATA1 0x2B
#define SI1145_REG_UVINDEX0 0x2C
#define SI1145_REG_UVINDEX1 0x2D
#define SI1145_REG_PARAMRD 0x2E
#define SI1145_REG_CHIPSTAT 0x30

#define SI1145_ADDR 0x60


#ifndef ARDUINO
#include <chrono>
inline void delay (uint16_t millisecs)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(millisecs));
}

#include <stdio.h>
void Adafruit_SI1145::print_config() const {
    fprintf(stderr, "registers:\n");
    for (uint16_t reg = 0x00; reg <= 0x30; ++reg) {
        uint16_t val = read8(reg);
        fprintf(stderr, "%2X %2X %3d\n", reg, val, val);
    }
}
#endif


Adafruit_SI1145::Adafruit_SI1145()
#ifdef USE_GPIOLIB
: m_i2c("/dev/i2c-1", SI1145_ADDR)
#endif
{
    _addr = SI1145_ADDR;
    initialized_v = false;
}

Adafruit_SI1145::Adafruit_SI1145(boolean proximity, boolean interrupt, uint32_t microseconds_rate)
#ifdef USE_GPIOLIB
: m_i2c("/dev/i2c-1", SI1145_ADDR)
#endif
{
    _addr = SI1145_ADDR;
    initialized_v = false;
    begin(proximity, interrupt, microseconds_rate);
}


boolean Adafruit_SI1145::begin(boolean proximity, boolean interrupt, uint32_t microseconds) {
#if defined(ARDUINO)
    Wire.begin();
#endif
    
    uint8_t id = read8(SI1145_REG_PARTID);
    if (id != 0x45) return false; // look for SI1145
    
    reset();
    
    
    /***********************************/
    // enable UVindex measurement coefficients!
    
    write8(SI1145_REG_UCOEFF0, 0x29);
    write8(SI1145_REG_UCOEFF1, 0x89);
    write8(SI1145_REG_UCOEFF2, 0x02);
    write8(SI1145_REG_UCOEFF3, 0x00);
    
    // enable UV sensor and others
    writeParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
               SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
               ((proximity) ? SI1145_PARAM_CHLIST_ENPS1 : 0));
    
    if (interrupt) {
        
        // enable interrupt on every sample
        write8(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);
        write8(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);
        
    }
    
    /****************************** Prox Sense 1 */
    
    if (proximity) {
        
        // program LED current
        write8(SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
        writeParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
        // prox sensor #1 uses LED #1
        writeParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
        // fastest clocks, clock div 1
        writeParam(SI1145_PARAM_PSADCGAIN, 0);
        // take 511 clocks to measure
        writeParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
        // in prox mode, high range
        writeParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE|
                   SI1145_PARAM_PSADCMISC_PSMODE);
        
    }
    
    writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);
    // fastest clocks, clock div 1
    writeParam(SI1145_PARAM_ALSIRADCGAIN, 0);
    // take 511 clocks to measure
    writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    // in high range mode
    writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);
    
    
    // fastest clocks, clock div 1
    writeParam(SI1145_PARAM_ALSVISADCGAIN, 0);
    // take 511 clocks to measure
    writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    // in high range mode (not normal signal)
    writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);
    
    /************************/
    
    // measurement rate for auto
    
    if (microseconds < 32) microseconds = 32;
    
    uint32_t rate = microseconds / 31.25;
    uint8_t rate0 = rate % 255;
    uint8_t rate1 = rate / 255;
    
    write8(SI1145_REG_MEASRATE0, rate0);
    write8(SI1145_REG_MEASRATE1, rate1);
    
    // auto run
    write8(SI1145_REG_COMMAND, ((proximity) ? SI1145_PSALS_AUTO : SI1145_ALS_AUTO));
    
    initialized_v = true;
    
    return true;
}

void Adafruit_SI1145::reset() {
    write8(SI1145_REG_MEASRATE0, 0);
    write8(SI1145_REG_MEASRATE1, 0);
    write8(SI1145_REG_PSLED21, 0);
    write8(SI1145_REG_PSLED3, 0);
    write8(SI1145_REG_IRQEN, 0);
    write8(SI1145_REG_IRQMODE1, 0);
    write8(SI1145_REG_IRQMODE2, 0);
    write8(SI1145_REG_INTCFG, 0);
    write8(SI1145_REG_IRQSTAT, 0xFF);
    
    write8(SI1145_REG_COMMAND, SI1145_RESET);
    delay(10);
    write8(SI1145_REG_HWKEY, 0x17);
    
    initialized_v = false;
    
    delay(10);
}


//////////////////////////////////////////////////////

// returns the UV index * 100 (divide by 100 to get the index)
uint16_t Adafruit_SI1145::readUV() const {
    return read16(0x2C);
}

// returns visible+IR light levels
uint16_t Adafruit_SI1145::readVisible() const {
    return read16(0x22);
}

// returns IR light levels
uint16_t Adafruit_SI1145::readIR() const {
    return read16(0x24);
}

uint16_t Adafruit_SI1145::readVisibleRange() const {
    return readParam(SI1145_PARAM_ALSVISADCMISC);
}

uint16_t Adafruit_SI1145::readVisibleSensitivity() const {
    return readParam(SI1145_PARAM_ALSVISADCGAIN);
}

uint16_t Adafruit_SI1145::readIRRange() const {
    return readParam(SI1145_PARAM_ALSIRADCMISC);
}

uint16_t Adafruit_SI1145::readIRSensitivity() const {
    return readParam(SI1145_PARAM_ALSIRADCGAIN);
}

// returns visible+IR light levels in Lux
uint32_t Adafruit_SI1145::readVisibleLux() const {
    uint16_t range = readVisibleRange();
    float gain = ((range & 32) == 32) ? 14.5 : 1;
    uint16_t sensitivity = readVisibleSensitivity();
    uint16_t multiplier = 1 << sensitivity;
    return readVisible() * (gain / (0.282 * multiplier));
}

// returns IR light levels in Lux
// (for discussion see http://forums.adafruit.com/viewtopic.php?f=19&p=302193 )
uint32_t Adafruit_SI1145::readIRLux() const {
    uint16_t range = readIRRange();
    float gain = ((range & 32) == 32) ? 14.5 : 1;
    uint16_t sensitivity = readIRSensitivity();
    uint16_t multiplier = 1 << sensitivity;
    return readIR() * (gain / (2.44 * multiplier));
}

// returns "Proximity" - assumes an IR LED is attached to LED
uint16_t Adafruit_SI1145::readProx() const {
    return read16(0x26);
}

/*********************************************************************/

uint8_t Adafruit_SI1145::writeParam(uint8_t p, uint8_t v) const {
    write8(SI1145_REG_PARAMWR, v);
    write8(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
    return read8(SI1145_REG_PARAMRD);
}

uint8_t Adafruit_SI1145::readParam(uint8_t p) const {
    write8(SI1145_REG_COMMAND, p | SI1145_PARAM_QUERY);
    return read8(SI1145_REG_PARAMRD);
}

/*********************************************************************/

uint8_t  Adafruit_SI1145::read8(uint8_t reg) const {
#if defined(ARDUINO)
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
    return Wire.read();
#elif defined(USE_GPIOLIB)
    return m_i2c.regread8(reg);
#endif
}

uint16_t Adafruit_SI1145::read16(uint8_t reg) const {
#if defined(ARDUINO)
    uint16_t ret;
    
    Wire.beginTransmission(_addr); // start transmission to device
    Wire.write(reg); // sends register address to read from
    Wire.endTransmission(); // end transmission
    
    Wire.requestFrom(_addr, (uint8_t)2);// send data n-bytes read
    ret = Wire.read(); // receive DATA
    ret |= (uint16_t)Wire.read() << 8; // receive DATA
    
    return ret;
#elif defined(USE_GPIOLIB)
    return m_i2c.regread16(reg);
#endif
}

void Adafruit_SI1145::write8(uint8_t reg, uint8_t val) const {
#if defined(ARDUINO)
    Wire.beginTransmission(_addr); // start transmission to device
    Wire.write(reg); // sends register address to write
    Wire.write(val); // sends value
    Wire.endTransmission(); // end transmission
#elif defined(USE_GPIOLIB)
    m_i2c.regwrite8(reg, val);
#endif
}
