/*
CrystalFontz Library for Arduino http://josh.to/crema
By Josh Blake
Version 0.1

Description: This software provides basic functionality to communicate with a CrystalFontz CFI-533 TMI (I2C) display module.
Not all functions have been implemented yet.

This software is distrubuted under the Creative Commons Attribution-ShareAlike 3.0 Unported License.
http://creativecommons.org/licenses/by-sa/3.0/deed.en_US
*/

#include <Arduino.h>
#include <Wire.h>

#ifndef CrystalFontz_h
#define CrystalFontz_h

typedef struct
{
    uint8_t command;
    uint8_t length;
    uint8_t data[24];
    union
    { 
        uint8_t asByte[2];
        uint16_t asWord;
    } crc;
} COMMAND_PACKET;

class CrystalFontz {
	
public:
    
    int i2c_addr; //Default 7-bit Address
    
    uint8_t brightness;

    uint8_t contrast;
    
    uint8_t KP[3];
	
    CrystalFontz(uint8_t i2c_address);
    
    CrystalFontz();
    
    void write(COMMAND_PACKET* packet);
    
    void print(char* string, uint8_t len, uint8_t col, uint8_t row);
    
    void init();
    
    void clear();
    
    void setCharacter(uint8_t num, uint8_t* rows);
	
    void setBrightness(uint8_t val);
    
    //void ping(unsigned char *data, unsigned short len);
    
    void getKP();
    
private:
    
    uint16_t CRC(uint8_t *ptr, uint16_t len);

};

#endif

