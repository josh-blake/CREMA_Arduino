/*
CrystalFontz Library for Arduino http://josh.to/crema
By Josh Blake
Version 0.1

Description: This software provides basic functionality to communicate with a CrystalFontz CFI-533 TMI (I2C) display module.
Not all functions have been implemented yet.

This software is distrubuted under the Creative Commons Attribution-ShareAlike 3.0 Unported License.
http://creativecommons.org/licenses/by-sa/3.0/deed.en_US
*/

#include "CrystalFontz.h"
#include <util/crc16.h>

CrystalFontz::CrystalFontz(uint8_t i2c_address)
{
    i2c_addr = i2c_address;	
}

CrystalFontz::CrystalFontz()
{
     i2c_addr = 0x2A;
     brightness = 2;
     contrast = 16;
}

void CrystalFontz::write(COMMAND_PACKET* packet) 
{
    Wire.beginTransmission(i2c_addr);
    Wire.write(packet->command);
    Wire.write(packet->length);
    Wire.write(packet->data, packet->length);
    Wire.write(packet->crc.asByte[0]);
    Wire.write(packet->crc.asByte[1]);
    Wire.endTransmission();
}

void CrystalFontz::print(char* string, uint8_t len, uint8_t col, uint8_t row) 
{
    COMMAND_PACKET packet;
    packet.command = 0x1F;
    packet.length = len + 2;
    packet.data[0] = col;
    packet.data[1] = row;
    for(uint8_t i=0;i<len;i++) packet.data[i+2] = string[i];
    packet.crc.asWord = CRC((uint8_t*)&packet, packet.length+2);
    write((COMMAND_PACKET*)&packet);
    delay(5);
}

/*void CrystalFontz::replyWait(unsigned char command)
{
    COMMAND_PACKET packet;
    while(Wire.requestFrom(i2c_addr,4,true))
    {
        packet.command = Wire.read();
        packet.length = Wire.read();
        packet.crc.asByte[0] = Wire.read();
        packet.crc.asByte[1] = Wire.read();
        if ((command | 0x40) && packet.command) break;
    }
}*/

void CrystalFontz::init() 
{
    clear();
}

void CrystalFontz::clear() 
{
    COMMAND_PACKET packet;
    packet.command = 0x06;
    packet.length = 0;
    packet.crc.asWord = CRC((uint8_t*)&packet, 2);
    write((COMMAND_PACKET*)&packet);
    delay(5);
}

void CrystalFontz::setCharacter(uint8_t char_num, uint8_t* rows) 
{
    COMMAND_PACKET packet;
    packet.command = 0x09;
    packet.length = 9;
    packet.data[0] = char_num;
    for(uint8_t i=0;i<8;i++) packet.data[i+1] = rows[i];
    packet.crc.asWord = CRC((uint8_t*)&packet, 11);
    write((COMMAND_PACKET*)&packet);
    delay(5);
}

void CrystalFontz::setBrightness(uint8_t val) //0 to 100
{
    brightness = val;
    COMMAND_PACKET packet;
    packet.command = 0x0E;
    packet.length = 1;
    packet.data[0] = brightness;
    packet.crc.asWord = CRC((uint8_t*)&packet, 3);
    write((COMMAND_PACKET*)&packet);
    delay(50);
}

/*void CrystalFontz::ping(unsigned char *data, unsigned short len) 
{
    COMMAND_PACKET packet;
    packet.command = 0x00;
    packet.length = len;
    for(unsigned char i=0;i<len;i++) packet.data[i+2] = data[i];
    packet.crc.asWord = CRC((unsigned char*)&packet, packet.length+2);
    write(packet);
}*/

void CrystalFontz::getKP()
{
    COMMAND_PACKET packet;
    packet.command = 0x18;
    packet.length = 0;
    packet.crc.asWord = CRC((uint8_t*)&packet, 2);
    write((COMMAND_PACKET*)&packet);
    delay(2);
    Wire.requestFrom(i2c_addr, 7);
    packet.command = Wire.read();
    packet.length = Wire.read();
    for(uint8_t i = 0; i<packet.length; i++) packet.data[i] = Wire.read();
    packet.crc.asByte[0] = Wire.read();
    packet.crc.asByte[1] = Wire.read();
    if ( CRC((uint8_t*)&packet, packet.length+2) && packet.crc.asWord ) {KP[0] = packet.data[0]; KP[1] = packet.data[1]; KP[2] = packet.data[2];}
}

uint16_t CrystalFontz::CRC(uint8_t *ptr, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while(len--) crc = _crc_ccitt_update(crc, *ptr++);
    return(~crc);
}
