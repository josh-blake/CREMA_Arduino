/*
ControlleR for Espresso MAchines - CREMA (C) 2013 http://josh.to/crema
By Josh Blake
Version 0.1

This software is distrubuted under the Creative Commons Attribution-ShareAlike 3.0 Unported License.
http://creativecommons.org/licenses/by-sa/3.0/deed.en_US
*/

#include <SPI.h>
#include <Wire.h>
#include <avr/eeprom.h>
#include "pid.c"
#include "CrystalFontz.h"

#define BOILERA
//#define BOILERB
#define MENU

#define PID_RANGE 15

//PT100 Configuration & Thermal Constants - Old Values
//#define CVDA 3.9083E-3 //Callendar Van Dusen Constant A
//#define CVDB -5.775E-7 //Callendar Van Dusen Constant B
#define CVDA 3.9083E-3
#define CVDB -5.775E-7

//ADS1247 LSB and Reference Values
#define ADRBIAS 750 //Bias Resistor Value - Theoretical: 750
#define ADRO 150 //Reference Resistance - Theoretical: 150
#define ADIDAC 0.0015 //IDAC Current in Amps
#define ADPGA 32 //AD PGA Setting
//#define ADLSB ( ( (ADRBIAS * 2 * ADIDAC) / ( 2 ^ 23 - 1) ) / ( ADIDAC * ADPGA ) ) //Define AD LSB
#define ADLSB 5.5879361138267652781921956768269153627056315786399339008E-6

//Define Max and Mins
#define MAX_TEMP 150.0
#define MIN_TEMP 0.0
#define MAX_TIME 999.9
//#define MIN_TIME 0.0

//Define LCD Button Masks
#define KP_UP 0x01
#define KP_OK 0x02
#define KP_CANCEL 0x04
#define KP_LEFT 0x08
#define KP_RIGHT 0x10
#define KP_DOWN 0x20

//Define Volatile Variables
volatile boolean INTAL;
volatile boolean INTSW;
volatile uint8_t PB;
volatile uint32_t time = 0;
volatile uint32_t timeLoop = 0;
volatile uint32_t timeSleep = 0;
volatile uint8_t mode = 2;

char lcdBuffer[4][17];
//Define Custom Characters
unsigned char custChar0[] =  //Thermometer
{
  0b00000100,
  0b00001010,
  0b00001010,
  0b00001110,
  0b00011111,
  0b00011111,
  0b00001110,
  0b00000000
};

unsigned char custChar1[] =   //Brew
{
  0b0011000,
  0b0010100,
  0b0011000,
  0b0010110,
  0b0011101,
  0b0000110,
  0b0000101,
  0b0000000
};

unsigned char custChar2[] =  //Hot Water
{
  0b0010100,
  0b0011100,
  0b0010100,
  0b0010100,
  0b0001001,
  0b0001101,
  0b0001010,
  0b0000000
};

unsigned char custChar3[] =  //Steam
{
  0b0001100,
  0b0010000,
  0b0001000,
  0b0000111,
  0b0011010,
  0b0000010,
  0b0000010,
  0b0000000
};

CrystalFontz LCD = CrystalFontz();

//Define Custom Structs
typedef struct DSM3231M
{
  uint8_t address;
  
  uint8_t h;
  uint8_t m;
  uint8_t s;
  boolean p;
  boolean mode;
  
  uint8_t a1h;
  uint8_t a1m;
  uint8_t a1s;
  boolean a1p;
  boolean a1en;

  void setInt(uint8_t val)
  {
    Wire.beginTransmission(0xD0>>1);
    Wire.write(0x0F);
    Wire.write(val);
    Wire.endTransmission();
  }
  
  uint8_t getInt()
  {
    Wire.beginTransmission(0xD0>>1);
    Wire.write(0x0F);
    Wire.endTransmission();
    Wire.requestFrom(address>>1, 1, true);
    return Wire.read();
  }
  
  void getTime()
  {
    Wire.beginTransmission(address>>1);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(address>>1, 3, true);
    s = Wire.read();
    m = Wire.read();
    h = Wire.read();
    p = (boolean) (h & (1<<5));
    mode = (boolean) (h & (1<<6));
    h &= 0x1F;
  }
  
  void setTime()
  {
    Wire.beginTransmission(address>>1);
    Wire.write(0x00);
    Wire.write(s);
    Wire.write(m);
    if (mode) //12 Hour Mode
    {
      Wire.write(1<<6 | p<<5 | h);
    }
    else //24 Hour Mode
    {
      Wire.write(h);
    }
    Wire.endTransmission();
  }
  
  void getA1()
  {
    Wire.beginTransmission(address>>1);
    Wire.write(0x07);
    Wire.endTransmission();
    Wire.requestFrom(address>>1, 3, true);
    a1s = 0x7F & Wire.read();
    a1m = 0x7F & Wire.read();
    a1h = 0x7F & Wire.read();
    Wire.beginTransmission(address>>1);
    Wire.write(0x0E);
    Wire.endTransmission();
    Wire.requestFrom(address>>1, 1, true);
    a1en = 0x1 & Wire.read();
  }
  
  void setA1()
  {
    Wire.beginTransmission(address>>1);
    Wire.write(0x07);
    Wire.write(a1s);
    Wire.write(a1m);
    Wire.write((1<<6) | (a1p<<5) | a1h);
    Wire.write(1<<7);
    Wire.endTransmission();
    Wire.beginTransmission(address>>1);
    Wire.write(0x0E);
    Wire.write(0b100 | a1en);
    Wire.endTransmission();
  }
  
} DSM3231M_t;

DSM3231M_t rtc;

//Define BOILER struct
typedef struct BOILER
{
  float current_temp;
  float target_temp;
  float K_P;
  float K_I;
  float K_D;
  unsigned int pwm;
  pidData_t pid;
} boiler_t;

typedef struct PUMP
{
  void duty(uint8_t val)
  {
    switch(val)
    {
      case 11: //Marching Soldiers for Steam Boiler ... A pulse every second or so
        TCCR4C &= ~(1<<COM4D0); //Non-inverted PWM output for frequencies less 50% duty
        cli();
        TC4H = 0x3; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0xFA;
        TC4H = 0x0; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0x9;
        sei();
        TCCR4B = (1<<CS43) | (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 16384 and start timer
      break;

      case 1:
        TCCR4C &= ~(1<<COM4D0); //Non-inverted PWM output for frequencies less 50% duty
        cli();
        TC4H = 0x2; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x8B;
        TC4H = 0x0; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0x43; //0x43, 0x84, 0xC5, 0x106, 0x147
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer
      break;
      
      case 2:
        TCCR4C &= ~(1<<COM4D0); //Non-inverted PWM output for frequencies less 50% duty
        cli();
        TC4H = 0x2; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x8B;
        TC4H = 0x0; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0x84; //0x43, 0x84, 0xC5, 0x106, 0x147
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer
      break;
      
      case 3:
        TCCR4C &= ~(1<<COM4D0); //Non-inverted PWM output for frequencies less 50% duty
        cli();
        TC4H = 0x2; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x8B;
        TC4H = 0x0; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0xC5; //0x43, 0x84, 0xC5, 0x106, 0x147
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer
      break;
      
      case 4:
        TCCR4C &= ~(1<<COM4D0); //Non-inverted PWM output for frequencies less 50% duty
        cli();
        TC4H = 0x2; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x8B;
        TC4H = 0x1; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0x06; //0x43, 0x84, 0xC5, 0x106, 0x147
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer
      break;
      
      case 5:
        TCCR4C &= ~(1<<COM4D0); //Non-inverted PWM output for frequencies less 50% duty
        cli();
        TC4H = 0x2; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x8B;
        TC4H = 0x1; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0x47; //0x43, 0x84, 0xC5, 0x106, 0x147
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer
      break;
      
      case 6:
        TCCR4C |= (1<<COM4D0); //Inverted PWM output for frequencies above 50% duty
        cli();
        TC4H = 0x2; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x8B;
        TC4H = 0x1; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0x06; //0x43, 0x84, 0xC5, 0x106, 0x147
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer      
      break;
      
      case 7:
        TCCR4C |= (1<<COM4D0); //Inverted PWM output for frequencies above 50% duty
        cli();
        TC4H = 0x2; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x8B;
        TC4H = 0x0; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0xC5; //0x43, 0x84, 0xC5, 0x106, 0x147
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer
      break;
      
      case 8:
        TCCR4C |= (1<<COM4D0); //Inverted PWM output for frequencies above 50% duty
        cli();
        TC4H = 0x0; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0x43; //0x43, 0x84, 0xC5, 0x106, 0x147
        TC4H = 0x1; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x47;
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer
      break;
      
      case 9:
        TCCR4C |= (1<<COM4D0); //Inverted PWM output for frequencies above 50% duty
        cli();
        TC4H = 0x0; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0x43; //0x43, 0x84, 0xC5, 0x106, 0x147
        TC4H = 0x2; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x8B;
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer
      break;
      
      case 10:
        TCCR4C |= (1<<COM4D0); //Inverted PWM output for frequencies above 50% duty
        cli();
        TC4H = 0x2; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x8B;
        TC4H = 0x0; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0x00; //0x43, 0x84, 0xC5, 0x106, 0x147
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer
      break;
      
      default:
        TCCR4C &= ~(1<<COM4D0); //Inverted PWM output for frequencies above 50% duty
        cli();
        TC4H = 0x2; //Set 10-bit register OCR4C with TOP value 0x28B
        OCR4C = 0x8B;
        TC4H = 0x0; //Set 10-bit register OCR4D with MATCH value
        OCR4D = 0x00; //0x43, 0x84, 0xC5, 0x106, 0x147
        sei();
        TCCR4B = (1<<CS43) | (1<<CS41) | (1<<CS40); //Set prescaler to 2048 and start timer
      break;
    }
  }
} pump_t;

typedef struct PRESET
{
  float tempA;
  float tempB;
  float duration;
  uint8_t duty;
} preset_t;

typedef struct PRESETINDEX
{
  char name[17];
  preset_t* preset;
  uint8_t length;
} presetIndex_t;

//Define Presets
const preset_t presetSleep[] = {0, 0, 0, 0};
const preset_t presetWarm[] = {75, 0, 0, 0};

preset_t presetBR[] = {103.0, 0, -1, 10};
preset_t presetHW[] = {95.0, 0, -1, 10};
preset_t presetST[] = {150.0, 150.0, -1, 0};

preset_t preset0[] = {103.0, 0, 4, 6, 103.0, 0, 3, 0, 103.0, 0, -1, 10};
preset_t preset1[] = {103.0, 0, -1, 6, 103.0, 0, 3, 0, 103.0, 0, -1, 10};
preset_t preset2[] = {103.0, 0, 0, 0, 103.0, 0, 0, 0, 103.0, 0, -1, 10};

//Define Preset Index
presetIndex_t presetIndex[] = 
{
  "Hot Water",
  (preset_t*)&presetHW,
  1,
  
  "Manual Brew",
  (preset_t*)&presetBR,
  1,

  "Preset 1",
  (preset_t*)&preset0,
  3,
  
  "Preset 2",
  (preset_t*)&preset1,
  3,
  
  "Preset 3",
  (preset_t*)&preset2,
  3,
};

//Point to Current Preset
uint8_t activePresetIndex = 1;
uint8_t activePresetStep = 0;
preset_t* activePreset = (preset_t*)presetIndex[1].preset;

//Define Boilers
boiler_t boilerA;
boiler_t boilerB;

//Point to Working Boiler A
boiler_t* activeBoilerA = &boilerA;

//Point to Working Boiler B
boiler_t* activeBoilerB = &boilerB;

//Define Pump
pump_t pump;

//Point to Working Pump
pump_t* activePump = &pump;

#ifdef MENU
//Define Menu Element
typedef struct MENUELEMENT
{
  void* L0_func; const void* L0_args;
  void* L1_func; const void* L1_args;
  void* K_func; const void* K_args;
  void* L_func; const void* L_args;
  void* R_func; const void* R_args;
  void* U_func; const void* U_args;
  void* D_func; const void* D_args;
} menuElement_t;

//Null Argument
PROGMEM const void* const MA_null[] = {};

// 0 Arguments
PROGMEM const void* const MA0_L0[] = {(char*)&lcdBuffer[0], (char*)"Edit Presets"};
PROGMEM const void* const MA0_L1[] = {(char*)&lcdBuffer[1], (char*)""};
PROGMEM const void* const MA0_K[] = {(void*)9};
PROGMEM const void* const MA0_L[] = {(void*)8};
PROGMEM const void* const MA0_R[] = {(void*)1};

//1 Arguments
PROGMEM const void* const MA1_L0[] = {(char*)&lcdBuffer[0], (char*)"Edit PID"};
PROGMEM const void* const MA1_L1[] = {(char*)&lcdBuffer[1], (char*)"Parameters"};
PROGMEM const void* const MA1_L[] = {(void*)0};
PROGMEM const void* const MA1_R[] = {(void*)2};

//2 Arguments
PROGMEM const void* const MA2_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Clock"};
PROGMEM const void* const MA2_L1[] = {(char*)&lcdBuffer[1], (char*)""};
PROGMEM const void* const MA2_K[] = {(void*)59};
PROGMEM const void* const MA2_L[] = {(void*)1};
PROGMEM const void* const MA2_R[] = {(void*)3};

//3 Arguments
PROGMEM const void* const MA3_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Wakeup Time"};
PROGMEM const void* const MA3_L1[] = {(char*)&lcdBuffer[1], (char*)""};
PROGMEM const void* const MA3_K[] = {(void*)62};
PROGMEM const void* const MA3_L[] = {(void*)2};
PROGMEM const void* const MA3_R[] = {(void*)4};

//4 Arguments
PROGMEM const void* const MA4_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Sleep Delay"};
PROGMEM const void* const MA4_L1[] = {(char*)&lcdBuffer[1], (char*)""};
PROGMEM const void* const MA4_L[] = {(void*)3};
PROGMEM const void* const MA4_R[] = {(void*)5};

//5 Arguments
PROGMEM const void* const MA5_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Brightness"};
PROGMEM const void* const MA5_L1[] = {(char*)&lcdBuffer[1], (char*)""};
PROGMEM const void* const MA5_L[] = {(void*)4};
PROGMEM const void* const MA5_R[] = {(void*)6};

//6 Arguments
PROGMEM const void* const MA6_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Contrast"};
PROGMEM const void* const MA6_L1[] = {(char*)&lcdBuffer[1], (char*)""};
PROGMEM const void* const MA6_L[] = {(void*)5};
PROGMEM const void* const MA6_R[] = {(void*)7};

//7 Arguments
PROGMEM const void* const MA7_L0[] = {(char*)&lcdBuffer[0], (char*)"Restore Defaults"};
PROGMEM const void* const MA7_L1[] = {(char*)&lcdBuffer[1], (char*)""};
PROGMEM const void* const MA7_L[] = {(void*)6};
PROGMEM const void* const MA7_R[] = {(void*)8};

//8 Arguments
PROGMEM const void* const MA8_L0[] = {(char*)&lcdBuffer[0], (char*)"CREMA (C) 2013"};
PROGMEM const void* const MA8_L1[] = {(char*)&lcdBuffer[1], (char*)"http://josh.to"};
PROGMEM const void* const MA8_L[] = {(void*)7};
PROGMEM const void* const MA8_R[] = {(void*)0};

//9 Edit Preset: Manual Brew
PROGMEM const void* const MA9_L0[] = {(char*)&lcdBuffer[0], (char*)"Edit Preset:"};
PROGMEM const void* const MA9_L1[] = {(char*)&lcdBuffer[1], (char*)"Manual Brew"};
PROGMEM const void* const MA9_K[] = {(void*)15};
PROGMEM const void* const MA9_L[] = {(void*)0};
PROGMEM const void* const MA9_U[] = {(void*)14};
PROGMEM const void* const MA9_D[] = {(void*)10};

//10 Edit Preset: Hot Water
PROGMEM const void* const MA10_L1[] = {(char*)&lcdBuffer[1], (char*)"Hot Water"};
PROGMEM const void* const MA10_K[] = {(void*)19};
PROGMEM const void* const MA10_L[] = {(void*)0};
PROGMEM const void* const MA10_U[] = {(void*)9};
PROGMEM const void* const MA10_D[] = {(void*)11};

//11 Edit Preset: Steam
PROGMEM const void* const MA11_L1[] = {(char*)&lcdBuffer[1], (char*)"Steam"};
PROGMEM const void* const MA11_K[] = {(void*)21};
PROGMEM const void* const MA11_L[] = {(void*)0};
PROGMEM const void* const MA11_U[] = {(void*)10};
PROGMEM const void* const MA11_D[] = {(void*)12};

//12 Edit Preset: Preset 1
PROGMEM const void* const MA12_L1[] = {(char*)&lcdBuffer[1], (char*)"Preset 1"};
PROGMEM const void* const MA12_K[] = {(void*)23};
PROGMEM const void* const MA12_L[] = {(void*)0};
PROGMEM const void* const MA12_U[] = {(void*)11};
PROGMEM const void* const MA12_D[] = {(void*)13};

//13 Edit Preset: Preset 2
PROGMEM const void* const MA13_L1[] = {(char*)&lcdBuffer[1], (char*)"Preset 2"};
PROGMEM const void* const MA13_K[] = {(void*)35};
PROGMEM const void* const MA13_L[] = {(void*)0};
PROGMEM const void* const MA13_U[] = {(void*)12};
PROGMEM const void* const MA13_D[] = {(void*)14};

//14 Edit Preset: Preset 3
PROGMEM const void* const MA14_L1[] = {(char*)&lcdBuffer[1], (char*)"Preset 3"};
PROGMEM const void* const MA14_K[] = {(void*)47};
PROGMEM const void* const MA14_L[] = {(void*)0};
PROGMEM const void* const MA14_U[] = {(void*)13};
PROGMEM const void* const MA14_D[] = {(void*)9};

//15 Manual Brew: Temp A
PROGMEM const void* const MA15_L0[] = {(char*)&lcdBuffer[0], (char*)"MB: Temp A"};
PROGMEM const void* const MA15_L1[] = {(char*)&lcdBuffer[1], (float*)&presetBR[0].tempA};
PROGMEM const void* const MA15_K[] = {(void*)0};
PROGMEM const void* const MA15_L[] = {(void*)9};
PROGMEM const void* const MA15_R[] = {(void*)16};
PROGMEM const void* const MA15_U[] = {(float*)&presetBR[0].tempA};
PROGMEM const void* const MA15_D[] = {(float*)&presetBR[0].tempA};

//16 Manual Brew: Temp B
PROGMEM const void* const MA16_L0[] = {(char*)&lcdBuffer[0], (char*)"MB: Temp B"};
PROGMEM const void* const MA16_L1[] = {(char*)&lcdBuffer[1], (float*)&presetBR[0].tempB};
PROGMEM const void* const MA16_K[] = {(void*)0};
PROGMEM const void* const MA16_L[] = {(void*)15};
PROGMEM const void* const MA16_R[] = {(void*)17};
PROGMEM const void* const MA16_U[] = {(float*)&presetBR[0].tempB};
PROGMEM const void* const MA16_D[] = {(float*)&presetBR[0].tempB};

//17 Manual Brew: Duration
PROGMEM const void* const MA17_L0[] = {(char*)&lcdBuffer[0], (char*)"MB: Duration"};
PROGMEM const void* const MA17_L1[] = {(char*)&lcdBuffer[1], (float*)&presetBR[0].duration};
PROGMEM const void* const MA17_K[] = {(void*)0};
PROGMEM const void* const MA17_L[] = {(void*)16};
PROGMEM const void* const MA17_R[] = {(void*)18};
PROGMEM const void* const MA17_U[] = {(float*)&presetBR[0].duration};
PROGMEM const void* const MA17_D[] = {(float*)&presetBR[0].duration};

//18 Manual Brew: Duty
PROGMEM const void* const MA18_L0[] = {(char*)&lcdBuffer[0], (char*)"MB: Duty"};
PROGMEM const void* const MA18_L1[] = {(char*)&lcdBuffer[1], (float*)&presetBR[0].duty};
PROGMEM const void* const MA18_K[] = {(void*)0};
PROGMEM const void* const MA18_L[] = {(void*)17};
PROGMEM const void* const MA18_R[] = {(void*)9};
PROGMEM const void* const MA18_U[] = {(float*)&presetBR[0].duty};
PROGMEM const void* const MA18_D[] = {(float*)&presetBR[0].duty};

//19 Hot Water: Temp A
PROGMEM const void* const MA19_L0[] = {(char*)&lcdBuffer[0], (char*)"HW: Temp A"};
PROGMEM const void* const MA19_L1[] = {(char*)&lcdBuffer[1], (float*)&presetHW[0].tempA};
PROGMEM const void* const MA19_K[] = {(void*)0};
PROGMEM const void* const MA19_L[] = {(void*)10};
PROGMEM const void* const MA19_R[] = {(void*)20};
PROGMEM const void* const MA19_U[] = {(float*)&presetHW[0].tempA};
PROGMEM const void* const MA19_D[] = {(float*)&presetHW[0].tempA};

//20 Hot Water: Temp B
PROGMEM const void* const MA20_L0[] = {(char*)&lcdBuffer[0], (char*)"HW: Temp B"};
PROGMEM const void* const MA20_L1[] = {(char*)&lcdBuffer[1], (float*)&presetHW[0].tempB};
PROGMEM const void* const MA20_K[] = {(void*)0};
PROGMEM const void* const MA20_L[] = {(void*)19};
PROGMEM const void* const MA20_R[] = {(void*)10};
PROGMEM const void* const MA20_U[] = {(float*)&presetHW[0].tempB};
PROGMEM const void* const MA20_D[] = {(float*)&presetHW[0].tempB};

//21 Steam: Temp A
PROGMEM const void* const MA21_L0[] = {(char*)&lcdBuffer[0], (char*)"ST: Temp A"};
PROGMEM const void* const MA21_L1[] = {(char*)&lcdBuffer[1], (float*)&presetST[0].tempA};
PROGMEM const void* const MA21_K[] = {(void*)0};
PROGMEM const void* const MA21_L[] = {(void*)11};
PROGMEM const void* const MA21_R[] = {(void*)22};
PROGMEM const void* const MA21_U[] = {(float*)&presetST[0].tempA};
PROGMEM const void* const MA21_D[] = {(float*)&presetST[0].tempA};

//22 Steam: Temp B
PROGMEM const void* const MA22_L0[] = {(char*)&lcdBuffer[0], (char*)"ST: Temp B"};
PROGMEM const void* const MA22_L1[] = {(char*)&lcdBuffer[1], (float*)&presetST[0].tempB};
PROGMEM const void* const MA22_K[] = {(void*)0};
PROGMEM const void* const MA22_L[] = {(void*)21};
PROGMEM const void* const MA22_R[] = {(void*)11};
PROGMEM const void* const MA22_U[] = {(float*)&presetST[0].tempB};
PROGMEM const void* const MA22_D[] = {(float*)&presetST[0].tempB};

//23 P1(0): Temp A
PROGMEM const void* const MA23_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(1): Temp A"};
PROGMEM const void* const MA23_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[0].tempA};
PROGMEM const void* const MA23_K[] = {(void*)0};
PROGMEM const void* const MA23_L[] = {(void*)12};
PROGMEM const void* const MA23_R[] = {(void*)24};
PROGMEM const void* const MA23_U[] = {(float*)&preset0[0].tempA};
PROGMEM const void* const MA23_D[] = {(float*)&preset0[0].tempA};

//24 P1(0): Temp B
PROGMEM const void* const MA24_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(1): Temp B"};
PROGMEM const void* const MA24_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[0].tempB};
PROGMEM const void* const MA24_K[] = {(void*)0};
PROGMEM const void* const MA24_L[] = {(void*)23};
PROGMEM const void* const MA24_R[] = {(void*)25};
PROGMEM const void* const MA24_U[] = {(float*)&preset0[0].tempB};
PROGMEM const void* const MA24_D[] = {(float*)&preset0[0].tempB};

//25 P1(0): Duration
PROGMEM const void* const MA25_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(1): Duration"};
PROGMEM const void* const MA25_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[0].duration};
PROGMEM const void* const MA25_K[] = {(void*)0};
PROGMEM const void* const MA25_L[] = {(void*)24};
PROGMEM const void* const MA25_R[] = {(void*)26};
PROGMEM const void* const MA25_U[] = {(float*)&preset0[0].duration};
PROGMEM const void* const MA25_D[] = {(float*)&preset0[0].duration};

//26 P1(0): Duty
PROGMEM const void* const MA26_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(1): Duty"};
PROGMEM const void* const MA26_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[0].duty};
PROGMEM const void* const MA26_K[] = {(void*)0};
PROGMEM const void* const MA26_L[] = {(void*)25};
PROGMEM const void* const MA26_R[] = {(void*)27};
PROGMEM const void* const MA26_U[] = {(float*)&preset0[0].duty};
PROGMEM const void* const MA26_D[] = {(float*)&preset0[0].duty};

//27 P1(1): Temp A
PROGMEM const void* const MA27_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(2): Temp A"};
PROGMEM const void* const MA27_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[1].tempA};
PROGMEM const void* const MA27_K[] = {(void*)0};
PROGMEM const void* const MA27_L[] = {(void*)26};
PROGMEM const void* const MA27_R[] = {(void*)28};
PROGMEM const void* const MA27_U[] = {(float*)&preset0[1].tempA};
PROGMEM const void* const MA27_D[] = {(float*)&preset0[1].tempA};

//28 P1(0): Temp B
PROGMEM const void* const MA28_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(2): Temp B"};
PROGMEM const void* const MA28_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[1].tempB};
PROGMEM const void* const MA28_K[] = {(void*)0};
PROGMEM const void* const MA28_L[] = {(void*)27};
PROGMEM const void* const MA28_R[] = {(void*)29};
PROGMEM const void* const MA28_U[] = {(float*)&preset0[1].tempB};
PROGMEM const void* const MA28_D[] = {(float*)&preset0[1].tempB};

//29 P1(0): Duration
PROGMEM const void* const MA29_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(2): Duration"};
PROGMEM const void* const MA29_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[1].duration};
PROGMEM const void* const MA29_K[] = {(void*)0};
PROGMEM const void* const MA29_L[] = {(void*)28};
PROGMEM const void* const MA29_R[] = {(void*)30};
PROGMEM const void* const MA29_U[] = {(float*)&preset0[1].duration};
PROGMEM const void* const MA29_D[] = {(float*)&preset0[1].duration};

//30 P1(0): Duty
PROGMEM const void* const MA30_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(2): Duty"};
PROGMEM const void* const MA30_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[1].duty};
PROGMEM const void* const MA30_K[] = {(void*)0};
PROGMEM const void* const MA30_L[] = {(void*)29};
PROGMEM const void* const MA30_R[] = {(void*)31};
PROGMEM const void* const MA30_U[] = {(float*)&preset0[1].duty};
PROGMEM const void* const MA30_D[] = {(float*)&preset0[1].duty};

//31 P1(0): Temp A
PROGMEM const void* const MA31_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(3): Temp A"};
PROGMEM const void* const MA31_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[2].tempA};
PROGMEM const void* const MA31_K[] = {(void*)0};
PROGMEM const void* const MA31_L[] = {(void*)30};
PROGMEM const void* const MA31_R[] = {(void*)32};
PROGMEM const void* const MA31_U[] = {(float*)&preset0[2].tempA};
PROGMEM const void* const MA31_D[] = {(float*)&preset0[2].tempA};

//32 P1(0): Temp B
PROGMEM const void* const MA32_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(3): Temp B"};
PROGMEM const void* const MA32_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[2].tempB};
PROGMEM const void* const MA32_K[] = {(void*)0};
PROGMEM const void* const MA32_L[] = {(void*)31};
PROGMEM const void* const MA32_R[] = {(void*)33};
PROGMEM const void* const MA32_U[] = {(float*)&preset0[2].tempB};
PROGMEM const void* const MA32_D[] = {(float*)&preset0[2].tempB};

//33 P1(0): Duration
PROGMEM const void* const MA33_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(3): Duration"};
PROGMEM const void* const MA33_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[2].duration};
PROGMEM const void* const MA33_K[] = {(void*)0};
PROGMEM const void* const MA33_L[] = {(void*)32};
PROGMEM const void* const MA33_R[] = {(void*)34};
PROGMEM const void* const MA33_U[] = {(float*)&preset0[2].duration};
PROGMEM const void* const MA33_D[] = {(float*)&preset0[2].duration};

//34 P1(0): Duty
PROGMEM const void* const MA34_L0[] = {(char*)&lcdBuffer[0], (char*)"P1(3): Duty"};
PROGMEM const void* const MA34_L1[] = {(char*)&lcdBuffer[1], (float*)&preset0[2].duty};
PROGMEM const void* const MA34_K[] = {(void*)0};
PROGMEM const void* const MA34_L[] = {(void*)33};
PROGMEM const void* const MA34_R[] = {(void*)12};
PROGMEM const void* const MA34_U[] = {(float*)&preset0[2].duty};
PROGMEM const void* const MA34_D[] = {(float*)&preset0[2].duty};

//35 P1(0): Temp A
PROGMEM const void* const MA35_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(1): Temp A"};
PROGMEM const void* const MA35_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[0].tempA};
PROGMEM const void* const MA35_K[] = {(void*)0};
PROGMEM const void* const MA35_L[] = {(void*)13};
PROGMEM const void* const MA35_R[] = {(void*)36};
PROGMEM const void* const MA35_U[] = {(float*)&preset1[0].tempA};
PROGMEM const void* const MA35_D[] = {(float*)&preset1[0].tempA};

//36 P1(0): Temp B
PROGMEM const void* const MA36_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(1): Temp B"};
PROGMEM const void* const MA36_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[0].tempB};
PROGMEM const void* const MA36_K[] = {(void*)0};
PROGMEM const void* const MA36_L[] = {(void*)35};
PROGMEM const void* const MA36_R[] = {(void*)37};
PROGMEM const void* const MA36_U[] = {(float*)&preset1[0].tempB};
PROGMEM const void* const MA36_D[] = {(float*)&preset1[0].tempB};

//37 P1(0): Duration
PROGMEM const void* const MA37_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(1): Duration"};
PROGMEM const void* const MA37_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[0].duration};
PROGMEM const void* const MA37_K[] = {(void*)0};
PROGMEM const void* const MA37_L[] = {(void*)36};
PROGMEM const void* const MA37_R[] = {(void*)38};
PROGMEM const void* const MA37_U[] = {(float*)&preset1[0].duration};
PROGMEM const void* const MA37_D[] = {(float*)&preset1[0].duration};

//38 P1(0): Duty
PROGMEM const void* const MA38_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(1): Duty"};
PROGMEM const void* const MA38_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[0].duty};
PROGMEM const void* const MA38_K[] = {(void*)0};
PROGMEM const void* const MA38_L[] = {(void*)37};
PROGMEM const void* const MA38_R[] = {(void*)39};
PROGMEM const void* const MA38_U[] = {(float*)&preset1[0].duty};
PROGMEM const void* const MA38_D[] = {(float*)&preset1[0].duty};

//39 P1(1): Temp A
PROGMEM const void* const MA39_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(2): Temp A"};
PROGMEM const void* const MA39_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[1].tempA};
PROGMEM const void* const MA39_K[] = {(void*)0};
PROGMEM const void* const MA39_L[] = {(void*)38};
PROGMEM const void* const MA39_R[] = {(void*)40};
PROGMEM const void* const MA39_U[] = {(float*)&preset1[1].tempA};
PROGMEM const void* const MA39_D[] = {(float*)&preset1[1].tempA};

//40 P1(0): Temp B
PROGMEM const void* const MA40_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(2): Temp B"};
PROGMEM const void* const MA40_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[1].tempB};
PROGMEM const void* const MA40_K[] = {(void*)0};
PROGMEM const void* const MA40_L[] = {(void*)39};
PROGMEM const void* const MA40_R[] = {(void*)41};
PROGMEM const void* const MA40_U[] = {(float*)&preset1[1].tempB};
PROGMEM const void* const MA40_D[] = {(float*)&preset1[1].tempB};

//41 P1(0): Duration
PROGMEM const void* const MA41_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(2): Duration"};
PROGMEM const void* const MA41_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[1].duration};
PROGMEM const void* const MA41_K[] = {(void*)0};
PROGMEM const void* const MA41_L[] = {(void*)40};
PROGMEM const void* const MA41_R[] = {(void*)42};
PROGMEM const void* const MA41_U[] = {(float*)&preset1[1].duration};
PROGMEM const void* const MA41_D[] = {(float*)&preset1[1].duration};

//42 P1(0): Duty
PROGMEM const void* const MA42_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(2): Duty"};
PROGMEM const void* const MA42_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[1].duty};
PROGMEM const void* const MA42_K[] = {(void*)0};
PROGMEM const void* const MA42_L[] = {(void*)41};
PROGMEM const void* const MA42_R[] = {(void*)43};
PROGMEM const void* const MA42_U[] = {(float*)&preset1[1].duty};
PROGMEM const void* const MA42_D[] = {(float*)&preset1[1].duty};

//43 P1(0): Temp A
PROGMEM const void* const MA43_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(3): Temp A"};
PROGMEM const void* const MA43_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[2].tempA};
PROGMEM const void* const MA43_K[] = {(void*)0};
PROGMEM const void* const MA43_L[] = {(void*)42};
PROGMEM const void* const MA43_R[] = {(void*)44};
PROGMEM const void* const MA43_U[] = {(float*)&preset1[2].tempA};
PROGMEM const void* const MA43_D[] = {(float*)&preset1[2].tempA};

//44 P1(0): Temp B
PROGMEM const void* const MA44_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(3): Temp B"};
PROGMEM const void* const MA44_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[2].tempB};
PROGMEM const void* const MA44_K[] = {(void*)0};
PROGMEM const void* const MA44_L[] = {(void*)43};
PROGMEM const void* const MA44_R[] = {(void*)45};
PROGMEM const void* const MA44_U[] = {(float*)&preset1[2].tempB};
PROGMEM const void* const MA44_D[] = {(float*)&preset1[2].tempB};

//45 P1(0): Duration
PROGMEM const void* const MA45_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(3): Duration"};
PROGMEM const void* const MA45_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[2].duration};
PROGMEM const void* const MA45_K[] = {(void*)0};
PROGMEM const void* const MA45_L[] = {(void*)44};
PROGMEM const void* const MA45_R[] = {(void*)46};
PROGMEM const void* const MA45_U[] = {(float*)&preset1[2].duration};
PROGMEM const void* const MA45_D[] = {(float*)&preset1[2].duration};

//46 P1(0): Duty
PROGMEM const void* const MA46_L0[] = {(char*)&lcdBuffer[0], (char*)"P2(3): Duty"};
PROGMEM const void* const MA46_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[2].duty};
PROGMEM const void* const MA46_K[] = {(void*)0};
PROGMEM const void* const MA46_L[] = {(void*)45};
PROGMEM const void* const MA46_R[] = {(void*)13};
PROGMEM const void* const MA46_U[] = {(float*)&preset1[2].duty};
PROGMEM const void* const MA46_D[] = {(float*)&preset1[2].duty};

//47 P1(0): Temp A
PROGMEM const void* const MA47_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(1): Temp A"};
PROGMEM const void* const MA47_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[0].tempA};
PROGMEM const void* const MA47_K[] = {(void*)0};
PROGMEM const void* const MA47_L[] = {(void*)14};
PROGMEM const void* const MA47_R[] = {(void*)48};
PROGMEM const void* const MA47_U[] = {(float*)&preset2[0].tempA};
PROGMEM const void* const MA47_D[] = {(float*)&preset2[0].tempA};

//48 P1(0): Temp B
PROGMEM const void* const MA48_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(1): Temp B"};
PROGMEM const void* const MA48_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[0].tempB};
PROGMEM const void* const MA48_K[] = {(void*)0};
PROGMEM const void* const MA48_L[] = {(void*)47};
PROGMEM const void* const MA48_R[] = {(void*)49};
PROGMEM const void* const MA48_U[] = {(float*)&preset2[0].tempB};
PROGMEM const void* const MA48_D[] = {(float*)&preset2[0].tempB};

//49 P1(0): Duration
PROGMEM const void* const MA49_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(1): Duration"};
PROGMEM const void* const MA49_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[0].duration};
PROGMEM const void* const MA49_K[] = {(void*)0};
PROGMEM const void* const MA49_L[] = {(void*)48};
PROGMEM const void* const MA49_R[] = {(void*)50};
PROGMEM const void* const MA49_U[] = {(float*)&preset2[0].duration};
PROGMEM const void* const MA49_D[] = {(float*)&preset2[0].duration};

//50 P1(0): Duty
PROGMEM const void* const MA50_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(1): Duty"};
PROGMEM const void* const MA50_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[0].duty};
PROGMEM const void* const MA50_K[] = {(void*)0};
PROGMEM const void* const MA50_L[] = {(void*)49};
PROGMEM const void* const MA50_R[] = {(void*)51};
PROGMEM const void* const MA50_U[] = {(float*)&preset2[0].duty};
PROGMEM const void* const MA50_D[] = {(float*)&preset2[0].duty};

//51 P1(1): Temp A
PROGMEM const void* const MA51_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(2): Temp A"};
PROGMEM const void* const MA51_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[1].tempA};
PROGMEM const void* const MA51_K[] = {(void*)0};
PROGMEM const void* const MA51_L[] = {(void*)50};
PROGMEM const void* const MA51_R[] = {(void*)52};
PROGMEM const void* const MA51_U[] = {(float*)&preset2[1].tempA};
PROGMEM const void* const MA51_D[] = {(float*)&preset2[1].tempA};

//52 P1(0): Temp B
PROGMEM const void* const MA52_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(2): Temp B"};
PROGMEM const void* const MA52_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[1].tempB};
PROGMEM const void* const MA52_K[] = {(void*)0};
PROGMEM const void* const MA52_L[] = {(void*)51};
PROGMEM const void* const MA52_R[] = {(void*)53};
PROGMEM const void* const MA52_U[] = {(float*)&preset2[1].tempB};
PROGMEM const void* const MA52_D[] = {(float*)&preset2[1].tempB};

//53 P1(0): Duration
PROGMEM const void* const MA53_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(2): Duration"};
PROGMEM const void* const MA53_L1[] = {(char*)&lcdBuffer[1], (float*)&preset1[1].duration};
PROGMEM const void* const MA53_K[] = {(void*)0};
PROGMEM const void* const MA53_L[] = {(void*)52};
PROGMEM const void* const MA53_R[] = {(void*)54};
PROGMEM const void* const MA53_U[] = {(float*)&preset2[1].duration};
PROGMEM const void* const MA53_D[] = {(float*)&preset2[1].duration};

//54 P1(0): Duty
PROGMEM const void* const MA54_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(2): Duty"};
PROGMEM const void* const MA54_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[1].duty};
PROGMEM const void* const MA54_K[] = {(void*)0};
PROGMEM const void* const MA54_L[] = {(void*)53};
PROGMEM const void* const MA54_R[] = {(void*)55};
PROGMEM const void* const MA54_U[] = {(float*)&preset2[1].duty};
PROGMEM const void* const MA54_D[] = {(float*)&preset2[1].duty};

//55 P1(0): Temp A
PROGMEM const void* const MA55_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(3): Temp A"};
PROGMEM const void* const MA55_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[2].tempA};
PROGMEM const void* const MA55_K[] = {(void*)0};
PROGMEM const void* const MA55_L[] = {(void*)54};
PROGMEM const void* const MA55_R[] = {(void*)56};
PROGMEM const void* const MA55_U[] = {(float*)&preset2[2].tempA};
PROGMEM const void* const MA55_D[] = {(float*)&preset2[2].tempA};

//56 P1(0): Temp B
PROGMEM const void* const MA56_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(3): Temp B"};
PROGMEM const void* const MA56_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[2].tempB};
PROGMEM const void* const MA56_K[] = {(void*)0};
PROGMEM const void* const MA56_L[] = {(void*)55};
PROGMEM const void* const MA56_R[] = {(void*)57};
PROGMEM const void* const MA56_U[] = {(float*)&preset2[2].tempB};
PROGMEM const void* const MA56_D[] = {(float*)&preset2[2].tempB};

//57 P1(0): Duration
PROGMEM const void* const MA57_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(3): Duration"};
PROGMEM const void* const MA57_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[2].duration};
PROGMEM const void* const MA57_K[] = {(void*)0};
PROGMEM const void* const MA57_L[] = {(void*)56};
PROGMEM const void* const MA57_R[] = {(void*)58};
PROGMEM const void* const MA57_U[] = {(float*)&preset2[2].duration};
PROGMEM const void* const MA57_D[] = {(float*)&preset2[2].duration};

//58 P1(0): Duty
PROGMEM const void* const MA58_L0[] = {(char*)&lcdBuffer[0], (char*)"P3(3): Duty"};
PROGMEM const void* const MA58_L1[] = {(char*)&lcdBuffer[1], (float*)&preset2[2].duty};
PROGMEM const void* const MA58_K[] = {(void*)0};
PROGMEM const void* const MA58_L[] = {(void*)57};
PROGMEM const void* const MA58_R[] = {(void*)14};
PROGMEM const void* const MA58_U[] = {(float*)&preset2[2].duty};
PROGMEM const void* const MA58_D[] = {(float*)&preset2[2].duty};

//59 Set Time H
PROGMEM const void* const MA59_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Time (HH)"};
PROGMEM const void* const MA59_L1[] = {(char*)&lcdBuffer[1]};
PROGMEM const void* const MA59_K[] = {(void*)0};
PROGMEM const void* const MA59_L[] = {(void*)2};
PROGMEM const void* const MA59_R[] = {(void*)60};
PROGMEM const void* const MA59_U[] = {(void*)&rtc.h, (uint8_t*)0x12};
PROGMEM const void* const MA59_D[] = {(void*)&rtc.h, (uint8_t*)0x12};

//60 Set Time M
PROGMEM const void* const MA60_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Time (MM)"};
PROGMEM const void* const MA60_L1[] = {(char*)&lcdBuffer[1]};
PROGMEM const void* const MA60_K[] = {(void*)0};
PROGMEM const void* const MA60_L[] = {(void*)59};
PROGMEM const void* const MA60_R[] = {(void*)61};
PROGMEM const void* const MA60_U[] = {(void*)&rtc.m, (uint8_t*)0x59};
PROGMEM const void* const MA60_D[] = {(void*)&rtc.m, (uint8_t*)0x59};

//61 Set Time AP
PROGMEM const void* const MA61_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Time (A/P)"};
PROGMEM const void* const MA61_L1[] = {(char*)&lcdBuffer[1]};
PROGMEM const void* const MA61_K[] = {(void*)0};
PROGMEM const void* const MA61_L[] = {(void*)60};
PROGMEM const void* const MA61_R[] = {(void*)2};
PROGMEM const void* const MA61_U[] = {(void*)&rtc.p};
PROGMEM const void* const MA61_D[] = {(void*)&rtc.p};

//62 Set Wakeup H
PROGMEM const void* const MA62_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Wakeup (HH)"};
PROGMEM const void* const MA62_L1[] = {(char*)&lcdBuffer[1]};
PROGMEM const void* const MA62_K[] = {(void*)0};
PROGMEM const void* const MA62_L[] = {(void*)3};
PROGMEM const void* const MA62_R[] = {(void*)63};
PROGMEM const void* const MA62_U[] = {(void*)&rtc.a1h, (uint8_t*)0x12};
PROGMEM const void* const MA62_D[] = {(void*)&rtc.a1h, (uint8_t*)0x12};

//63 Set Wakeup M
PROGMEM const void* const MA63_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Wakeup (MM)"};
PROGMEM const void* const MA63_L1[] = {(char*)&lcdBuffer[1]};
PROGMEM const void* const MA63_K[] = {(void*)0};
PROGMEM const void* const MA63_L[] = {(void*)62};
PROGMEM const void* const MA63_R[] = {(void*)64};
PROGMEM const void* const MA63_U[] = {(void*)&rtc.a1m, (uint8_t*)0x59};
PROGMEM const void* const MA63_D[] = {(void*)&rtc.a1m, (uint8_t*)0x59};

//64 Set Wakeup AP
PROGMEM const void* const MA64_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Wakeup (A/P)"};
PROGMEM const void* const MA64_L1[] = {(char*)&lcdBuffer[1]};
PROGMEM const void* const MA64_K[] = {(void*)0};
PROGMEM const void* const MA64_L[] = {(void*)63};
PROGMEM const void* const MA64_R[] = {(void*)65};
PROGMEM const void* const MA64_U[] = {(void*)&rtc.a1p};
PROGMEM const void* const MA64_D[] = {(void*)&rtc.a1p};

//65 Set Wakeup Enabled?
PROGMEM const void* const MA65_L0[] = {(char*)&lcdBuffer[0], (char*)"Set Wakeup?"};
PROGMEM const void* const MA65_L1[] = {(char*)&lcdBuffer[1]};
PROGMEM const void* const MA65_K[] = {(void*)0};
PROGMEM const void* const MA65_L[] = {(void*)64};
PROGMEM const void* const MA65_R[] = {(void*)3};
PROGMEM const void* const MA65_U[] = {(void*)&rtc.a1en};
PROGMEM const void* const MA65_D[] = {(void*)&rtc.a1en};

//66 
PROGMEM const void* const MA66_L0[] = {(char*)&lcdBuffer[0], (char*)"PID Parameters:"};
PROGMEM const void* const MA66_L1[] = {(char*)&lcdBuffer[1], (char*)"Boiler A"};
PROGMEM const void* const MA66_K[] = {(void*)68};
PROGMEM const void* const MA66_L[] = {(void*)1};
PROGMEM const void* const MA66_R[] = {(void*)67};

//67
PROGMEM const void* const MA67_L0[] = {(char*)&lcdBuffer[0], (char*)"PID Parameters:"};
PROGMEM const void* const MA67_L1[] = {(char*)&lcdBuffer[1], (char*)"Boiler B"};
PROGMEM const void* const MA67_K[] = {(void*)71};
PROGMEM const void* const MA67_L[] = {(void*)66};
PROGMEM const void* const MA67_R[] = {(void*)1};
/*
//68 B1:Kp
PROGMEM const void* const MA68_L0[] = {(char*)&lcdBuffer[0], (char*)"Boiler A: Kp"};
PROGMEM const void* const MA68_L1[] = {(char*)&lcdBuffer[1], (float*)&boilerA.K_P};
PROGMEM const void* const MA68_K[] = {(void*)0};
PROGMEM const void* const MA68_L[] = {(void*)66};
PROGMEM const void* const MA68_R[] = {(void*)69};
PROGMEM const void* const MA68_U[] = {(float*)&boilerA.K_P};
PROGMEM const void* const MA68_D[] = {(float*)&boilerA.K_P};

//69 B1:Ki
PROGMEM const void* const MA69_L0[] = {(char*)&lcdBuffer[0], (char*)"Boiler A: Ki"};
PROGMEM const void* const MA69_L1[] = {(char*)&lcdBuffer[1], (float*)&boilerA.K_I};
PROGMEM const void* const MA69_K[] = {(void*)0};
PROGMEM const void* const MA69_L[] = {(void*)68};
PROGMEM const void* const MA69_R[] = {(void*)70};
PROGMEM const void* const MA69_U[] = {(float*)&boilerA.K_I};
PROGMEM const void* const MA69_D[] = {(float*)&boilerA.K_I};

//70 B1:Kd
PROGMEM const void* const MA70_L0[] = {(char*)&lcdBuffer[0], (char*)"Boiler A: Kd"};
PROGMEM const void* const MA70_L1[] = {(char*)&lcdBuffer[1], (float*)&boilerA.K_D};
PROGMEM const void* const MA70_K[] = {(void*)0};
PROGMEM const void* const MA70_L[] = {(void*)69};
PROGMEM const void* const MA70_R[] = {(void*)66};
PROGMEM const void* const MA70_U[] = {(float*)&boilerA.K_D};
PROGMEM const void* const MA70_D[] = {(float*)&boilerA.K_D};

//71 B1:Kp
PROGMEM const void* const MA71_L0[] = {(char*)&lcdBuffer[0], (char*)"Boiler B: Kp"};
PROGMEM const void* const MA71_L1[] = {(char*)&lcdBuffer[1], (float*)&boilerB.K_P};
PROGMEM const void* const MA71_K[] = {(void*)0};
PROGMEM const void* const MA71_L[] = {(void*)67};
PROGMEM const void* const MA71_R[] = {(void*)72};
PROGMEM const void* const MA71_U[] = {(float*)&boilerB.K_P};
PROGMEM const void* const MA71_D[] = {(float*)&boilerB.K_P};

//72 B1:Kp
PROGMEM const void* const MA72_L0[] = {(char*)&lcdBuffer[0], (char*)"Boiler B: Ki"};
PROGMEM const void* const MA72_L1[] = {(char*)&lcdBuffer[1], (float*)&boilerB.K_I};
PROGMEM const void* const MA72_K[] = {(void*)0};
PROGMEM const void* const MA72_L[] = {(void*)71};
PROGMEM const void* const MA72_R[] = {(void*)73};
PROGMEM const void* const MA72_U[] = {(float*)&boilerB.K_I};
PROGMEM const void* const MA72_D[] = {(float*)&boilerB.K_I};

//73 B1:Kp
PROGMEM const void* const MA73_L0[] = {(char*)&lcdBuffer[0], (char*)"Boiler B: Kd"};
PROGMEM const void* const MA73_L1[] = {(char*)&lcdBuffer[1], (float*)&boilerB.K_D};
PROGMEM const void* const MA73_K[] = {(void*)0};
PROGMEM const void* const MA73_L[] = {(void*)72};
PROGMEM const void* const MA73_R[] = {(void*)67};
PROGMEM const void* const MA73_U[] = {(float*)&boilerB.K_D};
PROGMEM const void* const MA73_D[] = {(float*)&boilerB.K_D};
*/
//Menu Function:Argument Array
PROGMEM const menuElement_t menu[] =
{
// 0 - Edit Presets
  (void*)menuDisplayText_P, MA0_L0,
  (void*)menuDisplayText_P, MA0_L1,
  (void*)menuGoto_P, MA0_K,
  (void*)menuGoto_P, MA0_L,
  (void*)menuGoto_P, MA0_R,
  (void*)menuIdle, MA_null,
  (void*)menuIdle, MA_null,
// 1 - Edit PID Parameters
  (void*)menuDisplayText_P, MA1_L0,
  (void*)menuDisplayText_P, MA1_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA1_L,
  (void*)menuGoto_P, MA1_R,
  (void*)menuIdle, MA_null,
  (void*)menuIdle, MA_null,
// 2 - Set Time
  (void*)menuDisplayText_P, MA2_L0,
  (void*)menuDisplayText_P, MA2_L1,
  (void*)menuGoto_P, MA2_K,
  (void*)menuGoto_P, MA2_L,
  (void*)menuGoto_P, MA2_R,
  (void*)menuIdle, MA_null,
  (void*)menuIdle, MA_null,
// 3 - Set Wakeup Time
  (void*)menuDisplayText_P, MA3_L0,
  (void*)menuDisplayText_P, MA3_L1,
  (void*)menuGoto_P, MA3_K,
  (void*)menuGoto_P, MA3_L,
  (void*)menuGoto_P, MA3_R,
  (void*)menuIdle, MA_null,
  (void*)menuIdle, MA_null,
// 4 - Set Sleep Delay  
  (void*)menuDisplayText_P, MA4_L0,
  (void*)menuDisplayText_P, MA4_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA4_L,
  (void*)menuGoto_P, MA4_R,
  (void*)menuIdle, MA_null,
  (void*)menuIdle, MA_null,
// 5 - Set Brightness
  (void*)menuDisplayText_P, MA5_L0,
  (void*)menuDisplayText_P, MA5_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA5_L,
  (void*)menuGoto_P, MA5_R,
  (void*)menuIdle, MA_null,
  (void*)menuIdle, MA_null,
// 6 - Set Contrast
  (void*)menuDisplayText_P, MA6_L0,
  (void*)menuDisplayText_P, MA6_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA6_L,
  (void*)menuGoto_P, MA6_R,
  (void*)menuIdle, MA_null,
  (void*)menuIdle, MA_null,
 // 7 - Restore Defaults
  (void*)menuDisplayText_P, MA7_L0,
  (void*)menuDisplayText_P, MA7_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA7_L,
  (void*)menuGoto_P, MA7_R,
  (void*)menuIdle, MA_null,
  (void*)menuIdle, MA_null,
 // 8 - About
  (void*)menuDisplayText_P, MA8_L0,
  (void*)menuDisplayText_P, MA8_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA8_L,
  (void*)menuGoto_P, MA8_R,
  (void*)menuIdle, MA_null,
  (void*)menuIdle, MA_null,
// 9 - Edit Preset: Manual Brew
  (void*)menuDisplayText_P, MA9_L0,
  (void*)menuDisplayText_P, MA9_L1,
  (void*)menuGoto_P, MA9_K,
  (void*)menuGoto_P, MA9_L,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA9_U,
  (void*)menuGoto_P, MA9_D,
// 10 - Edit Preset: Hot Water
  (void*)menuDisplayText_P, MA9_L0,
  (void*)menuDisplayText_P, MA10_L1,
  (void*)menuGoto_P, MA10_K,
  (void*)menuGoto_P, MA10_L,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA10_U,
  (void*)menuGoto_P, MA10_D,
// 11 - Edit Preset: Steam
  (void*)menuDisplayText_P, MA9_L0,
  (void*)menuDisplayText_P, MA11_L1,
  (void*)menuGoto_P, MA11_K,
  (void*)menuGoto_P, MA11_L,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA11_U,
  (void*)menuGoto_P, MA11_D,
// 12 - Edit Preset: Preset 1
  (void*)menuDisplayText_P, MA9_L0,
  (void*)menuDisplayText_P, MA12_L1,
  (void*)menuGoto_P, MA12_K,
  (void*)menuGoto_P, MA12_L,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA12_U,
  (void*)menuGoto_P, MA12_D,
// 13 - Edit Preset: Preset 2
  (void*)menuDisplayText_P, MA9_L0,
  (void*)menuDisplayText_P, MA13_L1,
  (void*)menuGoto_P, MA13_K,
  (void*)menuGoto_P, MA13_L,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA13_U,
  (void*)menuGoto_P, MA13_D,
// 14 - Edit Preset: Preset 3
  (void*)menuDisplayText_P, MA9_L0,
  (void*)menuDisplayText_P, MA14_L1,
  (void*)menuGoto_P, MA14_K,
  (void*)menuGoto_P, MA14_L,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA14_U,
  (void*)menuGoto_P, MA14_D,
// 15 - MB: Temp A
  (void*)menuDisplayText_P, MA15_L0,
  (void*)menuDisplayPresetTemp_P, MA15_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA15_L,
  (void*)menuGoto_P, MA15_R,
  (void*)menuPresetIncTemp_P, MA15_U,
  (void*)menuPresetDecTemp_P, MA15_D, 
// 16 - MB: Temp B
  (void*)menuDisplayText_P, MA16_L0,
  (void*)menuDisplayPresetTemp_P, MA16_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA16_L,
  (void*)menuGoto_P, MA16_R,
  (void*)menuPresetIncTemp_P, MA16_U,
  (void*)menuPresetDecTemp_P, MA16_D,  
 // 17 - MB: Duration
  (void*)menuDisplayText_P, MA17_L0,
  (void*)menuDisplayPresetTime_P, MA17_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA17_L,
  (void*)menuGoto_P, MA17_R,
  (void*)menuPresetIncTime_P, MA17_U,
  (void*)menuPresetDecTime_P, MA17_D,  
 // 18 - MB: Duty
  (void*)menuDisplayText_P, MA18_L0,
  (void*)menuDisplayPresetDuty_P, MA18_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA18_L,
  (void*)menuGoto_P, MA18_R,
  (void*)menuPresetIncDuty_P, MA18_U,
  (void*)menuPresetDecDuty_P, MA18_D,
 // 19 - HW: Temp A
  (void*)menuDisplayText_P, MA19_L0,
  (void*)menuDisplayPresetTemp_P, MA19_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA19_L,
  (void*)menuGoto_P, MA19_R,
  (void*)menuPresetIncTemp_P, MA19_U,
  (void*)menuPresetDecTemp_P, MA19_D,
 // 20 - HW: Temp B
  (void*)menuDisplayText_P, MA20_L0,
  (void*)menuDisplayPresetTemp_P, MA20_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA20_L,
  (void*)menuGoto_P, MA20_R,
  (void*)menuPresetIncTemp_P, MA20_U,
  (void*)menuPresetDecTemp_P, MA20_D,
 // 21 - Steam: Temp A
  (void*)menuDisplayText_P, MA21_L0,
  (void*)menuDisplayPresetTemp_P, MA21_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA21_L,
  (void*)menuGoto_P, MA21_R,
  (void*)menuPresetIncTemp_P, MA21_U,
  (void*)menuPresetDecTemp_P, MA21_D,
 // 22 - Steam: Temp B
  (void*)menuDisplayText_P, MA22_L0,
  (void*)menuDisplayPresetTemp_P, MA22_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA22_L,
  (void*)menuGoto_P, MA22_R,
  (void*)menuPresetIncTemp_P, MA22_U,
  (void*)menuPresetDecTemp_P, MA22_D,

// 23 - P1(0): Temp A
  (void*)menuDisplayText_P, MA23_L0,
  (void*)menuDisplayPresetTemp_P, MA23_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA23_L,
  (void*)menuGoto_P, MA23_R,
  (void*)menuPresetIncTemp_P, MA23_U,
  (void*)menuPresetDecTemp_P, MA23_D, 
// 24 - P1(0): Temp B
  (void*)menuDisplayText_P, MA24_L0,
  (void*)menuDisplayPresetTemp_P, MA24_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA24_L,
  (void*)menuGoto_P, MA24_R,
  (void*)menuPresetIncTemp_P, MA24_U,
  (void*)menuPresetDecTemp_P, MA24_D,  
 // 25 - P1(0): Duration
  (void*)menuDisplayText_P, MA25_L0,
  (void*)menuDisplayPresetTime_P, MA25_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA25_L,
  (void*)menuGoto_P, MA25_R,
  (void*)menuPresetIncTime_P, MA25_U,
  (void*)menuPresetDecTime_P, MA25_D,  
 // 26 - P1(0): Duty
  (void*)menuDisplayText_P, MA26_L0,
  (void*)menuDisplayPresetDuty_P, MA26_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA26_L,
  (void*)menuGoto_P, MA26_R,
  (void*)menuPresetIncDuty_P, MA26_U,
  (void*)menuPresetDecDuty_P, MA26_D,
  // 27 - P1(0): Temp A
  (void*)menuDisplayText_P, MA27_L0,
  (void*)menuDisplayPresetTemp_P, MA27_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA27_L,
  (void*)menuGoto_P, MA27_R,
  (void*)menuPresetIncTemp_P, MA27_U,
  (void*)menuPresetDecTemp_P, MA27_D, 
// 28 - P1(0): Temp B
  (void*)menuDisplayText_P, MA28_L0,
  (void*)menuDisplayPresetTemp_P, MA28_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA28_L,
  (void*)menuGoto_P, MA28_R,
  (void*)menuPresetIncTemp_P, MA28_U,
  (void*)menuPresetDecTemp_P, MA28_D,  
 // 29 - P1(0): Duration
  (void*)menuDisplayText_P, MA29_L0,
  (void*)menuDisplayPresetTime_P, MA29_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA29_L,
  (void*)menuGoto_P, MA29_R,
  (void*)menuPresetIncTime_P, MA29_U,
  (void*)menuPresetDecTime_P, MA29_D,  
 // 30 - P1(0): Duty
  (void*)menuDisplayText_P, MA30_L0,
  (void*)menuDisplayPresetDuty_P, MA30_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA30_L,
  (void*)menuGoto_P, MA30_R,
  (void*)menuPresetIncDuty_P, MA30_U,
  (void*)menuPresetDecDuty_P, MA30_D,
  // 31 - P1(0): Temp A
  (void*)menuDisplayText_P, MA31_L0,
  (void*)menuDisplayPresetTemp_P, MA31_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA31_L,
  (void*)menuGoto_P, MA31_R,
  (void*)menuPresetIncTemp_P, MA31_U,
  (void*)menuPresetDecTemp_P, MA31_D, 
// 32 - P1(0): Temp B
  (void*)menuDisplayText_P, MA32_L0,
  (void*)menuDisplayPresetTemp_P, MA32_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA32_L,
  (void*)menuGoto_P, MA32_R,
  (void*)menuPresetIncTemp_P, MA32_U,
  (void*)menuPresetDecTemp_P, MA32_D,  
 // 33 - P1(0): Duration
  (void*)menuDisplayText_P, MA33_L0,
  (void*)menuDisplayPresetTime_P, MA33_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA33_L,
  (void*)menuGoto_P, MA33_R,
  (void*)menuPresetIncTime_P, MA33_U,
  (void*)menuPresetDecTime_P, MA33_D,  
 // 34 - P1(0): Duty
  (void*)menuDisplayText_P, MA34_L0,
  (void*)menuDisplayPresetDuty_P, MA34_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA34_L,
  (void*)menuGoto_P, MA34_R,
  (void*)menuPresetIncDuty_P, MA34_U,
  (void*)menuPresetDecDuty_P, MA34_D,

// 35 - P1(0): Temp A
  (void*)menuDisplayText_P, MA35_L0,
  (void*)menuDisplayPresetTemp_P, MA35_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA35_L,
  (void*)menuGoto_P, MA35_R,
  (void*)menuPresetIncTemp_P, MA35_U,
  (void*)menuPresetDecTemp_P, MA35_D, 
// 36 - P1(0): Temp B
  (void*)menuDisplayText_P, MA36_L0,
  (void*)menuDisplayPresetTemp_P, MA36_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA36_L,
  (void*)menuGoto_P, MA36_R,
  (void*)menuPresetIncTemp_P, MA36_U,
  (void*)menuPresetDecTemp_P, MA36_D,  
 // 37 - P1(0): Duration
  (void*)menuDisplayText_P, MA37_L0,
  (void*)menuDisplayPresetTime_P, MA37_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA37_L,
  (void*)menuGoto_P, MA37_R,
  (void*)menuPresetIncTime_P, MA37_U,
  (void*)menuPresetDecTime_P, MA37_D,  
 // 38 - P1(0): Duty
  (void*)menuDisplayText_P, MA38_L0,
  (void*)menuDisplayPresetDuty_P, MA38_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA38_L,
  (void*)menuGoto_P, MA38_R,
  (void*)menuPresetIncDuty_P, MA38_U,
  (void*)menuPresetDecDuty_P, MA38_D,
  // 39 - P1(0): Temp A
  (void*)menuDisplayText_P, MA39_L0,
  (void*)menuDisplayPresetTemp_P, MA39_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA39_L,
  (void*)menuGoto_P, MA39_R,
  (void*)menuPresetIncTemp_P, MA39_U,
  (void*)menuPresetDecTemp_P, MA39_D, 
// 40 - P1(0): Temp B
  (void*)menuDisplayText_P, MA40_L0,
  (void*)menuDisplayPresetTemp_P, MA40_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA40_L,
  (void*)menuGoto_P, MA40_R,
  (void*)menuPresetIncTemp_P, MA40_U,
  (void*)menuPresetDecTemp_P, MA40_D,  
 // 41 - P1(0): Duration
  (void*)menuDisplayText_P, MA41_L0,
  (void*)menuDisplayPresetTime_P, MA41_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA41_L,
  (void*)menuGoto_P, MA41_R,
  (void*)menuPresetIncTime_P, MA41_U,
  (void*)menuPresetDecTime_P, MA41_D,  
 // 42 - P1(0): Duty
  (void*)menuDisplayText_P, MA42_L0,
  (void*)menuDisplayPresetDuty_P, MA42_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA42_L,
  (void*)menuGoto_P, MA42_R,
  (void*)menuPresetIncDuty_P, MA42_U,
  (void*)menuPresetDecDuty_P, MA42_D,
  // 43 - P1(0): Temp A
  (void*)menuDisplayText_P, MA43_L0,
  (void*)menuDisplayPresetTemp_P, MA43_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA43_L,
  (void*)menuGoto_P, MA43_R,
  (void*)menuPresetIncTemp_P, MA43_U,
  (void*)menuPresetDecTemp_P, MA43_D, 
// 44 - P1(0): Temp B
  (void*)menuDisplayText_P, MA44_L0,
  (void*)menuDisplayPresetTemp_P, MA44_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA44_L,
  (void*)menuGoto_P, MA44_R,
  (void*)menuPresetIncTemp_P, MA44_U,
  (void*)menuPresetDecTemp_P, MA44_D,  
 // 45 - P1(0): Duration
  (void*)menuDisplayText_P, MA45_L0,
  (void*)menuDisplayPresetTime_P, MA45_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA45_L,
  (void*)menuGoto_P, MA45_R,
  (void*)menuPresetIncTime_P, MA45_U,
  (void*)menuPresetDecTime_P, MA45_D,  
 // 46 - P1(0): Duty
  (void*)menuDisplayText_P, MA46_L0,
  (void*)menuDisplayPresetDuty_P, MA46_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA46_L,
  (void*)menuGoto_P, MA46_R,
  (void*)menuPresetIncDuty_P, MA46_U,
  (void*)menuPresetDecDuty_P, MA46_D,
// 47 - P1(0): Temp A
  (void*)menuDisplayText_P, MA47_L0,
  (void*)menuDisplayPresetTemp_P, MA47_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA47_L,
  (void*)menuGoto_P, MA47_R,
  (void*)menuPresetIncTemp_P, MA47_U,
  (void*)menuPresetDecTemp_P, MA47_D, 
// 48 - P1(0): Temp B
  (void*)menuDisplayText_P, MA48_L0,
  (void*)menuDisplayPresetTemp_P, MA48_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA48_L,
  (void*)menuGoto_P, MA48_R,
  (void*)menuPresetIncTemp_P, MA48_U,
  (void*)menuPresetDecTemp_P, MA48_D,  
 // 49 - P1(0): Duration
  (void*)menuDisplayText_P, MA49_L0,
  (void*)menuDisplayPresetTime_P, MA49_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA49_L,
  (void*)menuGoto_P, MA49_R,
  (void*)menuPresetIncTime_P, MA49_U,
  (void*)menuPresetDecTime_P, MA49_D,  
 // 50 - P1(0): Duty
  (void*)menuDisplayText_P, MA50_L0,
  (void*)menuDisplayPresetDuty_P, MA50_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA50_L,
  (void*)menuGoto_P, MA50_R,
  (void*)menuPresetIncDuty_P, MA50_U,
  (void*)menuPresetDecDuty_P, MA50_D,
  // 51 - P1(0): Temp A
  (void*)menuDisplayText_P, MA51_L0,
  (void*)menuDisplayPresetTemp_P, MA51_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA51_L,
  (void*)menuGoto_P, MA51_R,
  (void*)menuPresetIncTemp_P, MA51_U,
  (void*)menuPresetDecTemp_P, MA51_D, 
// 52 - P1(0): Temp B
  (void*)menuDisplayText_P, MA52_L0,
  (void*)menuDisplayPresetTemp_P, MA52_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA52_L,
  (void*)menuGoto_P, MA52_R,
  (void*)menuPresetIncTemp_P, MA52_U,
  (void*)menuPresetDecTemp_P, MA52_D,  
 // 53 - P1(0): Duration
  (void*)menuDisplayText_P, MA53_L0,
  (void*)menuDisplayPresetTime_P, MA53_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA53_L,
  (void*)menuGoto_P, MA53_R,
  (void*)menuPresetIncTime_P, MA53_U,
  (void*)menuPresetDecTime_P, MA53_D,  
 // 54 - P1(0): Duty
  (void*)menuDisplayText_P, MA54_L0,
  (void*)menuDisplayPresetDuty_P, MA54_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA54_L,
  (void*)menuGoto_P, MA54_R,
  (void*)menuPresetIncDuty_P, MA54_U,
  (void*)menuPresetDecDuty_P, MA54_D,
  // 55 - P1(0): Temp A
  (void*)menuDisplayText_P, MA55_L0,
  (void*)menuDisplayPresetTemp_P, MA55_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA55_L,
  (void*)menuGoto_P, MA55_R,
  (void*)menuPresetIncTemp_P, MA55_U,
  (void*)menuPresetDecTemp_P, MA55_D, 
// 56 - P1(0): Temp B
  (void*)menuDisplayText_P, MA56_L0,
  (void*)menuDisplayPresetTemp_P, MA56_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA56_L,
  (void*)menuGoto_P, MA56_R,
  (void*)menuPresetIncTemp_P, MA56_U,
  (void*)menuPresetDecTemp_P, MA56_D,  
 // 57 - P1(0): Duration
  (void*)menuDisplayText_P, MA57_L0,
  (void*)menuDisplayPresetTime_P, MA57_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA57_L,
  (void*)menuGoto_P, MA57_R,
  (void*)menuPresetIncTime_P, MA57_U,
  (void*)menuPresetDecTime_P, MA57_D,  
 // 58 - P1(0): Duty
  (void*)menuDisplayText_P, MA58_L0,
  (void*)menuDisplayPresetDuty_P, MA58_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA58_L,
  (void*)menuGoto_P, MA58_R,
  (void*)menuPresetIncDuty_P, MA58_U,
  (void*)menuPresetDecDuty_P, MA58_D,  
// 59 - Set Time (HH)
  (void*)menuDisplayText_P, MA59_L0,
  (void*)menuDisplayTime, MA59_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA59_L,
  (void*)menuGoto_P, MA59_R,
  (void*)menuTimeIncBCD_P, MA59_U,
  (void*)menuTimeDecBCD_P, MA59_D,
// 60 - Set Time (MM)
  (void*)menuDisplayText_P, MA60_L0,
  (void*)menuDisplayTime, MA60_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA60_L,
  (void*)menuGoto_P, MA60_R,
  (void*)menuTimeIncBCD_P, MA60_U,
  (void*)menuTimeDecBCD_P, MA60_D,
// 61 - Set Time (MM)
  (void*)menuDisplayText_P, MA61_L0,
  (void*)menuDisplayTime, MA61_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA61_L,
  (void*)menuGoto_P, MA61_R,
  (void*)menuTimeAP_P, MA61_U,
  (void*)menuTimeAP_P, MA61_D,
// 62 - Set Wakeup (HH)
  (void*)menuDisplayText_P, MA62_L0,
  (void*)menuDisplayA1, MA62_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA62_L,
  (void*)menuGoto_P, MA62_R,
  (void*)menuA1IncBCD_P, MA62_U,
  (void*)menuA1DecBCD_P, MA62_D,
// 63 - Set Wakeup (MM)
  (void*)menuDisplayText_P, MA63_L0,
  (void*)menuDisplayA1, MA63_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA63_L,
  (void*)menuGoto_P, MA63_R,
  (void*)menuA1IncBCD_P, MA63_U,
  (void*)menuA1DecBCD_P, MA63_D,
// 64 - Set Wakeup (AP)
  (void*)menuDisplayText_P, MA64_L0,
  (void*)menuDisplayA1, MA64_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA64_L,
  (void*)menuGoto_P, MA64_R,
  (void*)menuA1AP_P, MA64_U,
  (void*)menuA1AP_P, MA64_D,
// 65 - Set Wakeup (EN)
  (void*)menuDisplayText_P, MA65_L0,
  (void*)menuDisplayA1, MA65_L1,
  (void*)menuIdle, MA_null,
  (void*)menuGoto_P, MA65_L,
  (void*)menuGoto_P, MA65_R,
  (void*)menuA1AP_P, MA65_U,
  (void*)menuA1AP_P, MA65_D,
};

menuElement_t* curMenuElement = (menuElement_t*)&menu[0];

void menuDisplayText_P(void** args)
{
  sprintf((char*)pgm_read_word(&args[0]), "%-16s", (PGM_P)pgm_read_word(&args[1]));
}

void menuDisplayPresetTime_P(void** args)
{
  sprintf((char*)pgm_read_word(&args[0]), "%0.1fs", *(float*)pgm_read_word(&args[1]));
}

void menuDisplayPresetTemp_P(void** args)
{
  sprintf((char*)pgm_read_word(&args[0]), "%0.1f%cC", *(float*)pgm_read_word(&args[1]), 0xDF);
}

void menuDisplayPresetDuty_P(void** args)
{
  sprintf((char*)pgm_read_word(&args[0]), "%u%%", *(uint8_t*)pgm_read_word(&args[1])*10);
}

void menuDisplayTime(void** args)
{
  sprintf((char*)pgm_read_word(&args[0]), "%02hhX:%02hhX%c", rtc.h, rtc.m, rtc.p ? 'p' : 'a');
}

void menuDisplayA1(void** args)
{
  sprintf((char*)pgm_read_word(&args[0]), "%02hhX:%02hhX%c %s", rtc.a1h, rtc.a1m, rtc.a1p ? 'p' : 'a', rtc.a1en ? "On" : "Off");
}

void menuTimeIncBCD_P(void** args)
{
  if (((*(uint8_t*)pgm_read_word(&args[0]) & 0xF) + 1) > 9) {*(uint8_t*)pgm_read_word(&args[0]) = ((*(uint8_t*)pgm_read_word(&args[0]) >> 4) + 1) << 4;}
  else {*(uint8_t*)pgm_read_word(&args[0]) += 1;}
  //if (*(uint8_t*)pgm_read_word(&args[0]) > *(uint8_t*)pgm_read_word(&args[1])) *(uint8_t*)pgm_read_word(&args[0]) = 0x01;
  rtc.setTime();
}

void menuTimeDecBCD_P(void** args)
{
  if (((*(uint8_t*)pgm_read_word(&args[0]) & 0xF) - 1) < 0) {*(uint8_t*)pgm_read_word(&args[0]) = (((*(uint8_t*)pgm_read_word(&args[0]) >> 4) - 1) << 4) + 9;}
  else {*(uint8_t*)pgm_read_word(&args[0]) -= 1;}
  //if (*(uint8_t*)pgm_read_word(&args[0]) < 0x01) *(uint8_t*)pgm_read_word(&args[0]) = *(uint8_t*)pgm_read_word(&args[1]);
  rtc.setTime();
}

void menuTimeAP_P(void** args)
{
  *(boolean*)pgm_read_word(&args[0]) = *(boolean*)pgm_read_word(&args[0]) ? false : true;
  rtc.setTime();
}

void menuA1IncBCD_P(void** args)
{
  if (((*(uint8_t*)pgm_read_word(&args[0]) & 0xF) + 1) > 9) {*(uint8_t*)pgm_read_word(&args[0]) = ((*(uint8_t*)pgm_read_word(&args[0]) >> 4) + 1) << 4;}
  else {*(uint8_t*)pgm_read_word(&args[0]) += 1;}
  //if (*(uint8_t*)pgm_read_word(&args[0]) > *(uint8_t*)pgm_read_word(&args[1])) *(uint8_t*)pgm_read_word(&args[0]) = 0;
  rtc.setA1();
}

void menuA1DecBCD_P(void** args)
{
  if (((*(uint8_t*)pgm_read_word(&args[0]) & 0xF) - 1) < 0) {*(uint8_t*)pgm_read_word(&args[0]) = (((*(uint8_t*)pgm_read_word(&args[0]) >> 4) - 1) << 4) + 9;}
  else {*(uint8_t*)pgm_read_word(&args[0]) -= 1;}
  //if (*(uint8_t*)pgm_read_word(&args[0]) < 0) *(uint8_t*)pgm_read_word(&args[0]) = *(uint8_t*)pgm_read_word(&args[1]);
  rtc.setA1();
}

void menuA1AP_P(void** args)
{
  *(boolean*)pgm_read_word(&args[0]) = *(boolean*)pgm_read_word(&args[0]) ? false : true;
  rtc.setA1();
}

void menuPresetIncTemp_P(void** args)
{
  if(*(float*)pgm_read_word(&args[0]) + 0.1 <= MAX_TEMP) *(float*)pgm_read_word(&args[0]) += 0.1;
}

void menuPresetDecTemp_P(void** args)
{
  if(*(float*)pgm_read_word(&args[0]) - 0.1 >= 0) *(float*)pgm_read_word(&args[0]) -= 0.1;
}

void menuPresetIncTime_P(void** args)
{
  if(*(float*)pgm_read_word(&args[0]) < 0) {*(float*)pgm_read_word(&args[0]) = 0;}
  else if(*(float*)pgm_read_word(&args[0]) + 0.1 <= MAX_TIME) {*(float*)pgm_read_word(&args[0]) += 0.1;}
}

void menuPresetDecTime_P(void** args)
{
  if(*(float*)pgm_read_word(&args[0]) - 0.1 >= 0) {*(float*)pgm_read_word(&args[0]) -= 0.1;}
  else {*(float*)pgm_read_word(&args[0]) = -1;}
}

void menuPresetIncDuty_P(void** args)
{
  if(*(uint8_t*)pgm_read_word(&args[0]) + 1 <= 10) *(uint8_t*)pgm_read_word(&args[0]) += 1;
}

void menuPresetDecDuty_P(void** args)
{
  if(*(uint8_t*)pgm_read_word(&args[0]) - 1 >= 0) *(uint8_t*)pgm_read_word(&args[0]) -= 1;
}

void menuGoto_P(void** args)
{
  curMenuElement = (menuElement_t*)&menu[(int)pgm_read_word(&args[0])];
}

void menuMode_P(void** args)
{
  mode = (uint8_t)pgm_read_word(&args[0]);
}

void menuIdle(void** args)
{
  return;
}
#endif MENU

//Port B Interrupt Vector: Switch Changes
ISR(PCINT0_vect)
{
  INTSW = true;
  PB = PINB & ((1<<PINB7) | (1<<PINB6) | (1<<PINB4));
  if ( PB == 0xD0 ) //No Switches ON, Idle Mode
  {
    mode = 2;
  }
    
  if ( PB == 0xC0 ) //Only Brew Switch ON, Brew Mode
  {
    mode = 3;
  }
  
  if ( PB == 0x90 ) //Only HW Switch is ON, Hot Water
  {
    mode = 4;
  }
  
  if ( PB == 0x50 ) //Only Steam Switch is ON, Steam w/o Pump
  {
    mode = 5;
  }
  
  if ( PB == 0x10 ) //Steam Switch and HW Switch is ON, Steam w/ Pump
  {
    mode = 6;
  }
}

//INT.6 Interrupt Vector: Alarm Toggle Low
ISR(INT6_vect)
{
  INTAL = true;
}

void setup()
{ 
  //Initialize Boiler A PWM
  DDRB |= (1<<DDB5); //Set Pin 9 As Output
  ICR1 = 0x7FFF; //Defines the number of ticks (15 625Hz) per half PWM period (due to PWM symmetry).
  OCR1A = 0x0; //Match Value - Change this from 0x0000 to ICR1 value to adjust PWM.
  TCCR1A = (1<<COM1A1); //Clear OC1A (Pin 9) On Match Up and Set on Match Down.
  TCCR1B = (1<<WGM13) | (1<<CS12) | (1<<CS10); //Start Timer With 1024 Prescaler. 
  
  //Initialize Boiler B PWM
  DDRC |= (1<<PORTC6); //Set Pin 5 As Output
  ICR3 = 0x7FFF; //Defines the number of ticks (15 625Hz) per half PWM period (due to PWM symmetry).
  OCR3A = 0x0; //Match Value - Change this from 0x0000 to ICR1 value to adjust PWM.
  TCCR3A = (1<<COM3A1); //Clear OC3A (Pin 5) On Match Up and Set on Match Down.
  TCCR3B = (1<<WGM33) | (1<<CS32) | (1<<CS30); //Start Timer With 1024 Prescaler.
  
  //Initialize Pump PWM
  DDRD |= (1<<PORTD7); //Sets Pin 6 As Output
  TCCR4C |= (1<<COM4D1); //Clear on up count
  TCCR4D = (1<<WGM40); //Phase & Frequency correct PWM
  
  //A2D Setup
  DDRB |= (1<<DDB1) | (1<<DDB2); //Set SCK, MOSI as Output
  DDRD |= (1<<DDD4); //Set SS (PD4) as Output
  PORTD |= 1<<PORTD4; //Pull SS High
  SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1<<CPHA); //Set SPI and Enable
  PORTD &= ~(1<<PORTD4); //Pull SS Low to Enable Communications with ADS1247
  
  SPI.transfer(0x06); //Reset
  delay(210); // Required reset delay before rewriting registers
  SPI.transfer(0x4B); //Set IDAC1 Register (0Bh) Write 01h - Output reference current on ANIN0,1
  SPI.transfer(0x00);
  SPI.transfer(0x01);
  SPI.transfer(0x4A); //Set IDAC0 Register (0Ah) Write 07h - Select 1.5mA reference current for RTD
  SPI.transfer(0x00);
  SPI.transfer(0x07);
  SPI.transfer(0x43); //Set SYS0 Register (03h) Write 52h - PGA:32, Sample at 20sps
  SPI.transfer(0x00);
  SPI.transfer(0x52);
  SPI.transfer(0x40); //Set MUX0 Register (00h) Write 08h - Change MUX0 according to what you want to measure
  SPI.transfer(0x00);
  SPI.transfer(0x08);
  SPI.transfer(0x42); //Set MUX1 Register (02h) Write 20h - Select internal reference always on. Use 33h if wanting an on chip temp read.
  SPI.transfer(0x00);
  SPI.transfer(0x20);
  SPI.transfer(0x16); //SDATAC - Do not automatically load most recent conversion into output buffer - manually load most recent using RDATA command
  SPI.transfer(0xFF); //NOP prevents CS from going low too soon
  PORTD |= 1<<PORTD4; //CS High
  
  //Enable I2C Bus
  Wire.begin();
  
  //Initialize RTC
  rtc.setInt(0x00);
  
  rtc.address = 0xD0;
  rtc.s = 0x00;
  rtc.m = 0x00;
  rtc.h = 0x12;
  rtc.p = false;
  rtc.mode = true; //12Hr
  rtc.setTime();
  
  //Set up alarm
  rtc.a1s = 0x00;
  rtc.a1m = 0x00;
  rtc.a1h = 0x12;
  rtc.a1p = false;
  rtc.a1en = false;
  rtc.setA1();
  rtc.setInt(0x00);
  
  //Initialize the LCD
  LCD.init();
  LCD.setBrightness(2);
  LCD.setCharacter(1, custChar0);
  LCD.setCharacter(2, custChar1);
  LCD.setCharacter(3, custChar2);
  LCD.setCharacter(4, custChar3);
  
  /*
  eeprom_write_block((void*)&boilerA.K_P, (void*)0, 4);
  eeprom_write_block((void*)&boilerA.K_I, (void*)4, 4);
  eeprom_write_block((void*)&boilerA.K_D, (void*)8, 4);
  eeprom_write_block((void*)&boilerA.K_P, (void*)12, 4);
  eeprom_write_block((void*)&boilerA.K_I, (void*)16, 4);
  eeprom_write_block((void*)&boilerA.K_D, (void*)20, 4);
  eeprom_write_block((void*)&activePresetIndex, (void*)24, 1);
  eeprom_write_block((void*)&presetIndex[0].name, (void*)25, 16);
  eeprom_write_block((void*)&presetIndex[1].name, (void*)42, 16);
  eeprom_write_block((void*)&presetIndex[2].name, (void*)58, 16);
  eeprom_write_block((void*)&presetIndex[3].name, (void*)74, 16);
  eeprom_write_block((void*)&presetIndex[4].name, (void*)90, 16);
  eeprom_write_block((void*)&presetIndex[5].name, (void*)106, 16);
  eeprom_write_block((void*)&presetIndex[6].name, (void*)122, 16);
  eeprom_write_block((void*)&presetIndex[7].name, (void*)138, 16);
  eeprom_write_block((void*)&presetBR[0], (void*)154, 13);
  eeprom_write_block((void*)&presetHW[0], (void*)167, 13);
  eeprom_write_block((void*)&presetST[0], (void*)180, 13);
  eeprom_write_block((void*)&preset0[0], (void*)193, 39);
  eeprom_write_block((void*)&preset1[0], (void*)232, 39);
  eeprom_write_block((void*)&preset2[1], (void*)271, 39);
  eeprom_write_block((void*)&preset3[2], (void*)310, 39);  
  eeprom_write_block((void*)&preset4[0], (void*)349, 39);
  
  //Read in settings from EEPROM
  eeprom_read_block((void*)&boilerA.K_P, (void*)0, 4);
  eeprom_read_block((void*)&boilerA.K_I, (void*)4, 4);
  eeprom_read_block((void*)&boilerA.K_D, (void*)8, 4);
  eeprom_read_block((void*)&boilerA.K_P, (void*)12, 4);
  eeprom_read_block((void*)&boilerA.K_I, (void*)16, 4);
  eeprom_read_block((void*)&boilerA.K_D, (void*)20, 4);
  eeprom_read_block((void*)&activePresetIndex, (void*)24, 1);
  eeprom_read_block((void*)&presetIndex[0].name, (void*)25, 16);
  eeprom_read_block((void*)&presetIndex[1].name, (void*)42, 16);
  eeprom_read_block((void*)&presetIndex[2].name, (void*)58, 16);
  eeprom_read_block((void*)&presetIndex[3].name, (void*)74, 16);
  eeprom_read_block((void*)&presetIndex[4].name, (void*)90, 16);
  eeprom_read_block((void*)&presetIndex[5].name, (void*)106, 16);
  eeprom_read_block((void*)&presetIndex[6].name, (void*)122, 16);
  eeprom_read_block((void*)&presetIndex[7].name, (void*)138, 16);
  eeprom_read_block((void*)&presetBR[0], (void*)154, 13);
  eeprom_read_block((void*)&presetHW[0], (void*)167, 13);
  eeprom_read_block((void*)&presetST[0], (void*)180, 13);
  eeprom_read_block((void*)&preset0[0], (void*)193, 39);
  eeprom_read_block((void*)&preset1[0], (void*)232, 39);
  eeprom_read_block((void*)&preset2[1], (void*)271, 39);
  eeprom_read_block((void*)&preset3[2], (void*)310, 39);  
  eeprom_read_block((void*)&preset4[0], (void*)349, 39);
  */
  //Initialize PID(s)
  boilerA.K_P = 4000;
  boilerA.K_I = 4;
  boilerA.K_D = 700;
  
  boilerB.K_P = 4000;
  boilerB.K_I = 2;
  boilerB.K_D = 680;
  
  pid_Init(boilerA.K_P * PID_SCALING_FACTOR, boilerA.K_I * PID_SCALING_FACTOR, boilerA.K_D * PID_SCALING_FACTOR, &boilerA.pid);
  pid_Init(boilerB.K_P * PID_SCALING_FACTOR, boilerB.K_I * PID_SCALING_FACTOR, boilerB.K_D * PID_SCALING_FACTOR, &boilerB.pid);
  
  //Initialize Switch Interrupts
  DDRB &= ~((1<<DDB7) | (1<<DDB6) | (1<<DDB4));
  PORTB |= ((1<<PORTB7) | (1<<PORTB6) | (1<<PORTB4));
  PCMSK0 |= (1<<PCINT7) | (1<<PCINT6) | (1<<PCINT4);
  PCICR = (1<<PCIE0);
  
  //Initialize Alarm Interrupt
  DDRE &= ~(1<<DDE6);
  PORTE |= (1<<PORTE6);
  EICRB = (1<<ISC61); //Falling Edge Only for INT6
  EIMSK = (1<<INT6);
}
  
void loop()
{
  timeLoop = millis();
  
  if (INTAL) //Alarm Interrupt Goto Mode 2 and Update Sleep Timeout
  {
    INTAL = false;
    if (rtc.getInt() & 0x01)
    {
      mode = 2;
      timeSleep = millis();
    }
    rtc.setInt(0x00); //Clear all flags
  }

  //Test if a Switch Interrupt has Occurred and Update Sleep Timeout 
  if (INTSW)
  {
    INTSW = false;
    timeSleep = millis();
  }
  
  //Test for Keypad Button Presses and Update Sleep Timeout
  LCD.getKP();
  if (LCD.KP[1]) {timeSleep = millis();}
  
  //Manage Sleep Timeout
  if ((millis() - timeSleep) > 5400000) {mode = 0;}//Two-hour timeout 7200000, 1.5 5400000
  
  //Get the time from the RTC
  rtc.getTime();
  
  //Default Display - Can be superceded!
  sprintf((char*)&lcdBuffer[0], "%.1f%cC %c %02hhX:%02hhX%c", activeBoilerA->current_temp, 0xDF, (PINB & 1<<PINB5) ? 0x01 : 0x20, rtc.h, rtc.m, rtc.p ? 'p' : 'a');
  sprintf((char*)&lcdBuffer[1], "");
  
  switch (mode)
  {
    case 0: //Deep Sleep
      activePreset = (preset_t*)&presetSleep[0]; //Point to Sleep Preset
      sprintf((char*)lcdBuffer[1], "Sleep"); //Render Display
      if (LCD.KP[1]) {mode = 2;}
    break;
    
    case 1: //Stand By
    
    break;
    
    case 2: //Idle Mode
      if (LCD.KP[1] & KP_CANCEL) {mode = 0;}
      if (LCD.KP[1] & (KP_OK | KP_LEFT | KP_RIGHT)) {mode = 7;} //Check for User OK
      if (LCD.KP[1] & KP_DOWN) {if (activePresetIndex+1 < sizeof(presetIndex)/sizeof(presetIndex_t)) {activePresetIndex++;}} //Check for User DOWN
      if (LCD.KP[1] & KP_UP) {if (activePresetIndex-1 >= 0) {activePresetIndex--;}} //Check for User UP
     
      activePresetStep = 0;
      activePreset = presetIndex[activePresetIndex].preset; //Point back to beginning of Preset
      
      sprintf((char*)lcdBuffer[1], "%-15s", presetIndex[activePresetIndex].name); //Render Display
    break;
    
    case 3: //Brew Mode
      if (activePresetStep == 0) //At the beginning of the preset?
      {
        activePreset = presetIndex[activePresetIndex].preset; //Point back to beginning of brew preset
        activePresetStep = 1;
        time = millis();
      }
      else
      { 
        if (activePreset->duration != -1) //Timed Preset?
        {
          if(((float)(millis()-time))/1000 > activePreset->duration) //Is the time up?
          {
            if (presetIndex[activePresetIndex].length > activePresetStep) //Are there more steps in the Preset?
            {
              activePreset++;
              activePresetStep++;
              time = millis();
            }
            else //No more steps in the current Preset, go to Idle
            {
              mode = 2;
            }
          }
          else
          {
            if (LCD.KP[1] & KP_OK) //Check for User OK, Force next step
            {
              if (presetIndex[activePresetIndex].length > activePresetStep) //Are there more steps in the Preset?
              {
                activePreset++;
                activePresetStep++;
                time = millis();
              }
              else //No more steps in the current Preset, go to Idle
              {
                mode = 2;
              }
            }
            sprintf((char*)lcdBuffer[1], "Step %hhu (%3.1fs)", activePresetStep, ((float)(millis() - time))/1000); //Render Display
          }
        }
        else
        {
          if (LCD.KP[1] & KP_OK) //Check for User OK
          {
            if (presetIndex[activePresetIndex].length > activePresetStep) //Are there more steps in the Preset?
            {
              activePreset++;
              activePresetStep++;
              time = millis();
            }
            else //No more steps in the current Preset, go to Idle
            {
              mode = 2;
            }
          }
          sprintf((char*)lcdBuffer[1], "Step %hhu (%3.1fs)", activePresetStep, ((float)(millis() - time))/1000); //Render Display
        }
      }
    break;
    
    case 4: //Hot Water Mode
      activePreset = (preset_t*)&presetHW[0]; //Point to Manual Brew Preset
      sprintf((char*)lcdBuffer[1], "Hot Water"); //Render Display
    break;
    
    case 5: //Steam Mode
      activePreset = (preset_t*)&presetST[0]; //Point to Steam Preset
      activePreset->duty = 0; //Turn off Pump
      sprintf((char*)lcdBuffer[1], "Steam"); //Render Display
    break;

    case 6: //Steam Mode w/ Marching Soldiers
      activePreset = (preset_t*)&presetST[0]; //Point to Steam Preset
      activePreset->duty = 11; //Turn on Marching Soldiers
      sprintf((char*)lcdBuffer[1], "Steam"); //Render Display
    break;
    
    case 7: //Menu Mode
      #ifdef MENU
      //Callback Display Finctions w/ Args
      ((void(*)(...))((PGM_P)pgm_read_word(&curMenuElement->L0_func)))((PGM_P)pgm_read_word(&curMenuElement->L0_args));
      ((void(*)(...))((PGM_P)pgm_read_word(&curMenuElement->L1_func)))((PGM_P)pgm_read_word(&curMenuElement->L1_args));
      //Test for user input and Callback Button Functions w/ Args
      if (LCD.KP[1] & KP_CANCEL) {curMenuElement = (menuElement_t*)&menu[0]; mode = 2;}
      if (LCD.KP[1] & KP_OK) {((void(*)(...))((PGM_P)pgm_read_word(&curMenuElement->K_func)))((PGM_P)pgm_read_word(&curMenuElement->K_args));}
      if (LCD.KP[1] & KP_LEFT) {((void(*)(...))((PGM_P)pgm_read_word(&curMenuElement->L_func)))((PGM_P)pgm_read_word(&curMenuElement->L_args));}
      if (LCD.KP[1] & KP_RIGHT) {((void(*)(...))((PGM_P)pgm_read_word(&curMenuElement->R_func)))((PGM_P)pgm_read_word(&curMenuElement->R_args));}
      if (LCD.KP[1] & KP_UP) {((void(*)(...))((PGM_P)pgm_read_word(&curMenuElement->U_func)))((PGM_P)pgm_read_word(&curMenuElement->U_args));}
      if (LCD.KP[1] & KP_DOWN) {((void(*)(...))((PGM_P)pgm_read_word(&curMenuElement->D_func)))((PGM_P)pgm_read_word(&curMenuElement->D_args));}
      #endif
    break;
  }
  
  if (mode == 3 | mode == 4 | mode == 5 | mode == 6) {activePump->duty(activePreset->duty);} else {activePump->duty(0);} //Turn Pump off when not in use
  
  #ifdef BOILERA
  activeBoilerA->target_temp = activePreset->tempA;
  activeBoilerA->current_temp = LtoT(TCtoL(getADC()));
  
  if (activeBoilerA->current_temp < (activeBoilerA->target_temp - PID_RANGE))
  {
    pid_Reset_Integrator(&activeBoilerA->pid);
    activeBoilerA->pwm = ICR1;
  }
  else if (activeBoilerA->current_temp > (activeBoilerA->target_temp + PID_RANGE))
  {
    pid_Reset_Integrator(&activeBoilerA->pid);
    activeBoilerA->pwm = 0;
  }
  else
  {
    activeBoilerA->pwm = pid_Controller(activeBoilerA->target_temp, activeBoilerA->current_temp, &activeBoilerA->pid);
  }
  if (activeBoilerA->pwm >= ICR1) activeBoilerA->pwm = ICR1;
  if (activeBoilerA->pwm <= 0) activeBoilerA->pwm = 0;
  OCR1A = activeBoilerA->pwm; //OCRnx are double buffered - you don't need to stop the timer to update the MATCH value!
  #endif
  
  #ifdef BOILERB
  activeBoilerB->target_temp = activePreset->tempB;
  activeBoilerB->current_temp = LtoT(TCtoL(getADC()));
  if (activeBoilerB->current_temp < (activeBoilerB->target_temp - PID_RANGE))
  {
    pid_Reset_Integrator(&activeBoilerB->pid);
    activeBoilerB->pwm = ICR1;
  }
  else if (activeBoilerB->current_temp > (activeBoilerB->target_temp + PID_RANGE))
  {
    pid_Reset_Integrator(&activeBoilerB->pid);
    activeBoilerB->pwm = 0;
  }
  else
  {
    activeBoilerB->pwm = pid_Controller(activeBoilerB->target_temp, activeBoilerB->current_temp, &activeBoilerB->pid);
  }
  if (activeBoilerB->pwm >= ICR1) activeBoilerB->pwm = ICR1;
  if (activeBoilerB->pwm <= 0) activeBoilerB->pwm = 0;
  OCR3A = activeBoilerB->pwm;
  #endif
  
  //Update Display
  //Turn off Backlight if in Sleep Mode
  if (mode) {if (LCD.brightness == 0) LCD.setBrightness(2);} else {if (LCD.brightness != 0) LCD.setBrightness(0);}
  
  //Line 0
  sprintf((char*)&lcdBuffer[2], "%-16s", lcdBuffer[0]); //Left Justify and Pad with Spaces for LCD
  LCD.print(lcdBuffer[2], 16, 0, 0);
  
  //Line 1
  sprintf((char*)&lcdBuffer[3], "%-16s", lcdBuffer[1]); //Left Justify and Pad with Spaces for LCD
  LCD.print(lcdBuffer[3], 16, 0, 1);
  
  delay(100 + timeLoop - millis()); //Iterate every 100ms to ensure PID retains accuracy
}

unsigned long getADC() //Will Return 24 Bit Two's Compliment
{
  unsigned long val = 0;
  PORTD &= ~(1<<PORTD4); //Pull SS Low to Enable Communications with ADS1247
  SPI.transfer(0x12); //Issue RDATA
  val |= SPI.transfer(0xFF);
  val <<= 8;
  val |= SPI.transfer(0xFF);
  val <<= 8;
  val |= SPI.transfer(0xFF);
  PORTD |= 1<<PORTD4; //Pull SS High to Disable Communications with ADS1247
  return val;
}

long TCtoL (unsigned long UL) //24 Bit Two's Compliment to 32 Bit Signed Long
{
  if (UL & 0x800000) {UL |= ~0xFFFFFF; UL+=1;}
  return UL;
}

float LtoT(long val) //Inverse of the Callendar Van Dusen Equation: Convert Resistance to Temperature
{
  return -1 * ( CVDA / ( 2 * CVDB ) - sqrt( 25 * CVDA * CVDA + CVDB * ( val * ADLSB + ADRO ) - 100 * CVDB ) / ( 10 * CVDB ) );
}
