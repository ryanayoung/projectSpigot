// Masks for Cirque Register Access Protocol (RAP)
#define WRITE_MASK  0x80
#define READ_MASK   0xA0


//Comment out if you want to use relative mode
//  I'll eventually want to change this to a serial menu
//#define ABSMODE


#define SPISPEEDMAX 20000000 //for esp32
//#define SPISPEEDMAX 10000000 //for teensy 3.2

// Register config values for this demo
#define SYSCONFIG_1   0x00
//#define FEEDCONFIG_1  0x03 //absolute
#ifdef ABSMODE
  #define FEEDCONFIG_1  0x83 //absolute, y-inverted
#else
  #define FEEDCONFIG_1  0x41 //relative, x-inverted
#endif

#define FEEDCONFIG_2  0x11 //relative mode configts, intellimouse and all taps enabled.

#define Z_IDLE_COUNT  0x05

// Coordinate scaling values
#define PINNACLE_XMAX     2047    // max value Pinnacle can report for X
#define PINNACLE_YMAX     1535    // max value Pinnacle can report for Y
#define PINNACLE_X_LOWER  127     // min "reachable" X value
#define PINNACLE_X_UPPER  1919    // max "reachable" X value
#define PINNACLE_Y_LOWER  63      // min "reachable" Y value
#define PINNACLE_Y_UPPER  1471    // max "reachable" Y value
#define PINNACLE_X_RANGE  (PINNACLE_X_UPPER-PINNACLE_X_LOWER)
#define PINNACLE_Y_RANGE  (PINNACLE_Y_UPPER-PINNACLE_Y_LOWER)
#define ZONESCALE 256   // divisor for reducing x,y values to an array index for the LUT
#define ROWS_Y ((PINNACLE_YMAX + 1) / ZONESCALE)
#define COLS_X ((PINNACLE_XMAX + 1) / ZONESCALE)

// ADC-attenuation settings (held in BIT_7 and BIT_6)
// 1X = most sensitive, 4X = least sensitive
#define ADC_ATTENUATE_1X   0x00
#define ADC_ATTENUATE_2X   0x40
#define ADC_ATTENUATE_3X   0x80
#define ADC_ATTENUATE_4X   0xC0

// Convenient way to store and access measurements
typedef struct _absData
{
  uint16_t xValue;
  uint16_t yValue;
  uint16_t zValue;
  uint8_t buttonFlags;
  bool touchDown;
  bool hovering;
} absData_t;

typedef struct _relData
{
  uint8_t buttons;
  int8_t xDelta;
  int8_t yDelta;
  int8_t wheelCount;
} relData_t;
