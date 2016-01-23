#ifndef FS_T6963C_LIB
#define FS_T6963C_LIB

// System libraries
#include <stdint.h>

// Data types
typedef struct LCD {
  uint16_t width;
  uint16_t height;
  uint8_t fontSize;
  uint8_t brightness;
  uint8_t * controlPins; // WR, RD, CE, CD, RST, FS
  uint8_t * dataPins; // DB0...DB7
  uint8_t backlightPin;
} LCD;

// Pre-proc.
#define STATUS_READY 0x3 // bin 1100 0000
#define NUM_DATA_PINS 8
#define NUM_CNTRL_PINS 6
enum BRIGHTNESS_STATES {
  BCK_OFF = 0,
  BCK_LOW = 25,
  BCK_MEDIUM = 100,
  BCK_HIGH = 200,
  BCK_FULL = 255
};
enum CNTRL_PINS {
  WR,
  RD,
  CE,
  CD,
  RST,
  FS
};
enum DATA_PINS {
  DB0,
  DB1,
  DB2,
  DB3,
  DB4,
  DB5,
  DB6,
  DB7,
  DB8
};
enum MODES {
  READ,
  WRITE,
  COMMAND = 2,
  STATUS = 2, // same value but diff name just to distinguish actual function
  DATA
};


// Public function declarations
/*
 * @param pinout: An array of T6963C pins as indices (1-22) and Teensy pins as values
 */
uint8_t LcdInit(uint16_t lcdWidth, uint16_t lcdHeight, uint8_t fontSize, uint8_t brightness, uint8_t * controlPins, uint8_t * dataPins, uint8_t backlightPin);
void LCDWriteChar(char c);
uint8_t LCDGetStatusByte();
int LCDSetBrightness(uint8_t value, int delayTime);
uint8_t LCDWaitUntilReady();


#endif
