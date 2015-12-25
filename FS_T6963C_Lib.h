#ifndef FS_T6963C_LIB
#define FS_T6963C_LIB

// System libraries
#include <stdint.h>

// Data types
typedef uint8_t Byte;
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
#define STATUS_READY 0x3
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
  COMMAND,
  DATA
};


// Public function declarations
/*
 * @param pinout: An array of T6963C pins as indices (1-22) and Teensy pins as values
 */
uint8_t LcdInit(uint16_t lcdWidth, uint16_t lcdHeight, uint8_t fontSize, uint8_t brightness, uint8_t * controlPins, uint8_t * dataPins, uint8_t backlightPin);
void WriteChar(char c);
Byte GetStatusByte(uint8_t currentRW, uint8_t currentCD);


#endif
