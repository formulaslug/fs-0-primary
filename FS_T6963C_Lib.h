#ifndef FS_T6963C_LIB
#define FS_T6963C_LIB

// System libraries
#include <stdint.h>

// Data types
typedef unsigned char byte;
typedef struct LCD {
  uint16_t width;
  uint16_t height;
  uint8_t fontSize;
  uint8_t brightness;
  uint8_t * controlPins; // WR, RD, CE, CD, RST, FS
  uint8_t * dataPins; // DB0...DB7
  uint8_t backlightPin;
} LCD;

// Function declarations
/*
 * @param pinout: An array of T6963C pins as indices (1-22) and Teensy pins as values
 */
uint8_t LcdInit(uint16_t lcdWidth, uint16_t lcdHeight, uint8_t fontSize, uint8_t brightness, uint8_t * controlPins, uint8_t * dataPins, uint8_t backlightPin);

void WriteChar(char c);


#endif
