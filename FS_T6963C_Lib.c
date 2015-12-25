// System libraries
#include <stdint.h>
#include "Arduino.h"

// User libraries
#include "FS_T6963C_Lib.h"

LCD lcd = {};

// function defintions
uint8_t LcdInit(uint16_t lcdWidth, uint16_t lcdHeight, uint8_t fontSize, uint8_t brightness, uint8_t * controlPins, uint8_t * dataPins, uint8_t backlightPin)
{
  // set inc.s
  int i;
  // set initial lcd vals
  lcd.width = lcdWidth;
  lcd.height = lcdHeight;
  lcd.fontSize = fontSize;
  lcd.brightness = brightness;
  lcd.controlPins = controlPins;
  lcd.dataPins = dataPins;
  lcd.backlightPin = backlightPin;

  /*CONTROLS*/
  // set digital outputs
  pinMode(*(lcd.controlPins), HIGH); // WR
  pinMode(*(lcd.controlPins + 1), HIGH); // RD
  pinMode(*(lcd.controlPins + 2), HIGH); // CE
  pinMode(*(lcd.controlPins + 3), HIGH); // CD
  pinMode(*(lcd.controlPins + 4), HIGH); // RST
  pinMode(*(lcd.controlPins + 5), HIGH); // FS
  // init digital outputs
  // trigger reset
  pinMode(*(lcd.controlPins + 4), LOW); // RST..start
  for (i = 0; i < 6; i++) {
    asm("nop");
  }
  pinMode(*(lcd.controlPins + 4), HIGH); // RST..end
  pinMode(*(lcd.controlPins + 2), LOW); // CE
  pinMode(*(lcd.controlPins + 3), HIGH); // CD
  pinMode(*(lcd.controlPins), LOW); // WR
  pinMode(*(lcd.controlPins + 1), HIGH); // RD
  pinMode(*(lcd.controlPins + 5), (fontSize == 6 ? HIGH : LOW)); // FS
  

  // set analog outputs (PWM)
  pinMode(lcd.backlightPin, OUTPUT); // LED_A
  // init analog outputs
  analogWrite(lcd.backlightPin, lcd.brightness);

  /*DATA*/
  int i;
  for (i = 0; i < 7; i++) {
    pinMode(*(lcd.dataPins + i), HIGH);
  }
  return 1;
}

byte GetStatusByte()
{
  pinMode(*(lcd.controlPins), INPUT);
}

void WriteChar(char c)
{
  int i;
  char bit;
  for (i = 0; i < 7; i++) {
    bit = c << (7 - i);
    bit = c >> 7;
    digitalWrite(*(lcd.dataPins + i), bit);
  }
}

/*PRIVATE FUNCTIONS*/
