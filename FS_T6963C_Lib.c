// System libraries
#include <stdint.h>
#include "Arduino.h"

// User libraries
#include "FS_T6963C_Lib.h"

LCD lcd = {};

// Private function declarations
void setMode(uint8_t RW, uint8_t CD);

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
  for (i = 0; i < 7; i++) {
    pinMode(*(lcd.dataPins + i), HIGH);
  }
  return 1;
}

Byte GetStatusByte(uint8_t currentRW, uint8_t currentCD)
{
  int i;
  // set mode
  setMode(READ, DATA);

  // read state of 8-bit data bus
  Byte byte = 0;
  for (i = 0; i < 7; i++) {
    // if high, set the corresponding bit
    if (digitalRead(*(lcd.dataPins + i)) == HIGH) {
      byte |= 1 << i;
    }
  }

  // return to previous
  setMode(currentRW, currentCD);

  return byte;
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
void setMode(uint8_t RW, uint8_t CD)
{
  int i;
  // set Read/Write state
  switch(RW) {
    case READ:
      // set WR high
      digitalWrite(*(lcd.controlPins + RD), HIGH);
      // set RD low
      digitalWrite(*(lcd.controlPins + WR), LOW);
      // set data bus as inputs
      for (i = 0; i < 7; i++) {
        pinMode(*(lcd.dataPins + i), INPUT);
      }
      break;
    case WRITE:
      // set WR low
      digitalWrite(*(lcd.controlPins + RD), LOW);
      // set RD high
      digitalWrite(*(lcd.controlPins + WR), HIGH);
      // set data bus as inputs
      for (i = 0; i < 7; i++) {
        pinMode(*(lcd.dataPins + i), HIGH);
      }
      break;
  }
  // set Command/Data state
  switch(CD) {
    case COMMAND:
      // low = command
      digitalWrite(*(lcd.controlPins + CD), LOW);
      break;
    case DATA:
      // high = data
      digitalWrite(*(lcd.controlPins + CD), HIGH);
      break;
  }
}
