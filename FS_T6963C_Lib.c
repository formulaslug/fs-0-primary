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
  for (i = 0; i < NUM_CNTRL_PINS; i++) {
    pinMode(*(lcd.controlPins + i), HIGH); // WR
  }
  // init digital outputs
  // trigger reset
  pinMode(*(lcd.controlPins + RST), LOW); // RST..start
  for (i = 0; i < 6; i++) {
    asm("nop");
  }
  pinMode(*(lcd.controlPins + RST), HIGH); // RST..end
  /* pinMode(*(lcd.controlPins + CE), LOW); // CE */
  /* pinMode(*(lcd.controlPins + CD), HIGH); // CD */
  /* pinMode(*(lcd.controlPins + WR), LOW); // WR */
  /* pinMode(*(lcd.controlPins + RD), HIGH); // RD */
  /* pinMode(*(lcd.controlPins + FS), (fontSize == 6 ? HIGH : LOW)); // FS */
  

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

Byte LCDGetStatusByte(uint8_t currentRW, uint8_t currentCD)
{
  int i;
  // set mode
  setMode(READ, STATUS);

  // wait 150 ns...100MGHz is about 10ns per cycle...1ms is pleeeenty
  delay(200);

  // read state of 8-bit data bus
  Byte byte = 0;
  for (i = 0; i < 7; i++) {
    // if high, set the corresponding bit
    if (digitalRead(*(lcd.dataPins + i)) == HIGH) {
      byte |= 1 << i;
    }
  }

  digitalWrite(*(lcd.controlPins + CE), HIGH);

  /* // trigger reset */
  pinMode(*(lcd.controlPins + RST), LOW); // RST..start
  delay(500);
  for (i = 0; i < 6; i++) {
    asm("nop");
  }
  pinMode(*(lcd.controlPins + RST), HIGH); // RST..end
  if (digitalRead(*(lcd.dataPins + 0)) == HIGH) {
    byte |= 1 << 1;
  }
  

  // return to previous
  /* setMode(currentRW, currentCD); */

  return byte;
}

void LCDWriteChar(char c)
{
  int i;
  char bit;
  for (i = 0; i < 7; i++) {
    bit = c << (7 - i);
    bit = c >> 7;
    digitalWrite(*(lcd.dataPins + i), bit);
  }
}

void LCDSetBrightness(uint8_t value)
{
  int i;
  // return if no change
  if (value == lcd.brightness) {
    return;
  }
  // change brightness level by inc/dec.s of 1
  if (value > lcd.brightness) {
    for (i = (lcd.brightness + 1); i <= value; i++) {
      analogWrite(lcd.backlightPin, i);
      delay(5);
    }
  } else {
    for (i = (lcd.brightness - 1); i >= value; i--) {
      analogWrite(lcd.backlightPin, i);
      delay(5);
    }
  }
  // set new value
  lcd.brightness = value;
}






/*PRIVATE FUNCTIONS*/
void setMode(uint8_t RW, uint8_t CD)
{
  int i;
  // set Read/Write state
  switch(RW) {
    case READ:
      // set WR high
      digitalWrite(*(lcd.controlPins + WR), HIGH);
      // set RD low
      digitalWrite(*(lcd.controlPins + RD), LOW);
      // set data bus as inputs
      for (i = 0; i < 7; i++) {
        pinMode(*(lcd.dataPins + i), INPUT);
      }
      break;
    case WRITE:
      // set WR low
      digitalWrite(*(lcd.controlPins + WR), LOW);
      // set RD high
      digitalWrite(*(lcd.controlPins + RD), HIGH);
      // set data bus as inputs
      for (i = 0; i < 7; i++) {
        pinMode(*(lcd.dataPins + i), HIGH);
      }
      break;
  }
  // set CE low
  digitalWrite(*(lcd.controlPins + CE), LOW);
  // set Command/Data state
  switch(CD) {
    case COMMAND:
      // low = command
      digitalWrite(*(lcd.controlPins + CD), HIGH);
      break;
    case DATA:
      // high = data
      digitalWrite(*(lcd.controlPins + CD), LOW);
      break;
  }
}
