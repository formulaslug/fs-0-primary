// System libraries
#include <stdint.h>
#include "Arduino.h"

// User libraries
#include "FS_T6963C_Lib.h"

LCD lcd = {};

// Private function declarations
/* void setMode(uint8_t RW, uint8_t CD); */
void setRW(uint8_t value);
void setCD(uint8_t value);
void setCE(uint8_t value);
void triggerRST();

// function defintions
uint8_t LcdInit(uint16_t lcdWidth, uint16_t lcdHeight, uint8_t fontSize, uint8_t brightness, uint8_t * controlPins, uint8_t * dataPins, uint8_t backlightPin)
{
  int i;
  // set initial lcd vals
  lcd.width = lcdWidth;
  lcd.height = lcdHeight;
  lcd.fontSize = fontSize;
  lcd.brightness = brightness;
  lcd.controlPins = controlPins;
  lcd.dataPins = dataPins;
  lcd.backlightPin = backlightPin;

  // CONTROLS
  // set digital outputs
  for (i = 0; i < NUM_CNTRL_PINS; i++) {
    pinMode(*(lcd.controlPins + i), HIGH); // WR
  }
  // init digital outputs
  // trigger reset
  triggerRST();

  

  // set analog outputs (PWM)
  pinMode(lcd.backlightPin, OUTPUT); // LED_A
  // init analog outputs
  analogWrite(lcd.backlightPin, lcd.brightness);

  // DATA
  for (i = 0; i < 7; i++) {
    pinMode(*(lcd.dataPins + i), HIGH);
  }

  uint8_t statusByte = 0, tmp = 0;
  // Get status and wait until ready
  while (!(statusByte & STATUS_READY)) {
    statusByte = LCDGetStatusByte(WRITE, DATA);
    for (i = NUM_DATA_PINS; i >= 0; i--) {
      tmp = statusByte << (7-i);
      tmp = tmp >> 7;
    }
  }
  // Now ready
  // Set lines to operational levels
  /* pinMode(*(lcd.controlPins + CD), HIGH); // CD */
  /* pinMode(*(lcd.controlPins + WR), LOW); // WR */
  /* pinMode(*(lcd.controlPins + RD), HIGH); // RD */
  /* pinMode(*(lcd.controlPins + FS), (fontSize == 6 ? HIGH : LOW)); // FS */
  // inc brightness to operational level
  LCDSetBrightness(BCK_FULL, 5);

  return 1;
}

// uint8_t currentRW, uint8_t currentCD 
Byte LCDGetStatusByte()
{
  // set inc.s
  int i;
  // set mode
  setRW(READ);
  setCD(STATUS);
  setCE(ON);

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
  // lean off CE
  setCE(OFF);
  // return byte
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

/*
 * @param delay: Delay time for each inc/dec in milliseconds
 */
void LCDSetBrightness(uint8_t value, uint8_t delayTime)
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
      delay(delayTime);
    }
  } else {
    for (i = (lcd.brightness - 1); i >= value; i--) {
      analogWrite(lcd.backlightPin, i);
      delay(delayTime);
    }
  }
  // set new value
  lcd.brightness = value;
}






/*PRIVATE FUNCTIONS*/
void setRW(uint8_t value)
{
  int i;
  // set Read/Write state
  switch(value) {
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
}
void setCD(uint8_t value)
{
  switch(value) {
    // or STATUS
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
void setCE(uint8_t value)
{
  switch (value) {
    case ON:
      digitalWrite(*(lcd.controlPins + CE), LOW);
      break;
    case OFF:
      digitalWrite(*(lcd.controlPins + CE), HIGH);
      break;
  }
}
void triggerRST()
{
  int i;
  // trigger reset
  pinMode(*(lcd.controlPins + RST), LOW); // RST..start
  // hold for 7 cycles
  for (i = 0; i < 6; i++) {
    asm("nop");
  }
  // lean off
  pinMode(*(lcd.controlPins + RST), HIGH); // RST..end
}
