// System libraries
#include <stdint.h>
#include "Arduino.h"

// User libraries
#include "FS_T6963C_Lib.h"

// Pre-proc.
#define SUCCESS 1
#define FAILURE 0
#define ARBITRARY_NUM_REQUESTS 1000
#define ON 1
#define OFF 0
#define LARGE 1
#define SMALL 0

LCD lcd = {};

// Private function declarations
/* void setMode(uint8_t RW, uint8_t CD); */
void setRW(uint8_t value);
void setCD(uint8_t value);
void setCE(uint8_t value);
void triggerRST();
void setFS(uint8_t value);

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

  // set the fontSize default to SMALL (6 pt.)
  setFS((fontSize == 6 ? SMALL : LARGE));

  // trigger hardware reset
  triggerRST();

  // set backlight PWM output
  pinMode(lcd.backlightPin, OUTPUT); // LED_A
  /* // init backlight to requested brihtness */
  /* analogWrite(lcd.backlightPin, lcd.brightness); */

  // DATA
  for (i = 0; i < 7; i++) {
    pinMode(*(lcd.dataPins + i), HIGH);
  }

  // Wait until T6963C is ready for op
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }


  // NOW perform other necessary initializations..like setting the graphics home address



  // Set lines to operational levels
  /* pinMode(*(lcd.controlPins + CD), HIGH); // CD */
  /* pinMode(*(lcd.controlPins + WR), LOW); // WR */
  /* pinMode(*(lcd.controlPins + RD), HIGH); // RD */
  /* pinMode(*(lcd.controlPins + FS), (fontSize == 6 ? HIGH : LOW)); // FS */

  // inc brightness to requested level to show init complete
  LCDSetBrightness(lcd.brightness, 5);

  // flash backlight
  /* for (; k > 0; k--) { */
  /*   LCDSetBrightness(BCK_OFF, 0); */
  /*   delay(250); */
  /*   LCDSetBrightness(BCK_FULL, 0); */
  /* } */

  return SUCCESS;
}

/*
 * @desc Waits until the controller's two LSBs are SET and is therefore ready for an operation
 * @return Returns false (0) if controller hung in non-operational state for more than ARBITRARY_NUM_REQUESTS
 */
uint8_t LCDWaitUntilReady()
{
  int i = 1; // b/c already made 1 request upon entering while loop
  uint8_t statusByte = LCDGetStatusByte();
  // loop until first two LSBs are SET
  while (!(statusByte & STATUS_READY)) {
    if (i >= ARBITRARY_NUM_REQUESTS) {
      return FAILURE;
    }
    statusByte = LCDGetStatusByte();
    i++;
  }
  return SUCCESS;
}

// uint8_t currentRW, uint8_t currentCD 
uint8_t LCDGetStatusByte()
{
  // set inc.s
  int i;
  // set mode
  setRW(READ);
  setCD(STATUS);
  setCE(ON);

  // wait 150 ns...100MGHz is about 10ns per cycle...1ms is pleeeenty
  delay(1);

  // read state of 8-bit data bus
  uint8_t statusByte = 0;
  for (i = 0; i < 7; i++) {
    // if high, set the corresponding bit
    if (digitalRead(*(lcd.dataPins + i)) == HIGH) {
      statusByte |= 1 << i;
    }
  }
  // lean off CE
  setCE(OFF);
  // return byte
  return statusByte;
}

/* void LCDWriteChar(char c) */
/* { */
/*   int i; */
/*   char bit; */
/*   for (i = 0; i < 7; i++) { */
/*     bit = c << (7 - i); */
/*     bit = c >> 7; */
/*     digitalWrite(*(lcd.dataPins + i), bit); */
/*   } */
/* } */

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
  // just jump to brightness if delay is 0ms
  if (delayTime == 0) {
    analogWrite(lcd.backlightPin, value);
  } else {
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
  }
  // set new value on lcd struct
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
void setFS(uint8_t value)
{
  switch (value) {
    // 6 pt.
    case SMALL:
      digitalWrite(*(lcd.controlPins + FS), HIGH);
      break;
    // 8 pt.
    case LARGE:
      digitalWrite(*(lcd.controlPins + FS), LOW);
      break;
  }
}
