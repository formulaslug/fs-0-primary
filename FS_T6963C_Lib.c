// System libraries
#include <stdint.h>
#include "Arduino.h"
// Private libraries
#include "FS_T6963C_Lib.h"
// Pre-proc.
#define SUCCESS 1
#define FAILURE 0
#define ARBITRARY_NUM_REQUESTS 100000000
#define ON 1
#define OFF 0
#define LARGE 1
#define SMALL 0
#define BYTE 8

LCD lcd = {};

void setRW(uint8_t value);
void setCD(uint8_t value);
void setCE(uint8_t value);
void triggerRST();
void setFS(uint8_t value);
uint8_t writeData(uint8_t byte);
uint8_t readData(uint8_t * byte);
uint8_t writeCommand(uint8_t byte);

// function defintions
uint8_t LcdInit(uint16_t lcdWidth, uint16_t lcdHeight, uint8_t fontSize, uint8_t brightness, uint8_t * controlPins, uint8_t * dataPins, uint8_t backlightPin)
{
  int i;
  // set initial lcd vals
  lcd.width = lcdWidth;
  lcd.height = lcdHeight;
  lcd.fontSize = fontSize;
  lcd.brightness = 0; // initialized in LCDSetBrightness()
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

  // set backlight PWM pin output
  pinMode(lcd.backlightPin, OUTPUT); // LED_A
  // init backlight to requested brihtness
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
  /* LCDSetBrightness(lcd.brightness, 5); */
  /* LCDSetBrightness(0, 0); */
  /* LCDSetBrightness(200, 1000); */
  /* LCDSetBrightness(lcd.brightness, 10); */


  // flash backlight
  /* for (; k > 0; k--) { */
  /*   LCDSetBrightness(BCK_OFF, 0); */
  /*   delay(250); */
  /*   LCDSetBrightness(BCK_FULL, 0); */
  /* } */


  // Setting "GRAPHICS HOME ADDRESS"
  uint8_t data[2], command;
  data[0] = 0x00;
  data[1] = 0x40; // to 128
  command = 0x42;
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeData(data[0]);
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeData(data[1]);
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeCommand(command);

  // Set "GRAPHICS AREA SET"
  data[0] = 0x1e;
  data[1] = 0x00; // to 128
  command = 0x43;
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeData(data[0]);
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeData(data[1]);
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeCommand(command);
  

  // Setting "TEXT HOME ADDRESS"
  data[0] = 0x41; // from 129
  data[1] = 0xff; // to 256
  command = 0x40;
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeData(data[0]);
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeData(data[1]);
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeCommand(command);


  // Set "TEXT AREA SET"
  data[0] = 0x1e;
  data[1] = 0x00;
  command = 0x41;
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeData(data[0]);
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeData(data[1]);
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeCommand(command);


  // Set "MODE SET"
  command = 0x84; // 10000100 - MODE SET command, CG-ROM mode, Text only (attribute) mode
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeCommand(command);


  // Set "DISPLAY MODE"
  command = 0x9f; // 10011111 - DISPLAY MODE command, GRPH on (b/c txt attribute mode), TEXT on, CUR on, BLK on
  if (!LCDWaitUntilReady()) {
    return FAILURE;
  }
  writeCommand(command);

  

  





  // show feedback that lcd has been initialized
  LCDSetBrightness(brightness, 1000);

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

/*
 * @param delay: total time in milliseconds for the transition
 */
uint8_t LCDSetBrightness(uint8_t value, int delayTime)
{
  int i;
  // return if no change
  if (value == lcd.brightness) {
    return 0;
  }
  // just jump to brightness if delay is 0ms
  if (delayTime == 0) {
    analogWrite(lcd.backlightPin, value);
  } else {
    // absolute value of the difference from previous value 
    int valueDiffAbs = value < lcd.brightness ? 
      -(value - lcd.brightness) : (value - lcd.brightness); 
    // delay time between each increment
    int delayTimeEach = delayTime/valueDiffAbs;
    // change brightness level acording to the delay
    if (value > lcd.brightness) {
      for (i = (lcd.brightness + 1); i <= value; i++) {
        analogWrite(lcd.backlightPin, i);
        delay(delayTimeEach);
      }
    } else {
      for (i = (lcd.brightness - 1); i >= value; i--) {
        analogWrite(lcd.backlightPin, i);
        delay(delayTimeEach);
      }
    }
  }
  // set new value on lcd struct
  lcd.brightness = value;
  return 1;
}

/*
 * @param
 */
uint8_t LCDSetGraphicsHomeAddress()
{
  return FAILURE;
  // wait until status=ready
  // send first data byte (low address byte)
  // wait until status=ready
  // send second data byte (high address byte)
  // wait until status=ready
  // send command to set graphics home address given previously received bytes
}







/*PRIVATE FUNCTIONS*/
void setRW(uint8_t value)
{
  int i;
  // set Read/Write state
  switch(value) {
    case READ:
      // set WR high
      digitalWrite(lcd.controlPins[WR], HIGH);
      // set RD low
      digitalWrite(lcd.controlPins[RD], LOW);
      // set data bus as inputs
      for (i = 0; i < 7; i++) {
        pinMode(lcd.dataPins[i], INPUT);
      }
      break;
    case WRITE:
      // set WR low
      digitalWrite(lcd.controlPins[WR], LOW);
      // set RD high
      digitalWrite(lcd.controlPins[RD], HIGH);
      // set data bus as inputs
      for (i = 0; i < 7; i++) {
        pinMode(lcd.dataPins[i], HIGH);
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
      digitalWrite(lcd.controlPins[CD], HIGH);
      break;
    case DATA:
      // high = data
      digitalWrite(lcd.controlPins[CD], LOW);
      break;
  }
}
void setCE(uint8_t value)
{
  switch (value) {
    case ON:
      digitalWrite(lcd.controlPins[CE], LOW);
      break;
    case OFF:
      digitalWrite(lcd.controlPins[CE], HIGH);
      break;
  }
}
void triggerRST()
{
  int i;
  // trigger reset
  pinMode(lcd.controlPins[RST], LOW); // RST..start
  // hold for 7 cycles
  for (i = 0; i < 6; i++) {
    asm("nop");
  }
  // lean off
  pinMode(lcd.controlPins[RST], HIGH); // RST..end
}
void setFS(uint8_t value)
{
  switch (value) {
    // 6 pt.
    case SMALL:
      digitalWrite(lcd.controlPins[FS], HIGH);
      break;
    // 8 pt.
    case LARGE:
      digitalWrite(lcd.controlPins[FS], LOW);
      break;
  }
}
uint8_t writeData(uint8_t byte) {
  int i;
  uint8_t tmp;

  // set the bits on the data bus
  for (i = 0; i < BYTE; i++) {
    tmp = byte << ((BYTE - 1) - i);
    tmp >>= (BYTE - 1);
    if (tmp) {
      digitalWrite(lcd.dataPins[i], HIGH);
    } else {
      digitalWrite(lcd.dataPins[i], LOW);
    }
  }

  // set and/or pulse the control lines
  setCD(DATA);
  setRW(WRITE);
  setCE(ON);
  // delay 80ns
  //
  delay(1);
  /* for (i = 0; i < 8; i++) { */
  /*   asm("nop"); */
  /* } */
  setCE(OFF);

  return SUCCESS;
}

uint8_t readData(uint8_t * byte) {
  int i;
  uint8_t tmp;

  // set and/or pulse control lines
  setCD(DATA);
  setRW(READ);
  setCE(ON);

  // delay 150ns
  for (i = 0; i < 15; i++) {
    asm("nop");
  }
  // read the bits on the data bus
  for (i = 0; i < BYTE; i++) {
    tmp = digitalRead(lcd.dataPins[i]) == HIGH ? 1 : 0;
    *byte |= (tmp << i);
  }
  setCE(OFF);

  return SUCCESS;
}

uint8_t writeCommand(uint8_t byte) {
  int i;
  uint8_t tmp;
  
  // set the bits on the data bus for the command id
  for (i = 0; i < BYTE; i++) {
    tmp = (byte << ((BYTE - 1) - i)) >> (BYTE - 1);
    if (tmp) {
      digitalWrite(lcd.dataPins[i], HIGH);
    } else {
      digitalWrite(lcd.dataPins[i], LOW);
    }
  }

  setCD(COMMAND);
  setRW(WRITE);
  setCE(ON);
  // delay 80ns
  delay(1);
  /* for (i = 0; i < 8; i++) { */
  /*   asm("nop"); */
  /* } */
  setCE(OFF);

  return SUCCESS;

}
