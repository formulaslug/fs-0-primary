/*
 * @authors Milo Webster, 
 * @desc Pimary control system for the UCSC's Formula Slug Electric FSAE Vehicle
 * @dict LV = Low Voltage System, HV = High Voltage System, RTD = Ready-To-Drive
 */


// Rev notes: Need some button debounce


// System libraries
#include <stdint.h>
#include <stdio.h>
// LCD - T6963
/* #include <U8glib.h> */
/* #include "T6963.h" */
/* #include "gfxdata.h" */
/* #include "Times_New_Roman__14.h" */

/* T6963 LCD(240,64,6,32);// 240x64 Pixel and 6x8 Font */

// User libraries
extern "C" {
  #include "FS_T6963C_Lib.h"
}

// Pre-proc. Dirs.
#define NUM_LEDS 5
#define NUM_BUTTONS 2
#define LCD_BACKLIGHT_VIN_PIN 22
// operating on all 8 bits so that can be notted "~"
#define true 0xff
#define false 0x00
#define LED_ON 0xff
#define LED_OFF 0x0
#define TORQUE_INPUT A9
enum States {
  LV_STARTUP,
  LV_ACTIVE,
  HV_SD,
  HV_STARTUP,
  HV_ACTIVE,
  RTD_SD,
  RTD_STARTUP,
  RTD_ACTIVE,
};
enum Leds {
  BLUE,
  YELLOW,
  RED,
  STATUS_LED,
  SPEED
};
enum Buttons {
  HV_TOGGLE,
  RTD_TOGGLE
};

// Data types
typedef struct Dynamics {
  uint16_t torque;
  uint16_t speed;
  uint8_t topSpeed = 60;
} Dynamics;

typedef struct Vehicle { // the main attributes of the vehicle
  uint8_t state;
  uint8_t leds[NUM_LEDS]; // led values are 0x0 or 0xff to allow for bit-wise not
  Dynamics dynamics;
  int i;
} Vehicle;

// Globals
Vehicle vehicle = {};
const uint8_t ledPins[NUM_LEDS] = {2, 3, 4, 13, 5};
const uint8_t buttonPins[NUM_BUTTONS] = {7, 8};
uint8_t dataPins[NUM_DATA_PINS] = {14, 18, 15, 19, 16, 20, 17, 21}; // order of array is order of corresponding 8 pins 11-18 on the lcd, use this to lookup needed pin on teensy
uint8_t controlPins[NUM_CNTRL_PINS] = {0, 1, 2, 3, 4, 5}; // order of array is order of corresponding 8 pins 11-18 on the lcd, use this to lookup needed pin on teensy
const uint8_t lcdLighting[13] = {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 140, 150};

// U8GLIB_T6963_240X64(d0, d1, d2, d3, d4, d5, d6, d7, cs, a0, wr, rd [, reset]);
/* U8GLIB_T6963_240X64(teensy2LcdPins[0], teensy2LcdPins[1], teensy2LcdPins[2], teensy2LcdPins[3], teensy2LcdPins[4], teensy2LcdPins[5], teensy2LcdPins[6], teensy2LcdPins[7], 6, 9, 10, 11, 12); */


// Main setup function
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Setup Complete.");

  // init leds pins
  int i;
  for (i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], HIGH);
  }
  // init buttons pins
  for (i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT);
  }

  // init the vehicle
  vehicle.state = LV_STARTUP;
  vehicle.i = 0;
  vehicle.dynamics.torque = 10;
  for (i = 0; i < NUM_LEDS; i++) {
    vehicle.leds[i] = LED_OFF;
  }
  // show setup complete
  vehicle.leds[STATUS_LED] = ON;



}

int up = 1;
int glb = 0;
int length = 200;
int setupIncomplete = 1;
uint8_t statusByte = 0;
uint8_t tmp = 0;
// Main control run loop
void loop() {
  int i;

  // wait for lcd to heat up
  if (setupIncomplete) {
    // New LCD Stuff
    LcdInit(240, 64, 6, BCK_LOW, controlPins, dataPins, 22);
    Serial.println("Status: Ready");
    setupIncomplete = 0;
    /* if (!(statusByte & STATUS_READY)) { */
    /*   statusByte = LCDGetStatusByte(WRITE, DATA); */
    /*   Serial.print("Status: "); */
    /*   for (i = NUM_DATA_PINS; i >= 0; i--) { */
    /*     tmp = statusByte << (7-i); */
    /*     tmp = tmp >> 7; */
    /*     Serial.print(tmp); */
    /*   } */
    /*   Serial.println("."); */
    /* } else { */
    /*   Serial.println("Status: Ready"); */
    /*   setupIncomplete = 0; */
    /*   // inc brightness to operational level */
    /*   LCDSetBrightness(250); */
    /* } */
  }
  statusByte = LCDGetStatusByte();
  Serial.print("Status: ");
  for (i = NUM_DATA_PINS; i >= 0; i--) {
    tmp = statusByte << (7-i);
    tmp = tmp >> 7;
    Serial.print(tmp);
  }
  Serial.println(".");

  /* Serial.println("Cycle"); */
  if (glb == length) {
    up = 0;
  } else if (glb == 0) {
    up = true;
  }
  /* analogWrite(LCD_BACKLIGHT_VIN_PIN, 50+glb); */

  if (up) {
    glb += 2;
  } else {
    glb -= 2;
  }

  // Vehicle's main state machine (FSM)
  switch (vehicle.state) {
    case LV_STARTUP:
      // perform LV_STARTUP functions
        // HERE
      // transition to LV_ACTIVE
      vehicle.state = LV_ACTIVE;
      break;
    case LV_ACTIVE:
      // set led feedback
      vehicle.leds[BLUE] = LED_ON;
      vehicle.leds[YELLOW] = LED_OFF;
      vehicle.leds[RED] = LED_OFF;
      // wait to move to HV_STARTUP
      if (digitalRead(buttonPins[HV_TOGGLE]) == LOW) {
        vehicle.state = HV_STARTUP;
      }
      break;
    case HV_SD:
      delay(250);
      // perform HV_SD functions
        // HERE
      // transition to LV_ACTIVE
      vehicle.state = LV_ACTIVE;
      break;
    case HV_STARTUP:
      delay(250);
      // perform LV_STARTUP functions
        // HERE
      // transition to LV_ACTIVE
      vehicle.state = HV_ACTIVE;
      break;
    case HV_ACTIVE:
      // set led feedback
      vehicle.leds[BLUE] = LED_ON;
      vehicle.leds[YELLOW] = LED_ON;
      vehicle.leds[RED] = LED_OFF;
      // wait to move to RTD_STARTUP until user input
      if (digitalRead(buttonPins[RTD_TOGGLE]) == LOW) {
        vehicle.state = RTD_STARTUP;
      } else if (digitalRead(buttonPins[HV_TOGGLE]) == LOW) {
        // Or move back to LV active
        vehicle.state = HV_SD;
      }
      break;
    case RTD_SD:
      delay(250);
      // perform HV_SD functions
        // HERE
      // transition to LV_ACTIVE
      vehicle.state = HV_ACTIVE;
      break;
    case RTD_STARTUP:
      delay(250);
      // perform LV_STARTUP functions
        // HERE
      // transition to LV_ACTIVE
      vehicle.i = 0;
      vehicle.state = RTD_ACTIVE;
      break;
    case RTD_ACTIVE:
      // show entire system is hot
      if (vehicle.i <= 1) {
        vehicle.leds[BLUE] = LED_ON;
        vehicle.leds[YELLOW] = LED_ON;
        vehicle.leds[RED] = LED_ON;
      } else {
        // get speed
        vehicle.dynamics.torque = (int)(analogRead(TORQUE_INPUT) / 2);
        // show speed
        vehicle.leds[SPEED] = ~vehicle.leds[SPEED];
        // wait to transition back
        if (digitalRead(buttonPins[RTD_TOGGLE]) == LOW) {
          // move back to HV_ACTIVE
          vehicle.leds[SPEED] = LED_OFF;
          vehicle.dynamics.torque = 50;
          vehicle.state = RTD_SD;
        }
      }
      break;
  }

  // set leds to their states after FSM processing
  for (i = 0; i < NUM_LEDS; i++) {
    if (vehicle.leds[i]) {
      digitalWrite(ledPins[i], HIGH);
    } else {
      digitalWrite(ledPins[i], LOW);
    }
  }

  // add some delay for temporary visual effects
  /* delay(vehicle.dynamics.torque); // vehicle.ledTime */
  // an incrementor for num cycles awareness in FSM
  (vehicle.i)++;
}


