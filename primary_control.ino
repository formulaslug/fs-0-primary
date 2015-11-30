/*
 * @authors Milo Webster, 
 * @desc Pimary control system for the UCSC's Formula Slug Electric FSAE Vehicle
 * @dict LV = Low Voltage System, HV = High Voltage System, RTD = Ready-To-Drive
 */
// System libraries
#include <stdint.h>
#include <stdio.h>

// User libraries

// Pre-proc. Dirs.
#define NUM_LEDS 4
#define NUM_BUTTONS 2
// operating on all 8 bits so that can be notted "~"
#define true 0xff
#define false 0x00
#define ON 0xff
#define OFF 0x0
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
  STATUS
};
enum Buttons {
  HV_TOGGLE,
  RTD_TOGGLE
};

// Data types
typedef struct Vehicle { // the main attributes of the vehicle
  uint8_t state;
  uint8_t leds[NUM_LEDS]; // led values are 0x0 or 0xff to allow for bit-wise not
  int i;
  int ledTime;
} Vehicle;

// Globals
Vehicle vehicle = {};
const uint8_t ledPins[NUM_LEDS] = {2, 3, 4, 13};
const uint8_t buttonPins[NUM_BUTTONS] = {7, 8};

// Main setup function
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // init leds
  int i;
  for (i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], HIGH);
  }
  // init buttons
  for (i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT);
  }
  // init the vehicle
  vehicle.state = LV_STARTUP;
  vehicle.i = 0;
  vehicle.ledTime = 50;
  for (i = 0; i < NUM_LEDS; i++) {
    vehicle.leds[i] = OFF;
  }
  // show setup complete
  vehicle.leds[STATUS] = ON;
  // output confirmation
  Serial.println("Setup Complete.");
}

// Main control run loop
void loop() {
  int i;

  // Vehicle's main state machine (FSM)
  switch (vehicle.state) {
    case LV_STARTUP:
      // toggle (invert) LV led during startup
      vehicle.leds[BLUE] = ~vehicle.leds[BLUE];
      // temp artificial timer for startup
      if (vehicle.i >= 20) {
        vehicle.i = 0;
        vehicle.state = LV_ACTIVE;
      }
      break;
    case LV_ACTIVE:
      // set the led to show LV is active
      vehicle.leds[BLUE] = ON;
      // wait to move to HV_STARTUP
      if (digitalRead(buttonPins[HV_TOGGLE]) == LOW) {
        vehicle.i = 0;
        vehicle.state = HV_STARTUP;
      }
      break;
    case HV_SD:
      break;
    case HV_STARTUP:
      // toggle (invert) HV led during startup
      vehicle.leds[YELLOW] = ~vehicle.leds[YELLOW];
      // temp artificial timer for startup
      if (vehicle.i >= 20) {
        vehicle.i = 0;
        vehicle.state = HV_ACTIVE;
      }
      break;
    case HV_ACTIVE:
      // set the led to show LV is active
      vehicle.leds[YELLOW] = ON;
      // wait to move to RTD_STARTUP until user input
      if (digitalRead(buttonPins[RTD_TOGGLE]) == LOW) {
        vehicle.i = 0;
        vehicle.state = RTD_STARTUP;
      }
      break;
    case RTD_SD:
      break;
    case RTD_STARTUP:
      // toggle (invert) RTD led during startup
      vehicle.leds[RED] = ~vehicle.leds[RED];
      // temp artificial timer for startup
      if (vehicle.i >= 20) {
        vehicle.i = 0;
        vehicle.state = RTD_ACTIVE;
      }
      break;
    case RTD_ACTIVE:
      // show entire system is hot
      if (vehicle.i == 0) {
        vehicle.leds[RED] = ON;
        vehicle.ledTime = 20;
      } else if (vehicle.i < 20) {
        vehicle.leds[BLUE] = ~vehicle.leds[BLUE];
        vehicle.leds[YELLOW] = ~vehicle.leds[YELLOW];
        vehicle.leds[RED] = ~vehicle.leds[RED];
      } else if (vehicle.i == 20) {
        vehicle.leds[BLUE] = ON;
        vehicle.leds[YELLOW] = ON;
        vehicle.leds[RED] = ON;
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

  // add some delay for visual effects
  delay(50);//vehicle.ledTime
  (vehicle.i)++;
}

