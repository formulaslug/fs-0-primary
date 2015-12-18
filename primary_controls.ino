/*
 * @authors Milo Webster, 
 * @desc Pimary control system for the UCSC's Formula Slug Electric FSAE Vehicle
 * @dict LV = Low Voltage System, HV = High Voltage System, RTD = Ready-To-Drive
 */


// Rev notes: Need some button debounce


// System libraries
#include <stdint.h>
#include <stdio.h>

// User libraries

// Pre-proc. Dirs.
#define NUM_LEDS 5
#define NUM_BUTTONS 2
#define NUM_LCD_DATA_PINS
// operating on all 8 bits so that can be notted "~"
#define true 0xff
#define false 0x00
#define ON 0xff
#define OFF 0x0
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
  STATUS,
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
const uint8_t lcdPinds[NUM_LCD_DATA_PINDS] = {14, 18, 15, 19, 16, 20, 17, 21}; // order of array is order of corresponding 8 pins 11-18 on the lcd, use this to lookup needed pin on teensy

// Main setup function
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Setup Complete.");

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
  vehicle.dynamics.torque = 50;
  for (i = 0; i < NUM_LEDS; i++) {
    vehicle.leds[i] = OFF;
  }
  // show setup complete
  vehicle.leds[STATUS] = ON;
  // output confirmation
}

// Main control run loop
void loop() {
  int i;

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
      vehicle.leds[BLUE] = ON;
      vehicle.leds[YELLOW] = OFF;
      vehicle.leds[RED] = OFF;
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
      vehicle.leds[BLUE] = ON;
      vehicle.leds[YELLOW] = ON;
      vehicle.leds[RED] = OFF;
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
        vehicle.leds[BLUE] = ON;
        vehicle.leds[YELLOW] = ON;
        vehicle.leds[RED] = ON;
      } else {
        // get speed
        vehicle.dynamics.torque = (int)(analogRead(TORQUE_INPUT) / 2);
        // show speed
        vehicle.leds[SPEED] = ~vehicle.leds[SPEED];
        // wait to transition back
        if (digitalRead(buttonPins[RTD_TOGGLE]) == LOW) {
          // move back to HV_ACTIVE
          vehicle.leds[SPEED] = OFF;
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
  delay(vehicle.dynamics.torque); // vehicle.ledTime
  // an incrementor for num cycles awareness in FSM
  (vehicle.i)++;
}


