// System libraries
#include <stdint.h>
#include <stdio.h>

// User libraries

#define NUM_LEDS 4
// operating on all 8 bits so that can be notted "~"
#define true 0xff
#define false 0x00
#define ON 0xff
#define OFF 0x0

// Data types
/* typedef uint8_t bool; */
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
  RED
};
// the main attributes of the vehicle
typedef struct Vehicle {
  uint8_t state;
  uint8_t leds[NUM_LEDS]; // led values are 0x0 or 0xff to allow for bit-wise not
  int i;
  int ledTime;
} Vehicle;

Vehicle vehicle = {};

const uint8_t ledPins[NUM_LEDS] = {2, 3, 4, 13};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // init leds
  int i;
  for (i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], HIGH);
  }
  // init the vehicle
  vehicle.state = LV_STARTUP;
  vehicle.i = 0;
  vehicle.ledTime = 50;
  for (i = 0; i < NUM_LEDS; i++) {
    vehicle.leds[i] = OFF;
  }
  /* printf("Performed setup"); */
  /* putc('d', stdout); */
  Serial.println("Done");
  // show setup complete
  vehicle.leds[3] = ON;
  digitalWrite(ledPins[3], HIGH);
}

void loop() {
  int i;
  switch (vehicle.state) {
    case LV_STARTUP:
      // invert the led for this state
      vehicle.leds[BLUE] = ~vehicle.leds[BLUE];
      if (vehicle.i >= 20) {
        vehicle.i = 0;
        vehicle.state = LV_ACTIVE;
      }
      break;
    case LV_ACTIVE:
      vehicle.leds[BLUE] = ON;
      vehicle.state = HV_STARTUP;
      break;
    case HV_SD:
      break;
    case HV_STARTUP:
      vehicle.leds[YELLOW] = ~vehicle.leds[YELLOW];
      if (vehicle.i >= 20) {
        vehicle.i = 0;
        vehicle.state = HV_ACTIVE;
      }
      break;
    case HV_ACTIVE:
      vehicle.leds[YELLOW] = ON;
      vehicle.state = RTD_STARTUP;
      break;
    case RTD_SD:
      break;
    case RTD_STARTUP:
      vehicle.leds[RED] = ~vehicle.leds[RED];
      if (vehicle.i >= 20) {
        vehicle.i = 0;
        vehicle.state = RTD_ACTIVE;
      }
      break;
    case RTD_ACTIVE:
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
  // set leds to their states
  for (i = 0; i < NUM_LEDS; i++) {
    if (vehicle.leds[i]) {
      digitalWrite(ledPins[i], HIGH);
    } else {
      digitalWrite(ledPins[i], LOW);
    }
  }
  delay(vehicle.ledTime);
  (vehicle.i)++;
}

