/*
 * @desc Pimary control system for the UCSC's Formula Slug Electric FSAE Vehicle
 * @dict LV = Low Voltage System, HV = High Voltage System, RTD = Ready-To-Drive
 */

// Rev notes: Need some button debounce

#include <cstdint>
#include <cstdio>

// LCD - T6963
// #include <U8glib.h>
// #include "T6963.h"
// #include "gfxdata.h"
// #include "Times_New_Roman__14.h"

// T6963 LCD(240, 64, 6, 32); // 240x64 Pixel and 6x8 Font

extern "C" {
  #include "FS_T6963C_Lib.h"
}

#include <array>
#include <cstdint>

constexpr uint8_t NUM_LEDS = 5;
constexpr uint8_t NUM_BUTTONS = 2;
constexpr uint8_t LCD_BACKLIGHT_VIN_PIN = 22;
constexpr uint8_t ON = 1;
constexpr uint8_t OFF = 0;

// Operating on all 8 bits so that can be notted "~"
constexpr uint8_t LED_ON = 0xff;
constexpr uint8_t LED_OFF = 0x00;
constexpr uint8_t TORQUE_INPUT = A9;

enum States {
  LV_STARTUP,
  LV_ACTIVE,

  HV_STARTUP,
  HV_ACTIVE,
  HV_SHUTDOWN,

  RTD_STARTUP,
  RTD_ACTIVE,
  RTD_SHUTDOWN,
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

struct Dynamics {
  uint16_t torque;
  uint16_t speed;
  uint8_t topSpeed = 60;
};

// U8GLIB_T6963_240X64(d0, d1, d2, d3, d4, d5, d6, d7, cs, a0, wr, rd [, reset]);
/* U8GLIB_T6963_240X64(teensy2LcdPins[0], teensy2LcdPins[1], teensy2LcdPins[2],
 *                     teensy2LcdPins[3], teensy2LcdPins[4], teensy2LcdPins[5],
 *                     teensy2LcdPins[6], teensy2LcdPins[7], 6, 9, 10, 11, 12);
 */

class Vehicle {
 public:
  Vehicle() {
    for (auto& led : ledStates) {
      led = LED_OFF;
    }
    ledStates[STATUS_LED] = ON;
  }

  uint8_t state = LV_STARTUP;

  Dynamics dynamics;

  // LED values are 0x00 or 0xff to allow for bitwise not
  std::array<uint8_t, NUM_LEDS> ledStates;

  int numCycles = 0;
};

// Globals
Vehicle gVehicle;
const std::array<uint8_t, NUM_LEDS> gLedPins{2, 3, 4, 13, 5};
const std::array<uint8_t, NUM_BUTTONS> gButtonPins{7, 8};

/* Order of array is order of corresponding 8 pins 11-18 on the LCD.
 * Use this to look up needed pin on Teensy
 */
std::array<uint8_t, NUM_DATA_PINS> gDataPins{14, 18, 15, 19, 16, 20, 17, 21};
std::array<uint8_t, NUM_CNTRL_PINS> gControlPins{0, 1, 2, 3, 4, 5};

std::array<uint8_t, 13> gLcdLighting{20, 30, 40, 50, 60, 70, 80, 90, 100, 110,
                                     120, 140, 150};

void setup() {
  Serial.begin(9600);
  Serial.println("Setup Complete.");

  // Init LEDs
  for (auto& ledPin : gLedPins) {
    pinMode(ledPin, HIGH);
  }
  // Init buttons
  for (auto& buttonPin : gButtonPins) {
    pinMode(buttonPin, INPUT);
  }

  // Init vehicle
  gVehicle.dynamics.torque = 10;
}

// Used for LCD
bool setupIncomplete = true;
uint8_t lcdStatus = 0;

// Used for debugging LCD
int printCounter = 0;
int cycles = 0, readies = 0, notReadies = 0;

// Main control run loop
void loop() {
  /* Left off trying to finish lcd initialization by setting the graphics home
   * address detailed in the first pdf and then expanded on in the others
   */

  // Init LCD
  if (setupIncomplete) {
    // Wait for lcd to heat up
    delay(1000);
    if (!LcdInit(240, 64, 8, BCK_FULL, &gControlPins[0], &gDataPins[0],
                 LCD_BACKLIGHT_VIN_PIN)) {
      Serial.println("Error: LCD Failed to Initialize");
    } else {
      Serial.println("Status: Ready");
      setupIncomplete = false;
    }

    /* if (lcdStatus & STATUS_READY) {
     *   Serial.println("Status: Ready");
     *   setupIncomplete = false;
     *
     *   // Increase brightness to operational level
     *   LCDSetBrightness(250);
     * }
     * else {
     *   lcdStatus = LCDGetStatusByte(WRITE, DATA);
     *   Serial.print("Status: ");
     *   for (int i = 0; i < NUM_DATA_PINS; i++) {
     *     Serial.print((lcdStatus & (1 << i)) >> i);
     *   }
     *   Serial.println(".");
     * }
     */
  } else {
    /* int brightness = LCDSetBrightness(200, 1000);
     * Serial.print("Time between is :");
     * Serial.println(brightness);
     */
    lcdStatus = LCDGetStatusByte();

    // If LCD is ready
    if (lcdStatus & STATUS_READY) {
      readies++;
    } else {
      notReadies++;
    }

    if (printCounter % 500 == 0) {
      Serial.print("Status: Readys=");
      Serial.print(readies);
      Serial.print(", Not-Readys=");
      Serial.print(notReadies);
      Serial.print(", Cycles=");
      Serial.print(cycles);
      Serial.println(".");
      printCounter = 0;
    }

    printCounter++;
    cycles++;
  }


  /* Serial.print("Status: ");
   * for (int i = 0; i < NUM_DATA_PINS; i++) {
   *   Serial.print((lcdStatus & (1 << i)) >> i);
   * }
   * Serial.println(".");
   */

  // Vehicle's main state machine (FSM)
  switch (gVehicle.state) {
    case LV_STARTUP:
      // Perform LV_STARTUP functions
      // HERE

      // Transition to LV_ACTIVE
      gVehicle.state = LV_ACTIVE;
      break;
    case LV_ACTIVE:
      // Set LED feedback
      gVehicle.ledStates[BLUE] = LED_ON;
      gVehicle.ledStates[YELLOW] = LED_OFF;
      gVehicle.ledStates[RED] = LED_OFF;

      // Wait to move to HV_STARTUP
      if (digitalRead(gButtonPins[HV_TOGGLE]) == LOW) {
        gVehicle.state = HV_STARTUP;
      }
      break;
    case HV_SHUTDOWN:
      // Perform HV_SHUTDOWN functions
      // HERE

      // Transition to LV_ACTIVE
      gVehicle.state = LV_ACTIVE;
      break;
    case HV_STARTUP:
      // Perform LV_STARTUP functions
      // HERE

      // Transition to LV_ACTIVE
      gVehicle.state = HV_ACTIVE;
      break;
    case HV_ACTIVE:
      // Set LED feedback
      gVehicle.ledStates[BLUE] = LED_ON;
      gVehicle.ledStates[YELLOW] = LED_ON;
      gVehicle.ledStates[RED] = LED_OFF;

      // Wait to move to RTD_STARTUP until user input
      if (digitalRead(gButtonPins[RTD_TOGGLE]) == LOW) {
        gVehicle.state = RTD_STARTUP;
      } else if (digitalRead(gButtonPins[HV_TOGGLE]) == LOW) {
        // Or move back to LV active
        gVehicle.state = HV_SHUTDOWN;
      }
      break;
    case RTD_SHUTDOWN:
      // Perform HV_SHUTDOWN functions
      // HERE

      // Transition to LV_ACTIVE
      gVehicle.state = HV_ACTIVE;
      break;
    case RTD_STARTUP:
      // Perform LV_STARTUP functions
      // HERE

      // Transition to LV_ACTIVE
      gVehicle.numCycles = 0;
      gVehicle.state = RTD_ACTIVE;
      break;
    case RTD_ACTIVE:
      // Show entire system is hot
      if (gVehicle.numCycles <= 1) {
        gVehicle.ledStates[BLUE] = LED_ON;
        gVehicle.ledStates[YELLOW] = LED_ON;
        gVehicle.ledStates[RED] = LED_ON;
      } else {
        // Get speed
        gVehicle.dynamics.torque = (int) analogRead(TORQUE_INPUT) / 2;

        // Show speed
        gVehicle.ledStates[SPEED] = ~gVehicle.ledStates[SPEED];

        // Wait to transition back
        if (digitalRead(gButtonPins[RTD_TOGGLE]) == LOW) {
          // Move back to HV_ACTIVE
          gVehicle.ledStates[SPEED] = LED_OFF;
          gVehicle.dynamics.torque = 50;
          gVehicle.state = RTD_SHUTDOWN;
        }
      }
      break;
  }

  // An incrementor for num cycles awareness in FSM
  gVehicle.numCycles++;
}

