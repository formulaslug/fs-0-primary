// Copyright (c) Formula Slug 2016. All Rights Reserved.

#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdint.h>

#include <array>

constexpr uint8_t k_numLEDs = 5;
constexpr uint8_t k_numButtons = 2;
static constexpr uint8_t ON = 1;
static constexpr uint8_t OFF = 0;

// Operating on all 8 bits so that can be notted "~"
static constexpr uint8_t LED_ON = 0xff;
static constexpr uint8_t LED_OFF = 0x00;

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

enum DriveProfiles {
  SAFE,       // restricted performace (pre-op)
  ENDURANCE,  // competition ready: endurance mode
  SPORT       // competition ready: insane mode?
};

enum Leds { BLUE, YELLOW, RED, STATUS_LED, SPEED };

enum Buttons { HV_TOGGLE, RTD_TOGGLE };

enum AnalogInputs { THROTTLE_VOLTAGE };

class Vehicle {
 public:
  Vehicle();

  uint8_t state = LV_STARTUP;

  struct Dynamics {
    uint16_t throttleVoltage = 1;
    uint16_t speed;
    uint8_t maxSpeed = 60;
    uint8_t driveProfile = SAFE;
  };

  Dynamics dynamics;

  // LED values are 0x00 or 0xff to allow for bitwise not
  std::array<uint8_t, k_numLEDs> ledStates;
};

#endif  // VEHICLE_H
