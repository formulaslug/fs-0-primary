#ifndef VEHICLE_H
#define VEHICLE_H

#include <cstdint>
#include <array>

constexpr uint8_t NUM_LEDS = 5;
constexpr uint8_t NUM_BUTTONS = 2;
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

class Vehicle {
 public:
  Vehicle();

  uint8_t state = LV_STARTUP;

  struct Dynamics {
    uint16_t torque = 10;
    uint16_t speed;
    uint8_t topSpeed = 60;
  };

  Dynamics dynamics;

  // LED values are 0x00 or 0xff to allow for bitwise not
  std::array<uint8_t, NUM_LEDS> ledStates;
};

#endif // VEHICLE_H
