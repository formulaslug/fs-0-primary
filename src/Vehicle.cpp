#include "Vehicle.h"

Vehicle::Vehicle() {
  for (auto& led : ledStates) {
    led = LED_OFF;
  }
  ledStates[STATUS_LED] = ON;
}
