// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include "Vehicle.h"

Vehicle::Vehicle() {
  for (auto& led : ledStates) {
    led = LED_OFF;
  }
  ledStates[STATUS_LED] = ON;
}
