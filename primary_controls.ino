/*
 * @desc Pimary control system for the UCSC's Formula Slug Electric FSAE Vehicle
 * @dict LV = Low Voltage System, HV = High Voltage System, RTD = Ready-To-Drive
 */

// TODO: Need some button debounce

#include <cstdint>
#include <array>

#include "CANopen.h"
#include "Timer.h"
#include "Vehicle.h"

int main() {
  const std::array<uint8_t, NUM_LEDS> gLedPins{2, 3, 4, 5};
  const std::array<uint8_t, NUM_BUTTONS> gButtonPins{7, 8};

  constexpr uint32_t k_ID = 0x680;
  constexpr uint32_t k_baudRate = 500000;
  constexpr uint8_t TORQUE_INPUT = A9;

  Serial.begin(9600);

  // Init LEDs
  for (auto& ledPin : gLedPins) {
    pinMode(ledPin, HIGH);
  }
  // Init buttons
  for (auto& buttonPin : gButtonPins) {
    pinMode(buttonPin, INPUT);
  }

  Vehicle vehicle;

  CANopen canBus(k_ID, k_baudRate);
  CAN_message_t txMsg;
  CAN_message_t rxMsg;

  Timer txTimer(100);
  Timer rxTimer(3);

  while (1) {
    txTimer.update();
    rxTimer.update();

    if (txTimer.isExpired()) {
      txMsg.len = 8;
      txMsg.id = 0x222;
      for (uint32_t i = 0; i < 8; i++) {
        txMsg.buf[i] = '0' + i;
      }

      digitalWriteFast(13, 1);
      for (uint32_t i = 0; i < 6; i++) {
        if (!canBus.write(txMsg)) {
          Serial.println("tx failed");
        }
        txMsg.buf[0]++;
      }
      digitalWriteFast(13, 0);
    }

    if (rxTimer.isExpired()) {
      while (canBus.read(rxMsg)) {
        Serial.print(rxMsg.id, HEX);
        for (uint32_t i = 0; i < 8; i++) {
          Serial.print(":");
          Serial.print(rxMsg.buf[i], HEX);
        }
        Serial.print("\r\n");
      }
    }

    // Vehicle's main state machine (FSM)
    switch (vehicle.state) {
      case LV_STARTUP:
        // Perform LV_STARTUP functions HERE

        vehicle.state = LV_ACTIVE;
        break;
      case LV_ACTIVE:
        // Set LED feedback
        vehicle.ledStates[BLUE] = LED_ON;
        vehicle.ledStates[YELLOW] = LED_OFF;
        vehicle.ledStates[RED] = LED_OFF;

        // Wait to move to HV_STARTUP
        if (digitalReadFast(gButtonPins[HV_TOGGLE]) == LOW) {
          vehicle.state = HV_STARTUP;
        }
        break;
      case HV_SHUTDOWN:
        // Perform HV_SHUTDOWN functions HERE

        // Transition to LV_ACTIVE
        vehicle.state = LV_ACTIVE;
        break;
      case HV_STARTUP:
        // Perform LV_STARTUP functions HERE

        vehicle.state = HV_ACTIVE;
        break;
      case HV_ACTIVE:
        // Set LED feedback
        vehicle.ledStates[BLUE] = LED_ON;
        vehicle.ledStates[YELLOW] = LED_ON;
        vehicle.ledStates[RED] = LED_OFF;

        // Wait to move to RTD_STARTUP until user input
        if (digitalReadFast(gButtonPins[RTD_TOGGLE]) == LOW) {
          vehicle.state = RTD_STARTUP;
        } else if (digitalReadFast(gButtonPins[HV_TOGGLE]) == LOW) {
          // Or move back to LV active
          vehicle.state = HV_SHUTDOWN;
        }
        break;
      case RTD_SHUTDOWN:
        // Perform HV_SHUTDOWN functions HERE

        vehicle.state = HV_ACTIVE;
        break;
      case RTD_STARTUP:
        // Perform LV_STARTUP functions HERE

        // Show entire system is hot
        vehicle.ledStates[BLUE] = LED_ON;
        vehicle.ledStates[YELLOW] = LED_ON;
        vehicle.ledStates[RED] = LED_ON;

        vehicle.state = RTD_ACTIVE;
        break;
      case RTD_ACTIVE:
        // Get speed
        vehicle.dynamics.torque = static_cast<int>(analogRead(TORQUE_INPUT) / 2);

        // Show speed
        vehicle.ledStates[SPEED] = ~vehicle.ledStates[SPEED];

        // Wait to transition back
        if (digitalReadFast(gButtonPins[RTD_TOGGLE]) == LOW) {
          // Start moving back to HV_ACTIVE
          vehicle.ledStates[SPEED] = LED_OFF;
          vehicle.dynamics.torque = 50;
          vehicle.state = RTD_SHUTDOWN;
        }
        break;
    }
  }
}
