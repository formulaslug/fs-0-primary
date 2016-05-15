/* @desc Primary control system for UCSC's FSAE Electric Vehicle
 *       CAN nodeID=3
 * @dict LV = Low Voltage System, HV = High Voltage System, RTD = Ready-To-Drive
 */

// TODO: Need some button debounce

#include <cstdint>
#include <array>

#include <IntervalTimer.h>
#include "core_controls/CANopen.h"
#include "Vehicle.h"

void _20msISR();
void _3msISR();

void canTx();
void canRx();

static CANopen* g_canBus = nullptr;
static CAN_message_t g_txMsg;
static CAN_message_t g_rxMsg;
static bool g_msgSent = false; // flag s.t. true=[message recently sent over can bus]
static bool g_msgRecv = false; // flag s.t. true=[message recently received over can bus]

int main() {
  const std::array<uint8_t, k_numLEDs> gLedPins{2, 3, 4, 5};
  const std::array<uint8_t, k_numButtons> gButtonPins{7, 8};

  constexpr uint8_t k_torqueInput = A9;

  Serial.begin(115200);

  // Init LEDs
  for (auto& ledPin : gLedPins) {
    pinMode(ledPin, HIGH);
  }
  // Init buttons
  for (auto& buttonPin : gButtonPins) {
    pinMode(buttonPin, INPUT);
  }

  Vehicle vehicle;

  constexpr uint32_t k_ID = 0x680;
  constexpr uint32_t k_baudRate = 250000;
  g_canBus = new CANopen(k_ID, k_baudRate);
  g_txMsg.id = 0x003; // id of node on CAN bus

  IntervalTimer _20msInterrupt;
  _20msInterrupt.begin(_20msISR, 20000);

  IntervalTimer _3msInterrupt;
  _3msInterrupt.begin(_3msISR, 3000);

  while (1) {
    // Service global flags
    cli();
    if (g_msgSent) {
      g_canBus->printTx(g_txMsg);

      // Clear flag
      g_msgSent = false;
    }
    if (g_msgRecv) {
      g_canBus->printRx(g_rxMsg);

      // Clear flag
      g_msgRecv = false;
    }
    sei();

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
        vehicle.dynamics.torque =
            static_cast<int>(analogRead(k_torqueInput) / 2);

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

void _20msISR() {
  canTx();
}

void _3msISR() {
  canRx();
}

void canTx() {
  static uint8_t heartbeatCount = 0;
  ++heartbeatCount;

  g_txMsg.len = 2; // max length message codes in bytes

  // write a heartbeat to the CAN bus every 1s, (20ms * 50 = 1s)
  if (heartbeatCount >= 50) {
    // define msg code
    for (uint32_t i = 0; i < 2; ++i) {
      // set in message buff, each byte of the message, from least to most significant
      g_txMsg.buf[i] = (k_statusHeartbeat >> ((1 - i) * 8)) & 0xff;
    }
    // write to bus
    g_canBus->sendMessage(g_txMsg);
    // reset count
    heartbeatCount = 0;
    // set flag
    g_msgSent = true;
  }
}

void canRx() {
  while (g_canBus->recvMessage(g_rxMsg)) {
    // set flag
    g_msgRecv = true;
  }
}
