/* @desc Primary control system for UCSC's FSAE Electric Vehicle
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
inline void loadBuf(uint8_t * src, char * dest, uint32_t len);

static CANopen* g_canBus = nullptr;
static CAN_message_t g_txMsg;
static CAN_message_t g_rxMsg;
static bool g_msgSent = false;
static bool g_msgRecv = false;

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

  IntervalTimer _20msInterrupt;
  _20msInterrupt.begin(_20msISR, 20000);

  IntervalTimer _3msInterrupt;
  _3msInterrupt.begin(_3msISR, 3000);

  while (1) {
    // service global flags
    if (g_msgSent) {
      // prepare message code, output buffer
      char buf[9] = "00000000";
      loadBuf(g_txMsg.buf, buf, 8);
      // print
      Serial.print("[EVENT]: CAN message transmitted. >>");
      Serial.println(buf);
      // clear flag
      g_msgSent = false;
    }
    if (g_msgRecv) {
      // prepare message code, output buffer
      char buf[9] = "00000000";
      loadBuf(g_rxMsg.buf, buf, 8);
      // print
      Serial.print("[EVENT]: CAN message received. >>");
      Serial.println(buf);
      // clear flag
      g_msgRecv = false;
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
  static uint8_t count = 0;
  ++count;

  g_txMsg.len = 8;
  g_txMsg.id = 0x222;

  // write a heartbeat to the CAN bus every 1s
  if (count >= 50) {
    // define msg code
    for (uint32_t i = 0; i < 8; ++i) {
      // set in msg buff, the 7-ith bit of the status code
      g_txMsg.buf[7-i] = '0' + ((k_statusHeartbeat >> i) & 0x1);
    }
    // write to bus
    g_canBus->sendMessage(g_txMsg);
    // reset count
    count = 0;
    // set flag
    g_msgSent = true;
  }
}

void canRx() {
  while (g_canBus->recvMessage(g_rxMsg)) {
    g_msgRecv = true;
  }
}

// Cleans up serial printing of can messages for 8-bit message codes
// loads every bit of src into every char of dest for a length of len
inline void loadBuf(uint8_t * src, char * dest, uint32_t len) {
  cli(); // disable interrupts
  for (uint32_t i = 0; i < len; i++) {
    dest[i] = src[i]; // set every char in the buffer
  }
  sei(); // enable interrupts
}
