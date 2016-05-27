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
#include "CircularBuffer.h"

void _100msISR();
void _20msISR();
void _3msISR();

void canTx();
void canRx();
void canHeartbeat();
void updateThrottleTPDO(uint16_t throttleVoltage);

static CANopen* g_canBus = nullptr;
// when messages are enqueued to g_canTxQueue, they are printed over serial
static CircularBuffer<CAN_message_t> g_canTxQueue(10);
static CircularBuffer<CAN_message_t> g_canTxLogsQueue(10);
// when messages are dequeued from g_canRxQueue, they are printed over serial
static CircularBuffer<CAN_message_t> g_canRxQueue(10);

// TODO: refactor so that
//  - there is a standardized way of dealing with received packets outside of ISRs (print, etc.)
//  - there is some way to print transmited packets without losing them from being
//    removed from queue
//  - a brute force method is to make a g_canTxLogsQueue that has items enqueued to it after
//    they have been successfully transmited over CAN
//
//  ^ implemented it


int main() {
  const std::array<uint8_t, k_numLEDs> g_LedPins{2, 3, 4, 5};
  const std::array<uint8_t, k_numButtons> g_ButtonPins{7, 8};

  constexpr uint8_t k_torqueInput = A9;

  Serial.begin(115200);

  // Init LEDs
  for (auto& ledPin : g_LedPins) {
    pinMode(ledPin, HIGH);
  }
  // Init buttons
  for (auto& buttonPin : g_ButtonPins) {
    pinMode(buttonPin, INPUT);
  }

  Vehicle vehicle;

  constexpr uint32_t k_ID = 0x680;
  constexpr uint32_t k_baudRate = 250000;
  g_canBus = new CANopen(k_ID, k_baudRate);
  // g_txMsg.id = 0x003; // id of node on CAN bus

  IntervalTimer _100msinterrupt;
  _100msinterrupt.begin(_100msISR, 100000);

  IntervalTimer _20msinterrupt;
  _20msinterrupt.begin(_20msISR, 20000);

  IntervalTimer _3msInterrupt;
  _3msInterrupt.begin(_3msISR, 3000);

  while (1) {
    // Service global flags
    cli();

    // temporarily ugly
    CAN_message_t queueMsg;
    // print all received messages
    queueMsg = g_canRxQueue.PopFront();
    while (queueMsg) {
      // print
      g_canBus->printRx(queueMsg);
      // dequeue another message
      queueMsg = g_canRxQueue.PopFront();
    }
    // print all sent messages
    queueMsg = g_canTxLogsQueue.PopFront();
    while (queueMsg) {
      // print
      g_canBus->printTx(queueMsg);
      // dequeue another message
      queueMsg = g_canTxLogsQueue.PopFront();
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
        if (digitalReadFast(g_ButtonPins[HV_TOGGLE]) == LOW) {
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
        if (digitalReadFast(g_ButtonPins[RTD_TOGGLE]) == LOW) {
          vehicle.state = RTD_STARTUP;
        } else if (digitalReadFast(g_ButtonPins[HV_TOGGLE]) == LOW) {
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
        if (digitalReadFast(g_ButtonPins[RTD_TOGGLE]) == LOW) {
          // Start moving back to HV_ACTIVE
          vehicle.ledStates[SPEED] = LED_OFF;
          vehicle.dynamics.torque = 50;
          vehicle.state = RTD_SHUTDOWN;
        }
        break;
    }
  }
}

void _100msISR() {
  // enqueue heartbeat message to g_canTxQueue
  canHeartbeat();
}

void _20msISR() {
  // dequeue and transmit all messages in g_canTxQueue
  canTx();
}

void _3msISR() {
  // enqueue to g_canRxQueue, any received messages
  canRx();
}

// transmit all enqueued messages, in g_canTxQueue, of type CAN_message_t
// enqueue them onto the transmit logs queue after so that they can be printed
void canTx() {
  static CAN_message_t queueMsg;

  queueMsg = g_canTxQueue.PopFront();
  while (queueMsg) {
    // write message
    g_canBus->sendMessage(queueMsg);
    // enqueue them onto the logs queue
    g_canTxLogsQueue.PushBack(queueMsg);
    // dequeue new message
    queueMsg = g_canTxQueue.PopFront();
  }
}

// enqueue any messages apearing on the CAN bus
void canRx() {
  static CAN_message_t rxMsgTmp;
  while (g_canBus->recvMessage(rxMsgTmp)) {
    g_canRxQueue.PushBack(rxMsgTmp);
  }
}

// Writes the node's heartbeat to the CAN bus every 1s
void canHeartbeat() {
  // push heartbeat message to g_canTxQueue
  static uint8_t heartbeatCount = 0;
  // heartbeat message formatted with: COB-ID=0x001, len=2
  static CAN_message_t heartbeatMsg = {0x003,0,2,0,[0,0,0,0,0,0,0,0]};

  // enqueue a heartbeat message to be written to the CAN bus every 1s, (100ms * 10 = 1s)
  if (heartbeatCount == 0) {
    // TODO: add this statically into the initialization of heartbeatMsg
    // populate payload (only once)
    for (uint32_t i = 0; i < 2; ++i) {
      // set in message buff, each byte of the message, from least to most significant
      heartbeatMsg.buf[i] = (cobid_statusHeartbeat >> ((1 - i) * 8)) & 0xff;
    }
  } else if (heartbeatCount >= 10) {
    g_canTxQueue.PushBack(heartbeatMsg);
    heartbeatCount = 1;
  }
  ++heartbeatCount;
}

// From the perspective of the Primary Teensy..TPDO 5 maps to RPDO 5 on Master
void updateThrottleTPDO(uint16_t throttleVoltage) {
  // throttle message formate with: COB-ID=0x241, len=7
  static CAN_message_t throttleMsg = {cobid_TPDO5,0,7,0,[0,0,0,0,0,0,0,0]};

  // insert new throttle voltage value
  throttleMsg.buf[0] = (throttleVoltage >> 8) & 0xff; // MSB byte
  throttleMsg.buf[1] = (throttleVoltage) & 0xff; // LSB byte

  // enqueue the new value to be written to CAN bus
  g_canTxQueue.pushBack(throttleVoltage);
}
