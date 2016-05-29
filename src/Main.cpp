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
void updateThrottleTPDO(uint16_t throttleVoltage, uint8_t forwardSwitch);

static CANopen* g_canBus = nullptr;
// when messages are enqueued to g_canTxQueue, they are printed over serial
static CircularBuffer<CAN_message_t> g_canTxQueue(10);
static CircularBuffer<CAN_message_t> g_canTxLogsQueue(10);
// when messages are dequeued from g_canRxQueue, they are printed over serial
static CircularBuffer<CAN_message_t> g_canRxQueue(10);

// global vehicle so that properties can be accessed from within ISRs
static Vehicle g_vehicle;

int main() {
  const std::array<uint8_t, k_numButtons> buttonPins{7, 8};
  const std::array<uint8_t, 1> analogInputPins{9};

  Serial.begin(115200);

  // Init buttons
  for (auto& buttonPin : buttonPins) {
    pinMode(buttonPin, INPUT);
  }
  // Init digital inputs (ADC)
  for (auto& inputPin : analogInputPins) {
    pinMode(inputPin, INPUT);
  }

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
    while (queueMsg.id) {
      // print
      g_canBus->printRx(queueMsg);
      // dequeue another message
      queueMsg = g_canRxQueue.PopFront();
    }
    // print all sent messages
    queueMsg = g_canTxLogsQueue.PopFront();
    while (queueMsg.id) {
      // print
      g_canBus->printTx(queueMsg);
      // dequeue another message
      queueMsg = g_canTxLogsQueue.PopFront();
    }
    sei();

    // (TEMPORARY) Update all analog inputs readings. In production, this will go in the fsm
    // TODO: clean this input and give a leeway of 3 or 4 before setting the new value
    g_vehicle.dynamics.throttleVoltage = analogRead(analogInputPins[THROTTLE_VOLTAGE]);

    // Vehicle's main state machine (FSM)
    switch (g_vehicle.state) {
      case LV_STARTUP:
        // Perform LV_STARTUP functions HERE
        g_vehicle.state = LV_ACTIVE;
        break;
      case LV_ACTIVE:
        // Set LED feedback
        g_vehicle.ledStates[BLUE] = LED_ON;
        g_vehicle.ledStates[YELLOW] = LED_OFF;
        g_vehicle.ledStates[RED] = LED_OFF;

        // Wait to move to HV_STARTUP
        if (digitalReadFast(buttonPins[HV_TOGGLE]) == LOW) {
          g_vehicle.state = HV_STARTUP;
        }
        break;
      case HV_SHUTDOWN:
        // Perform HV_SHUTDOWN functions HERE

        // Transition to LV_ACTIVE
        g_vehicle.state = LV_ACTIVE;
        break;
      case HV_STARTUP:
        // Perform LV_STARTUP functions HERE

        g_vehicle.state = HV_ACTIVE;
        break;
      case HV_ACTIVE:
        // Set LED feedback
        g_vehicle.ledStates[BLUE] = LED_ON;
        g_vehicle.ledStates[YELLOW] = LED_ON;
        g_vehicle.ledStates[RED] = LED_OFF;

        // Wait to move to RTD_STARTUP until user input
        if (digitalReadFast(buttonPins[RTD_TOGGLE]) == LOW) {
          g_vehicle.state = RTD_STARTUP;
        } else if (digitalReadFast(buttonPins[HV_TOGGLE]) == LOW) {
          // Or move back to LV active
          g_vehicle.state = HV_SHUTDOWN;
        }
        break;
      case RTD_SHUTDOWN:
        // Perform HV_SHUTDOWN functions HERE

        g_vehicle.state = HV_ACTIVE;
        break;
      case RTD_STARTUP:
        // Perform LV_STARTUP functions HERE

        // Show entire system is hot
        g_vehicle.ledStates[BLUE] = LED_ON;
        g_vehicle.ledStates[YELLOW] = LED_ON;
        g_vehicle.ledStates[RED] = LED_ON;

        g_vehicle.state = RTD_ACTIVE;
        break;
      case RTD_ACTIVE:
        // update current throttle voltage
        g_vehicle.dynamics.throttleVoltage = analogRead(analogInputPins[THROTTLE_VOLTAGE]);

        // Show speed
        g_vehicle.ledStates[SPEED] = ~g_vehicle.ledStates[SPEED];

        // Wait to transition back
        if (digitalReadFast(buttonPins[RTD_TOGGLE]) == LOW) {
          // Start moving back to HV_ACTIVE
          g_vehicle.ledStates[SPEED] = LED_OFF;
          g_vehicle.dynamics.throttleVoltage = 1;
          g_vehicle.state = RTD_SHUTDOWN;
        }
        break;
    }
  }
}

/**
 * @desc Performs all periodic, low frequency tasks
 */
void _100msISR() {
  // enqueue heartbeat message to g_canTxQueue
  canHeartbeat();
  // enqueue throttle voltage periodically as well
  updateThrottleTPDO(g_vehicle.dynamics.throttleVoltage, 1);
}

/**
 * @desc Processes and transmits all messages in g_canTxQueue
 */
void _20msISR() {
  canTx();
}

/**
 * @desc Processes all received CAN messages into g_canRxQueue
 */
void _3msISR() {
  canRx();
}

/**
 * @desc Transmits all enqueued messages, in g_canTxQueue, of type CAN_message_t. Enqueue
 *       them onto the transmit logs queue after so that they can be printed
 */
void canTx() {
  while (g_canTxQueue.NumElems() > 0) {
    // write message
    g_canBus->sendMessage(g_canTxQueue[0]);
    // enqueue them onto the logs queue
    g_canTxLogsQueue.PushBack(g_canTxQueue[0]);
    // dequeue new message
    g_canTxQueue.PopFront();
  }
}

/**
 * @desc Enqueue any messages apearing on the CAN bus
 */
void canRx() {
  static CAN_message_t rxMsgTmp;
  while (g_canBus->recvMessage(rxMsgTmp)) {
    g_canRxQueue.PushBack(rxMsgTmp);
  }
}

/**
 * @desc Writes the node's heartbeat to the CAN bus every 1s
 */
void canHeartbeat() {
  // push heartbeat message to g_canTxQueue
  static uint8_t heartbeatCount = 0;
  // static uint8_t heartbeatMsgPayload[8] = {0,0,0,0,0,0,0,0};
  // heartbeat message formatted with: COB-ID=0x001, len=2
  static CAN_message_t heartbeatMsg = {
    cobid_node3Heartbeat, 0, 2, 0, {0, 0, 0, 0, 0, 0, 0, 0}
  };
  // static CAN_message_t heartbeatMsg = {0x003,0,2,0,heartbeatMsgPayload};

  // enqueue a heartbeat message to be written to the CAN bus every 1s, (100ms * 10 = 1s)
  if (heartbeatCount == 0) {
    // TODO: add this statically into the initialization of heartbeatMsg
    // populate payload (only once)
    for (uint32_t i = 0; i < 2; ++i) {
      // set in message buff, each byte of the message, from least to most significant
      heartbeatMsg.buf[i] = (payload_heartbeat >> ((1 - i) * 8)) & 0xff;
    }
  } else if (heartbeatCount >= 10) {
    g_canTxQueue.PushBack(heartbeatMsg);
    heartbeatCount = 1;
  }
  ++heartbeatCount;
}

/**
 * @desc Writes (queues) all teensy<->teensy specific information to the CAN bus
 */
void canTeensy2Teensy() {
}

/**
 * @desc From the perspective of the Primary Teensy..TPDO 5 maps to RPDO 5 on Master
 * @param throttleVoltage The current, cleaned throttle voltage to be sent to Master
 * @param forwardSwitch A boolean corresponding to moving forward (must be 1 bit)
 */
void updateThrottleTPDO(uint16_t throttleVoltage, uint8_t forwardSwitch) {
  // throttle message formate with: COB-ID=0x241, len=7
  // static uint8_t throttleMsgPayload[8] = {0,0,0,0,0,0,0,0};
  static CAN_message_t throttleMsg = {
    cobid_TPDO5, 0, 7, 0, {0, 0, 0, 0, 0, 0, 0, 0}
  };

  // insert new throttle voltage value
  throttleMsg.buf[0] = (throttleVoltage >> 8) & 0xff; // MSB
  throttleMsg.buf[1] = (throttleVoltage) & 0xff; // LSB

  // insert new forward switch value
  throttleMsg.buf[6] = (forwardSwitch & 0x1) << 7; // sets byte as: on=0x80, off=0x00

  // enqueue the new value to be written to CAN bus
  g_canTxQueue.PushBack(throttleMsg);
}
