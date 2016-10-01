// Copyright (c) Formula Slug 2016. All Rights Reserved.

/* @desc Primary control system for UCSC's FSAE Electric Vehicle
 *       CAN nodeID=3
 * @dict LV = Low Voltage System, HV = High Voltage System, RTD = Ready-To-Drive
 */

// TODO: Need some button debounce

#include <IntervalTimer.h>
#include <stdint.h>

#include <array>

#include "Vehicle.h"
#include "core_controls/CANopen.h"
#include "core_controls/InterruptMutex.h"

// timer interrupt handlers
void _1sISR();
void _100msISR();
void _20msISR();
void _3msISR();

// declarations of the can message packing functions
CAN_message_t canGetHeartbeat();
CAN_message_t canGetThrottleTPDO(uint16_t throttleVoltage,
                                 uint8_t forwardSwitch);
CAN_message_t canGetPrimary2Secondary();

// contains and controls all CAN related functions
static CANopen* g_canBus = nullptr;

// global vehicle so that properties can be accessed from within ISRs
static Vehicle g_vehicle;

int main() {
  const std::array<uint8_t, kNumButtons> buttonPins{7, 8};
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

  constexpr uint32_t kID = 0x680;
  constexpr uint32_t kBaudRate = 250000;
  g_canBus = new CANopen(kID, kBaudRate);

  IntervalTimer _1sInterrupt;
  _1sInterrupt.begin(_1sISR, 1000000);

  IntervalTimer _100msInterrupt;
  _100msInterrupt.begin(_100msISR, 100000);

  IntervalTimer _20msInterrupt;
  _20msInterrupt.begin(_20msISR, 20000);

  IntervalTimer _3msInterrupt;
  _3msInterrupt.begin(_3msISR, 3000);

  InterruptMutex interruptMut;

  while (1) {
    {
      std::lock_guard<InterruptMutex> lock(interruptMut);

      // print all transmitted messages
      g_canBus->printTxAll();
      // print all received messages
      g_canBus->printRxAll();
    }

    // (TEMPORARY) Update all analog inputs readings. In production, this will
    // go in the fsm
    // TODO: clean this input and give a leeway of 3 or 4 before setting the new
    // value
    g_vehicle.dynamics.throttleVoltage =
        analogRead(analogInputPins[kThrottleVoltage]);

    // Vehicle's main state machine (FSM)
    switch (g_vehicle.state) {
      case kLVStartup:
        // Perform kLVStartup functions HERE
        g_vehicle.state = kLVActive;
        break;
      case kLVActive:
        // Set LED feedback
        g_vehicle.ledStates[kBlue] = kLEDOn;
        g_vehicle.ledStates[kYellow] = kLEDOff;
        g_vehicle.ledStates[kRed] = kLEDOff;

        // Wait to move to kHVStartup
        if (digitalReadFast(buttonPins[kHVToggle]) == LOW) {
          g_vehicle.state = kHVStartup;
        }
        break;
      case kHVShutdown:
        // Perform kHVShutdown functions HERE

        // Transition to kLVActive
        g_vehicle.state = kLVActive;
        break;
      case kHVStartup:
        // Perform kLVStartup functions HERE

        g_vehicle.state = kHVActive;
        break;
      case kHVActive:
        // Set LED feedback
        g_vehicle.ledStates[kBlue] = kLEDOn;
        g_vehicle.ledStates[kYellow] = kLEDOn;
        g_vehicle.ledStates[kRed] = kLEDOff;

        // Wait to move to kRTDStartup until user input
        if (digitalReadFast(buttonPins[kRTDToggle]) == LOW) {
          g_vehicle.state = kRTDStartup;
        } else if (digitalReadFast(buttonPins[kHVToggle]) == LOW) {
          // Or move back to LV active
          g_vehicle.state = kHVShutdown;
        }
        break;
      case kRTDShutdown:
        // Perform kHVShutdown functions HERE

        g_vehicle.state = kHVActive;
        break;
      case kRTDStartup:
        // Perform kLVStartup functions HERE

        // Show entire system is hot
        g_vehicle.ledStates[kBlue] = kLEDOn;
        g_vehicle.ledStates[kYellow] = kLEDOn;
        g_vehicle.ledStates[kRed] = kLEDOn;

        g_vehicle.state = kRTDActive;
        break;
      case kRTDActive:
        // update current throttle voltage
        g_vehicle.dynamics.throttleVoltage =
            analogRead(analogInputPins[kThrottleVoltage]);

        // Show speed
        g_vehicle.ledStates[kSpeed] = ~g_vehicle.ledStates[kSpeed];

        // Wait to transition back
        if (digitalReadFast(buttonPins[kRTDToggle]) == LOW) {
          // Start moving back to HV_ACTIVE
          g_vehicle.ledStates[kSpeed] = kLEDOff;
          g_vehicle.dynamics.throttleVoltage = 1;
          g_vehicle.state = kRTDShutdown;
        }
        break;
    }
  }
}

/**
 * @desc Performs period tasks every second
 */
void _1sISR() {
  // enqueue heartbeat message to g_canTxQueue
  g_canBus->queueTxMessage(canGetHeartbeat());
}

/**
 * @desc Performs periodic tasks every 1/10 second
 */
void _100msISR() {
  // enqueue throttle voltage periodically as well
  g_canBus->queueTxMessage(
      canGetThrottleTPDO(g_vehicle.dynamics.throttleVoltage, 1));
  // enqueue primary-to-secondary message
  g_canBus->queueTxMessage(canGetPrimary2Secondary());
}

/**
 * @desc Processes and transmits all messages in g_canTxQueue
 */
void _20msISR() { g_canBus->processTxMessages(); }

/**
 * @desc Processes all received CAN messages into g_canRxQueue
 */
void _3msISR() { g_canBus->processRxMessages(); }

/**
 * @desc Writes the node's heartbeat to the CAN bus every 1s
 * @return The packaged message of type CAN_message_t
 */
CAN_message_t canGetHeartbeat() {
  static bool didInit = false;
  // heartbeat message formatted with: COB-ID=0x001, len=2
  static CAN_message_t heartbeatMessage = {
      kCobid_node3Heartbeat, 0, 2, 0, {0, 0, 0, 0, 0, 0, 0, 0}};

  // insert the heartbeat payload on the first call
  if (!didInit) {
    // TODO: add this statically into the initialization of heartbeatMessage
    // populate payload (only once)
    for (uint32_t i = 0; i < 2; ++i) {
      // set in message buff, each byte of the message, from least to most
      // significant
      heartbeatMessage.buf[i] = (kPayloadHeartbeat >> ((1 - i) * 8)) & 0xff;
    }
    didInit = true;
  }
  // return the packed/formatted message
  return heartbeatMessage;
}

/**
 * @desc Writes (queues) all primary-to-secondary teensy specific information to
 *       the CAN bus
 * @return The packaged message of type CAN_message_t
 */
CAN_message_t canGetPrimary2Secondary() {
  // payload format (MSB to LSB): state (matches fsm state enum), profile,
  // speed,
  //    throttleVoltage[1], throttleVoltage[0]
  static CAN_message_t p2sMessage = {// p2s=primary to secondary
                                     kCobid_p2s,
                                     0,
                                     5,
                                     0,
                                     {0, 0, 0, 0, 0, 0, 0, 0}};

  // insert current state
  p2sMessage.buf[0] = g_vehicle.state;
  // insert current profile
  p2sMessage.buf[1] = g_vehicle.dynamics.driveProfile;
  // insert current speed
  p2sMessage.buf[2] = g_vehicle.dynamics.speed;
  // insert current throttleVoltage
  p2sMessage.buf[3] = (g_vehicle.dynamics.throttleVoltage >> 8) & 0xff;  // MSB
  p2sMessage.buf[4] = (g_vehicle.dynamics.throttleVoltage) & 0xff;       // LSB

  // return the packed/formatted message
  return p2sMessage;
}

/**
 * @desc From the perspective of the Primary Teensy..TPDO 5 maps to RPDO 5 on
 *       Master
 * @param throttleVoltage The current, cleaned throttle voltage to be sent to
 *                        Master
 * @param forwardSwitch A boolean corresponding to moving forward (must be 1
 *                      bit)
 * @return The packaged message of type CAN_message_t
 */
CAN_message_t canGetThrottleTPDO(uint16_t throttleVoltage,
                                 uint8_t forwardSwitch) {
  // throttle message formate with: COB-ID=0x241, len=7
  // static uint8_t throttleMessagePayload[8] = {0,0,0,0,0,0,0,0};
  static CAN_message_t throttleMessage = {
      kCobid_TPDO5, 0, 7, 0, {0, 0, 0, 0, 0, 0, 0, 0}};

  // insert new throttle voltage value
  throttleMessage.buf[0] = (throttleVoltage >> 8) & 0xff;  // MSB
  throttleMessage.buf[1] = throttleVoltage & 0xff;         // LSB

  // insert new forward switch value
  throttleMessage.buf[6] = (forwardSwitch & 0x1)
                           << 7;  // sets byte as: on=0x80, off=0x00

  // enqueue the new value to be written to CAN bus
  return throttleMessage;
}
