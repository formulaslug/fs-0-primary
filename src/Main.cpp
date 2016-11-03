// Copyright (c) Formula Slug 2016. All Rights Reserved.

/* @desc Primary control system for UCSC's FSAE Electric Vehicle
 *       CAN nodeID=3
 * @dict LV = Low Voltage System, HV = High Voltage System, RTD = Ready-To-Drive
 */

// TODO: Need some button debounce

#include <stdint.h>

#include <array>
#include <memory>

#include <IntervalTimer.h>

#include "Vehicle.h"
#include "fs-0-core/CANopen.h"
#include "fs-0-core/CANopenPDO.h"
#include "fs-0-core/InterruptMutex.h"
#include "fs-0-core/make_unique.h"

// timer interrupt handlers
void _1sISR();
void _100msISR();
void _20msISR();
void _3msISR();

// contains and controls all CAN related functions
static std::unique_ptr<CANopen> g_canBus;

// global vehicle so that properties can be accessed from within ISRs
static Vehicle g_vehicle;

/**
 * If the range is ordered as (min, max), minimum input values map to 0.
 * If the range is ordered as (max, min), maximum input values map to 1.
 *
 * @param range Range of input value
 * @param input Value within range
 * @return Normalized value between 0 and 1 inclusive
 */
double normalize(const std::array<double, 2> range, double input) {
  return (input - range[0]) / (range[1] - range[0]);
}

int main() {
  const std::array<uint8_t, kNumButtons> buttonPins{7, 8};
  const std::array<uint8_t, 1> analogInputPins{9};

  Serial.begin(115200);

  // Configure input pins
  pinMode(A1, INPUT);
  pinMode(A3, INPUT);
  pinMode(A0, INPUT);
  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);

  // Turn off startup sound
  pinMode(20, OUTPUT);
  digitalWriteFast(20, LOW);

  // Turn off brake light
  pinMode(21, OUTPUT);
  digitalWriteFast(21, LOW);

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
  g_canBus = std::make_unique<CANopen>(kID, kBaudRate);

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
    /*g_vehicle.dynamics.throttleVoltage =
        analogRead(analogInputPins[kThrottleVoltage]);*/

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
  const HeartbeatMessage heartbeatMessage(kCobid_node3Heartbeat);
  g_canBus->queueTxMessage(heartbeatMessage);
}

/**
 * @desc Performs periodic tasks every 1/10 second
 */
void _100msISR() {
  double leftThrottle = normalize({500, 750}, analogRead(A0));
  double rightThrottle = normalize({550, 295}, analogRead(A3));
  double throttle = (leftThrottle + rightThrottle) / 2;
  bool driveButton = digitalReadFast(23);

  // enqueue throttle voltage periodically as well
  const ThrottleMessage throttleMessage(65536 * throttle, driveButton);
  g_canBus->queueTxMessage(throttleMessage);
}

/**
 * @desc Processes and transmits all messages in g_canTxQueue
 */
void _20msISR() { g_canBus->processTxMessages(); }

/**
 * @desc Processes all received CAN messages into g_canRxQueue
 */
void _3msISR() { g_canBus->processRxMessages(); }
