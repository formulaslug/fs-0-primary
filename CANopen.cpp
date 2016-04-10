#include "CANopen.h"
#include <kinetis_flexcan.h>

CANopen::CANopen(uint32_t id, uint32_t baud) : FlexCAN(baud) {
  CAN_filter_t mask;
  mask.rtr = 0;
  mask.ext = 0;
  mask.id = id;
  begin(mask);

  pinMode(k_canLED, OUTPUT);
  digitalWrite(k_canLED, 0);

  CAN_filter_t filter;
  filter.ext = 0;
  filter.rtr = 0;

  filter.id = 0x620;
  setFilter(filter, 0);
  setFilter(filter, 1);
  setFilter(filter, 2);
  setFilter(filter, 3);

  filter.id = 0x002;
  setFilter(filter, 4);
  setFilter(filter, 5);
  setFilter(filter, 6);
  setFilter(filter, 7);
}

CANopen::~CANopen() {
  end();
}

bool CANopen::sendMessage(const CAN_message_t& msg) {
  if (write(msg)) {
    digitalWrite(k_canLED, 0);
    return true;
  } else {
    digitalWrite(k_canLED, 1);
    return false;
  }
}

bool CANopen::recvMessage(CAN_message_t& msg) {
  return read(msg);
}
