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

  setFilters({0x620, 0x002});
}

CANopen::~CANopen() {
  end();
}

void CANopen::setFilters(std::initializer_list<uint32_t> filters) {
  CAN_filter_t filter;
  filter.rtr = 0;
  filter.ext = 0;

  uint32_t i = 0;
  for (auto id : filters) {
    // Set remaining filters to last entry if the provided list is too short
    if (i < filters.size()) {
      filter.id = id;
      i++;
    }
    setFilter(filter, i);
  }
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
