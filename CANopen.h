#ifndef CAN_OPEN_H
#define CAN_OPEN_H

#include <FlexCAN.h>

class CANopen : public FlexCAN {
 public:
  CANopen(uint32_t id, uint32_t baud);
  virtual ~CANopen();

  bool sendMessage(const CAN_message_t& msg);
  bool recvMessage(CAN_message_t& msg);

  static constexpr uint32_t k_canLED = 13;
};

#endif // CAN_OPEN_H
