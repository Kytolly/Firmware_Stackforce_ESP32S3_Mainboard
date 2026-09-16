#ifndef SF_CAN_H_
#define SF_CAN_H_
#include <Arduino.h>
#include "driver/twai.h"


#define CAN_RX 41
#define CAN_TX 35

// #define CAN_RX 10
// #define CAN_TX 9

class SF_CAN {
public:
  uint32_t deviceID;

  uint32_t rec_id;
  uint8_t rec_buf[8];

  SF_CAN() {
  }

  void init(int TX, int RX);
  void sendMsg(uint32_t* id, uint8_t* buf);
  void setMode(uint8_t MODE);
  void receiveMsg(uint8_t* buf);
  void setDeviceID(uint32_t deviceId);
  void listenAllMsg(uint32_t printRateMS);
private:
  TaskHandle_t taskHandle;
  twai_message_t r_message;
  twai_message_t t_message;
  int _sendState;

  unsigned long previousMillis;
  unsigned long currentMillis;
};
#endif
