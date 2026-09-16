#include "SF_CAN.h"

/**
  @ 函数名称: init
  * 函数功能: CAN引脚设置 ; 默认速率为1Mbit
  * 函数返回: 无
*/
void SF_CAN::init(int TX, int RX) {
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX, (gpio_num_t)RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    if (twai_start() == ESP_OK) {
      if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) {
        Serial.println("CAN初始化失败:CAN_ERR 1");
      }
    } else {
      Serial.println("CAN初始化失败:CAN_ERR 2");
    }
    Serial.println("CAN初始化成功");
  } else {
    Serial.println("CAN初始化失败:CAN_ERR 3");
  }
}

/**
  @ 函数名称: setMode
  * 函数功能: 设置发送ID的帧格式: 0--CAN标准帧（11位）; 1--CAN拓展帧（29位）
  * 函数返回: 无
*/
void SF_CAN::setMode(uint8_t MODE) {
  t_message.flags = MODE;
}


/**
  @ 函数名称: sendMsg
  * 函数功能: 发送ID包 + 8个字节的数据包
  * 函数返回: 无
*/
void SF_CAN::sendMsg(uint32_t* id, uint8_t* buf) {

  t_message.identifier = id[0];
  t_message.data_length_code = 8;
  for (int i = 0; i < 8; i++) {
    t_message.data[i] = buf[i];
  }
  _sendState = twai_transmit(&t_message, pdMS_TO_TICKS(1000));
  if (_sendState != ESP_OK) {
    Serial.printf("CAN数据包发送失败 \n");
  }
}

/**
  @ 函数名称: receiveMsg
  * 函数功能: 判断是否是Device ID,接收8个字节的数据包
  * 函数返回: 无
  * 测试备注：pdMS_TO_TICKS（）这个会阻塞，最好放在线程里接收。目前传参为0还可以
*/
void SF_CAN::receiveMsg(uint8_t* buf) {

  if (twai_receive(&r_message, pdMS_TO_TICKS(0)) == ESP_OK) {
    // Serial.print(r_message.identifier, HEX);
    rec_id = r_message.identifier;
    if (rec_id == deviceID) {
      if (!(r_message.rtr)) {
        // Serial.println("OK");
        for (int i = 0; i < r_message.data_length_code; i++) {
          // Serial.printf("0x%02x", r_message.data[i]);
          buf[i] = r_message.data[i];
          rec_buf[i] = buf[i];
        }
        // Serial.println();
      }
    }
  }
}

/**
  @ 函数名称: listenAllMsg
  * 函数功能: 打印总线上的数据
  * 函数返回: 无
*/
void SF_CAN::listenAllMsg(uint32_t printRateMS) {
  currentMillis = millis();
  if (currentMillis - previousMillis >= printRateMS) {
    previousMillis = currentMillis;
    Serial.printf("0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x \n ", r_message.identifier,
                  r_message.data[0], r_message.data[1], r_message.data[2], r_message.data[3], r_message.data[4], r_message.data[5], r_message.data[6], r_message.data[7]);
  }
}

/**
  @ 函数名称: setDeviceID
  * 函数功能: 设置本设备的ID
  * 函数返回: 无
*/
void SF_CAN::setDeviceID(uint32_t deviceId) {
  deviceID = deviceId;
}