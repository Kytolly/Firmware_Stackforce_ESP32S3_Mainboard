#include "host_command_input.h"
#include "command_sources.h"
#include "host_command_protocol.h"

const char *HostCommandInput::source() const { return command_sources::HostRegistration::source; }
const char *HostCommandInput::ingress() const { return command_sources::HostRegistration::ingress; }
bool HostCommandInput::enabled() const { return command_sources::HostRegistration::enabled; }

HighLevelCommand HostCommandInput::context(float forward, float steering, float height,
                                           float roll, float wheel_throttle, uint8_t control_mode,
                                           uint8_t motion_mode, uint8_t steady_state,
                                           uint32_t timestamp_us) const {
  HighLevelCommand result;
  result.source = source(); result.ingress = ingress(); result.timestamp_us = timestamp_us;
  result.forward = forward; result.steering = steering; result.height = height; result.roll = roll;
  result.wheel_throttle = wheel_throttle;
  result.control_mode = control_mode; result.motion_mode = motion_mode; result.steady_state = steady_state;
  return result;
}

bool HostCommandInput::poll(HighLevelCommand &command) {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n') {
      const HostDecodeStatus status = decodeHostCommand(rx_.c_str(), command);
      rx_ = "";
      if (status == HostDecodeStatus::OK) return true;
      continue;
    }
    if (c != '\r' && rx_.length() < 160) rx_ += c;
  }
  return false;
}
