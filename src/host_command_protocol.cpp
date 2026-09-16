#include "host_command_protocol.h"
#include <stdio.h>
#include "command_sources.h"

HostDecodeStatus decodeHostCommand(const char *frame, HighLevelCommand &command) {
  float forward, steering, height, roll, wheel_throttle;
  unsigned int control_mode, motion_mode, steady_state;
  unsigned long timestamp;
  int version;
  const int parsed = sscanf(frame, "HCMD,%d,%lu,%f,%f,%f,%f,%f,%u,%u,%u",
                            &version, &timestamp, &forward, &steering, &height, &roll, &wheel_throttle,
                            &control_mode, &motion_mode, &steady_state);
  if (parsed != 10) return HostDecodeStatus::MALFORMED;
  if (version != 1) return HostDecodeStatus::UNSUPPORTED_VERSION;
  if (forward < -41.667f || forward > 41.667f || steering < -41.667f || steering > 41.667f ||
      height < -25 || height > 25 || roll < -37.5f || roll > 37.5f ||
      wheel_throttle < 0 || wheel_throttle > 1 ||
      control_mode > 1 || motion_mode > 1 || steady_state > 1)
    return HostDecodeStatus::OUT_OF_RANGE;

  command.source = command_sources::HostRegistration::source;
  command.ingress = command_sources::HostRegistration::ingress;
  command.timestamp_us = (uint32_t)timestamp;
  command.forward = forward;
  command.steering = steering;
  command.height = height;
  command.roll = roll;
  command.wheel_throttle = wheel_throttle;
  command.control_mode = (uint8_t)control_mode;
  command.motion_mode = (uint8_t)motion_mode;
  command.steady_state = (uint8_t)steady_state;
  return HostDecodeStatus::OK;
}
