#include "ppm_remote_input.h"
#include "command_sources.h"

const char *PpmRemoteInputPlugin::source() const { return command_sources::RemotePpmRegistration::source; }
const char *PpmRemoteInputPlugin::ingress() const { return command_sources::RemotePpmRegistration::ingress; }
bool PpmRemoteInputPlugin::enabled() const { return command_sources::RemotePpmRegistration::enabled; }

CommandContext PpmRemoteInputPlugin::context(float forward, float steering, float height,
                                             float roll, uint8_t control_mode,
                                             uint8_t motion_mode, uint8_t steady_state,
                                             uint32_t timestamp_us) const {
  CommandContext result;
  result.source = source();
  result.ingress = ingress();
  result.timestamp_us = timestamp_us;
  result.forward = forward;
  result.steering = steering;
  result.height = height;
  result.roll = roll;
  result.control_mode = control_mode;
  result.motion_mode = motion_mode;
  result.steady_state = steady_state;
  return result;
}
