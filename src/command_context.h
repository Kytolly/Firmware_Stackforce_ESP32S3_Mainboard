#ifndef STACKFORCE_COMMAND_CONTEXT_H
#define STACKFORCE_COMMAND_CONTEXT_H

#include <stdint.h>
#include "command_sources.h"

struct CommandContext {
  const char *source = command_sources::RemotePpmRegistration::source;
  const char *ingress = command_sources::RemotePpmRegistration::ingress;
  float forward = 0;
  float steering = 0;
  float height = 0;
  float roll = 0;
  uint8_t control_mode = 0;
  uint8_t motion_mode = 0;
  uint8_t steady_state = 0;
};

#endif
