#ifndef STACKFORCE_HIGH_LEVEL_COMMAND_H
#define STACKFORCE_HIGH_LEVEL_COMMAND_H

#include <stdint.h>
#include "command_sources.h"

// Explicit representation of the existing mapped native locomotion intent.
struct HighLevelCommand {
  const char *source = command_sources::ActiveRegistration::source;
  const char *ingress = command_sources::ActiveRegistration::ingress;
  uint32_t timestamp_us = 0;
  float forward = 0;
  float steering = 0;
  float height = 0;
  float roll = 0;
  float wheel_throttle = 0;
  uint8_t control_mode = 0;
  uint8_t motion_mode = 0;
  uint8_t steady_state = 0;
};

using CommandContext = HighLevelCommand;

#endif
