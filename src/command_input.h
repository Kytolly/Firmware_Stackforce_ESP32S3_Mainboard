#ifndef STACKFORCE_COMMAND_INPUT_H
#define STACKFORCE_COMMAND_INPUT_H

#include <stdint.h>
#include "command_context.h"

class CommandInputPlugin {
public:
  virtual ~CommandInputPlugin() = default;
  virtual const char *source() const = 0;
  virtual const char *ingress() const = 0;
  virtual bool enabled() const = 0;
  virtual CommandContext context(float forward, float steering, float height,
                                 float roll, uint8_t control_mode,
                                 uint8_t motion_mode, uint8_t steady_state,
                                 uint32_t timestamp_us) const = 0;
};

// Architecture slot only. No HOST instance is registered or enabled yet.
class HostCommandInputPlugin {
public:
  virtual ~HostCommandInputPlugin() = default;
};

#endif
