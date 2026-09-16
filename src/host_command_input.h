#ifndef STACKFORCE_HOST_COMMAND_INPUT_H
#define STACKFORCE_HOST_COMMAND_INPUT_H

#include <Arduino.h>
#include "command_input.h"

class HostCommandInput final : public CommandInputPlugin {
public:
  const char *source() const override;
  const char *ingress() const override;
  bool enabled() const override;
  HighLevelCommand context(float forward, float steering, float height,
                           float roll, float wheel_throttle, uint8_t control_mode,
                           uint8_t motion_mode, uint8_t steady_state,
                           uint32_t timestamp_us) const override;
  bool poll(HighLevelCommand &command);

private:
  String rx_;
};

#endif
