#ifndef STACKFORCE_PPM_REMOTE_INPUT_H
#define STACKFORCE_PPM_REMOTE_INPUT_H

#include "command_input.h"

class PpmRemoteInputPlugin final : public CommandInputPlugin {
public:
  const char *source() const override;
  const char *ingress() const override;
  bool enabled() const override;
  HighLevelCommand context(float forward, float steering, float height,
                         float roll, float wheel_throttle, uint8_t control_mode, uint8_t motion_mode,
                         uint8_t steady_state, uint32_t timestamp_us) const override;
};

#endif
