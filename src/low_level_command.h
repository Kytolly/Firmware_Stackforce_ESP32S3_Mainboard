#ifndef STACKFORCE_LOW_LEVEL_COMMAND_H
#define STACKFORCE_LOW_LEVEL_COMMAND_H

#include <stdint.h>

// Explicit representation of the native controller's actuator boundary.
// It does not replace the existing Servo, BLDC, or CAN APIs.
struct LowLevelCommand {
  LowLevelCommand(const char *actuator_value, const char *device_value,
                  uint8_t channel_value, float target_value,
                  const char *unit_value, uint32_t timestamp_value)
      : actuator(actuator_value), device(device_value), channel(channel_value),
        target(target_value), unit(unit_value), timestamp_us(timestamp_value) {}

  const char *actuator;
  const char *device;
  uint8_t channel;
  float target;
  const char *unit;
  uint32_t timestamp_us;
};

#endif
