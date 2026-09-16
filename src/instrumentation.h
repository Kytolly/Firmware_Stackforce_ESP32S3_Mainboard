#ifndef STACKFORCE_INSTRUMENTATION_H
#define STACKFORCE_INSTRUMENTATION_H

#include <stdint.h>
#include "command_context.h"

void instrumentationBegin();
void instrumentationObserveCommand(const CommandContext &context);
void instrumentationObserveServo(uint8_t channel, float target_deg);
void instrumentationObserveWheel(const char *device, uint8_t channel, float target_raw);
void instrumentationObserveImu(float pitch, float roll, float yaw,
                               float gyro_x, float gyro_y, float gyro_z);
void instrumentationObserveWheelFeedback(float m0_raw, float m1_raw);
void instrumentationService();

#endif
