#include "instrumentation.h"

#ifdef STACKFORCE_INSTRUMENTATION
#include <Arduino.h>
#include "low_level_command.h"

namespace {
struct Snapshot {
  CommandContext command;
  float servo[8] = {0};
  float wheel[4] = {0};
  float pitch = 0, roll = 0, yaw = 0;
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;
  float m0_feedback = 0, m1_feedback = 0;
  uint8_t servo_mask = 0;
  uint8_t wheel_mask = 0;
  bool command_seen = false;
  bool imu_seen = false;
  bool feedback_seen = false;
  uint32_t last_emit_ms = 0;
  uint64_t seq = 0;
} snapshot;

void emitSnapshot() {
  if (!Serial || Serial.availableForWrite() < 256) return;
  const uint64_t timestamp_us = micros();
  Serial.printf(
      "{\"record_type\":\"SAMPLE\",\"timestamp_us\":%llu,\"seq\":%llu,"
      "\"source\":\"%s\",\"ingress\":\"%s\","
      "\"command\":{\"forward\":%.3f,\"steering\":%.3f,\"height\":%.3f,\"roll\":%.3f,\"wheel_throttle\":%.3f,"
      "\"control_mode\":%u,\"motion_mode\":%u,\"steady_state\":%u},"
      "\"servo_targets_deg\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
      "\"wheel_targets_raw\":[%.3f,%.3f,%.3f,%.3f],"
      "\"imu\":{\"orientation_deg\":[%.3f,%.3f,%.3f],\"gyro_deg_s\":[%.3f,%.3f,%.3f]},"
      "\"wheel_feedback_raw\":[%.3f,%.3f]}\n",
      (unsigned long long)timestamp_us, (unsigned long long)snapshot.seq++,
      snapshot.command.source, snapshot.command.ingress,
      snapshot.command.forward, snapshot.command.steering, snapshot.command.height, snapshot.command.roll, snapshot.command.wheel_throttle,
      snapshot.command.control_mode, snapshot.command.motion_mode, snapshot.command.steady_state,
      snapshot.servo[0], snapshot.servo[1], snapshot.servo[2], snapshot.servo[3],
      snapshot.servo[4], snapshot.servo[5], snapshot.servo[6], snapshot.servo[7],
      snapshot.wheel[0], snapshot.wheel[1], snapshot.wheel[2], snapshot.wheel[3],
      snapshot.pitch, snapshot.roll, snapshot.yaw, snapshot.gyro_x, snapshot.gyro_y, snapshot.gyro_z,
      snapshot.m0_feedback, snapshot.m1_feedback);
}
}

void instrumentationBegin() {
  if (!Serial || Serial.availableForWrite() < 128) return;
  Serial.printf("{\"record_type\":\"CONFIG\",\"timestamp_us\":%lu,\"seq\":%llu,\"source\":\"%s\",\"ingress\":\"%s\",\"firmware_version\":\"%s\",\"robot_revision\":\"%s\"}\n",
                (unsigned long)micros(), (unsigned long long)snapshot.seq++,
                snapshot.command.source, snapshot.command.ingress,
                STACKFORCE_FIRMWARE_VERSION, STACKFORCE_ROBOT_REVISION);
}

void instrumentationObserveCommand(const CommandContext &context) { snapshot.command = context; snapshot.command_seen = true; }
void instrumentationObserveServo(uint8_t channel, float target_deg) { if (channel >= 1 && channel <= 8) snapshot.servo[channel - 1] = target_deg; }
void instrumentationObserveWheel(const char *device, uint8_t channel, float target_raw) {
  if (device && device[0] == 'R' && channel < 2) snapshot.wheel[channel] = target_raw;
  else if (device && device[0] == 'F' && channel >= 1 && channel <= 2) snapshot.wheel[2 + channel - 1] = target_raw;
}
void instrumentationObserveImu(float pitch, float roll, float yaw, float gyro_x, float gyro_y, float gyro_z) {
  snapshot.pitch = pitch; snapshot.roll = roll; snapshot.yaw = yaw;
  snapshot.gyro_x = gyro_x; snapshot.gyro_y = gyro_y; snapshot.gyro_z = gyro_z; snapshot.imu_seen = true;
}
void instrumentationObserveWheelFeedback(float m0_raw, float m1_raw) { snapshot.m0_feedback = m0_raw; snapshot.m1_feedback = m1_raw; snapshot.feedback_seen = true; }
void instrumentationService() {
  if (millis() - snapshot.last_emit_ms >= 50 && snapshot.command_seen && snapshot.imu_seen && snapshot.feedback_seen) {
    snapshot.last_emit_ms = millis();
    emitSnapshot();
  }
}

#else
void instrumentationBegin() {}
void instrumentationObserveCommand(const CommandContext &) {}
void instrumentationObserveServo(uint8_t, float) {}
void instrumentationObserveWheel(const char *, uint8_t, float) {}
void instrumentationObserveImu(float, float, float, float, float, float) {}
void instrumentationObserveWheelFeedback(float, float) {}
void instrumentationService() {}
#endif
