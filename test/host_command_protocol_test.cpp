#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../src/host_command_protocol.h"

static void valid(const char *frame, float forward, float steering, float height,
                  float roll, float throttle, unsigned mode) {
  HighLevelCommand command;
  assert(decodeHostCommand(frame, command) == HostDecodeStatus::OK);
  assert(fabs(command.forward - forward) < 0.0001f);
  assert(fabs(command.steering - steering) < 0.0001f);
  assert(fabs(command.height - height) < 0.0001f);
  assert(fabs(command.roll - roll) < 0.0001f);
  assert(fabs(command.wheel_throttle - throttle) < 0.0001f);
  assert(command.control_mode == mode);
}

int main() {
  valid("HCMD,1,1,0,0,0,0,0,0,0,0", 0, 0, 0, 0, 0, 0);
  valid("HCMD,1,2,20,0,0,0,1,0,0,0", 20, 0, 0, 0, 1, 0);
  valid("HCMD,1,3,-20,0,0,0,1,0,0,0", -20, 0, 0, 0, 1, 0);
  valid("HCMD,1,4,0,10,0,0,1,0,0,0", 0, 10, 0, 0, 1, 0);
  valid("HCMD,1,5,0,-10,0,0,1,0,0,0", 0, -10, 0, 0, 1, 0);
  valid("HCMD,1,6,0,0,15,0,1,0,0,0", 0, 0, 15, 0, 1, 0);
  valid("HCMD,1,7,0,0,0,-12,1,0,0,0", 0, 0, 0, -12, 1, 0);
  valid("HCMD,1,8,0,0,0,0,1,1,1,1", 0, 0, 0, 0, 1, 1);
  HighLevelCommand command;
  assert(decodeHostCommand("bad", command) == HostDecodeStatus::MALFORMED);
  assert(decodeHostCommand("HCMD,2,1,0,0,0,0,0,0,0,0", command) == HostDecodeStatus::UNSUPPORTED_VERSION);
  assert(decodeHostCommand("HCMD,1,1,51,0,0,0,0,0,0,0", command) == HostDecodeStatus::OUT_OF_RANGE);
  assert(decodeHostCommand("HCMD,1,1,0,0", command) == HostDecodeStatus::MALFORMED);
  assert(decodeHostCommand("HCMD,1,9,1,2,3,4,0.5,1,0,1", command) == HostDecodeStatus::OK);
  assert(decodeHostCommand("HCMD,1,10,1,2,3,4,0.5,1,0,1", command) == HostDecodeStatus::OK);
  puts("host command protocol tests: PASS");
}
