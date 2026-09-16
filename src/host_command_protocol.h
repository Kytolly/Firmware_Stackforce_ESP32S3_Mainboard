#ifndef STACKFORCE_HOST_COMMAND_PROTOCOL_H
#define STACKFORCE_HOST_COMMAND_PROTOCOL_H

#include "high_level_command.h"

enum class HostDecodeStatus {
  OK,
  MALFORMED,
  UNSUPPORTED_VERSION,
  OUT_OF_RANGE
};

HostDecodeStatus decodeHostCommand(const char *frame, HighLevelCommand &command);

#endif
