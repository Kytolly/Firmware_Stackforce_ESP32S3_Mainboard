#ifndef STACKFORCE_COMMAND_SOURCES_H
#define STACKFORCE_COMMAND_SOURCES_H

// The registry is the single source of truth for command ingress identity.
// An ingress implementation owns parsing; instrumentation only consumes the
// resulting context.
namespace command_sources {
constexpr const char *REMOTE_SOURCE = "REMOTE";
constexpr const char *REMOTE_INGRESS = "PPM";

struct RemotePpmRegistration {
  static constexpr const char *source = REMOTE_SOURCE;
  static constexpr const char *ingress = REMOTE_INGRESS;
};
}

#endif
