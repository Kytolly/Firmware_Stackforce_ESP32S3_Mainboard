#ifndef STACKFORCE_COMMAND_SOURCES_H
#define STACKFORCE_COMMAND_SOURCES_H

// The registry is the single source of truth for command ingress identity.
// An ingress implementation owns parsing; instrumentation only consumes the
// resulting context.
namespace command_sources {
struct RemotePpmRegistration {
  static constexpr const char *source = "REMOTE";
  static constexpr const char *plugin = "PPM Remote Input";
  static constexpr const char *ingress = "PPM";
  static constexpr bool enabled = true;
  static constexpr bool implemented = true;
};
}

#endif
