#ifndef STACKFORCE_COMMAND_SOURCES_H
#define STACKFORCE_COMMAND_SOURCES_H

// The registry is the single source of truth for command ingress identity.
// An ingress implementation owns parsing; instrumentation only consumes the
// resulting context.
namespace command_sources {
#ifndef COMMAND_SOURCE_PPM_ENABLED
#define COMMAND_SOURCE_PPM_ENABLED 1
#endif
#ifndef COMMAND_SOURCE_HOST_ENABLED
#define COMMAND_SOURCE_HOST_ENABLED 0
#endif
#if (COMMAND_SOURCE_PPM_ENABLED + COMMAND_SOURCE_HOST_ENABLED) != 1
#error "Exactly one command source must be enabled per build"
#endif

struct PpmRegistration {
  static constexpr const char *source = "PPM";
  static constexpr const char *plugin = "PPM Command Input";
  static constexpr const char *ingress = "PPM";
  static constexpr bool enabled = COMMAND_SOURCE_PPM_ENABLED != 0;
  static constexpr bool implemented = true;
};

struct HostRegistration {
  static constexpr const char *source = "HOST";
  static constexpr const char *plugin = "Host Command Input";
  static constexpr const char *ingress = "SERIAL";
  static constexpr bool enabled = COMMAND_SOURCE_HOST_ENABLED != 0;
  static constexpr bool implemented = true;
};

#if COMMAND_SOURCE_HOST_ENABLED
using ActiveRegistration = HostRegistration;
#else
using ActiveRegistration = PpmRegistration;
#endif
}

#endif
