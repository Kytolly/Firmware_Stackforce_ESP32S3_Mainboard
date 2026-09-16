# Firmware_Stackforce_ESP32S3_Mainboard
StackForce mainboard firmware for robot-side execution, instrumentation, safety, and remote control.

## Features
- [x] PPM remote control
- [x] Servo control
- [x] Wheel / BLDC motor control
- [x] CAN communication
- [x] IMU sensing
- [x] Five-bar inverse kinematics
- [x] Gait / motion generation
- [x] Pitch / roll stabilization
- [x] Remote-controlled locomotion
- [x] Robot-side instrumentation
- [x] Unified telemetry interface
- [x] Pluggable command-input interface
- [x] PPM remote input plugin
- [x] Host / companion-computer command input plugin
- [x] Command-source identification
- [x] High-level command interface
- [x] Low-level command interface
- [ ] Control-source arbitration
- [ ] Plugin command permissions & safety
- [ ] Session / rollout metadata
- [ ] Firmware safety state / unified STOP & timeout

## Command architecture

`HighLevelCommand` is an explicit representation of the robot's existing native high-level control interface. It is not newly invented HOST control semantics. The official PPM implementation already maps raw input into locomotion and posture intent before gait generation, stabilization, inverse kinematics, and actuator command generation.

`LowLevelCommand` explicitly represents the existing native hardware-command boundary immediately before Servo, local BLDC, and front-wheel CAN execution. These interfaces explicitize existing boundaries; they do not redefine control behavior.

```text
PPM  -> existing mapping -> HighLevelCommand -> native controller -> LowLevelCommand -> hardware
HOST -> validated frame  -> HighLevelCommand -> same controller   -> LowLevelCommand -> hardware
```

The command-source registry enables exactly one source per build. The default build enables PPM; the HOST build enables serial HOST input. Runtime arbitration, takeover, preemption, fallback, and direct HOST `LowLevelCommand` input are not implemented. A future HOST-side adapter may translate teleoperation, experiment, policy, or MPC output into the native `HighLevelCommand`; firmware does not interpret producer-specific policy semantics.

The current native interface contains forward/backward intent, steering, height, roll, wheel throttle, and the existing control/motion/stabilization mode flags. It has no independent external pitch command, so none is invented for HOST.
