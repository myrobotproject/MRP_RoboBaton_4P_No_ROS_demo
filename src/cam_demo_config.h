#pragma once

#include "cam_demo_common.h"

namespace robobaton_demo {

// Parses command-line arguments and validates them.
// Input: argc/argv received by main.
// Output: complete Options; invalid arguments throw std::invalid_argument.
Options ParseCommandLine(int argc, char** argv);

// Parses sensor_demo command-line arguments, first loading
// DEMO_DIR/config/sensor_config.yaml and then applying CLI overrides.
// Input: argc/argv received by main.
// Output: complete Options; invalid arguments throw std::invalid_argument.
Options ParseSensorDemoCommandLine(int argc, char** argv);

// Configures the libsc132 trigger output mode.
// Input: options.trigger_mode, defaulting to software_gpio.
// Side effect: sets the process-local SC132_TRIGGER_MODE environment variable,
// read during libsc132 initialization.
void ConfigureSc132TriggerMode(const Options& options);

// Selects a compatible sensor profile for specific startup combinations.
// Input: runtime options.
// Side effect: sets the process-local SC132_SENSOR_PROFILE environment variable
// when needed.
void ConfigureSc132SensorProfile(const Options& options);

}  // namespace robobaton_demo
