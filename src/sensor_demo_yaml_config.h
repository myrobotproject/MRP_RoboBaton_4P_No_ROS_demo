#pragma once

#include "cam_demo_common.h"

#include <string>

namespace robobaton_demo {

struct SensorDemoYamlConfigState {
  bool imu_print_rate_was_set = false;
  bool save_data_enabled = false;
  std::string save_data_format = "mp4";
  std::string save_data_path = "/root/demo/save_mp4/";
  bool capture_enabled = false;
  std::string capture_directory = "/root/demo/capture/";
  uint32_t capture_frame_count = kDefaultCaptureFrameCount;
  uint32_t capture_warmup_seconds = kDefaultCaptureWarmupSeconds;
};

// Relative path of the user YAML configuration inside the sensor_demo runtime
// package.
inline const char* SensorDemoYamlConfigRelativePath() noexcept {
  return "config/sensor_config.yaml";
}

// Resolves the sensor_demo YAML configuration path from DEMO_DIR; uses the
// current directory when DEMO_DIR is unset.
std::string SensorDemoYamlConfigPath();

// Reads or creates the sensor_demo YAML configuration and writes supported
// prefixed fields into options.
void LoadSensorDemoYamlConfig(Options* options, SensorDemoYamlConfigState* state);

// Parses sensor_demo CLI using already loaded YAML defaults; cam_demo does not
// link the YAML implementation.
Options ParseSensorDemoCommandLineWithConfig(int argc, char** argv, Options options,
                                             SensorDemoYamlConfigState state);

}  // namespace robobaton_demo
