#include "cam_demo_config.h"
#include "sensor_demo_yaml_config.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace robobaton_demo {
namespace {

constexpr const char* kSc132SensorProfileEnv = "SC132_SENSOR_PROFILE";
constexpr const char* kSc132TriggerModeEnv = "SC132_TRIGGER_MODE";
constexpr const char* kSc132Single25FpsBaseProfile =
    "sc132gs_linear_1088x1280_raw10_60fps_1lane";

struct ParseState {
  bool channels_set = false;
  bool camera_selector_set = false;
  bool record_frame_skip_set = false;
  bool record_bag_set = false;
  bool record_mp4_set = false;
  int requested_channels = kMaxChannels;
};

// Prints command-line arguments supported by the demo.
// Input: program is the executable name; include_imu_options controls whether
// sensor_demo-only IMU options are shown.
// Output: help text is written to stdout.
void PrintUsage(const char* program, bool include_imu_options) {
  std::cout << "Usage: " << program << " [options]\n"
            << "  --width <pixels>  Output width: 1280x1088 (default), 640x480, 720x480, or 1280x720, either axis\n"
            << "  --height <pixels> Output height: must pair with a supported width above\n"
            << "  --fps <25|30|40|50|60>       Camera and encoder fps, default 30\n"
            << "  --rotate <0|90|180|270> Output rotation, default 0; 180 is supported only at 30fps\n"
            << "  --bps <kbps>      Encoder bitrate in kbps, default " << kDefaultBps << "\n"
            << "  --codec <h264|h265> Encoder format, default h264\n"
            << "  --url <path>      RTSP URL path, default /PRR\n"
            << "  --rtsp-base-port <port> RTSP first channel port, default "
            << kDefaultRtspBasePort << "\n"
            << "  --diagnostics     Print source liveness and per-channel RTSP timing diagnostics\n"
            << "  --diag-interval-ms <ms> Diagnostics interval, default 1000\n"
            << "  --max-skew-ns <ns> Frame-set timestamp skew limit, default "
            << kDefaultFrameSetMaxSkewNs << "\n"
            << "  --frame-timeout-ms <ms> Frame-set pending timeout, default 100\n"
            << "  --trigger-mode <software_gpio|none> SC132 trigger output mode, default "
            << kDefaultSc132TriggerMode << "\n";
  if (include_imu_options) {
    std::cout << "  " << SensorDemoYamlConfigRelativePath()
              << " YAML config is loaded before CLI options; missing file is created with defaults\n";
    std::cout << "  --sample-rate-hz <25|50|100|200|500|1000|2000> IMU sample rate, default "
              << kDefaultImuSampleRateHz << "\n";

    std::cout << "  --imu-sample-drop-policy <allow-counted|strict> IMU timing sample-drop policy, default allow-counted\n";
    std::cout << "  --imu-start-order <imu-first|camera-first> IMU startup order, default camera-first\n";
    std::cout << "  --print-rate-hz HZ IMU terminal output rate, default min(sample-rate-hz, 10); 0 disables IMU sample output\n";
    std::cout << "  --print-metrics Include metrics diagnostics section in each IMU output record, default off\n";
    std::cout << "  --record-bag <absolute-path> Write one ROS1 bag while sensor_demo runs\n";
    std::cout << "  --record-mp4-dir <absolute-directory> Store RTSP H.264 plus exact timestamp indexes and IMU CSV\n";
    std::cout << "  --record-frame-skip <0|1> With --record-bag, 0 saves every frame-set, 1 saves alternate frame-sets; default 0\n";
    std::cout << "  --capture-dir <absolute-directory> Tee raw NV12 planes plus encoded access units after warmup; default off\n";
    std::cout << "  --capture-frame-count <1.." << kMaxCaptureFrameCount
              << "> Frames per camera to capture, default " << kDefaultCaptureFrameCount << "\n";
    std::cout << "  --capture-warmup-seconds <1.." << kMaxCaptureWarmupSeconds
              << "> Wait after camera start before capture, default "
              << kDefaultCaptureWarmupSeconds << "\n";
  }
  std::cout << "  -h, --help        Show this help\n";
}

// Reads the value after the current option.
// Input: argc/argv, current option index, and option name.
// Output: the option value string, while advancing index to the value position.
// Throws std::invalid_argument when the value is missing.
std::string RequireValue(int argc, char** argv, int* index, const char* name) {
  if (*index + 1 >= argc) {
    throw std::invalid_argument(std::string("missing value for ") + name);
  }
  ++(*index);
  return std::string(argv[*index]);
}

// Parses an integer command-line argument.
// Input: text is the option text; name is used in diagnostics.
// Output: int value.
// Throws when the text has a non-numeric suffix or is outside stoi capacity.
int ParseInt(const std::string& text, const char* name) {
  size_t parsed = 0;
  const int value = std::stoi(text, &parsed);
  if (parsed != text.size()) {
    throw std::invalid_argument(std::string("invalid integer for ") + name);
  }
  return value;
}

// Parses an unsigned mask, including 0x prefixes.
// Input: text is the option text; name is used in diagnostics.
// Output: uint32_t value.
// Throws when the text has a non-numeric suffix or exceeds uint32_t.
uint32_t ParseUint32(const std::string& text, const char* name) {
  size_t parsed = 0;
  const unsigned long value = std::stoul(text, &parsed, 0);
  if (parsed != text.size() || value > 0xffffffffUL) {
    throw std::invalid_argument(std::string("invalid unsigned integer for ") + name);
  }
  return static_cast<uint32_t>(value);
}

// Parses a long long command-line argument.
// Input: text is the option text; name is used in diagnostics.
// Output: long long value.
// Throws when the text has a non-numeric suffix or is outside stoll capacity.
long long ParseLongLong(const std::string& text, const char* name) {
  size_t parsed = 0;
  const long long value = std::stoll(text, &parsed);
  if (parsed != text.size()) {
    throw std::invalid_argument(std::string("invalid integer for ") + name);
  }
  return value;
}

// Parses an unsigned 64-bit CLI value, including 0x prefixes and rejecting
// negative signs.
uint64_t ParseUint64(const std::string& text, const char* name) {
  if (!text.empty() && text.front() == '-') {
    throw std::invalid_argument(std::string("invalid unsigned integer for ") + name);
  }
  size_t parsed = 0;
  const unsigned long long value = std::stoull(text, &parsed, 0);
  if (parsed != text.size()) {
    throw std::invalid_argument(std::string("invalid unsigned integer for ") + name);
  }
  return static_cast<uint64_t>(value);
}

// Converts untrusted CLI codec text to a strongly typed enum. Only h264/h265
// are accepted.
// Input: lower-case codec text. Output: VideoCodec. Invalid values throw
// std::invalid_argument.
VideoCodec ParseVideoCodec(const std::string& text) {
  if (text == "h264") {
    return VideoCodec::kH264;
  }
  if (text == "h265") {
    return VideoCodec::kH265;
  }
  throw std::invalid_argument("--codec must be h264 or h265");
}

// Maps sensor_demo policy text to the public enum accepted by ABI v2 reserved[0].
uint32_t ParseImuSampleDropPolicy(const std::string& text) {
  if (text == "allow-counted") {
    return ICM42688_SAMPLE_DROP_POLICY_ALLOW_COUNTED;
  }
  if (text == "strict") {
    return ICM42688_SAMPLE_DROP_POLICY_STRICT;
  }
  throw std::invalid_argument(
      "--imu-sample-drop-policy must be allow-counted or strict");
}

// Converts sensor_demo startup-order text to an internal strongly typed enum.
ImuStartOrder ParseImuStartOrder(const std::string& text) {
  if (text == "imu-first") {
    return ImuStartOrder::kImuFirst;
  }
  if (text == "camera-first") {
    return ImuStartOrder::kCameraFirst;
  }
  throw std::invalid_argument("--imu-start-order must be imu-first or camera-first");
}

void ApplyChannels(Options* options, ParseState* state, int channels) {
  options->channels = channels;
  state->requested_channels = channels;
  state->channels_set = true;
  if (!state->camera_selector_set) {
    options->camera_mask = CameraMaskFromChannelCount(options->channels);
  }
}

void ApplyCameraId(Options* options, ParseState* state, int camera_id) {
  if (camera_id < 0 || camera_id >= kMaxChannels) {
    throw std::invalid_argument("--camera-id must be 0, 1, 2, or 3");
  }
  // Single-camera diagnostics select by physical camera id.
  options->camera_mask = 1U << static_cast<uint32_t>(camera_id);
  options->channels = 1;
  state->camera_selector_set = true;
}

void ApplyCameraMask(Options* options, ParseState* state, uint32_t camera_mask) {
  // camera mask accepts only single-camera diagnostics or the full four-camera set.
  if (!IsSupportedCameraMask(camera_mask)) {
    throw std::invalid_argument("--camera-mask supports only 0x1, 0x2, 0x4, 0x8, or 0xF");
  }
  options->camera_mask = camera_mask;
  options->channels = CameraMaskPopCount(camera_mask);
  state->camera_selector_set = true;
}

void ValidateOptions(const Options& options, bool record_frame_skip_set);

void ValidateSelectorState(const ParseState& state, const Options& options) {
  if (state.channels_set && state.camera_selector_set &&
      state.requested_channels != CameraMaskPopCount(options.camera_mask)) {
    throw std::invalid_argument("--channels conflicts with --camera-id/--camera-mask");
  }
}

void FinalizeParsedOptions(Options* options, const ParseState& config_state,
                           const ParseState& cli_state,
                           const SensorDemoYamlConfigState& sensor_config_state,
                           bool accept_imu_options) {
  const bool cli_selects_camera = cli_state.channels_set || cli_state.camera_selector_set;
  const ParseState& selector_state = cli_selects_camera ? cli_state : config_state;
  if (!selector_state.channels_set && !selector_state.camera_selector_set) {
    options->camera_mask = CameraMaskFromChannelCount(options->channels);
  }
  ValidateSelectorState(selector_state, *options);

  if (accept_imu_options && !sensor_config_state.imu_print_rate_was_set) {
    options->imu_print_rate_hz =
        std::min(options->imu_sample_rate_hz, kDefaultImuPrintRateHz);
  }

  ValidateOptions(*options, cli_state.record_frame_skip_set);
}

// Validates an IMU sample rate against the discrete ODR table currently exposed
// by the libicm42688 C ABI.
// Input: user-provided sample_rate_hz.
// Output: true when supported, false otherwise.
bool IsSupportedImuSampleRateHz(uint32_t sample_rate_hz) {
  return sample_rate_hz == 25U || sample_rate_hz == 50U || sample_rate_hz == 100U ||
         sample_rate_hz == 200U || sample_rate_hz == 500U || sample_rate_hz == 1000U ||
         sample_rate_hz == 2000U;
}

// Validates a camera frame rate against the discrete frame-rate table currently
// exposed by libsc132.
bool IsSupportedCameraFps(int fps) {
  return fps == 25 || fps == 30 || fps == 40 || fps == 50 || fps == 60;
}

// Checks whether runtime options are within the demo support range.
// Input: parsed Options.
// Output: none.
// Throws std::invalid_argument for invalid options.
void ValidateOptions(const Options& options, bool record_frame_skip_set) {
  // The delivery path supports only the full four-camera set; internal
  // diagnostics support only one physical sensor.
  if (options.channels != 1 && options.channels != kMaxChannels) {
    throw std::invalid_argument("--channels is an internal debug option and only supports 1 or 4");
  }
  if (!IsSupportedCameraMask(options.camera_mask) ||
      options.channels != CameraMaskPopCount(options.camera_mask)) {
    throw std::invalid_argument("--camera-mask supports only 0x1, 0x2, 0x4, 0x8, or 0xF");
  }
  // 640x480/720x480/1280x720 are full-frame scales from the libsc132 VSE
  // hardware node; scaling does not change FOV.
  if (!IsSupportedOutputResolution(options.width, options.height)) {
    throw std::invalid_argument(
        "--width/--height must be 1280x1088, 640x480, 720x480, or 1280x720 "
        "(either axis order)");
  }
  if (!IsSupportedCameraFps(options.fps)) {
    throw std::invalid_argument("--fps must be 25, 30, 40, 50, or 60");
  }
  if (options.bps <= 0 ||
      static_cast<unsigned long long>(options.bps) >
          static_cast<unsigned long long>(std::numeric_limits<uint32_t>::max())) {
    throw std::invalid_argument("--bps must fit the v2 uint32 bitrate field");
  }
  if (options.url.size() < 2U || options.url.size() > 56U || options.url.front() != '/') {
    throw std::invalid_argument("--url must be a 2..56 byte path starting with '/'");
  }
  // Reject query, fragment, escape, and path-ambiguous characters before any
  // side effects.
  for (unsigned char character : options.url) {
    if (character < 0x21U || character > 0x7eU || character == '?' ||
        character == '#' || character == '%' || character == '\\') {
      throw std::invalid_argument("--url contains a v2-forbidden character");
    }
  }
  if (options.rtsp_base_port <= 0 ||
      options.rtsp_base_port > kMaxRtspPort - (kMaxChannels - 1)) {
    throw std::invalid_argument("--rtsp-base-port must keep four channel ports in 1..65535");
  }
  if (options.rotate_degrees != 0 && options.rotate_degrees != 90 &&
      options.rotate_degrees != 180 && options.rotate_degrees != 270) {
    throw std::invalid_argument("--rotate must be 0, 90, 180, or 270");
  }
  // External 180-degree rotation enters the lower-layer 270-degree slow path;
  // keep only 30fps as the verified combination.
  if (InternalRotateDegrees(options) == 270 && options.fps != 30) {
    throw std::invalid_argument("--rotate 180 is supported only at 30fps");
  }
  if (options.diagnostic_interval_ms < 100) {
    throw std::invalid_argument("--diag-interval-ms must be >= 100");
  }
  if (options.frame_set_max_skew_ns == 0) {
    throw std::invalid_argument("--max-skew-ns must be positive");
  }
  if (options.frame_set_timeout_ms == 0) {
    throw std::invalid_argument("--frame-timeout-ms must be positive");
  }
  if (!IsSupportedImuSampleRateHz(options.imu_sample_rate_hz)) {
    throw std::invalid_argument("--sample-rate-hz must be 25, 50, 100, 200, 500, 1000, or 2000");
  }
  if (options.record_frame_skip > 1U) {
    throw std::invalid_argument("--record-frame-skip must be 0 or 1");
  }
  if (!options.record_bag_path.empty() && options.record_bag_path.front() != '/') {
    throw std::invalid_argument("--record-bag/save_data.save_path path must be absolute");
  }
  if (!options.record_mp4_directory.empty() &&
      options.record_mp4_directory.front() != '/') {
    throw std::invalid_argument("--record-mp4-dir/save_data.save_path path must be absolute");
  }
  if (!options.record_bag_path.empty() && !options.record_mp4_directory.empty()) {
    throw std::invalid_argument("ROS bag and MP4 recording are mutually exclusive");
  }
  if (record_frame_skip_set && options.record_bag_path.empty()) {
    throw std::invalid_argument("--record-frame-skip requires --record-bag");
  }
  if (!options.record_mp4_directory.empty() && options.record_frame_skip != 0U) {
    throw std::invalid_argument("MP4 recording does not support frame skip");
  }
  if (!options.record_mp4_directory.empty() &&
      options.video_codec != VideoCodec::kH264) {
    throw std::invalid_argument("MP4 recording requires --codec h264");
  }
  if (options.imu_print_rate_hz > options.imu_sample_rate_hz) {
    throw std::invalid_argument("--print-rate-hz must not exceed --sample-rate-hz");
  }
  if (options.imu_sample_drop_policy > ICM42688_SAMPLE_DROP_POLICY_STRICT) {
    throw std::invalid_argument(
        "--imu-sample-drop-policy must be allow-counted or strict");
  }
  if (options.trigger_mode != "software_gpio" && options.trigger_mode != "none") {
    throw std::invalid_argument("--trigger-mode must be one of software_gpio or none");
  }
  if (!options.capture_directory.empty() && options.capture_directory.front() != '/') {
    throw std::invalid_argument("--capture-dir/capture.save_path path must be absolute");
  }
  if (options.capture_frame_count == 0U ||
      options.capture_frame_count > kMaxCaptureFrameCount) {
    throw std::invalid_argument("--capture-frame-count must be in 1.." +
                                std::to_string(kMaxCaptureFrameCount));
  }
  if (options.capture_warmup_seconds == 0U ||
      options.capture_warmup_seconds > kMaxCaptureWarmupSeconds) {
    throw std::invalid_argument("--capture-warmup-seconds must be in 1.." +
                                std::to_string(kMaxCaptureWarmupSeconds));
  }
}

}  // namespace

// Parses camera/RTSP command-line arguments. The sensor_demo path may receive
// already loaded YAML defaults.
// Input: argc/argv from main; accept_imu_options controls sensor_demo-only
// options.
// Output: Options. --help prints help and exits the process.
// Throws std::invalid_argument for unknown options or invalid values.
Options ParseCommandLineImpl(int argc, char** argv, bool accept_imu_options,
                             Options options,
                             SensorDemoYamlConfigState sensor_config_state) {
  ParseState config_parse_state;
  config_parse_state.requested_channels = options.channels;

  ParseState cli_parse_state;
  cli_parse_state.requested_channels = options.channels;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--channels") {
      ApplyChannels(&options, &cli_parse_state,
                    ParseInt(RequireValue(argc, argv, &i, "--channels"), "--channels"));
    } else if (arg == "--camera-id") {
      ApplyCameraId(&options, &cli_parse_state,
                    ParseInt(RequireValue(argc, argv, &i, "--camera-id"), "--camera-id"));
    } else if (arg == "--camera-mask") {
      ApplyCameraMask(&options, &cli_parse_state,
                      ParseUint32(RequireValue(argc, argv, &i, "--camera-mask"),
                                  "--camera-mask"));
    } else if (arg == "--width") {
      options.width = ParseInt(RequireValue(argc, argv, &i, "--width"), "--width");
    } else if (arg == "--height") {
      options.height = ParseInt(RequireValue(argc, argv, &i, "--height"), "--height");
    } else if (arg == "--fps") {
      options.fps = ParseInt(RequireValue(argc, argv, &i, "--fps"), "--fps");
    } else if (arg == "--bps") {
      options.bps = ParseLongLong(RequireValue(argc, argv, &i, "--bps"), "--bps");
    } else if (arg == "--codec") {
      options.video_codec = ParseVideoCodec(RequireValue(argc, argv, &i, "--codec"));
    } else if (arg == "--url") {
      options.url = RequireValue(argc, argv, &i, "--url");
    } else if (arg == "--rtsp-base-port") {
      options.rtsp_base_port =
          ParseInt(RequireValue(argc, argv, &i, "--rtsp-base-port"), "--rtsp-base-port");
    } else if (arg == "--rotate") {
      options.rotate_degrees = ParseInt(RequireValue(argc, argv, &i, "--rotate"), "--rotate");
    } else if (arg == "--diagnostics") {
      options.diagnostics = true;
    } else if (arg == "--diag-interval-ms") {
      options.diagnostic_interval_ms =
          ParseInt(RequireValue(argc, argv, &i, "--diag-interval-ms"), "--diag-interval-ms");
    } else if (arg == "--max-skew-ns") {
      options.frame_set_max_skew_ns =
          ParseUint64(RequireValue(argc, argv, &i, "--max-skew-ns"), "--max-skew-ns");
    } else if (arg == "--frame-timeout-ms") {
      options.frame_set_timeout_ms = static_cast<uint32_t>(
          ParseUint32(RequireValue(argc, argv, &i, "--frame-timeout-ms"),
                      "--frame-timeout-ms"));
    } else if (arg == "--trigger-mode") {
      options.trigger_mode = RequireValue(argc, argv, &i, "--trigger-mode");
    } else if (accept_imu_options && arg == "--sample-rate-hz") {
      options.imu_sample_rate_hz =
          ParseUint32(RequireValue(argc, argv, &i, "--sample-rate-hz"), "--sample-rate-hz");
    } else if (accept_imu_options && arg == "--imu-sample-drop-policy") {
      options.imu_sample_drop_policy =
          ParseImuSampleDropPolicy(RequireValue(argc, argv, &i, "--imu-sample-drop-policy"));
    } else if (accept_imu_options && arg == "--imu-start-order") {
      options.imu_start_order =
          ParseImuStartOrder(RequireValue(argc, argv, &i, "--imu-start-order"));
    } else if (accept_imu_options && arg == "--print-rate-hz") {
      options.imu_print_rate_hz =
          ParseUint32(RequireValue(argc, argv, &i, "--print-rate-hz"),
                      "--print-rate-hz");
      sensor_config_state.imu_print_rate_was_set = true;
    } else if (accept_imu_options && arg == "--print-metrics") {
      options.imu_print_metrics = true;
    } else if (accept_imu_options && arg == "--record-bag") {
      if (cli_parse_state.record_mp4_set) {
        throw std::invalid_argument("--record-bag and --record-mp4-dir are mutually exclusive");
      }
      options.record_bag_path = RequireValue(argc, argv, &i, "--record-bag");
      if (options.record_bag_path.empty() || options.record_bag_path.front() != '/') {
        throw std::invalid_argument("--record-bag path must be absolute");
      }
      options.record_mp4_directory.clear();
      cli_parse_state.record_bag_set = true;
    } else if (accept_imu_options && arg == "--record-mp4-dir") {
      if (cli_parse_state.record_bag_set) {
        throw std::invalid_argument("--record-bag and --record-mp4-dir are mutually exclusive");
      }
      options.record_mp4_directory =
          RequireValue(argc, argv, &i, "--record-mp4-dir");
      if (options.record_mp4_directory.empty() ||
          options.record_mp4_directory.front() != '/') {
        throw std::invalid_argument("--record-mp4-dir path must be absolute");
      }
      options.record_bag_path.clear();
      cli_parse_state.record_mp4_set = true;
    } else if (accept_imu_options && arg == "--record-frame-skip") {
      options.record_frame_skip =
          ParseUint32(RequireValue(argc, argv, &i, "--record-frame-skip"),
                      "--record-frame-skip");
      cli_parse_state.record_frame_skip_set = true;
    } else if (accept_imu_options && arg == "--capture-dir") {
      options.capture_directory = RequireValue(argc, argv, &i, "--capture-dir");
      if (options.capture_directory.empty() ||
          options.capture_directory.front() != '/') {
        throw std::invalid_argument("--capture-dir path must be absolute");
      }
      options.capture_enabled = true;
    } else if (accept_imu_options && arg == "--capture-frame-count") {
      options.capture_frame_count =
          ParseUint32(RequireValue(argc, argv, &i, "--capture-frame-count"),
                      "--capture-frame-count");
    } else if (accept_imu_options && arg == "--capture-warmup-seconds") {
      options.capture_warmup_seconds =
          ParseUint32(RequireValue(argc, argv, &i, "--capture-warmup-seconds"),
                      "--capture-warmup-seconds");
    } else if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0], accept_imu_options);
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + arg);
    }
  }

  FinalizeParsedOptions(&options, config_parse_state, cli_parse_state,
                        sensor_config_state, accept_imu_options);
  return options;
}

Options ParseCommandLine(int argc, char** argv) {
  return ParseCommandLineImpl(argc, argv, false, Options{}, SensorDemoYamlConfigState{});
}

Options ParseSensorDemoCommandLineWithConfig(int argc, char** argv, Options options,
                                             SensorDemoYamlConfigState sensor_config_state) {
  return ParseCommandLineImpl(argc, argv, true, std::move(options), sensor_config_state);
}

// Writes the command-line selected trigger mode to the environment variable used
// by libsc132.
// Input: options.trigger_mode, supporting software_gpio, none, and compatibility
// aliases.
// Side effect: overwrites process-local SC132_TRIGGER_MODE; software_gpio mode
// uses GPIO417.
void ConfigureSc132TriggerMode(const Options& options) {
  // Command-line arguments take precedence over the shell environment.
  if (setenv(kSc132TriggerModeEnv, options.trigger_mode.c_str(), 1) != 0) {
    throw std::runtime_error("set SC132_TRIGGER_MODE failed");
  }
  std::cout << kSc132TriggerModeEnv << "=" << options.trigger_mode
            << " (GPIO417 is used when mode=software_gpio)\n";
}

// Automatically fills a compatible 60fps base sensor profile for internal
// single-camera 25fps smoke runs.
// Input: options.camera_mask/options.fps.
// Side effect: sets a compatible profile when internal diagnostics enable only
// one 25fps sensor and SC132_SENSOR_PROFILE is not preset.
void ConfigureSc132SensorProfile(const Options& options) {
  const char* current_profile = std::getenv(kSc132SensorProfileEnv);
  if (current_profile != nullptr && current_profile[0] != '\0') {
    std::cout << "SC132 sensor profile already configured\n";
    return;
  }

  // Four-camera runs use the libsc132 default profile; single-camera 25fps runs
  // use the SDK-compatible 60fps base profile.
  if (CameraMaskPopCount(options.camera_mask) != 1 || options.fps != 25) {
    return;
  }

  // setenv affects only this process and does not modify the board-wide shell
  // environment.
  if (setenv(kSc132SensorProfileEnv, kSc132Single25FpsBaseProfile, 1) != 0) {
    throw std::runtime_error("set SC132_SENSOR_PROFILE failed");
  }
  std::cout << "Auto selected single-sensor 25fps base profile\n";
}

}  // namespace robobaton_demo
