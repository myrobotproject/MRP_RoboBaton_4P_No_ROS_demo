#include "cam_demo_common.h"

#include <limits.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

extern "C" {
#include "icm42688_driver.h"
}

#ifndef ROBOBATON_RELEASE_VERSION
#define ROBOBATON_RELEASE_VERSION "0.0.0+unknown"
#endif

namespace robobaton_demo {
volatile sig_atomic_t g_imu_signal_stop = 0;

namespace {
#ifdef RELEASE008_TESTING
// Tests only count empty-queue waits; the atomic counter avoids concurrent
// read/write races.
std::atomic<uint32_t> g_idle_wait_count{0U};
#endif


struct IcmCallbackContext {
  static constexpr std::size_t kPendingCapacity = 64U;

  std::atomic<bool> accepting{true};
  std::atomic<bool> callback_failed{false};
  std::atomic<uint32_t> emitted{0U};
  std::mutex pending_mutex;
  // Fixed-capacity FIFO preserves burst order and avoids dynamic allocation on
  // the callback path.
  std::array<icm42688_sample_t, kPendingCapacity> pending_samples{};
  std::size_t pending_head = 0U;
  std::size_t pending_size = 0U;
  ImuSampleObserver observer = nullptr;
  void* observer_user = nullptr;
};

// The callback only publishes data or the first error; stop/destroy are run by
// the lifecycle owner.
void IcmCallback(const icm42688_sample_t* sample, void* user) noexcept {
  auto* context = static_cast<IcmCallbackContext*>(user);
  if (context == nullptr || !context->accepting.load(std::memory_order_acquire)) {
    return;
  }
  try {
    if (sample == nullptr || sample->struct_size != sizeof(*sample)) {
      // Invalid input closes admission directly, avoiding exception-object
      // allocation on the producer thread.
      context->callback_failed.store(true, std::memory_order_release);
      context->accepting.store(false, std::memory_order_release);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(context->pending_mutex);
      // Recheck admission and enqueue in the same lock domain.
      if (!context->accepting.load(std::memory_order_acquire)) {
        return;
      }
      if (context->pending_size == IcmCallbackContext::kPendingCapacity) {
        // Capacity exhaustion must not overwrite or drop old samples. Close
        // admission so the owner returns non-zero.
        context->callback_failed.store(true, std::memory_order_release);
        context->accepting.store(false, std::memory_order_release);
        return;
      }
      const std::size_t tail =
          (context->pending_head + context->pending_size) %
          IcmCallbackContext::kPendingCapacity;
      context->pending_samples[tail] = *sample;
      ++context->pending_size;
    }
  } catch (...) {
    context->callback_failed.store(true, std::memory_order_release);
    context->accepting.store(false, std::memory_order_release);
  }
}

}  // namespace

#ifdef RELEASE008_TESTING
void ResetImuIdleWaitCountForTest() {
  g_idle_wait_count.store(0U, std::memory_order_release);
}

uint32_t ImuIdleWaitCountForTest() {
  return g_idle_wait_count.load(std::memory_order_acquire);
}

std::size_t ImuPendingCapacityForTest() {
  return IcmCallbackContext::kPendingCapacity;
}
#endif

int RunIcmConsumer(const ImuConsumerOptions& options, ImuSampleObserver observer,
                   void* observer_user) {
  if (options.sample_rate_hz == 0U) {
    return 1;
  }

  icm42688_config_t config = ICM42688_CONFIG_INIT;
  config.sample_rate_hz = options.sample_rate_hz;
  config.fifo_watermark_samples = 1U;
  config.read_mode = ICM42688_READ_MODE_SENSOR_TIMESTAMP_FIFO;
  // Policy is passed through ABI v2 reserved[0]; the public config layout must
  // not be extended.
  config.reserved[ICM42688_CONFIG_SAMPLE_DROP_POLICY_INDEX] = options.sample_drop_policy;

  if (options.final_health != nullptr) {
    *options.final_health = ICM42688_RUNTIME_HEALTH_INIT;
  }
  IcmCallbackContext context;
  context.observer = observer;
  context.observer_user = observer_user;
  icm42688_handle_t* handle = nullptr;

  int result = icm42688_create(&config, &handle);
  if (result != ICM42688_STATUS_OK || handle == nullptr) {
    if (handle != nullptr) {
      icm42688_destroy(handle);
    }
    return 1;
  }

  result = icm42688_set_callback(handle, IcmCallback, &context);
  if (result != ICM42688_STATUS_OK) {
    context.accepting.store(false, std::memory_order_release);
    icm42688_destroy(handle);
    return 1;
  }

  result = icm42688_start(handle);
  if (result != ICM42688_STATUS_OK) {
    context.accepting.store(false, std::memory_order_release);
    // After start failure, the create owner still destroys a non-null handle.
    icm42688_destroy(handle);
    return 1;
  }

  bool count_reached = false;
  const auto emit_sample = [&](icm42688_sample_t sample) noexcept {
    try {
      if (options.system_clock != nullptr) {
        sample.host_timestamp_ns = options.system_clock->MapRawNs(sample.host_timestamp_ns);
        sample.sample_timestamp_ns = options.system_clock->MapRawNs(sample.sample_timestamp_ns);
      }
      // The observer runs on the owner thread; producer callbacks do not perform
      // blocking I/O.
      if (context.observer != nullptr) {
        context.observer(sample, context.observer_user);
      }
      context.emitted.fetch_add(1U, std::memory_order_acq_rel);
      return true;
    } catch (...) {
      context.callback_failed.store(true, std::memory_order_release);
      context.accepting.store(false, std::memory_order_release);
      return false;
    }
  };

  while (g_imu_signal_stop == 0 &&
         (options.stop_requested == nullptr ||
          !options.stop_requested->load(std::memory_order_acquire)) &&
         !context.callback_failed.load(std::memory_order_acquire)) {
    icm42688_sample_t sample{};
    bool has_sample = false;
    {
      std::lock_guard<std::mutex> lock(context.pending_mutex);
      if (context.pending_size != 0U) {
        // Pop the oldest sample in FIFO order and run the observer after
        // unlocking.
        sample = context.pending_samples[context.pending_head];
        context.pending_head =
            (context.pending_head + 1U) % IcmCallbackContext::kPendingCapacity;
        --context.pending_size;
        has_sample = true;
      }
    }
    // Continue draining immediately when the queue has backlog; only empty
    // queues enter the short wait.
    if (has_sample) {
      if (!emit_sample(sample)) {
        break;
      }
      if (options.count != 0U &&
          context.emitted.load(std::memory_order_acquire) >= options.count) {
        count_reached = true;
        break;
      }
      continue;
    }
    const bool owner_stop_requested =
        g_imu_signal_stop != 0 ||
        (options.stop_requested != nullptr &&
         options.stop_requested->load(std::memory_order_acquire));
    if (!owner_stop_requested && icm42688_is_running(handle) == 0) {
      // If the producer exits before count is reached and before the owner asks
      // it to stop, fail closed only after confirming drain at an empty-queue
      // boundary.
      context.callback_failed.store(true, std::memory_order_release);
      context.accepting.store(false, std::memory_order_release);
      break;
    }
#ifdef RELEASE008_TESTING
    // Test counting exists only in the empty-queue branch to verify backlog
    // periods do not wait.
    g_idle_wait_count.fetch_add(1U, std::memory_order_relaxed);
#endif
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  const bool owner_stop_requested =
      g_imu_signal_stop != 0 ||
      (options.stop_requested != nullptr &&
       options.stop_requested->load(std::memory_order_acquire));
  const bool drain_after_stop =
      owner_stop_requested && !count_reached &&
      !context.callback_failed.load(std::memory_order_acquire);
  const auto close_admission = [&]() noexcept {
    std::lock_guard<std::mutex> lock(context.pending_mutex);
    context.accepting.store(false, std::memory_order_release);
  };

  // Count/failure paths preserve the prior semantics: close callback admission before stop.
  // Owner/signal stop keeps admission open until blocking stop joins the producer, then drains
  // every sample that had already entered the adapter FIFO.
  if (!drain_after_stop) {
    close_admission();
  }
  const int stop_result = icm42688_stop(handle);
  if (drain_after_stop) {
    close_admission();
  }
  if (drain_after_stop && stop_result == ICM42688_STATUS_OK &&
      !context.callback_failed.load(std::memory_order_acquire)) {
    while (true) {
      icm42688_sample_t sample{};
      {
        std::lock_guard<std::mutex> lock(context.pending_mutex);
        if (context.pending_size == 0U) {
          break;
        }
        sample = context.pending_samples[context.pending_head];
        context.pending_head =
            (context.pending_head + 1U) % IcmCallbackContext::kPendingCapacity;
        --context.pending_size;
      }
      if (!emit_sample(sample)) {
        break;
      }
    }
  }
  int health_result = ICM42688_STATUS_OK;
  if (options.final_health != nullptr) {
    health_result = stop_result == ICM42688_STATUS_OK
                        ? icm42688_get_runtime_health(handle, options.final_health)
                        : ICM42688_STATUS_INVALID_STATE;
  }
  const bool failed = context.callback_failed.load(std::memory_order_acquire) ||
                      stop_result != ICM42688_STATUS_OK ||
                      health_result != ICM42688_STATUS_OK;
  icm42688_destroy(handle);
  return failed ? 1 : 0;
}

uint32_t ImuPrintEverySamples(uint32_t sample_rate_hz, uint32_t print_rate_hz) {
  if (sample_rate_hz == 0U || print_rate_hz == 0U) {
    return 0U;
  }
  // Use quotient and remainder for ceiling division to avoid adding two
  // uint32_t values and overflowing.
  return sample_rate_hz / print_rate_hz +
         (sample_rate_hz % print_rate_hz == 0U ? 0U : 1U);
}

void PrintImuSample(const icm42688_sample_t& sample, void* user) {
  auto* state = static_cast<ImuPrintState*>(user);
  if (state == nullptr) {
    return;
  }
  ++state->observed_samples;
  if (state->print_every_samples == 0U) {
    return;
  }
  // Attempt output only at fixed sampling points. Samples are still fully
  // consumed after the output sink fails.
  if (state->observed_samples != 1U &&
      (state->observed_samples - 1U) % state->print_every_samples != 0U) {
    return;
  }
  if (!state->output_available) {
    ++state->dropped_output_lines;
    return;
  }

  const uint64_t timestamp_ns = sample.sample_timestamp_ns;
  const double accel_norm =
      std::sqrt(sample.accel_mps2[0] * sample.accel_mps2[0] +
                sample.accel_mps2[1] * sample.accel_mps2[1] +
                sample.accel_mps2[2] * sample.accel_mps2[2]);

  // Keep each terminal record within PIPE_BUF and write the complete multi-line
  // block with one write. Slow or closed sinks only affect logging and do not
  // block the capture owner.
  std::array<char, PIPE_BUF> line;
  const unsigned long long ts_ns = static_cast<unsigned long long>(timestamp_ns);
  std::size_t line_length = 0U;
  const auto append_to_line = [&](const char* format, auto... args) -> bool {
    const std::size_t remaining = line.size() - line_length;
    const int appended = std::snprintf(line.data() + line_length, remaining, format,
                                       args...);
    if (appended < 0 || static_cast<std::size_t>(appended) >= remaining) {
      return false;
    }
    line_length += static_cast<std::size_t>(appended);
    return true;
  };

  // Write the required data section first. Append the metrics diagnostic section
  // only when the CLI switch is enabled; always append the final delimiter.
  bool formatted = append_to_line(
      "*******************************IMU*******************************\n"
      "imu data:\n"
      "sample_seq=%llu\n"
      "ts_ns=%llu\n"
      "temp_c=%.6f accel_norm_mps2=%.6f\n"
      "accel_mps2=[%.6f, %.6f, %.6f]\n"
      "gyro_rps  =[%.6f, %.6f, %.6f]\n",
      static_cast<unsigned long long>(sample.sample_sequence), ts_ns,
      sample.temperature_c, accel_norm,
      sample.accel_mps2[0], sample.accel_mps2[1], sample.accel_mps2[2],
      sample.gyro_rps[0], sample.gyro_rps[1], sample.gyro_rps[2]);
  if (formatted && state->print_metrics) {
    const unsigned long long host_ns =
        static_cast<unsigned long long>(sample.host_timestamp_ns);
    const double dt_ms = state->last_timestamp_ns == 0U
                             ? 0.0
                             : static_cast<double>(timestamp_ns - state->last_timestamp_ns) /
                                   1000000.0;
    const double host_ts_gap_ms = host_ns >= ts_ns
                                      ? static_cast<double>(host_ns - ts_ns) / 1000000.0
                                      : -static_cast<double>(ts_ns - host_ns) / 1000000.0;
    formatted = append_to_line(
        "metrics:\n"
        "host_ts_ns=%llu\n"
        "host_ts_gap_ms=%.6f dt_ms=%.6f uncertainty_us=%u\n"
        "gpio_gap_count=%u fifo_overflow_count=%u mapper_failure_count=%u\n",
        host_ns, host_ts_gap_ms, dt_ms,
        sample.timestamp_uncertainty_us, sample.gpio_event_gap_count,
        sample.fifo_overflow_count, sample.mapper_failure_count);
  }
  formatted = formatted && append_to_line(
      "%s", "*****************************************************************\n");
  if (!formatted) {
    ++state->dropped_output_lines;
    return;
  }

  const ssize_t written = ::write(state->output_fd, line.data(), line_length);
  if (written >= 0 && static_cast<std::size_t>(written) == line_length) {
    state->last_timestamp_ns = timestamp_ns;
    return;
  }

  // Slow sinks or signal interruptions only drop the current log; the owner
  // never retries. Partial writes and EPIPE/permanent fd errors disable later
  // logs to avoid continuously producing truncated lines.
  ++state->dropped_output_lines;
  if (written >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
    state->output_available = false;
  }
}

}  // namespace robobaton_demo

#if !defined(RELEASE008_TESTING) && !defined(SENSOR_DEMO_NO_MAIN)
namespace {

using robobaton_demo::ImuConsumerOptions;
using robobaton_demo::ScopedNonblockingFd;

void SignalHandler(int) { robobaton_demo::g_imu_signal_stop = 1; }

std::string RequireValue(int argc, char** argv, int* index, const char* name) {
  if (*index + 1 >= argc) {
    throw std::invalid_argument(std::string("missing value for ") + name);
  }
  ++(*index);
  return std::string(argv[*index]);
}

// Parses a decimal CLI argument and rejects values outside uint32_t to avoid
// truncating large integers into valid sample rates or counts.
uint32_t ParseUint32Argument(const std::string& text, const char* name) {
  size_t parsed = 0U;
  const unsigned long value = std::stoul(text, &parsed, 10);
  if (parsed != text.size()) {
    throw std::invalid_argument(std::string("invalid unsigned integer for ") + name);
  }
  constexpr unsigned long kMaxUint32 = 0xffffffffUL;
  if (value > kMaxUint32) {
    throw std::out_of_range(std::string("out of range for ") + name);
  }
  return static_cast<uint32_t>(value);
}


ImuConsumerOptions ParseCommandLine(int argc, char** argv, uint32_t* print_rate_hz,
                                    bool* print_metrics) {
  if (print_rate_hz == nullptr || print_metrics == nullptr) {
    throw std::invalid_argument("CLI output pointer is null");
  }
  uint32_t requested_print_rate_hz = 0U;
  bool print_rate_was_set = false;
  bool print_metrics_enabled = false;
  ImuConsumerOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--sample-rate-hz") {
      options.sample_rate_hz =
          ParseUint32Argument(RequireValue(argc, argv, &index, "--sample-rate-hz"),
                              "--sample-rate-hz");
      if (options.sample_rate_hz != 25U && options.sample_rate_hz != 50U &&
          options.sample_rate_hz != 100U && options.sample_rate_hz != 200U &&
          options.sample_rate_hz != 500U && options.sample_rate_hz != 1000U &&
          options.sample_rate_hz != 2000U) {
        throw std::invalid_argument("--sample-rate-hz must be 25, 50, 100, 200, 500, 1000, or 2000");
      }
    } else if (argument == "--count") {
      options.count =
          ParseUint32Argument(RequireValue(argc, argv, &index, "--count"), "--count");
    } else if (argument == "--print-rate-hz") {
      requested_print_rate_hz =
          ParseUint32Argument(RequireValue(argc, argv, &index, "--print-rate-hz"),
                              "--print-rate-hz");
      print_rate_was_set = true;
    } else if (argument == "--print-metrics") {
      print_metrics_enabled = true;
    } else if (argument == "--help" || argument == "-h") {
      std::cout << "Usage: imu_reader_demo [options]:\n"
                << "  --sample-rate-hz <25|50|100|200|500|1000|2000> IMU sample rate, default "
                << robobaton_demo::kDefaultImuSampleRateHz << "\n"
                << "  --count N Number of IMU samples to consume before exit, default 0 (run until signal)\n"
                << "  --print-rate-hz HZ Terminal output rate, default min(sample-rate-hz, 10); 0 disables output\n"
                << "  --print-metrics Include metrics diagnostics section in each output record, default off\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  // The default 10Hz limit applies only to terminal logs; the owner still
  // consumes all IMU samples.
  *print_rate_hz = print_rate_was_set
                       ? requested_print_rate_hz
                       : std::min(options.sample_rate_hz,
                                  robobaton_demo::kDefaultImuPrintRateHz);
  // Validate final values after full parsing so argument order does not affect
  // the result.
  if (*print_rate_hz > options.sample_rate_hz) {
    throw std::invalid_argument("--print-rate-hz must not exceed --sample-rate-hz");
  }
  *print_metrics = print_metrics_enabled;
  return options;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--version") {
      std::cout << "imu_reader_demo " << ROBOBATON_RELEASE_VERSION << "\n"
                << "libicm42688 " << icm42688_get_version() << " abi="
                << ICM42688_ABI_VERSION_MAJOR << "." << ICM42688_ABI_VERSION_MINOR << "\n";
      return 0;
    }
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
    // Closed stdout only means the log sink is unavailable; SIGPIPE must not
    // terminate capture.
    signal(SIGPIPE, SIG_IGN);
    uint32_t print_rate_hz = 0U;
    bool print_metrics = false;
    ImuConsumerOptions options = ParseCommandLine(argc, argv, &print_rate_hz, &print_metrics);
    robobaton_demo::FrozenSystemClock system_clock;
    system_clock.PrintTimeBase(std::cout);
    options.system_clock = &system_clock;
    robobaton_demo::ImuPrintState state;
    state.print_every_samples =
        robobaton_demo::ImuPrintEverySamples(options.sample_rate_hz, print_rate_hz);
    state.print_metrics = print_metrics;
    // Non-blocking flags belong to the shared OFD. RAII strictly limits the
    // change to the capture window. On setup failure, CLI output is disabled;
    // destruction covers normal and exceptional paths and restores caller state.
    ScopedNonblockingFd output_mode(state.output_fd);
    state.output_available = output_mode.active();
    const int result = robobaton_demo::RunIcmConsumer(options, robobaton_demo::PrintImuSample,
                                                      &state);
    if (result != 0) {
      std::cerr << "fatal: ICM consumer lifecycle failed\n";
    }
    return result;
  } catch (const std::exception& error) {
    std::cerr << "fatal: " << error.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "fatal: unknown exception\n";
    return 1;
  }
}
#endif
