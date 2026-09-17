#pragma once

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "cam_demo_common.h"
#include "prrtsp_v2.h"

namespace robobaton_demo {

// Schema identifier for the top-level tee capture session metadata file.
inline constexpr const char* kTeeCaptureSessionSchema =
    "robobaton_tee_capture_session_v1";

struct TeeCaptureRecorderStats {
  uint64_t raw_frames_retained = 0U;
  uint64_t raw_retain_failures = 0U;
  uint64_t raw_frames_written = 0U;
  uint64_t raw_frames_quota_dropped = 0U;
  uint64_t raw_frames_queue_dropped = 0U;
  uint64_t warmup_skipped_frames = 0U;
  uint64_t encoded_aus_seen = 0U;
  uint64_t encoded_aus_matched = 0U;
  uint64_t encoded_aus_unmatched = 0U;
  uint64_t prefix_aus_buffered = 0U;
  uint64_t prefix_overflow_events = 0U;
  std::array<uint64_t, kMaxChannels> raw_frames_written_by_camera{};
  std::array<uint64_t, kMaxChannels> encoded_aus_matched_by_camera{};
};

struct TeeCaptureFinishResult {
  bool data_complete = false;
  bool cleanup_complete = false;
  std::string error;

  operator bool() const noexcept { return data_complete && cleanup_complete; }
};

// In-process tee capture recorder. After sensor_demo starts, it waits
// capture_warmup_seconds from full camera startup, then saves capture_frame_count
// consecutive frames per camera as compact NV12 plane bytes and RTSP encoded
// access units. When disabled, Start is never called and this class does not
// participate in the frame path, preserving existing RTSP/MP4/bag semantics.
//
// Threading model: ObserveRawFrame runs on pipeline worker threads, retains the
// frame once, and enqueues copied metadata. ObserveEncodedFrame runs on RTSP
// encoder callback threads; the AU is borrowed only until callback return and is
// copied immediately into per-camera buffers. Pixel copies and file writes are
// performed on the private writer thread, so the frame path is not blocked.
class TeeCaptureRecorder final {
 public:
  TeeCaptureRecorder();
  ~TeeCaptureRecorder();

  TeeCaptureRecorder(const TeeCaptureRecorder&) = delete;
  TeeCaptureRecorder& operator=(const TeeCaptureRecorder&) = delete;

  // Called after the sc132 frame-set starts successfully. Creates the output
  // directory skeleton and fixes the warmup end time. Directory creation
  // failure throws std::runtime_error.
  void Start(const Options& options);

  // pipeline on_queued_frame hook. After warmup, retains capture_frame_count
  // frames per camera; each frame is retained once and enqueued without copying
  // pixels or writing files.
  void ObserveRawFrame(int camera_id, const QueuedFrame& frame) noexcept;

  // RTSP encoded observer. Copies access units into per-camera buffers. The
  // decode prefix is reset at the latest key frame so target frames can be
  // decoded starting from that prefix.
  void ObserveEncodedFrame(int camera_id,
                           const prrtsp_encoded_frame_v2& frame) noexcept;

  // Called after RTSP close completes. Stops the writer, releases all retained
  // frames, and writes frames.jsonl/prefix.h264/session.json. Returns whether
  // the captured data is complete.
  TeeCaptureFinishResult Finish(bool session_success) noexcept;
  void Abort() noexcept;

  bool enabled() const noexcept { return enabled_.load(std::memory_order_acquire); }
  bool HasFatalError() const noexcept;
  std::string ErrorMessage() const;
  TeeCaptureRecorderStats SnapshotStats() const;

#ifdef RELEASE008_TESTING
  // Test hook: warmup == 0 enters the capture window immediately after Start.
  void SetWarmupSecondsForTest(uint32_t seconds) noexcept;
  void SetCaptureFrameCountForTest(uint32_t count) noexcept;
  void FailRawWriteForTest() noexcept;
  void FailEncodedWriteForTest() noexcept;
  uint64_t PendingRawJobsForTest(int camera_id) const noexcept;
  uint64_t PendingEncodedAusForTest(int camera_id) const noexcept;
#endif

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  std::atomic<bool> enabled_{false};
};

}  // namespace robobaton_demo
