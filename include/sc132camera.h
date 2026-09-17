#ifndef SC132_CAMERA_H
#define SC132_CAMERA_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SC132_ABI_VERSION_MAJOR 2U
#define SC132_ABI_VERSION_MINOR 0U

#define SC132_STATUS_OK ((int32_t)0)
#define SC132_STATUS_INVALID_ARGUMENT ((int32_t)-1)
#define SC132_STATUS_INVALID_STATE ((int32_t)-2)
#define SC132_STATUS_STARTUP_FAILED ((int32_t)-3)

#define SC132_FRAME_SET_MAX_CAMERAS 4U
#define SC132_NATIVE_OUTPUT_WIDTH 1280U
#define SC132_NATIVE_OUTPUT_HEIGHT 1088U
#define SC132_FRAME_SET_DEFAULT_MAX_SKEW_NS 10000000ULL

/*
 * Frame-set output canvases: native is 1280x1088 (internally 1088x1280 in
 * sensor axes for 0/180 rotation). VSE also exposes full-frame hardware-scaled
 * canvases 640x480, 720x480, and 1280x720; both axis orders are accepted
 * (external 90/270 rotation swaps delivered width and height). Scaling does
 * not change FOV: the full frame is stretched to the target size, so the
 * aspect ratio changes relative to the native canvas. Software rotation
 * (180/270) is performed by Nano2D after scaling.
 */

typedef struct sc132_frame sc132_frame_t;

typedef struct sc132_frame_info {
  uint32_t struct_size;
  uint32_t camera_id;
  uint64_t sequence;
  uint32_t frame_id;
  uint32_t reserved0;
  uint64_t timestamp_ns;
  const void *y_data;
  const void *uv_data;
  uint64_t y_phys;
  uint64_t uv_phys;
  uint64_t y_size;
  uint64_t uv_size;
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t vstride;
  uint32_t reserved[8];
} sc132_frame_info_t;

typedef struct sc132_frame_set_item {
  sc132_frame_t *frame;
  uint32_t camera_id;
  uint32_t frame_id;
  uint64_t sequence;
  uint64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
} sc132_frame_set_item_t;

typedef struct sc132_frame_set {
  uint32_t struct_size;
  uint32_t camera_count;
  uint64_t group_id;
  uint64_t group_timestamp_ns;
  uint64_t max_skew_ns;
  sc132_frame_set_item_t items[SC132_FRAME_SET_MAX_CAMERAS];
  uint32_t reserved[8];
} sc132_frame_set_t;

/*
 * DMA frames are read-only. The following contract defines time domains,
 * borrowed references, and retention across callbacks.
 * timestamp_ns is in ns. In software_gpio mode, frame sets use the GPIO417
 * rising-edge time in CLOCK_MONOTONIC_RAW, identical across all cameras in the
 * group. Other modes prefer sensor/VIO per-frame time and fall back to system
 * frame delivery time when unavailable. No mode guarantees the same domain as
 * wall clock time.
 * frame_set and the item array are valid only during the callback;
 * items[i].frame is a borrowed reference.
 * Consumers must not call sc132_frame_release directly on the callback
 * reference owned by the library. To keep a frame across callbacks, call
 * sc132_frame_retain inside the callback and later call sc132_frame_release
 * after final use.
 * user_data must remain valid until blocking sc132_stop returns; request_stop
 * returning by itself does not mean callbacks have exited.
 * C++ callbacks must be noexcept and catch all exceptions internally;
 * exceptions must not cross this C ABI.
 * Y/UV addresses in frame_info are read-only and valid only while the matching
 * frame reference is valid.
 */
typedef void (*sc132_frame_set_callback_t)(const sc132_frame_set_t *frame_set,
                                            void *user_data);

typedef struct sc132_frame_set_config {
  uint32_t struct_size;
  sc132_frame_set_callback_t callback;
  void *user_data;
  uint32_t camera_count;
  uint32_t width;
  uint32_t height;
  uint32_t timeout_ms;
  uint64_t max_skew_ns;
  uint32_t reserved[8];
} sc132_frame_set_config_t;

#define SC132_FRAME_SET_CONFIG_INIT \
  { sizeof(sc132_frame_set_config_t), NULL, NULL, 4U, SC132_NATIVE_OUTPUT_WIDTH, SC132_NATIVE_OUTPUT_HEIGHT, 100U, SC132_FRAME_SET_DEFAULT_MAX_SKEW_NS, {0U} }

/* Product release SemVer; returned storage is process-static and read-only. */
const char *sc132_get_version(void);
int32_t sc132_set_fps(uint32_t fps);
int32_t sc132_set_output_rotation(uint32_t rotate_clockwise_degrees);
int32_t sc132_start_frame_set(const sc132_frame_set_config_t *config,
                              uint32_t camera_mask);
int32_t sc132_frame_retain(sc132_frame_t *frame);
void sc132_frame_release(sc132_frame_t *frame);
int32_t sc132_frame_get_info(const sc132_frame_t *frame,
                             sc132_frame_info_t *out_info);
/*
 * Two-phase shutdown prevents blocking callbacks from waiting on retained
 * frames while retained frames wait on callbacks.
 * request_stop only linearizes stop, rejects new capture/callback admission,
 * and broadcasts wakeups. It does not drain/release frames, call vendor APIs,
 * join trigger/worker/dispatcher threads, or wait for callbacks, so it can
 * return quickly from any thread.
 * Calling request_stop while idle also latches STOPPING. A non-callback thread
 * must then call blocking sc132_stop to complete the corresponding stop
 * generation before start/config are opened again.
 * The normal lifecycle owner then calls blocking sc132_stop. It drains
 * pending/queued frames, waits for inflight callbacks, joins library-created
 * threads, and closes I2C. If a vendor worker never exits, sc132_stop may block
 * indefinitely, and the lifecycle remains STOPPING until real quiescence rather
 * than pretending to be STOPPED.
 * In the rare case that an internal pthread_join fails, sc132_stop keeps thread
 * ownership, I2C, and callback/config state, releases the cleanup owner, and
 * remains STOPPING. An external non-callback thread must retry sc132_stop;
 * start/unload are forbidden until it succeeds.
 * After sc132_start_frame_set returns STARTUP_FAILED, an external non-callback
 * thread must call sc132_stop to complete quiescence before unloading this
 * library. Until then, later start/config calls return INVALID_STATE. The
 * library does not create a detached cleanup guard.
 * When stop is called from the dispatcher callback thread, it only publishes
 * request_stop and returns immediately. An external non-callback thread must
 * call stop again to let the sole cleanup owner drain/join, avoiding a
 * deterministic dispatcher self-deadlock.
 */
void sc132_request_stop(void);
void sc132_stop(void);

#ifdef __cplusplus
}
#endif

#endif
