#ifndef PRRTSP_V2_H
#define PRRTSP_V2_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(__clang__)
#define PRRTSP_API __attribute__((visibility("default")))
#else
#define PRRTSP_API
#endif

#define PRRTSP_OK ((int32_t)0)
#define PRRTSP_E_INVALID_ARGUMENT ((int32_t)-1)
#define PRRTSP_E_UNSUPPORTED ((int32_t)-2)
#define PRRTSP_E_NO_MEMORY ((int32_t)-3)
#define PRRTSP_E_BUSY ((int32_t)-4)
#define PRRTSP_E_STATE ((int32_t)-5)
#define PRRTSP_E_CODEC ((int32_t)-6)
#define PRRTSP_E_RTSP ((int32_t)-7)
#define PRRTSP_E_TIMEOUT ((int32_t)-8)
#define PRRTSP_E_INTERNAL ((int32_t)-9)
#define PRRTSP_E_CLEANUP_REQUIRED ((int32_t)-10)

#define PRRTSP_STREAM_OPENING ((uint32_t)1)
#define PRRTSP_STREAM_OPEN ((uint32_t)2)
#define PRRTSP_STREAM_CLOSING ((uint32_t)3)
#define PRRTSP_STREAM_ERROR ((uint32_t)4)

#define PRRTSP_STREAM_CONFIG_V2_0_SIZE ((uint32_t)232)
#define PRRTSP_STREAM_CONFIG_V2_1_SIZE ((uint32_t)240)
#define PRRTSP_STREAM_CONFIG_V2_2_SIZE ((uint32_t)256)
#define PRRTSP_ENCODED_FRAME_V2_0_SIZE ((uint32_t)104)
#define PRRTSP_NV12_FRAME_V2_0_SIZE ((uint32_t)152)
#define PRRTSP_STREAM_STATUS_V2_0_SIZE ((uint32_t)104)
#define PRRTSP_PATH_CAPACITY_BYTES ((uint32_t)128)
#define PRRTSP_PATH_CONTENT_MAX_BYTES_V2_0 ((uint32_t)56)

/* The external NV12 flag is valid only when struct_size >= PRRTSP_STREAM_CONFIG_V2_1_SIZE. */
#define PRRTSP_STREAM_FLAG_EXTERNAL_NV12 ((uint32_t)1)

#define PRRTSP_CODEC_DEFAULT ((uint32_t)0)
#define PRRTSP_CODEC_H264 ((uint32_t)1)
#define PRRTSP_CODEC_H265 ((uint32_t)2)

#define PRRTSP_ENCODED_FRAME_FLAG_KEY_FRAME ((uint32_t)1)

typedef struct prrtsp_stream prrtsp_stream_t;
typedef void (*prrtsp_frame_release_callback)(void *user);

typedef struct prrtsp_encoded_frame_v2 {
    uint32_t struct_size;
    uint32_t flags;
    uint32_t codec;
    uint32_t reserved0;
    uint64_t data_address;
    uint64_t size_bytes;
    uint64_t timestamp_ns;
    uint64_t reserved[8];
} prrtsp_encoded_frame_v2;

/*
 * Encoded AUs passed to the callback are borrowed only until the callback
 * returns. Do not store data_address, block, or re-enter the same stream.
 * timestamp_ns preserves the original ns value of the matching input frame;
 * it is not reconstructed from encoder microsecond PTS.
 */
typedef void (*prrtsp_encoded_frame_callback)(
    const prrtsp_encoded_frame_v2 *frame, void *user);

typedef struct prrtsp_stream_config_v2 {
    uint32_t struct_size;
    uint32_t flags;
    uint32_t width;
    uint32_t height;
    uint32_t fps_num;
    uint32_t fps_den;
    uint32_t bitrate_kbps;
    uint32_t rotation_clockwise;
    uint32_t port;
    uint32_t operation_timeout_ms;
    char path[128];
    uint64_t reserved[8];
    uint32_t codec;
    uint32_t reserved_v2_1;
    prrtsp_encoded_frame_callback encoded_frame_callback;
    void *encoded_frame_user;
} prrtsp_stream_config_v2;

typedef struct prrtsp_nv12_frame_v2 {
    uint32_t struct_size;
    uint32_t flags;
    uint32_t width;
    uint32_t height;
    uint32_t y_stride;
    uint32_t uv_stride;
    uint32_t y_vstride;
    uint32_t uv_vstride;
    uint64_t y_virtual_address;
    uint64_t uv_virtual_address;
    uint64_t y_physical_address;
    uint64_t uv_physical_address;
    uint64_t y_size_bytes;
    uint64_t uv_size_bytes;
    uint64_t timestamp_ns;
    uint64_t reserved[8];
} prrtsp_nv12_frame_v2;

typedef struct prrtsp_stream_status_v2 {
    uint32_t struct_size;
    uint32_t state;
    int32_t last_error;
    uint32_t reserved0;
    uint64_t frames_accepted;
    uint64_t frames_failed;
    uint64_t last_timestamp_ns;
    uint64_t reserved[8];
} prrtsp_stream_status_v2;

/* Product release SemVer; returned storage is process-static and read-only. */
PRRTSP_API const char *prrtsp_get_version(void);
PRRTSP_API int32_t prrtsp_stream_open(const prrtsp_stream_config_v2 *config,
                                      prrtsp_stream_t **out_stream);
PRRTSP_API int32_t prrtsp_stream_send(prrtsp_stream_t *stream,
                                      const prrtsp_nv12_frame_v2 *frame);
/*
 * External NV12 mode borrows the caller's virtual/physical addresses directly;
 * the caller must keep the frame lease alive until the release callback.
 * When release_callback is non-null, this function consumes the frame lease on
 * every return path and eventually invokes the callback exactly once.
 * The callback may run before return, or may be delayed until the input slot is
 * recycled or the stream closes successfully.
 * The callback must not re-enter the same stream.
 */
PRRTSP_API int32_t prrtsp_stream_send_external(
    prrtsp_stream_t *stream,
    const prrtsp_nv12_frame_v2 *frame,
    prrtsp_frame_release_callback release_callback,
    void *release_user);
PRRTSP_API int32_t prrtsp_stream_get_status(prrtsp_stream_t *stream,
                                            prrtsp_stream_status_v2 *status);
PRRTSP_API int32_t prrtsp_stream_close(prrtsp_stream_t **stream);

#ifdef __cplusplus
}
#endif

#endif
