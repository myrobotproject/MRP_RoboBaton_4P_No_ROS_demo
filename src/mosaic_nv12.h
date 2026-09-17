#pragma once

#include <array>
#include <cstdint>

namespace robobaton_demo {

constexpr uint32_t kMosaicCameraCount = 4U;
constexpr uint32_t kMosaicInputWidth = 1280U;
constexpr uint32_t kMosaicInputHeight = 1088U;
constexpr uint32_t kMosaicOutputWidth = kMosaicInputWidth * 2U;
constexpr uint32_t kMosaicOutputHeight = kMosaicInputHeight * 2U;

struct Nv12ImageView {
  const uint8_t* y_data = nullptr;
  const uint8_t* uv_data = nullptr;
  uint32_t width = 0U;
  uint32_t height = 0U;
  uint32_t stride = 0U;
  uint32_t vstride = 0U;
  uint64_t y_size_bytes = 0U;
  uint64_t uv_size_bytes = 0U;
};

struct MutableNv12ImageView {
  uint8_t* y_data = nullptr;
  uint8_t* uv_data = nullptr;
  uint32_t width = 0U;
  uint32_t height = 0U;
  uint32_t stride = 0U;
  uint32_t vstride = 0U;
  uint64_t y_size_bytes = 0U;
  uint64_t uv_size_bytes = 0U;
};

enum class MosaicNv12Status {
  kOk = 0,
  kInvalidArgument,
  kInvalidSource,
  kInvalidDestination,
};

const char* MosaicNv12StatusName(MosaicNv12Status status) noexcept;

// Mosaics four 1280x1088 NV12 inputs by camera id into a 2x2 2560x2176 packed
// NV12 output.
MosaicNv12Status CopyNv12Mosaic2x2(
    const std::array<Nv12ImageView, kMosaicCameraCount>& sources,
    const MutableNv12ImageView& destination) noexcept;

}  // namespace robobaton_demo
