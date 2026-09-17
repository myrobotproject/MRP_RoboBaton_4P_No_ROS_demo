# RoboBaton 4P non-ROS Demo

Chinese version: [README_Chinese.md](README_Chinese.md)

![RoboBaton 4P](image/4P_Cam.png)

This repository provides the RoboBaton 4P non-ROS demo runtime package for the X5 board, demo source code, public C headers, matching prebuilt runtime libraries, and build/package scripts. The description below is based only on the contents of this repository.

## Repository Contents

```text
demo/       Runtime package deployable to X5 /root/demo
include/    Public C ABI headers
lib/        Prebuilt runtime libraries matching include/ and demo/
src/        Demo source code
config/     Default sensor_demo YAML configuration
scripts/    Build, packaging, runtime verification, and helper scripts
patch/      Board-side fix packages and SC132 tuning configuration
docs/       Repository-local supplementary notes
image/      README image assets
```

Most users only need to deploy the whole `demo/` directory. Do not replace a single ELF, `.so`, or configuration file in isolation, because the programs, headers, libraries, and runtime package are versioned as one release set.

## Version Matching

The root `VERSION`, `demo/VERSION`, `demo/manifest.sha256`, `include/`, and `lib/` should be used as one release set. On the board, query each program and the project-owned runtime libraries it actually links:

```bash
cd /root/demo
./cam_demo --version
./mosaic_rtsp_demo --version
./sensor_demo --version
./imu_reader_demo --version
./serial_port_demo --version
```

These commands do not initialize the camera, IMU, or UART, so they are suitable for a basic post-deployment check.

## Demo Programs

| Program | Purpose |
|---|---|
| `sensor_demo` | Four cameras, RTSP, and IMU together, with optional ROS1 bag or H.264 MP4 recording |
| `cam_demo` | Four cameras with RTSP streaming |
| `mosaic_rtsp_demo` | Four NV12 inputs mosaiced into one RTSP output |
| `imu_reader_demo` | Standalone IMU reader |
| `serial_port_demo` | UART1/UART7 serial example; not for DEBUG_UART |

## Default Run

After deployment, run on X5:

```bash
cd /root/demo
./sensor_demo
```

Camera/RTSP only:

```bash
./cam_demo
```

Four-view mosaic RTSP:

```bash
./mosaic_rtsp_demo
./mosaic_rtsp_demo --fps 40
./mosaic_rtsp_demo --fps 50
```

`mosaic_rtsp_demo` uses the fixed RTSP URL `rtsp://<x5-ip>:558/PRR` and supports `--fps <25|30|40|50|60>`.

IMU only:

```bash
./imu_reader_demo
```

UART1/UART7 example:

```bash
./serial_port_demo
```

## Data Recording

`sensor_demo` does not record data by default. Select exactly one recording mode when persistence is required:

```bash
# ROS1 bag v2.0
./sensor_demo --record-bag /data/run.bag

# H.264 MP4 session
./sensor_demo --record-mp4-dir /data/mp4_session
```

ROS1 bag and MP4 modes are mutually exclusive. MP4 mode uses H.264 and requires the complete four-camera input mask. Frame skip applies only to ROS1 bag recording.

## Configuration

`config/sensor_config.yaml` is the default `sensor_demo` configuration template. The runtime package copy is `demo/config/sensor_config.yaml`. `sensor_demo` loads YAML defaults from the runtime package and then applies command-line overrides.

Commonly adjusted fields include:

- Camera frame rate: `25/30/40/50/60fps`, default `30fps`
- Camera canvas: native `1280x1088`, plus VSE scaled canvases `640x480`, `720x480`, and `1280x720`
- RTSP codec: H.264 by default, H.265 optional
- IMU sample rate: `25/50/100/200/500/1000/2000Hz`, default `1000Hz`
- RTSP path: `/PRR` by default

## Build From Source

This repository is cross-compiled on a development host, and the generated runtime package runs on the X5 board. Prepare an X5 aarch64 cross toolchain before building, and keep `include/` and `lib/` from the same release set.

```bash
export TOOLCHAIN_FILE="/path/to/aarch64_x5_host_toolchain.cmake"
cmake -S . -B build_x5 \
  -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE"
cmake --build build_x5 -j
```

Single-demo build scripts are also available:

```bash
scripts/build_cam_demo.sh "$TOOLCHAIN_FILE"
scripts/build_sensor_demo.sh "$TOOLCHAIN_FILE"
scripts/build_imu_reader_demo.sh "$TOOLCHAIN_FILE"
scripts/build_serial_port_demo.sh "$TOOLCHAIN_FILE"
scripts/build_mosaic_rtsp_demo.sh "$TOOLCHAIN_FILE"
```

For a complete runtime package:

```bash
scripts/package_runtime.sh --toolchain-file "$TOOLCHAIN_FILE"
python3 scripts/verify_runtime_package.py demo
```

## Deployment Notes

The usual board target directory is `/root/demo`. A conservative deployment flow is:

```text
upload to a unique temporary directory
-> verify manifest.sha256
-> ensure old demo processes have exited
-> back up the old /root/demo
-> atomically promote the new directory
-> run --version or --help for a basic check
-> restore the latest backup on failure
```

Do not delete `/root/demo` before uploading the new package. Do not mix same-named `.so` files from system directories or other projects.

## Helper Scripts

| Script | Purpose |
|---|---|
| `scripts/package_runtime.sh` | Create the complete runtime package |
| `scripts/verify_runtime_package.py` | Verify runtime package integrity and provenance |
| `scripts/cam_demo_regression.sh` | Camera demo regression entry point |
| `scripts/mp4_extract.py` | Offline MP4 session extraction |
| `scripts/rosbag_info.py` | Inspect ROS1 bag metadata |
| `scripts/rosbag_extract.py` | Extract ROS1 bag data |
| `scripts/nv12_tee_extract.py` | Offline conversion of tee-captured NV12/encoded frames |
| `scripts/wifi_setup.sh` | X5 board WiFi AP/STA setup |
| `scripts/env_setup/*.sh` | Board time sync, PTP, UART/PPS, and environment setup |

## Support Boundaries

- Camera output is NV12, `1280x1088` by default.
- The four camera RTSP streams use default ports `554..557` and path `/PRR`.
- `mosaic_rtsp_demo` outputs one mosaic RTSP stream on port `558` and path `/PRR`.
- `rotate=180` is supported only at `30fps`.
- The IMU uses the sensor-timestamp FIFO path. This repository does not provide TF or calibration files.
- DEBUG_UART is `1.8V`; UART1/UART7 are `3.3V`.
- Camera operation depends on the X5 board-side `cam-service`.

## Troubleshooting

For startup, shared-library, RTSP, IMU, or UART issues, keep:

- Root `VERSION` and `demo/VERSION`
- `demo/manifest.sha256` verification result
- Exact command and exit code
- `--version` output
- Necessary stdout/stderr logs

Do not submit real IP addresses, credentials, or internal paths in issue material.

## License

License and third-party component scope are defined by [LICENSE](LICENSE) and [LICENSE_SCOPE.md](LICENSE_SCOPE.md).
