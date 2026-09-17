# RoboBaton 4P non-ROS Demo

English version: [README.md](README.md)

![RoboBaton 4P](image/4P_Cam.png)

本仓库提供 RoboBaton 4P 在 X5 板端使用的 non-ROS demo 运行包、示例源码、公开 C 头文件、预编译运行库和配套构建/打包脚本。

## 仓库内容

```text
demo/       可直接部署到 X5 /root/demo 的运行包
include/    公开 C ABI 头文件
lib/        与 include/ 和 demo/ 匹配的预编译运行库
src/        demo 示例源码
config/     sensor_demo 默认 YAML 配置
scripts/    构建、打包、运行包验证和辅助脚本
patch/      板端修复包和 SC132 tuning 配置
docs/       本仓内部补充说明
image/      README 图片资源
```

普通用户通常只需要整体部署 `demo/`。不要只替换单个 ELF、单个 `.so` 或单个配置文件，否则容易造成程序、头文件、动态库和运行包版本不一致。

## 版本匹配

本仓根目录的 `VERSION`、`demo/VERSION`、`demo/manifest.sha256`、`include/` 和 `lib/` 应作为同一发布组合使用。板端可在 `/root/demo` 查询各程序及其实际链接的自研运行库版本：

```bash
cd /root/demo
./cam_demo --version
./mosaic_rtsp_demo --version
./sensor_demo --version
./imu_reader_demo --version
./serial_port_demo --version
```

这些命令不需要初始化相机、IMU 或 UART，适合用于部署后的基础检查。

## Demo 程序

| 程序 | 用途 |
|---|---|
| `sensor_demo` | 四路相机、RTSP 和 IMU 联合运行，可选 ROS1 bag 或 H.264 MP4 保存 |
| `cam_demo` | 四路相机与 RTSP 推流 |
| `mosaic_rtsp_demo` | 四路 NV12 拼接后通过单路 RTSP 输出 |
| `imu_reader_demo` | 独立 IMU 读取 |
| `serial_port_demo` | UART1/UART7 串口示例，不适用于 DEBUG_UART |

## 默认运行

部署完成后在 X5 上执行：

```bash
cd /root/demo
./sensor_demo
```

只运行相机/RTSP：

```bash
./cam_demo
```

运行四路拼接 RTSP：

```bash
./mosaic_rtsp_demo
./mosaic_rtsp_demo --fps 40
./mosaic_rtsp_demo --fps 50
```

`mosaic_rtsp_demo` 固定 RTSP 地址为 `rtsp://<x5-ip>:558/PRR`，支持 `--fps <25|30|40|50|60>`。

只运行 IMU：

```bash
./imu_reader_demo
```

运行 UART1/UART7 示例：

```bash
./serial_port_demo
```

## 数据保存

`sensor_demo` 默认不自动保存数据。需要保存时显式选择一种模式：

```bash
# ROS1 bag v2.0
./sensor_demo --record-bag /data/run.bag

# H.264 MP4 session
./sensor_demo --record-mp4-dir /data/mp4_session
```

ROS1 bag 与 MP4 互斥。MP4 模式使用 H.264，并要求完整四路相机输入；frame skip 仅适用于 ROS1 bag。

## 配置

`config/sensor_config.yaml` 是 `sensor_demo` 的默认配置模板，运行包内对应路径为 `demo/config/sensor_config.yaml`。`sensor_demo` 会按运行包路径读取 YAML 默认值，再由命令行参数覆盖。

常见可调项包括：

- 相机帧率：`25/30/40/50/60fps`，默认 `30fps`
- 相机画布：native `1280x1088`，以及 VSE 缩放画布 `640x480`、`720x480`、`1280x720`
- RTSP 编码：默认 H.264，可选 H.265
- IMU 采样率：`25/50/100/200/500/1000/2000Hz`，默认 `1000Hz`
- RTSP path：默认 `/PRR`

## 从源码构建

本仓库面向开发机交叉编译，生成的运行包在 X5 板端运行。构建前需要准备 X5 aarch64 交叉编译 toolchain，并保持 `include/` 与 `lib/` 来自同一发布组合。

```bash
export TOOLCHAIN_FILE="/path/to/aarch64_x5_host_toolchain.cmake"
cmake -S . -B build_x5 \
  -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE"
cmake --build build_x5 -j
```

也可以使用单 demo 构建脚本：

```bash
scripts/build_cam_demo.sh "$TOOLCHAIN_FILE"
scripts/build_sensor_demo.sh "$TOOLCHAIN_FILE"
scripts/build_imu_reader_demo.sh "$TOOLCHAIN_FILE"
scripts/build_serial_port_demo.sh "$TOOLCHAIN_FILE"
scripts/build_mosaic_rtsp_demo.sh "$TOOLCHAIN_FILE"
```

维护完整运行包时使用：

```bash
scripts/package_runtime.sh --toolchain-file "$TOOLCHAIN_FILE"
python3 scripts/verify_runtime_package.py demo
```

## 部署建议

板端目标目录通常为 `/root/demo`。建议按以下顺序部署：

```text
上传到唯一临时目录
-> 校验 manifest.sha256
-> 确认旧 demo 已退出
-> 备份旧 /root/demo
-> 原子切换到新目录
-> 运行 --version 或 --help 做基础验证
-> 失败时恢复最近备份
```

不要先删除 `/root/demo` 再上传新包；也不要混用系统目录或其他工程中的同名 `.so`。

## 辅助脚本

| 脚本 | 用途 |
|---|---|
| `scripts/package_runtime.sh` | 生成完整运行包 |
| `scripts/verify_runtime_package.py` | 校验运行包完整性和 provenance |
| `scripts/cam_demo_regression.sh` | 相机 demo 回归入口 |
| `scripts/mp4_extract.py` | MP4 session 离线提取 |
| `scripts/rosbag_info.py` | 查看 ROS1 bag 信息 |
| `scripts/rosbag_extract.py` | 提取 ROS1 bag 数据 |
| `scripts/nv12_tee_extract.py` | tee capture NV12/encoded 帧离线转换 |
| `scripts/wifi_setup.sh` | X5 板端 WiFi AP/STA 配置 |
| `scripts/env_setup/*.sh` | 板端时间同步、PTP、UART/PPS 等环境配置 |

## 支持边界

- 相机输出为 NV12，默认 `1280x1088`。
- 四路相机默认 RTSP 端口为 `554..557`，path 为 `/PRR`。
- `mosaic_rtsp_demo` 输出单路拼接 RTSP，默认端口为 `558`，path 为 `/PRR`。
- `rotate=180` 只支持 `30fps`。
- IMU 使用 sensor-timestamp FIFO；本仓不提供 TF 或标定文件。
- DEBUG_UART 为 `1.8V`；UART1/UART7 为 `3.3V`。
- 相机运行依赖 X5 板端 `cam-service`。

## 故障排查

遇到启动、动态库、RTSP、IMU 或 UART 问题时，优先保留以下信息：

- 根目录 `VERSION` 与 `demo/VERSION`
- `demo/manifest.sha256` 校验结果
- 实际执行命令和退出码
- `--version` 输出
- 必要的 stdout/stderr 日志

请不要在问题材料中提交真实 IP、凭据或内部路径。

## 许可证

许可证和第三方组件说明以本仓库的 [LICENSE](LICENSE) 与 [LICENSE_SCOPE.md](LICENSE_SCOPE.md) 为准。
