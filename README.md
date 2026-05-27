# myXreal

C++17 driver and desktop tooling for XREAL Air 2 Ultra: IMU streaming, stereo camera capture, rectification, and real-time debug visualization.

Current dashboard view with stereo feeds, IMU telemetry, 3DoF glasses renderer, and calibration panel.

## What is in this repo
- `imu_driver` shared library for IMU + stereo camera access.
- `imu_debug` GUI (ImGui + D3D11) with live stereo camera feeds, IMU diagnostics, 6DoF/3DoF views, and calibration display.
- `imu_dump` CLI IMU stream viewer.

## Prerequisites (Windows)
- Visual Studio 2022 (x64 toolchain)
- CMake 3.20+
- vcpkg/system packages: Eigen3, OpenCV, yaml-cpp
- XREAL Air 2 Ultra connected for live device tests

## Build
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

Debug build:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug
```

## Run
From build outputs (DLLs/calibration are copied automatically by CMake post-build steps):

```bash
build/apps/imu_debug/Release/imu_debug.exe
build/apps/imu_dump/Release/imu_dump.exe
```

## Quick guidance
- Start with `imu_debug` for full device diagnostics.
- If calibration is not detected, verify `config/calibration.json` is present next to the executable.

## Calibration per glasses
Each pair of glasses has its own factory calibration.
- Keep one calibration JSON per device serial number.
- Pass the matching file when launching tools (or copy it next to the executable as `calibration.json`).
- Mixing calibration files between devices can degrade rectification and pose quality.

## Troubleshooting
- **No camera/IMU stream**: reconnect glasses, close conflicting camera apps, relaunch executable.
- **Build errors on OpenCV/Eigen/yaml-cpp**: verify dependency installation and CMake toolchain configuration.
- **No useful stereo image**: ensure both stereo eyes stream correctly and rectification is enabled.
