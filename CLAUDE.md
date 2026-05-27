# myXreal — Agent Instructions

## Project Overview

C++17 desktop application for XREAL Air 2 Ultra AR glasses — IMU data streaming, stereo camera capture, fisheye rectification, and calibration diagnostics.

## Build System

**Visual Studio 2022** + **CMake** (v3.20+). Default config: **Release**, **x64**.

### Build commands

```bash
# Build everything (Release)
cmake --build build --config Release

# Build specific target
cmake --build build --config Release --target imu_debug
cmake --build build --config Release --target imu_dump
cmake --build build --config Release --target rectify_diag

# Re-configure CMake (after adding files or changing CMakeLists.txt)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

# Debug build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug
```

### Build outputs

```
build/apps/imu_debug/Release/imu_debug.exe     # GUI debug tool
build/apps/imu_dump/Release/imu_dump.exe       # CLI IMU logger
build/tests/Release/rectify_diag.exe            # Epipolar diagnostic
build/src/driver/Release/imu_driver.dll         # Shared library
build/Release/calibration.lib                   # Static calibration library
```

### Dependencies

- **System (find_package):** Eigen3, OpenCV (core,imgproc,imgcodecs,calib3d,features2d,video), yaml-cpp
- **FetchContent (auto-downloaded):** nlohmann/json v3.11.3, hidapi 0.14.0, Dear ImGui v1.90.4
- **Windows SDK:** Direct3D 11, DirectX GI, DirectShow (strmiids)

### Running executables

Executables auto-copy the needed DLLs (`imu_driver.dll`, `hidapi_winapi.dll`, `calibration.json`) next to themselves via CMake `POST_BUILD` commands. Run from the build output directory:

```bash
cd build/apps/imu_debug/Release && ./imu_debug.exe
cd build/apps/imu_dump/Release && ./imu_dump.exe
cd build/tests/Release && ./rectify_diag.exe
```

The `calibration.json` (from `config/`) is copied next to the executable automatically during build.

## Project Structure

```
CMakeLists.txt              → Root build: 4 sub-projects + calibration lib

config/
  calibration.json          → Factory calibration (FSN G428X00585)
  camera_calib.yaml         → Camera calibration YAML

include/myxreal/
  calibration_types.h       → CameraIntrinsics, CameraExtrinsics, CalibrationData
  calibration_loader.h      → Calibration I/O (JSON + YAML)
  stereo_rectifier.h        → Custom fisheye624 → pinhole rectification
  self_calibrator.h         → Feature-tracking self-calibration
  imu_driver.h              → C API: IMU init/stream/clock sync/alignment
  stereo_camera.h           → C API: stereo capture via DirectShow

src/
  core/
    CMakeLists.txt
    calibration_loader.cpp  → Calibration I/O (JSON + YAML)
    stereo_rectifier.cpp    → Custom fisheye624 → pinhole rectification
    self_calibrator.cpp     → Feature-tracking self-calibration
  driver/
    CMakeLists.txt          → Shared library (imu_driver.dll)
    imu_driver.cpp          → C API implementation
    stereo_camera.cpp       → C API: stereo capture via DirectShow
    ring_buffer.h           → Lock-free SPSC ring buffer

apps/
  imu_debug/                → GUI tool (ImGui + D3D11)
    CMakeLists.txt
    main.cpp                → Real-time plots, camera view, calibration UI
  imu_dump/                 → CLI tool
    CMakeLists.txt
    main.cpp                → Live console table of IMU data

tests/
  CMakeLists.txt
  rectify_diag.cpp          → Epipolar quality verification

scripts/
  da3_sidecar.py            → Python DA3 sidecar (stdio IPC inference)
  stereo_distance.py        → Python stereo depth utility

models/
  DA3-SMALL/                → DA3 ONNX model files
```

## Camera Models

Three models defined in `calibration_types.h`:
- **Pinhole** — standard OpenCV 5-param (k1,k2,p1,p2,k3)
- **Fisheye624** — XREAL factory 12-param (6 radial + 2 tangential + 4 thin-prism) — NOT supported by OpenCV, uses custom projection in `StereoRectifier`
- **Fisheye4** — OpenCV Kannala-Brandt 4-param

## Documentation Maintenance

When you make changes to the project structure, source files, CMake files, or APIs — including adding/renaming/removing files — **update the Obsidian project overview** to match.

**Target:** `C:\Users\Damien Mazeas\Documents\Obsidian Vault\myXreal\Project Overview.md`

Keep this in sync: project map, calibration data sections, API tables, build system info, and the data flow diagram.
