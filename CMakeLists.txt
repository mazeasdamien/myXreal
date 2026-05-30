cmake_minimum_required(VERSION 3.20)
project(myXreal VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# 0. Dependencies from vcpkg / system
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs calib3d features2d video)

# 0b. yaml-cpp
find_package(yaml-cpp CONFIG REQUIRED)

# 1. nlohmann/json
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

# 2. hidapi
FetchContent_Declare(
    hidapi
    GIT_REPOSITORY https://github.com/libusb/hidapi.git
    GIT_TAG hidapi-0.14.0
)
set(HIDAPI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(HIDAPI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(hidapi)

# 3. Dear ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.90.4
)
FetchContent_MakeAvailable(imgui)

# ImGui static library with Win32 + DX11 backends
add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

# Calibration + rectification library (static)
# Only factory calibration (calibration.json); no self-calibration.
add_library(calibration STATIC
    src/core/calibration_loader.cpp
    src/core/stereo_rectifier.cpp
)
target_include_directories(calibration PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(calibration PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
    nlohmann_json::nlohmann_json
)

# Sub-projects
add_subdirectory(src/driver)
add_subdirectory(apps/imu_debug)
add_subdirectory(apps/imu_dump)
add_subdirectory(apps/vr_stereo_scene)
