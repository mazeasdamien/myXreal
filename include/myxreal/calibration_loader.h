#pragma once

#include "myxreal/calibration_types.h"
#include <string>
#include <cstdint>

// Forward-declare OpenCV types (keep the header light)
namespace cv { class Mat; }

enum CameraId { CAM_LEFT = 0, CAM_RIGHT = 1 };

// ---------------------------------------------------------------------------
// Load calibration from a YAML file (OpenCV stereo or Kalibr format).
// If load_undistort_maps is true, precomputes cv::initUndistortRectifyMap
// for both cameras (requires OpenCV). Returns nullptr on failure.
// ---------------------------------------------------------------------------
CalibrationData* calibration_load(const char* path, bool load_undistort_maps = true);

// ---------------------------------------------------------------------------
// Load calibration from the XREAL factory calibration.json format.
// Supports fisheye624 camera model with 12 distortion coefficients.
// ---------------------------------------------------------------------------
CalibrationData* calibration_load_json(const char* path);

// ---------------------------------------------------------------------------
// Save calibration to JSON file (XREAL factory format, but supports Fisheye4).
// Returns true on success.
// ---------------------------------------------------------------------------
bool calibration_save_json(const char* path, const CalibrationData& calib);

// ---------------------------------------------------------------------------
// Release all resources (undistort maps, etc.). Safe to call with nullptr.
// ---------------------------------------------------------------------------
void calibration_free(CalibrationData* calib);

// ---------------------------------------------------------------------------
// Undistort a greyscale frame in-place using the precomputed maps.
// Returns false if maps were not loaded (load_undistort_maps was false).
// ---------------------------------------------------------------------------
bool calibration_undistort(cv::Mat& frame, CameraId cam);

// ---------------------------------------------------------------------------
// Globally accessible singleton — set once by the consumer that calls load.
// ---------------------------------------------------------------------------
void calibration_set_singleton(CalibrationData* calib);
CalibrationData* calibration_get_singleton();
