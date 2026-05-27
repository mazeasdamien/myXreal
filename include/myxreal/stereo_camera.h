#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#ifdef IMU_DRIVER_EXPORTS
#define IMU_API __declspec(dllexport)
#else
#define IMU_API __declspec(dllimport)
#endif
#else
#define IMU_API __attribute__((visibility("default")))
#endif

#include <stdint.h>

// Fixed camera resolution (portrait orientation per eye after descrambling)
#define STEREO_EYE_WIDTH   480
#define STEREO_EYE_HEIGHT  640
#define STEREO_EYE_PIXELS  (STEREO_EYE_WIDTH * STEREO_EYE_HEIGHT)  // 307200

// Single camera frame for one eye.
// The data pointer is valid until the next call to stereo_poll_pair().
typedef struct {
    uint64_t      timestamp_ns;   // system-monotonic timestamp of this frame
    uint64_t      frame_index;    // monotonic counter per eye
    int           width;
    int           height;
    int           camera_id;      // 0 = left, 1 = right
    const uint8_t* data;          // greyscale pixels (owned by driver, valid until next poll)
    float         fps;            // capture FPS for this eye
    bool          is_rectified;   // set by StereoRectifier
} CameraFrame;

// Forward declaration
struct CalibrationData;

// Synchronized pair of left + right camera frames.
typedef struct {
    CameraFrame            left;
    CameraFrame            right;
    uint64_t               timestamp_ns;     // midpoint timestamp
    float                  sync_delta_ms;    // absolute time difference left vs right, milliseconds
    int                    drops;            // cumulative dropped pairs (sync delta > 5ms)
    const CalibrationData* calib;            // set by stereo_set_calibration()
} StereoPair;

// Initialize the stereo camera driver.
// Returns 0 on success, non-zero on failure.
IMU_API int stereo_init(void);

// Start the camera capture thread. Returns 0 on success.
IMU_API int stereo_start_streaming(void);

// Stop the camera capture thread.
IMU_API void stereo_stop_streaming(void);

// Poll for the latest stereo pair. Non-blocking.
// Returns 1 if a new pair was popped, 0 if none available.
// The data pointers in left/right frames remain valid until the next call.
IMU_API int stereo_poll_pair(StereoPair* out_pair);

// Number of stereo pairs currently available in the ring buffer.
IMU_API int stereo_available_pairs(void);

// Attach calibration data for downstream VIO consumers.
// The pointer is copied into every StereoPair emitted by stereo_poll_pair().
IMU_API void stereo_set_calibration(const CalibrationData* calib);

// Shut down the camera driver and release all resources.
IMU_API void stereo_shutdown(void);

#ifdef __cplusplus
}
#endif
