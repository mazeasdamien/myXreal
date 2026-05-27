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

// Forward declaration
struct CalibrationData;

// Single calibrated IMU sample.
typedef struct {
    uint64_t timestamp_ns;
    float    temperature_c;
    float    accel[3];   // m/s^2, calibrated
    float    gyro[3];    // rad/s, calibrated
    float    fps;
} ImuSample;

typedef struct {
    int      valid;
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t release_bcd;
    char     manufacturer[128];
    char     product[128];
    char     serial[128];
    char     firmware[128];
} DeviceInfo;

typedef struct {
    int      valid;
    int      connected;
    int      imu_streaming;
    int      led_state_valid;
    int      led_state;
    int      mode_3d_valid;
    int      mode_3d_enabled;
    int      brightness_valid;
    int      brightness_percent;
    uint64_t timestamp_ns;
} DeviceState;

// Affine clock model: system_ns = device_ns * scale + offset_ns.
// Fitted from (device_ns, system_ns) pairs via linear regression with outlier rejection.
typedef struct {
    double   offset_ns;
    double   scale;
    double   uncertainty_ns;
    uint64_t last_update_ns;
    int      sample_count;
    int      is_converged;
} ClockModel;

// Initialize the driver and open the device.
// calibration_path: path to calibration.json, or NULL to skip calibration.
// Returns 0 on success, non-zero on failure.
IMU_API int imu_init(const char* calibration_path);

// Start the background streaming thread. Returns 0 on success.
IMU_API int imu_start_streaming(void);

// Stop the streaming thread.
IMU_API void imu_stop_streaming(void);

// Poll for a new sample. Non-blocking. Returns 1 if a sample was popped, 0 if none.
IMU_API int imu_poll_sample(ImuSample* out_sample);

// Number of samples currently available in the ring buffer.
IMU_API int imu_available_samples(void);

// Attach calibration data for downstream VIO consumers.
IMU_API void imu_set_calibration(const CalibrationData* calib);

// Raw device metadata / state (best-effort from HID path).
IMU_API int imu_get_device_info(DeviceInfo* out_info);
IMU_API int imu_get_device_state(DeviceState* out_state);
IMU_API int imu_refresh_device_state(void);

// Tear down the driver and release all resources.
IMU_API void imu_shutdown(void);

// --- Clock synchronization ---
// Get a copy of the current clock model (thread-safe).
IMU_API ClockModel clock_get_model(void);

// Convert between IMU device-ns and system-ns using the fitted model.
IMU_API uint64_t clock_to_system_ns(uint64_t device_ns);
IMU_API uint64_t clock_to_device_ns(uint64_t system_ns);

// Returns 1 once enough samples have been collected for a reliable model.
IMU_API int clock_is_converged(void);

// Copy up to max_count recent residuals (system_ns - predicted_system_ns) into out.
// Returns number of residuals copied.
IMU_API int clock_get_residuals(double* out, int max_count);

// --- Time alignment verification ---

// Per-metric pass/fail and overall verdict.
// Histogram buckets: 21 bins, 1ms width, covering [-10, +10] ms.
// Bin 0 = -10..-9ms, bin 10 = 0..+1ms, bin 20 = +9..+10ms.
typedef struct {
    double  mean_delta_us;
    double  std_delta_us;
    double  min_delta_us;
    double  max_delta_us;
    double  mean_samples_per_frame;
    double  cross_corr_lag_ms;
    double  cross_corr_peak;
    int     histogram[21];
    int     total_frames;
    int     pass_mean_delta;          // mean < 5 ms
    int     pass_std_delta;           // std  < 3 ms
    int     pass_corr_lag;            // lag  < 5 ms
    int     pass_samples_per_frame;   // 33 +/- 2 at 1kHz/30fps
    int     overall_pass;
} AlignmentReport;

// Feed the verifier. Timestamps must be in unified system-ns.
// gyro_magnitude correlates with visual frame diff (no gravity bias unlike accel).
IMU_API void alignment_record_imu(uint64_t system_ns, float accel_magnitude, float gyro_magnitude);
IMU_API void alignment_record_frame(uint64_t system_ns, float frame_diff);

// Compute current report from the live ring buffers.
IMU_API AlignmentReport alignment_get_report(void);

// --- Shake test (5-second stress recording) ---
IMU_API void alignment_start_shake_test(void);
IMU_API int  alignment_shake_test_active(void);   // returns 1 while recording
IMU_API AlignmentReport alignment_get_shake_report(void);
IMU_API int  alignment_shake_completed_gen(void); // incremented when a test finishes

#ifdef __cplusplus
}
#endif
