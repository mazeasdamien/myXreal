#pragma once
#include <string>
#include <vector>
#include <memory>
#include <functional>

namespace xreal {

struct Telemetry {
    uint64_t timestamp_ns;
    float temperature_c;
    float accel[3];       // Calibrated accelerometer data (in m/s^2)
    float gyro[3];        // Calibrated gyroscope data (in rad/s)
    float quaternion[4];  // Fused orientation quaternion [w, x, y, z] (sensor-to-world, corrected)
    float euler[3];       // Roll, pitch, yaw (in degrees)
    float fps;            // Sensor polling loop frequency

    // --- INS state (from ImuNavigator) ---
    float position[3];     // World-frame position (m)
    float velocity[3];     // World-frame velocity (m/s)
    float gyro_bias[3];    // Online gyro bias estimate (rad/s)
    float accel_bias[3];   // Online accel bias estimate (m/s^2)
    float gravity[3];      // Estimated gravity in world frame (m/s^2)

    // --- INS diagnostics ---
    float mahony_quaternion[4];  // Parallel Mahony filter output for comparison
    float covariance_diag[15];   // 15x15 error-state covariance diagonal
};

using TelemetryCallback = std::function<void(const Telemetry&)>;

#if defined(_WIN32)
  #if defined(xreal_air_driver_EXPORTS)
    #define XREAL_EXPORT __declspec(dllexport)
  #else
    #define XREAL_EXPORT __declspec(dllimport)
  #endif
#else
  #define XREAL_EXPORT __attribute__((visibility("default")))
#endif

class XREAL_EXPORT XRealAirDriver {
public:
    XRealAirDriver();
    ~XRealAirDriver();

    // Initialize driver, load calibration JSON
    bool initialize(const std::string& calibration_json_path = "");

    // Get the current brightness (0 to 8). Returns -1 if not connected/unknown.
    int get_brightness() const;

    // Set the brightness (0 to 8)
    bool set_brightness(int brightness);

    // Get the current display mode. 1 = 2D, 3 = 3D. Returns -1 if unknown.
    int get_display_mode() const;

    // Set the display mode. 1 = 2D, 3 = 3D.
    bool set_display_mode(int mode);

    // Check if calibration was loaded from device flash
    bool is_calibration_from_device() const;


#if defined(__ANDROID__) || defined(__linux__)
    // Initialize the driver on Android using an open file descriptor from UsbManager
    bool initialize_with_fd(int fd, const std::string& calibration_json_path);
#endif

    // Start streaming telemetry asynchronously on a background thread.
    // filter_type can be "Mahony" or "Madgwick".
    bool start_streaming(const std::string& filter_type, TelemetryCallback callback);

    // Stop telemetry streaming
    void stop_streaming();

    // Check streaming status
    bool is_streaming() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace xreal
