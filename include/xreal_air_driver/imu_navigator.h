#pragma once
#include <Eigen/Dense>
#include <array>
#include <vector>
#include <mutex>
#include <cmath>

namespace xreal {

class Calibration;

// Full inertial navigation state snapshot (double precision)
struct InertialState {
    uint64_t  timestamp_ns = 0;
    double    q_WB[4] = {1.0, 0.0, 0.0, 0.0}; // [w, x, y, z] world←body
    double    p_WB[3] = {0.0, 0.0, 0.0};       // world-frame position (m)
    double    v_WB[3] = {0.0, 0.0, 0.0};       // world-frame velocity (m/s)
    double    bg[3]   = {0.0, 0.0, 0.0};       // gyro bias, body frame (rad/s)
    double    ba[3]   = {0.0, 0.0, 0.0};       // accel bias, body frame (m/s^2)
    double    g_W[3]  = {0.0, -9.81, 0.0};     // gravity in world frame (m/s^2)
    double    cov_diag[15] = {};                // diagonal of 15x15 error-state covariance
};

// Integration method used by the strapdown propagator
enum class IntegratorLabel {
    ZeroOrder   // Zeroth-order hold: constant angular rate + constant acceleration over dt
    // RK4  // (future) 4th-order Runge-Kutta
};

class ImuNavigator {
public:
    ImuNavigator();

    // Configure from calibration (noise params, cross-axis matrices)
    void configure(const Calibration& calib);

    // Main update — call at IMU rate (~1kHz)
    // dt: seconds between samples
    // accel: calibrated accelerometer (m/s^2, in IMU body frame)
    // gyro: calibrated gyroscope (rad/s, in IMU body frame)
    void update(float dt, const std::array<float, 3>& accel,
                const std::array<float, 3>& gyro);

    // State accessors
    std::array<float, 4> get_quaternion() const;        // [w, x, y, z] sensor-to-world
    std::array<float, 3> get_position() const;           // world frame (m)
    std::array<float, 3> get_velocity() const;           // world frame (m/s)
    std::array<float, 3> get_gyro_bias() const;          // rad/s
    std::array<float, 3> get_accel_bias() const;         // m/s^2
    std::array<float, 3> get_gravity() const;            // world frame (m/s^2)

    // Error-state covariance diagonal (15 elements, for diagnostics)
    std::array<float, 15> get_covariance_diag() const;

    // Full state snapshot (double precision)
    InertialState get_state() const;
    InertialState get_state(uint64_t timestamp_ns) const;

    // Angular difference between two quaternions (degrees) — inline for test access
    static float orientation_delta_deg(const std::array<float, 4>& q1,
                                        const std::array<float, 4>& q2) {
        double dqw = static_cast<double>(q1[0]) * q2[0] +
                     static_cast<double>(q1[1]) * q2[1] +
                     static_cast<double>(q1[2]) * q2[2] +
                     static_cast<double>(q1[3]) * q2[3];
        if (dqw > 1.0) dqw = 1.0;
        if (dqw < -1.0) dqw = -1.0;
        double angle_rad = 2.0 * std::acos(std::abs(dqw));
        return static_cast<float>(angle_rad * 180.0 / 3.141592653589793);
    }

    // Which integrator is being used
    IntegratorLabel get_integrator_label() const { return IntegratorLabel::ZeroOrder; }

    // Status
    bool is_initialized() const { return initialized_; }
    float init_progress() const; // 0.0 - 1.0 during stationary init
    double get_accel_variance() const; // accel magnitude variance during init

    // Reset to pre-init state
    void reset();

private:
    mutable std::mutex mutex_;

    // --- Noise parameters (from calibration) ---
    double gyro_noise_density_   = 0.00035;  // rad/s/sqrt(Hz)
    double gyro_random_walk_     = 1e-5;     // rad/s^2/sqrt(Hz)
    double accel_noise_density_  = 0.00667;  // m/s^2/sqrt(Hz)
    double accel_random_walk_    = 0.00068;  // m/s^3/sqrt(Hz)

    // --- Nominal state ---
    // Quaternion [w, x, y, z] sensor-to-world
    double qw_ = 1.0, qx_ = 0.0, qy_ = 0.0, qz_ = 0.0;
    // Position in world frame (m)
    double px_ = 0.0, py_ = 0.0, pz_ = 0.0;
    // Velocity in world frame (m/s)
    double vx_ = 0.0, vy_ = 0.0, vz_ = 0.0;
    // Gyro bias (rad/s) — body frame
    double bgx_ = 0.0, bgy_ = 0.0, bgz_ = 0.0;
    // Accel bias (m/s^2) — body frame
    double bax_ = 0.0, bay_ = 0.0, baz_ = 0.0;
    // Gravity vector in world frame (m/s^2)
    double gx_ = 0.0, gy_ = -9.81, gz_ = 0.0;

    // --- 15x15 error-state covariance ---
    // Order: [dtheta(3), dp(3), dv(3), dbg(3), dba(3)]
    Eigen::Matrix<double, 15, 15> P_;

    bool initialized_ = false;

    // --- Stationary initialization ---
    enum class InitPhase { Collecting, Done };
    InitPhase init_phase_ = InitPhase::Collecting;
    std::vector<std::array<float, 3>> init_accel_buf_;
    std::vector<std::array<float, 3>> init_gyro_buf_;
    int init_samples_collected_ = 0;
    double last_accel_var_ = 0.0;
    static constexpr int kInitSamplesRequired = 1000; // 1.0s at 1kHz
    static constexpr int kInitMaxSamples = 2000;
    static constexpr double kInitAccelVarThreshold = 0.002; // m^2/s^4

    bool try_initialize();

    // --- Quaternion helpers ---
    static void quat_multiply(double aw, double ax, double ay, double az,
                               double bw, double bx, double by, double bz,
                               double& rw, double& rx, double& ry, double& rz);
    static void quat_from_axis_angle(double wx, double wy, double wz, double angle,
                                      double& qw, double& qx, double& qy, double& qz);
};

} // namespace xreal
