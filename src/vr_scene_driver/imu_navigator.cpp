#include "xreal_air_driver/imu_navigator.h"
#include "xreal_air_driver/calibration.h"
#include <cmath>
#include <iostream>
#include <algorithm>

namespace xreal {

ImuNavigator::ImuNavigator() {
    reset();
}

void ImuNavigator::configure(const Calibration& calib) {
    const auto& noises = calib.imu_noises();
    gyro_noise_density_  = noises[0] > 0.0 ? noises[0] : 0.00035;
    gyro_random_walk_    = noises[1] > 0.0 ? noises[1] : 1e-5;
    accel_noise_density_ = noises[2] > 0.0 ? noises[2] : 0.00667;
    accel_random_walk_   = noises[3] > 0.0 ? noises[3] : 0.00068;
}

void ImuNavigator::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    qw_ = 1.0; qx_ = 0.0; qy_ = 0.0; qz_ = 0.0;
    px_ = py_ = pz_ = 0.0;
    vx_ = vy_ = vz_ = 0.0;
    bgx_ = bgy_ = bgz_ = 0.0;
    bax_ = bay_ = baz_ = 0.0;
    gx_ = 0.0; gy_ = -9.81; gz_ = 0.0;

    P_.setIdentity();
    P_.diagonal().segment<3>(0)  *= 1.0;    // orientation uncertainty ~1 rad
    P_.diagonal().segment<3>(3)  *= 100.0;  // position uncertainty ~100 m
    P_.diagonal().segment<3>(6)  *= 0.0001; // velocity uncertainty ~0.01 m/s
    P_.diagonal().segment<3>(9)  *= 0.0001; // gyro bias uncertainty ~0.01 rad/s
    P_.diagonal().segment<3>(12) *= 0.01;   // accel bias uncertainty ~0.1 m/s^2

    initialized_ = false;
    init_phase_ = InitPhase::Collecting;
    init_accel_buf_.clear();
    init_gyro_buf_.clear();
    init_samples_collected_ = 0;
    last_accel_var_ = 0.0;
}

float ImuNavigator::init_progress() const {
    if (initialized_) return 1.0f;
    return static_cast<float>(init_samples_collected_) / static_cast<float>(kInitSamplesRequired);
}

// --- Quaternion helpers ---

void ImuNavigator::quat_multiply(double aw, double ax, double ay, double az,
                                  double bw, double bx, double by, double bz,
                                  double& rw, double& rx, double& ry, double& rz) {
    rw = aw*bw - ax*bx - ay*by - az*bz;
    rx = aw*bx + ax*bw + ay*bz - az*by;
    ry = aw*by - ax*bz + ay*bw + az*bx;
    rz = aw*bz + ax*by - ay*bx + az*bw;
}

void ImuNavigator::quat_from_axis_angle(double wx, double wy, double wz, double angle,
                                         double& qw, double& qx, double& qy, double& qz) {
    double half = angle * 0.5;
    double s = std::sin(half);
    qw = std::cos(half);
    qx = wx * s;
    qy = wy * s;
    qz = wz * s;
}

// --- Quaternion to rotation matrix (body-to-world) ---
static Eigen::Matrix3d quat_to_rot(double qw, double qx, double qy, double qz) {
    Eigen::Matrix3d R;
    R(0,0) = 1.0 - 2.0*(qy*qy + qz*qz);
    R(0,1) = 2.0*(qx*qy - qw*qz);
    R(0,2) = 2.0*(qx*qz + qw*qy);
    R(1,0) = 2.0*(qx*qy + qw*qz);
    R(1,1) = 1.0 - 2.0*(qx*qx + qz*qz);
    R(1,2) = 2.0*(qy*qz - qw*qx);
    R(2,0) = 2.0*(qx*qz - qw*qy);
    R(2,1) = 2.0*(qy*qz + qw*qx);
    R(2,2) = 1.0 - 2.0*(qx*qx + qy*qy);
    return R;
}

// --- Skew-symmetric matrix from 3-vector ---
static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<    0.0, -v.z(),  v.y(),
          v.z(),    0.0, -v.x(),
         -v.y(),  v.x(),    0.0;
    return S;
}

// --- Main update ---

void ImuNavigator::update(float dt, const std::array<float, 3>& accel,
                           const std::array<float, 3>& gyro) {
    // Clamp dt to sane range
    if (dt <= 0.0 || dt > 0.1) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // --- Stationary initialization ---
    if (!initialized_) {
        init_accel_buf_.push_back(accel);
        init_gyro_buf_.push_back(gyro);
        init_samples_collected_++;

        if (init_samples_collected_ >= kInitMaxSamples) {
            // Force initialize with what we have
            try_initialize();
        } else if (init_samples_collected_ >= kInitSamplesRequired) {
            // Check rest condition every 50 samples after hitting the minimum
            if (init_samples_collected_ % 50 == 0) {
                try_initialize();
            }
        }
        // If not initialized yet, accumulate more samples
        if (!initialized_) return;
    }

    // Convert to doubles
    double ax = accel[0], ay = accel[1], az = accel[2];
    double wx = gyro[0],  wy = gyro[1],  wz = gyro[2];

    // --- Bias correction ---
    double wcx = wx - bgx_;
    double wcy = wy - bgy_;
    double wcz = wz - bgz_;
    double acx = ax - bax_;
    double acy = ay - bay_;
    double acz = az - baz_;

    // --- Quaternion integration (exponential map) ---
    double w_norm = std::sqrt(wcx*wcx + wcy*wcy + wcz*wcz);
    double angle = w_norm * dt;
    double dqw, dqx, dqy, dqz;
    if (angle > 1e-9) {
        double s = std::sin(angle * 0.5) / w_norm;
        dqw = std::cos(angle * 0.5);
        dqx = wcx * s;
        dqy = wcy * s;
        dqz = wcz * s;
    } else {
        dqw = 1.0; dqx = 0.0; dqy = 0.0; dqz = 0.0;
    }

    double new_qw, new_qx, new_qy, new_qz;
    quat_multiply(qw_, qx_, qy_, qz_, dqw, dqx, dqy, dqz, new_qw, new_qx, new_qy, new_qz);

    // Normalize quaternion
    double qn = std::sqrt(new_qw*new_qw + new_qx*new_qx + new_qy*new_qy + new_qz*new_qz);
    qw_ = new_qw / qn; qx_ = new_qx / qn; qy_ = new_qy / qn; qz_ = new_qz / qn;

    // --- Rotation matrix from quaternion ---
    Eigen::Matrix3d R = quat_to_rot(qw_, qx_, qy_, qz_);

    // --- Rotate accel to world, subtract gravity ---
    Eigen::Vector3d a_body(acx, acy, acz);
    Eigen::Vector3d a_world = R * a_body;
    a_world(0) -= gx_;
    a_world(1) -= gy_;
    a_world(2) -= gz_;

    // --- Velocity and position integration (midpoint) ---
    px_ += vx_ * dt + 0.5 * a_world(0) * dt * dt;
    py_ += vy_ * dt + 0.5 * a_world(1) * dt * dt;
    pz_ += vz_ * dt + 0.5 * a_world(2) * dt * dt;
    vx_ += a_world(0) * dt;
    vy_ += a_world(1) * dt;
    vz_ += a_world(2) * dt;

    // --- Covariance propagation (15x15 error-state) ---
    // Continuous-time F matrix blocks

    // F_theta_theta = -[w_corrected]×
    Eigen::Matrix3d F_th_th = -skew(Eigen::Vector3d(wcx, wcy, wcz));

    // F_theta_bg = -I3
    Eigen::Matrix3d F_th_bg = -Eigen::Matrix3d::Identity();

    // F_v_theta = -R * [a_corrected]×
    Eigen::Matrix3d F_v_th = -R * skew(a_body);

    // F_v_ba = -R
    Eigen::Matrix3d F_v_ba = -R;

    // Build F (15x15)
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();

    // Row 0-2: dtheta
    F.block<3,3>(0, 0) = F_th_th;
    F.block<3,3>(0, 9) = F_th_bg;

    // Row 3-5: dp = dv
    F.block<3,3>(3, 6) = Eigen::Matrix3d::Identity();

    // Row 6-8: dv
    F.block<3,3>(6, 0) = F_v_th;
    F.block<3,3>(6,12) = F_v_ba;

    // Row 9-11: dbg = 0 (random walk, no deterministic part)
    // Row 12-14: dba = 0

    // Discretize: Phi ≈ I + F*dt
    Eigen::Matrix<double, 15, 15> Phi = Eigen::Matrix<double, 15, 15>::Identity() + F * dt;

    // Process noise (continuous-time spectral densities)
    double sg2  = gyro_noise_density_  * gyro_noise_density_;
    double sa2  = accel_noise_density_ * accel_noise_density_;
    double sbg2 = gyro_random_walk_    * gyro_random_walk_;
    double sba2 = accel_random_walk_   * accel_random_walk_;

    // G * Qc * G^T * dt
    // G has block structure applied to noise vector [ng, na, nbg, nba]
    // ng -> -I3 on dtheta
    // na -> -R on dv
    // nbg -> +I3 on dbg
    // nba -> +I3 on dba
    Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();

    // dtheta block from gyro noise: (-I3)*sg2*(-I3)^T = sg2*I3
    Qd.block<3,3>(0,0) = sg2 * dt * Eigen::Matrix3d::Identity();

    // dv block from accel noise: (-R)*sa2*(-R)^T = sa2 * R*R^T = sa2*I3
    Qd.block<3,3>(6,6) = sa2 * dt * Eigen::Matrix3d::Identity();

    // dbg block from gyro bias random walk
    Qd.block<3,3>(9,9) = sbg2 * dt * Eigen::Matrix3d::Identity();

    // dba block from accel bias random walk
    Qd.block<3,3>(12,12) = sba2 * dt * Eigen::Matrix3d::Identity();

    // P = Phi * P * Phi^T + Qd
    P_ = Phi * P_ * Phi.transpose() + Qd;

    // Enforce symmetry
    P_ = 0.5 * (P_ + P_.transpose());
}

bool ImuNavigator::try_initialize() {
    const size_t n = init_accel_buf_.size();
    if (n < static_cast<size_t>(kInitSamplesRequired)) return false;

    // Compute accel magnitude variance over the buffer
    double sum_mag = 0.0, sum_mag2 = 0.0;
    for (size_t i = 0; i < n; i++) {
        double mag = std::sqrt(init_accel_buf_[i][0]*init_accel_buf_[i][0] +
                               init_accel_buf_[i][1]*init_accel_buf_[i][1] +
                               init_accel_buf_[i][2]*init_accel_buf_[i][2]);
        sum_mag  += mag;
        sum_mag2 += mag * mag;
    }
    double mean_mag = sum_mag / n;
    double var_mag  = sum_mag2 / n - mean_mag * mean_mag;
    last_accel_var_ = var_mag;

    if (var_mag > kInitAccelVarThreshold) {
        // Headset is moving — keep collecting
        // Drop oldest quarter of buffer to chase a new rest period
        size_t drop = n / 4;
        init_accel_buf_.erase(init_accel_buf_.begin(), init_accel_buf_.begin() + drop);
        init_gyro_buf_.erase(init_gyro_buf_.begin(), init_gyro_buf_.begin() + drop);
        init_samples_collected_ = static_cast<int>(init_accel_buf_.size());
        return false;
    }

    // --- At rest: estimate biases and gravity ---

    // Average gyro → initial bg estimate
    double sum_wx = 0.0, sum_wy = 0.0, sum_wz = 0.0;
    double sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0;
    for (size_t i = 0; i < n; i++) {
        sum_wx += init_gyro_buf_[i][0];
        sum_wy += init_gyro_buf_[i][1];
        sum_wz += init_gyro_buf_[i][2];
        sum_ax += init_accel_buf_[i][0];
        sum_ay += init_accel_buf_[i][1];
        sum_az += init_accel_buf_[i][2];
    }
    bgx_ = sum_wx / n;
    bgy_ = sum_wy / n;
    bgz_ = sum_wz / n;

    double ax_avg = sum_ax / n;
    double ay_avg = sum_ay / n;
    double az_avg = sum_az / n;

    // Gravity direction in sensor frame = measured reaction force
    // (accel at rest measures upward reaction = -gravity in sensor frame)
    double a_norm = std::sqrt(ax_avg*ax_avg + ay_avg*ay_avg + az_avg*az_avg);
    if (a_norm < 1.0) {
        // Abnormal — accel magnitude too low, skip init
        return false;
    }

    Eigen::Vector3d g_meas(ax_avg / a_norm, ay_avg / a_norm, az_avg / a_norm);

    // World gravity direction is [0, -1, 0] (+Y up)
    Eigen::Vector3d g_world(0.0, -1.0, 0.0);

    // Find rotation that aligns g_meas to g_world
    Eigen::Vector3d axis = g_meas.cross(g_world);
    double axis_norm = axis.norm();

    if (axis_norm > 1e-9) {
        axis /= axis_norm;
        double cos_angle = g_meas.dot(g_world);
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);
        double angle = std::acos(cos_angle);
        quat_from_axis_angle(axis(0), axis(1), axis(2), angle, qw_, qx_, qy_, qz_);
    } else {
        // Parallel or anti-parallel
        if (g_meas.dot(g_world) > 0.0) {
            qw_ = 1.0; qx_ = 0.0; qy_ = 0.0; qz_ = 0.0;
        } else {
            // 180 degrees — pick arbitrary perpendicular axis
            qw_ = 0.0; qx_ = 1.0; qy_ = 0.0; qz_ = 0.0;
        }
    }

    // Set gravity magnitude from measured accel
    double g_mag = a_norm;
    gx_ = 0.0;
    gy_ = -g_mag;
    gz_ = 0.0;

    // Estimate accel bias from the residual between measured specific force
    // and expected specific force at rest (reaction force = +R^T * [0, g_mag, 0]^T)
    // At rest: meas = -g_body + ba  →  ba = meas + g_body = meas + R^T * [0, g_mag, 0]^T
    {
        Eigen::Matrix3d R_init = quat_to_rot(qw_, qx_, qy_, qz_);
        Eigen::Vector3d g_world_vec(0.0, g_mag, 0.0);
        Eigen::Vector3d g_body = R_init.transpose() * g_world_vec;
        Eigen::Vector3d ba_est(ax_avg, ay_avg, az_avg);
        ba_est += g_body; // ba = a_meas + g_body
        bax_ = ba_est(0);
        bay_ = ba_est(1);
        baz_ = ba_est(2);
    }

    // Reset position and velocity (we are at rest at initialization)
    px_ = py_ = pz_ = 0.0;
    vx_ = vy_ = vz_ = 0.0;

    initialized_ = true;
    init_phase_ = InitPhase::Done;

    std::cout << "[ImuNavigator] Initialized: bg=[" << bgx_ << ", " << bgy_ << ", " << bgz_
              << "] rad/s, g_mag=" << g_mag << " m/s^2, var=" << var_mag << std::endl;

    return true;
}

// --- State accessors ---

std::array<float, 4> ImuNavigator::get_quaternion() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {static_cast<float>(qw_), static_cast<float>(qx_),
            static_cast<float>(qy_), static_cast<float>(qz_)};
}

std::array<float, 3> ImuNavigator::get_position() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {static_cast<float>(px_), static_cast<float>(py_), static_cast<float>(pz_)};
}

std::array<float, 3> ImuNavigator::get_velocity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {static_cast<float>(vx_), static_cast<float>(vy_), static_cast<float>(vz_)};
}

std::array<float, 3> ImuNavigator::get_gyro_bias() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {static_cast<float>(bgx_), static_cast<float>(bgy_), static_cast<float>(bgz_)};
}

std::array<float, 3> ImuNavigator::get_accel_bias() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {static_cast<float>(bax_), static_cast<float>(bay_), static_cast<float>(baz_)};
}

std::array<float, 3> ImuNavigator::get_gravity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {static_cast<float>(gx_), static_cast<float>(gy_), static_cast<float>(gz_)};
}

std::array<float, 15> ImuNavigator::get_covariance_diag() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::array<float, 15> diag;
    for (int i = 0; i < 15; i++) {
        diag[i] = static_cast<float>(P_(i, i));
    }
    return diag;
}

InertialState ImuNavigator::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    InertialState s;
    s.q_WB[0] = qw_; s.q_WB[1] = qx_; s.q_WB[2] = qy_; s.q_WB[3] = qz_;
    s.p_WB[0] = px_; s.p_WB[1] = py_; s.p_WB[2] = pz_;
    s.v_WB[0] = vx_; s.v_WB[1] = vy_; s.v_WB[2] = vz_;
    s.bg[0] = bgx_; s.bg[1] = bgy_; s.bg[2] = bgz_;
    s.ba[0] = bax_; s.ba[1] = bay_; s.ba[2] = baz_;
    s.g_W[0] = gx_; s.g_W[1] = gy_; s.g_W[2] = gz_;
    for (int i = 0; i < 15; i++) s.cov_diag[i] = P_(i, i);
    return s;
}

InertialState ImuNavigator::get_state(uint64_t timestamp_ns) const {
    auto s = get_state();
    s.timestamp_ns = timestamp_ns;
    return s;
}

double ImuNavigator::get_accel_variance() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_accel_var_;
}

} // namespace xreal
