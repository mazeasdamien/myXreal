#include "xreal_air_driver/sensor_fusion.h"
#include <cmath>
#include <algorithm>

namespace xreal {

// Mahony Filter
MahonyFilter::MahonyFilter(float kp, float ki) : kp_(kp), ki_(ki) {}

void MahonyFilter::update(float dt, const std::array<float, 3>& accel, const std::array<float, 3>& gyro) {
    float q1 = q_[0], q2 = q_[1], q3 = q_[2], q4 = q_[3];
    float ax = accel[0], ay = accel[1], az = accel[2];
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];

    // Normalize accelerometer measurement
    float norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Estimated direction of gravity (world +Y is up)
    float vx = 2.0f * (q2 * q3 + q1 * q4);
    float vy = 1.0f - 2.0f * (q2 * q2 + q4 * q4);
    float vz = 2.0f * (q3 * q4 - q1 * q2);

    // Error is cross product of measured and estimated gravity directions
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    // Apply integral feedback if enabled
    if (ki_ > 0.0f) {
        integral_fb_[0] += ex * ki_ * dt;
        integral_fb_[1] += ey * ki_ * dt;
        integral_fb_[2] += ez * ki_ * dt;
        gx += integral_fb_[0];
        gy += integral_fb_[1];
        gz += integral_fb_[2];
    }

    // Apply proportional feedback
    gx += ex * kp_;
    gy += ey * kp_;
    gz += ez * kp_;

    // Integrate quaternion rate
    float pa = q2, pb = q3, pc = q4;
    q_[0] += (-pa * gx - pb * gy - pc * gz) * (0.5f * dt);
    q_[1] += (q1 * gx + pb * gz - pc * gy) * (0.5f * dt);
    q_[2] += (q1 * gy - pa * gz + pc * gx) * (0.5f * dt);
    q_[3] += (q1 * gz + pa * gy - pb * gx) * (0.5f * dt);

    // Normalize quaternion
    norm = std::sqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
    q_[0] /= norm;
    q_[1] /= norm;
    q_[2] /= norm;
    q_[3] /= norm;
}

std::array<float, 4> MahonyFilter::get_quaternion() const {
    return q_;
}

void MahonyFilter::reset(const std::array<float, 4>& q) {
    q_ = q;
    integral_fb_ = {0.0f, 0.0f, 0.0f};
}

// Madgwick Filter
MadgwickFilter::MadgwickFilter(float beta) : beta_(beta) {}

void MadgwickFilter::update(float dt, const std::array<float, 3>& accel, const std::array<float, 3>& gyro) {
    float q1 = q_[0], q2 = q_[1], q3 = q_[2], q4 = q_[3];
    float ax = accel[0], ay = accel[1], az = accel[2];
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];

    // Normalize accelerometer measurement
    float norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Auxiliary variables
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    float q4q4 = q4 * q4;

    // Gradient descent step equations for Y-up gravity reference (f_g and J_g)
    float f_1 = _2q2 * q3 + _2q1 * q4 - ax;
    float f_2 = 1.0f - 2.0f * (q2q2 + q4q4) - ay;
    float f_3 = _2q3 * q4 - _2q1 * q2 - az;

    // Jacobian elements (J_g^T * f_g)
    float s1 = _2q4 * f_1 - _2q2 * f_3;
    float s2 = _2q3 * f_1 - 4.0f * q2 * f_2 - _2q1 * f_3;
    float s3 = _2q2 * f_1 + _2q4 * f_3;
    float s4 = _2q1 * f_1 - 4.0f * q4 * f_2 + _2q3 * f_3;

    // Normalize step size
    norm = std::sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    if (norm > 1e-6f) {
        s1 /= norm;
        s2 /= norm;
        s3 /= norm;
        s4 /= norm;
    }

    // Compute quaternion derivative
    float qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
    float qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
    float qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
    float qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

    // Integrate to yield quaternion
    q_[0] += qDot1 * dt;
    q_[1] += qDot2 * dt;
    q_[2] += qDot3 * dt;
    q_[3] += qDot4 * dt;

    // Normalize quaternion
    norm = std::sqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
    q_[0] /= norm;
    q_[1] /= norm;
    q_[2] /= norm;
    q_[3] /= norm;
}

std::array<float, 4> MadgwickFilter::get_quaternion() const {
    return q_;
}

// Convert quaternion to Euler angles
std::array<float, 3> quaternion_to_euler(const std::array<float, 4>& q) {
    float w = q[0], x = q[1], y = q[2], z = q[3];

    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w * y - z * x);
    float pitch = 0.0f;
    if (std::abs(sinp) >= 1.0f) {
        pitch = std::copysign(3.14159265f / 2.0f, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    constexpr float RAD_TO_DEG = 180.0f / 3.141592653589793f;
    return { roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG };
}

} // namespace xreal
