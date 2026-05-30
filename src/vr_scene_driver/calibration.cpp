#include "xreal_air_driver/calibration.h"
#include "xreal_air_driver/simd_math.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace xreal {

static std::array<float, 3> rotate_vector_by_quaternion(const std::array<float, 4>& q, const std::array<float, 3>& v) {
    // q is [x, y, z, w] in standard calibration layout
    float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    float vx = v[0], vy = v[1], vz = v[2];

    // t = 2 * cross(q_xyz, v)
    float tx = 2.0f * (qy * vz - qz * vy);
    float ty = 2.0f * (qz * vx - qx * vz);
    float tz = 2.0f * (qx * vy - qy * vx);

    // v_rot = v + qw * t + cross(q_xyz, t)
    float rx = vx + qw * tx + (qy * tz - qz * ty);
    float ry = vy + qw * ty + (qz * tx - qx * tz);
    float rz = vz + qw * tz + (qx * ty - qy * tx);

    return {rx, ry, rz};
}

Calibration::Calibration() {
    accel_bias_simd_ = float4(accel_bias_[0], accel_bias_[1], accel_bias_[2], 0.0f);
    scale_accel_simd_ = float4(scale_accel_[0], scale_accel_[1], scale_accel_[2], 0.0f);
    scale_gyro_simd_ = float4(scale_gyro_[0], scale_gyro_[1], scale_gyro_[2], 0.0f);
    accel_q_gyro_simd_ = float4(accel_q_gyro_[0], accel_q_gyro_[1], accel_q_gyro_[2], accel_q_gyro_[3]);
}

bool Calibration::load_from_json(const std::string& filepath) {
    try {
        std::ifstream f(filepath);
        if (!f.is_open()) {
            std::cerr << "[Calibration] Failed to open: " << filepath << std::endl;
            return false;
        }

        json data;
        f >> data;

        auto dev1 = data.at("IMU").at("device_1");

        // Parse biases
        accel_bias_ = dev1.at("accel_bias").get<std::array<float, 3>>();
        gyro_bias_ = dev1.at("gyro_bias").get<std::array<float, 3>>();

        // Parse scales
        scale_accel_ = dev1.at("scale_accel").get<std::array<float, 3>>();
        scale_gyro_ = dev1.at("scale_gyro").get<std::array<float, 3>>();

        // Parse alignment quaternions
        accel_q_gyro_ = dev1.at("accel_q_gyro").get<std::array<float, 4>>();

        // Populate SIMD variables
        accel_bias_simd_ = float4(accel_bias_[0], accel_bias_[1], accel_bias_[2], 0.0f);
        scale_accel_simd_ = float4(scale_accel_[0], scale_accel_[1], scale_accel_[2], 0.0f);
        scale_gyro_simd_ = float4(scale_gyro_[0], scale_gyro_[1], scale_gyro_[2], 0.0f);
        accel_q_gyro_simd_ = float4(accel_q_gyro_[0], accel_q_gyro_[1], accel_q_gyro_[2], accel_q_gyro_[3]);

        // Parse temp bias table
        gyro_bias_temp_data_.clear();
        if (dev1.contains("gyro_bias_temp_data")) {
            for (const auto& item : dev1.at("gyro_bias_temp_data")) {
                TempBiasSample sample;
                sample.temp = item.at("temp").get<float>();
                sample.bias = item.at("bias").get<std::array<float, 3>>();
                gyro_bias_temp_data_.push_back(sample);
            }
            // Ensure sorted by temperature for linear interpolation
            std::sort(gyro_bias_temp_data_.begin(), gyro_bias_temp_data_.end(),
                      [](const TempBiasSample& a, const TempBiasSample& b) {
                          return a.temp < b.temp;
                      });
        }

        // Parse IMU noise parameters and calibration matrices (optional)
        if (dev1.contains("imu_noises")) {
            imu_noises_ = dev1.at("imu_noises").get<std::array<float, 4>>();
        }
        if (dev1.contains("imu_intrinsics")) {
            const auto& intr = dev1.at("imu_intrinsics");
            if (intr.contains("accl_calib_mat")) {
                accl_calib_mat_ = intr.at("accl_calib_mat").get<std::array<float, 9>>();
            }
            if (intr.contains("gyro_calib_mat")) {
                gyro_calib_mat_ = intr.at("gyro_calib_mat").get<std::array<float, 9>>();
            }
        }

        loaded_ = true;
        std::cout << "[Calibration] Loaded factory calibration successfully from " << filepath << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Calibration] Exception during loading: " << e.what() << ". Falling back to identity." << std::endl;
        loaded_ = false;
        return false;
    }
}

bool Calibration::load_from_json_string(const std::string& json_str) {
    try {
        json data = json::parse(json_str);

        auto dev1 = data.at("IMU").at("device_1");

        // Parse biases
        accel_bias_ = dev1.at("accel_bias").get<std::array<float, 3>>();
        gyro_bias_ = dev1.at("gyro_bias").get<std::array<float, 3>>();

        // Parse scales
        scale_accel_ = dev1.at("scale_accel").get<std::array<float, 3>>();
        scale_gyro_ = dev1.at("scale_gyro").get<std::array<float, 3>>();

        // Parse alignment quaternions
        accel_q_gyro_ = dev1.at("accel_q_gyro").get<std::array<float, 4>>();

        // Populate SIMD variables
        accel_bias_simd_ = float4(accel_bias_[0], accel_bias_[1], accel_bias_[2], 0.0f);
        scale_accel_simd_ = float4(scale_accel_[0], scale_accel_[1], scale_accel_[2], 0.0f);
        scale_gyro_simd_ = float4(scale_gyro_[0], scale_gyro_[1], scale_gyro_[2], 0.0f);
        accel_q_gyro_simd_ = float4(accel_q_gyro_[0], accel_q_gyro_[1], accel_q_gyro_[2], accel_q_gyro_[3]);

        // Parse temp bias table
        gyro_bias_temp_data_.clear();
        if (dev1.contains("gyro_bias_temp_data")) {
            for (const auto& item : dev1.at("gyro_bias_temp_data")) {
                TempBiasSample sample;
                sample.temp = item.at("temp").get<float>();
                sample.bias = item.at("bias").get<std::array<float, 3>>();
                gyro_bias_temp_data_.push_back(sample);
            }
            std::sort(gyro_bias_temp_data_.begin(), gyro_bias_temp_data_.end(),
                      [](const TempBiasSample& a, const TempBiasSample& b) {
                          return a.temp < b.temp;
                      });
        }

        // Parse IMU noise parameters and calibration matrices (optional)
        if (dev1.contains("imu_noises")) {
            imu_noises_ = dev1.at("imu_noises").get<std::array<float, 4>>();
        }
        if (dev1.contains("imu_intrinsics")) {
            const auto& intr = dev1.at("imu_intrinsics");
            if (intr.contains("accl_calib_mat")) {
                accl_calib_mat_ = intr.at("accl_calib_mat").get<std::array<float, 9>>();
            }
            if (intr.contains("gyro_calib_mat")) {
                gyro_calib_mat_ = intr.at("gyro_calib_mat").get<std::array<float, 9>>();
            }
        }

        loaded_ = true;
        std::cout << "[Calibration] Loaded factory calibration successfully from on-device download." << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Calibration] Exception during parsing JSON string: " << e.what() << ". Falling back to identity." << std::endl;
        loaded_ = false;
        return false;
    }
}


float4 Calibration::get_interpolated_gyro_bias(float temp_c) const {
    if (gyro_bias_temp_data_.empty()) {
        return float4(gyro_bias_.data());
    }

    if (temp_c <= gyro_bias_temp_data_.front().temp) {
        return float4(gyro_bias_temp_data_.front().bias.data());
    }
    if (temp_c >= gyro_bias_temp_data_.back().temp) {
        return float4(gyro_bias_temp_data_.back().bias.data());
    }

    auto it = std::lower_bound(gyro_bias_temp_data_.begin(), gyro_bias_temp_data_.end(), temp_c,
                               [](const TempBiasSample& s, float t) {
                                   return s.temp < t;
                               });

    const auto& t2 = *it;
    const auto& t1 = *(it - 1);
    float factor = (temp_c - t1.temp) / (t2.temp - t1.temp);
    float b_interp[3] = {
        t1.bias[0] + factor * (t2.bias[0] - t1.bias[0]),
        t1.bias[1] + factor * (t2.bias[1] - t1.bias[1]),
        t1.bias[2] + factor * (t2.bias[2] - t1.bias[2])
    };
    return float4(b_interp);
}

void Calibration::apply(const std::array<float, 3>& raw_accel,
                        const std::array<float, 3>& raw_gyro,
                        float temp_c,
                        std::array<float, 3>& out_accel,
                        std::array<float, 3>& out_gyro) const {
    float4 acc_raw(raw_accel.data());
    float4 gyro_raw(raw_gyro.data());

    // Step 1: Rotate gyro to accel reference frame using SIMD
    float4 aligned_gyro = rotate_vector_by_quaternion_simd(accel_q_gyro_simd_, gyro_raw);

    // Step 2: Convert to physical units using SIMD
    float4 acc_phy = acc_raw * 9.80665f;
    constexpr float DEG_TO_RAD = 3.141592653589793f / 180.0f;
    float4 gyro_phy = aligned_gyro * DEG_TO_RAD;

    // Step 3: Subtract bias vectors in aligned (raw sensor) coordinate system
    float4 gyro_bias_vec = get_interpolated_gyro_bias(temp_c);
    float4 acc_sub = acc_phy - accel_bias_simd_;
    float4 gyro_sub = gyro_phy - gyro_bias_vec;

    // Step 4: Scale factor multiplication in aligned (raw sensor) coordinate system
    float4 acc_scaled = acc_sub * scale_accel_simd_;
    float4 gyro_scaled = gyro_sub * scale_gyro_simd_;

    // Step 5: Map to pre-biased coordinate system: [x, y, z] -> [-x, -z, -y]
    float4 acc_pre = pre_bias_swap(acc_scaled);
    float4 gyro_pre = pre_bias_swap(gyro_scaled);

    // Step 6: Post-biased coordinate transformation: [x, y, z] -> [x, -y, -z]
    float4 acc_final = post_bias_swap(acc_pre);
    float4 gyro_final = post_bias_swap(gyro_pre);

    acc_final.store3(out_accel.data());
    gyro_final.store3(out_gyro.data());
}

} // namespace xreal
