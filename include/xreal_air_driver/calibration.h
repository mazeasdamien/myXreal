#pragma once
#include "xreal_air_driver/simd_math.h"
#include <string>
#include <vector>
#include <array>

#if defined(_WIN32)
  #if defined(xreal_air_driver_EXPORTS)
    #define XREAL_EXPORT __declspec(dllexport)
  #else
    #define XREAL_EXPORT __declspec(dllimport)
  #endif
#else
  #define XREAL_EXPORT __attribute__((visibility("default")))
#endif

namespace xreal {

struct TempBiasSample {
    float temp;
    std::array<float, 3> bias;
};

class XREAL_EXPORT Calibration {
public:
    Calibration();
    bool load_from_json(const std::string& filepath);
    bool load_from_json_string(const std::string& json_str);
    void apply(const std::array<float, 3>& raw_accel,
               const std::array<float, 3>& raw_gyro,
               float temp_c,
               std::array<float, 3>& out_accel,
               std::array<float, 3>& out_gyro) const;

    bool is_loaded() const { return loaded_; }

    const std::array<float, 4>& imu_noises() const { return imu_noises_; }
    const std::array<float, 9>& accl_calib_mat() const { return accl_calib_mat_; }
    const std::array<float, 9>& gyro_calib_mat() const { return gyro_calib_mat_; }

private:
    std::array<float, 3> accel_bias_ = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyro_bias_ = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> scale_accel_ = {1.0f, 1.0f, 1.0f};
    std::array<float, 3> scale_gyro_ = {1.0f, 1.0f, 1.0f};
    std::array<float, 4> accel_q_gyro_ = {1.0f, 0.0f, 0.0f, 0.0f}; // [w, x, y, z]
    std::vector<TempBiasSample> gyro_bias_temp_data_;
    std::array<float, 4> imu_noises_ = {0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 9> accl_calib_mat_ = {1,0,0, 0,1,0, 0,0,1};
    std::array<float, 9> gyro_calib_mat_ = {1,0,0, 0,1,0, 0,0,1};
    bool loaded_ = false;

    float4 accel_bias_simd_;
    float4 scale_accel_simd_;
    float4 scale_gyro_simd_;
    float4 accel_q_gyro_simd_;

    float4 get_interpolated_gyro_bias(float temp_c) const;
};

} // namespace xreal
