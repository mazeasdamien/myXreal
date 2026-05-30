#pragma once
#include <array>

namespace xreal {

class FusionFilter {
public:
    virtual ~FusionFilter() = default;
    virtual void update(float dt, const std::array<float, 3>& accel, const std::array<float, 3>& gyro) = 0;
    virtual std::array<float, 4> get_quaternion() const = 0; // [w, x, y, z]
};

class MahonyFilter : public FusionFilter {
public:
    MahonyFilter(float kp = 0.5f, float ki = 0.0f);
    void update(float dt, const std::array<float, 3>& accel, const std::array<float, 3>& gyro) override;
    std::array<float, 4> get_quaternion() const override;
    void reset(const std::array<float, 4>& q = {1.0f, 0.0f, 0.0f, 0.0f});

private:
    float kp_;
    float ki_;
    std::array<float, 4> q_ = {1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> integral_fb_ = {0.0f, 0.0f, 0.0f};
};

class MadgwickFilter : public FusionFilter {
public:
    MadgwickFilter(float beta = 0.041f);
    void update(float dt, const std::array<float, 3>& accel, const std::array<float, 3>& gyro) override;
    std::array<float, 4> get_quaternion() const override;

private:
    float beta_;
    std::array<float, 4> q_ = {1.0f, 0.0f, 0.0f, 0.0f};
};

// Converts quaternion [w, x, y, z] to roll, pitch, yaw in degrees
std::array<float, 3> quaternion_to_euler(const std::array<float, 4>& q);

} // namespace xreal
