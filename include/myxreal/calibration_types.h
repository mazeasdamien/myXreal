#pragma once

#include <Eigen/Dense>
#include <cstdint>

// ---------------------------------------------------------------------------
// Camera model enumeration
// ---------------------------------------------------------------------------
enum class CameraModel : int {
    Pinhole    = 0,  // OpenCV-compatible: k1,k2,p1,p2,k3
    Fisheye624 = 1,  // XREAL factory: 12-param (6 radial + 2 tangential + 4 thin-prism)
    Fisheye4   = 2,  // OpenCV fisheye: 4-param Kannala-Brandt (k1..k4)
};

// ---------------------------------------------------------------------------
// Camera intrinsics — supports both OpenCV pinhole and XREAL fisheye624 models
// ---------------------------------------------------------------------------
struct CameraIntrinsics {
    double fx = 0, fy = 0, cx = 0, cy = 0;
    int    width = 0, height = 0;
    CameraModel model = CameraModel::Pinhole;

    // OpenCV pinhole params (used when model == Pinhole)
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;

    // Fisheye624 params (used when model == Fisheye624)
    // kc[0..5] = radial (theta polynomial), kc[6..7] = tangential, kc[8..11] = thin-prism
    double kc[12] = {};

    // Fisheye4 params (used when model == Fisheye4)
    // Kannala-Brandt distortion: k1, k2, k3, k4
    double fisheye_k[4] = {-0.15, 0.02, -0.005, 0.001};

    bool has_valid_focal() const { return fx > 0 && fy > 0; }
    bool has_valid_center() const { return cx > 0 && cy > 0; }
};

// ---------------------------------------------------------------------------
// Camera extrinsics — transform from camera frame to IMU frame
//   p_imu = R * p_cam + t
// ---------------------------------------------------------------------------
struct CameraExtrinsics {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero(); // metres
};

struct DisplayExtrinsics {
    Eigen::Matrix3d R_imu_display = Eigen::Matrix3d::Identity(); // p_imu = R * p_display + t
    Eigen::Vector3d t_imu_display = Eigen::Vector3d::Zero();      // metres
    bool valid = false;
};

// ---------------------------------------------------------------------------
// IMU noise / misalignment parameters (standard VIO model)
// ---------------------------------------------------------------------------
struct ImuIntrinsics {
    double accel_noise_density = 0.0;   // m/s²/√Hz
    double accel_random_walk   = 0.0;   // m/s³/√Hz
    double gyro_noise_density  = 0.0;   // rad/s/√Hz
    double gyro_random_walk    = 0.0;   // rad/s²/√Hz
    Eigen::Matrix3d T_imu_acc = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d T_imu_gyr = Eigen::Matrix3d::Identity();
};

// ---------------------------------------------------------------------------
// Validation report for a single check
// ---------------------------------------------------------------------------
struct CalibCheck {
    const char* name;
    double      value;
    double      min_val;
    double      max_val;
    bool        pass;
};

struct CalibValidation {
    CalibCheck checks[24];
    int        count = 0;
    int        failed = 0;

    void add(const char* name, double value, double min_val, double max_val) {
        if (count >= 24) return;
        bool ok = (value >= min_val && value <= max_val);
        checks[count] = {name, value, min_val, max_val, ok};
        if (!ok) failed++;
        count++;
    }
};

// ---------------------------------------------------------------------------
// Aggregate calibration data
// ---------------------------------------------------------------------------
struct CalibrationData {
    CameraIntrinsics left;
    CameraIntrinsics right;
    CameraExtrinsics T_left_imu;
    CameraExtrinsics T_right_imu;
    DisplayExtrinsics T_left_display_imu;   // display frame -> IMU frame
    DisplayExtrinsics T_right_display_imu;  // display frame -> IMU frame
    Eigen::Matrix4d  T_left_right   = Eigen::Matrix4d::Identity();
    double           baseline_m     = 0.0;
    ImuIntrinsics    imu;
    double           cam_imu_time_offset_ns = 0.0;
    bool             is_valid       = false;

    CalibValidation  validation;
};
