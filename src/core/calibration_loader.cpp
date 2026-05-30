#include "myxreal/calibration_loader.h"

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <cmath>
#include <cstring>
#include <mutex>
#include <fstream>

// =============================================================================
// Internal: undistort map storage (keeps OpenCV types out of the public header)
// =============================================================================
struct UndistortMaps {
    cv::Mat map1, map2;
};

static UndistortMaps g_undistort_left;
static UndistortMaps g_undistort_right;
static bool          g_maps_valid = false;

// =============================================================================
// Singleton
// =============================================================================
static CalibrationData* g_singleton = nullptr;
static std::mutex       g_singleton_mutex;

void calibration_set_singleton(CalibrationData* c) {
    std::lock_guard<std::mutex> lk(g_singleton_mutex);
    g_singleton = c;
}

CalibrationData* calibration_get_singleton() {
    std::lock_guard<std::mutex> lk(g_singleton_mutex);
    return g_singleton;
}

// =============================================================================
// Helpers
// =============================================================================

static bool parseMatrix(const YAML::Node& node, cv::Mat& m) {
    if (!node || !node["rows"] || !node["cols"] || !node["data"]) return false;
    int rows = node["rows"].as<int>();
    int cols = node["cols"].as<int>();
    m.create(rows, cols, CV_64FC1);
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            m.at<double>(r, c) = node["data"][r * cols + c].as<double>();
    return true;
}

static bool parseEigen3(const YAML::Node& node, Eigen::Matrix3d& R) {
    if (!node || !node["rows"] || !node["cols"]) return false;
    int rows = node["rows"].as<int>();
    int cols = node["cols"].as<int>();
    if (rows != 3 || cols != 3) return false;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R(r, c) = node["data"][r * 3 + c].as<double>();
    return true;
}

static bool parseEigenVec3(const YAML::Node& node, Eigen::Vector3d& v) {
    if (!node || !node["rows"] || !node["cols"]) return false;
    int rows = node["rows"].as<int>();
    int cols = node["cols"].as<int>();
    int n = std::max(rows, cols);
    if (n < 3) return false;
    for (int i = 0; i < 3; ++i)
        v(i) = node["data"][i].as<double>();
    return true;
}

// Parse a 4x4 T_cam_imu from Kalibr YAML (list of 4 lists)
static bool parseKalibrTransform(const YAML::Node& node, Eigen::Matrix4d& T) {
    if (!node || node.size() != 4) return false;
    for (int r = 0; r < 4; ++r) {
        if (node[r].size() != 4) return false;
        for (int c = 0; c < 4; ++c)
            T(r, c) = node[r][c].as<double>();
    }
    return true;
}

static bool parseKalibrVec(const YAML::Node& node, int expected, std::vector<double>& out) {
    if (!node || node.size() != (size_t)expected) return false;
    out.resize(expected);
    for (int i = 0; i < expected; ++i)
        out[i] = node[i].as<double>();
    return true;
}

// =============================================================================
// Validation
// =============================================================================
static void validate(CalibrationData& c) {
    c.validation.count  = 0;
    c.validation.failed = 0;
    auto& v = c.validation;
    auto& L = c.left, &R = c.right;

    v.add("Left fx",           L.fx, 100.0, 10000.0);
    v.add("Left fy",           L.fy, 100.0, 10000.0);
    v.add("Left cx",           L.cx, 0.0, (double)L.width);
    v.add("Left cy",           L.cy, 0.0, (double)L.height);
    v.add("Left k1",           L.k1, -1.0, 1.0);
    v.add("Left FOV (fx/w)",   L.fx / (double)L.width, 0.5, 2.0);

    v.add("Right fx",          R.fx, 100.0, 10000.0);
    v.add("Right fy",          R.fy, 100.0, 10000.0);
    v.add("Right cx",          R.cx, 0.0, (double)R.width);
    v.add("Right cy",          R.cy, 0.0, (double)R.height);
    v.add("Right k1",          R.k1, -1.0, 1.0);
    v.add("Right FOV (fx/w)",  R.fx / (double)R.width, 0.5, 2.0);

    v.add("Baseline (mm)",     c.baseline_m * 1000.0, 20.0, 200.0);

    // IMU noise: 0 = not available (OK), > 0 must be in [min, max]
    v.add("Accel noise dens",  c.imu.accel_noise_density, 0.0, 1.0);
    v.add("Gyro noise dens",   c.imu.gyro_noise_density, 0.0, 1.0);
    v.add("Accel random walk", c.imu.accel_random_walk, 0.0, 1.0);
    v.add("Gyro random walk",  c.imu.gyro_random_walk, 0.0, 1.0);
}

// =============================================================================
// Format auto-detection and loading
// =============================================================================

static bool loadOpenCV(const YAML::Node& root, CalibrationData& c) {
    // OpenCV stereo format: M1, D1, M2, D2, R, T
    cv::Mat M1, D1, M2, D2, R, T;
    if (!parseMatrix(root["M1"], M1)) return false;
    if (!parseMatrix(root["M2"], M2)) return false;
    parseMatrix(root["D1"], D1);
    parseMatrix(root["D2"], D2);

    // Left
    c.left.fx = M1.at<double>(0, 0);
    c.left.fy = M1.at<double>(1, 1);
    c.left.cx = M1.at<double>(0, 2);
    c.left.cy = M1.at<double>(1, 2);
    if (!D1.empty()) {
        c.left.k1 = D1.at<double>(0, 0);
        c.left.k2 = (D1.cols > 1) ? D1.at<double>(0, 1) : 0;
        c.left.p1 = (D1.cols > 2) ? D1.at<double>(0, 2) : 0;
        c.left.p2 = (D1.cols > 3) ? D1.at<double>(0, 3) : 0;
        c.left.k3 = (D1.cols > 4) ? D1.at<double>(0, 4) : 0;
    }

    // Right
    c.right.fx = M2.at<double>(0, 0);
    c.right.fy = M2.at<double>(1, 1);
    c.right.cx = M2.at<double>(0, 2);
    c.right.cy = M2.at<double>(1, 2);
    if (!D2.empty()) {
        c.right.k1 = D2.at<double>(0, 0);
        c.right.k2 = (D2.cols > 1) ? D2.at<double>(0, 1) : 0;
        c.right.p1 = (D2.cols > 2) ? D2.at<double>(0, 2) : 0;
        c.right.p2 = (D2.cols > 3) ? D2.at<double>(0, 3) : 0;
        c.right.k3 = (D2.cols > 4) ? D2.at<double>(0, 4) : 0;
    }

    // Resolution — try explicit fields, fall back to image_width/image_height
    if (root["image_width"] && root["image_height"]) {
        c.left.width  = root["image_width"].as<int>();
        c.left.height = root["image_height"].as<int>();
        c.right.width  = c.left.width;
        c.right.height = c.left.height;
    } else {
        // Extrinsic-only file: caller must set width/height separately
        c.left.width  = c.left.width  ? c.left.width  : 640;
        c.left.height = c.left.height ? c.left.height : 480;
        c.right.width  = c.left.width;
        c.right.height = c.left.height;
    }

    // Stereo extrinsics R, T (left → right in OpenCV convention)
    if (parseMatrix(root["R"], R) && parseMatrix(root["T"], T)) {
        // OpenCV stereo: R_cv, T_cv describe right cam relative to left cam
        // R is 3x3 rotation, T is 3x1 translation
        c.T_left_right = Eigen::Matrix4d::Identity();
        for (int r = 0; r < 3; ++r) {
            for (int col = 0; col < 3; ++col)
                c.T_left_right(r, col) = R.at<double>(r, col);
            c.T_left_right(r, 3) = T.at<double>(r, 0);
        }
        c.baseline_m = std::sqrt(T.at<double>(0, 0) * T.at<double>(0, 0)
                               + T.at<double>(1, 0) * T.at<double>(1, 0)
                               + T.at<double>(2, 0) * T.at<double>(2, 0));

        // Fill T_left_imu, T_right_imu as identity (no IMU extrinsics in OpenCV fmt)
        c.T_left_imu.R  = Eigen::Matrix3d::Identity();
        c.T_left_imu.t  = Eigen::Vector3d::Zero();
        c.T_right_imu.R = c.T_left_imu.R;
        c.T_right_imu.t = c.T_left_imu.t;
    }

    return true;
}

static bool loadKalibr(const YAML::Node& root, CalibrationData& c) {
    // cam0 = left, cam1 = right (convention)
    if (!root["cam0"] || !root["cam1"]) return false;

    auto loadCam = [](const YAML::Node& node, CameraIntrinsics& ci, CameraExtrinsics& ce) -> bool {
        auto& intr = node["intrinsics"];
        if (!intr || intr.size() < 4) return false;
        ci.fx = intr[0].as<double>();
        ci.fy = intr[1].as<double>();
        ci.cx = intr[2].as<double>();
        ci.cy = intr[3].as<double>();

        auto& res = node["resolution"];
        if (res && res.size() >= 2) {
            ci.width  = res[0].as<int>();
            ci.height = res[1].as<int>();
        }

        auto& dist = node["distortion_coeffs"];
        if (dist && dist.size() >= 4) {
            ci.k1 = dist[0].as<double>();
            ci.k2 = dist[1].as<double>();
            ci.p1 = dist[2].as<double>();
            ci.p2 = dist[3].as<double>();
            if (dist.size() >= 5) ci.k3 = dist[4].as<double>();
        }

        // T_cam_imu
        auto& T_ci = node["T_cam_imu"];
        if (T_ci) {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            if (parseKalibrTransform(T_ci, T)) {
                ce.R = T.block<3, 3>(0, 0);
                ce.t = T.block<3, 1>(0, 3);
            }
        }
        return true;
    };

    if (!loadCam(root["cam0"], c.left, c.T_left_imu))   return false;
    if (!loadCam(root["cam1"], c.right, c.T_right_imu)) return false;

    // Compute T_left_right = T_left_imu * inv(T_right_imu) in IMU frame,
    // then transform to camera frame: T_left_right_cam = inv(T_left_imu) * T_left_right_imu * T_right_imu
    // Actually for stereo baseline we want right cam relative to left cam:
    //   p_left = T_left_imu * p_imu
    //   p_right = T_right_imu * p_imu
    // => p_imu = inv(T_left_imu) * p_left
    // => p_right = T_right_imu * inv(T_left_imu) * p_left
    // => T_left_right = T_right_imu * inv(T_left_imu)
    Eigen::Matrix4d TL = Eigen::Matrix4d::Identity();
    TL.block<3,3>(0,0) = c.T_left_imu.R;
    TL.block<3,1>(0,3) = c.T_left_imu.t;

    Eigen::Matrix4d TR = Eigen::Matrix4d::Identity();
    TR.block<3,3>(0,0) = c.T_right_imu.R;
    TR.block<3,1>(0,3) = c.T_right_imu.t;

    c.T_left_right = TR * TL.inverse();
    c.baseline_m = c.T_left_right.block<3, 1>(0, 3).norm();

    // Time offset
    if (root["cam0"]["timeshift_cam_imu"])
        c.cam_imu_time_offset_ns = root["cam0"]["timeshift_cam_imu"].as<double>() * 1e9;
    if (root["cam1"]["timeshift_cam_imu"])
        c.cam_imu_time_offset_ns = root["cam1"]["timeshift_cam_imu"].as<double>() * 1e9;

    // IMU intrinsics
    if (root["imu0"]) {
        auto& imu = root["imu0"];
        if (imu["gyroscope_noise_density"])
            c.imu.gyro_noise_density = imu["gyroscope_noise_density"].as<double>();
        if (imu["accelerometer_noise_density"])
            c.imu.accel_noise_density = imu["accelerometer_noise_density"].as<double>();
        if (imu["gyroscope_random_walk"])
            c.imu.gyro_random_walk = imu["gyroscope_random_walk"].as<double>();
        if (imu["accelerometer_random_walk"])
            c.imu.accel_random_walk = imu["accelerometer_random_walk"].as<double>();
    }

    return true;
}

// =============================================================================
// Undistort map precomputation
// =============================================================================
static void precomputeUndistortMaps(const CalibrationData& c) {
    if (c.left.width <= 0 || c.left.height <= 0) return;

    auto buildMap = [](const CameraIntrinsics& ci, UndistortMaps& um) {
        cv::Mat K = (cv::Mat_<double>(3, 3)
            << ci.fx, 0.0,  ci.cx,
               0.0,  ci.fy, ci.cy,
               0.0,  0.0,  1.0);
        cv::Mat D = (cv::Mat_<double>(5, 1)
            << ci.k1, ci.k2, ci.p1, ci.p2, ci.k3);

        cv::Size sz(ci.width, ci.height);
        cv::Mat newK = cv::getOptimalNewCameraMatrix(K, D, sz, 1.0, sz);
        cv::initUndistortRectifyMap(K, D, cv::Mat(), newK, sz, CV_32FC1,
                                     um.map1, um.map2);
    };

    buildMap(c.left,  g_undistort_left);
    buildMap(c.right, g_undistort_right);
    g_maps_valid = true;
}

// =============================================================================
// Public API
// =============================================================================
CalibrationData* calibration_load(const char* path, bool load_undistort_maps) {
    if (!path || !path[0]) return nullptr;

    // Auto-detect JSON format by file extension
    const char* ext = strrchr(path, '.');
    if (ext && (_stricmp(ext, ".json") == 0)) {
        (void)load_undistort_maps;
        return calibration_load_json(path);
    }

    FILE* f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[Calib] Cannot open %s\n", path);
        return nullptr;
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::string yaml_str(sz, '\0');
    fread(&yaml_str[0], 1, sz, f);
    fclose(f);

    YAML::Node root;
    try {
        root = YAML::Load(yaml_str);
    } catch (const YAML::Exception& e) {
        fprintf(stderr, "[Calib] YAML parse error: %s\n", e.what());
        return nullptr;
    }

    auto* calib = new CalibrationData();
    bool ok = false;

    // Auto-detect format
    if (root["cam0"]) {
        printf("[Calib] Detected Kalibr format\n");
        ok = loadKalibr(root, *calib);
    } else if (root["M1"] || root["M2"]) {
        printf("[Calib] Detected OpenCV stereo format\n");
        ok = loadOpenCV(root, *calib);
    } else {
        fprintf(stderr, "[Calib] Unknown calibration format\n");
    }

    if (!ok) {
        fprintf(stderr, "[Calib] Failed to parse calibration\n");
        delete calib;
        return nullptr;
    }

    // Final sanity: ensure all essential fields are non-zero
    if (!calib->left.has_valid_focal() || !calib->right.has_valid_focal()) {
        fprintf(stderr, "[Calib] Invalid focal lengths\n");
        delete calib;
        return nullptr;
    }

    // Compute baseline if not already set (from T_left_right)
    if (calib->baseline_m <= 0) {
        calib->baseline_m = calib->T_left_right.block<3, 1>(0, 3).norm();
    }

    validate(*calib);
    calib->is_valid = (calib->validation.failed == 0);

    printf("[Calib] Loaded: %dx%d stereo, baseline %.1f mm, valid=%s\n",
           calib->left.width, calib->left.height,
           calib->baseline_m * 1000.0,
           calib->is_valid ? "YES" : "NO");
    if (!calib->is_valid) {
        for (int i = 0; i < calib->validation.count; ++i) {
            auto& ch = calib->validation.checks[i];
            if (!ch.pass)
                printf("[Calib]   FAIL %s: %.4f not in [%.4f, %.4f]\n",
                       ch.name, ch.value, ch.min_val, ch.max_val);
        }
    }

    if (load_undistort_maps) {
        precomputeUndistortMaps(*calib);
    }

    return calib;
}

bool calibration_undistort(cv::Mat& frame, CameraId cam) {
    if (!g_maps_valid) return false;
    UndistortMaps& um = (cam == CAM_LEFT) ? g_undistort_left : g_undistort_right;
    if (um.map1.empty() || um.map2.empty()) return false;
    cv::Mat out;
    cv::remap(frame, out, um.map1, um.map2, cv::INTER_LINEAR);
    frame = out;
    return true;
}

// =============================================================================
// XREAL factory calibration.json loader
// =============================================================================
CalibrationData* calibration_load_json(const char* path) {
    using json = nlohmann::json;

    std::ifstream f(path);
    if (!f.is_open()) {
        fprintf(stderr, "[Calib-JSON] Cannot open %s\n", path);
        return nullptr;
    }

    json data;
    try {
        f >> data;
    } catch (const std::exception& e) {
        fprintf(stderr, "[Calib-JSON] JSON parse error: %s\n", e.what());
        return nullptr;
    }

    if (!data.contains("SLAM_camera")) {
        fprintf(stderr, "[Calib-JSON] No SLAM_camera section\n");
        return nullptr;
    }

    auto parseCam = [](const json& dev, CameraIntrinsics& ci, CameraExtrinsics& ce, const char* label) {
        if (!dev.contains("fc") || !dev.contains("cc") || !dev.contains("kc")) {
            fprintf(stderr, "[Calib-JSON] %s missing camera params\n", label);
            return false;
        }

        ci.fx = dev["fc"][0].get<double>();
        ci.fy = dev["fc"][1].get<double>();
        ci.cx = dev["cc"][0].get<double>();
        ci.cy = dev["cc"][1].get<double>();
        ci.model = CameraModel::Fisheye624;

        for (int i = 0; i < 12; ++i)
            ci.kc[i] = dev["kc"][i].get<double>();

        if (dev.contains("resolution")) {
            ci.width  = dev["resolution"][0].get<int>();
            ci.height = dev["resolution"][1].get<int>();
        }

        // IMU-to-camera extrinsics
        if (dev.contains("imu_p_cam")) {
            ce.t(0) = dev["imu_p_cam"][0].get<double>();
            ce.t(1) = dev["imu_p_cam"][1].get<double>();
            ce.t(2) = dev["imu_p_cam"][2].get<double>();
        }

        if (dev.contains("imu_q_cam")) {
            double x = dev["imu_q_cam"][0].get<double>();
            double y = dev["imu_q_cam"][1].get<double>();
            double z = dev["imu_q_cam"][2].get<double>();
            double w = dev["imu_q_cam"][3].get<double>();

            // JPL quaternion to rotation matrix
            double nrm = std::sqrt(x*x + y*y + z*z + w*w);
            if (nrm > 1e-9) { x /= nrm; y /= nrm; z /= nrm; w /= nrm; }

            ce.R(0, 0) = 1.0 - 2.0*(y*y + z*z);
            ce.R(0, 1) = 2.0*(x*y - w*z);
            ce.R(0, 2) = 2.0*(x*z + w*y);
            ce.R(1, 0) = 2.0*(x*y + w*z);
            ce.R(1, 1) = 1.0 - 2.0*(x*x + z*z);
            ce.R(1, 2) = 2.0*(y*z - w*x);
            ce.R(2, 0) = 2.0*(x*z - w*y);
            ce.R(2, 1) = 2.0*(y*z + w*x);
            ce.R(2, 2) = 1.0 - 2.0*(x*x + y*y);
        }

        return true;
    };

    auto parseDisplay = [](const json& disp, const char* p_key, const char* q_key, DisplayExtrinsics& de, const char* label) {
        if (!disp.contains(p_key) || !disp.contains(q_key)) {
            fprintf(stderr, "[Calib-JSON] %s missing %s or %s\n", label, p_key, q_key);
            de.valid = false;
            return;
        }

        const auto& p = disp[p_key];
        const auto& q = disp[q_key];
        if (!p.is_array() || p.size() < 3 || !q.is_array() || q.size() < 4) {
            fprintf(stderr, "[Calib-JSON] %s invalid pose array sizes\n", label);
            de.valid = false;
            return;
        }

        de.t_imu_display(0) = p[0].get<double>();
        de.t_imu_display(1) = p[1].get<double>();
        de.t_imu_display(2) = p[2].get<double>();

        // display.target_q_*_display is Hamilton (qx,qy,qz,qw)
        double x = q[0].get<double>();
        double y = q[1].get<double>();
        double z = q[2].get<double>();
        double w = q[3].get<double>();
        double nrm = std::sqrt(x*x + y*y + z*z + w*w);
        if (nrm <= 1e-9) {
            fprintf(stderr, "[Calib-JSON] %s invalid quaternion norm\n", label);
            de.valid = false;
            return;
        }
        x /= nrm; y /= nrm; z /= nrm; w /= nrm;

        de.R_imu_display(0, 0) = 1.0 - 2.0*(y*y + z*z);
        de.R_imu_display(0, 1) = 2.0*(x*y - z*w);
        de.R_imu_display(0, 2) = 2.0*(x*z + y*w);
        de.R_imu_display(1, 0) = 2.0*(x*y + z*w);
        de.R_imu_display(1, 1) = 1.0 - 2.0*(x*x + z*z);
        de.R_imu_display(1, 2) = 2.0*(y*z - x*w);
        de.R_imu_display(2, 0) = 2.0*(x*z - y*w);
        de.R_imu_display(2, 1) = 2.0*(y*z + x*w);
        de.R_imu_display(2, 2) = 1.0 - 2.0*(x*x + y*y);

        de.valid = true;
        fprintf(stdout, "[Calib-JSON] %s display extrinsics parsed\n", label);
    };

    auto* calib = new CalibrationData();

    auto& slam = data["SLAM_camera"];
    if (!parseCam(slam.at("device_1"), calib->left,  calib->T_left_imu,  "device_1")) { delete calib; return nullptr; }
    if (!parseCam(slam.at("device_2"), calib->right, calib->T_right_imu, "device_2")) { delete calib; return nullptr; }

    if (data.contains("display") && data["display"].is_object()) {
        const auto& disp = data["display"];
        parseDisplay(disp, "target_p_left_display", "target_q_left_display", calib->T_left_display_imu, "left");
        parseDisplay(disp, "target_p_right_display", "target_q_right_display", calib->T_right_display_imu, "right");
    } else {
        fprintf(stderr, "[Calib-JSON] No display section; composite overlay display extrinsics unavailable\n");
    }

    if (!calib->T_left_display_imu.valid || !calib->T_right_display_imu.valid) {
        fprintf(stderr, "[Calib-JSON] Display extrinsics incomplete; composite overlay will fallback\n");
    }

    // Compute T_left_right = T_right_imu * inv(T_left_imu)
    //   p_imu = inv(T_left_imu) * p_left_cam
    //   p_right_cam = T_right_imu * p_imu = T_right_imu * inv(T_left_imu) * p_left_cam
    Eigen::Matrix4d TL = Eigen::Matrix4d::Identity();
    TL.block<3,3>(0,0) = calib->T_left_imu.R;
    TL.block<3,1>(0,3) = calib->T_left_imu.t;

    Eigen::Matrix4d TR = Eigen::Matrix4d::Identity();
    TR.block<3,3>(0,0) = calib->T_right_imu.R;
    TR.block<3,1>(0,3) = calib->T_right_imu.t;

    // T_left_right maps left camera → right camera
    // TL = maps left_cam → IMU, TR = maps right_cam → IMU
    // p_right = TR^{-1} * TL * p_left
    calib->T_left_right = TR.inverse() * TL;
    calib->baseline_m = calib->T_left_right.block<3,1>(0,3).norm();

    // IMU noise params
    if (data.contains("IMU") && data["IMU"].contains("device_1")) {
        auto& imu = data["IMU"]["device_1"];
        if (imu.contains("imu_noises") && imu["imu_noises"].size() >= 4) {
            calib->imu.accel_noise_density = imu["imu_noises"][0].get<double>();
            calib->imu.accel_random_walk   = imu["imu_noises"][1].get<double>();
            calib->imu.gyro_noise_density  = imu["imu_noises"][2].get<double>();
            calib->imu.gyro_random_walk    = imu["imu_noises"][3].get<double>();
        }

        if (imu.contains("imu_intrinsics")) {
            auto& intr = imu["imu_intrinsics"];
            if (intr.contains("accl_calib_mat")) {
                auto& m = intr["accl_calib_mat"];
                if (m.size() >= 9) {
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 3; ++c)
                            calib->imu.T_imu_acc(r, c) = m[r*3 + c].get<double>();
                }
            }
            if (intr.contains("gyro_calib_mat")) {
                auto& m = intr["gyro_calib_mat"];
                if (m.size() >= 9) {
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 3; ++c)
                            calib->imu.T_imu_gyr(r, c) = m[r*3 + c].get<double>();
                }
            }
        }
    }

    // Validate
    calib->validation.count  = 0;
    calib->validation.failed = 0;
    auto& v = calib->validation;

    auto validateCam = [&](const CameraIntrinsics& ci, const char* prefix) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s fx", prefix);
        v.add(buf, ci.fx, 100.0, 10000.0);
        snprintf(buf, sizeof(buf), "%s fy", prefix);
        v.add(buf, ci.fy, 100.0, 10000.0);
        snprintf(buf, sizeof(buf), "%s cx", prefix);
        v.add(buf, ci.cx, 0.0, (double)ci.width);
        snprintf(buf, sizeof(buf), "%s cy", prefix);
        v.add(buf, ci.cy, 0.0, (double)ci.height);
        snprintf(buf, sizeof(buf), "%s kc[0]", prefix);
        v.add(buf, ci.kc[0], -1.0, 1.0);
        snprintf(buf, sizeof(buf), "%s kc[1]", prefix);
        v.add(buf, ci.kc[1], -1.0, 1.0);
        snprintf(buf, sizeof(buf), "%s FOV (fx/w)", prefix);
        v.add(buf, ci.fx / (double)ci.width, 0.2, 2.0);
    };

    validateCam(calib->left,  "Left");
    validateCam(calib->right, "Right");
    v.add("Baseline (mm)",   calib->baseline_m * 1000.0, 20.0, 200.0);
    v.add("Accel noise dens",  calib->imu.accel_noise_density, 0.0, 1.0);
    v.add("Gyro noise dens",   calib->imu.gyro_noise_density, 0.0, 1.0);
    v.add("Accel random walk", calib->imu.accel_random_walk, 0.0, 1.0);
    v.add("Gyro random walk",  calib->imu.gyro_random_walk, 0.0, 1.0);

    calib->is_valid = (calib->validation.failed == 0);

    printf("[Calib-JSON] Loaded: %dx%d stereo, baseline %.1f mm, fx=(%.1f,%.1f), fisheye624\n",
           calib->left.width, calib->left.height,
           calib->baseline_m * 1000.0,
           calib->left.fx, calib->right.fx);

    if (!calib->is_valid) {
        for (int i = 0; i < calib->validation.count; ++i) {
            auto& ch = calib->validation.checks[i];
            if (!ch.pass)
                printf("[Calib-JSON]   FAIL %s: %.4f not in [%.4f, %.4f]\n",
                       ch.name, ch.value, ch.min_val, ch.max_val);
        }
    }

    return calib;
}

void calibration_free(CalibrationData* calib) {
    delete calib;
    if (g_singleton == calib) {
        std::lock_guard<std::mutex> lk(g_singleton_mutex);
        if (g_singleton == calib) g_singleton = nullptr;
    }
}

// =============================================================================
// Save calibration to JSON + fisheye4 initializer
// =============================================================================
bool calibration_save_json(const char* path, const CalibrationData& calib) {
    using json = nlohmann::json;

    json slam;
    json dev1, dev2;

    auto writeCam = [](json& dev, const CameraIntrinsics& ci, const CameraExtrinsics& ce) {
        dev["fc"] = {ci.fx, ci.fy};
        dev["cc"] = {ci.cx, ci.cy};
        if (ci.model == CameraModel::Fisheye624) {
            json kc_arr = json::array();
            for (int i = 0; i < 12; ++i) kc_arr.push_back(ci.kc[i]);
            dev["kc"] = kc_arr;
        } else if (ci.model == CameraModel::Fisheye4) {
            // Store 4-param model — pad with zeros to keep array size for compat
            json kc_arr = json::array();
            for (int i = 0; i < 4; ++i) kc_arr.push_back(ci.fisheye_k[i]);
            for (int i = 4; i < 12; ++i) kc_arr.push_back(0.0);
            dev["kc"] = kc_arr;
            dev["model"] = "fisheye4";
        } else {
            json kc_arr = json::array();
            double pk[] = {ci.k1, ci.k2, ci.p1, ci.p2, ci.k3};
            for (int i = 0; i < 5; ++i) kc_arr.push_back(pk[i]);
            for (int i = 5; i < 12; ++i) kc_arr.push_back(0.0);
            dev["kc"] = kc_arr;
            dev["model"] = "pinhole";
        }
        dev["resolution"] = {ci.width, ci.height};
        dev["imu_p_cam"] = {ce.t(0), ce.t(1), ce.t(2)};
    };

    writeCam(dev1, calib.left,  calib.T_left_imu);
    writeCam(dev2, calib.right, calib.T_right_imu);

    slam["device_1"] = dev1;
    slam["device_2"] = dev2;

    json data;
    data["SLAM_camera"] = slam;

    std::ofstream out(path);
    if (!out.is_open()) {
        fprintf(stderr, "[Calib-JSON] Cannot write %s\n", path);
        return false;
    }
    out << data.dump(2) << std::endl;
    printf("[Calib-JSON] Saved to %s\n", path);
    return true;
}

