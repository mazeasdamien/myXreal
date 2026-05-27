#pragma once

#include "myxreal/calibration_types.h"
#include "myxreal/stereo_camera.h"

#include <opencv2/core/mat.hpp>
#include <cstddef>

// ---------------------------------------------------------------------------
// Epipolar validation result
// ---------------------------------------------------------------------------
struct EpipolarReport {
    double mean_v_disp = 0.0;    // mean |left.y - right.y| in pixels
    double max_v_disp  = 0.0;    // max vertical disparity in pixels
    int    num_matches = 0;
    bool   pass_mean   = false;  // mean < 1.5 px
    bool   pass_max    = false;  // max  < 3.0 px
    bool   overall     = false;

    double elapsed_ms  = 0.0;
};

// ---------------------------------------------------------------------------
// Stereo rectification for XREAL Air 2 Ultra fisheye624 cameras.
// Uses custom pixel-by-pixel remap table generation (not OpenCV stereoRectify)
// because the 12-coefficient fisheye624 model isn't supported by OpenCV.
// ---------------------------------------------------------------------------
class StereoRectifier {
public:
    void init(const CalibrationData& calib);

    // Rectify a raw stereo pair. Returns new StereoPair with rectified data
    // owned by the rectifier's internal buffers (valid until next call).
    StereoPair rectify(const StereoPair& raw);

    // Run epipolar validation on the last rectified pair.
    // Pass the camera model so thresholds can be adjusted per model.
    EpipolarReport validateEpipolar(int max_features = 500, CameraModel model = CameraModel::Fisheye624);

    // Accessors
    const cv::Mat& Q()   const { return Q_; }
    const cv::Mat& P1()  const { return P1_; }
    const cv::Mat& P2()  const { return P2_; }
    const cv::Mat& R1()  const { return R1_; }
    const cv::Mat& R2()  const { return R2_; }
    double baseline()    const { return baseline_m_; }
    double focalLength() const { return fx_rect_; }
    double fx_rect()     const { return fx_rect_; }
    double fy_rect()     const { return fy_rect_; }
    double cx_rect()     const { return cx_rect_; }
    double cy_rect()     const { return cy_rect_; }
    int    rect_width()  const { return rect_w_; }
    int    rect_height() const { return rect_h_; }
    bool   initialized() const { return initialized_; }

private:
    // Custom fisheye624 path
    void project_fisheye624(const double kc[12], double fx, double fy,
                            double cx, double cy, const cv::Mat& p_cam,
                            double& u_dist, double& v_dist) const;
    void build_rectification(const CalibrationData& calib);
    void init_fisheye624(const CalibrationData& calib);

    // OpenCV fisheye4 path (Kannala-Brandt 4-param)
    void init_fisheye4(const CalibrationData& calib);

    cv::Mat map_x_l_, map_y_l_, map_x_r_, map_y_r_;  // CV_32FC1 remap tables
    cv::Mat R1_, R2_;  // rectification rotations

    // Virtual rectified pinhole intrinsics
    cv::Mat P1_, P2_, Q_;
    double  fx_rect_ = 0.0, fy_rect_ = 0.0;
    double  cx_rect_ = 0.0, cy_rect_ = 0.0;
    double  baseline_m_ = 0.0;
    int     rect_w_ = 0, rect_h_ = 0;
    bool    initialized_ = false;
    CameraModel model_ = CameraModel::Fisheye624;

    // Rectified pixel buffers
    std::vector<uint8_t> rect_left_, rect_right_;
};
