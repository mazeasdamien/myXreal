#include "myxreal/stereo_rectifier.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <cmath>
#include <cstdio>
#include <chrono>
#include <algorithm>

// =============================================================================
// Project a 3D ray into distorted pixel coords using the fisheye624 model.
// p_cam: 3x1 direction vector in camera frame (not necessarily unit length).
// =============================================================================
void StereoRectifier::project_fisheye624(const double kc[12], double fx, double fy,
                                          double cx, double cy, const cv::Mat& p_cam,
                                          double& u_dist, double& v_dist) const {
    double X = p_cam.at<double>(0);
    double Y = p_cam.at<double>(1);
    double Z = p_cam.at<double>(2);

    double a = X / Z;
    double b = Y / Z;
    double r = std::sqrt(a*a + b*b);
    double theta = std::atan(r);

    // Radial distortion (kc[0..5]): theta_d = theta + sum_{i=0..5} kc[i] * theta^{2i+3}
    double theta_d = theta;
    double th_pow = theta;
    double th2 = theta * theta;
    for (int i = 0; i < 6; ++i) {
        th_pow *= th2; // theta^3, theta^5, ..., theta^13
        theta_d += kc[i] * th_pow;
    }

    double u_rd = 0.0, v_rd = 0.0;
    if (r > 1e-9) {
        u_rd = a * (theta_d / r);
        v_rd = b * (theta_d / r);
    }
    double rd2 = u_rd*u_rd + v_rd*v_rd;

    // Tangential distortion (kc[6..7])
    double p0 = kc[6];
    double p1 = kc[7];
    double t_x = 2.0 * p0 * u_rd * v_rd + p1 * (rd2 + 2.0 * u_rd*u_rd);
    double t_y = p0 * (rd2 + 2.0 * v_rd*v_rd) + 2.0 * p1 * u_rd * v_rd;

    // Thin-prism distortion (kc[8..11])
    double s0 = kc[8];
    double s1 = kc[9];
    double s2 = kc[10];
    double s3 = kc[11];
    double tp_x = s0 * rd2 + s1 * rd2 * rd2;
    double tp_y = s2 * rd2 + s3 * rd2 * rd2;

    u_dist = fx * (u_rd + t_x + tp_x) + cx;
    v_dist = fy * (v_rd + t_y + tp_y) + cy;
}

// =============================================================================
// Build rectification rotation matrices from the stereo extrinsics.
// Computes R1_ (left rect rotation) and R2_ (right rect rotation), plus
// the virtual rectified camera intrinsics.
// =============================================================================
void StereoRectifier::build_rectification(const CalibrationData& calib) {
    // Extract R_lr (left->right rotation) and t_lr (left->right translation)
    cv::Mat R_lr = (cv::Mat_<double>(3, 3)
        << calib.T_left_right(0, 0), calib.T_left_right(0, 1), calib.T_left_right(0, 2),
           calib.T_left_right(1, 0), calib.T_left_right(1, 1), calib.T_left_right(1, 2),
           calib.T_left_right(2, 0), calib.T_left_right(2, 1), calib.T_left_right(2, 2));

    cv::Mat t_lr = (cv::Mat_<double>(3, 1)
        << calib.T_left_right(0, 3), calib.T_left_right(1, 3), calib.T_left_right(2, 3));

    printf("[Rectify] T_left_right R (left->right):\n");
    printf("[Rectify]   [ %+.6f  %+.6f  %+.6f ]\n", R_lr.at<double>(0,0), R_lr.at<double>(0,1), R_lr.at<double>(0,2));
    printf("[Rectify]   [ %+.6f  %+.6f  %+.6f ]\n", R_lr.at<double>(1,0), R_lr.at<double>(1,1), R_lr.at<double>(1,2));
    printf("[Rectify]   [ %+.6f  %+.6f  %+.6f ]\n", R_lr.at<double>(2,0), R_lr.at<double>(2,1), R_lr.at<double>(2,2));
    printf("[Rectify] T_left_right t (left->right): [ %+.4f  %+.4f  %+.4f ] m\n",
           t_lr.at<double>(0,0), t_lr.at<double>(1,0), t_lr.at<double>(2,0));

    // Baseline vector in left camera frame: position of right camera
    cv::Mat T = -R_lr.t() * t_lr;
    printf("[Rectify] Baseline in left-cam frame: [ %+.4f  %+.4f  %+.4f ] m  (len=%.3f mm)\n",
           T.at<double>(0,0), T.at<double>(1,0), T.at<double>(2,0), cv::norm(T) * 1000.0);
    baseline_m_ = cv::norm(T);

    // Rectification rotation: rows are orthonormal basis of rectified frame
    cv::Mat e1 = T / cv::norm(T);                    // baseline direction → X axis
    cv::Mat z_dir = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);  // optical axis
    cv::Mat e2 = z_dir.cross(e1);
    e2 = e2 / cv::norm(e2);                           // Y axis
    cv::Mat e3 = e1.cross(e2);                        // Z axis
    e3 = e3 / cv::norm(e3);

    cv::Mat R_rect = cv::Mat::eye(3, 3, CV_64F);
    R_rect.row(0) = e1.t();
    R_rect.row(1) = e2.t();
    R_rect.row(2) = e3.t();

    // R1 = R_rect (left camera → rectified)
    // R2 = R_rect * R_lr^T (right camera → rectified, via left frame)
    R1_ = R_rect.clone();
    R2_ = R_rect * R_lr.t();

    printf("[Rectify] R1 (left rect rotation):\n");
    printf("[Rectify]   [ %+.6f  %+.6f  %+.6f ]\n", R1_.at<double>(0,0), R1_.at<double>(0,1), R1_.at<double>(0,2));
    printf("[Rectify]   [ %+.6f  %+.6f  %+.6f ]\n", R1_.at<double>(1,0), R1_.at<double>(1,1), R1_.at<double>(1,2));
    printf("[Rectify]   [ %+.6f  %+.6f  %+.6f ]\n", R1_.at<double>(2,0), R1_.at<double>(2,1), R1_.at<double>(2,2));
    printf("[Rectify] R2 (right rect rotation):\n");
    printf("[Rectify]   [ %+.6f  %+.6f  %+.6f ]\n", R2_.at<double>(0,0), R2_.at<double>(0,1), R2_.at<double>(0,2));
    printf("[Rectify]   [ %+.6f  %+.6f  %+.6f ]\n", R2_.at<double>(1,0), R2_.at<double>(1,1), R2_.at<double>(1,2));
    printf("[Rectify]   [ %+.6f  %+.6f  %+.6f ]\n", R2_.at<double>(2,0), R2_.at<double>(2,1), R2_.at<double>(2,2));

    // Virtual rectified camera intrinsics
    fx_rect_ = std::min(calib.left.fx, calib.right.fx);
    fy_rect_ = std::min(calib.left.fy, calib.right.fy);
    cx_rect_ = (calib.left.cx + calib.right.cx) / 2.0;
    cy_rect_ = (calib.left.cy + calib.right.cy) / 2.0;

    // Build Q matrix for disparity-to-depth
    Q_ = (cv::Mat_<double>(4, 4)
        << 1.0, 0.0, 0.0, -cx_rect_,
           0.0, 1.0, 0.0, -cy_rect_,
           0.0, 0.0, 0.0,  fx_rect_,
           0.0, 0.0,  1.0/baseline_m_, 0.0);

    // Build P1, P2 (rectified projection matrices)
    P1_ = (cv::Mat_<double>(3, 4)
        << fx_rect_, 0.0,       cx_rect_, 0.0,
           0.0,      fy_rect_,  cy_rect_, 0.0,
           0.0,      0.0,       1.0,      0.0);

    P2_ = (cv::Mat_<double>(3, 4)
        << fx_rect_, 0.0,       cx_rect_, -fx_rect_ * baseline_m_,
           0.0,      fy_rect_,  cy_rect_, 0.0,
           0.0,      0.0,       1.0,      0.0);
}

// =============================================================================
// Public API
// =============================================================================
void StereoRectifier::init_fisheye624(const CalibrationData& calib) {
    if (calib.left.model != CameraModel::Fisheye624 ||
        calib.right.model != CameraModel::Fisheye624) {
        fprintf(stderr, "[Rectify] init_fisheye624 called but model mismatch\n");
        return;
    }

    rect_w_ = calib.left.width;
    rect_h_ = calib.left.height;
    model_ = CameraModel::Fisheye624;

    printf("[Rectify] Fisheye624 path: %dx%d (WxH)\n", rect_w_, rect_h_);
    printf("[Rectify] Left  intrinsics: fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n",
           calib.left.fx, calib.left.fy, calib.left.cx, calib.left.cy);
    printf("[Rectify] Right intrinsics: fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n",
           calib.right.fx, calib.right.fy, calib.right.cx, calib.right.cy);

    build_rectification(calib);

    map_x_l_ = cv::Mat(rect_h_, rect_w_, CV_32FC1);
    map_y_l_ = cv::Mat(rect_h_, rect_w_, CV_32FC1);
    map_x_r_ = cv::Mat(rect_h_, rect_w_, CV_32FC1);
    map_y_r_ = cv::Mat(rect_h_, rect_w_, CV_32FC1);

    cv::Mat R1t = R1_.t();
    cv::Mat R2t = R2_.t();

    for (int v = 0; v < rect_h_; ++v) {
        for (int u = 0; u < rect_w_; ++u) {
            double x_rect = (u - cx_rect_) / fx_rect_;
            double y_rect = (v - cy_rect_) / fy_rect_;
            cv::Mat p_rect = (cv::Mat_<double>(3, 1) << x_rect, y_rect, 1.0);

            cv::Mat p_cam_l = R1t * p_rect;
            double u_l, v_l;
            project_fisheye624(calib.left.kc, calib.left.fx, calib.left.fy,
                               calib.left.cx, calib.left.cy, p_cam_l, u_l, v_l);
            map_x_l_.at<float>(v, u) = static_cast<float>(u_l);
            map_y_l_.at<float>(v, u) = static_cast<float>(v_l);

            cv::Mat p_cam_r = R2t * p_rect;
            double u_r, v_r;
            project_fisheye624(calib.right.kc, calib.right.fx, calib.right.fy,
                               calib.right.cx, calib.right.cy, p_cam_r, u_r, v_r);
            map_x_r_.at<float>(v, u) = static_cast<float>(u_r);
            map_y_r_.at<float>(v, u) = static_cast<float>(v_r);
        }
    }

    rect_left_.resize(rect_w_ * rect_h_);
    rect_right_.resize(rect_w_ * rect_h_);

    initialized_ = true;

    printf("[Rectify] Fisheye624 init: %dx%d rectified, fx=%.1f fy=%.1f, "
           "cx=%.1f cy=%.1f, baseline=%.1f mm\n",
           rect_w_, rect_h_, fx_rect_, fy_rect_, cx_rect_, cy_rect_,
           baseline_m_ * 1000.0);
}

// ---------------------------------------------------------------------------
// OpenCV fisheye4 path: use cv::fisheye::stereoRectify + initUndistortRectifyMap
// ---------------------------------------------------------------------------
void StereoRectifier::init_fisheye4(const CalibrationData& calib) {
    rect_w_ = calib.left.width;
    rect_h_ = calib.left.height;
    model_ = CameraModel::Fisheye4;

    printf("[Rectify] Fisheye4 path: %dx%d (WxH)\n", rect_w_, rect_h_);

    cv::Mat K1 = (cv::Mat_<double>(3, 3)
        << calib.left.fx,  0.0, calib.left.cx,
           0.0, calib.left.fy,  calib.left.cy,
           0.0, 0.0, 1.0);

    cv::Mat K2 = (cv::Mat_<double>(3, 3)
        << calib.right.fx, 0.0, calib.right.cx,
           0.0, calib.right.fy, calib.right.cy,
           0.0, 0.0, 1.0);

    cv::Mat D1 = (cv::Mat_<double>(4, 1)
        << calib.left.fisheye_k[0], calib.left.fisheye_k[1],
           calib.left.fisheye_k[2], calib.left.fisheye_k[3]);

    cv::Mat D2 = (cv::Mat_<double>(4, 1)
        << calib.right.fisheye_k[0], calib.right.fisheye_k[1],
           calib.right.fisheye_k[2], calib.right.fisheye_k[3]);

    printf("[Rectify] Left  D: [%.4f, %.4f, %.4f, %.4f]\n",
           D1.at<double>(0), D1.at<double>(1), D1.at<double>(2), D1.at<double>(3));
    printf("[Rectify] Right D: [%.4f, %.4f, %.4f, %.4f]\n",
           D2.at<double>(0), D2.at<double>(1), D2.at<double>(2), D2.at<double>(3));

    // Extract R, T from T_left_right
    cv::Mat R_lr = (cv::Mat_<double>(3, 3)
        << calib.T_left_right(0, 0), calib.T_left_right(0, 1), calib.T_left_right(0, 2),
           calib.T_left_right(1, 0), calib.T_left_right(1, 1), calib.T_left_right(1, 2),
           calib.T_left_right(2, 0), calib.T_left_right(2, 1), calib.T_left_right(2, 2));

    cv::Mat T_lr = (cv::Mat_<double>(3, 1)
        << calib.T_left_right(0, 3), calib.T_left_right(1, 3), calib.T_left_right(2, 3));

    cv::Size img_size(rect_w_, rect_h_);

    // Run fisheye stereoRectify
    cv::fisheye::stereoRectify(K1, D1, K2, D2, img_size,
                                R_lr, T_lr,
                                R1_, R2_, P1_, P2_, Q_,
                                cv::CALIB_ZERO_DISPARITY,
                                img_size, 0.0, 1.0);

    // Extract rectified intrinsics from P1
    fx_rect_ = P1_.at<double>(0, 0);
    fy_rect_ = P1_.at<double>(1, 1);
    cx_rect_ = P1_.at<double>(0, 2);
    cy_rect_ = P1_.at<double>(1, 2);
    baseline_m_ = calib.baseline_m;

    printf("[Rectify] R1:\n  [ %+.6f  %+.6f  %+.6f ]\n"
           "  [ %+.6f  %+.6f  %+.6f ]\n"
           "  [ %+.6f  %+.6f  %+.6f ]\n",
           R1_.at<double>(0,0), R1_.at<double>(0,1), R1_.at<double>(0,2),
           R1_.at<double>(1,0), R1_.at<double>(1,1), R1_.at<double>(1,2),
           R1_.at<double>(2,0), R1_.at<double>(2,1), R1_.at<double>(2,2));

    // Build remap tables
    map_x_l_ = cv::Mat(rect_h_, rect_w_, CV_32FC1);
    map_y_l_ = cv::Mat(rect_h_, rect_w_, CV_32FC1);
    map_x_r_ = cv::Mat(rect_h_, rect_w_, CV_32FC1);
    map_y_r_ = cv::Mat(rect_h_, rect_w_, CV_32FC1);

    cv::fisheye::initUndistortRectifyMap(K1, D1, R1_, P1_, img_size, CV_32FC1,
                                          map_x_l_, map_y_l_);
    cv::fisheye::initUndistortRectifyMap(K2, D2, R2_, P2_, img_size, CV_32FC1,
                                          map_x_r_, map_y_r_);

    rect_left_.resize(rect_w_ * rect_h_);
    rect_right_.resize(rect_w_ * rect_h_);

    initialized_ = true;

    printf("[Rectify] Fisheye4 init: %dx%d rectified, fx=%.1f fy=%.1f, "
           "cx=%.1f cy=%.1f, baseline=%.1f mm\n",
           rect_w_, rect_h_, fx_rect_, fy_rect_, cx_rect_, cy_rect_,
           baseline_m_ * 1000.0);
}

// ---------------------------------------------------------------------------
// init() — dispatcher
// ---------------------------------------------------------------------------
void StereoRectifier::init(const CalibrationData& calib) {
    if (!calib.is_valid) {
        fprintf(stderr, "[Rectify] Calibration not valid, skipping init\n");
        return;
    }

    if (calib.left.model == CameraModel::Fisheye624 &&
        calib.right.model == CameraModel::Fisheye624) {
        init_fisheye624(calib);
    } else if (calib.left.model == CameraModel::Fisheye4 &&
               calib.right.model == CameraModel::Fisheye4) {
        init_fisheye4(calib);
    } else {
        fprintf(stderr, "[Rectify] Unsupported or mismatched camera models: L=%d R=%d\n",
                (int)calib.left.model, (int)calib.right.model);
    }
}

StereoPair StereoRectifier::rectify(const StereoPair& raw) {
    StereoPair out = raw;

    if (!initialized_) return out;

    cv::Mat left_raw(raw.left.height, raw.left.width, CV_8UC1,
                     const_cast<uint8_t*>(raw.left.data));
    cv::Mat right_raw(raw.right.height, raw.right.width, CV_8UC1,
                      const_cast<uint8_t*>(raw.right.data));

    cv::Mat left_rect(rect_h_, rect_w_, CV_8UC1, rect_left_.data());
    cv::Mat right_rect(rect_h_, rect_w_, CV_8UC1, rect_right_.data());

    cv::remap(left_raw,  left_rect,  map_x_l_, map_y_l_, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    cv::remap(right_raw, right_rect, map_x_r_, map_y_r_, cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    out.left.data         = rect_left_.data();
    out.left.width        = rect_w_;
    out.left.height       = rect_h_;
    out.left.is_rectified = true;

    out.right.data         = rect_right_.data();
    out.right.width        = rect_w_;
    out.right.height       = rect_h_;
    out.right.is_rectified = true;

    return out;
}

EpipolarReport StereoRectifier::validateEpipolar(int max_features, CameraModel model) {
    EpipolarReport r;

    if (!initialized_ || rect_left_.empty() || rect_right_.empty())
        return r;

    auto t0 = std::chrono::steady_clock::now();

    cv::Mat left_img(rect_h_, rect_w_, CV_8UC1, rect_left_.data());
    cv::Mat right_img(rect_h_, rect_w_, CV_8UC1, rect_right_.data());

    auto orb = cv::ORB::create(max_features);
    std::vector<cv::KeyPoint> kpL, kpR;
    cv::Mat descL, descR;
    orb->detectAndCompute(left_img, cv::Mat(), kpL, descL);
    orb->detectAndCompute(right_img, cv::Mat(), kpR, descR);

    if (descL.empty() || descR.empty()) {
        r.elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        return r;
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(descL, descR, matches);

    std::sort(matches.begin(), matches.end(),
              [](const cv::DMatch& a, const cv::DMatch& b) { return a.distance < b.distance; });
    int keep = std::min(200, (int)matches.size());
    matches.resize(keep);

    double sum_v = 0;
    double max_v = 0;
    int n = 0;
    for (auto& m : matches) {
        double dy = std::abs(kpL[m.queryIdx].pt.y - kpR[m.trainIdx].pt.y);
        sum_v += dy;
        if (dy > max_v) max_v = dy;
        n++;
    }

    if (n > 0) {
        r.mean_v_disp = sum_v / n;
        r.max_v_disp  = max_v;
    }
    r.num_matches = n;

    // Model-aware thresholds:
    //   Pinhole:       mean < 2.0, max < 5.0  (looser — OpenCV pinhole rectify is approximate)
    //   Fisheye4:      mean < 1.0, max < 2.5  (tighter — fisheye4 rectification should be accurate)
    //   Fisheye624:    mean < 1.5, max < 3.0  (legacy — our custom pipeline)
    double mean_thr, max_thr;
    switch (model) {
        case CameraModel::Pinhole:
            mean_thr = 2.0; max_thr = 5.0; break;
        case CameraModel::Fisheye4:
            mean_thr = 1.0; max_thr = 2.5; break;
        case CameraModel::Fisheye624:
        default:
            mean_thr = 1.5; max_thr = 3.0; break;
    }

    r.pass_mean   = (r.mean_v_disp < mean_thr);
    r.pass_max    = (r.max_v_disp  < max_thr);
    r.overall     = (r.pass_mean && r.pass_max);

    r.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    return r;
}
