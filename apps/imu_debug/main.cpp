#include "myxreal/imu_driver.h"
#include "myxreal/stereo_camera.h"
#include "myxreal/calibration_loader.h"
#include "myxreal/stereo_rectifier.h"

// OpenCV before Windows headers to avoid InputArray name collision
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <nlohmann/json.hpp>

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#include <d3d11.h>
#include <tchar.h>
#include <windows.h>
#include <deque>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <chrono>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <cctype>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include <memory>
#include <filesystem>
#include <cstdint>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

static constexpr int kHistory = 600;
static std::deque<float> g_hist_fps;

// D3D11 globals (must be declared before camera texture functions use them)
static ID3D11Device*           g_pd3dDevice = nullptr;
static ID3D11DeviceContext*    g_pd3dDeviceContext = nullptr;
static IDXGISwapChain*         g_pSwapChain = nullptr;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;

// Camera textures
static ID3D11Texture2D*          g_texLeft  = nullptr;
static ID3D11Texture2D*          g_texRight = nullptr;
static ID3D11ShaderResourceView* g_srvLeft  = nullptr;
static ID3D11ShaderResourceView* g_srvRight = nullptr;
static bool g_camTexOK = false;

// Stereo rectifier + orientation dashboard
static StereoRectifier g_rectifier;
static CalibrationData* g_runtime_calib = nullptr;
static float           g_world_view_yaw_deg = 35.0f;
static float           g_world_view_pitch_deg = -20.0f;
static float           g_world_view_scale = 70.0f;
static float           g_world_cam_distance = 5.0f;
static bool            g_show_glasses_3dof = true;
static bool            g_show_vision_cone = true;
static float           g_vision_cone_fov_deg = 52.0f;
static float           g_vision_cone_depth_m = 1.6f;
static std::string     g_track_status;
static float           g_pose_x_m = 0.0f;
static float           g_pose_y_m = 0.0f;
static float           g_pose_z_m = 0.0f;
static float           g_pose_yaw_deg = 0.0f;
static float           g_pose_pitch_deg = 0.0f;
static float           g_pose_roll_deg = 0.0f;
static int             g_6dof_quality = 0;

static bool            g_feature_overlay_enabled = true;
static int             g_feature_count = 0;
static int             g_feature_max_corners = 500;
static cv::Mat         g_left_overlay_bgr;
static cv::Mat         g_right_overlay_bgr;
static uint64_t        g_last_pose_imu_ts_ns = 0;
static float           g_pose_vx_mps = 0.0f;
static float           g_pose_vy_mps = 0.0f;
static float           g_pose_vz_mps = 0.0f;

static bool            g_flip_display = false;   // horizontal flip on stereo preprocessing path
static bool            g_clahe_enabled = true;   // CLAHE on stereo preprocessing path
static bool            g_rect_enabled = true;    // fisheye rectification toggle
static cv::Mat         g_latest_left_display;
static cv::Mat         g_latest_right_display;
static cv::Ptr<cv::CLAHE> g_clahe;               // created on first use

static std::string g_console_buffer;
static uint64_t    g_console_last_size = 0;
static bool        g_console_autoscroll = true;
static float       g_last_imu_fps = 0.0f;

static bool        g_hotplug_reconnect_request = false;

static std::string g_full_state_buffer;
static bool        g_full_state_freeze = false;
static bool        g_full_state_auto_refresh = true;

static float       g_camera_panel_w = 0.0f;
static float       g_full_panel_w = 0.0f;
static bool        g_panel_sizes_loaded = false;
static bool        g_settings_dirty = false;
static uint64_t    g_settings_last_save_ms = 0;
static constexpr uint64_t kSettingsSaveThrottleMs = 400;
static const char* kUiSettingsPath = "imu_debug_ui_settings.json";

static uint64_t now_tick_ms();

struct UiPersistSettings {
    bool flip_display = false;
    bool clahe_enabled = true;
    bool rect_enabled = true;
    bool feature_overlay_enabled = true;
    bool full_state_auto_refresh = true;
    bool full_state_freeze = false;
    bool console_autoscroll = true;

    int feature_max_corners = 500;

    bool show_glasses_3dof = true;
    bool show_vision_cone = true;
    float vision_cone_fov_deg = 52.0f;
    float vision_cone_depth_m = 1.6f;
    float world_view_yaw_deg = 35.0f;
    float world_view_pitch_deg = -20.0f;
    float world_view_scale = 70.0f;
    float world_cam_distance = 5.0f;

    float camera_panel_w = 0.0f;
    float full_panel_w = 0.0f;
};

static UiPersistSettings read_ui_persist_settings_from_state() {
    UiPersistSettings s;
    s.flip_display = g_flip_display;
    s.clahe_enabled = g_clahe_enabled;
    s.rect_enabled = g_rect_enabled;
    s.feature_overlay_enabled = g_feature_overlay_enabled;
    s.full_state_auto_refresh = g_full_state_auto_refresh;
    s.full_state_freeze = g_full_state_freeze;
    s.console_autoscroll = g_console_autoscroll;

    s.feature_max_corners = g_feature_max_corners;

    s.show_glasses_3dof = g_show_glasses_3dof;
    s.show_vision_cone = g_show_vision_cone;
    s.vision_cone_fov_deg = g_vision_cone_fov_deg;
    s.vision_cone_depth_m = g_vision_cone_depth_m;
    s.world_view_yaw_deg = g_world_view_yaw_deg;
    s.world_view_pitch_deg = g_world_view_pitch_deg;
    s.world_view_scale = g_world_view_scale;
    s.world_cam_distance = g_world_cam_distance;

    s.camera_panel_w = g_camera_panel_w;
    s.full_panel_w = g_full_panel_w;
    return s;
}

static void apply_ui_persist_settings_to_state(const UiPersistSettings& s) {
    g_flip_display = s.flip_display;
    g_clahe_enabled = s.clahe_enabled;
    g_rect_enabled = s.rect_enabled;
    g_feature_overlay_enabled = s.feature_overlay_enabled;
    g_full_state_auto_refresh = s.full_state_auto_refresh;
    g_full_state_freeze = s.full_state_freeze;
    g_console_autoscroll = s.console_autoscroll;

    g_feature_max_corners = std::clamp(s.feature_max_corners, 100, 1200);

    g_show_glasses_3dof = s.show_glasses_3dof;
    g_show_vision_cone = s.show_vision_cone;
    g_vision_cone_fov_deg = std::clamp(s.vision_cone_fov_deg, 20.0f, 120.0f);
    g_vision_cone_depth_m = std::clamp(s.vision_cone_depth_m, 0.3f, 6.0f);
    g_world_view_yaw_deg = std::clamp(s.world_view_yaw_deg, -180.0f, 180.0f);
    g_world_view_pitch_deg = std::clamp(s.world_view_pitch_deg, -89.0f, 89.0f);
    g_world_view_scale = std::clamp(s.world_view_scale, 20.0f, 180.0f);
    g_world_cam_distance = std::clamp(s.world_cam_distance, 0.8f, 20.0f);

    g_camera_panel_w = (std::max)(0.0f, s.camera_panel_w);
    g_full_panel_w = (std::max)(0.0f, s.full_panel_w);
    g_panel_sizes_loaded = (g_camera_panel_w > 0.0f || g_full_panel_w > 0.0f);
}

static bool load_ui_persist_settings(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;

    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return false; }
    long sz = ftell(f);
    if (sz <= 0) { fclose(f); return false; }
    if (fseek(f, 0, SEEK_SET) != 0) { fclose(f); return false; }

    std::string payload;
    payload.resize((size_t)sz);
    size_t got = fread(payload.data(), 1, payload.size(), f);
    fclose(f);
    if (got != payload.size()) return false;

    try {
        nlohmann::json j = nlohmann::json::parse(payload);
        UiPersistSettings s;

        s.flip_display = j.value("flip_display", s.flip_display);
        s.clahe_enabled = j.value("clahe_enabled", s.clahe_enabled);
        s.rect_enabled = j.value("rect_enabled", s.rect_enabled);
        s.feature_overlay_enabled = j.value("feature_overlay_enabled", s.feature_overlay_enabled);
        s.full_state_auto_refresh = j.value("full_state_auto_refresh", s.full_state_auto_refresh);
        s.full_state_freeze = j.value("full_state_freeze", s.full_state_freeze);
        s.console_autoscroll = j.value("console_autoscroll", s.console_autoscroll);

        s.feature_max_corners = j.value("feature_max_corners", s.feature_max_corners);

        s.show_glasses_3dof = j.value("show_glasses_3dof", s.show_glasses_3dof);
        s.show_vision_cone = j.value("show_vision_cone", s.show_vision_cone);
        s.vision_cone_fov_deg = j.value("vision_cone_fov_deg", s.vision_cone_fov_deg);
        s.vision_cone_depth_m = j.value("vision_cone_depth_m", s.vision_cone_depth_m);
        s.world_view_yaw_deg = j.value("world_view_yaw_deg", s.world_view_yaw_deg);
        s.world_view_pitch_deg = j.value("world_view_pitch_deg", s.world_view_pitch_deg);
        s.world_view_scale = j.value("world_view_scale", s.world_view_scale);
        s.world_cam_distance = j.value("world_cam_distance", s.world_cam_distance);

        s.camera_panel_w = (std::max)(0.0f, j.value("camera_panel_w", s.camera_panel_w));
        s.full_panel_w = (std::max)(0.0f, j.value("full_panel_w", s.full_panel_w));

        apply_ui_persist_settings_to_state(s);
        return true;
    } catch (...) {
        return false;
    }
}

static bool save_ui_persist_settings(const char* path) {
    UiPersistSettings s = read_ui_persist_settings_from_state();

    nlohmann::json j;
    j["flip_display"] = s.flip_display;
    j["clahe_enabled"] = s.clahe_enabled;
    j["rect_enabled"] = s.rect_enabled;
    j["feature_overlay_enabled"] = s.feature_overlay_enabled;
    j["full_state_auto_refresh"] = s.full_state_auto_refresh;
    j["full_state_freeze"] = s.full_state_freeze;
    j["console_autoscroll"] = s.console_autoscroll;

    j["feature_max_corners"] = s.feature_max_corners;

    j["show_glasses_3dof"] = s.show_glasses_3dof;
    j["show_vision_cone"] = s.show_vision_cone;
    j["vision_cone_fov_deg"] = s.vision_cone_fov_deg;
    j["vision_cone_depth_m"] = s.vision_cone_depth_m;
    j["world_view_yaw_deg"] = s.world_view_yaw_deg;
    j["world_view_pitch_deg"] = s.world_view_pitch_deg;
    j["world_view_scale"] = s.world_view_scale;
    j["world_cam_distance"] = s.world_cam_distance;

    j["camera_panel_w"] = s.camera_panel_w;
    j["full_panel_w"] = s.full_panel_w;

    std::string out = j.dump(2);
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    size_t wrote = fwrite(out.data(), 1, out.size(), f);
    fclose(f);
    return wrote == out.size();
}

static void mark_settings_dirty() {
    g_settings_dirty = true;
}

static void maybe_save_ui_settings_throttled() {
    if (!g_settings_dirty) return;
    uint64_t now = now_tick_ms();
    if (now - g_settings_last_save_ms < kSettingsSaveThrottleMs) return;
    if (save_ui_persist_settings(kUiSettingsPath)) {
        g_settings_dirty = false;
        g_settings_last_save_ms = now;
    }
}

static void save_ui_settings_now() {
    if (save_ui_persist_settings(kUiSettingsPath)) {
        g_settings_dirty = false;
        g_settings_last_save_ms = now_tick_ms();
    }
}

static bool load_ui_settings_now() {
    bool ok = load_ui_persist_settings(kUiSettingsPath);
    g_settings_last_save_ms = now_tick_ms();
    return ok;
}

static bool g_ignore_setting_changes_this_frame = false;

static bool set_checkbox_and_track(const char* label, bool* v) {
    bool changed = ImGui::Checkbox(label, v);
    if (changed && !g_ignore_setting_changes_this_frame) mark_settings_dirty();
    return changed;
}

static bool set_slider_int_and_track(const char* label, int* v, int min_v, int max_v) {
    bool changed = ImGui::SliderInt(label, v, min_v, max_v);
    if (changed && !g_ignore_setting_changes_this_frame) mark_settings_dirty();
    return changed;
}

static bool set_slider_float_and_track(const char* label, float* v, float min_v, float max_v, const char* fmt) {
    bool changed = ImGui::SliderFloat(label, v, min_v, max_v, fmt);
    if (changed && !g_ignore_setting_changes_this_frame) mark_settings_dirty();
    return changed;
}

static bool set_button_and_track(const char* label) {
    bool clicked = ImGui::Button(label);
    if (clicked && !g_ignore_setting_changes_this_frame) mark_settings_dirty();
    return clicked;
}

static int add_feature_overlay(const cv::Mat& gray, cv::Mat& out_bgr) {
    cv::cvtColor(gray, out_bgr, cv::COLOR_GRAY2BGR);
    if (!g_feature_overlay_enabled) return 0;

    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(gray, corners, g_feature_max_corners, 0.01, 8.0);
    for (const auto& p : corners)
        cv::circle(out_bgr, p, 2, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    return (int)corners.size();
}

static void UploadCameraTextureBgr(ID3D11Texture2D* tex, const cv::Mat& bgr) {
    if (!tex || bgr.empty()) return;
    D3D11_MAPPED_SUBRESOURCE mapped = {};
    if (FAILED(g_pd3dDeviceContext->Map(tex, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) return;
    for (int y = 0; y < bgr.rows; ++y) {
        const uint8_t* src = bgr.ptr<uint8_t>(y);
        uint8_t* dst = (uint8_t*)mapped.pData + y * mapped.RowPitch;
        for (int x = 0; x < bgr.cols; ++x) {
            dst[x * 4 + 0] = src[x * 3 + 2];
            dst[x * 4 + 1] = src[x * 3 + 1];
            dst[x * 4 + 2] = src[x * 3 + 0];
            dst[x * 4 + 3] = 255;
        }
    }
    g_pd3dDeviceContext->Unmap(tex, 0);
}

static void draw_pose_widget(const char* id, const ImVec2& size) {
    ImGui::BeginChild(id, size, true);
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    dl->AddRectFilled(p0, ImVec2(p0.x + avail.x, p0.y + avail.y), IM_COL32(16, 18, 24, 255));
    dl->AddRect(p0, ImVec2(p0.x + avail.x, p0.y + avail.y), IM_COL32(80, 85, 95, 255));

    const ImVec2 c(p0.x + avail.x * 0.5f, p0.y + avail.y * 0.5f);
    const float yaw = g_pose_yaw_deg * 0.0174532925f;
    const float pitch = g_pose_pitch_deg * 0.0174532925f;
    const float roll = g_pose_roll_deg * 0.0174532925f;

    auto proj = [&](float x, float y, float z) {
        float cr = std::cos(roll), sr = std::sin(roll);
        float cp = std::cos(pitch), sp = std::sin(pitch);
        float cy = std::cos(yaw), sy = std::sin(yaw);
        float xr = cy * (cp * x + sp * (sr * y + cr * z)) - sy * (cr * y - sr * z);
        float yr = sy * (cp * x + sp * (sr * y + cr * z)) + cy * (cr * y - sr * z);
        float zr = -sp * x + cp * (sr * y + cr * z);
        float s = (std::min)(avail.x, avail.y) * 0.26f;
        return ImVec2(c.x + xr * s, c.y - yr * s - zr * s * 0.25f);
    };

    ImVec2 o = proj(0, 0, 0);
    ImVec2 x = proj(1, 0, 0);
    ImVec2 y = proj(0, 1, 0);
    ImVec2 z = proj(0, 0, 1);

    dl->AddLine(o, x, IM_COL32(255, 80, 80, 255), 2.0f);
    dl->AddLine(o, y, IM_COL32(80, 255, 120, 255), 2.0f);
    dl->AddLine(o, z, IM_COL32(80, 160, 255, 255), 2.0f);
    dl->AddCircleFilled(o, 3.0f, IM_COL32(230, 230, 230, 255));

    char buf[256];
    snprintf(buf, sizeof(buf), "6DoF pose | q=%d | xyz=(%.2f, %.2f, %.2f)", g_6dof_quality, g_pose_x_m, g_pose_y_m, g_pose_z_m);
    dl->AddText(ImVec2(p0.x + 8, p0.y + 8), IM_COL32(210, 230, 255, 255), buf);

    ImGui::Dummy(avail);
    ImGui::EndChild();
}

static void draw_glasses_3dof_widget(const char* id, const ImVec2& size) {
    ImGui::BeginChild(id, size, true);
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    if (avail.x < 10.0f || avail.y < 10.0f) {
        ImGui::EndChild();
        return;
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + avail.x, p0.y + avail.y), IM_COL32(12, 16, 22, 255));
    dl->AddRect(p0, ImVec2(p0.x + avail.x, p0.y + avail.y), IM_COL32(80, 85, 95, 255));

    const float yaw_pose = g_pose_yaw_deg * 0.0174532925f;
    const float pitch_pose = g_pose_pitch_deg * 0.0174532925f;
    const float roll_pose = g_pose_roll_deg * 0.0174532925f;

    const float cyh = std::cos(yaw_pose), syh = std::sin(yaw_pose);
    const float cph = std::cos(pitch_pose), sph = std::sin(pitch_pose);
    const float crh = std::cos(roll_pose), srh = std::sin(roll_pose);

    const float yaw_view = g_world_view_yaw_deg * 0.0174532925f;
    const float pitch_view = g_world_view_pitch_deg * 0.0174532925f;
    const float cyv = std::cos(yaw_view), syv = std::sin(yaw_view);
    const float cpv = std::cos(pitch_view), spv = std::sin(pitch_view);

    auto rotate_head = [&](float x, float y, float z, float& ox, float& oy, float& oz) {
        ox = cyh * (cph * x + sph * (srh * y + crh * z)) - syh * (crh * y - srh * z);
        oy = syh * (cph * x + sph * (srh * y + crh * z)) + cyh * (crh * y - srh * z);
        oz = -sph * x + cph * (srh * y + crh * z);
    };

    auto project_world = [&](float x, float y, float z, ImVec2& out) -> bool {
        float xr = cyv * x - syv * z;
        float zr = syv * x + cyv * z;
        float yr = cpv * y - spv * zr;
        float zr2 = spv * y + cpv * zr;
        float d = zr2 + g_world_cam_distance;
        if (d <= 0.1f) return false;
        float s = g_world_view_scale / d;
        out.x = p0.x + avail.x * 0.5f + xr * s;
        out.y = p0.y + avail.y * 0.63f - yr * s;
        return true;
    };

    auto draw_seg = [&](float ax, float ay, float az, float bx, float by, float bz, ImU32 col, float thick) {
        float arx, ary, arz, brx, bry, brz;
        rotate_head(ax, ay, az, arx, ary, arz);
        rotate_head(bx, by, bz, brx, bry, brz);
        ImVec2 pa, pb;
        if (project_world(arx, ary, arz, pa) && project_world(brx, bry, brz, pb)) {
            dl->AddLine(pa, pb, col, thick);
        }
    };

    const float hw = 0.130f;
    const float hh = 0.030f;
    const float zf = 0.12f;
    const float zt = 0.02f;

    draw_seg(-hw,  hh, zf,  hw,  hh, zf, IM_COL32(230, 230, 240, 255), 2.0f);
    draw_seg(-hw, -hh, zf,  hw, -hh, zf, IM_COL32(230, 230, 240, 255), 2.0f);
    draw_seg(-hw, -hh, zf, -hw,  hh, zf, IM_COL32(230, 230, 240, 255), 2.0f);
    draw_seg( hw, -hh, zf,  hw,  hh, zf, IM_COL32(230, 230, 240, 255), 2.0f);
    draw_seg(-0.015f, 0.0f, zf, 0.015f, 0.0f, zf, IM_COL32(255, 210, 110, 255), 2.4f);

    draw_seg(-hw,  hh, zf, -0.165f, 0.020f, zt, IM_COL32(180, 190, 215, 255), 1.8f);
    draw_seg( hw,  hh, zf,  0.165f, 0.020f, zt, IM_COL32(180, 190, 215, 255), 1.8f);

    draw_seg(0.0f, 0.0f, 0.0f, 0.18f, 0.0f, 0.0f, IM_COL32(255, 90, 90, 255), 2.0f);
    draw_seg(0.0f, 0.0f, 0.0f, 0.0f, 0.18f, 0.0f, IM_COL32(90, 255, 130, 255), 2.0f);
    draw_seg(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.18f, IM_COL32(90, 170, 255, 255), 2.0f);

    if (g_show_vision_cone) {
        const float fov = std::clamp(g_vision_cone_fov_deg, 10.0f, 140.0f) * 0.0174532925f;
        const float cone_z = std::clamp(g_vision_cone_depth_m, 0.2f, 8.0f);
        const float cone_hw = std::tan(0.5f * fov) * cone_z;
        const float cone_hh = cone_hw * 0.58f;

        const ImU32 cone_col = IM_COL32(80, 185, 255, 120);
        draw_seg(0.0f, 0.0f, 0.0f, -cone_hw, -cone_hh, cone_z, cone_col, 1.4f);
        draw_seg(0.0f, 0.0f, 0.0f,  cone_hw, -cone_hh, cone_z, cone_col, 1.4f);
        draw_seg(0.0f, 0.0f, 0.0f,  cone_hw,  cone_hh, cone_z, cone_col, 1.4f);
        draw_seg(0.0f, 0.0f, 0.0f, -cone_hw,  cone_hh, cone_z, cone_col, 1.4f);

        draw_seg(-cone_hw, -cone_hh, cone_z, cone_hw, -cone_hh, cone_z, cone_col, 1.0f);
        draw_seg( cone_hw, -cone_hh, cone_z, cone_hw,  cone_hh, cone_z, cone_col, 1.0f);
        draw_seg( cone_hw,  cone_hh, cone_z,-cone_hw,  cone_hh, cone_z, cone_col, 1.0f);
        draw_seg(-cone_hw,  cone_hh, cone_z,-cone_hw, -cone_hh, cone_z, cone_col, 1.0f);
    }

    char txt[256];
    snprintf(txt, sizeof(txt), "3DoF glasses | yaw=%.1f pitch=%.1f roll=%.1f", g_pose_yaw_deg, g_pose_pitch_deg, g_pose_roll_deg);
    dl->AddText(ImVec2(p0.x + 8, p0.y + 8), IM_COL32(205, 225, 255, 255), txt);
    dl->AddText(ImVec2(p0.x + 8, p0.y + 26), IM_COL32(160, 185, 210, 255), "Head frame renderer + vision cone");

    ImGui::Dummy(avail);
    ImGui::EndChild();
}

static bool g_settings_loaded_once = false;

static const char* raw_na() {
    return "N/A";
}

static const char* camera_model_name(CameraModel model) {
    switch (model) {
        case CameraModel::Pinhole: return "Pinhole";
        case CameraModel::Fisheye624: return "Fisheye624";
        case CameraModel::Fisheye4: return "Fisheye4";
        default: return "Unknown";
    }
}

static std::string build_full_state_snapshot(
    bool imu_ok,
    bool cam_ok,
    bool have_cam,
    bool have_imu_sample,
    const ImuSample& last_imu,
    int imu_q,
    int cam_q,
    int imu_buffer_q,
    float app_fps,
    int drained_frame,
    int drained_total,
    int imu_window,
    int imu_window_total,
    bool imu_clock_converged,
    const StereoPair* cam_pair) {

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    oss << "XREAL Air 2 Ultra — Raw Device Streams\n";
    oss << "======================================\n\n";

    oss << "[DEVICE STATUS]\n";
    oss << "imu_ok=" << (imu_ok ? "YES" : "NO")
        << " cam_ok=" << (cam_ok ? "YES" : "NO")
        << " have_cam_pair=" << (have_cam ? "YES" : "NO") << "\n";
    oss << "imu_queue=" << imu_q << " cam_queue=" << cam_q
        << " imu_buffer=" << imu_buffer_q
        << " imu_fps=" << g_last_imu_fps
        << " app_fps=" << app_fps
        << " clock_converged=" << (imu_clock_converged ? "YES" : "NO") << "\n";
    oss << "drained_frame=" << drained_frame
        << " drained_total=" << drained_total
        << " imu_window=" << imu_window
        << " imu_window_total=" << imu_window_total << "\n\n";

    DeviceInfo dev_info = {};
    DeviceState dev_state = {};
    const int info_ok = imu_get_device_info(&dev_info);
    imu_refresh_device_state();
    const int state_ok = imu_get_device_state(&dev_state);

    oss << "[RAW DEVICE INFO/STATE]\n";
    if (info_ok == 0 && dev_info.valid) {
        oss << "vendor_id=0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << dev_info.vendor_id
            << " product_id=0x" << std::setw(4) << dev_info.product_id
            << " release_bcd=0x" << std::setw(4) << dev_info.release_bcd << std::dec << std::setfill(' ') << "\n";
        oss << "manufacturer=" << (dev_info.manufacturer[0] ? dev_info.manufacturer : raw_na()) << "\n";
        oss << "product=" << (dev_info.product[0] ? dev_info.product : raw_na()) << "\n";
        oss << "serial=" << (dev_info.serial[0] ? dev_info.serial : raw_na()) << "\n";
        oss << "firmware=" << (dev_info.firmware[0] ? dev_info.firmware : raw_na()) << "\n";
    } else {
        oss << "vendor_id=" << raw_na() << "\n";
        oss << "product_id=" << raw_na() << "\n";
        oss << "release_bcd=" << raw_na() << "\n";
        oss << "manufacturer=" << raw_na() << "\n";
        oss << "product=" << raw_na() << "\n";
        oss << "serial=" << raw_na() << "\n";
        oss << "firmware=" << raw_na() << "\n";
    }

    if (state_ok == 0 && dev_state.valid) {
        oss << "connected=" << (dev_state.connected ? "YES" : "NO")
            << " imu_streaming=" << (dev_state.imu_streaming ? "YES" : "NO")
            << " state_timestamp_ns=" << dev_state.timestamp_ns << "\n";
    } else {
        oss << "connected=" << raw_na() << " imu_streaming=" << raw_na()
            << " state_timestamp_ns=" << raw_na() << "\n";
    }

    if (state_ok == 0 && dev_state.led_state_valid) {
        oss << "led_state=" << dev_state.led_state << "\n";
    } else {
        oss << "led_state=" << raw_na() << "\n";
    }

    if (state_ok == 0 && dev_state.mode_3d_valid) {
        oss << "mode_3d_enabled=" << (dev_state.mode_3d_enabled ? "YES" : "NO") << "\n";
    } else {
        oss << "mode_3d_enabled=" << raw_na() << "\n";
    }

    if (state_ok == 0 && dev_state.brightness_valid) {
        oss << "brightness_percent=" << dev_state.brightness_percent << "\n";
    } else {
        oss << "brightness_percent=" << raw_na() << "\n";
    }

    oss << "\n";

    oss << "[RAW IMU SAMPLE]\n";
    if (have_imu_sample) {
        oss << "timestamp_ns=" << last_imu.timestamp_ns
            << " temperature_c=" << last_imu.temperature_c
            << " fps=" << last_imu.fps << "\n";
        oss << "accel_mps2=[" << last_imu.accel[0] << ", " << last_imu.accel[1] << ", " << last_imu.accel[2] << "]\n";
        oss << "gyro_rads=[" << last_imu.gyro[0] << ", " << last_imu.gyro[1] << ", " << last_imu.gyro[2] << "]\n";
    } else {
        oss << "No IMU sample yet.\n";
    }
    oss << "\n";

    oss << "[RAW STEREO PAIR]\n";
    if (cam_pair) {
        oss << "pair_timestamp_ns=" << cam_pair->timestamp_ns
            << " sync_delta_ms=" << cam_pair->sync_delta_ms
            << " drops=" << cam_pair->drops << "\n";

        auto append_frame = [&oss](const char* name, const CameraFrame& f) {
            oss << name << ":"
                << " ts_ns=" << f.timestamp_ns
                << " idx=" << f.frame_index
                << " size=" << f.width << "x" << f.height
                << " camera_id=" << f.camera_id
                << " fps=" << f.fps
                << " is_rectified=" << (f.is_rectified ? "YES" : "NO")
                << " data_ptr=" << (f.data ? "set" : "null") << "\n";
        };
        append_frame("left", cam_pair->left);
        append_frame("right", cam_pair->right);
    } else {
        oss << "No stereo pair yet.\n";
    }

    return oss.str();
}

static void refresh_full_state_snapshot(
    bool imu_ok,
    bool cam_ok,
    bool have_cam,
    bool have_imu_sample,
    const ImuSample& last_imu,
    int imu_q,
    int cam_q,
    int imu_buffer_q,
    float app_fps,
    int drained_frame,
    int drained_total,
    int imu_window,
    int imu_window_total,
    bool imu_clock_converged,
    const StereoPair* cam_pair) {
    g_full_state_buffer = build_full_state_snapshot(
        imu_ok,
        cam_ok,
        have_cam,
        have_imu_sample,
        last_imu,
        imu_q,
        cam_q,
        imu_buffer_q,
        app_fps,
        drained_frame,
        drained_total,
        imu_window,
        imu_window_total,
        imu_clock_converged,
        cam_pair);
}

static std::string mutable_text_buffer;
static const char* readonly_text_ptr(const std::string& s) {
    mutable_text_buffer = s;
    mutable_text_buffer.push_back('\0');
    return mutable_text_buffer.data();
}

static void refresh_console_feed() {
    const char* path = "imu_debug.log";
    FILE* f = fopen(path, "rb");
    if (!f) return;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return; }
    long size = ftell(f);
    if (size < 0) { fclose(f); return; }

    if ((uint64_t)size < g_console_last_size) {
        g_console_buffer.clear();
        g_console_last_size = 0;
    }

    if ((uint64_t)size == g_console_last_size) {
        fclose(f);
        return;
    }

    if (fseek(f, (long)g_console_last_size, SEEK_SET) != 0) { fclose(f); return; }
    size_t append_n = (size_t)((uint64_t)size - g_console_last_size);
    size_t old_sz = g_console_buffer.size();
    g_console_buffer.resize(old_sz + append_n);
    size_t got = fread(&g_console_buffer[old_sz], 1, append_n, f);
    g_console_buffer.resize(old_sz + got);
    g_console_last_size += (uint64_t)got;

    const size_t max_keep = 128 * 1024;
    if (g_console_buffer.size() > max_keep) {
        size_t drop = g_console_buffer.size() - max_keep;
        size_t nl = g_console_buffer.find('\n', drop);
        if (nl != std::string::npos) drop = nl + 1;
        g_console_buffer.erase(0, drop);
    }

    fclose(f);
}

static uint64_t now_tick_ms() {
    return GetTickCount64();
}

static void draw_camera_block(bool cam_ok, bool have_cam, bool g_camTexOK_local) {
    const CalibrationData* ui_calib = g_runtime_calib;
    if (cam_ok && have_cam && g_camTexOK_local) {
        float avail_w = ImGui::GetContentRegionAvail().x;
        constexpr int columns = 2;
        float spacing = ImGui::GetStyle().ItemSpacing.x;
        float pw = (avail_w - spacing * (columns - 1)) / (float)columns;
        if (pw < 120.0f) pw = 120.0f;
        float ph = pw * 640.0f / 480.0f;

        ImGui::PushID("camera_feeds");
        if (ImGui::BeginTable("feed_table", columns, ImGuiTableFlags_SizingStretchSame)) {
            for (int c = 0; c < columns; ++c)
                ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch, 1.0f);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Image((ImTextureID)g_srvLeft, ImVec2(pw, ph));
            ImGui::TableSetColumnIndex(1);
            ImGui::Image((ImTextureID)g_srvRight, ImVec2(pw, ph));

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("LEFT");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted("RIGHT");

            ImGui::EndTable();
        }
        ImGui::PopID();

        ImGui::TextColored(g_rect_enabled ? ImVec4(0.4f, 1.0f, 0.5f, 1.0f) : ImVec4(1.0f, 0.6f, 0.3f, 1.0f),
            "%s (%dx%d)", g_rect_enabled ? (g_rectifier.initialized() ? "RECTIFIED" : "RECTIFY (no calib!)") : "RAW", STEREO_EYE_WIDTH, STEREO_EYE_HEIGHT);

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.8f, 0.9f, 1.0f, 1.0f), "Stereo preprocessing settings");
        set_checkbox_and_track("Flip stereo display", &g_flip_display);
        ImGui::SameLine();
        set_checkbox_and_track("CLAHE", &g_clahe_enabled);
        ImGui::SameLine();
        set_checkbox_and_track("Rectify", &g_rect_enabled);

        set_checkbox_and_track("Feature overlay", &g_feature_overlay_enabled);
        set_slider_int_and_track("Feature max corners", &g_feature_max_corners, 100, 1200);

        ImGui::Text("Features: %d", g_feature_count);
        set_slider_float_and_track("3D view yaw", &g_world_view_yaw_deg, -180.0f, 180.0f, "%.1f");
        set_slider_float_and_track("3D view pitch", &g_world_view_pitch_deg, -89.0f, 89.0f, "%.1f");
        set_slider_float_and_track("3D view scale", &g_world_view_scale, 20.0f, 180.0f, "%.1f");
        set_slider_float_and_track("3D camera distance", &g_world_cam_distance, 0.8f, 20.0f, "%.1f");
        if (set_button_and_track("Reset 3D view")) {
            g_world_view_yaw_deg = 35.0f;
            g_world_view_pitch_deg = -20.0f;
            g_world_view_scale = 70.0f;
            g_world_cam_distance = 5.0f;
        }
        if (!g_track_status.empty()) ImGui::TextUnformatted(g_track_status.c_str());

        ImGui::SeparatorText("6DoF");
        ImGui::Text("pos [m]: x=%.2f y=%.2f z=%.2f", g_pose_x_m, g_pose_y_m, g_pose_z_m);
        ImGui::Text("rpy [deg]: roll=%.1f pitch=%.1f yaw=%.1f", g_pose_roll_deg, g_pose_pitch_deg, g_pose_yaw_deg);
        ImGui::Text("quality: %d", g_6dof_quality);

        draw_pose_widget("pose_widget", ImVec2(-1.0f, 130.0f));

        ImGui::SeparatorText("3DoF glasses renderer");
        set_checkbox_and_track("Show 3DoF glasses", &g_show_glasses_3dof);
        ImGui::SameLine();
        set_checkbox_and_track("Show vision cone", &g_show_vision_cone);
        set_slider_float_and_track("Vision cone FOV", &g_vision_cone_fov_deg, 20.0f, 120.0f, "%.1f deg");
        set_slider_float_and_track("Vision cone depth", &g_vision_cone_depth_m, 0.3f, 6.0f, "%.2f m");
        if (g_show_glasses_3dof) {
            draw_glasses_3dof_widget("glasses_3dof_widget", ImVec2(-1.0f, 170.0f));
        }

        ImGui::SeparatorText("Camera calibration");
        if (ui_calib && ui_calib->is_valid) {
            ImGui::Text("Status: loaded");
            ImGui::Text("Left  model=%s  size=%dx%d  fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                camera_model_name(ui_calib->left.model), ui_calib->left.width, ui_calib->left.height,
                ui_calib->left.fx, ui_calib->left.fy, ui_calib->left.cx, ui_calib->left.cy);
            ImGui::Text("Right model=%s  size=%dx%d  fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                camera_model_name(ui_calib->right.model), ui_calib->right.width, ui_calib->right.height,
                ui_calib->right.fx, ui_calib->right.fy, ui_calib->right.cx, ui_calib->right.cy);
            ImGui::Text("Baseline: %.3f m", ui_calib->baseline_m);
            ImGui::Text("T_left_right t [m]: [%.4f %.4f %.4f]",
                ui_calib->T_left_right(0, 3), ui_calib->T_left_right(1, 3), ui_calib->T_left_right(2, 3));
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.35f, 1.0f), "Status: missing or invalid calibration");
        }

    } else {
        ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1),
            cam_ok ? "Waiting for camera frames..." : "Camera not available (no XREAL glasses?)");
    }
}

static bool CreateCameraTextures() {
    D3D11_TEXTURE2D_DESC desc = {};
    desc.Width = STEREO_EYE_WIDTH; desc.Height = STEREO_EYE_HEIGHT;
    desc.MipLevels = 1; desc.ArraySize = 1;
    desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    desc.SampleDesc.Count = 1;
    desc.Usage = D3D11_USAGE_DYNAMIC;
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

    if (FAILED(g_pd3dDevice->CreateTexture2D(&desc, nullptr, &g_texLeft))) return false;
    if (FAILED(g_pd3dDevice->CreateTexture2D(&desc, nullptr, &g_texRight))) return false;

    D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    srvDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
    srvDesc.Texture2D.MipLevels = 1;

    if (FAILED(g_pd3dDevice->CreateShaderResourceView(g_texLeft, &srvDesc, &g_srvLeft))) return false;
    if (FAILED(g_pd3dDevice->CreateShaderResourceView(g_texRight, &srvDesc, &g_srvRight))) return false;
    return true;
}

static void UploadCameraTexture(ID3D11Texture2D* tex, const uint8_t* pixels, int w, int h) {
    D3D11_MAPPED_SUBRESOURCE mapped = {};
    if (FAILED(g_pd3dDeviceContext->Map(tex, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) return;
    for (int y = 0; y < h; ++y) {
        uint8_t* dst = (uint8_t*)mapped.pData + y * mapped.RowPitch;
        const uint8_t* src = pixels + y * w;
        for (int x = 0; x < w; ++x) {
            dst[x*4 + 0] = src[x]; // R = grey
            dst[x*4 + 1] = src[x]; // G = grey
            dst[x*4 + 2] = src[x]; // B = grey
            dst[x*4 + 3] = 255;    // A = opaque
        }
    }
    g_pd3dDeviceContext->Unmap(tex, 0);
}

static void ReleaseCameraTextures() {
    if (g_srvLeft)  { g_srvLeft->Release();  g_srvLeft  = nullptr; }
    if (g_srvRight) { g_srvRight->Release(); g_srvRight = nullptr; }
    if (g_texLeft)  { g_texLeft->Release();  g_texLeft  = nullptr; }
    if (g_texRight) { g_texRight->Release(); g_texRight = nullptr; }
}
static HWND g_hwnd = nullptr;
static WNDCLASSEXW g_wc = {};
static bool g_running = true;

static bool CreateDeviceD3D(HWND hWnd) {
    DXGI_SWAP_CHAIN_DESC sd = {};
    sd.BufferCount = 2;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    const D3D_FEATURE_LEVEL fl[] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0 };
    D3D_FEATURE_LEVEL fl_out;
    HRESULT hr = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0,
        fl, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &fl_out, &g_pd3dDeviceContext);
    if (FAILED(hr)) return false;

    ID3D11Texture2D* pBB = nullptr;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBB));
    if (pBB) {
        g_pd3dDevice->CreateRenderTargetView(pBB, nullptr, &g_mainRenderTargetView);
        pBB->Release();
    }
    return true;
}

static void CleanupDeviceD3D() {
    ReleaseCameraTextures();
    if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
    if (g_pSwapChain)          { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
    if (g_pd3dDeviceContext)   { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
    if (g_pd3dDevice)          { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
}

static void RenderFrame() {
    ImGui::Render();
    const float cc[4] = { 0.10f, 0.12f, 0.15f, 1.00f };
    g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
    g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, cc);
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
    g_pSwapChain->Present(1, 0);
}

// Win32
static LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam)) return true;
    switch (msg) {
    case WM_SIZE:
        if (g_pd3dDevice && wParam != SIZE_MINIMIZED) {
            if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
            g_pSwapChain->ResizeBuffers(0, LOWORD(lParam), HIWORD(lParam), DXGI_FORMAT_UNKNOWN, 0);
            ID3D11Texture2D* pBB = nullptr;
            g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBB));
            if (pBB) { g_pd3dDevice->CreateRenderTargetView(pBB, nullptr, &g_mainRenderTargetView); pBB->Release(); }
        }
        return 0;
    case WM_DESTROY: g_running = false; PostQuitMessage(0); return 0;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

static bool CreateWin32Window() {
    g_wc = { sizeof(g_wc), CS_CLASSDC, WndProc, 0L, 0L,
             GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr, L"IMUDebug", nullptr };
    RegisterClassExW(&g_wc);
    g_hwnd = CreateWindowW(g_wc.lpszClassName, L"XREAL IMU + Stereo Camera",
        WS_OVERLAPPEDWINDOW, 100, 100, 1440, 960, nullptr, nullptr, g_wc.hInstance, nullptr);
    if (!g_hwnd) return false;
    ShowWindow(g_hwnd, SW_SHOWDEFAULT);
    UpdateWindow(g_hwnd);
    return true;
}

static void push_history(std::deque<float>& dq, float val) {
    dq.push_back(val);
    while ((int)dq.size() > kHistory) dq.pop_front();
}

static const char* g_cam_calib_path = "calibration.json";

// Main
int main(int argc, char** argv) {
    // Set CWD to exe directory so relative paths (calibration.json, etc.) resolve
    char exe_path[MAX_PATH];
    DWORD n = GetModuleFileNameA(nullptr, exe_path, MAX_PATH);
    if (n > 0 && n < MAX_PATH) {
        std::string ep(exe_path);
        size_t pos = ep.find_last_of("\\/");
        if (pos != std::string::npos) SetCurrentDirectoryA(ep.substr(0, pos).c_str());
    }

    // Redirect stdout/stderr to log file for diagnostics
    freopen("imu_debug.log", "w", stdout);
    freopen("imu_debug.log", "a", stderr);
    setvbuf(stdout, nullptr, _IONBF, 0);  // unbuffered
    setvbuf(stderr, nullptr, _IONBF, 0);

    const char* calib_path = argc > 1 ? argv[1] : "calibration.json";
    if (argc > 2) g_cam_calib_path = argv[2];

    bool imu_ok = (imu_init(calib_path) == 0);
    if (imu_ok) imu_start_streaming();

    bool cam_ok = (stereo_init() == 0);
    if (cam_ok) cam_ok = (stereo_start_streaming() == 0);

    // Load VIO calibration (intrinsics, extrinsics, IMU noise)
    const char* cam_calib_path = argc > 2 ? argv[2] : "calibration.json";
    CalibrationData* calib = calibration_load(cam_calib_path, true);
    if (calib) {
        calibration_set_singleton(calib);
        imu_set_calibration(calib);
        stereo_set_calibration(calib);
        g_rectifier.init(*calib);

    }

    if (!imu_ok && !cam_ok) {
        fprintf(stderr, "Neither IMU nor camera available.\n");
        return 1;
    }

    if (!CreateWin32Window() || !CreateDeviceD3D(g_hwnd)) {
        imu_stop_streaming(); imu_shutdown();
        stereo_stop_streaming(); stereo_shutdown();
        return 1;
    }

    g_camTexOK = CreateCameraTextures();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    g_ignore_setting_changes_this_frame = true;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplWin32_Init(g_hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    if (!g_settings_loaded_once) {
        load_ui_settings_now();
        g_settings_loaded_once = true;
    }

    ImuSample sample;
    ImuSample last_imu_sample = {};
    bool have_imu_sample = false;
    StereoPair cam_pair;
    bool have_cam = false;

    static StereoPair g_rect_pair;
    std::deque<ImuSample> imu_queue;
    uint64_t prev_frame_ts_ns = 0;
    int drained_pairs_total = 0;
    int imu_window_total = 0;
    int drained_now = 0;
    int imu_samples_this_frame = 0;
    bool imu_clock_converged = false;
    bool have_new_cam_frame = false;

    ReleaseCameraTextures();
    g_camTexOK = CreateCameraTextures();
    if (!g_camTexOK) {
        fprintf(stderr, "Failed to create camera textures.\n");
    }


    while (g_running) {
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
            TranslateMessage(&msg); DispatchMessageW(&msg);
            if (msg.message == WM_QUIT) g_running = false;
        }
        if (!g_running) break;

        if (g_hotplug_reconnect_request) {
            g_hotplug_reconnect_request = false;

            imu_stop_streaming();
            imu_shutdown();
            stereo_stop_streaming();
            stereo_shutdown();

            imu_ok = (imu_init(calib_path) == 0);
            if (imu_ok) imu_start_streaming();

            cam_ok = (stereo_init() == 0);
            if (cam_ok) cam_ok = (stereo_start_streaming() == 0);

            have_imu_sample = false;
            have_cam = false;
            g_last_imu_fps = 0.0f;
            imu_queue.clear();
            prev_frame_ts_ns = 0;
            drained_pairs_total = 0;
            imu_window_total = 0;
            drained_now = 0;
            imu_samples_this_frame = 0;
            imu_clock_converged = false;
            have_new_cam_frame = false;

            g_console_buffer.clear();
            g_console_last_size = 0;
        }

        while (imu_poll_sample(&sample)) {
            push_history(g_hist_fps, sample.fps);
            g_last_imu_fps = sample.fps;
            last_imu_sample = sample;
            have_imu_sample = true;
            imu_queue.push_back(sample);
        }

        drained_now = 0;
        have_new_cam_frame = false;
        if (stereo_poll_pair(&cam_pair)) {
            have_cam = true;
            have_new_cam_frame = true;
            while (stereo_available_pairs() > 0) {
                if (!stereo_poll_pair(&cam_pair)) break;
                drained_now++;
            }
            drained_pairs_total += drained_now;
        }

        imu_samples_this_frame = 0;
        bool rect_pair_ready = false;
        if (have_new_cam_frame) {
            const bool do_rectify_for_time = g_rect_enabled && g_rectifier.initialized();
            if (do_rectify_for_time) {
                g_rect_pair = g_rectifier.rectify(cam_pair);
                rect_pair_ready = true;
            }
            const uint64_t frame_ts_ns = do_rectify_for_time ? g_rect_pair.timestamp_ns : cam_pair.timestamp_ns;
            const uint64_t lower_bound_ns = (prev_frame_ts_ns > 0)
                ? prev_frame_ts_ns
                : (frame_ts_ns > 50000000ULL ? frame_ts_ns - 50000000ULL : 0ULL);

            while (!imu_queue.empty()) {
                const ImuSample& front = imu_queue.front();
                const uint64_t imu_ts_sys_ns = clock_to_system_ns(front.timestamp_ns);

                if (imu_ts_sys_ns <= lower_bound_ns) {
                    imu_queue.pop_front();
                    continue;
                }

                if (imu_ts_sys_ns <= frame_ts_ns) {
                    imu_samples_this_frame++;
                    imu_queue.pop_front();
                    continue;
                }

                break;
            }

            imu_window_total += imu_samples_this_frame;
            prev_frame_ts_ns = frame_ts_ns;
            imu_clock_converged = clock_is_converged() != 0;
        }

        if (have_cam && g_camTexOK) {
            const bool do_rectify = g_rect_enabled && g_rectifier.initialized();

            if (do_rectify && !rect_pair_ready) {
                g_rect_pair = g_rectifier.rectify(cam_pair);
                rect_pair_ready = true;
            }

            // Select source images: rectified or raw
            int src_h = do_rectify ? g_rect_pair.left.height  : cam_pair.left.height;
            int src_w = do_rectify ? g_rect_pair.left.width   : cam_pair.left.width;
            const uint8_t* src_l = do_rectify ? g_rect_pair.left.data  : cam_pair.left.data;
            const uint8_t* src_r = do_rectify ? g_rect_pair.right.data : cam_pair.right.data;
            cv::Mat left_src(src_h, src_w, CV_8UC1, const_cast<uint8_t*>(src_l));
            cv::Mat right_src(src_h, src_w, CV_8UC1, const_cast<uint8_t*>(src_r));

            cv::Mat left_proc = left_src.clone();
            cv::Mat right_proc = right_src.clone();

            if (g_flip_display) {
                cv::flip(left_proc, left_proc, 1);
                cv::flip(right_proc, right_proc, 1);
            }

            if (g_clahe_enabled) {
                if (!g_clahe) g_clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
                g_clahe->apply(left_proc, left_proc);
                g_clahe->apply(right_proc, right_proc);
            }

            const int left_features = add_feature_overlay(left_proc, g_left_overlay_bgr);
            const int right_features = add_feature_overlay(right_proc, g_right_overlay_bgr);
            g_feature_count = (left_features + right_features) / 2;

            UploadCameraTextureBgr(g_texLeft, g_left_overlay_bgr);
            UploadCameraTextureBgr(g_texRight, g_right_overlay_bgr);
            g_latest_left_display = left_proc;
            g_latest_right_display = right_proc;

            g_track_status = imu_clock_converged ? "6DoF: tracking" : "6DoF: waiting clock converge";

            if (have_imu_sample) {
                const float ax = last_imu_sample.accel[0];
                const float ay = last_imu_sample.accel[1];
                const float az = last_imu_sample.accel[2];
                const float g = std::sqrt(ax * ax + ay * ay + az * az) + 1e-6f;

                g_pose_pitch_deg = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * 57.29578f;
                g_pose_roll_deg = std::atan2(ay, az) * 57.29578f;

                if (g_last_pose_imu_ts_ns > 0 && last_imu_sample.timestamp_ns > g_last_pose_imu_ts_ns) {
                    const float dt = (float)(last_imu_sample.timestamp_ns - g_last_pose_imu_ts_ns) * 1e-9f;
                    g_pose_yaw_deg += last_imu_sample.gyro[2] * dt * 57.29578f;
                    g_pose_vx_mps += (ax / g) * dt;
                    g_pose_vy_mps += (ay / g) * dt;
                    g_pose_vz_mps += ((az - 9.80665f) / g) * dt;
                    g_pose_x_m += g_pose_vx_mps * dt;
                    g_pose_y_m += g_pose_vy_mps * dt;
                    g_pose_z_m += g_pose_vz_mps * dt;
                }
                g_last_pose_imu_ts_ns = last_imu_sample.timestamp_ns;
            }

            g_6dof_quality = imu_clock_converged ? 100 : 0;

        }

        refresh_console_feed();

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        int imu_q = imu_available_samples();
        int cam_q = stereo_available_pairs();

        static bool main_window_init = false;
        if (!main_window_init) {
            const ImGuiViewport* vp = ImGui::GetMainViewport();
            const float ui_margin = 8.0f;
            ImGui::SetNextWindowPos(
                ImVec2(vp->WorkPos.x + ui_margin, vp->WorkPos.y + ui_margin),
                ImGuiCond_FirstUseEver);
            float win_w = vp->WorkSize.x - 2.0f * ui_margin;
            if (win_w < 900.0f) win_w = 900.0f;
            float win_h = vp->WorkSize.y - 2.0f * ui_margin;
            if (win_h < 700.0f) win_h = 700.0f;
            ImGui::SetNextWindowSize(ImVec2(win_w, win_h), ImGuiCond_FirstUseEver);
            main_window_init = true;
        }

        g_runtime_calib = calib;

        ImGui::Begin("XREAL IMU + Stereo Camera", nullptr,
            ImGuiWindowFlags_MenuBar);

        float total_h = ImGui::GetContentRegionAvail().y;
        float total_w = ImGui::GetContentRegionAvail().x;
        const float splitter_w = 6.0f;
        const float min_camera_w = 420.0f;
        const float min_right_w = 300.0f;

        if (!g_panel_sizes_loaded) {
            g_camera_panel_w = total_w * 0.45f;
            g_panel_sizes_loaded = true;
        }

        auto clamp_panel_sizes = [&]() {
            float max_camera_w = total_w - min_right_w - splitter_w;
            if (max_camera_w < min_camera_w) max_camera_w = min_camera_w;
            g_camera_panel_w = std::clamp(g_camera_panel_w, min_camera_w, max_camera_w);
        };
        clamp_panel_sizes();

        if (g_full_state_auto_refresh && !g_full_state_freeze) {
            refresh_full_state_snapshot(
                imu_ok,
                cam_ok,
                have_cam,
                have_imu_sample,
                last_imu_sample,
                imu_q,
                cam_q,
                (int)imu_queue.size(),
                io.Framerate,
                drained_now,
                drained_pairs_total,
                imu_samples_this_frame,
                imu_window_total,
                imu_clock_converged,
                have_cam ? &cam_pair : nullptr);
        }

        // Left column: Camera
        ImGui::BeginChild("camera_panel", ImVec2(g_camera_panel_w, 0.0f), true);
        draw_camera_block(cam_ok, have_cam, g_camTexOK);
        ImGui::EndChild();

        ImGui::SameLine(0.0f, 0.0f);
        ImGui::InvisibleButton("##splitter_camera_right", ImVec2(splitter_w, total_h));
        if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        }
        if (ImGui::IsItemActive()) {
            g_camera_panel_w += ImGui::GetIO().MouseDelta.x;
            clamp_panel_sizes();
            mark_settings_dirty();
        }

        ImGui::SameLine(0.0f, 0.0f);
        ImGui::BeginChild("right_column", ImVec2(0.0f, 0.0f), false);

        // Upper right: Device State (resizable)
        float right_h = ImGui::GetContentRegionAvail().y;
        if (g_full_panel_w <= 0.0f) g_full_panel_w = right_h * 0.55f;
        float state_h = std::clamp(g_full_panel_w, 150.0f, right_h - 150.0f);

        ImGui::BeginChild("full_state_panel", ImVec2(0.0f, state_h), true);
        ImGui::TextColored(ImVec4(0.6f, 0.95f, 1.0f, 1.0f), "XREAL Air 2 Ultra - Full Device State");
        if (set_button_and_track("Copy Full State")) {
            ImGui::SetClipboardText(g_full_state_buffer.c_str());
        }
        ImGui::SameLine();
        set_checkbox_and_track("Auto-refresh full state", &g_full_state_auto_refresh);
        ImGui::SameLine();
        set_checkbox_and_track("Freeze", &g_full_state_freeze);
        ImGui::SameLine();
        if (set_button_and_track("Refresh now")) {
            refresh_full_state_snapshot(
                imu_ok,
                cam_ok,
                have_cam,
                have_imu_sample,
                last_imu_sample,
                imu_q,
                cam_q,
                (int)imu_queue.size(),
                io.Framerate,
                drained_now,
                drained_pairs_total,
                imu_samples_this_frame,
                imu_window_total,
                imu_clock_converged,
                have_cam ? &cam_pair : nullptr);
        }
        ImGui::SameLine();
        if (set_button_and_track("Reconnect device")) {
            g_hotplug_reconnect_request = true;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(Refresh now = UI snapshot, Reconnect device = real hotplug re-init)");

        ImGui::InputTextMultiline(
            "##full_state_dump",
            const_cast<char*>(readonly_text_ptr(g_full_state_buffer)),
            g_full_state_buffer.size() + 1,
            ImVec2(-1.0f, -1.0f),
            ImGuiInputTextFlags_ReadOnly);
        ImGui::EndChild();

        // Draggable horizontal splitter between device state and console
        ImGui::InvisibleButton("##splitter_state_console", ImVec2(-1.0f, splitter_w));
        if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
        }
        if (ImGui::IsItemActive()) {
            g_full_panel_w += ImGui::GetIO().MouseDelta.y;
            mark_settings_dirty();
        }

        // Lower right: Console (remaining space)
        ImGui::BeginChild("console_panel", ImVec2(0.0f, 0.0f), true);
        ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.5f, 1.0f), "Detailed Console Feed");
        if (set_button_and_track("Copy Console")) {
            ImGui::SetClipboardText(g_console_buffer.c_str());
        }
        ImGui::SameLine();
        set_checkbox_and_track("Auto-scroll", &g_console_autoscroll);
        ImGui::SameLine();
        if (set_button_and_track("Clear View")) g_console_buffer.clear();

        ImGui::Text("Log size: %zu bytes", g_console_buffer.size());
        ImGui::InputTextMultiline(
            "##console_feed",
            const_cast<char*>(readonly_text_ptr(g_console_buffer)),
            g_console_buffer.size() + 1,
            ImVec2(-1.0f, -1.0f),
            ImGuiInputTextFlags_ReadOnly);
        if (g_console_autoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 2.0f) {
            ImGui::SetScrollHereY(1.0f);
        }
        ImGui::EndChild(); // console_panel
        ImGui::EndChild(); // right_column

        ImGui::End();

        if (g_ignore_setting_changes_this_frame) {
            g_ignore_setting_changes_this_frame = false;
        }
        maybe_save_ui_settings_throttled();

        RenderFrame();
    }

    save_ui_settings_now();

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    CleanupDeviceD3D();
    DestroyWindow(g_hwnd);
    UnregisterClassW(g_wc.lpszClassName, g_wc.hInstance);

    calibration_free(calib);
    imu_stop_streaming(); imu_shutdown();
    stereo_stop_streaming(); stereo_shutdown();
    return 0;
}
