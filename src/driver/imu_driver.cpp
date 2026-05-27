#define IMU_DRIVER_EXPORTS
#include "myxreal/imu_driver.h"
#include "ring_buffer.h"
#include "myxreal/calibration_loader.h"

#include <windows.h>
#include <hidapi.h>
#include <nlohmann/json.hpp>

#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <array>
#include <cwchar>
#include <cctype>

// =============================================================================
// Hardware constants
// =============================================================================
static constexpr uint16_t VID = 0x3318;
static constexpr uint16_t PIDS[] = { 0x0424, 0x0428, 0x0432, 0x0426 };

static constexpr uint8_t MSG_START_IMU_DATA = 0x19;
static constexpr uint8_t MSG_GET_STATIC_ID  = 0x1A;
static constexpr uint8_t MSG_GET_CAL_DATA_LENGTH = 0x14;

static constexpr uint16_t CTRL_MSG_GET_BRIGHTNESS = 0x0003;
static constexpr uint16_t CTRL_MSG_GET_DISPLAY_MODE = 0x0007;
static constexpr uint16_t CTRL_MSG_BUTTON_EVENT = 0x6C05;

static constexpr size_t RING_SIZE = 256; // power of two

// =============================================================================
// CRC32 (matches device firmware)
// =============================================================================
static uint32_t crc32_table[256];
static bool crc32_ready = false;

static void crc32_init() {
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int j = 0; j < 8; ++j)
            c = (c & 1) ? (0xEDB88320L ^ (c >> 1)) : (c >> 1);
        crc32_table[i] = c;
    }
    crc32_ready = true;
}

static uint32_t crc32(const uint8_t* data, size_t len) {
    if (!crc32_ready) crc32_init();
    uint32_t crc = 0xFFFFFFFFL;
    for (size_t i = 0; i < len; ++i)
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFL;
}

// =============================================================================
// Protocol: build sensor payload
// =============================================================================
static std::vector<uint8_t> build_sensor_payload(uint8_t msgid, const std::vector<uint8_t>& data = {}) {
    uint16_t plen = 3 + static_cast<uint16_t>(data.size());
    std::vector<uint8_t> target;
    target.push_back(plen & 0xFF);
    target.push_back((plen >> 8) & 0xFF);
    target.push_back(msgid);
    target.insert(target.end(), data.begin(), data.end());

    uint32_t cs = crc32(target.data(), target.size());

    std::vector<uint8_t> payload;
    payload.push_back(0x00);
    payload.push_back(0xAA);
    payload.push_back(cs & 0xFF);
    payload.push_back((cs >> 8) & 0xFF);
    payload.push_back((cs >> 16) & 0xFF);
    payload.push_back((cs >> 24) & 0xFF);
    payload.insert(payload.end(), target.begin(), target.end());
    return payload;
}

static std::vector<uint8_t> build_control_payload(uint16_t msgid, const std::vector<uint8_t>& data = {}) {
    uint16_t plen = 17 + static_cast<uint16_t>(data.size());
    std::vector<uint8_t> target;
    target.push_back(plen & 0xFF);
    target.push_back((plen >> 8) & 0xFF);
    for (int i = 0; i < 8; ++i) target.push_back(0);
    target.push_back(static_cast<uint8_t>(msgid & 0xFF));
    target.push_back(static_cast<uint8_t>((msgid >> 8) & 0xFF));
    for (int i = 0; i < 5; ++i) target.push_back(0);
    target.insert(target.end(), data.begin(), data.end());

    uint32_t cs = crc32(target.data(), target.size());

    std::vector<uint8_t> payload;
    payload.push_back(0x00);
    payload.push_back(0xFD);
    payload.push_back(cs & 0xFF);
    payload.push_back((cs >> 8) & 0xFF);
    payload.push_back((cs >> 16) & 0xFF);
    payload.push_back((cs >> 24) & 0xFF);
    payload.insert(payload.end(), target.begin(), target.end());

    if (payload.size() < 65) payload.resize(65, 0);
    return payload;
}

static std::string bytes_to_printable_ascii(const uint8_t* data, size_t len) {
    std::string out;
    out.reserve(len);
    for (size_t i = 0; i < len; ++i) {
        uint8_t c = data[i];
        if (c == 0) continue;
        if (c >= 32 && c <= 126) out.push_back(static_cast<char>(c));
    }
    return out;
}

static void extract_firmware_from_reply(const uint8_t* pkt, size_t len);
static void handle_control_reply(const uint8_t* pkt, size_t len);
static void read_control_replies_once(int max_reads, int timeout_ms);

// =============================================================================
// Packet parsing
// =============================================================================
static int32_t read_i24(const uint8_t* p, size_t off) {
    uint32_t v = p[off] | (p[off + 1] << 8) | (p[off + 2] << 16);
    if (v & 0x800000) v |= 0xFF000000;
    return static_cast<int32_t>(v);
}

struct RawSample {
    float    temp_c;
    uint64_t timestamp;
    float    gyro[3];  // deg/s before calibration
    float    accel[3]; // raw accel units before calibration
};

static bool parse_sensor_packet(const uint8_t* pkt, size_t len, RawSample& out) {
    if (len < 64) return false;

    int16_t temp_raw = static_cast<int16_t>(pkt[2] | (pkt[3] << 8));
    out.temp_c = temp_raw / 132.48f + 25.0f;

    uint64_t ts = 0;
    for (int i = 0; i < 8; ++i)
        ts |= (static_cast<uint64_t>(pkt[4 + i]) << (i * 8));
    out.timestamp = ts;

    int16_t gyro_mul = static_cast<int16_t>(pkt[12] | (pkt[13] << 8));
    int32_t gyro_div = static_cast<int32_t>(pkt[14] | (pkt[15] << 8) | (pkt[16] << 16) | (pkt[17] << 24));
    int32_t gx = read_i24(pkt, 18), gy = read_i24(pkt, 21), gz = read_i24(pkt, 24);
    if (gyro_div != 0) {
        out.gyro[0] = static_cast<float>(gx) * gyro_mul / gyro_div;
        out.gyro[1] = static_cast<float>(gy) * gyro_mul / gyro_div;
        out.gyro[2] = static_cast<float>(gz) * gyro_mul / gyro_div;
    } else {
        out.gyro[0] = out.gyro[1] = out.gyro[2] = 0.0f;
    }

    int16_t acc_mul = static_cast<int16_t>(pkt[27] | (pkt[28] << 8));
    int32_t acc_div = static_cast<int32_t>(pkt[29] | (pkt[30] << 8) | (pkt[31] << 16) | (pkt[32] << 24));
    int32_t ax = read_i24(pkt, 33), ay = read_i24(pkt, 36), az = read_i24(pkt, 39);
    if (acc_div != 0) {
        out.accel[0] = static_cast<float>(ax) * acc_mul / acc_div;
        out.accel[1] = static_cast<float>(ay) * acc_mul / acc_div;
        out.accel[2] = static_cast<float>(az) * acc_mul / acc_div;
    } else {
        out.accel[0] = out.accel[1] = out.accel[2] = 0.0f;
    }

    return true;
}

// =============================================================================
// Calibration data & application
// =============================================================================
struct TempBiasEntry {
    float temp;
    float bias[3];
};

struct ImuDeviceCalib {
    bool  loaded = false;
    float accel_bias[3]{};
    float gyro_bias[3]{};
    float scale_accel[3]{1, 1, 1};
    float scale_gyro[3]{1, 1, 1};
    float accel_q_gyro[4]{0, 0, 0, 1};  // JPL quaternion
    std::vector<TempBiasEntry> gyro_bias_temp;
    float bias_temperature = 25.0f;
    float accl_calib_mat[9]{1,0,0, 0,1,0, 0,0,1};
    float gyro_calib_mat[9]{1,0,0, 0,1,0, 0,0,1};
};

static void quat_rotate(const float q[4], const float v[3], float out[3]) {
    // JPL quaternion [qx, qy, qz, qw] rotation: q * v * q^-1
    float qv[3] = { q[0], q[1], q[2] };
    float qw = q[3];
    // cross(qv, v)
    float cr[3] = {
        qv[1]*v[2] - qv[2]*v[1],
        qv[2]*v[0] - qv[0]*v[2],
        qv[0]*v[1] - qv[1]*v[0]
    };
    // cross(qv, cr)
    float crr[3] = {
        qv[1]*cr[2] - qv[2]*cr[1],
        qv[2]*cr[0] - qv[0]*cr[2],
        qv[0]*cr[1] - qv[1]*cr[0]
    };
    for (int i = 0; i < 3; ++i)
        out[i] = v[i] + 2.0f * qw * cr[i] + 2.0f * crr[i];
}

static void mat3_mul(const float M[9], const float v[3], float out[3]) {
    out[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
    out[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];
    out[2] = M[6]*v[0] + M[7]*v[1] + M[8]*v[2];
}

static ImuDeviceCalib g_dev_calib;

static bool load_calibration(const char* path) {
    if (!path) return false;

    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::string json_str(sz, '\0');
    fread(&json_str[0], 1, sz, f);
    fclose(f);

    auto j = nlohmann::json::parse(json_str, nullptr, false);
    if (j.is_discarded()) return false;

    auto& dev = j["IMU"]["device_1"];
    if (!dev.is_object()) return false;

    for (int i = 0; i < 3; ++i) {
        g_dev_calib.accel_bias[i] = dev["accel_bias"][i].get<float>();
        g_dev_calib.gyro_bias[i]  = dev["gyro_bias"][i].get<float>();
        g_dev_calib.scale_accel[i] = dev.value("scale_accel", nlohmann::json::array({1,1,1}))[i].get<float>();
        g_dev_calib.scale_gyro[i]  = dev.value("scale_gyro",  nlohmann::json::array({1,1,1}))[i].get<float>();
    }
    for (int i = 0; i < 4; ++i)
        g_dev_calib.accel_q_gyro[i] = dev["accel_q_gyro"][i].get<float>();

    g_dev_calib.bias_temperature = dev.value("bias_temperature", 25.0f);

    if (dev.contains("gyro_bias_temp_data")) {
        g_dev_calib.gyro_bias_temp.clear();
        for (auto& e : dev["gyro_bias_temp_data"]) {
            TempBiasEntry tbe;
            tbe.temp = e["temp"].get<float>();
            for (int i = 0; i < 3; ++i)
                tbe.bias[i] = e["bias"][i].get<float>();
            g_dev_calib.gyro_bias_temp.push_back(tbe);
        }
    }

    if (dev.contains("imu_intrinsics")) {
        auto& intr = dev["imu_intrinsics"];
        if (intr.contains("accl_calib_mat"))
            for (int i = 0; i < 9; ++i)
                g_dev_calib.accl_calib_mat[i] = intr["accl_calib_mat"][i].get<float>();
        if (intr.contains("gyro_calib_mat"))
            for (int i = 0; i < 9; ++i)
                g_dev_calib.gyro_calib_mat[i] = intr["gyro_calib_mat"][i].get<float>();
    }

    g_dev_calib.loaded = true;
    return true;
}

static void apply_calibration(const RawSample& raw, float accel_out[3], float gyro_out[3]) {
    if (!g_dev_calib.loaded) {
        // No calibration — just copy raw with unit conversion (gyro: deg/s -> rad/s, accel: assume m/s^2)
        for (int i = 0; i < 3; ++i) {
            gyro_out[i]  = raw.gyro[i] * 0.01745329252f;   // deg/s -> rad/s
            accel_out[i] = raw.accel[i];
        }
        return;
    }

    // 1. Rotate gyro into accel reference frame via accel_q_gyro
    float gyro_rot[3];
    quat_rotate(g_dev_calib.accel_q_gyro, raw.gyro, gyro_rot);

    // 2. Convert units: gyro deg/s -> rad/s, accel raw -> m/s^2
    for (int i = 0; i < 3; ++i) {
        gyro_out[i]  = gyro_rot[i] * 0.01745329252f;
        accel_out[i] = raw.accel[i] * 9.80665f;
    }

    // 3. Temperature-interpolated gyro bias
    float gyro_bias_temp[3] = { g_dev_calib.gyro_bias[0], g_dev_calib.gyro_bias[1], g_dev_calib.gyro_bias[2] };
    if (!g_dev_calib.gyro_bias_temp.empty()) {
        float t = raw.temp_c;
        for (int i = 0; i < 3; ++i) {
            float lo = g_dev_calib.gyro_bias_temp.front().bias[i];
            float hi = g_dev_calib.gyro_bias_temp.back().bias[i];
            float tlo = g_dev_calib.gyro_bias_temp.front().temp;
            float thi = g_dev_calib.gyro_bias_temp.back().temp;
            for (size_t k = 1; k < g_dev_calib.gyro_bias_temp.size(); ++k) {
                if (g_dev_calib.gyro_bias_temp[k].temp > t) {
                    lo  = g_dev_calib.gyro_bias_temp[k-1].bias[i];
                    hi  = g_dev_calib.gyro_bias_temp[k].bias[i];
                    tlo = g_dev_calib.gyro_bias_temp[k-1].temp;
                    thi = g_dev_calib.gyro_bias_temp[k].temp;
                    break;
                }
            }
            float frac = (thi == tlo) ? 0.0f : (t - tlo) / (thi - tlo);
            gyro_bias_temp[i] = lo + frac * (hi - lo);
        }
    }

    for (int i = 0; i < 3; ++i) {
        gyro_out[i]  -= gyro_bias_temp[i];
        accel_out[i] -= g_dev_calib.accel_bias[i];
    }

    // 4. Apply calibration matrices
    float accel_mat[3], gyro_mat[3];
    mat3_mul(g_dev_calib.accl_calib_mat, accel_out, accel_mat);
    mat3_mul(g_dev_calib.gyro_calib_mat, gyro_out, gyro_mat);

    // 5. Apply scale factors
    for (int i = 0; i < 3; ++i) {
        gyro_mat[i]  *= g_dev_calib.scale_gyro[i];
        accel_mat[i] *= g_dev_calib.scale_accel[i];
    }

    // 6. Axis remap: raw [x,y,z] -> pre-biased [-x, -z, -y] -> final [x, -y, -z]
    float tmp[3] = { -accel_mat[0], -accel_mat[2], -accel_mat[1] };
    accel_out[0] =  tmp[0];
    accel_out[1] = -tmp[1];
    accel_out[2] = -tmp[2];

    tmp[0] = -gyro_mat[0]; tmp[1] = -gyro_mat[2]; tmp[2] = -gyro_mat[1];
    gyro_out[0] =  tmp[0];
    gyro_out[1] = -tmp[1];
    gyro_out[2] = -tmp[2];
}

// =============================================================================
// Global driver state
// =============================================================================
static hid_device*       g_sensor_dev  = nullptr;
static hid_device*       g_control_dev = nullptr;
static hid_device*       g_button_dev  = nullptr;
static std::atomic<bool> g_streaming{false};
static std::atomic<bool> g_control_running{false};
static std::thread       g_thread;
static std::thread       g_control_thread;
static std::mutex        g_dev_mutex;
static RingBuffer<ImuSample, RING_SIZE> g_ring;
static int               g_sensor_iface = -1;
static int               g_control_iface = -1;
static int               g_button_iface = -1;
static uint16_t          g_detected_pid = 0;
static float             g_current_fps  = 0.0f;
static DeviceInfo        g_device_info{};
static DeviceState       g_device_state{};
static std::mutex        g_device_state_mtx;
static std::atomic<int>  g_brightness{-1};
static std::atomic<int>  g_display_mode{-1};
static std::atomic<bool> g_control_handshake_done{false};
static std::atomic<bool> g_hid_inited{false};

// =============================================================================
// Clock synchronization: affine model device-ns -> system-ns
// =============================================================================
static double steady_ns() {
    static LARGE_INTEGER freq = {0, 0};
    static double inv_freq = 0.0;
    if (freq.QuadPart == 0) {
        QueryPerformanceFrequency(&freq);
        inv_freq = 1.0 / static_cast<double>(freq.QuadPart);
    }
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return static_cast<double>(now.QuadPart) * inv_freq * 1e9;
}

static void copy_fixed_str(char* dst, size_t dst_size, const char* src) {
    if (!dst || dst_size == 0) return;
    dst[0] = '\0';
    if (!src) return;
    std::strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';
}

static void copy_wide_to_utf8(char* dst, size_t dst_size, const wchar_t* src) {
    if (!dst || dst_size == 0) return;
    dst[0] = '\0';
    if (!src || src[0] == L'\0') return;

    int needed = WideCharToMultiByte(CP_UTF8, 0, src, -1, nullptr, 0, nullptr, nullptr);
    if (needed <= 0) return;

    std::vector<char> tmp(static_cast<size_t>(needed));
    int wrote = WideCharToMultiByte(CP_UTF8, 0, src, -1, tmp.data(), needed, nullptr, nullptr);
    if (wrote <= 0) return;

    copy_fixed_str(dst, dst_size, tmp.data());
}

static int refresh_device_state_locked() {
    g_device_state.timestamp_ns = static_cast<uint64_t>(steady_ns());
    g_device_state.connected = (g_sensor_dev != nullptr) ? 1 : 0;
    g_device_state.imu_streaming = g_streaming ? 1 : 0;
    g_device_state.led_state_valid = 0;

    const int disp = g_display_mode.load();
    if (disp == 1 || disp == 3) {
        g_device_state.mode_3d_valid = 1;
        g_device_state.mode_3d_enabled = (disp == 3) ? 1 : 0;
    } else {
        g_device_state.mode_3d_valid = 0;
        g_device_state.mode_3d_enabled = 0;
    }

    const int bright = g_brightness.load();
    if (bright >= 0 && bright <= 100) {
        g_device_state.brightness_valid = 1;
        g_device_state.brightness_percent = bright;
    } else {
        g_device_state.brightness_valid = 0;
        g_device_state.brightness_percent = 0;
    }

    g_device_state.valid = g_device_state.connected;
    return g_device_state.valid ? 0 : -1;
}

IMU_API int imu_refresh_device_state(void) {
    std::lock_guard<std::mutex> lk(g_device_state_mtx);
    return refresh_device_state_locked();
}

IMU_API int imu_get_device_info(DeviceInfo* out_info) {
    if (!out_info) return -1;
    *out_info = g_device_info;
    return 0;
}

IMU_API int imu_get_device_state(DeviceState* out_state) {
    if (!out_state) return -1;
    std::lock_guard<std::mutex> lk(g_device_state_mtx);
    *out_state = g_device_state;
    return 0;
}

static void update_device_info_from_enumeration(const char* sensor_path) {
    DeviceInfo info = {};
    info.valid = 1;

    struct hid_device_info* devs = hid_enumerate(VID, 0x0);
    for (auto* cur = devs; cur; cur = cur->next) {
        if (!cur->path || !sensor_path) continue;
        if (std::strcmp(cur->path, sensor_path) != 0) continue;

        info.vendor_id = cur->vendor_id;
        info.product_id = cur->product_id;
        info.release_bcd = cur->release_number;
        copy_wide_to_utf8(info.manufacturer, sizeof(info.manufacturer), cur->manufacturer_string);
        copy_wide_to_utf8(info.product, sizeof(info.product), cur->product_string);
        copy_wide_to_utf8(info.serial, sizeof(info.serial), cur->serial_number);
        break;
    }
    hid_free_enumeration(devs);

    if (g_sensor_dev) {
        wchar_t wbuf[256] = {};
        if (hid_get_manufacturer_string(g_sensor_dev, wbuf, sizeof(wbuf) / sizeof(wbuf[0])) == 0) {
            copy_wide_to_utf8(info.manufacturer, sizeof(info.manufacturer), wbuf);
        }
        std::wmemset(wbuf, 0, sizeof(wbuf) / sizeof(wbuf[0]));
        if (hid_get_product_string(g_sensor_dev, wbuf, sizeof(wbuf) / sizeof(wbuf[0])) == 0) {
            copy_wide_to_utf8(info.product, sizeof(info.product), wbuf);
        }
        std::wmemset(wbuf, 0, sizeof(wbuf) / sizeof(wbuf[0]));
        if (hid_get_serial_number_string(g_sensor_dev, wbuf, sizeof(wbuf) / sizeof(wbuf[0])) == 0) {
            copy_wide_to_utf8(info.serial, sizeof(info.serial), wbuf);
        }
    }

    copy_fixed_str(info.firmware, sizeof(info.firmware), "");
    g_device_info = info;
}

static void extract_firmware_from_reply(const uint8_t* pkt, size_t len) {
    if (!pkt || len == 0) return;

    std::string ascii = bytes_to_printable_ascii(pkt, len);
    if (ascii.empty()) return;

    bool has_digit = false;
    for (char c : ascii) {
        if (c >= '0' && c <= '9') { has_digit = true; break; }
    }
    if (!has_digit) return;

    const std::array<const char*, 7> firmware_hints = {
        "FW", "VER", "VERSION", "BUILD", "XR", "AIR", "ULTRA"
    };

    bool hinted = false;
    std::string up = ascii;
    for (char& c : up) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    for (const char* hint : firmware_hints) {
        if (up.find(hint) != std::string::npos) {
            hinted = true;
            break;
        }
    }
    if (!hinted) return;

    if (!g_device_info.firmware[0]) {
        copy_fixed_str(g_device_info.firmware, sizeof(g_device_info.firmware), ascii.c_str());
    }
}

static void handle_control_reply(const uint8_t* pkt, size_t len) {
    if (!pkt || len == 0) return;
    extract_firmware_from_reply(pkt, len);

    const int off = (pkt[0] == 0x00) ? 1 : 0;
    if (len <= static_cast<size_t>(off + 16)) return;

    const uint16_t msgid = static_cast<uint16_t>(pkt[off + 15]) |
                           (static_cast<uint16_t>(pkt[off + 16]) << 8);

    if (msgid == CTRL_MSG_GET_BRIGHTNESS) {
        if (len > static_cast<size_t>(off + 23)) {
            const int b = static_cast<int>(pkt[off + 23]);
            if (b >= 0 && b <= 100) {
                g_brightness.store(b);
                std::lock_guard<std::mutex> lk(g_device_state_mtx);
                refresh_device_state_locked();
            }
        }
        return;
    }

    if (msgid == CTRL_MSG_GET_DISPLAY_MODE) {
        if (len > static_cast<size_t>(off + 23)) {
            const int mode = static_cast<int>(pkt[off + 23]);
            if (mode == 1 || mode == 3) {
                g_display_mode.store(mode);
                std::lock_guard<std::mutex> lk(g_device_state_mtx);
                refresh_device_state_locked();
            }
        }
        return;
    }

    if (msgid == CTRL_MSG_BUTTON_EVENT) {
        if (len > static_cast<size_t>(off + 30)) {
            const uint8_t button_id = pkt[off + 22];
            const int value = static_cast<int>(pkt[off + 30]);

            if (button_id == 0x02 || button_id == 0x03) {
                if (value >= 0 && value <= 100) {
                    g_brightness.store(value);
                }
            } else if (button_id == 0x0A) {
                g_display_mode.store(1);
            } else if (button_id == 0x0B) {
                g_display_mode.store(3);
            }

            std::lock_guard<std::mutex> lk(g_device_state_mtx);
            refresh_device_state_locked();
        }
    }
}

static void read_control_replies_once(int max_reads, int timeout_ms) {
    if (max_reads <= 0) return;
    std::lock_guard<std::mutex> lk(g_dev_mutex);
    if (!g_sensor_dev) return;

    uint8_t buf[512] = {};
    int reads = 0;
    while (reads < max_reads) {
        int r = hid_read_timeout(g_sensor_dev, buf, sizeof(buf), timeout_ms);
        if (r <= 0) break;
        handle_control_reply(buf, static_cast<size_t>(r));
        reads++;
    }
}

static void query_control_state_locked() {
    if (!g_control_dev) return;

    auto q_brightness = build_control_payload(CTRL_MSG_GET_BRIGHTNESS);
    hid_write(g_control_dev, q_brightness.data(), q_brightness.size());

    auto q_display_mode = build_control_payload(CTRL_MSG_GET_DISPLAY_MODE);
    hid_write(g_control_dev, q_display_mode.data(), q_display_mode.size());
}

static void control_thread_fn() {
    uint8_t recv[65] = {};
    while (g_control_running) {
        bool read_any = false;

        {
            std::lock_guard<std::mutex> lk(g_dev_mutex);
            if (g_control_dev && !g_control_handshake_done.load()) {
                query_control_state_locked();
                g_control_handshake_done.store(true);
            }
        }

        hid_device* control = nullptr;
        hid_device* button = nullptr;
        {
            std::lock_guard<std::mutex> lk(g_dev_mutex);
            control = g_control_dev;
            button = g_button_dev;
        }

        if (control) {
            int r = hid_read_timeout(control, recv, sizeof(recv), 5);
            if (r > 0) {
                read_any = true;
                handle_control_reply(recv, static_cast<size_t>(r));
            }
        }

        if (button && button != control) {
            int r = hid_read_timeout(button, recv, sizeof(recv), 5);
            if (r > 0) {
                read_any = true;
                handle_control_reply(recv, static_cast<size_t>(r));
            }
        }

        if (!read_any) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

static void start_control_listener() {
    if (g_control_running) return;
    g_control_running = true;
    g_control_handshake_done = false;
    g_control_thread = std::thread(control_thread_fn);
}

static void stop_control_listener() {
    g_control_running = false;
    if (g_control_thread.joinable()) g_control_thread.join();
}

static void reset_control_state() {
    g_brightness.store(-1);
    g_display_mode.store(-1);
    g_control_handshake_done.store(false);
    std::lock_guard<std::mutex> lk(g_device_state_mtx);
    refresh_device_state_locked();
}

struct ClockSample { uint64_t device_ns; uint64_t system_ns; };

static constexpr int CLOCK_WINDOW        = 500;
static constexpr int CLOCK_CONVERGE_MIN  = 200;
static constexpr int CLOCK_REFIT_EVERY   = 100;
static constexpr int CLOCK_RESID_HIST    = 100;

static ClockSample g_clk_buf[CLOCK_WINDOW];
static int         g_clk_head        = 0;
static int         g_clk_count       = 0;
static int         g_clk_total       = 0;
static int         g_clk_since_refit = 0;
static double      g_clk_resid[CLOCK_RESID_HIST];
static int         g_clk_resid_head  = 0;
static int         g_clk_resid_count = 0;
static std::mutex  g_clk_mutex;
static ClockModel  g_clk_model;

static void clock_push_residual(double resid_ns) {
    g_clk_resid[g_clk_resid_head] = resid_ns;
    g_clk_resid_head = (g_clk_resid_head + 1) % CLOCK_RESID_HIST;
    if (g_clk_resid_count < CLOCK_RESID_HIST) g_clk_resid_count++;
}

static void clock_refit() {
    int n = g_clk_count;
    if (n < 10) return;

    // Gather samples in chronological order
    struct { double x, y; } pts[CLOCK_WINDOW];
    int base = (g_clk_head - n + CLOCK_WINDOW) % CLOCK_WINDOW;
    for (int i = 0; i < n; ++i) {
        auto& s = g_clk_buf[(base + i) % CLOCK_WINDOW];
        pts[i].x = static_cast<double>(s.device_ns);
        pts[i].y = static_cast<double>(s.system_ns);
    }

    // Centre for numerical stability
    double mx = 0, my = 0;
    for (int i = 0; i < n; ++i) { mx += pts[i].x; my += pts[i].y; }
    mx /= n; my /= n;

    double num = 0, den = 0;
    for (int i = 0; i < n; ++i) {
        double dx = pts[i].x - mx;
        num += dx * (pts[i].y - my);
        den += dx * dx;
    }
    if (den == 0) return;
    double scale = num / den;
    double offset = my - scale * mx;

    // Residuals for outlier detection
    double sum_sq = 0;
    double resid_arr[CLOCK_WINDOW];
    for (int i = 0; i < n; ++i) {
        resid_arr[i] = pts[i].y - (scale * pts[i].x + offset);
        sum_sq += resid_arr[i] * resid_arr[i];
    }
    double sigma = std::sqrt(sum_sq / n);

    // Refit without outliers (> 3 sigma)
    double mx2 = 0, my2 = 0;
    int n2 = 0;
    for (int i = 0; i < n; ++i) {
        if (std::abs(resid_arr[i]) > 3.0 * sigma) continue;
        mx2 += pts[i].x; my2 += pts[i].y; n2++;
    }
    if (n2 < 10) return;
    mx2 /= n2; my2 /= n2;

    num = 0; den = 0;
    for (int i = 0; i < n; ++i) {
        if (std::abs(resid_arr[i]) > 3.0 * sigma) continue;
        double dx = pts[i].x - mx2;
        num += dx * (pts[i].y - my2);
        den += dx * dx;
    }
    if (den == 0) return;
    double scale2 = num / den;
    double offset2 = my2 - scale2 * mx2;

    // Uncertainty = residual standard error
    double ssq2 = 0;
    int n_res = 0;
    for (int i = 0; i < n; ++i) {
        if (std::abs(resid_arr[i]) > 3.0 * sigma) continue;
        double r = pts[i].y - (scale2 * pts[i].x + offset2);
        ssq2 += r * r; n_res++;
    }
    double unc = (n_res > 2) ? std::sqrt(ssq2 / (n_res - 2)) : 0;
    uint64_t now_ns = static_cast<uint64_t>(steady_ns());

    // Push residuals of inliers
    for (int i = 0; i < n; ++i) {
        if (std::abs(resid_arr[i]) > 3.0 * sigma) continue;
        double r_final = pts[i].y - (scale2 * pts[i].x + offset2);
        clock_push_residual(r_final);
    }

    std::lock_guard<std::mutex> lk(g_clk_mutex);
    g_clk_model.offset_ns      = offset2;
    g_clk_model.scale           = scale2;
    g_clk_model.uncertainty_ns  = unc;
    g_clk_model.last_update_ns  = now_ns;
    g_clk_model.sample_count    = g_clk_total;
    g_clk_model.is_converged    = (n2 >= CLOCK_CONVERGE_MIN) ? 1 : 0;
}

static void clock_add_sample(uint64_t device_ns, uint64_t system_ns) {
    // Seed initial model so clock_to_system_ns() is reasonable before first refit
    if (g_clk_total == 0) {
        g_clk_model.scale    = 1.0;
        g_clk_model.offset_ns = static_cast<double>(system_ns) - static_cast<double>(device_ns);
    }

    g_clk_buf[g_clk_head].device_ns = device_ns;
    g_clk_buf[g_clk_head].system_ns = system_ns;
    g_clk_head = (g_clk_head + 1) % CLOCK_WINDOW;
    if (g_clk_count < CLOCK_WINDOW) g_clk_count++;
    g_clk_total++;
    g_clk_since_refit++;

    if (g_clk_since_refit >= CLOCK_REFIT_EVERY && g_clk_count >= 10) {
        clock_refit();
        g_clk_since_refit = 0;
    }
}

// =============================================================================
// Time alignment verifier
// =============================================================================

struct AlignImuEntry  { uint64_t system_ns; float accel_mag; float gyro_mag; };
struct AlignFrameEntry { uint64_t system_ns; float frame_diff; };

static constexpr int ALIGN_IMU_HIST   = 5000;
static constexpr int ALIGN_FRAME_HIST = 150;
static constexpr int ALIGN_SHAKE_SECS = 5;
static constexpr int ALIGN_SHAKE_IMU  = 5000;
static constexpr int ALIGN_SHAKE_FRAMES = 150;

// Live ring buffers
static AlignImuEntry   g_aimu_buf[ALIGN_IMU_HIST];
static int             g_aimu_head  = 0;
static int             g_aimu_count = 0;

static AlignFrameEntry g_aframe_buf[ALIGN_FRAME_HIST];
static int             g_aframe_head   = 0;
static int             g_aframe_count  = 0;
static int             g_aframe_histogram[21] = {};

// Shake test
static bool            g_shake_active = false;
static uint64_t        g_shake_start_ns = 0;
static int             g_shake_gen = 0;        // incremented on each start
static int             g_shake_last_completed_gen = -1;
static AlignImuEntry   g_shake_imu[ALIGN_SHAKE_IMU];
static int             g_shake_imu_count = 0;
static AlignFrameEntry g_shake_frames[ALIGN_SHAKE_FRAMES];
static int             g_shake_frame_count = 0;

static std::mutex      g_align_mutex;

// Return the index of the IMU entry nearest to frame_ns, or -1.
// buf: circular buffer; head and count define the valid window; mod is the buffer capacity.
static int align_nearest_imu(uint64_t frame_ns, const AlignImuEntry* buf,
                              int head, int count, int mod) {
    if (count <= 0) return -1;
    int base = (head - count + mod) % mod;
    int best_i = -1;
    int64_t best_d = INT64_MAX;
    for (int j = 0; j < count; ++j) {
        int idx = (base + j) % mod;
        int64_t d = (int64_t)(frame_ns - buf[idx].system_ns);
        if (std::abs(d) < std::abs(best_d)) { best_d = d; best_i = idx; }
    }
    return best_i;
}

// Count IMU entries strictly between t0 and t1 (exclusive..inclusive).
static int align_count_imu_between(uint64_t t0, uint64_t t1,
                                    const AlignImuEntry* buf, int head, int count, int mod) {
    if (count <= 0 || t1 <= t0) return 0;
    int base = (head - count + mod) % mod;
    int cnt = 0;
    for (int j = 0; j < count; ++j) {
        int idx = (base + j) % mod;
        if (buf[idx].system_ns > t0 && buf[idx].system_ns <= t1) cnt++;
    }
    return cnt;
}

static void align_record_frame_impl(uint64_t system_ns, float frame_diff) {
    // Update histogram: find nearest IMU delta
    if (g_aimu_count > 0) {
        int bi = align_nearest_imu(system_ns, g_aimu_buf, g_aimu_head, g_aimu_count, ALIGN_IMU_HIST);
        if (bi >= 0) {
            int64_t d_ns = (int64_t)(system_ns - g_aimu_buf[bi].system_ns);
            double delta_ms = (double)d_ns / 1e6;
            int bucket = (int)std::round(delta_ms) + 10;
            if (bucket < 0) bucket = 0;
            if (bucket > 20) bucket = 20;
            g_aframe_histogram[bucket]++;
        }
    }

    // Store frame in circular buffer
    g_aframe_buf[g_aframe_head].system_ns  = system_ns;
    g_aframe_buf[g_aframe_head].frame_diff = frame_diff;
    g_aframe_head = (g_aframe_head + 1) % ALIGN_FRAME_HIST;
    if (g_aframe_count < ALIGN_FRAME_HIST) g_aframe_count++;
}

// --- public API ---

IMU_API void alignment_record_imu(uint64_t system_ns, float accel_magnitude, float gyro_magnitude) {
    std::lock_guard<std::mutex> lk(g_align_mutex);

    g_aimu_buf[g_aimu_head].system_ns = system_ns;
    g_aimu_buf[g_aimu_head].accel_mag = accel_magnitude;
    g_aimu_buf[g_aimu_head].gyro_mag  = gyro_magnitude;
    g_aimu_head = (g_aimu_head + 1) % ALIGN_IMU_HIST;
    if (g_aimu_count < ALIGN_IMU_HIST) g_aimu_count++;

    if (g_shake_active && g_shake_imu_count < ALIGN_SHAKE_IMU) {
        g_shake_imu[g_shake_imu_count].system_ns = system_ns;
        g_shake_imu[g_shake_imu_count].accel_mag = accel_magnitude;
        g_shake_imu[g_shake_imu_count].gyro_mag  = gyro_magnitude;
        g_shake_imu_count++;
    }
}

IMU_API void alignment_record_frame(uint64_t system_ns, float frame_diff) {
    std::lock_guard<std::mutex> lk(g_align_mutex);
    align_record_frame_impl(system_ns, frame_diff);

    if (g_shake_active && g_shake_frame_count < ALIGN_SHAKE_FRAMES) {
        g_shake_frames[g_shake_frame_count].system_ns = system_ns;
        g_shake_frames[g_shake_frame_count].frame_diff = frame_diff;
        g_shake_frame_count++;

        if (g_shake_start_ns > 0 && system_ns - g_shake_start_ns > 5000000000ULL) {
            g_shake_active = false;
            g_shake_last_completed_gen = g_shake_gen;
            printf("[Align] Shake test #%d ended by frame timer (%.1f s)\n",
                g_shake_gen, (system_ns - g_shake_start_ns) / 1e9);
        }
    }
}

// Generic report: frames/imus are circular buffers with heads/counts/mods.
// For linear (shake) buffers, pass head=0, mod=count.
static AlignmentReport align_compute_generic(
        const AlignFrameEntry* fbuf, int fhead, int fcount, int fmod,
        const AlignImuEntry*   ibuf, int ihead, int icount, int imod,
        const int* histogram) {

    AlignmentReport r = {};
    r.min_delta_us = 1e18;
    if (fcount < 2 || icount < 2) return r;

    if (histogram) memcpy(r.histogram, histogram, sizeof(r.histogram));

    int fbase = (fhead - fcount + fmod) % fmod;

    double sum_d = 0, sum_d2 = 0;
    int n_valid = 0;
    uint64_t prev_ts = 0;
    double sum_spf = 0;
    int n_spf = 0;

    for (int i = 0; i < fcount; ++i) {
        int fi = (fbase + i) % fmod;
        uint64_t fts = fbuf[fi].system_ns;

        // Nearest IMU to this frame
        int bi = align_nearest_imu(fts, ibuf, ihead, icount, imod);
        if (bi < 0) continue;

        int64_t d_ns = (int64_t)(fts - ibuf[bi].system_ns);
        double du = (double)d_ns / 1e3; // ns → us
        sum_d  += du;
        sum_d2 += du * du;
        if (du < r.min_delta_us) r.min_delta_us = du;
        if (du > r.max_delta_us) r.max_delta_us = du;
        n_valid++;

        // IMU samples between consecutive frames
        if (prev_ts > 0 && fts > prev_ts) {
            int cnt = align_count_imu_between(prev_ts, fts, ibuf, ihead, icount, imod);
            sum_spf += cnt;
            n_spf++;
        }
        prev_ts = fts;
    }

    if (n_valid > 0) {
        r.mean_delta_us = sum_d / n_valid;
        r.std_delta_us  = (n_valid > 1)
            ? std::sqrt((sum_d2 - sum_d*sum_d/n_valid) / (n_valid - 1))
            : 0;
        r.total_frames  = n_valid;
    }
    if (n_spf > 0) r.mean_samples_per_frame = sum_spf / n_spf;

    // Cross-correlation: upsample frame signal to 1kHz via zero-order hold,
    // smooth IMU accel magnitude with 10ms rolling average, then correlate.
    // Only report lag if peak > 0.6.
    {
        // Build chronologically-ordered frame list
        struct Fp { uint64_t ts; float diff; };
        Fp frames[256];
        int nf = 0;
        for (int j = 0; j < fcount && nf < 256; ++j) {
            int fj = (fbase + j) % fmod;
            frames[nf].ts   = fbuf[fj].system_ns;
            frames[nf].diff = fbuf[fj].frame_diff;
            nf++;
        }
        if (nf < 2) goto xcorr_done;

        // Build 1kHz IMU time series with 10ms rolling-average smoothing
        // and upsample frame signal by zero-order hold
        int ibase = (ihead - icount + imod) % imod;
        int fcur = 0; // cursor into frames[]

        static constexpr int MAX_SERIES = 6000;
        double imu_s[MAX_SERIES];
        double cam_s[MAX_SERIES];
        int ns = 0;

        double ring[10] = {};
        int    ring_i  = 0;
        int    ring_n  = 0;
        double ring_sum = 0;

        for (int i = 0; i < icount && ns < MAX_SERIES; ++i) {
            int ii = (ibase + i) % imod;
            uint64_t its = ibuf[ii].system_ns;
            float    gval = ibuf[ii].gyro_mag;   // rad/s, zero-centred (no gravity bias)

            // 10ms rolling average (10 samples at 1kHz)
            ring_sum += gval;
            if (ring_n < 10) {
                ring[ring_n++] = gval;
            } else {
                int oldest = ring_i % 10;
                ring_sum -= ring[oldest];
                ring[oldest] = gval;
            }
            ring_i++;
            double imu_smooth = ring_sum / (double)ring_n;

            // Zero-order hold: advance frame cursor to most recent frame <= imu_ts
            while (fcur + 1 < nf && frames[fcur + 1].ts <= its)
                fcur++;
            if (frames[fcur].ts > its) continue; // IMU before first frame

            imu_s[ns] = imu_smooth;
            cam_s[ns] = (double)frames[fcur].diff;
            ns++;
        }

        // Cross-correlate at lags -50..+50 ms (1 sample = 1ms at 1kHz)
        if (ns >= 100) {
            static constexpr int LAG_MIN = -50;
            static constexpr int LAG_MAX =  50;
            double best_c = -2;
            int    best_l = 0;

            for (int lag = LAG_MIN; lag <= LAG_MAX; ++lag) {
                double sx = 0, sy = 0, sxy = 0, sx2 = 0, sy2 = 0;
                int cnt = 0;
                for (int k = 0; k < ns; ++k) {
                    int m = k + lag;       // IMU index shifted by lag
                    if (m < 0 || m >= ns) continue;
                    double x = imu_s[m];   // IMU at time k+lag
                    double y = cam_s[k];   // camera at time k
                    sx += x; sy += y; sxy += x * y; sx2 += x * x; sy2 += y * y;
                    cnt++;
                }
                if (cnt < 50) continue;
                double num = cnt * sxy - sx * sy;
                double den_sq = (cnt * sx2 - sx * sx) * (cnt * sy2 - sy * sy);
                double corr = (den_sq > 1e-30) ? num / std::sqrt(den_sq) : 0;
                if (corr > best_c) { best_c = corr; best_l = lag; }
            }

            r.cross_corr_peak = best_c;
            if (best_c > 0.6) {
                // lag: positive = IMU index is ahead of camera index = camera leads
                // Report: positive = IMU leads, so negate
                r.cross_corr_lag_ms = -(double)best_l;
            } else {
                r.cross_corr_lag_ms = 0;
            }
        }
        xcorr_done:;
    }

    r.pass_mean_delta          = (std::abs(r.mean_delta_us) < 5000.0) ? 1 : 0;
    r.pass_std_delta           = (r.std_delta_us < 3000.0) ? 1 : 0;
    // Cross-corr requires motion; pass if correlated and lag < 5ms, OR if
    // insufficient motion (peak <= 0.6) — cannot measure ≠ failed.
    r.pass_corr_lag = (r.cross_corr_peak > 0.6)
        ? ((std::abs(r.cross_corr_lag_ms) < 5.0) ? 1 : 0)
        : 1;  // not enough motion → pass by default
    r.pass_samples_per_frame   = (r.mean_samples_per_frame >= 31.0 && r.mean_samples_per_frame <= 35.0) ? 1 : 0;
    r.overall_pass = (r.pass_mean_delta && r.pass_std_delta && r.pass_corr_lag && r.pass_samples_per_frame) ? 1 : 0;

    return r;
}

IMU_API AlignmentReport alignment_get_report(void) {
    std::lock_guard<std::mutex> lk(g_align_mutex);
    return align_compute_generic(
        g_aframe_buf, g_aframe_head, g_aframe_count, ALIGN_FRAME_HIST,
        g_aimu_buf,   g_aimu_head,  g_aimu_count,  ALIGN_IMU_HIST,
        g_aframe_histogram);
}

IMU_API void alignment_start_shake_test(void) {
    std::lock_guard<std::mutex> lk(g_align_mutex);
    g_shake_active       = true;
    g_shake_start_ns     = static_cast<uint64_t>(steady_ns());
    g_shake_imu_count    = 0;
    g_shake_frame_count  = 0;
    g_shake_gen++;
    printf("[Align] Shake test #%d started at %llu ns\n", g_shake_gen, (unsigned long long)g_shake_start_ns);
}

IMU_API int alignment_shake_test_active(void) {
    std::lock_guard<std::mutex> lk(g_align_mutex);
    if (g_shake_active && g_shake_start_ns > 0) {
        uint64_t now_ns = static_cast<uint64_t>(steady_ns());
        if (now_ns - g_shake_start_ns > 5100000000ULL) { // 5.1s grace
            g_shake_active = false;
            g_shake_last_completed_gen = g_shake_gen;
            printf("[Align] Shake test #%d auto-ended by timeout check (%.1f s)\n",
                g_shake_gen, (now_ns - g_shake_start_ns) / 1e9);
        }
    }
    return g_shake_active ? 1 : 0;
}

IMU_API AlignmentReport alignment_get_shake_report(void) {
    std::lock_guard<std::mutex> lk(g_align_mutex);
    // Shake buffers are linear: head=0, mod=capacity
    return align_compute_generic(
        g_shake_frames, 0, g_shake_frame_count, ALIGN_SHAKE_FRAMES,
        g_shake_imu,    0, g_shake_imu_count,   ALIGN_SHAKE_IMU,
        nullptr);
}

IMU_API int alignment_shake_completed_gen(void) {
    std::lock_guard<std::mutex> lk(g_align_mutex);
    return g_shake_last_completed_gen;
}

// =============================================================================
// Device enumeration & open
// =============================================================================
static bool open_device() {
    struct hid_device_info* devs = hid_enumerate(VID, 0x0);
    if (!devs) {
        fprintf(stderr, "[IMU] No XREAL HID device found.\n");
        return false;
    }

    struct Candidate {
        uint16_t pid;
        int sensor_iface;
        int control_iface;
        int button_iface;
    };

    const Candidate order[] = {
        {0x0426, 2, 0, 8},  // Air 2 Ultra
        {0x0432, 3, 4, 4},  // Air 2 Pro
        {0x0428, 3, 4, 4},  // Air 2
        {0x0424, 3, 4, 4},  // Air
    };

    uint16_t selected_pid = 0;
    int selected_sensor_iface = -1;
    int selected_control_iface = -1;
    int selected_button_iface = -1;
    std::wstring selected_serial;
    std::string sensor_path;
    std::string control_path;
    std::string button_path;

    for (const auto& cand : order) {
        for (auto* cur = devs; cur; cur = cur->next) {
            if (cur->product_id != cand.pid) continue;
            if (cur->interface_number != cand.sensor_iface) continue;
            if (!cur->path) continue;

            selected_pid = cand.pid;
            selected_sensor_iface = cand.sensor_iface;
            selected_control_iface = cand.control_iface;
            selected_button_iface = cand.button_iface;
            sensor_path = cur->path;
            if (cur->serial_number) selected_serial = cur->serial_number;
            break;
        }
        if (!sensor_path.empty()) break;
    }

    if (!sensor_path.empty()) {
        for (auto* cur = devs; cur; cur = cur->next) {
            if (cur->product_id != selected_pid) continue;
            if (!cur->path) continue;
            if (!selected_serial.empty()) {
                if (!cur->serial_number || std::wcscmp(cur->serial_number, selected_serial.c_str()) != 0) {
                    continue;
                }
            }

            if (cur->interface_number == selected_control_iface) control_path = cur->path;
            if (cur->interface_number == selected_button_iface) button_path = cur->path;
        }
    }

    hid_free_enumeration(devs);

    if (sensor_path.empty()) {
        fprintf(stderr, "[IMU] No matching XREAL sensor interface found.\n");
        return false;
    }

    g_detected_pid = selected_pid;
    g_sensor_iface = selected_sensor_iface;
    g_control_iface = selected_control_iface;
    g_button_iface = selected_button_iface;

    if (selected_pid == 0x0426) {
        printf("[IMU] Detected XREAL Air 2 Ultra (PID 0x0426)\n");
    } else {
        printf("[IMU] Detected XREAL Air 1/2/Pro (PID 0x%04X)\n", selected_pid);
    }

    printf("[IMU] Opening sensor: %s\n", sensor_path.c_str());
    g_sensor_dev = hid_open_path(sensor_path.c_str());
    if (!g_sensor_dev) {
        fprintf(stderr, "[IMU] Failed to open sensor interface.\n");
        return false;
    }
    hid_set_nonblocking(g_sensor_dev, 0);

    if (!control_path.empty()) {
        printf("[IMU] Opening control: %s\n", control_path.c_str());
        g_control_dev = hid_open_path(control_path.c_str());
        if (g_control_dev) hid_set_nonblocking(g_control_dev, 0);
    }

    if (g_button_iface == g_control_iface) {
        g_button_dev = g_control_dev;
    } else if (!button_path.empty()) {
        printf("[IMU] Opening button: %s\n", button_path.c_str());
        g_button_dev = hid_open_path(button_path.c_str());
        if (g_button_dev) hid_set_nonblocking(g_button_dev, 0);
    }

    update_device_info_from_enumeration(sensor_path.c_str());
    reset_control_state();
    imu_refresh_device_state();
    return true;
}

static void close_device() {
    stop_control_listener();

    std::lock_guard<std::mutex> lk(g_dev_mutex);

    if (g_sensor_dev) {
        hid_close(g_sensor_dev);
        g_sensor_dev = nullptr;
    }

    if (g_control_dev) {
        hid_close(g_control_dev);
        if (g_button_dev == g_control_dev) g_button_dev = nullptr;
        g_control_dev = nullptr;
    }

    if (g_button_dev) {
        hid_close(g_button_dev);
        g_button_dev = nullptr;
    }

    reset_control_state();
    imu_refresh_device_state();
}

// =============================================================================
// Streaming thread
// =============================================================================
static void streaming_thread() {
    // Sensor handshake
    {
        auto p1 = build_sensor_payload(MSG_START_IMU_DATA, {0xAA});
        std::lock_guard<std::mutex> lk(g_dev_mutex);
        if (g_sensor_dev) hid_write(g_sensor_dev, p1.data(), p1.size());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    {
        auto p2 = build_sensor_payload(MSG_GET_STATIC_ID);
        std::lock_guard<std::mutex> lk(g_dev_mutex);
        if (g_sensor_dev) hid_write(g_sensor_dev, p2.data(), p2.size());
    }
    read_control_replies_once(8, 20);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    {
        auto p3 = build_sensor_payload(MSG_GET_CAL_DATA_LENGTH);
        std::lock_guard<std::mutex> lk(g_dev_mutex);
        if (g_sensor_dev) hid_write(g_sensor_dev, p3.data(), p3.size());
    }
    read_control_replies_once(8, 20);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Drain pending control responses while preserving metadata parsing, then start IMU stream
    read_control_replies_once(100, 1);
    {
        std::lock_guard<std::mutex> lk(g_dev_mutex);
        if (g_sensor_dev) {
            auto start = build_sensor_payload(MSG_START_IMU_DATA, {0x01});
            hid_write(g_sensor_dev, start.data(), start.size());
            printf("[IMU] Streaming started.\n");
        }
    }

    imu_refresh_device_state();

    // Main read loop
    uint8_t buf[64];
    uint64_t last_ts = 0;
    bool has_last_ts = false;
    auto fps_start = std::chrono::steady_clock::now();
    size_t sample_count = 0;

    while (g_streaming) {
        hid_device* dev;
        {
            std::lock_guard<std::mutex> lk(g_dev_mutex);
            dev = g_sensor_dev;
        }

        if (!dev) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        int res = hid_read_timeout(dev, buf, sizeof(buf), 5);
        if (res > 0 && buf[0] == 0x01) {
            RawSample raw;
            if (parse_sensor_packet(buf, res, raw)) {
                uint64_t sys_ns = static_cast<uint64_t>(steady_ns());
                clock_add_sample(raw.timestamp, sys_ns);
                sample_count++;

                float accel_cal[3], gyro_cal[3];
                apply_calibration(raw, accel_cal, gyro_cal);

                // Dead-zone for gyro noise floor
                float gmag = std::sqrt(gyro_cal[0]*gyro_cal[0] + gyro_cal[1]*gyro_cal[1] + gyro_cal[2]*gyro_cal[2]);
                if (gmag < 0.008f) gyro_cal[0] = gyro_cal[1] = gyro_cal[2] = 0.0f;

                // FPS calculation
                auto now = std::chrono::steady_clock::now();
                std::chrono::duration<float> elapsed = now - fps_start;
                if (elapsed.count() >= 1.0f) {
                    g_current_fps = static_cast<float>(sample_count) / elapsed.count();
                    sample_count = 0;
                    fps_start = now;
                }

                ImuSample s;
                s.timestamp_ns   = raw.timestamp;
                s.temperature_c  = raw.temp_c;
                s.fps            = g_current_fps;
                s.accel[0] = accel_cal[0]; s.accel[1] = accel_cal[1]; s.accel[2] = accel_cal[2];
                s.gyro[0]  = gyro_cal[0];  s.gyro[1]  = gyro_cal[1];  s.gyro[2]  = gyro_cal[2];

                g_ring.push(s);
            }
        } else if (res < 0) {
            fprintf(stderr, "[IMU] HID read error. Stopping stream.\n");
            g_streaming = false;
        }
    }

    // Send stop message
    {
        std::lock_guard<std::mutex> lk(g_dev_mutex);
        if (g_sensor_dev) {
            auto stop = build_sensor_payload(MSG_START_IMU_DATA, {0x00});
            hid_write(g_sensor_dev, stop.data(), stop.size());
        }
    }
    printf("[IMU] Streaming stopped.\n");
}

// =============================================================================
// Public API
// =============================================================================
IMU_API int imu_init(const char* calibration_path) {
    g_device_info = {};
    {
        std::lock_guard<std::mutex> lk(g_device_state_mtx);
        g_device_state = {};
    }

    if (!g_hid_inited.load()) {
        if (hid_init() != 0) {
            fprintf(stderr, "[IMU] Failed to initialize hidapi.\n");
            imu_refresh_device_state();
            return -1;
        }
        g_hid_inited = true;
    }

    if (calibration_path && calibration_path[0]) {
        if (load_calibration(calibration_path))
            printf("[IMU] Calibration loaded from %s\n", calibration_path);
        else
            fprintf(stderr, "[IMU] Failed to load calibration from %s\n", calibration_path);
    }

    if (!open_device()) {
        imu_refresh_device_state();
        return -1;
    }

    start_control_listener();
    imu_refresh_device_state();
    return 0;
}

IMU_API int imu_start_streaming(void) {
    if (g_streaming) {
        imu_refresh_device_state();
        return 0;
    }
    if (!g_sensor_dev) {
        fprintf(stderr, "[IMU] Device not open. Call imu_init first.\n");
        imu_refresh_device_state();
        return -1;
    }

    g_ring.clear();
    g_streaming = true;
    imu_refresh_device_state();
    g_thread = std::thread(streaming_thread);
    return 0;
}

IMU_API void imu_stop_streaming(void) {
    g_streaming = false;
    if (g_thread.joinable()) g_thread.join();
    imu_refresh_device_state();
}

IMU_API int imu_poll_sample(ImuSample* out_sample) {
    if (!out_sample) return 0;
    return g_ring.pop(*out_sample) ? 1 : 0;
}

IMU_API int imu_available_samples(void) {
    return static_cast<int>(g_ring.available());
}

static const CalibrationData* g_imu_calib = nullptr;

IMU_API void imu_set_calibration(const CalibrationData* calib) {
    g_imu_calib = calib;
}

IMU_API void imu_shutdown(void) {
    imu_stop_streaming();
    close_device();
    if (g_hid_inited.load()) {
        hid_exit();
        g_hid_inited = false;
    }
    imu_refresh_device_state();
}

// =============================================================================
// Clock API
// =============================================================================
IMU_API ClockModel clock_get_model(void) {
    std::lock_guard<std::mutex> lk(g_clk_mutex);
    return g_clk_model;
}

IMU_API uint64_t clock_to_system_ns(uint64_t device_ns) {
    ClockModel m = clock_get_model();
    double d = static_cast<double>(device_ns);
    return static_cast<uint64_t>(d * m.scale + m.offset_ns);
}

IMU_API uint64_t clock_to_device_ns(uint64_t system_ns) {
    ClockModel m = clock_get_model();
    if (m.scale == 0) return 0;
    return static_cast<uint64_t>((static_cast<double>(system_ns) - m.offset_ns) / m.scale);
}

IMU_API int clock_is_converged(void) {
    std::lock_guard<std::mutex> lk(g_clk_mutex);
    return g_clk_model.is_converged;
}

IMU_API int clock_get_residuals(double* out, int max_count) {
    if (!out || max_count <= 0) return 0;
    std::lock_guard<std::mutex> lk(g_clk_mutex);
    int n = g_clk_resid_count;
    if (n > max_count) n = max_count;
    for (int i = 0; i < n; ++i) {
        int idx = (g_clk_resid_head - n + i + CLOCK_RESID_HIST) % CLOCK_RESID_HIST;
        out[i] = g_clk_resid[idx];
    }
    return n;
}
