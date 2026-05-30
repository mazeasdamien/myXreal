#include "xreal_air_driver/xreal_air_driver.h"
#include "xreal_air_driver/calibration.h"
#include "xreal_air_driver/sensor_fusion.h"
#include "xreal_air_driver/imu_navigator.h"
#include <hidapi.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <vector>
#include <iostream>
#include <cmath>

#if defined(__ANDROID__) || defined(__linux__)
#include <cstdint>
extern "C" {
    struct hid_device_;
    typedef struct hid_device_ hid_device;
    hid_device * hid_libusb_wrap_sys_device(intptr_t sys_dev, int interface_num);
}
#endif

namespace xreal {

constexpr uint16_t VID = 0x3318;

constexpr uint8_t MSG_START_IMU_DATA = 0x19;
constexpr uint8_t MSG_GET_STATIC_ID = 0x1A;
constexpr uint8_t MSG_GET_CAL_DATA_LENGTH = 0x14;
constexpr uint8_t MSG_CAL_DATA_GET_NEXT_SEGMENT = 0x15;

struct RawSample {
    float temp_c;
    uint64_t timestamp;
    std::array<float, 3> gyro;
    std::array<float, 3> accel;
};

static uint32_t calculate_crc32(const uint8_t* data, size_t len) {
    static uint32_t table[256];
    static bool table_initialized = false;
    if (!table_initialized) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int j = 0; j < 8; ++j) {
                if (c & 1) {
                    c = 0xEDB88320L ^ (c >> 1);
                } else {
                    c = c >> 1;
                }
            }
            table[i] = c;
        }
        table_initialized = true;
    }

    uint32_t crc = 0xFFFFFFFFL;
    for (size_t i = 0; i < len; ++i) {
        crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFL;
}

static std::vector<uint8_t> build_sensor_payload(uint8_t msgid, const std::vector<uint8_t>& data = {}) {
    uint16_t packet_len = 3 + static_cast<uint16_t>(data.size());
    std::vector<uint8_t> target;
    target.push_back(packet_len & 0xFF);
    target.push_back((packet_len >> 8) & 0xFF);
    target.push_back(msgid);
    target.insert(target.end(), data.begin(), data.end());

    uint32_t checksum = calculate_crc32(target.data(), target.size());

    std::vector<uint8_t> payload;
    payload.push_back(0x00);
    payload.push_back(0xAA);
    payload.push_back(checksum & 0xFF);
    payload.push_back((checksum >> 8) & 0xFF);
    payload.push_back((checksum >> 16) & 0xFF);
    payload.push_back((checksum >> 24) & 0xFF);
    payload.insert(payload.end(), target.begin(), target.end());

    return payload;
}

static std::vector<uint8_t> build_control_payload(uint16_t msgid, const std::vector<uint8_t>& data = {}) {
    uint16_t packet_len = 17 + static_cast<uint16_t>(data.size());
    std::vector<uint8_t> target;
    target.push_back(packet_len & 0xFF);
    target.push_back((packet_len >> 8) & 0xFF);
    for (int i = 0; i < 8; ++i) target.push_back(0);
    target.push_back(msgid & 0xFF);
    target.push_back((msgid >> 8) & 0xFF);
    for (int i = 0; i < 5; ++i) target.push_back(0);
    target.insert(target.end(), data.begin(), data.end());

    uint32_t checksum = calculate_crc32(target.data(), target.size());

    std::vector<uint8_t> payload;
    payload.push_back(0x00);
    payload.push_back(0xFD);
    payload.push_back(checksum & 0xFF);
    payload.push_back((checksum >> 8) & 0xFF);
    payload.push_back((checksum >> 16) & 0xFF);
    payload.push_back((checksum >> 24) & 0xFF);
    payload.insert(payload.end(), target.begin(), target.end());

    if (payload.size() < 65) {
        payload.resize(65, 0);
    }
    return payload;
}

static int32_t read_i24(const uint8_t* packet, size_t offset) {
    uint32_t val = packet[offset] | (packet[offset+1] << 8) | (packet[offset+2] << 16);
    if (val & 0x800000) {
        val |= 0xFF000000;
    }
    return static_cast<int32_t>(val);
}

static bool parse_sensor_packet(const uint8_t* packet, size_t len, RawSample& out_sample) {
    if (len < 64) return false;

    int16_t temp_raw = static_cast<int16_t>(packet[2] | (packet[3] << 8));
    out_sample.temp_c = temp_raw / 132.48f + 25.0f;

    uint64_t ts = 0;
    for (int i = 0; i < 8; ++i) {
        ts |= (static_cast<uint64_t>(packet[4 + i]) << (i * 8));
    }
    out_sample.timestamp = ts;

    int16_t gyro_mul = static_cast<int16_t>(packet[12] | (packet[13] << 8));
    int32_t gyro_div = static_cast<int32_t>(packet[14] | (packet[15] << 8) | (packet[16] << 16) | (packet[17] << 24));

    int32_t gyro_x = read_i24(packet, 18);
    int32_t gyro_y = read_i24(packet, 21);
    int32_t gyro_z = read_i24(packet, 24);

    if (gyro_div == 0) return false;
    out_sample.gyro[0] = static_cast<float>(gyro_x) * gyro_mul / gyro_div;
    out_sample.gyro[1] = static_cast<float>(gyro_y) * gyro_mul / gyro_div;
    out_sample.gyro[2] = static_cast<float>(gyro_z) * gyro_mul / gyro_div;

    int16_t accel_mul = static_cast<int16_t>(packet[27] | (packet[28] << 8));
    int32_t accel_div = static_cast<int32_t>(packet[29] | (packet[30] << 8) | (packet[31] << 16) | (packet[32] << 24));

    int32_t accel_x = read_i24(packet, 33);
    int32_t accel_y = read_i24(packet, 36);
    int32_t accel_z = read_i24(packet, 39);

    if (accel_div == 0) return false;
    out_sample.accel[0] = static_cast<float>(accel_x) * accel_mul / accel_div;
    out_sample.accel[1] = static_cast<float>(accel_y) * accel_mul / accel_div;
    out_sample.accel[2] = static_cast<float>(accel_z) * accel_mul / accel_div;

    return true;
}

class XRealAirDriver::Impl {
public:
    Impl() : streaming_(false), device_(nullptr), device_control_(nullptr), device_button_(nullptr),
             control_listening_(false), brightness_(-1), 
             display_mode_(-1), is_calibration_from_device_(false), detected_pid_(0), 
             sensor_interface_(-1), control_interface_(-1), button_interface_(-1), disconnected_(false),
             needs_sensor_handshake_(true), needs_control_handshake_(true) {}

    ~Impl() {
        stop_streaming();
        stop_control_thread();
        close_devices();
        hid_exit();
    }

    bool initialize(const std::string& calibration_json_path) {
        if (hid_init() < 0) {
            std::cerr << "[Driver] Failed to initialize hidapi." << std::endl;
            return false;
        }

        bool dev_opened = open_devices();
        bool cal_loaded = false;

        if (dev_opened) {
            std::string downloaded_json;
            if (download_calibration(downloaded_json)) {
                if (calibration_.load_from_json_string(downloaded_json)) {
                    cal_loaded = true;
                    is_calibration_from_device_ = true;
                }
            }
        }

        if (!cal_loaded && !calibration_json_path.empty()) {
            std::cout << "[Driver] Falling back to local file calibration: " << calibration_json_path << std::endl;
            if (calibration_.load_from_json(calibration_json_path)) {
                cal_loaded = true;
                is_calibration_from_device_ = false;
            }
        }

        if (!cal_loaded) {
            std::cerr << "[Driver] Warning: Calibration initialization failed. Using identity parameters." << std::endl;
        }

        // Always spin up the control thread to support hotplug detection/reconnection
        control_listening_ = true;
        control_thread_ = std::thread(&Impl::run_control, this);

        return true;
    }

    bool initialize_with_fd(int fd, const std::string& calibration_json_path) {
#if defined(__ANDROID__) || defined(__linux__)
        if (!calibration_.load_from_json(calibration_json_path)) {
            std::cerr << "[Driver] Warning: Calibration initialization failed. Using identity parameters." << std::endl;
        }

        if (hid_init() < 0) {
            std::cerr << "[Driver] Failed to initialize hidapi." << std::endl;
            return false;
        }

        sensor_interface_ = 2; 
        std::lock_guard<std::mutex> lock(device_mutex_);
        device_ = hid_libusb_wrap_sys_device(static_cast<intptr_t>(fd), sensor_interface_);
        if (!device_) {
            std::cerr << "[Driver] Failed to wrap USB file descriptor: " << fd << std::endl;
            disconnected_ = true;
            return false;
        }

        hid_set_nonblocking(device_, 0);
        disconnected_ = false;
        return true;
#else
        std::cerr << "[Driver] initialize_with_fd is not supported on this platform." << std::endl;
        return false;
#endif
    }

    bool start_streaming(const std::string& filter_type, TelemetryCallback callback) {
        if (streaming_) {
            return true;
        }

        if (!device_) {
            open_devices();
        }

        navigator_.configure(calibration_);
        mahony_filter_ = MahonyFilter(0.5f, 0.0f);
        mahony_aligned_ = false;

        callback_ = callback;
        streaming_ = true;
        thread_ = std::thread(&Impl::run, this);

        return true;
    }

    void stop_streaming() {
        if (!streaming_) return;

        streaming_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }

        std::lock_guard<std::mutex> lock(device_mutex_);
        if (device_) {
            auto stop_payload = build_sensor_payload(MSG_START_IMU_DATA, {0x00});
            hid_write(device_, stop_payload.data(), stop_payload.size());
        }
    }

    bool is_streaming() const {
        return streaming_.load();
    }

    int get_brightness() const {
        return brightness_.load();
    }

    bool set_brightness(int val) {
        if (val < 0 || val > 8) return false;
        
        std::lock_guard<std::mutex> lock(device_mutex_);
        if (!device_control_) return false;

        std::cout << "[Driver] Programmatically setting brightness to " << val << std::endl;
        auto payload = build_control_payload(0x0004, { static_cast<uint8_t>(val) });
        int written = hid_write(device_control_, payload.data(), payload.size());
        if (written >= 0) {
            brightness_ = val;
            return true;
        }
        return false;
    }

    int get_display_mode() const {
        return display_mode_.load();
    }

    bool set_display_mode(int mode) {
        if (mode != 1 && mode != 3) return false;
        
        std::lock_guard<std::mutex> lock(device_mutex_);
        if (!device_control_) return false;

        std::cout << "[Driver] Programmatically setting display mode to " << (mode == 3 ? "3D" : "2D") << std::endl;
        auto payload = build_control_payload(0x0008, { static_cast<uint8_t>(mode) });
        int written = hid_write(device_control_, payload.data(), payload.size());
        if (written >= 0) {
            display_mode_ = mode;
            return true;
        }
        return false;
    }

    bool is_calibration_from_device() const {
        return is_calibration_from_device_.load();
    }

private:
    bool open_devices() {
        std::lock_guard<std::mutex> lock(device_mutex_);
        return open_devices_under_lock();
    }

    bool open_devices_under_lock() {
        close_devices_under_lock();

        const uint16_t pids[] = { 0x0424, 0x0428, 0x0432, 0x0426 };
        struct hid_device_info* devs = nullptr;
        uint16_t found_pid = 0;

        std::cout << "[Driver] Enumerating HID devices for XREAL glasses..." << std::endl;
        for (uint16_t pid : pids) {
            devs = hid_enumerate(VID, pid);
            if (devs) {
                found_pid = pid;
                break;
            }
        }

        if (!devs) {
            std::cout << "[Driver] No matching XREAL USB device found." << std::endl;
            disconnected_ = true;
            return false;
        }

        detected_pid_ = found_pid;
        if (found_pid == 0x0426) { // Air 2 Ultra
            sensor_interface_ = 2;
            control_interface_ = 0;
            button_interface_ = 8;
            std::cout << "[Driver] Detected XREAL Air 2 Ultra (PID 0x0426)" << std::endl;
        } else {
            sensor_interface_ = 3;
            control_interface_ = 4;
            button_interface_ = 4;
            std::cout << "[Driver] Detected XREAL Air 1 / 2 / Pro (PID 0x" << std::hex << found_pid << std::dec << ")" << std::endl;
        }

        std::string sensor_path;
        std::string control_path;
        std::string button_path;
        struct hid_device_info* cur = devs;
        while (cur) {
            std::cout << "[Driver] Found Interface: " << cur->interface_number << " | Path: " << (cur->path ? cur->path : "NULL") << std::endl;
            if (cur->interface_number == sensor_interface_) {
                sensor_path = cur->path;
            } 
            if (cur->interface_number == control_interface_) {
                control_path = cur->path;
            }
            if (cur->interface_number == button_interface_) {
                button_path = cur->path;
            }
            cur = cur->next;
        }
        hid_free_enumeration(devs);

        if (sensor_path.empty()) {
            std::cerr << "[Driver] Error: Could not find HMD sensor interface (interface " << sensor_interface_ << ")." << std::endl;
            disconnected_ = true;
            return false;
        }

        std::cout << "[Driver] Opening sensor HID path: " << sensor_path << std::endl;
        device_ = hid_open_path(sensor_path.c_str());
        if (!device_) {
            std::cerr << "[Driver] Error: Failed to open sensor interface." << std::endl;
            disconnected_ = true;
            return false;
        }
        hid_set_nonblocking(device_, 0);

        if (!control_path.empty()) {
            std::cout << "[Driver] Opening control HID path: " << control_path << std::endl;
            device_control_ = hid_open_path(control_path.c_str());
            if (!device_control_) {
                std::cerr << "[Driver] Warning: Failed to open control interface." << std::endl;
            } else {
                hid_set_nonblocking(device_control_, 0);
            }
        } else {
            std::cerr << "[Driver] Warning: Could not find HMD control interface (interface " << control_interface_ << ")." << std::endl;
        }

        if (control_interface_ == button_interface_) {
            device_button_ = device_control_;
        } else {
            if (!button_path.empty()) {
                std::cout << "[Driver] Opening button HID path: " << button_path << std::endl;
                device_button_ = hid_open_path(button_path.c_str());
                if (!device_button_) {
                    std::cerr << "[Driver] Warning: Failed to open button interface." << std::endl;
                } else {
                    hid_set_nonblocking(device_button_, 0);
                }
            } else {
                std::cerr << "[Driver] Warning: Could not find HMD button interface (interface " << button_interface_ << ")." << std::endl;
            }
        }

        disconnected_ = false;
        return true;
    }

    void close_devices() {
        std::lock_guard<std::mutex> lock(device_mutex_);
        close_devices_under_lock();
    }

    void close_devices_under_lock() {
        if (device_) {
            hid_close(device_);
            device_ = nullptr;
        }
        if (device_control_) {
            hid_close(device_control_);
            if (device_button_ == device_control_) {
                device_button_ = nullptr;
            }
            device_control_ = nullptr;
        }
        if (device_button_) {
            hid_close(device_button_);
            device_button_ = nullptr;
        }
    }

    void stop_control_thread() {
        control_listening_ = false;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
    }

    void handle_disconnect() {
        std::lock_guard<std::mutex> lock(device_mutex_);
        if (disconnected_) return;
        disconnected_ = true;
        needs_sensor_handshake_ = true;
        needs_control_handshake_ = true;

        std::cout << "[Driver] Disconnection detected. Closing device handles." << std::endl;
        close_devices_under_lock();

        // Reset display mode and brightness to unknown during disconnect
        brightness_ = -1;
        display_mode_ = -1;
    }

    bool check_and_reconnect() {
        std::lock_guard<std::mutex> lock(device_mutex_);
        if (!disconnected_) return true; // Already connected or reconnected by another thread

        if (open_devices_under_lock()) {
            std::cout << "[Driver] Reconnection successful!" << std::endl;
            disconnected_ = false;
            return true;
        }
        return false;
    }

    bool download_calibration(std::string& out_json) {
        hid_device* active_device = nullptr;
        {
            std::lock_guard<std::mutex> lock(device_mutex_);
            active_device = device_;
        }
        if (!active_device) return false;

        std::cout << "[Driver] Querying factory calibration directly from device..." << std::endl;

        // Explicitly stop IMU streaming first to ensure we don't get flooded with IMU packets
        auto p_stop = build_sensor_payload(MSG_START_IMU_DATA, {0x00});
        {
            std::lock_guard<std::mutex> lock(device_mutex_);
            if (device_) hid_write(device_, p_stop.data(), p_stop.size());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Flush all pending packets
        uint8_t flush_buf[512];
        int flush_count = 0;
        while (true) {
            int read_res = 0;
            {
                std::lock_guard<std::mutex> lock(device_mutex_);
                if (device_) {
                    read_res = hid_read_timeout(device_, flush_buf, sizeof(flush_buf), 10);
                } else {
                    break;
                }
            }
            if (read_res <= 0 || flush_count >= 500) break;
            flush_count++;
        }

        // Perform sensor handshake
        auto p1 = build_sensor_payload(MSG_START_IMU_DATA, {0xAA});
        {
            std::lock_guard<std::mutex> lock(device_mutex_);
            if (device_) hid_write(device_, p1.data(), p1.size());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto p2 = build_sensor_payload(MSG_GET_STATIC_ID);
        {
            std::lock_guard<std::mutex> lock(device_mutex_);
            if (device_) hid_write(device_, p2.data(), p2.size());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        flush_count = 0;
        while (true) {
            int read_res = 0;
            {
                std::lock_guard<std::mutex> lock(device_mutex_);
                if (device_) {
                    read_res = hid_read_timeout(device_, flush_buf, sizeof(flush_buf), 10);
                } else {
                    break;
                }
            }
            if (read_res <= 0 || flush_count >= 100) break;
            flush_count++;
        }

        // Query length of calibration JSON
        auto p_len = build_sensor_payload(MSG_GET_CAL_DATA_LENGTH, {0x00});
        int written = -1;
        {
            std::lock_guard<std::mutex> lock(device_mutex_);
            if (device_) written = hid_write(device_, p_len.data(), p_len.size());
        }
        if (written < 0) {
            std::cerr << "[Driver] Failed to request calibration data length." << std::endl;
            return false;
        }

        uint8_t read_buf[1024];
        int read_bytes = 0;
        {
            std::lock_guard<std::mutex> lock(device_mutex_);
            if (device_) read_bytes = hid_read_timeout(device_, read_buf, sizeof(read_buf), 1000);
        }
        if (read_bytes <= 0) {
            std::cerr << "[Driver] Timeout waiting for calibration data length response." << std::endl;
            return false;
        }

        int offset_header = 0;
        if (read_buf[0] == 0x00 && read_buf[1] == 0xAA) {
            offset_header = 1;
        } else if (read_buf[0] == 0xAA) {
            offset_header = 0;
        } else {
            std::cerr << "[Driver] Invalid calibration length response magic: " << (int)read_buf[0] << std::endl;
            return false;
        }

        if (read_bytes < offset_header + 12) {
            std::cerr << "[Driver] Calibration length packet too short: " << read_bytes << " bytes." << std::endl;
            return false;
        }

        uint32_t detected_len = read_buf[offset_header + 8] | 
                                (read_buf[offset_header + 9] << 8) | 
                                (read_buf[offset_header + 10] << 16) | 
                                (read_buf[offset_header + 11] << 24);

        if (detected_len < 10000 || detected_len > 1000000) {
            std::cerr << "[Driver] Calibration length out of bounds: " << detected_len << " bytes." << std::endl;
            return false;
        }

        std::cout << "[Driver] Calibration downloading started. File length: " << detected_len << " bytes." << std::endl;

        std::string full_calibration_data;
        full_calibration_data.reserve(detected_len);

        uint32_t file_offset = 0;
        while (file_offset < detected_len) {
            std::vector<uint8_t> data_payload = { 0x00 };
            data_payload.push_back((file_offset >> 24) & 0xFF);
            data_payload.push_back((file_offset >> 16) & 0xFF);
            data_payload.push_back((file_offset >> 8) & 0xFF);
            data_payload.push_back(file_offset & 0xFF);

            auto p15 = build_sensor_payload(MSG_CAL_DATA_GET_NEXT_SEGMENT, data_payload);
            written = -1;
            {
                std::lock_guard<std::mutex> lock(device_mutex_);
                if (device_) written = hid_write(device_, p15.data(), p15.size());
            }
            if (written < 0) {
                std::cerr << "[Driver] Failed to request calibration segment at offset " << file_offset << std::endl;
                return false;
            }

            {
                std::lock_guard<std::mutex> lock(device_mutex_);
                if (device_) {
                    read_bytes = hid_read_timeout(device_, read_buf, sizeof(read_buf), 1000);
                } else {
                    read_bytes = 0;
                }
            }
            if (read_bytes <= 0) {
                std::cerr << "[Driver] Timeout reading calibration segment at offset " << file_offset << std::endl;
                return false;
            }

            int segment_offset = 0;
            if (read_buf[0] == 0x00 && read_buf[1] == 0xAA) {
                segment_offset = 1;
            } else if (read_buf[0] == 0xAA) {
                segment_offset = 0;
            } else {
                std::cerr << "[Driver] Invalid segment magic: " << (int)read_buf[0] << std::endl;
                return false;
            }

            if (read_bytes < segment_offset + 8) {
                std::cerr << "[Driver] Segment response packet too short: " << read_bytes << " bytes." << std::endl;
                return false;
            }

            uint16_t target_len = read_buf[segment_offset + 5] | (read_buf[segment_offset + 6] << 8);
            if (target_len <= 1) {
                std::cerr << "[Driver] Invalid target length in segment response: " << target_len << std::endl;
                return false;
            }
            int payload_len = target_len - 1;
            int segment_bytes = std::min<int>(payload_len, read_bytes - (segment_offset + 8));
            if (segment_bytes <= 0) {
                std::cerr << "[Driver] Invalid segment payload bytes count: " << segment_bytes << std::endl;
                return false;
            }

            uint32_t copy_bytes = std::min<uint32_t>(static_cast<uint32_t>(segment_bytes), detected_len - file_offset);
            full_calibration_data.append(reinterpret_cast<char*>(&read_buf[segment_offset + 8]), copy_bytes);
            file_offset += copy_bytes;
        }

        size_t offset = full_calibration_data.find("{\"FSN\"");
        if (offset == std::string::npos) {
            offset = full_calibration_data.find("{\"IMU\"");
        }

        std::string rotated;
        if (offset != std::string::npos) {
            std::cout << "[Driver] Rotating calibration buffer at offset: " << offset << std::endl;
            rotated = full_calibration_data.substr(offset) + full_calibration_data.substr(0, offset);
        } else {
            size_t first_brace = full_calibration_data.find('{');
            if (first_brace != std::string::npos) {
                rotated = full_calibration_data.substr(first_brace);
            } else {
                std::cerr << "[Driver] Failed to find start of JSON in calibration data." << std::endl;
                return false;
            }
        }

        int nest = 0;
        size_t end_pos = std::string::npos;
        for (size_t i = 0; i < rotated.size(); ++i) {
            if (rotated[i] == '{') {
                nest++;
            } else if (rotated[i] == '}') {
                nest--;
                if (nest == 0) {
                    end_pos = i + 1;
                    break;
                }
            }
        }

        if (end_pos == std::string::npos) {
            std::cerr << "[Driver] Failed to find matching closing brace in rotated JSON." << std::endl;
            return false;
        }

        out_json = rotated.substr(0, end_pos);
        std::cout << "[Driver] Successfully extracted JSON calibration data of size " << out_json.size() << " bytes." << std::endl;
        return true;
    }

    void run_control() {
        std::cout << "[Driver Control Debug] run_control thread started." << std::endl;
        needs_control_handshake_ = true;

        uint8_t recv[65];
        while (control_listening_) {
            hid_device* active_control = nullptr;
            hid_device* active_button = nullptr;
            {
                std::lock_guard<std::mutex> lock(device_mutex_);
                active_control = device_control_;
                active_button = device_button_;
            }

            if (!active_control) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                check_and_reconnect();
                continue;
            }

            if (needs_control_handshake_) {
                std::lock_guard<std::mutex> lock(device_mutex_);
                if (device_control_) {
                    std::cout << "[Driver Control Debug] Performing control handshake..." << std::endl;
                    auto init_cmd = build_control_payload(0x0003);
                    int w1 = hid_write(device_control_, init_cmd.data(), init_cmd.size());
                    std::cout << "[Driver Control Debug] init_cmd write return: " << w1 << std::endl;
                    if (w1 < 0) {
                        std::wcout << L"[Driver Control Debug] init_cmd write error: " << hid_error(device_control_) << std::endl;
                    }

                    auto query_disp = build_control_payload(0x0007);
                    int w2 = hid_write(device_control_, query_disp.data(), query_disp.size());
                    std::cout << "[Driver Control Debug] query_disp write return: " << w2 << std::endl;
                    if (w2 < 0) {
                        std::wcout << L"[Driver Control Debug] query_disp write error: " << hid_error(device_control_) << std::endl;
                    }
                    
                    needs_control_handshake_ = false;
                }
                continue;
            }

            bool read_any = false;
            int res = hid_read_timeout(active_control, recv, sizeof(recv), 5);
            if (res > 0) {
                read_any = true;
                int offset = (recv[0] == 0x00) ? 1 : 0;
                if (res > offset + 16) {
                    uint16_t msgid = recv[offset + 15] | (recv[offset + 16] << 8);
                    
                    // Handle response to command writes / status queries
                    if (msgid == 0x0003) { 
                        if (res > offset + 23) {
                            brightness_ = recv[offset + 23];
                            std::cout << "[Driver] Brightness value received: " << (int)brightness_ << std::endl;
                        }
                    } else if (msgid == 0x0007) { 
                        if (res > offset + 23) {
                            display_mode_ = recv[offset + 23];
                            std::cout << "[Driver] Display mode received: " << (int)display_mode_ << std::endl;
                        }
                    } else if (msgid == 0x6C05) { 
                        if (res > offset + 30) {
                            uint8_t button_id = recv[offset + 22];
                            uint8_t value = recv[offset + 30];
                            if (button_id == 0x02 || button_id == 0x03) {
                                brightness_ = value;
                                std::cout << "[Driver] Physical brightness button pressed: " << (int)value << std::endl;
                            } else if (button_id == 0x0A) {
                                display_mode_ = 1; 
                                std::cout << "[Driver] Physical button changed mode: 2D" << std::endl;
                            } else if (button_id == 0x0B) {
                                display_mode_ = 3; 
                                std::cout << "[Driver] Physical button changed mode: 3D" << std::endl;
                            }
                        }
                    }
                }
            } else if (res < 0) {
                std::cerr << "[Driver Control Debug] Error reading from control interface. Triggering disconnect." << std::endl;
                handle_disconnect();
                continue;
            }

            if (active_button && active_button != active_control) {
                int res_btn = hid_read_timeout(active_button, recv, sizeof(recv), 5);
                if (res_btn > 0) {
                    read_any = true;
                    int offset = (recv[0] == 0x00) ? 1 : 0;
                    if (res_btn > offset + 16) {
                        uint16_t msgid = recv[offset + 15] | (recv[offset + 16] << 8);
                        if (msgid == 0x6C05) { 
                            if (res_btn > offset + 30) {
                                uint8_t button_id = recv[offset + 22];
                                uint8_t value = recv[offset + 30];
                                if (button_id == 0x02 || button_id == 0x03) {
                                    brightness_ = value;
                                    std::cout << "[Driver] Physical brightness button pressed: " << (int)value << std::endl;
                                } else if (button_id == 0x0A) {
                                    display_mode_ = 1; 
                                    std::cout << "[Driver] Physical button changed mode: 2D" << std::endl;
                                } else if (button_id == 0x0B) {
                                    display_mode_ = 3; 
                                    std::cout << "[Driver] Physical button changed mode: 3D" << std::endl;
                                }
                            }
                        }
                    }
                } else if (res_btn < 0) {
                    std::cerr << "[Driver Control Debug] Error reading from button interface. Triggering disconnect." << std::endl;
                    handle_disconnect();
                    continue;
                }
            }

            if (!read_any) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        std::cout << "[Driver Control Debug] run_control thread exiting." << std::endl;
    }

    void run() {
        uint64_t last_timestamp = 0;
        bool has_last_timestamp = false;

        auto fps_start = std::chrono::steady_clock::now();
        size_t sample_count = 0;
        float current_fps = 0.0f;

        uint8_t read_buf[64];
        while (streaming_) {
            hid_device* active_device = nullptr;
            {
                std::lock_guard<std::mutex> lock(device_mutex_);
                active_device = device_;
            }

            if (!active_device) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                check_and_reconnect();
                continue;
            }

            if (needs_sensor_handshake_) {
                std::cout << "[Driver] Executing sensor handshake query sequence..." << std::endl;

                auto p1 = build_sensor_payload(MSG_START_IMU_DATA, {0xAA});
                {
                    std::lock_guard<std::mutex> lock(device_mutex_);
                    if (device_) hid_write(device_, p1.data(), p1.size());
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                auto p2 = build_sensor_payload(MSG_GET_STATIC_ID);
                {
                    std::lock_guard<std::mutex> lock(device_mutex_);
                    if (device_) hid_write(device_, p2.data(), p2.size());
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                auto p3 = build_sensor_payload(MSG_GET_CAL_DATA_LENGTH);
                {
                    std::lock_guard<std::mutex> lock(device_mutex_);
                    if (device_) hid_write(device_, p3.data(), p3.size());
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                {
                    std::lock_guard<std::mutex> lock(device_mutex_);
                    if (device_) {
                        uint8_t flush_buf[512];
                        int flush_count = 0;
                        while (hid_read_timeout(device_, flush_buf, sizeof(flush_buf), 1) > 0 && flush_count < 100) {
                            flush_count++;
                        }

                        auto start_payload = build_sensor_payload(MSG_START_IMU_DATA, {0x01});
                        hid_write(device_, start_payload.data(), start_payload.size());
                        std::cout << "[Driver] Telemetry stream active." << std::endl;
                        needs_sensor_handshake_ = false;
                        has_last_timestamp = false;
                    }
                }
                continue;
            }

            int res = hid_read_timeout(active_device, read_buf, sizeof(read_buf), 5);
            if (res > 0) {
                if (read_buf[0] == 0x01) {
                    RawSample raw;
                    if (parse_sensor_packet(read_buf, res, raw)) {
                        sample_count++;

                        std::array<float, 3> cal_accel;
                        std::array<float, 3> cal_gyro;
                        calibration_.apply(raw.accel, raw.gyro, raw.temp_c, cal_accel, cal_gyro);

                        const float GYRO_DEADZONE = 0.008f;
                        float gyro_mag = std::sqrt(cal_gyro[0] * cal_gyro[0] + cal_gyro[1] * cal_gyro[1] + cal_gyro[2] * cal_gyro[2]);
                        if (gyro_mag < GYRO_DEADZONE) {
                            cal_gyro[0] = 0.0f;
                            cal_gyro[1] = 0.0f;
                            cal_gyro[2] = 0.0f;
                        }

                        float dt = 0.001f;
                        if (has_last_timestamp) {
                            dt = static_cast<float>(raw.timestamp - last_timestamp) / 1e9f;
                            if (dt <= 0.0f || dt > 0.1f) {
                                dt = 0.001f;
                            }
                        }
                        last_timestamp = raw.timestamp;
                        has_last_timestamp = true;

                        navigator_.update(dt, cal_accel, cal_gyro);

                        // Sync Mahony quaternion to navigator after stationary init
                        // so both share the same world-frame convention
                        if (navigator_.is_initialized() && !mahony_aligned_) {
                            auto q_init = navigator_.get_quaternion();
                            mahony_filter_.reset(q_init);
                            mahony_aligned_ = true;
                        }

                        mahony_filter_.update(dt, cal_accel, cal_gyro);

                        auto q_nav = navigator_.get_quaternion();
                        std::array<float, 4> q_filter = { q_nav[0], q_nav[1], q_nav[2], q_nav[3] };
                        std::array<float, 3> euler = quaternion_to_euler(q_filter);

                        auto now = std::chrono::steady_clock::now();
                        std::chrono::duration<float> elapsed = now - fps_start;
                        if (elapsed.count() >= 1.0f) {
                            current_fps = static_cast<float>(sample_count) / elapsed.count();
                            sample_count = 0;
                            fps_start = now;
                        }

                        if (callback_) {
                            Telemetry telemetry;
                            telemetry.timestamp_ns = raw.timestamp;
                            telemetry.temperature_c = raw.temp_c;
                            std::copy(cal_accel.begin(), cal_accel.end(), telemetry.accel);
                            std::copy(cal_gyro.begin(), cal_gyro.end(), telemetry.gyro);
                            std::copy(q_filter.begin(), q_filter.end(), telemetry.quaternion);
                            std::copy(euler.begin(), euler.end(), telemetry.euler);
                            telemetry.fps = current_fps;

                            auto pos = navigator_.get_position();
                            auto vel = navigator_.get_velocity();
                            auto bg  = navigator_.get_gyro_bias();
                            auto ba  = navigator_.get_accel_bias();
                            auto grav = navigator_.get_gravity();
                            std::copy(pos.begin(), pos.end(), telemetry.position);
                            std::copy(vel.begin(), vel.end(), telemetry.velocity);
                            std::copy(bg.begin(),  bg.end(),  telemetry.gyro_bias);
                            std::copy(ba.begin(),  ba.end(),  telemetry.accel_bias);
                            std::copy(grav.begin(), grav.end(), telemetry.gravity);

                            auto mah_q = mahony_filter_.get_quaternion();
                            std::copy(mah_q.begin(), mah_q.end(), telemetry.mahony_quaternion);

                            auto cov_diag = navigator_.get_covariance_diag();
                            std::copy(cov_diag.begin(), cov_diag.end(), telemetry.covariance_diag);

                            callback_(telemetry);
                        }
                    }
                }
            } else if (res < 0) {
                std::cerr << "[Driver] Error reading from HID interface. Triggering disconnect." << std::endl;
                handle_disconnect();
            }
        }
    }

    std::atomic<bool> streaming_;
    hid_device* device_;
    hid_device* device_control_;
    hid_device* device_button_;
    std::thread control_thread_;
    std::atomic<bool> control_listening_;
    std::atomic<int> brightness_;
    std::atomic<int> display_mode_;
    std::atomic<bool> is_calibration_from_device_;

    uint16_t detected_pid_;
    int sensor_interface_;
    int control_interface_;
    int button_interface_;

    Calibration calibration_;
    ImuNavigator navigator_;
    MahonyFilter mahony_filter_{0.5f, 0.0f};
    bool mahony_aligned_ = false;
    TelemetryCallback callback_;
    std::thread thread_;

    std::mutex device_mutex_;
    std::atomic<bool> disconnected_;
    std::atomic<bool> needs_sensor_handshake_;
    std::atomic<bool> needs_control_handshake_;
};

XRealAirDriver::XRealAirDriver() : impl_(std::make_unique<Impl>()) {}
XRealAirDriver::~XRealAirDriver() = default;

bool XRealAirDriver::initialize(const std::string& calibration_json_path) {
    return impl_->initialize(calibration_json_path);
}

int XRealAirDriver::get_brightness() const {
    return impl_->get_brightness();
}

bool XRealAirDriver::set_brightness(int brightness) {
    return impl_->set_brightness(brightness);
}

int XRealAirDriver::get_display_mode() const {
    return impl_->get_display_mode();
}

bool XRealAirDriver::set_display_mode(int mode) {
    return impl_->set_display_mode(mode);
}

bool XRealAirDriver::is_calibration_from_device() const {
    return impl_->is_calibration_from_device();
}

#if defined(__ANDROID__) || defined(__linux__)
bool XRealAirDriver::initialize_with_fd(int fd, const std::string& calibration_json_path) {
    return impl_->initialize_with_fd(fd, calibration_json_path);
}
#endif

bool XRealAirDriver::start_streaming(const std::string& filter_type, TelemetryCallback callback) {
    return impl_->start_streaming(filter_type, callback);
}

void XRealAirDriver::stop_streaming() {
    impl_->stop_streaming();
}

bool XRealAirDriver::is_streaming() const {
    return impl_->is_streaming();
}

} // namespace xreal
