#define IMU_DRIVER_EXPORTS
#include "myxreal/stereo_camera.h"
#include "myxreal/imu_driver.h"
#include "ring_buffer.h"
#include "myxreal/calibration_loader.h"

#include <windows.h>
#include <dshow.h>
#include <initguid.h>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <cmath>

// Link DirectShow
#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "oleaut32.lib")

// =============================================================================
// DirectShow interfaces (avoid qedit.h dependency)
// =============================================================================
interface ISampleGrabberCB : public IUnknown {
    virtual HRESULT STDMETHODCALLTYPE SampleCB(double SampleTime, IMediaSample* pSample) = 0;
    virtual HRESULT STDMETHODCALLTYPE BufferCB(double SampleTime, BYTE* pBuffer, long BufferLen) = 0;
};

interface ISampleGrabber : public IUnknown {
    virtual HRESULT STDMETHODCALLTYPE SetOneShot(BOOL OneShot) = 0;
    virtual HRESULT STDMETHODCALLTYPE SetMediaType(const AM_MEDIA_TYPE* pType) = 0;
    virtual HRESULT STDMETHODCALLTYPE GetConnectedMediaType(AM_MEDIA_TYPE* pType) = 0;
    virtual HRESULT STDMETHODCALLTYPE SetBufferSamples(BOOL BufferSamples) = 0;
    virtual HRESULT STDMETHODCALLTYPE GetCurrentBuffer(long* pBufferSize, long* pBuffer) = 0;
    virtual HRESULT STDMETHODCALLTYPE GetCurrentSample(IMediaSample** ppSample) = 0;
    virtual HRESULT STDMETHODCALLTYPE SetCallback(ISampleGrabberCB* pCallback, long WhichMethodToCallback) = 0;
};

// CLSID / IID for SampleGrabber and NullRenderer
DEFINE_GUID(CLSID_SampleGrabber,
    0xC1F400A0, 0x3F08, 0x11D3, 0x9F, 0x0B, 0x00, 0x60, 0x08, 0x03, 0x9E, 0x37);
DEFINE_GUID(IID_ISampleGrabber,
    0x6B652FFF, 0x11FE, 0x4FCE, 0x92, 0xAD, 0x02, 0x66, 0xB5, 0xD7, 0xC7, 0x8F);
DEFINE_GUID(CLSID_NullRenderer,
    0xC1F400A4, 0x3F08, 0x11D3, 0x9F, 0x0B, 0x00, 0x60, 0x08, 0x03, 0x9E, 0x37);

// =============================================================================
// 128-chunk descrambling hardware map
// =============================================================================
static const int g_chunkReorderMap[128] = {
    119, 54, 21, 0, 108, 22, 51, 63, 93, 99, 67, 7, 32, 112, 52, 43,
    14, 35, 75, 116, 64, 71, 44, 89, 18, 88, 26, 61, 70, 56, 90, 79,
    87, 120, 81, 101, 121, 17, 72, 31, 53, 124, 127, 113, 111, 36, 48,
    19, 37, 83, 126, 74, 109, 5, 84, 41, 76, 30, 110, 29, 12, 115, 28,
    102, 105, 62, 103, 20, 3, 68, 49, 77, 117, 125, 106, 60, 69, 98, 9,
    16, 78, 47, 40, 2, 118, 34, 13, 50, 46, 80, 85, 66, 42, 123, 122,
    96, 11, 25, 97, 39, 6, 86, 1, 8, 82, 92, 59, 104, 24, 15, 73, 65,
    38, 58, 10, 23, 33, 55, 57, 107, 100, 94, 27, 95, 45, 91, 4, 114
};

// =============================================================================
// Descramble LUT: [isRight][2]  -> lut[pixel_index] = target_byte_offset
// We only build [isRight][0][0] (no swap, no flip) for the default path.
// =============================================================================
static uint32_t g_descrambleLut[2][307200]; // [isRight][pixel_index]
static bool g_lutReady = false;

static void buildDescrambleLut() {
    const int numBlocks = 128;
    const int blockSize = 2400;

    for (int isRight = 0; isRight < 2; ++isRight) {
        int p_y = 0, p_x = 0, lutIdx = 0;

        for (int t_idx = 0; t_idx < numBlocks; ++t_idx) {
            int blockOffset = 0;
            while (blockOffset < blockSize) {
                int remainingInBlock = blockSize - blockOffset;
                int p_y_new = (p_y + remainingInBlock < 640) ? (p_y + remainingInBlock) : 640;
                int segmentCount = p_y_new - p_y;

                for (int k = 0; k < segmentCount; ++k) {
                    int r = p_x;
                    int c = p_y + k;
                    int x_target, y_target;

                    if (isRight) {
                        y_target = c;
                        x_target = 480 + r;  // right half, no swap, no flip
                    } else {
                        y_target = 639 - c;
                        x_target = 479 - r;  // left half, no swap, no flip
                    }

                    g_descrambleLut[isRight][lutIdx++] = y_target * 960 + x_target;
                }

                blockOffset += segmentCount;
                p_y += segmentCount;

                if (p_y >= 640) { p_x++; p_y = 0; }
            }
        }
    }
    g_lutReady = true;
}

// =============================================================================
// Descramble a single frame into the stereo buffer at the appropriate half
// =============================================================================
static BYTE g_stereoBuffer[960 * 640]; // side-by-side: left [0..479], right [480..959]

static void descrambleFrame(const BYTE* raw, int alignmentOffset, bool isRight) {
    if (!g_lutReady) buildDescrambleLut();

    const int numBlocks = 128;
    const int blockSize = 2400;
    const uint32_t* lut = g_descrambleLut[isRight ? 1 : 0];

    for (int t_idx = 0; t_idx < numBlocks; ++t_idx) {
        int reorderedBlockIdx = g_chunkReorderMap[(alignmentOffset + t_idx) % numBlocks];
        const BYTE* src = raw + reorderedBlockIdx * blockSize;
        const uint32_t* blockLut = lut + t_idx * blockSize;

        for (int i = 0; i < blockSize; ++i) {
            g_stereoBuffer[blockLut[i]] = src[i];
        }
    }
}

// =============================================================================
// System time helpers — QPC-based to match IMU driver time domain exactly
// =============================================================================
static double steady_seconds() {
    static LARGE_INTEGER freq = {0, 0};
    static double inv_freq = 0.0;
    if (freq.QuadPart == 0) {
        QueryPerformanceFrequency(&freq);
        inv_freq = 1.0 / static_cast<double>(freq.QuadPart);
    }
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return static_cast<double>(now.QuadPart) * inv_freq;
}

static uint64_t steady_ns() {
    return static_cast<uint64_t>(steady_seconds() * 1e9);
}

// Pipeline latency: DirectShow SampleTime reflects when the frame enters the
// graph, which is 15-30 ms after sensor exposure midpoint (readout + ISP + USB).
// This constant shifts camera timestamps earlier to align with IMU (HID ~1 ms).
// Tune via shake-test cross-corr lag: set to the negative of the measured lag.
static constexpr int64_t kCamPipelineDelayNs = 16666667; // 16.667 ms

// =============================================================================
// Global camera state
// =============================================================================
static std::atomic<bool> g_cam_streaming{false};
static std::atomic<bool> g_cam_ready{false};
static std::thread         g_cam_thread;

// Per-eye tracking
static uint64_t  g_eye_frame_index[2] = {0, 0};
static uint64_t  g_eye_last_ts_ns[2]  = {0, 0};
static float     g_eye_fps[2]          = {0.0f, 0.0f};
static uint64_t  g_eye_fps_count[2]    = {0, 0};
static uint64_t  g_eye_fps_start_ns[2] = {0, 0};

// Per-eye pixel buffers — filled independently, paired when both are fresh
static BYTE g_eye_pixels[2][STEREO_EYE_PIXELS];
static bool g_eye_new[2] = {false, false};

// Drop counter (incremented when we see two frames of the same eye in a row,
// indicating a missed frame from the other eye)
static int g_drop_count = 0;

// Stereo pair ring buffer
struct StereoEntry {
    StereoPair pair;
    uint8_t    left_pixels[STEREO_EYE_PIXELS];
    uint8_t    right_pixels[STEREO_EYE_PIXELS];
};
static RingBuffer<StereoEntry, 64> g_stereo_ring;

// Staging area
static StereoEntry g_last_pair;
static bool        g_last_pair_valid = false;

// Calibration pointer
static const CalibrationData* g_calib = nullptr;

// Mutex for stereo buffer access
static std::mutex g_stereo_mutex;

// Camera clock model: maps DirectShow SampleTime → QPC system time.
// DirectShow's graph clock (timeGetTime) and QPC can drift at ~50-100 ppm;
// a single-epoch mapping accumulates ms-level error within seconds.
// We track (SampleTime, QPC) pairs and refit an affine model, same as the IMU.
struct CamClockSample { double ds_time_s; uint64_t qpc_ns; };
static constexpr int CAM_CLOCK_WINDOW       = 300;
static constexpr int CAM_CLOCK_REFIT_EVERY  =  50;
static CamClockSample g_cam_clk_buf[CAM_CLOCK_WINDOW];
static int            g_cam_clk_head        = 0;
static int            g_cam_clk_count       = 0;
static int            g_cam_clk_since_refit = 0;
static double         g_cam_scale           = 1.0;
static double         g_cam_offset_ns       = 0.0;
static std::mutex     g_cam_clk_mutex;

static void cam_clock_add(double ds_time_s, uint64_t qpc_ns) {
    std::lock_guard<std::mutex> lk(g_cam_clk_mutex);
    // Seed on first sample
    if (g_cam_clk_count == 0) {
        g_cam_scale     = 1.0;
        g_cam_offset_ns = static_cast<double>(qpc_ns) - ds_time_s * 1e9;
    }

    g_cam_clk_buf[g_cam_clk_head].ds_time_s = ds_time_s;
    g_cam_clk_buf[g_cam_clk_head].qpc_ns    = qpc_ns;
    g_cam_clk_head = (g_cam_clk_head + 1) % CAM_CLOCK_WINDOW;
    if (g_cam_clk_count < CAM_CLOCK_WINDOW) g_cam_clk_count++;
    g_cam_clk_since_refit++;

    if (g_cam_clk_since_refit >= CAM_CLOCK_REFIT_EVERY && g_cam_clk_count >= 10) {
        int n = g_cam_clk_count;
        int base = (g_cam_clk_head - n + CAM_CLOCK_WINDOW) % CAM_CLOCK_WINDOW;

        // Centre for stability
        double mx = 0, my = 0;
        for (int i = 0; i < n; ++i) {
            auto& s = g_cam_clk_buf[(base + i) % CAM_CLOCK_WINDOW];
            mx += s.ds_time_s;
            my += static_cast<double>(s.qpc_ns);
        }
        mx /= n; my /= n;

        double num = 0, den = 0;
        for (int i = 0; i < n; ++i) {
            auto& s = g_cam_clk_buf[(base + i) % CAM_CLOCK_WINDOW];
            double dx = s.ds_time_s - mx;
            num += dx * (static_cast<double>(s.qpc_ns) - my);
            den += dx * dx;
        }
        if (den > 0) {
            g_cam_scale     = num / den;  // QPC_ns per DS_time_s (should be ~1e9)
            g_cam_offset_ns = my - g_cam_scale * mx;
        }

        // Outlier rejection (single pass, 3-sigma)
        double sum_sq = 0;
        for (int i = 0; i < n; ++i) {
            auto& s = g_cam_clk_buf[(base + i) % CAM_CLOCK_WINDOW];
            double r = static_cast<double>(s.qpc_ns) - (g_cam_scale * s.ds_time_s + g_cam_offset_ns);
            sum_sq += r * r;
        }
        double sigma = std::sqrt(sum_sq / n);

        double mx2 = 0, my2 = 0;
        int n2 = 0;
        for (int i = 0; i < n; ++i) {
            auto& s = g_cam_clk_buf[(base + i) % CAM_CLOCK_WINDOW];
            double r = static_cast<double>(s.qpc_ns) - (g_cam_scale * s.ds_time_s + g_cam_offset_ns);
            if (std::abs(r) > 3.0 * sigma) continue;
            mx2 += s.ds_time_s;
            my2 += static_cast<double>(s.qpc_ns);
            n2++;
        }
        if (n2 >= 10) {
            mx2 /= n2; my2 /= n2;
            num = 0; den = 0;
            for (int i = 0; i < n; ++i) {
                auto& s = g_cam_clk_buf[(base + i) % CAM_CLOCK_WINDOW];
                double r = static_cast<double>(s.qpc_ns) - (g_cam_scale * s.ds_time_s + g_cam_offset_ns);
                if (std::abs(r) > 3.0 * sigma) continue;
                double dx = s.ds_time_s - mx2;
                num += dx * (static_cast<double>(s.qpc_ns) - my2);
                den += dx * dx;
            }
            if (den > 0) {
                g_cam_scale     = num / den;
                g_cam_offset_ns = my2 - g_cam_scale * mx2;
            }
        }
        g_cam_clk_since_refit = 0;
    }
}

static uint64_t cam_clock_to_qpc_ns(double ds_time_s) {
    std::lock_guard<std::mutex> lk(g_cam_clk_mutex);
    return static_cast<uint64_t>(ds_time_s * g_cam_scale + g_cam_offset_ns);
}

// =============================================================================
// DirectShow helper functions
// =============================================================================
static HRESULT FindVideoDevice(const wchar_t* targetName, IBaseFilter** ppSrcFilter) {
    ICreateDevEnum* pDevEnum = nullptr;
    IEnumMoniker*   pEnum   = nullptr;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                                  IID_PPV_ARGS(&pDevEnum));
    if (FAILED(hr)) return hr;

    hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum, 0);
    if (hr == S_FALSE) { pDevEnum->Release(); return E_FAIL; }

    IMoniker* pMoniker = nullptr;
    bool found = false;
    while (pEnum->Next(1, &pMoniker, nullptr) == S_OK) {
        IPropertyBag* pPropBag = nullptr;
        hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
        if (SUCCEEDED(hr)) {
            VARIANT var; VariantInit(&var);
            hr = pPropBag->Read(L"FriendlyName", &var, 0);
            if (SUCCEEDED(hr)) {
                if (targetName == nullptr || wcsstr(var.bstrVal, targetName) != nullptr) {
                    hr = pMoniker->BindToObject(0, 0, IID_IBaseFilter, (void**)ppSrcFilter);
                    if (SUCCEEDED(hr)) found = true;
                }
                VariantClear(&var);
            }
            pPropBag->Release();
        }
        pMoniker->Release();
        if (found) break;
    }
    pEnum->Release();
    pDevEnum->Release();
    return found ? S_OK : E_FAIL;
}

static HRESULT FindFilterPin(IBaseFilter* pFilter, PIN_DIRECTION PinDir, IPin** ppPin) {
    IEnumPins* pEnum = nullptr;
    IPin* pPin = nullptr;
    HRESULT hr = pFilter->EnumPins(&pEnum);
    if (FAILED(hr)) return hr;
    while (pEnum->Next(1, &pPin, nullptr) == S_OK) {
        PIN_DIRECTION dir;
        pPin->QueryDirection(&dir);
        if (dir == PinDir) { pEnum->Release(); *ppPin = pPin; return S_OK; }
        pPin->Release();
    }
    pEnum->Release();
    return E_FAIL;
}

static HRESULT SetPinFormat(IPin* pPin, const GUID& subtype, int width, int height) {
    IAMStreamConfig* pConfig = nullptr;
    HRESULT hr = pPin->QueryInterface(IID_PPV_ARGS(&pConfig));
    if (FAILED(hr)) return hr;

    int count = 0, size = 0;
    hr = pConfig->GetNumberOfCapabilities(&count, &size);
    if (FAILED(hr)) { pConfig->Release(); return hr; }

    BYTE* pCaps = new BYTE[size];
    bool ok = false;
    for (int i = 0; i < count && !ok; ++i) {
        AM_MEDIA_TYPE* pmt = nullptr;
        hr = pConfig->GetStreamCaps(i, &pmt, pCaps);
        if (SUCCEEDED(hr)) {
            if (pmt->majortype == MEDIATYPE_Video && pmt->subtype == subtype &&
                pmt->formattype == FORMAT_VideoInfo && pmt->cbFormat >= sizeof(VIDEOINFOHEADER)) {
                VIDEOINFOHEADER* vih = (VIDEOINFOHEADER*)pmt->pbFormat;
                if (vih->bmiHeader.biWidth == width && vih->bmiHeader.biHeight == height) {
                    hr = pConfig->SetFormat(pmt);
                    if (SUCCEEDED(hr)) ok = true;
                }
            }
            if (pmt->cbFormat) CoTaskMemFree((PVOID)pmt->pbFormat);
            if (pmt->pUnk) pmt->pUnk->Release();
            CoTaskMemFree((PVOID)pmt);
        }
    }
    delete[] pCaps;
    pConfig->Release();
    return ok ? S_OK : E_FAIL;
}

// =============================================================================
// SampleGrabber callback — the heart of the camera pipeline
// =============================================================================
class CameraGrabberCB : public ISampleGrabberCB {
public:
    STDMETHODIMP QueryInterface(REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        static const GUID IID_ISampleGrabberCB_ =
            { 0x0579154a, 0x2b53, 0x4994, { 0xb0,0xd0,0xe7,0x73,0x14,0x8e,0xff,0x85 } };
        if (riid == IID_IUnknown || riid == IID_ISampleGrabberCB_) {
            *ppv = static_cast<ISampleGrabberCB*>(this);
            return S_OK;
        }
        return E_NOINTERFACE;
    }
    STDMETHODIMP_(ULONG) AddRef() override  { return 1; }
    STDMETHODIMP_(ULONG) Release() override { return 1; }
    STDMETHODIMP SampleCB(double, IMediaSample*) override { return S_OK; }

    STDMETHODIMP BufferCB(double SampleTime, BYTE* pBuffer, long BufferLen) override {
        if (BufferLen < 640 * 241 * 2) return S_OK;

        const int numBlocks = 128;
        const int blockSize = 2400;

        // 1. Sync detection: find chunk with lowest sum of first 128 bytes
        int syncIndex = 0;
        int minScore = 0x7FFFFFFF;
        for (int i = 0; i < numBlocks; ++i) {
            int score = 0;
            const BYTE* bs = pBuffer + i * blockSize;
            for (int j = 0; j < 128; ++j) score += bs[j];
            if (score < minScore) { minScore = score; syncIndex = i; }
        }

        // 2. Find alignment offset
        int alignmentOffset = -1;
        for (int i = 0; i < numBlocks; ++i) {
            if (g_chunkReorderMap[i] == syncIndex) { alignmentOffset = i; break; }
        }
        if (alignmentOffset < 0) return S_OK;

        // 3. Eye detection
        bool isRightSensor = (pBuffer[307200 + 0x3b] != 0);
        int eye = isRightSensor ? 1 : 0;

        static int s_last_eye = -1;
        static int s_same_eye_streak = 0;
        if (eye == s_last_eye) {
            s_same_eye_streak++;
        } else {
            s_same_eye_streak = 0;
        }

        if (s_same_eye_streak >= 8 && (g_eye_frame_index[0] == 0 || g_eye_frame_index[1] == 0)) {
            eye = 1 - eye;
            isRightSensor = (eye == 1);
            s_same_eye_streak = 0;
            fprintf(stderr, "[Camera] Eye marker fallback engaged (bootstrapping alternating eyes).\n");
        }
        s_last_eye = eye;

        if (g_eye_frame_index[0] == 0 && g_eye_frame_index[1] == 0) {
            s_last_eye = eye;
            s_same_eye_streak = 0;
        }
        if (g_eye_frame_index[0] > 0 && g_eye_frame_index[1] > 0) {
            s_same_eye_streak = 0;
        }

        if (BufferLen < 307264) return S_OK;

        // 4. Timestamp — affine model DS SampleTime → QPC (same domain as IMU clock model)
        uint64_t qpc_ns = static_cast<uint64_t>(steady_seconds() * 1e9);
        cam_clock_add(SampleTime, qpc_ns);
        uint64_t ts_ns = cam_clock_to_qpc_ns(SampleTime);

        // 5. Descramble into shared stereo buffer, then copy to per-eye buffer
        {
            std::lock_guard<std::mutex> lock(g_stereo_mutex);
            descrambleFrame(pBuffer, alignmentOffset, isRightSensor); // LUT tied to physical sensor
            // Copy this eye's half into its dedicated buffer
            int col = (eye == 1) ? 480 : 0;
            for (int y = 0; y < 640; ++y)
                std::memcpy(g_eye_pixels[eye] + y * 480, g_stereoBuffer + y * 960 + col, 480);
        }

        // 6. Per-eye tracking
        g_eye_last_ts_ns[eye] = ts_ns;
        g_eye_frame_index[eye]++;

        // FPS
        g_eye_fps_count[eye]++;
        uint64_t fps_delta = ts_ns - g_eye_fps_start_ns[eye];
        if (fps_delta >= 1000000000ULL) {
            g_eye_fps[eye] = g_eye_fps_count[eye] * 1e9f / (float)fps_delta;
            g_eye_fps_count[eye] = 0;
            g_eye_fps_start_ns[eye] = ts_ns;
        }

        // 7. Drop detection: if this eye was already new (we're getting two
        //    frames of the same eye before the other eye updated), count a drop.
        if (g_eye_new[eye]) g_drop_count++;

        g_eye_new[eye] = true;

        // 8. Emit StereoPair only when BOTH eyes have fresh data
        if (g_eye_new[0] && g_eye_new[1]) {
            StereoEntry entry;
            std::memcpy(entry.left_pixels,  g_eye_pixels[0], STEREO_EYE_PIXELS);
            std::memcpy(entry.right_pixels, g_eye_pixels[1], STEREO_EYE_PIXELS);

            // Right camera fires ~16.667ms BEFORE left (60Hz alternating).
            // Use fixed compensation in the timestamp model.
            static constexpr int64_t kRightShiftNs = 16666667; // 16.667ms

            // Both left and right timestamps are in QPC system time (affine model
            // from DS SampleTime → QPC). Subtract pipeline delay so timestamps
            // align with IMU (which has ~1ms HID latency vs ~15-30ms camera pipeline).
            uint64_t left_sys_ns  = g_eye_last_ts_ns[0]  - kCamPipelineDelayNs;
            uint64_t right_sys_ns = g_eye_last_ts_ns[1] + kRightShiftNs - kCamPipelineDelayNs;

            entry.pair.left.timestamp_ns = left_sys_ns;
            entry.pair.left.frame_index  = g_eye_frame_index[0];
            entry.pair.left.width        = STEREO_EYE_WIDTH;
            entry.pair.left.height       = STEREO_EYE_HEIGHT;
            entry.pair.left.camera_id    = 0;
            entry.pair.left.data         = entry.left_pixels;
            entry.pair.left.fps          = g_eye_fps[0];
            entry.pair.left.is_rectified = false;

            entry.pair.right.timestamp_ns = right_sys_ns;
            entry.pair.right.frame_index  = g_eye_frame_index[1];
            entry.pair.right.width        = STEREO_EYE_WIDTH;
            entry.pair.right.height       = STEREO_EYE_HEIGHT;
            entry.pair.right.camera_id    = 1;
            entry.pair.right.data         = entry.right_pixels;
            entry.pair.right.fps          = g_eye_fps[1];
            entry.pair.right.is_rectified = false;

            int64_t diff = (int64_t)left_sys_ns - (int64_t)right_sys_ns;
            entry.pair.sync_delta_ms = std::abs((float)diff) / 1e6f;
            entry.pair.timestamp_ns  = (left_sys_ns + right_sys_ns) / 2;
            entry.pair.drops         = g_drop_count;
            entry.pair.calib         = g_calib;

            g_stereo_ring.push(entry);

            // Reset for next pair
            g_eye_new[0] = false;
            g_eye_new[1] = false;
        }

        return S_OK;
    }
};

// =============================================================================
// Camera thread (owns the DirectShow graph)
// =============================================================================
static void camera_thread() {
    g_cam_ready = false;
    HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    if (FAILED(hr)) {
        fprintf(stderr, "[Camera] CoInitializeEx failed: 0x%08X\n", (unsigned)hr);
        g_cam_streaming = false;
        return;
    }

    IGraphBuilder*   pGraph       = nullptr;
    IBaseFilter*     pCapSource   = nullptr;
    IBaseFilter*     pGrabberFlt  = nullptr;
    ISampleGrabber*  pGrabber     = nullptr;
    IBaseFilter*     pNullRender  = nullptr;
    IMediaControl*   pControl     = nullptr;
    CameraGrabberCB  callback;

    hr = CoCreateInstance(CLSID_FilterGraph, nullptr, CLSCTX_INPROC_SERVER,
                          IID_PPV_ARGS(&pGraph));
    if (FAILED(hr)) goto cleanup;

    // Find the camera
    hr = FindVideoDevice(L"UVC Camera 0", &pCapSource);
    if (FAILED(hr)) hr = FindVideoDevice(nullptr, &pCapSource);
    if (FAILED(hr) || !pCapSource) {
        fprintf(stderr, "[Camera] No video capture device found.\n");
        goto cleanup;
    }

    hr = pGraph->AddFilter(pCapSource, L"Capture Device");
    if (FAILED(hr)) goto cleanup;

    // Set format: 640x241 YUY2 (raw chunk format)
    {
        IPin* pCapOutPin = nullptr;
        hr = FindFilterPin(pCapSource, PINDIR_OUTPUT, &pCapOutPin);
        if (SUCCEEDED(hr)) {
            hr = SetPinFormat(pCapOutPin, MEDIASUBTYPE_YUY2, 640, 241);
            if (FAILED(hr)) {
                fprintf(stderr, "[Camera] Could not set 640x241 YUY2 format. "
                       "Camera may not be XREAL glasses.\n");
            }
            pCapOutPin->Release();
        }
    }
    if (FAILED(hr)) goto cleanup;

    // SampleGrabber
    hr = CoCreateInstance(CLSID_SampleGrabber, nullptr, CLSCTX_INPROC_SERVER,
                          IID_PPV_ARGS(&pGrabberFlt));
    if (FAILED(hr)) goto cleanup;
    hr = pGraph->AddFilter(pGrabberFlt, L"Grabber");
    if (FAILED(hr)) goto cleanup;
    hr = pGrabberFlt->QueryInterface(IID_ISampleGrabber, (void**)&pGrabber);
    if (FAILED(hr)) goto cleanup;

    // Connect source -> grabber
    {
        AM_MEDIA_TYPE mt = {};
        mt.majortype = MEDIATYPE_Video;
        mt.subtype   = MEDIASUBTYPE_YUY2;
        pGrabber->SetMediaType(&mt);

        IPin* pCapOutPin = nullptr;
        IPin* pGrabInPin = nullptr;
        hr = FindFilterPin(pCapSource, PINDIR_OUTPUT, &pCapOutPin);
        if (SUCCEEDED(hr)) {
            hr = FindFilterPin(pGrabberFlt, PINDIR_INPUT, &pGrabInPin);
            if (SUCCEEDED(hr)) {
                hr = pGraph->Connect(pCapOutPin, pGrabInPin);
                pGrabInPin->Release();
            }
            pCapOutPin->Release();
        }
    }
    if (FAILED(hr)) goto cleanup;

    // Null renderer
    hr = CoCreateInstance(CLSID_NullRenderer, nullptr, CLSCTX_INPROC_SERVER,
                          IID_PPV_ARGS(&pNullRender));
    if (FAILED(hr)) goto cleanup;
    pGraph->AddFilter(pNullRender, L"Null Renderer");

    // Connect grabber -> null renderer
    {
        IPin* pGrabOutPin = nullptr;
        IPin* pNullInPin  = nullptr;
        hr = FindFilterPin(pGrabberFlt, PINDIR_OUTPUT, &pGrabOutPin);
        if (SUCCEEDED(hr)) {
            hr = FindFilterPin(pNullRender, PINDIR_INPUT, &pNullInPin);
            if (SUCCEEDED(hr)) {
                hr = pGraph->Connect(pGrabOutPin, pNullInPin);
                pNullInPin->Release();
            }
            pGrabOutPin->Release();
        }
    }
    if (FAILED(hr)) goto cleanup;

    // Configure and start
    pGrabber->SetOneShot(FALSE);
    pGrabber->SetBufferSamples(TRUE);
    hr = pGrabber->SetCallback(&callback, 1);
    if (FAILED(hr)) goto cleanup;

    hr = pGraph->QueryInterface(IID_PPV_ARGS(&pControl));
    if (FAILED(hr)) goto cleanup;

    hr = pControl->Run();
    if (FAILED(hr)) {
        fprintf(stderr, "[Camera] Failed to start capture graph.\n");
        goto cleanup;
    }

    g_cam_ready = true;
    printf("[Camera] Streaming started at 640x241 YUY2 -> 480x640 per eye greyscale\n");

    // Wait for stop signal, pumping Windows messages
    while (g_cam_streaming) {
        MSG msg;
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Stop graph
    if (pControl) pControl->Stop();

cleanup:
    if (FAILED(hr) && hr != E_FAIL) {
        fprintf(stderr, "[Camera] Init failed: 0x%08X\n", (unsigned)hr);
    }

    if (pControl)     pControl->Release();
    if (pGrabber)     pGrabber->Release();
    if (pGrabberFlt)  pGrabberFlt->Release();
    if (pNullRender)  pNullRender->Release();
    if (pCapSource)   pCapSource->Release();
    if (pGraph)       pGraph->Release();

    CoUninitialize();
    printf("[Camera] Streaming stopped.\n");
}

// =============================================================================
// Public API
// =============================================================================
IMU_API int stereo_init(void) {
    g_drop_count = 0;
    std::memset(g_eye_frame_index, 0, sizeof(g_eye_frame_index));
    std::memset(g_eye_last_ts_ns, 0, sizeof(g_eye_last_ts_ns));
    std::memset(g_eye_fps, 0, sizeof(g_eye_fps));
    std::memset(g_eye_fps_count, 0, sizeof(g_eye_fps_count));
    std::memset(g_eye_fps_start_ns, 0, sizeof(g_eye_fps_start_ns));
    std::memset(g_eye_pixels, 0, sizeof(g_eye_pixels));
    g_eye_new[0] = false;
    g_eye_new[1] = false;
    g_cam_clk_head = 0;
    g_cam_clk_count = 0;
    g_cam_clk_since_refit = 0;
    g_cam_scale = 1.0;
    g_cam_offset_ns = 0.0;
    g_stereo_ring.clear();
    return 0;
}

IMU_API int stereo_start_streaming(void) {
    if (g_cam_streaming) return 0;
    g_cam_ready = false;
    g_cam_streaming = true;
    g_cam_thread = std::thread(camera_thread);

    auto t0 = std::chrono::steady_clock::now();
    while (g_cam_streaming && !g_cam_ready) {
        auto dt = std::chrono::steady_clock::now() - t0;
        if (dt >= std::chrono::milliseconds(1500)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!g_cam_streaming || !g_cam_ready) {
        fprintf(stderr, "[Camera] start_streaming failed: graph did not become ready.\n");
        g_cam_streaming = false;
        if (g_cam_thread.joinable()) g_cam_thread.join();
        return -1;
    }
    return 0;
}

IMU_API void stereo_stop_streaming(void) {
    g_cam_streaming = false;
    if (g_cam_thread.joinable()) g_cam_thread.join();
}

IMU_API int stereo_poll_pair(StereoPair* out_pair) {
    if (!out_pair) return 0;

    StereoEntry entry;
    if (!g_stereo_ring.pop(entry)) return 0;

    // Copy to staging area so pointers remain valid across calls
    g_last_pair = entry;
    g_last_pair_valid = true;

    // Fix up data pointers to point into the staging area
    g_last_pair.pair.left.data  = g_last_pair.left_pixels;
    g_last_pair.pair.right.data = g_last_pair.right_pixels;

    *out_pair = g_last_pair.pair;
    return 1;
}

IMU_API int stereo_available_pairs(void) {
    return (int)g_stereo_ring.available();
}

IMU_API void stereo_set_calibration(const CalibrationData* calib) {
    g_calib = calib;
}

IMU_API void stereo_shutdown(void) {
    stereo_stop_streaming();
}
