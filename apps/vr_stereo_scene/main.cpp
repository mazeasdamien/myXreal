#define NOMINMAX
#include <windows.h>
#include <d3d11.h>
#include <dxgi.h>
#include <d3dcompiler.h>
#include <directxmath.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <cmath>
#include <array>
#include <utility>
#include <cstring>
#include <algorithm>
#include <filesystem>
#include "xreal_air_driver/xreal_air_driver.h"
#include <hidapi.h>


#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

using namespace DirectX;

// ==============================================================================
// 1. DATA STRUCTURES
// ==============================================================================
struct Vertex {
    XMFLOAT3 pos;
    XMFLOAT3 normal;  // vertex normal for smooth shading
    XMFLOAT4 color;
};

struct ConstantBuffer {
    XMMATRIX mvp;
    XMMATRIX world;
    XMFLOAT4 cameraPos;
    XMFLOAT4 useLighting; // x = useLighting (1.0f/0.0f), y = useSpecular (1.0f/0.0f)
};

void GetTextLines(const std::string& text, float x, float y, float w, float h,
                  float spacing, float sw, float sh, XMFLOAT4 col, std::vector<Vertex>& out);
void AddStroke(float x1, float y1, float x2, float y2, float x, float y,
               float w, float h, float sw, float sh, XMFLOAT4 col, std::vector<Vertex>& out);

// Shader source code embedded as strings
const char* g_shaderCode = R"(
cbuffer ConstantBuffer : register(b0) {
    matrix mvp;
    matrix world;
    float4 cameraPos;
    float4 useLighting; // x = useLighting (1.0f/0.0f), y = useSpecular (1.0f/0.0f)
};

struct VS_INPUT {
    float4 pos    : POSITION;
    float3 normal : NORMAL;
    float4 color  : COLOR;
};

struct VS_OUTPUT {
    float4 pos       : SV_POSITION;
    float3 worldPos  : TEXCOORD0;
    float3 worldNorm : TEXCOORD1;
    float4 color     : COLOR;
};

VS_OUTPUT VS(VS_INPUT input) {
    VS_OUTPUT output;
    output.pos      = mul(input.pos, mvp);
    output.worldPos = mul(input.pos, world).xyz;
    // Transform normal by world matrix (no non-uniform scale assumed)
    output.worldNorm = normalize(mul(float4(input.normal, 0.0f), world).xyz);
    output.color    = input.color;
    return output;
}

float4 PS(VS_OUTPUT input) : SV_Target {
    if (useLighting.z > 0.5f) {
        // Shadow blob: solid dark semi-transparent, no discard needed
        return float4(0.0f, 0.0f, 0.0f, 0.45f);
    }
    if (useLighting.x < 0.5f) {
        return input.color;
    }
    // Use interpolated vertex normal (smooth shading)
    float3 normal = normalize(input.worldNorm);
    if (dot(normal, normal) < 0.01f) normal = float3(0.0f, 1.0f, 0.0f);

    float3 lightDir = normalize(float3(1.0f, 1.5f, -1.0f));
    float3 lightColor = float3(1.0f, 0.95f, 0.85f);

    float3 fillLightDir = normalize(float3(-1.0f, -0.5f, 1.0f));
    float3 fillLightColor = float3(0.15f, 0.2f, 0.3f);

    float3 ambient = float3(0.15f, 0.15f, 0.18f);

    float diffuseTerm = max(dot(normal, lightDir), 0.0f);
    float fillTerm = max(dot(normal, fillLightDir), 0.0f);

    float3 viewDir = normalize(cameraPos.xyz - input.worldPos);
    float3 halfVec = normalize(lightDir + viewDir);
    float specTerm = pow(max(dot(normal, halfVec), 0.0f), 32.0f);
    float3 specular = specTerm * float3(0.4f, 0.4f, 0.4f) * useLighting.y;

    float3 lighting = ambient + diffuseTerm * lightColor + fillTerm * fillLightColor;
    float3 finalColor = input.color.rgb * lighting + specular;
    return float4(finalColor, input.color.a);
}
)";

// ─── Post-process: lens-distortion correction (barrel → pincushion) ───────
const char* g_postShaderCode = R"(
Texture2D    g_sceneTex    : register(t0);
SamplerState g_linearSamp  : register(s0);

struct PP_OUT {
    float4 pos : SV_POSITION;
    float2 uv  : TEXCOORD0;
};

PP_OUT VS_PP(uint id : SV_VertexID) {
    PP_OUT o;
    // Fullscreen triangle (3 vertices cover NDC)
    o.uv  = float2((id == 1) ? 2.0 : 0.0, (id == 2) ? 2.0 : 0.0);
    o.pos = float4(o.uv.x * 2.0 - 1.0, 1.0 - o.uv.y * 2.0, 0.5, 1.0);
    return o;
}

float4 PS_PP(PP_OUT i) : SV_Target {
    return g_sceneTex.Sample(g_linearSamp, float2(i.uv.x, 1.0 - i.uv.y));
}
)";

// ─── Skybox gradient shader (overview background) ─────────────────────────
const char* g_skyShaderCode = R"(
cbuffer SkyBuffer : register(b0) { matrix mvp; matrix world; float4 cameraPos; float4 useLighting; };
struct SKY_IN  { float4 pos : POSITION; float3 normal : NORMAL; float4 color : COLOR; };
struct SKY_OUT { float4 pos : SV_POSITION; float4 color : COLOR; };
SKY_OUT VS_Sky(SKY_IN i) {
    SKY_OUT o;
    o.pos   = i.pos;   // already in NDC
    o.color = i.color;
    return o;
}
float4 PS_Sky(SKY_OUT i) : SV_Target { return i.color; }
)";


// ==============================================================================
// 2. MONITOR RESOLUTION DETECTOR
// ==============================================================================
// Helper function to send USB command to switch glasses display mode (1 = 2D, 3 = 3D SBS)
static bool set_glasses_display_mode(uint8_t mode) {
    if (hid_init() < 0) {
        std::cerr << "[VR Scene] Failed to initialize hidapi for mode switch." << std::endl;
        return false;
    }

    struct hid_device_info* devs = hid_enumerate(0x3318, 0x0426);
    struct hid_device_info* cur_dev = devs;
    const char* target_path = nullptr;
    while (cur_dev) {
        if (cur_dev->interface_number == 0) {
            target_path = cur_dev->path;
            break;
        }
        cur_dev = cur_dev->next;
    }

    if (!target_path) {
        if (devs) hid_free_enumeration(devs);
        std::cerr << "[VR Scene] XREAL control interface not found." << std::endl;
        return false;
    }

    hid_device* handle = hid_open_path(target_path);
    hid_free_enumeration(devs);

    if (!handle) {
        std::cerr << "[VR Scene] Failed to open XREAL control interface." << std::endl;
        return false;
    }

    // CRC32 Lookup Table
    static uint32_t crc_table[256];
    static bool crc_table_initialized = false;
    if (!crc_table_initialized) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int j = 0; j < 8; ++j) {
                if (c & 1) c = 0xEDB88320L ^ (c >> 1);
                else c = c >> 1;
            }
            crc_table[i] = c;
        }
        crc_table_initialized = true;
    }

    auto calculate_crc = [](const uint8_t* data, size_t len) -> uint32_t {
        uint32_t crc = 0xFFFFFFFFL;
        for (size_t i = 0; i < len; ++i) {
            crc = crc_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
        }
        return crc ^ 0xFFFFFFFFL;
    };

    uint8_t data_val = mode;
    uint16_t msgid = 0x08; // MSG_W_DISP_MODE
    uint16_t packet_len = 17 + 1; // 17 headers/footers + 1 byte data

    std::vector<uint8_t> checksum_target;
    checksum_target.push_back(packet_len & 0xff);
    checksum_target.push_back((packet_len >> 8) & 0xff);
    for (int i = 0; i < 8; ++i) checksum_target.push_back(0);
    checksum_target.push_back(msgid & 0xff);
    checksum_target.push_back((msgid >> 8) & 0xff);
    for (int i = 0; i < 5; ++i) checksum_target.push_back(0);
    checksum_target.push_back(data_val);

    uint32_t checksum = calculate_crc(checksum_target.data(), checksum_target.size());

    std::vector<uint8_t> payload;
    payload.push_back(0x00); // Report ID (Windows requirement)
    payload.push_back(0xFD); // CONTROL_HEAD
    payload.push_back(checksum & 0xff);
    payload.push_back((checksum >> 8) & 0xff);
    payload.push_back((checksum >> 16) & 0xff);
    payload.push_back((checksum >> 24) & 0xff);
    payload.insert(payload.end(), checksum_target.begin(), checksum_target.end());

    // Pad report to 65 bytes
    if (payload.size() < 65) {
        payload.resize(65, 0);
    }

    int written = hid_write(handle, payload.data(), payload.size());
    hid_close(handle);

    return written >= 0;
}
struct MonitorInfo {
    RECT rect;
    bool is_xreal;
};

BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData) {
    auto* monitors = reinterpret_cast<std::vector<MonitorInfo>*>(dwData);
    MONITORINFOEXA info = {};
    info.cbSize = sizeof(MONITORINFOEXA);
    if (GetMonitorInfoA(hMonitor, &info)) {
        LONG width = info.rcMonitor.right - info.rcMonitor.left;
        LONG height = info.rcMonitor.bottom - info.rcMonitor.top;

        const bool likely_xreal_by_name =
            (strstr(info.szDevice, "XREAL") != nullptr) ||
            (strstr(info.szDevice, "Nreal") != nullptr) ||
            (strstr(info.szDevice, "Air") != nullptr);

        const bool likely_xreal_by_mode =
            (width == 3840 && height == 1080) ||
            (width == 3840 && height == 1200);

        bool is_xreal = likely_xreal_by_name || likely_xreal_by_mode;
        monitors->push_back({ info.rcMonitor, is_xreal });
    }
    return TRUE;
}

// ==============================================================================
// 3. GLOBAL VARIABLES
// ==============================================================================
HWND                    g_hWnd = nullptr;
ID3D11Device*           g_pd3dDevice = nullptr;
ID3D11DeviceContext*    g_pImmediateContext = nullptr;
IDXGISwapChain*         g_pSwapChain = nullptr;
ID3D11RenderTargetView* g_pRenderTargetView = nullptr;
HWND                    g_hWndOverview = nullptr;
IDXGISwapChain*         g_pSwapChainOverview = nullptr;
ID3D11RenderTargetView* g_pRenderTargetViewOverview = nullptr;

ID3D11Buffer*           g_pHeadVertexBuffer = nullptr;
ID3D11Buffer*           g_pHeadIndexBuffer = nullptr;
ID3D11Buffer*           g_pVisorVertexBuffer = nullptr;
ID3D11Buffer*           g_pVisorIndexBuffer = nullptr;
ID3D11Texture2D*        g_pDepthStencilBufferOverview = nullptr;
ID3D11DepthStencilView* g_pDepthStencilViewOverview = nullptr;
ID3D11Buffer*           g_pDynamicLineBuffer = nullptr;
ID3D11Buffer*           g_pBeaverVertexBuffer = nullptr;
ID3D11Buffer*           g_pBeaverIndexBuffer = nullptr;
UINT                    g_beaverIndexCount = 0;

// Overview Camera Control
float                   g_overviewYaw = -0.5f; // Radians
float                   g_overviewPitch = 0.3f; // Radians
float                   g_overviewDistance = 8.0f; // Zoom
XMVECTOR                g_overviewFocus = XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f); // Orbit look-at point

bool                    g_mouseLButtonDown = false;
bool                    g_mouseRButtonDown = false;
int                     g_lastMouseX = 0;
int                     g_lastMouseY = 0;
ID3D11Texture2D*        g_pDepthStencilBuffer = nullptr;
ID3D11DepthStencilView* g_pDepthStencilView = nullptr;
ID3D11VertexShader*     g_pVertexShader = nullptr;
ID3D11PixelShader*      g_pPixelShader = nullptr;
ID3D11InputLayout*      g_pVertexLayout = nullptr;
ID3D11Buffer*           g_pCubeVertexBuffer = nullptr;
ID3D11Buffer*           g_pCubeIndexBuffer = nullptr;
ID3D11Buffer*           g_pGridVertexBuffer = nullptr;
ID3D11Buffer*           g_pAxesVertexBuffer = nullptr;
ID3D11Buffer*           g_pConstantBuffer = nullptr;
ID3D11RasterizerState*  g_pRasterizerState = nullptr;
ID3D11DepthStencilState* g_pDepthStencilState = nullptr;

UINT g_cubeIndexCount = 0;
UINT g_gridVertexCount = 0;
UINT g_axesVertexCount = 0;
UINT g_headIndexCount = 36;

// HMD Telemetry variables
xreal::XRealAirDriver g_driver;
std::mutex           g_poseMutex;
float                g_hmdQuaternion[4] = { 1.0f, 0.0f, 0.0f, 0.0f }; // [w, x, y, z] - raw sensor output
float                g_hmdQuaternionSmoothed[4] = { 1.0f, 0.0f, 0.0f, 0.0f }; // Final smoothed+predicted for rendering
float                g_hmdQuaternionTrend[4] = { 1.0f, 0.0f, 0.0f, 0.0f };   // Double-exp: second EMA (trend tracker)
float                g_lastGyro[3] = { 0.0f, 0.0f, 0.0f };                    // Latest calibrated gyro (rad/s) for prediction

// ─── MSAA render targets ────────────────────────────────────────────────────
static const UINT MSAA_COUNT = 4;
ID3D11Texture2D*         g_pMSAATarget   = nullptr; // VR window MSAA color
ID3D11RenderTargetView*  g_pMSAARTV      = nullptr;
ID3D11Texture2D*         g_pMSAADepth    = nullptr; // VR window MSAA depth
ID3D11DepthStencilView*  g_pMSAADSV      = nullptr;
ID3D11Texture2D*         g_pMSAATargetOV = nullptr; // Overview MSAA color
ID3D11RenderTargetView*  g_pMSAARTVOV    = nullptr;
ID3D11Texture2D*         g_pMSAADepthOV  = nullptr;
ID3D11DepthStencilView*  g_pMSAADSVOV    = nullptr;

// ─── Post-process resources (lens distortion) ───────────────────────────────
ID3D11Texture2D*          g_pPostTex      = nullptr; // resolved scene
ID3D11ShaderResourceView* g_pPostSRV      = nullptr;
ID3D11RenderTargetView*   g_pPostRTV      = nullptr;
ID3D11VertexShader*       g_pPostVS       = nullptr;
ID3D11PixelShader*        g_pPostPS       = nullptr;
ID3D11SamplerState*       g_pLinearSampler= nullptr;

// ─── Skybox (overview background gradient) ──────────────────────────────────
ID3D11Buffer*             g_pSkyboxVB     = nullptr;
UINT                      g_skyboxVerts   = 6;
ID3D11VertexShader*       g_pSkyVS        = nullptr;
ID3D11PixelShader*        g_pSkyPS        = nullptr;
ID3D11DepthStencilState*  g_pNoDepthState = nullptr;

// ─── Shadow rendering ────────────────────────────────────────────────────────
ID3D11BlendState*         g_pShadowBlend    = nullptr;
ID3D11DepthStencilState*  g_pDepthReadOnly  = nullptr;
ID3D11RasterizerState*    g_pRasterizerStateSolid = nullptr; // CULL_BACK for solid meshes

// ─── Recenter reference quaternion [w, x, y, z] ────────────────────────────
float g_recenterRefQuat[4] = { 1.0f, 0.0f, 0.0f, 0.0f }; // identity (no recenter)
bool                 g_hasHmdConnection = false;
bool                 g_autoRecenterDone = false;
XMVECTOR             g_cameraPos = XMVectorSet(0.0f, 0.0f, -3.0f, 1.0f); // Navigable virtual camera origin
float                g_fovDegrees = 52.0f; // Dynamic Field of View in degrees (Air 2 Ultra default is 52)
static constexpr float kBeaverScale = 0.22f;
static constexpr float kBeaverPosY = -0.35f;
static constexpr float kBeaverPosZ = 1.00f;
static constexpr float kBeaverHeadLockedDistance = 1.20f;
static constexpr float kBeaverHeadLockedYOffset = -0.10f;
static constexpr float kBeaverHeadLockedRollFixRad = XM_PI;
// ==============================================================================
XMMATRIX QuaternionToMatrix(const float* q) {
    // OpenXR / JPL: [w, x, y, z]
    // DirectX Math wants: [x, y, z, w]
    XMVECTOR vec = XMVectorSet(q[1], q[2], q[3], q[0]);
    return XMMatrixRotationQuaternion(vec);
}

// Generate smooth simulation fallback rotation (Sinusoidal pitch/yaw/roll wave)
static void GetRecenteredQuat(float out[4]);

XMMATRIX GetMockRotationMatrix(float timeSec) {
    float yaw = std::sin(timeSec * 0.5f) * 0.45f;
    float pitch = std::cos(timeSec * 0.3f) * 0.25f;
    float roll = std::sin(timeSec * 0.7f) * 0.15f;
    return XMMatrixRotationRollPitchYaw(pitch, yaw, roll);
}

static XMMATRIX GetHmdRenderRotation(float timeSec) {
    if (g_hasHmdConnection) {
        float qRel[4];
        GetRecenteredQuat(qRel);
        return QuaternionToMatrix(qRel);
    }
    return GetMockRotationMatrix(timeSec);
}

static XMMATRIX BuildHeadLockedBeaverWorld(const XMMATRIX& hmdRot, float timeSec) {
    const XMMATRIX mHeadFrame = hmdRot * XMMatrixTranslationFromVector(g_cameraPos);
    const XMMATRIX mHeadLockedOffset = XMMatrixTranslation(0.0f, kBeaverHeadLockedYOffset, kBeaverHeadLockedDistance);
    return XMMatrixScaling(kBeaverScale, kBeaverScale, kBeaverScale)
        * XMMatrixRotationX(-XM_PIDIV2)
        * XMMatrixRotationY(XM_PI + timeSec * 0.4f)
        * XMMatrixRotationZ(kBeaverHeadLockedRollFixRad)
        * mHeadLockedOffset
        * mHeadFrame;
}

static XMMATRIX BuildWorldFixedBeaverWorld(float timeSec) {
    return XMMatrixScaling(kBeaverScale, kBeaverScale, kBeaverScale)
        * XMMatrixRotationX(-XM_PIDIV2)
        * XMMatrixRotationY(XM_PI + timeSec * 0.4f)
        * XMMatrixTranslation(0.0f, kBeaverPosY, kBeaverPosZ);
}

// ==============================================================================
// 5. RENDERING INITIALIZATION
// ==============================================================================
HRESULT InitDevice(int width, int height, int overviewWidth, int overviewHeight) {
    HRESULT hr = S_OK;

    // Create D3D11 Device and SwapChain
    DXGI_SWAP_CHAIN_DESC sd = {};
    sd.BufferCount = 1;
    sd.BufferDesc.Width = width;
    sd.BufferDesc.Height = height;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = g_hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;

    D3D_FEATURE_LEVEL featureLevels[] = { D3D_FEATURE_LEVEL_11_0 };
    D3D_FEATURE_LEVEL featureLevel;

    hr = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0,
        featureLevels, 1, D3D11_SDK_VERSION, &sd,
        &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pImmediateContext
    );
    if (FAILED(hr)) return hr;

    // Create Render Target View
    ID3D11Texture2D* pBackBuffer = nullptr;
    hr = g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
    if (FAILED(hr)) return hr;

    hr = g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_pRenderTargetView);
    pBackBuffer->Release();
    if (FAILED(hr)) return hr;

    // Create Depth Stencil Buffer
    D3D11_TEXTURE2D_DESC descDepth = {};
    descDepth.Width = width;
    descDepth.Height = height;
    descDepth.MipLevels = 1;
    descDepth.ArraySize = 1;
    descDepth.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
    descDepth.SampleDesc.Count = 1;
    descDepth.SampleDesc.Quality = 0;
    descDepth.Usage = D3D11_USAGE_DEFAULT;
    descDepth.BindFlags = D3D11_BIND_DEPTH_STENCIL;

    hr = g_pd3dDevice->CreateTexture2D(&descDepth, nullptr, &g_pDepthStencilBuffer);
    if (FAILED(hr)) return hr;

    hr = g_pd3dDevice->CreateDepthStencilView(g_pDepthStencilBuffer, nullptr, &g_pDepthStencilView);
    if (FAILED(hr)) return hr;

    g_pImmediateContext->OMSetRenderTargets(1, &g_pRenderTargetView, g_pDepthStencilView);

    if (g_hWndOverview) {
        // Create second swap chain for overview window
        IDXGIDevice* pDXGIDevice = nullptr;
        hr = g_pd3dDevice->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void**>(&pDXGIDevice));
        if (SUCCEEDED(hr)) {
            IDXGIAdapter* pDXGIAdapter = nullptr;
            hr = pDXGIDevice->GetAdapter(&pDXGIAdapter);
            if (SUCCEEDED(hr)) {
                IDXGIFactory* pIDXGIFactory = nullptr;
                hr = pDXGIAdapter->GetParent(__uuidof(IDXGIFactory), reinterpret_cast<void**>(&pIDXGIFactory));
                if (SUCCEEDED(hr)) {
                    DXGI_SWAP_CHAIN_DESC sdOverview = {};
                    sdOverview.BufferCount = 1;
                    sdOverview.BufferDesc.Width = overviewWidth;
                    sdOverview.BufferDesc.Height = overviewHeight;
                    sdOverview.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
                    sdOverview.BufferDesc.RefreshRate.Numerator = 60;
                    sdOverview.BufferDesc.RefreshRate.Denominator = 1;
                    sdOverview.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
                    sdOverview.OutputWindow = g_hWndOverview;
                    sdOverview.SampleDesc.Count = 1;
                    sdOverview.SampleDesc.Quality = 0;
                    sdOverview.Windowed = TRUE;

                    hr = pIDXGIFactory->CreateSwapChain(g_pd3dDevice, &sdOverview, &g_pSwapChainOverview);
                    pIDXGIFactory->Release();
                }
                pDXGIAdapter->Release();
            }
            pDXGIDevice->Release();
        }
        if (FAILED(hr)) return hr;

        ID3D11Texture2D* pBackBufferOverview = nullptr;
        hr = g_pSwapChainOverview->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&pBackBufferOverview));
        if (FAILED(hr)) return hr;

        hr = g_pd3dDevice->CreateRenderTargetView(pBackBufferOverview, nullptr, &g_pRenderTargetViewOverview);
        pBackBufferOverview->Release();
        if (FAILED(hr)) return hr;

        // Create Depth Stencil Buffer for Overview Window (matching its dimensions)
        D3D11_TEXTURE2D_DESC descDepthOverview = {};
        descDepthOverview.Width = overviewWidth;
        descDepthOverview.Height = overviewHeight;
        descDepthOverview.MipLevels = 1;
        descDepthOverview.ArraySize = 1;
        descDepthOverview.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
        descDepthOverview.SampleDesc.Count = 1;
        descDepthOverview.SampleDesc.Quality = 0;
        descDepthOverview.Usage = D3D11_USAGE_DEFAULT;
        descDepthOverview.BindFlags = D3D11_BIND_DEPTH_STENCIL;

        hr = g_pd3dDevice->CreateTexture2D(&descDepthOverview, nullptr, &g_pDepthStencilBufferOverview);
        if (FAILED(hr)) return hr;

        hr = g_pd3dDevice->CreateDepthStencilView(g_pDepthStencilBufferOverview, nullptr, &g_pDepthStencilViewOverview);
        if (FAILED(hr)) return hr;
    }

    // Create Dynamic Line Buffer for FOV cone, PiP borders, and label
    D3D11_BUFFER_DESC dyDesc = {};
    dyDesc.Usage = D3D11_USAGE_DYNAMIC;
    dyDesc.ByteWidth = sizeof(Vertex) * 512; // 512 vertices max
    dyDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    dyDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    hr = g_pd3dDevice->CreateBuffer(&dyDesc, nullptr, &g_pDynamicLineBuffer);
    if (FAILED(hr)) return hr;

    // Compile & Create Shaders
    ID3DBlob* pVSBlob = nullptr;
    ID3DBlob* pErrorBlob = nullptr;
    hr = D3DCompile(g_shaderCode, strlen(g_shaderCode), nullptr, nullptr, nullptr, "VS", "vs_4_0", 0, 0, &pVSBlob, &pErrorBlob);
    if (FAILED(hr)) {
        if (pErrorBlob) {
            std::cerr << "VS Compile Error: " << (char*)pErrorBlob->GetBufferPointer() << std::endl;
            pErrorBlob->Release();
        }
        return hr;
    }

    hr = g_pd3dDevice->CreateVertexShader(pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), nullptr, &g_pVertexShader);
    if (FAILED(hr)) {
        pVSBlob->Release();
        return hr;
    }

    // Input Layout
    D3D11_INPUT_ELEMENT_DESC layout[] = {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT,    0,  0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
        { "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT,    0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
        { "COLOR",    0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 },
    };
    UINT numElements = ARRAYSIZE(layout);

    hr = g_pd3dDevice->CreateInputLayout(layout, numElements, pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), &g_pVertexLayout);
    pVSBlob->Release();
    if (FAILED(hr)) return hr;

    ID3DBlob* pPSBlob = nullptr;
    hr = D3DCompile(g_shaderCode, strlen(g_shaderCode), nullptr, nullptr, nullptr, "PS", "ps_4_0", 0, 0, &pPSBlob, &pErrorBlob);
    if (FAILED(hr)) {
        if (pErrorBlob) {
            std::cerr << "PS Compile Error: " << (char*)pErrorBlob->GetBufferPointer() << std::endl;
            pErrorBlob->Release();
        }
        return hr;
    }

    hr = g_pd3dDevice->CreatePixelShader(pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), nullptr, &g_pPixelShader);
    pPSBlob->Release();
    if (FAILED(hr)) return hr;

    // Cube Geometry (Colored corners)
    Vertex cubeVertices[] = {
        { XMFLOAT3(-0.5f,  0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f) },
        { XMFLOAT3( 0.5f,  0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f) },
        { XMFLOAT3( 0.5f, -0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 0.0f, 1.0f, 1.0f) },
        { XMFLOAT3(-0.5f, -0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(1.0f, 1.0f, 0.0f, 1.0f) },
        { XMFLOAT3(-0.5f,  0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(1.0f, 0.0f, 1.0f, 1.0f) },
        { XMFLOAT3( 0.5f,  0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 1.0f, 1.0f, 1.0f) },
        { XMFLOAT3( 0.5f, -0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f) },
        { XMFLOAT3(-0.5f, -0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 0.0f, 0.0f, 1.0f) },
    };

    WORD cubeIndices[] = {
        0, 1, 2, 0, 2, 3, // Front
        4, 6, 5, 4, 7, 6, // Back
        4, 5, 1, 4, 1, 0, // Top
        3, 2, 6, 3, 6, 7, // Bottom
        4, 0, 3, 4, 3, 7, // Left
        1, 5, 6, 1, 6, 2, // Right
    };
    g_cubeIndexCount = ARRAYSIZE(cubeIndices);

    D3D11_BUFFER_DESC bd = {};
    bd.Usage = D3D11_USAGE_DEFAULT;
    bd.ByteWidth = sizeof(cubeVertices);
    bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    
    D3D11_SUBRESOURCE_DATA InitData = {};
    InitData.pSysMem = cubeVertices;
    hr = g_pd3dDevice->CreateBuffer(&bd, &InitData, &g_pCubeVertexBuffer);
    if (FAILED(hr)) return hr;

    bd.ByteWidth = sizeof(cubeIndices);
    bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
    InitData.pSysMem = cubeIndices;
    hr = g_pd3dDevice->CreateBuffer(&bd, &InitData, &g_pCubeIndexBuffer);
    if (FAILED(hr)) return hr;

    // Grid Geometry (Cyan neon floor grid lines at Y = -1.5)
    std::vector<Vertex> gridVertices;
    float floorY = -1.5f;
    XMFLOAT4 gridColor(0.0f, 0.8f, 1.0f, 1.0f);
    
    for (float x = -5.0f; x <= 5.0f; x += 0.5f) {
        gridVertices.push_back({ XMFLOAT3(x, floorY, -5.0f), XMFLOAT3(0.0f,0.0f,0.0f), gridColor });
        gridVertices.push_back({ XMFLOAT3(x, floorY,  5.0f), XMFLOAT3(0.0f,0.0f,0.0f), gridColor });
    }
    for (float z = -5.0f; z <= 5.0f; z += 0.5f) {
        gridVertices.push_back({ XMFLOAT3(-5.0f, floorY, z), XMFLOAT3(0.0f,0.0f,0.0f), gridColor });
        gridVertices.push_back({ XMFLOAT3( 5.0f, floorY, z), XMFLOAT3(0.0f,0.0f,0.0f), gridColor });
    }
    g_gridVertexCount = (UINT)gridVertices.size();

    bd.ByteWidth = (UINT)(sizeof(Vertex) * gridVertices.size());
    bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    InitData.pSysMem = gridVertices.data();
    hr = g_pd3dDevice->CreateBuffer(&bd, &InitData, &g_pGridVertexBuffer);
    if (FAILED(hr)) return hr;

    // 3D Coordinate Axes (X = Red, Y = Green, Z = Blue)
    Vertex axesVertices[] = {
        // X
        { XMFLOAT3(-10.0f, 0.0f, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f) },
        { XMFLOAT3( 10.0f, 0.0f, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(1.0f, 0.0f, 0.0f, 1.0f) },
        // Y
        { XMFLOAT3(0.0f, -10.0f, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f) },
        { XMFLOAT3(0.0f,  10.0f, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 1.0f, 0.0f, 1.0f) },
        // Z
        { XMFLOAT3(0.0f, 0.0f, -10.0f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 0.0f, 1.0f, 1.0f) },
        { XMFLOAT3(0.0f, 0.0f,  10.0f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 0.0f, 1.0f, 1.0f) },
    };
    g_axesVertexCount = ARRAYSIZE(axesVertices);

    bd.ByteWidth = sizeof(axesVertices);
    bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    InitData.pSysMem = axesVertices;
    hr = g_pd3dDevice->CreateBuffer(&bd, &InitData, &g_pAxesVertexBuffer);
    if (FAILED(hr)) return hr;

    // Create Head Sphere (UV Sphere generation)
    std::vector<Vertex> headVertices;
    std::vector<uint32_t> headIndices;
    {
        float radius = 0.5f;
        UINT sliceCount = 16;
        UINT stackCount = 16;
        for (UINT i = 0; i <= stackCount; ++i) {
            float phi = XM_PI * i / stackCount;
            float y = radius * std::cos(phi);
            float r = radius * std::sin(phi);
            for (UINT j = 0; j <= sliceCount; ++j) {
                float theta = XM_2PI * j / sliceCount;
                float x = r * std::sin(theta);
                float z = r * std::cos(theta);
                XMFLOAT4 color(1.0f, 1.0f, 1.0f, 1.0f);
                headVertices.push_back({ XMFLOAT3(x, y, z), XMFLOAT3(0.0f,0.0f,0.0f), color });
            }
        }
        UINT ringVertexCount = sliceCount + 1;
        for (UINT i = 0; i < stackCount; ++i) {
            for (UINT j = 0; j < sliceCount; ++j) {
                headIndices.push_back(i * ringVertexCount + j);
                headIndices.push_back((i + 1) * ringVertexCount + j);
                headIndices.push_back((i + 1) * ringVertexCount + j + 1);
                
                headIndices.push_back(i * ringVertexCount + j);
                headIndices.push_back((i + 1) * ringVertexCount + j + 1);
                headIndices.push_back(i * ringVertexCount + j + 1);
            }
        }
        g_headIndexCount = static_cast<UINT>(headIndices.size());
    }

    WORD cube24Indices[] = {
        0, 1, 2, 0, 2, 3, // Front
        4, 6, 5, 4, 7, 6, // Back
        8, 9, 10, 8, 10, 11, // Top
        12, 14, 13, 12, 15, 14, // Bottom
        16, 17, 18, 16, 18, 19, // Left
        20, 22, 21, 20, 23, 22, // Right
    };

    // Visor Cube (Black body, neon blue visor face)
    Vertex visorVertices[] = {
        // Front Face (Z = 0.5) - Neon Blue/Cyan
        { XMFLOAT3(-0.5f,  0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 0.8f, 1.0f, 1.0f) },
        { XMFLOAT3( 0.5f,  0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 0.8f, 1.0f, 1.0f) },
        { XMFLOAT3( 0.5f, -0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 0.8f, 1.0f, 1.0f) },
        { XMFLOAT3(-0.5f, -0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.0f, 0.8f, 1.0f, 1.0f) },

        // Other faces - Matte Black/Dark Grey
        { XMFLOAT3(-0.5f,  0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3( 0.5f,  0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3( 0.5f, -0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3(-0.5f, -0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },

        { XMFLOAT3(-0.5f,  0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3( 0.5f,  0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3( 0.5f,  0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3(-0.5f,  0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },

        { XMFLOAT3(-0.5f, -0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3( 0.5f, -0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3( 0.5f, -0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3(-0.5f, -0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },

        { XMFLOAT3(-0.5f,  0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3(-0.5f,  0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3(-0.5f, -0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3(-0.5f, -0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },

        { XMFLOAT3( 0.5f,  0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3( 0.5f,  0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3( 0.5f, -0.5f,  0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
        { XMFLOAT3( 0.5f, -0.5f, -0.5f), XMFLOAT3(0.0f,0.0f,0.0f), XMFLOAT4(0.1f, 0.1f, 0.1f, 1.0f) },
    };

    // Create Head Buffers (using vector data)
    D3D11_BUFFER_DESC headBd = {};
    headBd.Usage = D3D11_USAGE_DEFAULT;
    headBd.ByteWidth = static_cast<UINT>(sizeof(Vertex) * headVertices.size());
    headBd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    D3D11_SUBRESOURCE_DATA headSub = {};
    headSub.pSysMem = headVertices.data();
    hr = g_pd3dDevice->CreateBuffer(&headBd, &headSub, &g_pHeadVertexBuffer);
    if (FAILED(hr)) return hr;

    headBd.ByteWidth = static_cast<UINT>(sizeof(uint32_t) * headIndices.size());
    headBd.BindFlags = D3D11_BIND_INDEX_BUFFER;
    headSub.pSysMem = headIndices.data();
    hr = g_pd3dDevice->CreateBuffer(&headBd, &headSub, &g_pHeadIndexBuffer);
    if (FAILED(hr)) return hr;

    // Create Visor Buffers
    D3D11_BUFFER_DESC visorBd = {};
    visorBd.Usage = D3D11_USAGE_DEFAULT;
    visorBd.ByteWidth = sizeof(visorVertices);
    visorBd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    D3D11_SUBRESOURCE_DATA visorSub = {};
    visorSub.pSysMem = visorVertices;
    hr = g_pd3dDevice->CreateBuffer(&visorBd, &visorSub, &g_pVisorVertexBuffer);
    if (FAILED(hr)) return hr;

    visorBd.ByteWidth = sizeof(cube24Indices);
    visorBd.BindFlags = D3D11_BIND_INDEX_BUFFER;
    visorSub.pSysMem = cube24Indices;
    hr = g_pd3dDevice->CreateBuffer(&visorBd, &visorSub, &g_pVisorIndexBuffer);
    if (FAILED(hr)) return hr;

    // Constant Buffer (WVP Matrix)
    bd.ByteWidth = sizeof(ConstantBuffer);
    bd.Usage = D3D11_USAGE_DEFAULT;
    bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    hr = g_pd3dDevice->CreateBuffer(&bd, nullptr, &g_pConstantBuffer);
    if (FAILED(hr)) return hr;

    // Rasterizer state (Wireframe grid and Solid cube)
    D3D11_RASTERIZER_DESC rd = {};
    rd.FillMode = D3D11_FILL_SOLID;
    rd.CullMode = D3D11_CULL_NONE;
    rd.DepthClipEnable = TRUE;
    hr = g_pd3dDevice->CreateRasterizerState(&rd, &g_pRasterizerState);
    if (FAILED(hr)) return hr;

    rd.CullMode = D3D11_CULL_BACK; // For solid meshes - prevents seeing back faces
    hr = g_pd3dDevice->CreateRasterizerState(&rd, &g_pRasterizerStateSolid);
    if (FAILED(hr)) return hr;

    // Depth Stencil state
    D3D11_DEPTH_STENCIL_DESC dsd = {};
    dsd.DepthEnable = TRUE;
    dsd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
    dsd.DepthFunc = D3D11_COMPARISON_LESS;
    hr = g_pd3dDevice->CreateDepthStencilState(&dsd, &g_pDepthStencilState);
    if (FAILED(hr)) return hr;

    // Load Beaver Mesh from bin
    std::string binPath;
    {
        char path[MAX_PATH];
        GetModuleFileNameA(NULL, path, MAX_PATH);
        std::string exePath(path);
        size_t lastSlash = exePath.find_last_of("\\/");
        if (lastSlash != std::string::npos) {
            binPath = exePath.substr(0, lastSlash + 1) + "funny_beaver.bin";
        } else {
            binPath = "funny_beaver.bin";
        }
    }

    std::ifstream binFile(binPath, std::ios::binary);
    if (binFile) {
        uint32_t numVertices = 0;
        uint32_t numIndices = 0;
        binFile.read(reinterpret_cast<char*>(&numVertices), sizeof(numVertices));
        binFile.read(reinterpret_cast<char*>(&numIndices), sizeof(numIndices));

        std::vector<Vertex> beaverVertices(numVertices);
        std::vector<uint32_t> beaverIndices(numIndices);
        binFile.read(reinterpret_cast<char*>(beaverVertices.data()), numVertices * sizeof(Vertex));
        binFile.read(reinterpret_cast<char*>(beaverIndices.data()), numIndices * sizeof(uint32_t));

        g_beaverIndexCount = numIndices;

        D3D11_BUFFER_DESC bdBeaver = {};
        bdBeaver.Usage = D3D11_USAGE_DEFAULT;
        bdBeaver.ByteWidth = sizeof(Vertex) * numVertices;
        bdBeaver.BindFlags = D3D11_BIND_VERTEX_BUFFER;

        D3D11_SUBRESOURCE_DATA InitDataBeaver = {};
        InitDataBeaver.pSysMem = beaverVertices.data();
        hr = g_pd3dDevice->CreateBuffer(&bdBeaver, &InitDataBeaver, &g_pBeaverVertexBuffer);
        if (FAILED(hr)) return hr;

        bdBeaver.ByteWidth = sizeof(uint32_t) * numIndices;
        bdBeaver.BindFlags = D3D11_BIND_INDEX_BUFFER;
        InitDataBeaver.pSysMem = beaverIndices.data();
        hr = g_pd3dDevice->CreateBuffer(&bdBeaver, &InitDataBeaver, &g_pBeaverIndexBuffer);
        if (FAILED(hr)) return hr;
    } else {
        std::cerr << "Failed to open funny_beaver.bin at " << binPath << std::endl;
        return E_FAIL;
    }


    // ── MSAA 4x render targets ───────────────────────────────────────────────
    UINT msaaQuality = 0;
    g_pd3dDevice->CheckMultisampleQualityLevels(DXGI_FORMAT_R8G8B8A8_UNORM, MSAA_COUNT, &msaaQuality);
    UINT msaaQ = (msaaQuality > 0) ? msaaQuality - 1 : 0;

    { // VR window MSAA color
        D3D11_TEXTURE2D_DESC td = {};
        td.Width = width; td.Height = height; td.MipLevels = 1; td.ArraySize = 1;
        td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        td.SampleDesc.Count = MSAA_COUNT; td.SampleDesc.Quality = msaaQ;
        td.Usage = D3D11_USAGE_DEFAULT; td.BindFlags = D3D11_BIND_RENDER_TARGET;
        hr = g_pd3dDevice->CreateTexture2D(&td, nullptr, &g_pMSAATarget); if (FAILED(hr)) return hr;
        hr = g_pd3dDevice->CreateRenderTargetView(g_pMSAATarget, nullptr, &g_pMSAARTV); if (FAILED(hr)) return hr;
        td.Format = DXGI_FORMAT_D24_UNORM_S8_UINT; td.BindFlags = D3D11_BIND_DEPTH_STENCIL;
        hr = g_pd3dDevice->CreateTexture2D(&td, nullptr, &g_pMSAADepth); if (FAILED(hr)) return hr;
        hr = g_pd3dDevice->CreateDepthStencilView(g_pMSAADepth, nullptr, &g_pMSAADSV); if (FAILED(hr)) return hr;
    }
    { // Overview window MSAA color
        D3D11_TEXTURE2D_DESC td = {};
        td.Width = overviewWidth; td.Height = overviewHeight; td.MipLevels = 1; td.ArraySize = 1;
        td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        td.SampleDesc.Count = MSAA_COUNT; td.SampleDesc.Quality = msaaQ;
        td.Usage = D3D11_USAGE_DEFAULT; td.BindFlags = D3D11_BIND_RENDER_TARGET;
        hr = g_pd3dDevice->CreateTexture2D(&td, nullptr, &g_pMSAATargetOV); if (FAILED(hr)) return hr;
        hr = g_pd3dDevice->CreateRenderTargetView(g_pMSAATargetOV, nullptr, &g_pMSAARTVOV); if (FAILED(hr)) return hr;
        td.Format = DXGI_FORMAT_D24_UNORM_S8_UINT; td.BindFlags = D3D11_BIND_DEPTH_STENCIL;
        hr = g_pd3dDevice->CreateTexture2D(&td, nullptr, &g_pMSAADepthOV); if (FAILED(hr)) return hr;
        hr = g_pd3dDevice->CreateDepthStencilView(g_pMSAADepthOV, nullptr, &g_pMSAADSVOV); if (FAILED(hr)) return hr;
    }

    // ── Post-process intermediate texture (MSAA-resolved scene) ─────────────
    {
        D3D11_TEXTURE2D_DESC td = {};
        td.Width = width; td.Height = height; td.MipLevels = 1; td.ArraySize = 1;
        td.Format = DXGI_FORMAT_R8G8B8A8_UNORM; td.SampleDesc.Count = 1; td.SampleDesc.Quality = 0;
        td.Usage = D3D11_USAGE_DEFAULT;
        td.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
        hr = g_pd3dDevice->CreateTexture2D(&td, nullptr, &g_pPostTex); if (FAILED(hr)) return hr;
        hr = g_pd3dDevice->CreateRenderTargetView(g_pPostTex, nullptr, &g_pPostRTV); if (FAILED(hr)) return hr;
        hr = g_pd3dDevice->CreateShaderResourceView(g_pPostTex, nullptr, &g_pPostSRV); if (FAILED(hr)) return hr;
    }
    {
        D3D11_SAMPLER_DESC sd2 = {};
        sd2.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
        sd2.AddressU = sd2.AddressV = sd2.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
        sd2.ComparisonFunc = D3D11_COMPARISON_ALWAYS; sd2.MaxLOD = D3D11_FLOAT32_MAX;
        hr = g_pd3dDevice->CreateSamplerState(&sd2, &g_pLinearSampler); if (FAILED(hr)) return hr;
    }
    // Compile post-process shaders
    {
        ID3DBlob* b = nullptr;
        hr = D3DCompile(g_postShaderCode, strlen(g_postShaderCode), nullptr, nullptr, nullptr, "VS_PP", "vs_4_0", 0, 0, &b, &pErrorBlob);
        if (FAILED(hr)) { if (pErrorBlob) { std::cerr << "PP VS: " << (char*)pErrorBlob->GetBufferPointer() << std::endl; pErrorBlob->Release(); } return hr; }
        hr = g_pd3dDevice->CreateVertexShader(b->GetBufferPointer(), b->GetBufferSize(), nullptr, &g_pPostVS); b->Release(); if (FAILED(hr)) return hr;
        hr = D3DCompile(g_postShaderCode, strlen(g_postShaderCode), nullptr, nullptr, nullptr, "PS_PP", "ps_4_0", 0, 0, &b, &pErrorBlob);
        if (FAILED(hr)) { if (pErrorBlob) { std::cerr << "PP PS: " << (char*)pErrorBlob->GetBufferPointer() << std::endl; pErrorBlob->Release(); } return hr; }
        hr = g_pd3dDevice->CreatePixelShader(b->GetBufferPointer(), b->GetBufferSize(), nullptr, &g_pPostPS); b->Release(); if (FAILED(hr)) return hr;
    }

    // ── Skybox gradient vertex buffer (6 verts, fullscreen quad in NDC) ──────
    {
        Vertex skyVerts[6] = {
            { XMFLOAT3(-1,-1,0), XMFLOAT3(), XMFLOAT4(0.04f,0.05f,0.10f,1) },
            { XMFLOAT3(-1, 1,0), XMFLOAT3(), XMFLOAT4(0.10f,0.14f,0.28f,1) },
            { XMFLOAT3( 1,-1,0), XMFLOAT3(), XMFLOAT4(0.04f,0.05f,0.10f,1) },
            { XMFLOAT3( 1,-1,0), XMFLOAT3(), XMFLOAT4(0.04f,0.05f,0.10f,1) },
            { XMFLOAT3(-1, 1,0), XMFLOAT3(), XMFLOAT4(0.10f,0.14f,0.28f,1) },
            { XMFLOAT3( 1, 1,0), XMFLOAT3(), XMFLOAT4(0.10f,0.14f,0.28f,1) },
        };
        D3D11_BUFFER_DESC sbd = {};
        sbd.Usage = D3D11_USAGE_DEFAULT;
        sbd.ByteWidth = sizeof(skyVerts);
        sbd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        D3D11_SUBRESOURCE_DATA ssd = { skyVerts, 0, 0 };
        hr = g_pd3dDevice->CreateBuffer(&sbd, &ssd, &g_pSkyboxVB);
        if (FAILED(hr)) return hr;
    }
    {
        ID3DBlob* b = nullptr;
        hr = D3DCompile(g_skyShaderCode, strlen(g_skyShaderCode), nullptr, nullptr, nullptr, "VS_Sky", "vs_4_0", 0, 0, &b, &pErrorBlob);
        if (FAILED(hr)) { if (pErrorBlob) { std::cerr << "Sky VS: " << (char*)pErrorBlob->GetBufferPointer() << std::endl; pErrorBlob->Release(); } return hr; }
        hr = g_pd3dDevice->CreateVertexShader(b->GetBufferPointer(), b->GetBufferSize(), nullptr, &g_pSkyVS); b->Release(); if (FAILED(hr)) return hr;
        hr = D3DCompile(g_skyShaderCode, strlen(g_skyShaderCode), nullptr, nullptr, nullptr, "PS_Sky", "ps_4_0", 0, 0, &b, &pErrorBlob);
        if (FAILED(hr)) { if (pErrorBlob) { std::cerr << "Sky PS: " << (char*)pErrorBlob->GetBufferPointer() << std::endl; pErrorBlob->Release(); } return hr; }
        hr = g_pd3dDevice->CreatePixelShader(b->GetBufferPointer(), b->GetBufferSize(), nullptr, &g_pSkyPS); b->Release(); if (FAILED(hr)) return hr;
    }
    // depth disabled state for sky
    {
        D3D11_DEPTH_STENCIL_DESC nd = {};
        nd.DepthEnable = FALSE;
        nd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
        nd.DepthFunc = D3D11_COMPARISON_ALWAYS;
        hr = g_pd3dDevice->CreateDepthStencilState(&nd, &g_pNoDepthState);
        if (FAILED(hr)) return hr;
    }

    // Shadow: alpha blending + depth read-only
    {
        D3D11_BLEND_DESC bdesc = {};
        bdesc.RenderTarget[0].BlendEnable = TRUE;
        bdesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
        bdesc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
        bdesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
        bdesc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
        bdesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
        bdesc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
        bdesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
        hr = g_pd3dDevice->CreateBlendState(&bdesc, &g_pShadowBlend); if (FAILED(hr)) return hr;

        D3D11_DEPTH_STENCIL_DESC dsr = {};
        dsr.DepthEnable = TRUE;
        dsr.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
        dsr.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
        hr = g_pd3dDevice->CreateDepthStencilState(&dsr, &g_pDepthReadOnly); if (FAILED(hr)) return hr;
    }


    return S_OK;
}

// Cleanup Device Resources
void CleanupDevice() {
    // New visual upgrade resources
    if (g_pDepthReadOnly)   { g_pDepthReadOnly->Release(); g_pDepthReadOnly = nullptr; }
    if (g_pShadowBlend)     { g_pShadowBlend->Release(); g_pShadowBlend = nullptr; }
    if (g_pNoDepthState)    { g_pNoDepthState->Release(); g_pNoDepthState = nullptr; }
    if (g_pSkyVS)           { g_pSkyVS->Release(); g_pSkyVS = nullptr; }
    if (g_pSkyPS)           { g_pSkyPS->Release(); g_pSkyPS = nullptr; }
    if (g_pSkyboxVB)        { g_pSkyboxVB->Release(); g_pSkyboxVB = nullptr; }
    if (g_pPostVS)          { g_pPostVS->Release(); g_pPostVS = nullptr; }
    if (g_pPostPS)          { g_pPostPS->Release(); g_pPostPS = nullptr; }
    if (g_pLinearSampler)   { g_pLinearSampler->Release(); g_pLinearSampler = nullptr; }
    if (g_pPostSRV)         { g_pPostSRV->Release(); g_pPostSRV = nullptr; }
    if (g_pPostRTV)         { g_pPostRTV->Release(); g_pPostRTV = nullptr; }
    if (g_pPostTex)         { g_pPostTex->Release(); g_pPostTex = nullptr; }
    if (g_pMSAADSVOV)       { g_pMSAADSVOV->Release(); g_pMSAADSVOV = nullptr; }
    if (g_pMSAADepthOV)     { g_pMSAADepthOV->Release(); g_pMSAADepthOV = nullptr; }
    if (g_pMSAARTVOV)       { g_pMSAARTVOV->Release(); g_pMSAARTVOV = nullptr; }
    if (g_pMSAATargetOV)    { g_pMSAATargetOV->Release(); g_pMSAATargetOV = nullptr; }
    if (g_pMSAADSV)         { g_pMSAADSV->Release(); g_pMSAADSV = nullptr; }
    if (g_pMSAADepth)       { g_pMSAADepth->Release(); g_pMSAADepth = nullptr; }
    if (g_pMSAARTV)         { g_pMSAARTV->Release(); g_pMSAARTV = nullptr; }
    if (g_pMSAATarget)      { g_pMSAATarget->Release(); g_pMSAATarget = nullptr; }
    if (g_pBeaverVertexBuffer) { g_pBeaverVertexBuffer->Release(); g_pBeaverVertexBuffer = nullptr; }
    if (g_pBeaverIndexBuffer) { g_pBeaverIndexBuffer->Release(); g_pBeaverIndexBuffer = nullptr; }
    if (g_pImmediateContext) { g_pImmediateContext->ClearState(); }
    if (g_pRenderTargetViewOverview) { g_pRenderTargetViewOverview->Release(); g_pRenderTargetViewOverview = nullptr; }
    if (g_pSwapChainOverview) { g_pSwapChainOverview->Release(); g_pSwapChainOverview = nullptr; }
    if (g_pHeadVertexBuffer) { g_pHeadVertexBuffer->Release(); g_pHeadVertexBuffer = nullptr; }
    if (g_pHeadIndexBuffer) { g_pHeadIndexBuffer->Release(); g_pHeadIndexBuffer = nullptr; }
    if (g_pVisorVertexBuffer) { g_pVisorVertexBuffer->Release(); g_pVisorVertexBuffer = nullptr; }
    if (g_pVisorIndexBuffer) { g_pVisorIndexBuffer->Release(); g_pVisorIndexBuffer = nullptr; }
    if (g_pDepthStencilBufferOverview) { g_pDepthStencilBufferOverview->Release(); g_pDepthStencilBufferOverview = nullptr; }
    if (g_pDepthStencilViewOverview) { g_pDepthStencilViewOverview->Release(); g_pDepthStencilViewOverview = nullptr; }
    if (g_pDynamicLineBuffer) { g_pDynamicLineBuffer->Release(); g_pDynamicLineBuffer = nullptr; }
    if (g_pRasterizerStateSolid) { g_pRasterizerStateSolid->Release(); g_pRasterizerStateSolid = nullptr; }
    if (g_pRasterizerState) { g_pRasterizerState->Release(); g_pRasterizerState = nullptr; }
    if (g_pDepthStencilState) { g_pDepthStencilState->Release(); g_pDepthStencilState = nullptr; }
    if (g_pConstantBuffer) { g_pConstantBuffer->Release(); g_pConstantBuffer = nullptr; }
    if (g_pCubeVertexBuffer) { g_pCubeVertexBuffer->Release(); g_pCubeVertexBuffer = nullptr; }
    if (g_pCubeIndexBuffer) { g_pCubeIndexBuffer->Release(); g_pCubeIndexBuffer = nullptr; }
    if (g_pGridVertexBuffer) { g_pGridVertexBuffer->Release(); g_pGridVertexBuffer = nullptr; }
    if (g_pAxesVertexBuffer) { g_pAxesVertexBuffer->Release(); g_pAxesVertexBuffer = nullptr; }
    if (g_pVertexLayout) { g_pVertexLayout->Release(); g_pVertexLayout = nullptr; }
    if (g_pVertexShader) { g_pVertexShader->Release(); g_pVertexShader = nullptr; }
    if (g_pPixelShader) { g_pPixelShader->Release(); g_pPixelShader = nullptr; }
    if (g_pDepthStencilView) { g_pDepthStencilView->Release(); g_pDepthStencilView = nullptr; }
    if (g_pDepthStencilBuffer) { g_pDepthStencilBuffer->Release(); g_pDepthStencilBuffer = nullptr; }
    if (g_pRenderTargetView) { g_pRenderTargetView->Release(); g_pRenderTargetView = nullptr; }
    if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
    if (g_pImmediateContext) { g_pImmediateContext->Release(); g_pImmediateContext = nullptr; }
    if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }

    // Reset non-D3D globals too!
    g_hWnd = nullptr;
    g_hWndOverview = nullptr;
    g_beaverIndexCount = 0;
    g_overviewYaw = -0.5f;
    g_overviewPitch = 0.3f;
    g_overviewDistance = 8.0f;
    g_overviewFocus = XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f);
    g_mouseLButtonDown = false;
    g_mouseRButtonDown = false;
    g_lastMouseX = 0;
    g_lastMouseY = 0;
    g_cubeIndexCount = 0;
    g_gridVertexCount = 0;
    g_axesVertexCount = 0;
    g_headIndexCount = 36;
    g_hmdQuaternion[0] = 1.0f; g_hmdQuaternion[1] = 0.0f; g_hmdQuaternion[2] = 0.0f; g_hmdQuaternion[3] = 0.0f;
    g_hmdQuaternionSmoothed[0] = 1.0f; g_hmdQuaternionSmoothed[1] = 0.0f; g_hmdQuaternionSmoothed[2] = 0.0f; g_hmdQuaternionSmoothed[3] = 0.0f;
    g_hmdQuaternionTrend[0] = 1.0f; g_hmdQuaternionTrend[1] = 0.0f; g_hmdQuaternionTrend[2] = 0.0f; g_hmdQuaternionTrend[3] = 0.0f;
    g_lastGyro[0] = 0.0f; g_lastGyro[1] = 0.0f; g_lastGyro[2] = 0.0f;
    g_recenterRefQuat[0] = 1.0f; g_recenterRefQuat[1] = 0.0f; g_recenterRefQuat[2] = 0.0f; g_recenterRefQuat[3] = 0.0f;
    g_hasHmdConnection = false;
    g_autoRecenterDone = false;
    g_cameraPos = XMVectorSet(0.0f, 0.0f, -3.0f, 1.0f);
    g_fovDegrees = 52.0f;
}


// Apply recenter reference to get head-relative orientation
static void GetRecenteredQuat(float out[4]) {
    // q_relative = q_ref_inv * q_current   (q_ref_inv = conjugate for unit quat)
    float qRef[4] = { g_recenterRefQuat[0], -g_recenterRefQuat[1], -g_recenterRefQuat[2], -g_recenterRefQuat[3] };
    float q[4];
    {
        std::lock_guard<std::mutex> lock(g_poseMutex);
        q[0]=g_hmdQuaternionSmoothed[0]; q[1]=g_hmdQuaternionSmoothed[1];
        q[2]=g_hmdQuaternionSmoothed[2]; q[3]=g_hmdQuaternionSmoothed[3];
    }
    out[0] = qRef[0]*q[0] - qRef[1]*q[1] - qRef[2]*q[2] - qRef[3]*q[3];
    out[1] = qRef[0]*q[1] + qRef[1]*q[0] + qRef[2]*q[3] - qRef[3]*q[2];
    out[2] = qRef[0]*q[2] - qRef[1]*q[3] + qRef[2]*q[0] + qRef[3]*q[1];
    out[3] = qRef[0]*q[3] + qRef[1]*q[2] - qRef[2]*q[1] + qRef[3]*q[0];
    // Normalize
    float n = out[0]*out[0]+out[1]*out[1]+out[2]*out[2]+out[3]*out[3];
    if (n > 1e-6f) { n = 1.0f / sqrtf(n); out[0]*=n; out[1]*=n; out[2]*=n; out[3]*=n; }
}

void UpdateCameraPosition(float dt) {
    XMMATRIX hmdRot;
    {
        if (g_hasHmdConnection) {
            float qRel[4];
            GetRecenteredQuat(qRel);
            hmdRot = QuaternionToMatrix(qRel);
        } else {
            hmdRot = XMMatrixIdentity();
        }
    }

    XMVECTOR forward = XMVector3TransformNormal(XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f), hmdRot);
    XMVECTOR right = XMVector3TransformNormal(XMVectorSet(1.0f, 0.0f, 0.0f, 0.0f), hmdRot);

    // Project onto horizontal XZ plane to prevent flying/sinking when looking up/down
    forward = XMVectorSetY(forward, 0.0f);
    right = XMVectorSetY(right, 0.0f);

    forward = XMVector3Normalize(forward);
    right = XMVector3Normalize(right);

    float speed = 2.0f * dt; // 2.0 units per second

    XMVECTOR moveDir = XMVectorSet(0.0f, 0.0f, 0.0f, 0.0f);

    // R key: reset position AND recenter head orientation
    static bool rWasDown = false;
    bool rIsDown = (GetAsyncKeyState('R') & 0x8000) != 0;
    if (rIsDown && !rWasDown) { // Edge-triggered to avoid repeated triggers
        g_cameraPos = XMVectorSet(0.0f, 0.0f, -3.0f, 1.0f);
        g_overviewFocus = g_cameraPos;
        // Store current HMD orientation as recenter reference
        std::lock_guard<std::mutex> lock(g_poseMutex);
        g_recenterRefQuat[0] = g_hmdQuaternionSmoothed[0];
        g_recenterRefQuat[1] = g_hmdQuaternionSmoothed[1];
        g_recenterRefQuat[2] = g_hmdQuaternionSmoothed[2];
        g_recenterRefQuat[3] = g_hmdQuaternionSmoothed[3];
        std::cout << "[Recenter] Head orientation recentered." << std::endl;
    }
    rWasDown = rIsDown;

    // Keyboard WASD controls
    if (GetAsyncKeyState('W') & 0x8000) moveDir += forward * speed;
    if (GetAsyncKeyState('S') & 0x8000) moveDir -= forward * speed;
    if (GetAsyncKeyState('D') & 0x8000) moveDir += right * speed;
    if (GetAsyncKeyState('A') & 0x8000) moveDir -= right * speed;

    // Height offset controls (Up / Down Arrow keys or E / Q keys)
    if ((GetAsyncKeyState(VK_UP) & 0x8000) || (GetAsyncKeyState('E') & 0x8000)) {
        moveDir += XMVectorSet(0.0f, speed, 0.0f, 0.0f);
    }
    if ((GetAsyncKeyState(VK_DOWN) & 0x8000) || (GetAsyncKeyState('Q') & 0x8000)) {
        moveDir -= XMVectorSet(0.0f, speed, 0.0f, 0.0f);
    }

    g_cameraPos += moveDir;

    // Adjust FOV dynamically using Page Up / Page Down keys
    if (GetAsyncKeyState(VK_PRIOR) & 0x8000) { // Page Up
        g_fovDegrees += 15.0f * dt;
    }
    if (GetAsyncKeyState(VK_NEXT) & 0x8000) { // Page Down
        g_fovDegrees -= 15.0f * dt;
    }

    // Cap FOV
    if (g_fovDegrees < 10.0f) g_fovDegrees = 10.0f;
    if (g_fovDegrees > 120.0f) g_fovDegrees = 120.0f;

    static float lastPrintedFov = -1.0f;
    if (fabs(g_fovDegrees - lastPrintedFov) > 0.5f) {
        std::cout << "[FOV] Current Field of View: " << g_fovDegrees << " degrees" << std::endl;
        lastPrintedFov = g_fovDegrees;
    }
}

// ==============================================================================
// 6. RENDERING FRAME LOOP
// ==============================================================================
// Forward declarations (defined later in file)
void GetTextLines(const std::string& text, float x, float y, float w, float h,
                  float spacing, float sw, float sh, XMFLOAT4 col, std::vector<Vertex>& out);
void AddStroke(float x1, float y1, float x2, float y2, float x, float y,
               float w, float h, float sw, float sh, XMFLOAT4 col, std::vector<Vertex>& out);

void Render(float timeSec) {

    // Use MSAA render target for VR window
    g_pImmediateContext->OMSetRenderTargets(1, &g_pMSAARTV, g_pMSAADSV);

    float clearColor[] = { 0.0f, 0.0f, 0.0f, 1.0f }; // Black = transparent in see-through AR
    g_pImmediateContext->ClearRenderTargetView(g_pMSAARTV, clearColor);
    g_pImmediateContext->ClearDepthStencilView(g_pMSAADSV, D3D11_CLEAR_DEPTH, 1.0f, 0);

    // Setup viewport
    RECT rect;
    GetClientRect(g_hWnd, &rect);
    LONG width = rect.right - rect.left;
    LONG height = rect.bottom - rect.top;

    // Viewport definitions for SBS Stereo
    D3D11_VIEWPORT vpLeft = {};
    vpLeft.Width = static_cast<float>(width) / 2.0f;
    vpLeft.Height = static_cast<float>(height);
    vpLeft.MinDepth = 0.0f;
    vpLeft.MaxDepth = 1.0f;
    vpLeft.TopLeftX = 0;
    vpLeft.TopLeftY = 0;

    D3D11_VIEWPORT vpRight = vpLeft;
    vpRight.TopLeftX = static_cast<float>(width) / 2.0f;

    // HMD Orientation Matrix (with recenter applied)
    XMMATRIX hmdRot = GetHmdRenderRotation(timeSec);
    float g_currentRelQuat[4];
    g_currentRelQuat[0]=1; g_currentRelQuat[1]=g_currentRelQuat[2]=g_currentRelQuat[3]=0;

    // World matrix (static identity, no rotation)
    XMMATRIX mWorld = XMMatrixIdentity();

    // Camera base settings
    XMVECTOR focusPos = XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f);
    XMVECTOR upVec = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);

    // Set shaders, state
    g_pImmediateContext->IASetInputLayout(g_pVertexLayout);
    g_pImmediateContext->VSSetShader(g_pVertexShader, nullptr, 0);
    g_pImmediateContext->PSSetShader(g_pPixelShader, nullptr, 0);
    g_pImmediateContext->RSSetState(g_pRasterizerState);
    g_pImmediateContext->OMSetDepthStencilState(g_pDepthStencilState, 1);

    // Render BOTH eyes
    constexpr bool kSwapSbsHalvesForXreal = true;
    for (int renderEye = 0; renderEye < 2; ++renderEye) {
        UINT stride = sizeof(Vertex);
        UINT offset = 0;

        const int outputEye = kSwapSbsHalvesForXreal ? (1 - renderEye) : renderEye;
        g_pImmediateContext->RSSetViewports(1, (outputEye == 0) ? &vpLeft : &vpRight);

        // 2. Compute View & Projection
        const float ipdOffset = (renderEye == 0) ? -0.032f : 0.032f;

        // Transform the camera coordinate system based on the head tracking rotation and camera position
        XMVECTOR localEyePos = XMVectorSet(ipdOffset, 0.0f, 0.0f, 1.0f);
        XMVECTOR targetEyePos = g_cameraPos + XMVector3Transform(localEyePos, hmdRot);
        XMVECTOR targetUpVec = XMVector3Transform(upVec, hmdRot);
        XMVECTOR targetLookVec = XMVector3Transform(XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f), hmdRot);
        
        XMMATRIX mView = XMMatrixLookAtLH(targetEyePos, targetEyePos + targetLookVec, targetUpVec);
        
        // Field of View matching XREAL glasses (~42 degrees diagonal)
        float aspect = (static_cast<float>(width) / 2.0f) / static_cast<float>(height);
        float diagHalfRad = XMConvertToRadians(g_fovDegrees) / 2.0f;
        float vertHalfRad = atanf(tanf(diagHalfRad) / sqrtf(aspect * aspect + 1.0f));
        float fovAngleY = vertHalfRad * 2.0f;
        XMMATRIX mProj = XMMatrixPerspectiveFovLH(fovAngleY, aspect, 0.1f, 100.0f);

        // Compute WVP matrix
        XMMATRIX mWVP = mWorld * mView * mProj;

        // 3. Render 3D Objects in current eye
        // Write Constant Buffer for WVP
        ConstantBuffer cb;
        cb.mvp = XMMatrixTranspose(mWVP);
        cb.world = XMMatrixIdentity();
        cb.cameraPos = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
        cb.useLighting = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
        g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cb, 0, 0);
        g_pImmediateContext->VSSetConstantBuffers(0, 1, &g_pConstantBuffer);
    g_pImmediateContext->PSSetConstantBuffers(0, 1, &g_pConstantBuffer);

        // Draw Beaver instead of Cube
        if (g_pBeaverVertexBuffer && g_pBeaverIndexBuffer) {
            XMMATRIX mWorldBeaver = BuildWorldFixedBeaverWorld(timeSec);

            g_pImmediateContext->RSSetState(g_pRasterizerStateSolid);
            g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pBeaverVertexBuffer, &stride, &offset);
            g_pImmediateContext->IASetIndexBuffer(g_pBeaverIndexBuffer, DXGI_FORMAT_R32_UINT, 0);
            g_pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            ConstantBuffer cbBeaver;
            cbBeaver.mvp = XMMatrixTranspose(mWorldBeaver * mView * mProj);
            cbBeaver.world = XMMatrixTranspose(mWorldBeaver);
            XMStoreFloat4(&cbBeaver.cameraPos, targetEyePos);
            cbBeaver.useLighting = XMFLOAT4(1.0f, 1.0f, 0.0f, 0.0f);
            g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cbBeaver, 0, 0);
            g_pImmediateContext->DrawIndexed(g_beaverIndexCount, 0, 0);
            g_pImmediateContext->RSSetState(g_pRasterizerState);
        }


    } // end eye loop

    // ── Resolve MSAA → intermediate texture, then apply lens distortion ──────
    g_pImmediateContext->ResolveSubresource(g_pPostTex, 0, g_pMSAATarget, 0, DXGI_FORMAT_R8G8B8A8_UNORM);

    // Render distortion pass to swap chain back buffer
    g_pImmediateContext->OMSetRenderTargets(1, &g_pRenderTargetView, nullptr);
    float blackClear[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    g_pImmediateContext->ClearRenderTargetView(g_pRenderTargetView, blackClear);

    RECT rcD; GetClientRect(g_hWnd, &rcD);
    D3D11_VIEWPORT vpFull = {};
    vpFull.Width  = (float)(rcD.right - rcD.left);
    vpFull.Height = (float)(rcD.bottom - rcD.top);
    vpFull.MinDepth = 0.0f; vpFull.MaxDepth = 1.0f;
    g_pImmediateContext->RSSetViewports(1, &vpFull);

    g_pImmediateContext->VSSetShader(g_pPostVS, nullptr, 0);
    g_pImmediateContext->PSSetShader(g_pPostPS, nullptr, 0);
    g_pImmediateContext->PSSetShaderResources(0, 1, &g_pPostSRV);
    g_pImmediateContext->PSSetSamplers(0, 1, &g_pLinearSampler);
    g_pImmediateContext->IASetInputLayout(nullptr);
    g_pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    g_pImmediateContext->Draw(3, 0); // fullscreen triangle

    // Restore main shader
    ID3D11ShaderResourceView* nullSRV = nullptr;
    g_pImmediateContext->PSSetShaderResources(0, 1, &nullSRV);
    g_pImmediateContext->VSSetShader(g_pVertexShader, nullptr, 0);
    g_pImmediateContext->PSSetShader(g_pPixelShader, nullptr, 0);
    g_pImmediateContext->IASetInputLayout(g_pVertexLayout);

    g_pSwapChain->Present(1, 0);
}


// Helper to draw a line segment in screen pixel coordinates
void AddStroke(float x1, float y1, float x2, float y2, float x, float y, float w, float h, float sw, float sh, XMFLOAT4 col, std::vector<Vertex>& out) {
    float px1 = ((x + x1 * w) / sw) * 2.0f - 1.0f;
    float py1 = 1.0f - ((y + y1 * h) / sh) * 2.0f;
    float px2 = ((x + x2 * w) / sw) * 2.0f - 1.0f;
    float py2 = 1.0f - ((y + y2 * h) / sh) * 2.0f;
    out.push_back({ XMFLOAT3(px1, py1, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), col });
    out.push_back({ XMFLOAT3(px2, py2, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), col });
}

// Helper to convert character strings into vector lines
void GetTextLines(const std::string& text, float x, float y, float w, float h, float spacing, float sw, float sh, XMFLOAT4 col, std::vector<Vertex>& out) {
    float curX = x;
    for (char c : text) {
        c = toupper(c);
        if (c == ' ') {
            curX += w + spacing;
            continue;
        }
        if (c == 'A') {
            AddStroke(0.0f, 1.0f, 0.5f, 0.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.5f, 0.0f, 1.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.2f, 0.6f, 0.8f, 0.6f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'E') {
            AddStroke(0.0f, 0.0f, 1.0f, 0.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 0.0f, 0.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 1.0f, 1.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 0.5f, 0.7f, 0.5f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'F') {
            AddStroke(0.0f, 0.0f, 1.0f, 0.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 0.0f, 0.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 0.5f, 0.7f, 0.5f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'G') {
            AddStroke(0.0f, 0.0f, 1.0f, 0.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 0.0f, 0.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 1.0f, 1.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(1.0f, 1.0f, 1.0f, 0.5f, curX, y, w, h, sw, sh, col, out);
            AddStroke(1.0f, 0.5f, 0.5f, 0.5f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'I') {
            AddStroke(0.5f, 0.0f, 0.5f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.2f, 0.0f, 0.8f, 0.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.2f, 1.0f, 0.8f, 1.0f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'L') {
            AddStroke(0.0f, 0.0f, 0.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 1.0f, 1.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'N') {
            AddStroke(0.0f, 0.0f, 0.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 0.0f, 1.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(1.0f, 0.0f, 1.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'O') {
            AddStroke(0.0f, 0.0f, 1.0f, 0.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 0.0f, 0.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(1.0f, 0.0f, 1.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 1.0f, 1.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'P') {
            AddStroke(0.0f, 0.0f, 0.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 0.0f, 1.0f, 0.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(1.0f, 0.0f, 1.0f, 0.5f, curX, y, w, h, sw, sh, col, out);
            AddStroke(1.0f, 0.5f, 0.0f, 0.5f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'S') {
            AddStroke(0.0f, 0.0f, 1.0f, 0.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 0.0f, 0.0f, 0.5f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.0f, 0.5f, 1.0f, 0.5f, curX, y, w, h, sw, sh, col, out);
            AddStroke(1.0f, 0.5f, 1.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(1.0f, 1.0f, 0.0f, 1.0f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'T') {
            AddStroke(0.0f, 0.0f, 1.0f, 0.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.5f, 0.0f, 0.5f, 1.0f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'V') {
            AddStroke(0.0f, 0.0f, 0.5f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.5f, 1.0f, 1.0f, 0.0f, curX, y, w, h, sw, sh, col, out);
        } else if (c == 'W') {
            AddStroke(0.0f, 0.0f, 0.2f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.2f, 1.0f, 0.5f, 0.5f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.5f, 0.5f, 0.8f, 1.0f, curX, y, w, h, sw, sh, col, out);
            AddStroke(0.8f, 1.0f, 1.0f, 0.0f, curX, y, w, h, sw, sh, col, out);
        }
        curX += w + spacing;
    }
}

void RenderOverview(float timeSec) {
    if (!g_hWndOverview || !g_pSwapChainOverview || !g_pRenderTargetViewOverview) return;
    if (!g_pMSAATargetOV || !g_pMSAARTVOV || !g_pMSAADSVOV) return;

    // Clear overview MSAA target to a dark grey background
    float clearColor[] = { 0.12f, 0.12f, 0.14f, 1.0f };
    g_pImmediateContext->ClearRenderTargetView(g_pMSAARTVOV, clearColor);
    g_pImmediateContext->ClearDepthStencilView(g_pMSAADSVOV, D3D11_CLEAR_DEPTH, 1.0f, 0);

    // Set viewport
    RECT rect;
    GetClientRect(g_hWndOverview, &rect);
    LONG width = rect.right - rect.left;
    LONG height = rect.bottom - rect.top;
    if (width == 0 || height == 0) return;

    D3D11_VIEWPORT vp = {};
    vp.Width = static_cast<float>(width);
    vp.Height = static_cast<float>(height);
    vp.MinDepth = 0.0f;
    vp.MaxDepth = 1.0f;
    g_pImmediateContext->RSSetViewports(1, &vp);

    // Robust State & Shader Binding
    g_pImmediateContext->IASetInputLayout(g_pVertexLayout);
    g_pImmediateContext->VSSetShader(g_pVertexShader, nullptr, 0);
    g_pImmediateContext->PSSetShader(g_pPixelShader, nullptr, 0);
    g_pImmediateContext->VSSetConstantBuffers(0, 1, &g_pConstantBuffer);
    g_pImmediateContext->PSSetConstantBuffers(0, 1, &g_pConstantBuffer);
    g_pImmediateContext->RSSetState(g_pRasterizerState);
    g_pImmediateContext->OMSetDepthStencilState(g_pDepthStencilState, 1);

    // Setup Render Targets using Overview-specific MSAA Targets
    g_pImmediateContext->OMSetRenderTargets(1, &g_pMSAARTVOV, g_pMSAADSVOV);

    // Orbit Camera Math
    float cosPitch = std::cos(g_overviewPitch);
    float sinPitch = std::sin(g_overviewPitch);
    float cosYaw = std::cos(g_overviewYaw);
    float sinYaw = std::sin(g_overviewYaw);

    XMVECTOR offsetVec = XMVectorSet(
        g_overviewDistance * sinYaw * cosPitch,
        g_overviewDistance * sinPitch,
        g_overviewDistance * cosYaw * cosPitch,
        0.0f
    );
    XMVECTOR overviewEye = g_overviewFocus - offsetVec;
    XMVECTOR overviewUp = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    if (sinPitch > 0.99f) overviewUp = XMVectorSet(0.0f, 0.0f, -1.0f, 0.0f);
    else if (sinPitch < -0.99f) overviewUp = XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f);

    XMMATRIX mView = XMMatrixLookAtLH(overviewEye, g_overviewFocus, overviewUp);

    float aspect = static_cast<float>(width) / static_cast<float>(height);
    XMMATRIX mProj = XMMatrixPerspectiveFovLH(XMConvertToRadians(60.0f), aspect, 0.1f, 100.0f);

    ConstantBuffer cb;
    UINT stride = sizeof(Vertex);
    UINT offset = 0;

    // 1. Draw the floor grid (Cyan grid only visible in overview)
    g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pGridVertexBuffer, &stride, &offset);
    g_pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
    XMMATRIX mGridWVP = XMMatrixIdentity() * mView * mProj;
    cb.mvp = XMMatrixTranspose(mGridWVP);
    cb.world = XMMatrixIdentity();
    cb.cameraPos = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
    cb.useLighting = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
    g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cb, 0, 0);
    g_pImmediateContext->VSSetConstantBuffers(0, 1, &g_pConstantBuffer);
    g_pImmediateContext->Draw(g_gridVertexCount, 0);

    XMMATRIX hmdRot = GetHmdRenderRotation(timeSec);

    // 3. Draw beaver at the origin instead of the grey cube
    if (g_pBeaverVertexBuffer && g_pBeaverIndexBuffer) {
        g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pBeaverVertexBuffer, &stride, &offset);
        g_pImmediateContext->IASetIndexBuffer(g_pBeaverIndexBuffer, DXGI_FORMAT_R32_UINT, 0);
        g_pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        XMMATRIX mWorldBeaver = BuildHeadLockedBeaverWorld(hmdRot, timeSec);
        XMMATRIX mWVP_Beaver = mWorldBeaver * mView * mProj;
        cb.mvp = XMMatrixTranspose(mWVP_Beaver);
        cb.world = XMMatrixTranspose(mWorldBeaver);
        XMStoreFloat4(&cb.cameraPos, overviewEye);
        cb.useLighting = XMFLOAT4(1.0f, 1.0f, 0.0f, 0.0f); // Lit beaver in overview
        g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cb, 0, 0);
        g_pImmediateContext->DrawIndexed(g_beaverIndexCount, 0, 0);
    }

    // 4. Draw User character (Head + Visor) - NOTE: Axis drawing has been removed

    // Scale, rotate, translate head to camera position
    XMMATRIX mWorldHead = XMMatrixScaling(0.3f, 0.3f, 0.3f) * hmdRot * XMMatrixTranslationFromVector(g_cameraPos);
    XMMATRIX mHeadWVP = mWorldHead * mView * mProj;
    cb.mvp = XMMatrixTranspose(mHeadWVP);
    cb.world = XMMatrixTranspose(mWorldHead);
    XMStoreFloat4(&cb.cameraPos, overviewEye);
    cb.useLighting = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f); // Vertex colour only: white sphere
    g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cb, 0, 0);

    // Draw Head block (renders as a sphere now)
    g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pHeadVertexBuffer, &stride, &offset);
    g_pImmediateContext->IASetIndexBuffer(g_pHeadIndexBuffer, DXGI_FORMAT_R32_UINT, 0);
    g_pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    g_pImmediateContext->DrawIndexed(g_headIndexCount, 0, 0);

    // Draw Visor (XREAL glasses representation) on the head
    XMMATRIX mLocalVisor = XMMatrixScaling(0.26f, 0.08f, 0.05f) * XMMatrixTranslation(0.0f, 0.04f, 0.151f);
    XMMATRIX mWorldVisor = mLocalVisor * hmdRot * XMMatrixTranslationFromVector(g_cameraPos);
    XMMATRIX mVisorWVP = mWorldVisor * mView * mProj;
    cb.mvp = XMMatrixTranspose(mVisorWVP);
    cb.world = XMMatrixTranspose(mWorldVisor);
    XMStoreFloat4(&cb.cameraPos, overviewEye);
    cb.useLighting = XMFLOAT4(1.0f, 0.0f, 0.0f, 0.0f); // Lit, matte visor (no specular)
    g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cb, 0, 0);

    g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pVisorVertexBuffer, &stride, &offset);
    g_pImmediateContext->IASetIndexBuffer(g_pVisorIndexBuffer, DXGI_FORMAT_R16_UINT, 0);
    g_pImmediateContext->DrawIndexed(36, 0, 0);

    // 5. Calculate FOV Cone (Frustum) lines dynamically
    XMVECTOR F = XMVector3Transform(XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f), hmdRot);
    XMVECTOR U_local = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    XMVECTOR U = XMVector3Transform(U_local, hmdRot);
    XMVECTOR R = XMVector3Transform(XMVectorSet(1.0f, 0.0f, 0.0f, 0.0f), hmdRot);

    float d = 1.8f; // Length of FOV frustum wireframe
    float diagHalfRad = XMConvertToRadians(g_fovDegrees) / 2.0f;
    float vertHalfRad = atanf(tanf(diagHalfRad) / sqrtf((16.0f / 9.0f) * (16.0f / 9.0f) + 1.0f));
    float h = d * std::tan(vertHalfRad);
    float w = h * (16.0f / 9.0f);

    XMVECTOR C_TL = g_cameraPos + d * F - w * R + h * U;
    XMVECTOR C_TR = g_cameraPos + d * F + w * R + h * U;
    XMVECTOR C_BL = g_cameraPos + d * F - w * R - h * U;
    XMVECTOR C_BR = g_cameraPos + d * F + w * R - h * U;

    XMFLOAT4 coneColor(1.0f, 0.8f, 0.0f, 1.0f); // Bright Gold/Yellow
    Vertex coneVertices[16];
    
    // Radial lines
    coneVertices[0] = { XMFLOAT3(XMVectorGetX(g_cameraPos), XMVectorGetY(g_cameraPos), XMVectorGetZ(g_cameraPos)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[1] = { XMFLOAT3(XMVectorGetX(C_TL), XMVectorGetY(C_TL), XMVectorGetZ(C_TL)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[2] = { XMFLOAT3(XMVectorGetX(g_cameraPos), XMVectorGetY(g_cameraPos), XMVectorGetZ(g_cameraPos)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[3] = { XMFLOAT3(XMVectorGetX(C_TR), XMVectorGetY(C_TR), XMVectorGetZ(C_TR)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[4] = { XMFLOAT3(XMVectorGetX(g_cameraPos), XMVectorGetY(g_cameraPos), XMVectorGetZ(g_cameraPos)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[5] = { XMFLOAT3(XMVectorGetX(C_BL), XMVectorGetY(C_BL), XMVectorGetZ(C_BL)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[6] = { XMFLOAT3(XMVectorGetX(g_cameraPos), XMVectorGetY(g_cameraPos), XMVectorGetZ(g_cameraPos)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[7] = { XMFLOAT3(XMVectorGetX(C_BR), XMVectorGetY(C_BR), XMVectorGetZ(C_BR)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    
    // Far plane lines
    coneVertices[8] =  { XMFLOAT3(XMVectorGetX(C_TL), XMVectorGetY(C_TL), XMVectorGetZ(C_TL)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[9] =  { XMFLOAT3(XMVectorGetX(C_TR), XMVectorGetY(C_TR), XMVectorGetZ(C_TR)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[10] = { XMFLOAT3(XMVectorGetX(C_TR), XMVectorGetY(C_TR), XMVectorGetZ(C_TR)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[11] = { XMFLOAT3(XMVectorGetX(C_BR), XMVectorGetY(C_BR), XMVectorGetZ(C_BR)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[12] = { XMFLOAT3(XMVectorGetX(C_BR), XMVectorGetY(C_BR), XMVectorGetZ(C_BR)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[13] = { XMFLOAT3(XMVectorGetX(C_BL), XMVectorGetY(C_BL), XMVectorGetZ(C_BL)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[14] = { XMFLOAT3(XMVectorGetX(C_BL), XMVectorGetY(C_BL), XMVectorGetZ(C_BL)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };
    coneVertices[15] = { XMFLOAT3(XMVectorGetX(C_TL), XMVectorGetY(C_TL), XMVectorGetZ(C_TL)), XMFLOAT3(0.0f,0.0f,0.0f), coneColor };

    // 6. Calculate PiP Border in Screen coordinates (top-left, shifted down slightly to fit label)
    int pipW = width / 4;
    int pipH = pipW * 9 / 16;
    int x_px = 20;
    int y_px = 40; // Shifted down from 20 to 40 to accommodate label

    float ndc_left = (static_cast<float>(x_px) / width) * 2.0f - 1.0f;
    float ndc_right = (static_cast<float>(x_px + pipW) / width) * 2.0f - 1.0f;
    float ndc_top = 1.0f - (static_cast<float>(y_px) / height) * 2.0f;
    float ndc_bottom = 1.0f - (static_cast<float>(y_px + pipH) / height) * 2.0f;

    XMFLOAT4 borderColor(0.0f, 0.8f, 1.0f, 1.0f); // Neon Cyan Border
    Vertex borderVertices[8];
    borderVertices[0] = { XMFLOAT3(ndc_left, ndc_top, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), borderColor };
    borderVertices[1] = { XMFLOAT3(ndc_right, ndc_top, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), borderColor };
    borderVertices[2] = { XMFLOAT3(ndc_right, ndc_top, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), borderColor };
    borderVertices[3] = { XMFLOAT3(ndc_right, ndc_bottom, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), borderColor };
    borderVertices[4] = { XMFLOAT3(ndc_right, ndc_bottom, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), borderColor };
    borderVertices[5] = { XMFLOAT3(ndc_left, ndc_bottom, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), borderColor };
    borderVertices[6] = { XMFLOAT3(ndc_left, ndc_bottom, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), borderColor };
    borderVertices[7] = { XMFLOAT3(ndc_left, ndc_top, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), borderColor };

    // Background quad for PiP to block out lines and objects behind it
    XMFLOAT4 quadColor(0.12f, 0.12f, 0.14f, 1.0f); // Match the window style
    Vertex quadVertices[6] = {
        { XMFLOAT3(ndc_left, ndc_top, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), quadColor },
        { XMFLOAT3(ndc_right, ndc_top, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), quadColor },
        { XMFLOAT3(ndc_left, ndc_bottom, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), quadColor },
        
        { XMFLOAT3(ndc_right, ndc_top, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), quadColor },
        { XMFLOAT3(ndc_right, ndc_bottom, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), quadColor },
        { XMFLOAT3(ndc_left, ndc_bottom, 0.0f), XMFLOAT3(0.0f,0.0f,0.0f), quadColor }
    };

    // Construct text vector lines dynamically
    std::vector<Vertex> lineVertices;
    for (int i = 0; i < 16; ++i) lineVertices.push_back(coneVertices[i]);
    for (int i = 0; i < 8; ++i) lineVertices.push_back(borderVertices[i]);
    for (int i = 0; i < 6; ++i) lineVertices.push_back(quadVertices[i]);

    int textStartIdx = lineVertices.size();
    XMFLOAT4 textColor(0.0f, 0.8f, 1.0f, 1.0f); // Neon Cyan text to match border
    GetTextLines("GLASSES POINT OF VIEW", 20.0f, 22.0f, 7.0f, 11.0f, 3.0f, static_cast<float>(width), static_cast<float>(height), textColor, lineVertices);
    int textCount = lineVertices.size() - textStartIdx;

    // Upload vertices to dynamic line buffer
    if (g_pDynamicLineBuffer && !lineVertices.empty()) {
        D3D11_MAPPED_SUBRESOURCE mapped;
        if (SUCCEEDED(g_pImmediateContext->Map(g_pDynamicLineBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
            size_t maxCount = lineVertices.size() < 512 ? lineVertices.size() : 512;
            size_t copySize = maxCount * sizeof(Vertex);
            memcpy(mapped.pData, lineVertices.data(), copySize);
            g_pImmediateContext->Unmap(g_pDynamicLineBuffer, 0);
        }
    }

    // 7. Render FOV Cone in Overview (World Space)
    if (g_pDynamicLineBuffer) {
        g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pDynamicLineBuffer, &stride, &offset);
        g_pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
        XMMATRIX mConeWVP = mView * mProj;
        cb.mvp = XMMatrixTranspose(mConeWVP);
        cb.world = XMMatrixIdentity();
        cb.cameraPos = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
        cb.useLighting = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
        g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cb, 0, 0);
        g_pImmediateContext->Draw(16, 0);
    }

    // 8. Render PiP Background Quad (Opaque backdrop to block floor grid/axis/objects behind it)
    if (g_pDynamicLineBuffer) {
        g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pDynamicLineBuffer, &stride, &offset);
        g_pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        cb.mvp = XMMatrixIdentity();
        cb.world = XMMatrixIdentity();
        cb.cameraPos = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
        cb.useLighting = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
        g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cb, 0, 0);
        g_pImmediateContext->Draw(6, 24);
    }

    // 9. Render PiP Viewport (Point of View from the glasses)
    D3D11_VIEWPORT vpPiP = {};
    vpPiP.Width = static_cast<float>(pipW);
    vpPiP.Height = static_cast<float>(pipH);
    vpPiP.MinDepth = 0.0f;
    vpPiP.MaxDepth = 1.0f;
    vpPiP.TopLeftX = static_cast<float>(x_px);
    vpPiP.TopLeftY = static_cast<float>(y_px);
    g_pImmediateContext->RSSetViewports(1, &vpPiP);

    // Setup view & projection for PiP view (Center view from user head position)
    XMVECTOR UPiP = U;
    XMMATRIX mViewPiP = XMMatrixLookAtLH(g_cameraPos, g_cameraPos + F, UPiP);
    float diagHalfRadPiP = XMConvertToRadians(g_fovDegrees) / 2.0f;
    float vertHalfRadPiP = atanf(tanf(diagHalfRadPiP) / sqrtf((16.0f / 9.0f) * (16.0f / 9.0f) + 1.0f));
    float fovAngleYPiP = vertHalfRadPiP * 2.0f;
    XMMATRIX mProjPiP = XMMatrixPerspectiveFovLH(fovAngleYPiP, 16.0f / 9.0f, 0.1f, 100.0f);

    // Clear depth stencil for PiP rendering
    g_pImmediateContext->ClearDepthStencilView(g_pMSAADSVOV, D3D11_CLEAR_DEPTH, 1.0f, 0);

    // Render 3D scene (beaver) in the PiP viewport
    if (g_pBeaverVertexBuffer && g_pBeaverIndexBuffer) {
        g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pBeaverVertexBuffer, &stride, &offset);
        g_pImmediateContext->IASetIndexBuffer(g_pBeaverIndexBuffer, DXGI_FORMAT_R32_UINT, 0);
        g_pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        XMMATRIX mWorldBeaver = BuildHeadLockedBeaverWorld(hmdRot, timeSec);
        XMMATRIX mWVP_PiP = mWorldBeaver * mViewPiP * mProjPiP;
        cb.mvp = XMMatrixTranspose(mWVP_PiP);
        cb.world = XMMatrixTranspose(mWorldBeaver);
        XMStoreFloat4(&cb.cameraPos, g_cameraPos);
        cb.useLighting = XMFLOAT4(1.0f, 1.0f, 0.0f, 0.0f); // Lit beaver in PiP
        g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cb, 0, 0);
        g_pImmediateContext->DrawIndexed(g_beaverIndexCount, 0, 0);
    }

    // 9. Render PiP Border & Label (2D Overlay)
    g_pImmediateContext->RSSetViewports(1, &vp); // Restore whole window viewport
    if (g_pDynamicLineBuffer) {
        g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pDynamicLineBuffer, &stride, &offset);
        g_pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
        cb.mvp = XMMatrixIdentity(); // Vertices are already in NDC
        cb.world = XMMatrixIdentity();
        cb.cameraPos = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
        cb.useLighting = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);
        g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, nullptr, &cb, 0, 0);
        
        // Draw Border (8 vertices starting at index 16)
        g_pImmediateContext->Draw(8, 16);
        
        // Draw Text Label (textCount vertices starting at textStartIdx)
        if (textCount > 0) {
            g_pImmediateContext->Draw(textCount, textStartIdx);
        }
    }

    // (shadow removed)

    // ── Resolve MSAA → swap chain back buffer ─────────────────────────────
    ID3D11Texture2D* pOVBack = nullptr;
    g_pSwapChainOverview->GetBuffer(0, IID_PPV_ARGS(&pOVBack));
    if (pOVBack) {
        g_pImmediateContext->ResolveSubresource(pOVBack, 0, g_pMSAATargetOV, 0, DXGI_FORMAT_R8G8B8A8_UNORM);
        pOVBack->Release();
    }
    // Overview Perf HUD overlay removed

    g_pSwapChainOverview->Present(1, 0);
}

// ==============================================================================
// 7. WIN32 WINDOW SYSTEM CALLBACK
// ==============================================================================
static LRESULT CALLBACK VRWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {
    PAINTSTRUCT ps;
    HDC hdc;

    // Mouse Controls inside Overview Window
    if (hWnd == g_hWndOverview) {
        switch (message) {
            case WM_LBUTTONDOWN:
                g_mouseLButtonDown = true;
                g_lastMouseX = (int)(short)LOWORD(lParam);
                g_lastMouseY = (int)(short)HIWORD(lParam);
                SetCapture(hWnd);
                return 0;

            case WM_LBUTTONUP:
                g_mouseLButtonDown = false;
                ReleaseCapture();
                return 0;

            case WM_RBUTTONDOWN:
                g_mouseRButtonDown = true;
                g_lastMouseX = (int)(short)LOWORD(lParam);
                g_lastMouseY = (int)(short)HIWORD(lParam);
                SetCapture(hWnd);
                return 0;

            case WM_RBUTTONUP:
                g_mouseRButtonDown = false;
                ReleaseCapture();
                return 0;

            case WM_MOUSEMOVE: {
                int x = (int)(short)LOWORD(lParam);
                int y = (int)(short)HIWORD(lParam);
                int dx = x - g_lastMouseX;
                int dy = y - g_lastMouseY;
                g_lastMouseX = x;
                g_lastMouseY = y;

                if (g_mouseLButtonDown) {
                    g_overviewYaw -= dx * 0.005f; // Orbit drag
                    g_overviewPitch += dy * 0.005f;
                    if (g_overviewPitch > 1.4f) g_overviewPitch = 1.4f;
                    if (g_overviewPitch < -1.4f) g_overviewPitch = -1.4f;
                }
                else if (g_mouseRButtonDown) {
                    // Pan focus target relative to camera angle and move glasses viewpoint with it
                    XMVECTOR camRight = XMVectorSet(std::cos(g_overviewYaw), 0.0f, -std::sin(g_overviewYaw), 0.0f);
                    XMVECTOR camUp = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
                    XMVECTOR deltaMove = camRight * (-dx * 0.001f * g_overviewDistance) + camUp * (dy * 0.001f * g_overviewDistance);
                    g_overviewFocus += deltaMove;
                    g_cameraPos += deltaMove;
                }
                return 0;
            }

            case WM_MOUSEWHEEL: {
                int delta = GET_WHEEL_DELTA_WPARAM(wParam);
                g_overviewDistance -= delta * 0.003f;
                if (g_overviewDistance < 1.5f) g_overviewDistance = 1.5f;
                if (g_overviewDistance > 30.0f) g_overviewDistance = 30.0f;

                const float zoomStep = delta * 0.003f;
                const float cosPitch = std::cos(g_overviewPitch);
                const float sinPitch = std::sin(g_overviewPitch);
                const float cosYaw = std::cos(g_overviewYaw);
                const float sinYaw = std::sin(g_overviewYaw);
                XMVECTOR moveForward = XMVector3Normalize(XMVectorSet(sinYaw * cosPitch, sinPitch, cosYaw * cosPitch, 0.0f));
                g_cameraPos += moveForward * zoomStep;
                g_overviewFocus += moveForward * zoomStep;
                return 0;
            }

            case WM_SIZE: {
                if (g_pSwapChainOverview && wParam != SIZE_MINIMIZED) {
                    UINT nW = LOWORD(lParam), nH = HIWORD(lParam);
                    if (nW > 0 && nH > 0) {
                        // Release all overview-specific resources
                        if (g_pRenderTargetViewOverview)  { g_pRenderTargetViewOverview->Release();  g_pRenderTargetViewOverview = nullptr; }
                        if (g_pMSAARTVOV)                 { g_pMSAARTVOV->Release();                 g_pMSAARTVOV = nullptr; }
                        if (g_pMSAATargetOV)               { g_pMSAATargetOV->Release();               g_pMSAATargetOV = nullptr; }
                        if (g_pMSAADSVOV)                  { g_pMSAADSVOV->Release();                  g_pMSAADSVOV = nullptr; }
                        if (g_pMSAADepthOV)                { g_pMSAADepthOV->Release();                g_pMSAADepthOV = nullptr; }
                        if (g_pDepthStencilViewOverview)   { g_pDepthStencilViewOverview->Release();   g_pDepthStencilViewOverview = nullptr; }
                        if (g_pDepthStencilBufferOverview) { g_pDepthStencilBufferOverview->Release(); g_pDepthStencilBufferOverview = nullptr; }
                        
                        g_pSwapChainOverview->ResizeBuffers(0, nW, nH, DXGI_FORMAT_UNKNOWN, 0);
                        
                        // Recreate back-buffer RTV
                        { ID3D11Texture2D* pB = nullptr; g_pSwapChainOverview->GetBuffer(0, IID_PPV_ARGS(&pB));
                          if (pB) { g_pd3dDevice->CreateRenderTargetView(pB, nullptr, &g_pRenderTargetViewOverview); pB->Release(); } }
                        
                        // Recreate MSAA colour + depth
                        D3D11_TEXTURE2D_DESC td = {};
                        td.Width=nW; td.Height=nH; td.MipLevels=1; td.ArraySize=1;
                        td.SampleDesc.Count=4; td.Usage=D3D11_USAGE_DEFAULT;
                        td.Format=DXGI_FORMAT_R8G8B8A8_UNORM; td.BindFlags=D3D11_BIND_RENDER_TARGET;
                        if (SUCCEEDED(g_pd3dDevice->CreateTexture2D(&td, nullptr, &g_pMSAATargetOV)))
                            g_pd3dDevice->CreateRenderTargetView(g_pMSAATargetOV, nullptr, &g_pMSAARTVOV);
                        
                        td.Format=DXGI_FORMAT_D24_UNORM_S8_UINT; td.BindFlags=D3D11_BIND_DEPTH_STENCIL;
                        if (SUCCEEDED(g_pd3dDevice->CreateTexture2D(&td, nullptr, &g_pMSAADepthOV)))
                            g_pd3dDevice->CreateDepthStencilView(g_pMSAADepthOV, nullptr, &g_pMSAADSVOV);
                            
                        // Recreate non-MSAA depth stencil overview resources
                        td.SampleDesc.Count=1; td.SampleDesc.Quality=0;
                        if (SUCCEEDED(g_pd3dDevice->CreateTexture2D(&td, nullptr, &g_pDepthStencilBufferOverview)))
                            g_pd3dDevice->CreateDepthStencilView(g_pDepthStencilBufferOverview, nullptr, &g_pDepthStencilViewOverview);
                    }
                }
                return 0;
            }
        }
    }

    switch (message) {
        case WM_PAINT:
            hdc = BeginPaint(hWnd, &ps);
            EndPaint(hWnd, &ps);
            break;

        case WM_DESTROY:
            PostQuitMessage(0);
            break;

        case WM_KEYDOWN:
            if (wParam == VK_ESCAPE) {
                DestroyWindow(hWnd);
            }
            break;

        default:
            return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// ==============================================================================
// 8. ENTRY POINT (MAIN)
// ==============================================================================
int run_vr_test_scene(int argc, char* argv[]) {
    std::cout << "==================================================================" << std::endl;
    std::cout << "       XREAL Air 2 Ultra - STEREOSCOPIC 3D VR DEMO SCENE          " << std::endl;
    std::cout << "==================================================================" << std::endl;
    // Detect monitor layout
    std::vector<MonitorInfo> monitors;
    EnumDisplayMonitors(nullptr, nullptr, MonitorEnumProc, reinterpret_cast<LPARAM>(&monitors));

    bool foundXReal = false;
    RECT xrealRect = {};

    for (const auto& mon : monitors) {
        if (mon.is_xreal) {
            xrealRect = mon.rect;
            foundXReal = true;
            break;
        }
    }

    if (!foundXReal) {
        std::cout << "[Monitor] XREAL 3D display not detected. Automatically switching glasses to 3D SBS mode..." << std::endl;
        if (set_glasses_display_mode(3)) {            std::cout << "[Monitor] Mode switch command sent. Polling for Windows display reconfiguration (up to 10 seconds)..." << std::endl;
            auto startTime = std::chrono::steady_clock::now();
            while (std::chrono::duration<float>(std::chrono::steady_clock::now() - startTime).count() < 10.0f) {
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                monitors.clear();
                EnumDisplayMonitors(nullptr, nullptr, MonitorEnumProc, reinterpret_cast<LPARAM>(&monitors));
                for (const auto& mon : monitors) {
                    if (mon.is_xreal) {
                        xrealRect = mon.rect;
                        foundXReal = true;
                        break;
                    }
                }
                if (foundXReal) {
                    std::cout << "[Monitor] Windows reconfigured. XREAL 3D display detected!" << std::endl;
                    break;
                }
            }
        }
    }

    int windowX = 100;
    int windowY = 100;
    int windowWidth = 1920; // Default window size
    int windowHeight = 540;  // 3840x1080 halfed for primary monitor windowed testing

    if (foundXReal) {
        std::cout << "[Monitor] XREAL 3D display detected at coordinates: ("
                  << xrealRect.left << ", " << xrealRect.top << ") to ("
                  << xrealRect.right << ", " << xrealRect.bottom << ")" << std::endl;
        windowX = xrealRect.left;
        windowY = xrealRect.top;
        windowWidth = 3840;
        windowHeight = 1080;
    } else {
        std::cout << "[Monitor] XREAL 3D display not found after timeout. Defaulting to a windowed preview on primary monitor." << std::endl;
    }

    // Register Win32 Class
    WNDCLASSEX wcex = {};
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = VRWndProc;
    wcex.hInstance = GetModuleHandle(nullptr);
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszClassName = "XREALVRDemoClass";
    if (!RegisterClassEx(&wcex) && GetLastError() != ERROR_CLASS_ALREADY_EXISTS) {
        std::cerr << "Failed to register window class." << std::endl;
        return 1;
    }

    // Create Window (WS_POPUP for borderless presentation on glasses)
    DWORD style = WS_OVERLAPPEDWINDOW;
    if (foundXReal) {
        style = WS_POPUP | WS_VISIBLE; // borderless full screen on HMD
    }

    RECT rc = { 0, 0, windowWidth, windowHeight };
    AdjustWindowRect(&rc, style, FALSE);

    g_hWnd = CreateWindow(
        "XREALVRDemoClass", "XREAL 3D VR Test Scene", style,
        windowX, windowY, rc.right - rc.left, rc.bottom - rc.top,
        nullptr, nullptr, GetModuleHandle(nullptr), nullptr
    );

    if (!g_hWnd) {
        std::cerr << "Failed to create window." << std::endl;
        return 1;
    }

    ShowWindow(g_hWnd, SW_SHOWNORMAL);
    UpdateWindow(g_hWnd);

    // Initialize D3D11
    if (FAILED(InitDevice(windowWidth, windowHeight, 1280, 720))) {
        std::cerr << "Direct3D 11 initialization failed." << std::endl;
        CleanupDevice();
        return 1;
    }
    std::cout << "[D3D11] Graphics Pipeline initialized successfully." << std::endl;

    // Load Driver & Stream Pose
    std::string local_cal_path = "calibration.json";
    {
        char exePath[MAX_PATH] = {};
        if (GetModuleFileNameA(nullptr, exePath, MAX_PATH)) {
            std::filesystem::path p(exePath);
            local_cal_path = (p.parent_path() / "calibration.json").string();
        }
    }

    std::cout << "[Driver] Attempting to connect to HMD driver..." << std::endl;
    bool init_ok = g_driver.initialize("");
    if (!g_driver.is_calibration_from_device()) {
        std::cout << "[Driver] On-device calibration download unavailable. Falling back to local file..." << std::endl;
        init_ok = g_driver.initialize(local_cal_path);
    }
    if (init_ok) {
        auto callback = [](const xreal::Telemetry& telemetry) {
            std::lock_guard<std::mutex> lock(g_poseMutex);

            // Store raw quaternion
            g_hmdQuaternion[0] = telemetry.quaternion[0];
            g_hmdQuaternion[1] = telemetry.quaternion[1];
            g_hmdQuaternion[2] = telemetry.quaternion[2];
            g_hmdQuaternion[3] = telemetry.quaternion[3];

            if (!g_autoRecenterDone) {
                g_recenterRefQuat[0] = telemetry.quaternion[0];
                g_recenterRefQuat[1] = telemetry.quaternion[1];
                g_recenterRefQuat[2] = telemetry.quaternion[2];
                g_recenterRefQuat[3] = telemetry.quaternion[3];
                g_autoRecenterDone = true;
                std::cout << "[Recenter] Initial head orientation captured." << std::endl;
            }

            // Store latest calibrated gyro for predictive compensation (Feature 4)
            g_lastGyro[0] = telemetry.gyro[0];
            g_lastGyro[1] = telemetry.gyro[1];
            g_lastGyro[2] = telemetry.gyro[2];

            const float* r = g_hmdQuaternion;
            float* s = g_hmdQuaternionSmoothed;
            float* t2 = g_hmdQuaternionTrend;

            // --- Feature 1: ADAPTIVE alpha based on angular velocity ---
            // Fast head turn -> higher alpha (more responsive)
            // Head still    -> lower alpha  (much smoother, kills jitter)
            float gyro_mag = std::sqrt(telemetry.gyro[0]*telemetry.gyro[0] +
                                       telemetry.gyro[1]*telemetry.gyro[1] +
                                       telemetry.gyro[2]*telemetry.gyro[2]);
            float t_remap = gyro_mag / 3.0f; // saturates at 3 rad/s (~170 deg/s)
            if (t_remap > 1.0f) t_remap = 1.0f;
            float alpha = 0.05f + 0.40f * t_remap; // range [0.05 .. 0.45]

            // Shortest-path: ensure we interpolate the short way around
            float dot = s[0]*r[0] + s[1]*r[1] + s[2]*r[2] + s[3]*r[3];
            float fsign = (dot < 0.0f) ? -1.0f : 1.0f;
            float r_adj[4] = { fsign*r[0], fsign*r[1], fsign*r[2], fsign*r[3] };

            // --- Feature 3: DOUBLE-EXPONENTIAL smoothing ---
            // First EMA (s): tracks the signal
            for (int i = 0; i < 4; i++) s[i] = s[i] + alpha * (r_adj[i] - s[i]);
            float norm = std::sqrt(s[0]*s[0]+s[1]*s[1]+s[2]*s[2]+s[3]*s[3]);
            if (norm > 1e-6f) { s[0]/=norm; s[1]/=norm; s[2]/=norm; s[3]/=norm; }

            // Second EMA (t2): tracks the trend (follows s)
            float dot2 = t2[0]*s[0]+t2[1]*s[1]+t2[2]*s[2]+t2[3]*s[3];
            float fsign2 = (dot2 < 0.0f) ? -1.0f : 1.0f;
            for (int i = 0; i < 4; i++) t2[i] = t2[i] + alpha * (fsign2*s[i] - t2[i]);
            norm = std::sqrt(t2[0]*t2[0]+t2[1]*t2[1]+t2[2]*t2[2]+t2[3]*t2[3]);
            if (norm > 1e-6f) { t2[0]/=norm; t2[1]/=norm; t2[2]/=norm; t2[3]/=norm; }

            // Double-exp forecast: 2*S - T (removes lag introduced by smoothing)
            float forecast[4];
            for (int i = 0; i < 4; i++) forecast[i] = 2.0f * s[i] - t2[i];
            norm = std::sqrt(forecast[0]*forecast[0]+forecast[1]*forecast[1]+
                             forecast[2]*forecast[2]+forecast[3]*forecast[3]);
            if (norm > 1e-6f) { for (int i=0;i<4;i++) forecast[i]/=norm; }

            // Store final smoothed result for rendering
            g_hmdQuaternionSmoothed[0] = forecast[0];
            g_hmdQuaternionSmoothed[1] = forecast[1];
            g_hmdQuaternionSmoothed[2] = forecast[2];
            g_hmdQuaternionSmoothed[3] = forecast[3];
        };

        if (g_driver.start_streaming("Mahony", callback)) {
            g_hasHmdConnection = true;
            std::cout << "[Driver] HMD connection established. Live head tracking active." << std::endl;
            std::cout << "[Driver] Automatically switching HMD display mode to 3D SBS." << std::endl;
            g_driver.set_display_mode(3);
        } else {
            std::cout << "[Driver] Failed to start streaming. Fallback to Mock Animation." << std::endl;
        }
    } else {
        std::cout << "[Driver] HMD device not detected or calibration missing. Fallback to Mock Animation." << std::endl;
    }

    if (!g_hasHmdConnection) {
        std::cout << "[VR Scene] Controls: Move mouse or watch sinusoidal mock tracking move camera." << std::endl;
        std::cout << "[VR Scene] Press ESC to close." << std::endl;
    }

    // Message/Render Loop
    auto startTime = std::chrono::high_resolution_clock::now();
    auto lastFrameTime = std::chrono::high_resolution_clock::now();
    MSG msg = {};
    while (WM_QUIT != msg.message) {
        if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        } else {
            auto nowTime = std::chrono::high_resolution_clock::now();
            float dt = std::chrono::duration<float>(nowTime - lastFrameTime).count();
            lastFrameTime = nowTime;

            if (dt > 0.1f) dt = 0.1f; // Clamp to avoid huge jumps on frame drops

            UpdateCameraPosition(dt);

            float timeSec = std::chrono::duration<float>(nowTime - startTime).count();
            Render(timeSec);
        }
    }

    // Clean up
    std::cout << "[Cleanup] Terminating telemetry stream..." << std::endl;
    if (g_hasHmdConnection) {
        std::cout << "[Cleanup] Restoring HMD display mode to 2D." << std::endl;
        g_driver.set_display_mode(1);
        g_driver.stop_streaming();
    }
    CleanupDevice();
    UnregisterClass("XREALVRDemoClass", GetModuleHandle(nullptr));
    
    std::cout << "[Cleanup] Exited." << std::endl;

    return 0;
}

#if defined(VR_TEST_SCENE_STANDALONE)
int main(int argc, char* argv[]) {
    return run_vr_test_scene(argc, argv);
}
#endif
