#pragma once
#include <cmath>

#if defined(_MSC_VER)
#define XREAL_FORCEINLINE __forceinline
#else
#define XREAL_FORCEINLINE inline __attribute__((always_inline))
#endif

#if defined(__ARM_NEON) || defined(__aarch64__)
#include <arm_neon.h>
#define SIMD_NEON 1
#elif defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <xmmintrin.h>
#include <emmintrin.h>
#define SIMD_SSE 1
#endif

namespace xreal {

struct alignas(16) float4 {
#if SIMD_NEON
    float32x4_t val;
#elif SIMD_SSE
    __m128 val;
#else
    float val[4];
#endif

    // Constructors
    XREAL_FORCEINLINE float4() {
#if SIMD_NEON
        val = vdupq_n_f32(0.0f);
#elif SIMD_SSE
        val = _mm_setzero_ps();
#else
        val[0] = val[1] = val[2] = val[3] = 0.0f;
#endif
    }

    XREAL_FORCEINLINE float4(float x, float y, float z, float w = 0.0f) {
#if SIMD_NEON
        float32x4_t v = vdupq_n_f32(x);
        v = vsetq_lane_f32(y, v, 1);
        v = vsetq_lane_f32(z, v, 2);
        val = vsetq_lane_f32(w, v, 3);
#elif SIMD_SSE
        val = _mm_set_ps(w, z, y, x);
#else
        val[0] = x; val[1] = y; val[2] = z; val[3] = w;
#endif
    }

    // Specialized fast constructor to load 3 floats from memory
    XREAL_FORCEINLINE float4(const float* ptr) {
#if SIMD_NEON
        float32x2_t low = vld1_f32(ptr);
        float32x1_t high = vld1_f32(ptr + 2);
        val = vcombine_f32(low, vset_lane_f32(0.0f, high, 1));
#elif SIMD_SSE
        __m128 low = _mm_castpd_ps(_mm_load_sd(reinterpret_cast<const double*>(ptr)));
        __m128 high = _mm_load_ss(ptr + 2);
        val = _mm_movelh_ps(low, high);
#else
        val[0] = ptr[0]; val[1] = ptr[1]; val[2] = ptr[2]; val[3] = 0.0f;
#endif
    }

#if SIMD_NEON
    XREAL_FORCEINLINE float4(float32x4_t v) : val(v) {}
#elif SIMD_SSE
    XREAL_FORCEINLINE float4(__m128 v) : val(v) {}
#endif

    XREAL_FORCEINLINE void store(float* ptr) const {
#if SIMD_NEON
        vst1q_f32(ptr, val);
#elif SIMD_SSE
        _mm_store_ps(ptr, val);
#else
        ptr[0] = val[0]; ptr[1] = val[1]; ptr[2] = val[2]; ptr[3] = val[3];
#endif
    }

    // Specialized fast store to write 3 floats to memory
    XREAL_FORCEINLINE void store3(float* ptr) const {
#if SIMD_NEON
        vst1_f32(ptr, vget_low_f32(val));
        vst1_lane_f32(ptr + 2, vget_high_f32(val), 0);
#elif SIMD_SSE
        _mm_storel_pi(reinterpret_cast<__m64*>(ptr), val);
        _mm_store_ss(ptr + 2, _mm_shuffle_ps(val, val, _MM_SHUFFLE(2, 2, 2, 2)));
#else
        ptr[0] = val[0]; ptr[1] = val[1]; ptr[2] = val[2];
#endif
    }

    XREAL_FORCEINLINE float x() const {
#if SIMD_NEON
        return vgetq_lane_f32(val, 0);
#elif SIMD_SSE
        return _mm_cvtss_f32(val);
#else
        return val[0];
#endif
    }

    XREAL_FORCEINLINE float y() const {
#if SIMD_NEON
        return vgetq_lane_f32(val, 1);
#elif SIMD_SSE
        return _mm_cvtss_f32(_mm_shuffle_ps(val, val, _MM_SHUFFLE(1, 1, 1, 1)));
#else
        return val[1];
#endif
    }

    XREAL_FORCEINLINE float z() const {
#if SIMD_NEON
        return vgetq_lane_f32(val, 2);
#elif SIMD_SSE
        return _mm_cvtss_f32(_mm_shuffle_ps(val, val, _MM_SHUFFLE(2, 2, 2, 2)));
#else
        return val[2];
#endif
    }

    XREAL_FORCEINLINE float w() const {
#if SIMD_NEON
        return vgetq_lane_f32(val, 3);
#elif SIMD_SSE
        return _mm_cvtss_f32(_mm_shuffle_ps(val, val, _MM_SHUFFLE(3, 3, 3, 3)));
#else
        return val[3];
#endif
    }
};

XREAL_FORCEINLINE float4 operator+(const float4& a, const float4& b) {
#if SIMD_NEON
    return float4(vaddq_f32(a.val, b.val));
#elif SIMD_SSE
    return float4(_mm_add_ps(a.val, b.val));
#else
    return float4(a.val[0] + b.val[0], a.val[1] + b.val[1], a.val[2] + b.val[2], a.val[3] + b.val[3]);
#endif
}

XREAL_FORCEINLINE float4 operator-(const float4& a, const float4& b) {
#if SIMD_NEON
    return float4(vsubq_f32(a.val, b.val));
#elif SIMD_SSE
    return float4(_mm_sub_ps(a.val, b.val));
#else
    return float4(a.val[0] - b.val[0], a.val[1] - b.val[1], a.val[2] - b.val[2], a.val[3] - b.val[3]);
#endif
}

XREAL_FORCEINLINE float4 operator*(const float4& a, const float4& b) {
#if SIMD_NEON
    return float4(vmulq_f32(a.val, b.val));
#elif SIMD_SSE
    return float4(_mm_mul_ps(a.val, b.val));
#else
    return float4(a.val[0] * b.val[0], a.val[1] * b.val[1], a.val[2] * b.val[2], a.val[3] * b.val[3]);
#endif
}

XREAL_FORCEINLINE float4 operator*(const float4& a, float scale) {
#if SIMD_NEON
    return float4(vmulq_n_f32(a.val, scale));
#elif SIMD_SSE
    return float4(_mm_mul_ps(a.val, _mm_set1_ps(scale)));
#else
    return float4(a.val[0] * scale, a.val[1] * scale, a.val[2] * scale, a.val[3] * scale);
#endif
}

XREAL_FORCEINLINE float4 cross_product(const float4& a, const float4& b) {
#if SIMD_NEON
    float32x4_t a1 = vextq_f32(a.val, a.val, 1);
    float32x4_t b1 = vextq_f32(b.val, b.val, 2);
    float32x4_t a2 = vextq_f32(a.val, a.val, 2);
    float32x4_t b2 = vextq_f32(b.val, b.val, 1);
    float32x4_t res = vsubq_f32(vmulq_f32(a1, b1), vmulq_f32(a2, b2));
    return float4(vsetq_lane_f32(0.0f, res, 3));
#elif SIMD_SSE
    __m128 a1 = _mm_shuffle_ps(a.val, a.val, _MM_SHUFFLE(3, 0, 2, 1));
    __m128 b1 = _mm_shuffle_ps(b.val, b.val, _MM_SHUFFLE(3, 1, 0, 2));
    __m128 a2 = _mm_shuffle_ps(a.val, a.val, _MM_SHUFFLE(3, 1, 0, 2));
    __m128 b2 = _mm_shuffle_ps(b.val, b.val, _MM_SHUFFLE(3, 0, 2, 1));
    return float4(_mm_sub_ps(_mm_mul_ps(a1, b1), _mm_mul_ps(a2, b2)));
#else
    return float4(
        a.val[1] * b.val[2] - a.val[2] * b.val[1],
        a.val[2] * b.val[0] - a.val[0] * b.val[2],
        a.val[0] * b.val[1] - a.val[1] * b.val[0],
        0.0f
    );
#endif
}

XREAL_FORCEINLINE float4 rotate_vector_by_quaternion_simd(const float4& q, const float4& v) {
    float qw = q.w();
    float4 t = cross_product(q, v) * 2.0f;
    return v + (t * qw) + cross_product(q, t);
}

XREAL_FORCEINLINE float4 pre_bias_swap(const float4& v) {
#if SIMD_NEON
    float32x4_t neg = vnegq_f32(v.val);
    float32x4_t tmp1 = vsetq_lane_f32(vgetq_lane_f32(neg, 1), neg, 2);
    float32x4_t tmp2 = vsetq_lane_f32(vgetq_lane_f32(neg, 2), tmp1, 1);
    return float4(vsetq_lane_f32(0.0f, tmp2, 3));
#elif SIMD_SSE
    __m128 neg = _mm_sub_ps(_mm_setzero_ps(), v.val);
    return float4(_mm_shuffle_ps(neg, neg, _MM_SHUFFLE(3, 1, 2, 0)));
#else
    return float4(-v.val[0], -v.val[2], -v.val[1], 0.0f);
#endif
}

XREAL_FORCEINLINE float4 post_bias_swap(const float4& v) {
#if SIMD_NEON
    alignas(16) static const float mul_data[4] = {1.0f, -1.0f, -1.0f, 0.0f};
    return float4(vmulq_f32(v.val, vld1q_f32(mul_data)));
#elif SIMD_SSE
    alignas(16) static const float mul_data[4] = {1.0f, -1.0f, -1.0f, 0.0f};
    return float4(_mm_mul_ps(v.val, _mm_load_ps(mul_data)));
#else
    return float4(v.val[0], -v.val[1], -v.val[2], 0.0f);
#endif
}

} // namespace xreal
