#pragma once

#include "CircularBuffer.h"
#include <cmath>

template<size_t Size>
class CombFilter {
public:
    void init(int delaySamples, float feedback, float damping) {
        delaySamples_ = delaySamples;
        feedback_ = feedback;
        damping_ = damping;
        filterState_ = 0.0f;
        buffer_.reset();
    }

    [[nodiscard]] float process(float input) {
        float delayed = buffer_.read(delaySamples_);
        // One-pole lowpass on feedback path for damping
        filterState_ = delayed * (1.0f - damping_) + filterState_ * damping_;
        buffer_.write(input + filterState_ * feedback_);
        return delayed;
    }

    void setFeedback(float feedback) {
        feedback_ = feedback;
    }

    void setDamping(float damping) {
        damping_ = damping;
    }

    void reset() {
        buffer_.reset();
        filterState_ = 0.0f;
    }

private:
    CircularBuffer<float, Size> buffer_;
    int delaySamples_ = 0;
    float feedback_ = 0.0f;
    float damping_ = 0.0f;
    float filterState_ = 0.0f;
};

namespace fdn {

inline void hadamard4(float& a, float& b, float& c, float& d) {
    float w = a + b;
    float x = a - b;
    float y = c + d;
    float z = c - d;
    a = (w + y) * 0.5f;
    b = (x + z) * 0.5f;
    c = (w - y) * 0.5f;
    d = (x - z) * 0.5f;
}

// Normalization scale for the 8-point Hadamard butterfly: 1/sqrt(8)
constexpr float kHadamard8Scale = 0.353553391f;

/// 8-point normalized Hadamard mixing matrix. In-place on float[8].
///
/// Implements H8 = (1/sqrt(8)) * [[H4, H4], [H4, -H4]] via a 3-stage butterfly.
/// Lossless: ||H8*x||_2 = ||x||_2 for all x. 24 add/sub + 8 multiply.
///
/// Used in 8-line FDN reverb engines (Lexicon 224-style).
/// Call once per sample, after reading from all 8 delay lines and
/// before writing feedback values back into the delay lines.
///
/// @param x  Pointer to float[8]. Modified in-place. Must not be null.
inline void hadamard8(float* x) noexcept
{
    // Stage 1: stride-1 butterflies (adjacent pairs)
    const float t0 = x[0] + x[1];
    const float t1 = x[0] - x[1];
    const float t2 = x[2] + x[3];
    const float t3 = x[2] - x[3];
    const float t4 = x[4] + x[5];
    const float t5 = x[4] - x[5];
    const float t6 = x[6] + x[7];
    const float t7 = x[6] - x[7];

    // Stage 2: stride-2 butterflies (pairs of pairs)
    const float u0 = t0 + t2;
    const float u1 = t1 + t3;
    const float u2 = t0 - t2;
    const float u3 = t1 - t3;
    const float u4 = t4 + t6;
    const float u5 = t5 + t7;
    const float u6 = t4 - t6;
    const float u7 = t5 - t7;

    // Stage 3: stride-4 butterflies + normalization (1/sqrt(8) applied once)
    x[0] = kHadamard8Scale * (u0 + u4);
    x[1] = kHadamard8Scale * (u1 + u5);
    x[2] = kHadamard8Scale * (u2 + u6);
    x[3] = kHadamard8Scale * (u3 + u7);
    x[4] = kHadamard8Scale * (u0 - u4);
    x[5] = kHadamard8Scale * (u1 - u5);
    x[6] = kHadamard8Scale * (u2 - u6);
    x[7] = kHadamard8Scale * (u3 - u7);
}

} // namespace fdn
