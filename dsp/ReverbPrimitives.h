#pragma once

#include "CircularBuffer.h"
#include <algorithm>
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

// ════════════════════════════════════════════════════════════════════════════
// ModulatedAllpassDelay — LFO-swept Schroeder allpass diffuser
// ════════════════════════════════════════════════════════════════════════════

static constexpr float kTwoPi = 6.28318530f;

namespace modulated_allpass {

constexpr float kDefaultGain = 0.6f;
constexpr float kMinCenterDelaySamples = 4.0f;
constexpr float kMaxDepthSamples = 64.0f;
constexpr int kHermiteGuard = 3;

} // namespace modulated_allpass

/// Schroeder allpass diffuser with sinusoidal LFO delay modulation.
///
/// Transfer function: H(z) = (z^(-D) - g) / (1 - g*z^(-D))   |H| = 1 for all w
/// Delay D is modulated: D(n) = center + depth * sin(2*pi * phase(n))
///
/// Used in FDN reverb diffusion chains (Lexicon 224-style).
/// Each instance owns its LFO phase — stagger initialPhase across instances
/// to prevent coherent modulation.
///
/// @tparam Size  Power-of-2 buffer size in samples.
template<size_t Size>
class ModulatedAllpassDelay {
public:
    void init(float centerDelaySamples,
              float gain,
              float rateHz,
              float depthSamples,
              float sampleRate,
              float initialPhase = 0.0f) {
        g_ = std::clamp(gain, -0.999f, 0.999f);
        depth_ = std::clamp(depthSamples, 0.0f, modulated_allpass::kMaxDepthSamples);
        center_ = std::clamp(centerDelaySamples,
                             modulated_allpass::kMinCenterDelaySamples,
                             static_cast<float>(Size - modulated_allpass::kHermiteGuard) - depth_ - 1.0f);
        phaseInc_ = rateHz / sampleRate;
        reset();
        phase_ = fmodf(initialPhase, 1.0f);
    }

    [[nodiscard]] float process(float x) {
        // 1. Compute modulated delay time
        const float lfo = sinf(kTwoPi * phase_);
        const float D_frac = center_ + depth_ * lfo;

        // 2. Clamp delay to safe Hermite read range
        const float D_safe = std::clamp(D_frac,
                                        1.0f,
                                        static_cast<float>(Size - modulated_allpass::kHermiteGuard));

        // 3. Read from buffer (Hermite fractional interpolation) — BEFORE write
        const float delayed = buf_.readHermite(D_safe);

        // 4. Schroeder allpass signal flow
        const float w = x + g_ * delayed;
        const float y = -g_ * w + delayed;

        // 5. Write to buffer — AFTER read
        buf_.write(w);

        // 6. Advance LFO phase
        phase_ += phaseInc_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;

        return y;
    }

    void setRate(float rateHz, float sampleRate) {
        phaseInc_ = rateHz / sampleRate;
    }

    void setDepth(float depthSamples) {
        depth_ = std::clamp(depthSamples, 0.0f, modulated_allpass::kMaxDepthSamples);
        depth_ = std::min(depth_,
                          static_cast<float>(Size - modulated_allpass::kHermiteGuard) - center_ - 1.0f);
    }

    void setCenterDelay(float centerDelaySamples) {
        center_ = std::clamp(centerDelaySamples,
                             modulated_allpass::kMinCenterDelaySamples,
                             static_cast<float>(Size - modulated_allpass::kHermiteGuard) - depth_ - 1.0f);
    }

    void reset() {
        buf_.reset();
        phase_ = 0.0f;
    }

private:
    CircularBuffer<float, Size> buf_;
    float g_ = 0.6f;
    float center_ = 0.0f;
    float depth_ = 0.0f;
    float phaseInc_ = 0.0f;
    float phase_ = 0.0f;
};

// ════════════════════════════════════════════════════════════════════════════
// BitCrusher — Fixed-point quantizer for vintage digital reverb hardware
// ════════════════════════════════════════════════════════════════════════════

namespace bit_crusher {

constexpr int kMinBits = 8;
constexpr int kMaxBits = 24;
constexpr int kDefaultBits = 12;

} // namespace bit_crusher

/// Fixed-point quantizer modeling vintage digital reverb hardware (e.g. Lexicon 224).
///
/// Quantizes input to B-bit resolution: q = round(x * 2^(B-1)) / 2^(B-1)
/// Input is clamped to [-1.0f, 1.0f] before quantization.
///
/// Optional first-order noise shaping redistributes quantization noise toward
/// high frequencies, producing a cleaner-sounding 12-bit artifact at the cost
/// of a slightly raised noise floor above fs/4.
///
/// Designed for use in the FDN feedback path (one instance per delay line).
/// At 12 bits, models the Lexicon 224 ADC/DAC round-trip in the feedback loop.
class BitCrusher {
public:
    enum class Mode {
        Round,       ///< Standard nearest-integer rounding. White quantization noise.
        NoiseShape,  ///< First-order error diffusion. Noise shaped to high frequencies.
    };

    /// Initialize the quantizer.
    ///
    /// @param bits  Bit depth in [8, 24]. Clamped to this range.
    ///              12 = Lexicon 224. 8 = lo-fi. 24 = transparent.
    /// @param mode  Rounding mode. Default: Round (standard quantization).
    void init(int bits, Mode mode = Mode::Round) noexcept {
        const int clamped = std::clamp(bits, bit_crusher::kMinBits, bit_crusher::kMaxBits);
        levels_ = static_cast<float>(1 << (clamped - 1));
        invLevels_ = 1.0f / levels_;
        mode_ = mode;
        errorAcc_ = 0.0f;
    }

    /// Process one sample through the quantizer.
    ///
    /// @param  x  Input sample. Clamped to [-1.0f, 1.0f] internally.
    /// @return    Quantized output sample.
    [[nodiscard]] float process(float x) noexcept {
        if (mode_ == Mode::Round) {
            const float clamped = std::clamp(x, -1.0f, 1.0f);
            return roundf(clamped * levels_) * invLevels_;
        } else {
            const float clamped = std::clamp(x - errorAcc_, -1.0f, 1.0f);
            const float q = roundf(clamped * levels_) * invLevels_;
            errorAcc_ = q - clamped;
            return q;
        }
    }

    /// Change bit depth without resetting the error accumulator.
    void setBits(int bits) noexcept {
        const int clamped = std::clamp(bits, bit_crusher::kMinBits, bit_crusher::kMaxBits);
        levels_ = static_cast<float>(1 << (clamped - 1));
        invLevels_ = 1.0f / levels_;
    }

    /// Reset the error accumulator to zero. Does not change bit depth or mode.
    void reset() noexcept {
        errorAcc_ = 0.0f;
    }

private:
    float levels_    = 2048.0f;
    float invLevels_ = 1.0f / 2048.0f;
    float errorAcc_  = 0.0f;
    Mode  mode_      = Mode::Round;
};
