#pragma once

#include "CircularBuffer.h"
#include "OnePoleFilter.h"
#include "ParameterSmoother.h"
#include <algorithm>
#include <cmath>
#include <cstring>

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

// ════════════════════════════════════════════════════════════════════════════
// FdnReverb<N> — Feedback Delay Network Reverb Engine (Lexicon 224-style)
// ════════════════════════════════════════════════════════════════════════════

namespace fdn_reverb {

/// Delay line lengths for 8-line FDN (mutually prime, 48kHz reference).
constexpr int kDelayLengths8[8] = { 1019, 1399, 1877, 2269, 2677, 3137, 3547, 4073 };

/// Delay line lengths for 4-line FDN (even-indexed subset of kDelayLengths8).
constexpr int kDelayLengths4[4] = { 1019, 1877, 2677, 3547 };

/// Power-of-2 buffer sizes for each delay line (N=8).
constexpr int kDelaySizes8[8] = { 1024, 2048, 2048, 4096, 4096, 4096, 4096, 8192 };

/// Power-of-2 buffer sizes for each delay line (N=4).
constexpr int kDelaySizes4[4] = { 1024, 2048, 4096, 4096 };

/// Pre-diffusion allpass center delays (samples, mutually prime, < L[0]).
constexpr float kPreDiffDelays[4]  = { 113.0f, 211.0f, 359.0f, 509.0f };

/// Post-diffusion allpass center delays (same lengths, reversed order).
constexpr float kPostDiffDelays[4] = { 509.0f, 359.0f, 211.0f, 113.0f };

/// Allpass gain for diffusion stages.
constexpr float kDiffusionGain = 0.6f;

/// Pre-diffusion LFO rate in Hz.
constexpr float kPreDiffRate  = 0.5f;

/// Pre-diffusion LFO depth in samples.
constexpr float kPreDiffDepth = 2.0f;

/// Post-diffusion LFO rate in Hz (slightly faster than pre).
constexpr float kPostDiffRate  = 0.7f;

/// Post-diffusion LFO depth in samples.
constexpr float kPostDiffDepth = 3.0f;

/// Default T60 decay time in seconds.
constexpr float kDefaultT60 = 2.0f;

/// Default HF damping cutoff frequency in Hz (Lexicon 224 character).
constexpr float kDefaultDampingHz = 8000.0f;

/// Default bit depth for feedback path quantization.
constexpr int kDefaultBits = 12;

/// Default wet/dry mix ratio (0=dry, 1=wet).
constexpr float kDefaultMix = 0.5f;

/// Minimum T60 to prevent decay gain > 1.0.
constexpr float kMinT60 = 0.05f;

/// Maximum T60.
constexpr float kMaxT60 = 30.0f;

} // namespace fdn_reverb

/// Feedback Delay Network reverb engine.
///
/// N must be 4 or 8. Architecture: pre-diffusion allpass chain →
/// N parallel delay lines → Hadamard mixing matrix → per-line HF damping →
/// per-line 12-bit quantization → feedback. Stereo output tapped from
/// two delay lines. Optional post-diffusion further smears the tail.
///
/// All delay-line data lives in an externally supplied working buffer
/// (caller responsibility). All other state is DTCM-resident class members.
///
/// @tparam N  Number of delay lines. Must be 4 or 8.
template<int N>
class FdnReverb
{
    static_assert(N == 4 || N == 8, "FdnReverb: N must be 4 or 8");

public:
    /// Initialize the reverb engine.
    ///
    /// @param sampleRate        Audio sample rate in Hz.
    /// @param workingBuffer     Pointer to external float buffer for delay lines.
    /// @param workingBufferSize Number of floats available in workingBuffer.
    ///                          Must be >= 29696 (N=8) or >= 15360 (N=4).
    void init(float sampleRate, float* workingBuffer, int workingBufferSize) noexcept
    {
        (void)workingBufferSize; // bounds assertion omitted for embedded target
        sampleRate_ = sampleRate;

        const int* lengths = (N == 8) ? fdn_reverb::kDelayLengths8 : fdn_reverb::kDelayLengths4;
        const int* sizes   = (N == 8) ? fdn_reverb::kDelaySizes8   : fdn_reverb::kDelaySizes4;

        // Partition working buffer into N delay lines
        int offset = 0;
        for (int i = 0; i < N; ++i) {
            delayLen_[i]  = lengths[i];
            delaySize_[i] = sizes[i];
            delayData_[i] = workingBuffer + offset;
            offset       += sizes[i];
        }

        // Zero all delay line data and write heads
        for (int i = 0; i < N; ++i) {
            memset(delayData_[i], 0, static_cast<size_t>(delaySize_[i]) * sizeof(float));
            writeHead_[i] = 0;
        }

        // Initialize HF damping filters (lowpass at default cutoff)
        for (int i = 0; i < N; ++i) {
            damping_[i].init(sampleRate);
            damping_[i].setFrequency(fdn_reverb::kDefaultDampingHz);
        }

        // Initialize bit crushers
        for (int i = 0; i < N; ++i)
            crush_[i].init(fdn_reverb::kDefaultBits);

        // Compute initial per-line decay gains (needs delayLen_ and sampleRate_)
        setT60(fdn_reverb::kDefaultT60);

        // Initialize pre-diffusion allpass chain
        for (int i = 0; i < 4; ++i) {
            preDiff_[i].init(fdn_reverb::kPreDiffDelays[i],
                             fdn_reverb::kDiffusionGain,
                             fdn_reverb::kPreDiffRate,
                             fdn_reverb::kPreDiffDepth,
                             sampleRate,
                             static_cast<float>(i) / 4.0f);
        }

        // Initialize post-diffusion (independent L and R paths)
        for (int i = 0; i < 4; ++i) {
            postDiffL_[i].init(fdn_reverb::kPostDiffDelays[i],
                               fdn_reverb::kDiffusionGain,
                               fdn_reverb::kPostDiffRate,
                               fdn_reverb::kPostDiffDepth,
                               sampleRate,
                               0.5f + static_cast<float>(i) / 4.0f);
            postDiffR_[i].init(fdn_reverb::kPostDiffDelays[i],
                               fdn_reverb::kDiffusionGain,
                               fdn_reverb::kPostDiffRate,
                               fdn_reverb::kPostDiffDepth,
                               sampleRate,
                               0.5f + static_cast<float>(i) / 4.0f + 1.0f / 8.0f);
        }

        // Initialize mix smoother and snap to default
        mixSmoother_.init(sampleRate, 20.0f);
        mix_ = fdn_reverb::kDefaultMix;
        mixSmoother_.snapTo(mix_);
        mixSmoother_.setTarget(mix_);
    }

    /// Process one mono input sample, produce stereo output.
    void process(float input, float& outL, float& outR) noexcept
    {
        // 1. Pre-diffusion: smear the input impulse
        float diffused = input;
        for (int i = 0; i < 4; ++i)
            diffused = preDiff_[i].process(diffused);

        // 2. Read all delay lines (before write this sample)
        float d[N];
        for (int i = 0; i < N; ++i) {
            const int readIdx = (writeHead_[i] - delayLen_[i]) & (delaySize_[i] - 1);
            d[i] = delayData_[i][readIdx];
        }

        // 3. Lossless mixing matrix
        if constexpr (N == 8)
            fdn::hadamard8(d);
        else
            fdn::hadamard4(d[0], d[1], d[2], d[3]);

        // 4. Per-line HF damping
        for (int i = 0; i < N; ++i)
            d[i] = damping_[i].process(d[i]);

        // 5. Per-line bit crushing (12-bit quantization in feedback path)
        for (int i = 0; i < N; ++i)
            d[i] = crush_[i].process(d[i]);

        // 6. Write to delay lines (input injection + scaled feedback)
        for (int i = 0; i < N; ++i) {
            delayData_[i][writeHead_[i]] = diffused + g_[i] * d[i];
            writeHead_[i] = (writeHead_[i] + 1) & (delaySize_[i] - 1);
        }

        // 7. Tap stereo outputs (from post-Hadamard, post-damp, post-crush values)
        float wetL = d[0];
        float wetR = d[N / 2];

        // 8. Post-diffusion on each channel independently
        for (int i = 0; i < 4; ++i) {
            wetL = postDiffL_[i].process(wetL);
            wetR = postDiffR_[i].process(wetR);
        }

        // 9. Wet/dry mix with parameter smoothing
        const float mix = mixSmoother_.process();
        outL = mix * wetL + (1.0f - mix) * input;
        outR = mix * wetR + (1.0f - mix) * input;
    }

    /// Set the T60 decay time. Recomputes per-line decay gains immediately.
    /// @param t60Seconds  Decay time in seconds. Clamped to [kMinT60, kMaxT60].
    void setT60(float t60Seconds) noexcept
    {
        t60_ = std::clamp(t60Seconds, fdn_reverb::kMinT60, fdn_reverb::kMaxT60);
        for (int i = 0; i < N; ++i) {
            g_[i] = powf(10.0f, -3.0f * static_cast<float>(delayLen_[i]) / (t60_ * sampleRate_));
            g_[i] = std::clamp(g_[i], 0.0f, 0.9999f);
        }
    }

    /// Set the HF damping cutoff frequency.
    /// @param freqHz     Lowpass cutoff in Hz. Clamped to [500, 20000].
    /// @param sampleRate Audio sample rate (updates filter if changed).
    void setDamping(float freqHz, float sampleRate) noexcept
    {
        dampingHz_ = std::clamp(freqHz, 500.0f, 20000.0f);
        for (int i = 0; i < N; ++i) {
            damping_[i].init(sampleRate);
            damping_[i].setFrequency(dampingHz_);
        }
    }

    /// Set the wet/dry mix. Smoothed toward target over ~20ms.
    /// @param mix  0.0f = fully dry, 1.0f = fully wet.
    void setMix(float mix) noexcept
    {
        mix_ = std::clamp(mix, 0.0f, 1.0f);
        mixSmoother_.setTarget(mix_);
    }

    /// Set the feedback path bit depth.
    /// @param bits  Bit depth [8, 24]. 12 = Lexicon 224.
    void setBits(int bits) noexcept
    {
        for (int i = 0; i < N; ++i)
            crush_[i].setBits(bits);
    }

    /// Reset all audio state to silence. Does not change T60, damping, mix, or bits.
    void reset() noexcept
    {
        for (int i = 0; i < N; ++i) {
            memset(delayData_[i], 0, static_cast<size_t>(delaySize_[i]) * sizeof(float));
            writeHead_[i] = 0;
        }
        for (int i = 0; i < 4; ++i) {
            preDiff_[i].reset();
            postDiffL_[i].reset();
            postDiffR_[i].reset();
        }
        for (int i = 0; i < N; ++i) {
            crush_[i].reset();
            damping_[i].reset();
        }
        mixSmoother_.snapTo(mix_);
        mixSmoother_.setTarget(mix_);
    }

private:
    // ── Delay lines (data in external working buffer) ─────────────────────
    float* delayData_[N];   // pointers into caller-supplied working buffer
    int    delaySize_[N];   // power-of-2 size for each line
    int    delayLen_[N];    // integer read offset in samples
    int    writeHead_[N];   // current write position (DTCM-resident)
    float  g_[N];           // per-line feedback gain derived from T60

    // ── Per-line processing ────────────────────────────────────────────────
    OnePoleFilter damping_[N];   // HF damping lowpass per line
    BitCrusher    crush_[N];     // 12-bit quantizer per line

    // ── Diffusion chains ──────────────────────────────────────────────────
    ModulatedAllpassDelay<1024> preDiff_[4];
    ModulatedAllpassDelay<1024> postDiffL_[4];
    ModulatedAllpassDelay<1024> postDiffR_[4];

    // ── Parameter smoothing ───────────────────────────────────────────────
    ParameterSmoother mixSmoother_;

    // ── Configuration state ───────────────────────────────────────────────
    float sampleRate_  = 48000.0f;
    float t60_         = fdn_reverb::kDefaultT60;
    float dampingHz_   = fdn_reverb::kDefaultDampingHz;
    float mix_         = fdn_reverb::kDefaultMix;
};
