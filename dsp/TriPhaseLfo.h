#pragma once

#include <array>
#include <cmath>
#include <cstdint>

/// Multi-tap analog-modeled LFO driven by a SINGLE phase accumulator.
///
/// Real multi-phase modulation generators (the Tri-Stereo Chorus 3-phase
/// generators, the BOSS DC-2's inverted/non-inverted triangle outputs)
/// derive every phase from ONE oscillator core. Drift and jitter are
/// therefore common-mode: the pairwise instantaneous phase differences
/// between taps are invariant under any frequency wobble. Three independent
/// `AnalogLfo` instances at equal base frequency do NOT have this property —
/// their drift oscillators run free and the phases walk apart. This class
/// exists to provide the hardware-correct behavior:
///
///   - One master phase accumulator with the exact drift/jitter engine
///     ported from `AnalogLfo` (see tick(); constants verified against
///     dsp/AnalogLfo.h).
///   - Up to MaxTaps read taps, each at a fixed phase offset in [0, 1).
///   - Per-tap linear blend between two shape endpoints (shared shape pair,
///     per-tap blend position m in [0, 1]).
///
/// Usage per sample: call tick() once, then read tap(i) for each tap.
///
/// Configurations:
///   - Tri-Stereo Chorus: 3 taps at 0, 1/3, 2/3 (0/120/240 degrees).
///   - BOSS DC-2:         2 taps at 0, 1/2, Triangle/Triangle. The bipolar
///     triangle satisfies tri(x) + tri(x + 0.5) = 0 for all x, so the two
///     taps sum to zero every sample — the DC-2's constant-average
///     ("motionless") delay property.
///
/// Constraints: no heap, no virtual dispatch, fixed-size std::array state,
/// single-precision float only, noexcept audio path.
template <int MaxTaps = 4>
class TriPhaseLfo {
    static_assert(MaxTaps >= 1 && MaxTaps <= 16, "MaxTaps must be in [1, 16]");

public:
    /// Shape semantics identical to AnalogLfo::Shape (same formulas).
    enum class Shape { Sine, Triangle, Saw, ReverseSaw, Square };

    /// Initialize with system sample rate.
    /// Defaults are identical to AnalogLfo::init(), including the LCG seed.
    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;
        invSampleRate_ = 1.0f / sampleRate;

        phase_ = 0.0f;
        baseFreq_ = 1.0f;
        shapeA_ = Shape::Sine;
        shapeB_ = Shape::Sine;

        // Drift oscillator: very slow sine that modulates frequency
        driftPhase_ = 0.0f;
        driftFreq_ = 0.13f;     // ~0.13 Hz — slow wander
        driftAmount_ = 0.012f;  // 1.2% frequency deviation

        // Per-sample jitter
        jitterAmount_ = 0.0003f;  // ~0.03% random phase noise
        lcgState_ = 7919u;       // prime seed

        // Second drift oscillator for more complex wander
        drift2Phase_ = 0.0f;
        drift2Freq_ = 0.07f;    // even slower secondary drift
        drift2Amount_ = 0.006f; // 0.6% secondary deviation

        activeTaps_ = 1;
        tapPhase_.fill(0.0f);
        tapBlend_.fill(0.0f);
        tapOut_.fill(0.0f);
    }

    /// Set the base (center) frequency in Hz.
    void setFrequency(float hz) noexcept {
        baseFreq_ = hz;
    }

    /// Set the two blend endpoint shapes, shared by all taps.
    /// tap output = shapeA at m = 0, shapeB at m = 1.
    void setShapes(Shape a, Shape b) noexcept {
        shapeA_ = a;
        shapeB_ = b;
    }

    /// Set the number of taps computed by tick(). Clamped to [1, MaxTaps].
    void setActiveTaps(int n) noexcept {
        if (n < 1) n = 1;
        if (n > MaxTaps) n = MaxTaps;
        activeTaps_ = n;
    }

    /// Set tap i's phase offset in cycles. Wrapped into [0, 1).
    /// Out-of-range tap indices are ignored.
    void setTapPhase(int i, float phase01) noexcept {
        if (i < 0 || i >= MaxTaps) return;
        phase01 -= floorf(phase01);       // wrap into [0, 1)
        if (phase01 >= 1.0f) phase01 = 0.0f;
        tapPhase_[static_cast<size_t>(i)] = phase01;
    }

    /// Set tap i's shape blend position m in [0, 1] (clamped).
    /// Out-of-range tap indices are ignored.
    void setTapBlend(int i, float m) noexcept {
        if (i < 0 || i >= MaxTaps) return;
        if (m < 0.0f) m = 0.0f;
        if (m > 1.0f) m = 1.0f;
        tapBlend_[static_cast<size_t>(i)] = m;
    }

    /// Set drift amount as a fraction (0.0 = perfect, 0.012 = 1.2% wobble).
    void setDriftAmount(float amount) noexcept {
        driftAmount_ = amount;
    }

    /// Set the drift oscillator rate in Hz (typically 0.05–0.3).
    void setDriftRate(float hz) noexcept {
        driftFreq_ = hz;
    }

    /// Set per-sample jitter amount (0.0 = none, 0.001 = noticeable).
    void setJitterAmount(float amount) noexcept {
        jitterAmount_ = amount;
    }

    void reset() noexcept {
        phase_ = 0.0f;
        driftPhase_ = 0.0f;
        drift2Phase_ = 0.0f;
        tapOut_.fill(0.0f);
    }

    /// Advance the master phase by one sample, then compute all active tap
    /// outputs. Call exactly once per sample, then read via tap(i).
    ///
    /// The drift/jitter engine below is ported VERBATIM from
    /// dsp/AnalogLfo.h — AnalogLfo::process(). If AnalogLfo's math changes,
    /// this block must be updated to match (and vice versa).
    void tick() noexcept {
        // --- Advance drift oscillators ---
        driftPhase_ += driftFreq_ * invSampleRate_;
        if (__builtin_expect(driftPhase_ >= 1.0f, 0)) driftPhase_ -= 1.0f;

        drift2Phase_ += drift2Freq_ * invSampleRate_;
        if (__builtin_expect(drift2Phase_ >= 1.0f, 0)) drift2Phase_ -= 1.0f;

        // Combined drift: two sine oscillators at different rates
        // This creates a more natural, non-periodic wander
        float drift = sinf(driftPhase_ * 6.283185307f) * driftAmount_
                    + sinf(drift2Phase_ * 6.283185307f) * drift2Amount_;

        // --- Per-sample jitter ---
        // Tiny random nudge to the phase increment each sample.
        // Uses LCG mapped to [-1, 1] then scaled by jitter amount.
        lcgState_ = lcgState_ * 1664525u + 1013904223u;
        float noise = static_cast<float>(lcgState_ >> 16) / 32768.0f - 1.0f;
        float jitter = noise * jitterAmount_;

        // --- Modulated frequency ---
        // Base frequency + drift modulation + jitter
        float modulatedFreq = baseFreq_ * (1.0f + drift) + baseFreq_ * jitter;
        if (__builtin_expect(modulatedFreq < 0.0f, 0)) modulatedFreq = 0.0f;

        // --- Phase accumulator ---
        phase_ += modulatedFreq * invSampleRate_;
        if (__builtin_expect(phase_ >= 1.0f, 0)) phase_ -= 1.0f;
        if (__builtin_expect(phase_ < 0.0f, 0)) phase_ += 1.0f;
        // --- End of AnalogLfo::process() port ---

        // --- Tap outputs ---
        // All taps read the SAME master phase; offsets are constant, so the
        // pairwise phase differences are invariant under drift and jitter.
        // Both phase_ and tapPhase_[i] are in [0, 1), so p is in [0, 2) and
        // a single conditional subtract wraps it. The subtraction p - 1.0f
        // is exact for p in [1, 2) (Sterbenz), which preserves the triangle
        // antiphase identity tri(x) + tri(x + 0.5) = 0 to float precision.
        for (int i = 0; i < activeTaps_; ++i) {
            const size_t k = static_cast<size_t>(i);
            float p = phase_ + tapPhase_[k];
            if (p >= 1.0f) p -= 1.0f;
            const float a = computeShape(shapeA_, p);
            const float b = computeShape(shapeB_, p);
            tapOut_[k] = a + (b - a) * tapBlend_[k];
        }
    }

    /// Tap i's bipolar output [-1, 1] as of the last tick().
    /// Index is NOT bounds-checked (hot path) — caller keeps i in
    /// [0, activeTaps).
    [[nodiscard]] float tap(int i) const noexcept {
        return tapOut_[static_cast<size_t>(i)];
    }

    /// Tap i's unipolar output [0, 1] as of the last tick().
    [[nodiscard]] float tapUnipolar(int i) const noexcept {
        return tapOut_[static_cast<size_t>(i)] * 0.5f + 0.5f;
    }

    /// Master phase [0, 1) — useful for syncing external effects.
    [[nodiscard]] float getPhase() const noexcept {
        return phase_;
    }

private:
    /// Shape formulas identical to AnalogLfo::computeShape().
    [[nodiscard]] static float computeShape(Shape s, float p) noexcept {
        switch (s) {
            case Shape::Sine:
                return sinf(p * 6.283185307f);
            case Shape::Triangle:
                return 1.0f - fabsf(p * 4.0f - 2.0f);
            case Shape::Saw:
                return p * 2.0f - 1.0f;
            case Shape::ReverseSaw:
                return 1.0f - p * 2.0f;
            case Shape::Square:
                return p < 0.5f ? 1.0f : -1.0f;
        }
        return 0.0f;
    }

    float sampleRate_ = 48000.0f;
    float invSampleRate_ = 1.0f / 48000.0f;

    // Master oscillator (single accumulator shared by all taps)
    float phase_ = 0.0f;
    float baseFreq_ = 1.0f;
    Shape shapeA_ = Shape::Sine;
    Shape shapeB_ = Shape::Sine;

    // Primary drift oscillator (constants from dsp/AnalogLfo.h)
    float driftPhase_ = 0.0f;
    float driftFreq_ = 0.13f;
    float driftAmount_ = 0.012f;

    // Secondary drift oscillator (slower, for complex wander)
    float drift2Phase_ = 0.0f;
    float drift2Freq_ = 0.07f;
    float drift2Amount_ = 0.006f;

    // Per-sample jitter
    float jitterAmount_ = 0.0003f;
    uint32_t lcgState_ = 7919u;

    // Taps
    int activeTaps_ = 1;
    std::array<float, MaxTaps> tapPhase_{};
    std::array<float, MaxTaps> tapBlend_{};
    std::array<float, MaxTaps> tapOut_{};
};
