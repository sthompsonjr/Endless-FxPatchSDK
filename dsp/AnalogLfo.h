#pragma once

#include <cmath>
#include <cstdint>

/// Analog-modeled LFO with frequency drift and per-sample jitter.
///
/// Real analog LFOs (CEM3340, etc.) are not perfectly stable:
///   - Component tolerances cause a slow frequency wobble (~0.1–0.3 Hz)
///   - Thermal noise introduces tiny random timing jitter each cycle
///
/// This class layers two imperfections onto a clean phase-accumulator LFO:
///   1. **Drift oscillator**: a very slow sine (~0.13 Hz default) that modulates
///      the main frequency by a configurable percentage (default ~1.2%).
///   2. **Sample jitter**: a tiny random offset added to the phase increment
///      each sample, making every cycle subtly unique.
///
/// The result sounds alive and organic — no two cycles are identical,
/// just like a real VCO/LFO circuit.
class AnalogLfo {
public:
    enum class Shape { Sine, Triangle, Saw, ReverseSaw, Square };

    /// Initialize with system sample rate.
    void init(float sampleRate) {
        sampleRate_ = sampleRate;
        invSampleRate_ = 1.0f / sampleRate;

        phase_ = 0.0f;
        baseFreq_ = 1.0f;
        shape_ = Shape::Sine;

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
    }

    /// Set the base (center) frequency in Hz.
    void setFrequency(float hz) {
        baseFreq_ = hz;
    }

    void setShape(Shape shape) {
        shape_ = shape;
    }

    /// Set drift amount as a fraction (0.0 = perfect, 0.012 = 1.2% wobble).
    void setDriftAmount(float amount) {
        driftAmount_ = amount;
    }

    /// Set the drift oscillator rate in Hz (typically 0.05–0.3).
    void setDriftRate(float hz) {
        driftFreq_ = hz;
    }

    /// Set per-sample jitter amount (0.0 = none, 0.001 = noticeable).
    void setJitterAmount(float amount) {
        jitterAmount_ = amount;
    }

    void reset() {
        phase_ = 0.0f;
        driftPhase_ = 0.0f;
        drift2Phase_ = 0.0f;
    }

    /// Process one sample. Returns bipolar output [-1, 1].
    [[nodiscard]] float process() {
        // --- Advance drift oscillators ---
        driftPhase_ += driftFreq_ * invSampleRate_;
        if (driftPhase_ >= 1.0f) driftPhase_ -= 1.0f;

        drift2Phase_ += drift2Freq_ * invSampleRate_;
        if (drift2Phase_ >= 1.0f) drift2Phase_ -= 1.0f;

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
        if (modulatedFreq < 0.0f) modulatedFreq = 0.0f;

        // --- Phase accumulator ---
        phase_ += modulatedFreq * invSampleRate_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        if (phase_ < 0.0f) phase_ += 1.0f;

        return computeShape(phase_);
    }

    /// Process one sample. Returns unipolar output [0, 1].
    [[nodiscard]] float processUnipolar() {
        return process() * 0.5f + 0.5f;
    }

    /// Get current phase [0, 1) — useful for syncing external effects.
    [[nodiscard]] float getPhase() const {
        return phase_;
    }

    /// Get the current instantaneous frequency including drift.
    /// Useful for debugging or visualizing the wobble.
    [[nodiscard]] float getCurrentFrequency() const {
        float drift = sinf(driftPhase_ * 6.283185307f) * driftAmount_
                    + sinf(drift2Phase_ * 6.283185307f) * drift2Amount_;
        return baseFreq_ * (1.0f + drift);
    }

private:
    [[nodiscard]] float computeShape(float p) const {
        switch (shape_) {
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

    // Main oscillator
    float phase_ = 0.0f;
    float baseFreq_ = 1.0f;
    Shape shape_ = Shape::Sine;

    // Primary drift oscillator
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
};
