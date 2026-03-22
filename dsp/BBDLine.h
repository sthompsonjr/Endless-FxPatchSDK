#pragma once

#include "CircularBuffer.h"
#include "Saturation.h"
#include <cmath>
#include <cstdint>

/// Bucket Brigade Device delay line emulation.
///
/// Models the signal path of analog BBD chips (MN3007, MN3207, etc.):
///   Input compander -> BBD stages -> reconstruction lowpass -> output expander
///
/// Features:
///   - Hermite cubic interpolation for smooth modulated delay reads
///   - Subtle input compander saturation (soft-clip compress/expand)
///   - Clock noise bleed proportional to clock rate
///   - 8 kHz reconstruction lowpass to remove clock artifacts
///
/// Template parameter Size must be power of 2; represents the BBD stage count
/// (real chips: 256–4096 stages). Typical: 1024 or 2048.
template<size_t Size>
class BBDLine {
    static_assert((Size & (Size - 1)) == 0, "BBDLine Size must be a power of 2");

public:
    /// Initialize the BBD line.
    /// @param sampleRate  System sample rate (e.g. 48000)
    void init(float sampleRate) {
        sampleRate_ = sampleRate;
        buffer_.reset();

        // Reconstruction lowpass: one-pole at 8 kHz (BBD anti-alias)
        float cutoff = 8000.0f;
        if (cutoff > sampleRate * 0.45f) {
            cutoff = sampleRate * 0.45f;
        }
        lpCoeff_ = expf(-6.283185307f * cutoff / sampleRate);
        lpState_ = 0.0f;

        // Clock noise state
        clockPhase_ = 0.0f;
        noiseState_ = 48271u;

        // Defaults
        delaySamples_ = static_cast<float>(Size / 2);
        compAmount_ = 0.3f;
        clockNoiseLevel_ = 0.005f;
    }

    /// Set the delay time in samples (fractional for modulation).
    /// Clamped to [1, Size-2] to keep Hermite interpolation in bounds.
    void setDelaySamples(float samples) {
        if (samples < 1.0f) samples = 1.0f;
        float maxDelay = static_cast<float>(Size - 2);
        if (samples > maxDelay) samples = maxDelay;
        delaySamples_ = samples;
    }

    /// Set delay time in milliseconds.
    void setDelayMs(float ms) {
        setDelaySamples(ms * 0.001f * sampleRate_);
    }

    /// Set compander amount (0.0 = bypass, 1.0 = heavy compression).
    /// Real BBDs use ~0.2–0.4 for subtle warmth.
    void setCompanderAmount(float amount) {
        compAmount_ = amount;
    }

    /// Set clock noise bleed level (0.0 = none, 0.02 = audible).
    /// Real BBDs leak clock signal into the audio path.
    void setClockNoiseLevel(float level) {
        clockNoiseLevel_ = level;
    }

    /// Process one sample through the BBD emulation.
    [[nodiscard]] float process(float input) {
        // --- Input compander (compress) ---
        // Soft-clip the input to emulate the compressor in the compander IC.
        // Driven slightly to add the characteristic BBD warmth.
        float compressed = input;
        if (compAmount_ > 0.0f) {
            float driven = input * (1.0f + compAmount_ * 2.0f);
            compressed = sat::softClip(driven);
        }

        // --- Write to BBD stages ---
        buffer_.write(compressed);

        // --- Read with Hermite interpolation ---
        // This is the core: fractional delay for chorus/vibrato modulation.
        float delayed = buffer_.readHermite(delaySamples_);

        // --- Clock noise injection ---
        // Real BBDs have clock bleed — a faint buzz at the clock frequency.
        // Clock freq ≈ sampleRate * (Size / delaySamples) in a real chip,
        // but we simulate it as a simple high-frequency noise burst.
        float clockNoise = 0.0f;
        if (clockNoiseLevel_ > 0.0f) {
            // Advance clock phase based on delay time
            // Shorter delay = faster clock = more audible noise
            float clockRate = sampleRate_ * (static_cast<float>(Size) / delaySamples_);
            clockPhase_ += clockRate / sampleRate_;
            if (clockPhase_ >= 1.0f) {
                clockPhase_ -= static_cast<float>(static_cast<int>(clockPhase_));
                // New noise sample at each clock tick
                noiseState_ = noiseState_ * 1664525u + 1013904223u;
            }
            // Clock bleed: square-ish pulse shaped by noise state
            float noiseSample = static_cast<float>(noiseState_ >> 16) / 32768.0f - 1.0f;
            float clockPulse = (clockPhase_ < 0.5f) ? 1.0f : -1.0f;
            clockNoise = clockPulse * noiseSample * clockNoiseLevel_;
        }

        float withNoise = delayed + clockNoise;

        // --- Reconstruction lowpass (one-pole at ~8 kHz) ---
        // Real BBDs have a sample-and-hold output that needs filtering.
        // This removes the staircase artifacts and clock residuals.
        lpState_ = (1.0f - lpCoeff_) * withNoise + lpCoeff_ * lpState_;

        // --- Output expander ---
        // Inverse of the input compander to restore dynamics.
        // A gentle expansion that undoes the compression character.
        float output = lpState_;
        if (compAmount_ > 0.0f) {
            // Expand: inverse soft-clip approximation
            // For small signals this is nearly unity; for larger signals
            // it gently boosts to restore the dynamic range.
            float absOut = fabsf(output);
            float expandGain = 1.0f + compAmount_ * 0.5f * absOut;
            output *= expandGain;
        }

        return output;
    }

    void reset() {
        buffer_.reset();
        lpState_ = 0.0f;
        clockPhase_ = 0.0f;
    }

private:
    CircularBuffer<float, Size> buffer_;
    float sampleRate_ = 48000.0f;
    float delaySamples_ = 512.0f;

    // Compander
    float compAmount_ = 0.3f;

    // Reconstruction lowpass (one-pole)
    float lpCoeff_ = 0.0f;
    float lpState_ = 0.0f;

    // Clock noise
    float clockPhase_ = 0.0f;
    float clockNoiseLevel_ = 0.005f;
    uint32_t noiseState_ = 48271u;
};
