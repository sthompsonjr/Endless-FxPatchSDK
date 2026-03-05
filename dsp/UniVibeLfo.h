#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

/// Uni-Vibe style incandescent bulb + LDR (photoresistor) modulation source.
///
/// Models the Shin-Ei Uni-Vibe's unique LFO system where an incandescent bulb
/// illuminates 4 cadmium-sulfide (CdS) photoresistors. The chain of nonlinear
/// interactions produces the characteristic asymmetric "throb":
///
///   Phase accumulator (sine)
///       -> Bulb nonlinearity (current->light is superlinear)
///       -> Thermal lag filter (asymmetric: fast heat ~140ms, slow cool ~300ms)
///       -> 4x LDR models (each with asymmetric response + manufacturing offsets)
///       -> 4 modulation outputs [0, 1]
///
/// This is a modulation source only — feed the 4 outputs into 4 allpass filters
/// to build the complete Uni-Vibe phaser effect.
class UniVibeLfo {
public:
    static constexpr int kNumStages = 4;

    void init(float sampleRate) {
        sampleRate_ = sampleRate;
        invSampleRate_ = 1.0f / sampleRate;

        phase_ = 0.0f;
        rate_ = 1.0f;
        intensity_ = 1.0f;
        bulbState_ = 0.0f;
        bulbBrightness_ = 0.0f;

        // Bulb thermal time constants
        // Rise ~140ms (heating), Fall ~300ms (cooling)
        bulbRiseCoeff_ = expf(-1.0f / (0.140f * sampleRate));
        bulbFallCoeff_ = expf(-1.0f / (0.300f * sampleRate));

        // LDR response time constants
        // Rise ~20ms (brightening — fast), Fall ~150ms (darkening — slow)
        ldrRiseCoeff_ = expf(-1.0f / (0.020f * sampleRate));
        ldrFallCoeff_ = expf(-1.0f / (0.150f * sampleRate));

        // Per-LDR manufacturing tolerance offsets
        // These create the subtle per-stage variation that makes the Uni-Vibe
        // sound richer than a simple 4-stage phaser with identical LFO taps.
        constexpr float kOffsets[kNumStages] = { 0.0f, 0.03f, -0.02f, 0.015f };
        for (int i = 0; i < kNumStages; ++i) {
            ldrs_[i].filterState = 0.0f;
            ldrs_[i].sensitivityOffset = kOffsets[i];
            ldrs_[i].output = 0.0f;
        }
    }

    /// Set LFO rate in Hz. Typical Uni-Vibe range: 0.5–12 Hz.
    void setRate(float hz) {
        rate_ = hz;
    }

    /// Set modulation intensity (0 = no modulation, 1 = full sweep).
    void setIntensity(float amount) {
        intensity_ = amount;
    }

    void reset() {
        phase_ = 0.0f;
        bulbState_ = 0.0f;
        bulbBrightness_ = 0.0f;
        for (int i = 0; i < kNumStages; ++i) {
            ldrs_[i].filterState = 0.0f;
            ldrs_[i].output = 0.0f;
        }
    }

    /// Advance the model by one sample. Call once per audio sample.
    void process() {
        // --- Phase accumulator ---
        phase_ += rate_ * invSampleRate_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;

        // Unipolar sine drive [0, 1]
        float sineDrive = 0.5f * (1.0f + sinf(phase_ * 6.283185307f));

        // --- Bulb nonlinearity ---
        // Incandescent light output is superlinear with current.
        // pow(x, 1.5) via x * sqrt(x) — no per-sample powf needed.
        float bulbDrive = sineDrive * sqrtf(sineDrive);

        // --- Thermal lag filter (asymmetric one-pole) ---
        // The filament has thermal mass: heats faster than it cools.
        // This is THE key to the Uni-Vibe sound — at high speeds the
        // bulb can't fully heat/cool, naturally compressing modulation depth.
        float coeff;
        if (bulbDrive > bulbState_) {
            coeff = bulbRiseCoeff_;  // heating: ~140ms
        } else {
            coeff = bulbFallCoeff_;  // cooling: ~300ms
        }
        bulbState_ = coeff * bulbState_ + (1.0f - coeff) * bulbDrive;
        bulbBrightness_ = bulbState_;

        // --- Per-LDR processing ---
        for (int i = 0; i < kNumStages; ++i) {
            // Apply manufacturing offset to brightness
            float brightness = bulbBrightness_ + ldrs_[i].sensitivityOffset;
            brightness = std::clamp(brightness, 0.0f, 1.0f);

            // LDR asymmetric response filter
            // CdS cells respond faster to increasing light than decreasing.
            float ldrCoeff;
            if (brightness > ldrs_[i].filterState) {
                ldrCoeff = ldrRiseCoeff_;   // brightening: ~20ms
            } else {
                ldrCoeff = ldrFallCoeff_;   // darkening: ~150ms
            }
            ldrs_[i].filterState = ldrCoeff * ldrs_[i].filterState
                                 + (1.0f - ldrCoeff) * brightness;

            // Perceptual curve: sqrt approximates the log response of CdS cells
            // (resistance is inversely proportional to a power of light intensity)
            float curved = sqrtf(ldrs_[i].filterState);

            // Scale by intensity and output
            ldrs_[i].output = std::clamp(curved * intensity_, 0.0f, 1.0f);
        }
    }

    /// Get the modulation output for a specific LDR stage (0–3).
    /// Returns a value in [0, 1] suitable for controlling allpass filter frequency.
    [[nodiscard]] float getLdrOutput(int stage) const {
        return ldrs_[stage].output;
    }

    /// Get the raw bulb brightness [0, 1] before LDR processing.
    /// Useful for driving an LED indicator on the pedal.
    [[nodiscard]] float getBulbBrightness() const {
        return bulbBrightness_;
    }

private:
    float sampleRate_ = 48000.0f;
    float invSampleRate_ = 1.0f / 48000.0f;

    // Phase accumulator
    float phase_ = 0.0f;
    float rate_ = 1.0f;
    float intensity_ = 1.0f;

    // Bulb thermal model
    float bulbState_ = 0.0f;
    float bulbRiseCoeff_ = 0.0f;
    float bulbFallCoeff_ = 0.0f;
    float bulbBrightness_ = 0.0f;

    // LDR response coefficients (shared across all 4 stages)
    float ldrRiseCoeff_ = 0.0f;
    float ldrFallCoeff_ = 0.0f;

    // Per-LDR state
    struct LdrState {
        float filterState = 0.0f;
        float sensitivityOffset = 0.0f;
        float output = 0.0f;
    };
    std::array<LdrState, kNumStages> ldrs_{};
};
