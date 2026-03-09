#pragma once

#include <algorithm>
#include <cmath>

/// LM741 Op-Amp open-loop signal model.
///
/// NOT a WDF element — a signal processing block that takes differential
/// input voltage (volts) and returns output voltage (volts).
///
/// Models:
///   - Open-loop gain: A0 = 100 (effective, reduced from physical 200k
///     for stability with one-sample delay feedback; still >> max closed-
///     loop gain Rf/R1 ≈ 7.4, so virtual ground approximation holds)
///   - Dominant pole: 100 Hz (single-pole IIR, raised from physical 5 Hz
///     for faster settling; GBW = A0*fp = 10 kHz preserved)
///   - Unity-gain bandwidth: ~10 kHz (A0_eff × fp)
///   - Slew rate: 0.5 V/µs
///   - Output rail clamping: ±Vrail (default 13V from ±15V supply)
///   - Input offset voltage: 1mV typical
///
/// All coefficients are precomputed in init() — the per-sample path
/// uses only multiply, add, clamp (no expf/sinf).
class WdfOpAmpLM741 {
public:
    void init(float sampleRate) {
        sampleRate_ = sampleRate;

        // Open-loop gain — reduced from physical 200k for stability with
        // one-sample delay feedback. Must satisfy A0 * β_max * (1-b1) < 2
        // where β_max ≈ 0.94 (at Rf=4.7k). With fp=100Hz, fs=48k:
        // (1-b1) ≈ 0.013, so A0 < 164. Use 100 for margin.
        // GBW = A0 * fp = 100 * 100 = 10 kHz.
        A0_ = 100.0f;

        // Dominant pole at 100 Hz: one-pole IIR coefficient
        // b1 = exp(-2*pi*fp/fs)
        // Raised from physical 5 Hz to 100 Hz for faster settling
        // while preserving GBW = 10 kHz.
        b1_ = expf(-6.283185307f * 100.0f / sampleRate_);
        oneMinusB1_ = 1.0f - b1_;

        // Slew rate: 0.5 V/µs = 500000 V/s
        slewLimitPerSample_ = 500000.0f / sampleRate_;

        // Rail voltage (±15V supply, ~2V dropout)
        Vrail_ = 13.0f;

        // Input offset voltage
        Vos_ = 0.001f;

        openLoopState_ = 0.0f;
        prevOutput_ = 0.0f;
    }

    /// Set rail voltage for "dying battery" effect.
    /// Default 13V. Lower values (e.g. 4V) produce saggy, compressed distortion.
    void setRailVoltage(float Vrail) {
        Vrail_ = std::max(Vrail, 0.1f);
    }

    [[nodiscard]] float process(float vDiff) noexcept {
        // 1. Add input offset voltage
        float vEff = vDiff + Vos_;

        // 2. Open-loop amplification
        float vAmplified = A0_ * vEff;

        // 3. Clamp to prevent float overflow before IIR filter
        vAmplified = std::clamp(vAmplified, -1e4f, 1e4f);

        // 4. Dominant pole IIR (5 Hz single pole)
        openLoopState_ = oneMinusB1_ * vAmplified + b1_ * openLoopState_;

        // 4b. Clamp IIR state to prevent accumulated drift
        openLoopState_ = std::clamp(openLoopState_, -2.0f * Vrail_, 2.0f * Vrail_);

        // 5. Slew rate limiting
        float delta = openLoopState_ - prevOutput_;
        delta = std::clamp(delta, -slewLimitPerSample_, slewLimitPerSample_);

        // 6. Apply slew-limited output
        float vOut = prevOutput_ + delta;

        // 7. Rail clamping
        vOut = std::clamp(vOut, -Vrail_, Vrail_);

        // 8. Store for next sample
        prevOutput_ = vOut;

        return vOut;
    }

    void reset() noexcept {
        openLoopState_ = 0.0f;
        prevOutput_ = 0.0f;
    }

private:
    float sampleRate_ = 48000.0f;
    float A0_ = 100.0f;
    float b1_ = 0.0f;
    float oneMinusB1_ = 1.0f;
    float slewLimitPerSample_ = 10.0f;
    float Vrail_ = 13.0f;
    float Vos_ = 0.001f;
    float openLoopState_ = 0.0f;
    float prevOutput_ = 0.0f;
};
