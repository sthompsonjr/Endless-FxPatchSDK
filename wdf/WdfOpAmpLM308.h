#pragma once

#include <algorithm>
#include <cmath>

/// LM308 Op-Amp open-loop signal model.
///
/// NOT a WDF element — a signal processing block that takes differential
/// input voltage (volts) and returns output voltage (volts).
///
/// The LM308 is the heart of the ProCo RAT distortion pedal. Its defining
/// characteristic is the slow slew rate (0.3 V/µs vs LM741's 0.5 V/µs),
/// which produces the RAT's signature aggressive, buzzy distortion.
///
/// The compensation capacitor (pin 1-8, default 30pF) controls slew rate.
/// The RAT uses an external 30pF cap; modifying this changes the distortion
/// character significantly.
///
/// Models:
///   - Open-loop gain: A0 = 100 (effective, same as LM741 model)
///   - Dominant pole: 100 Hz (GBW = 10 kHz)
///   - Slew rate: 0.3 V/µs (slower than LM741's 0.5 V/µs)
///   - Output rail: ±7.5V (9V battery − 1.5V dropout)
///   - Input offset voltage: 2mV typical (worse than LM741's 1mV)
///
/// Additional features beyond LM741:
///   - Compensation capacitor modulation (changes slew rate)
///   - Aging simulation (degraded A0, worse Vos, slower slew)
///   - Soft rail character (gradual compression before hard clip)
class WdfOpAmpLM308 {
public:
    void init(float sampleRate) {
        sampleRate_ = sampleRate;

        // Open-loop gain — same stability-reduced value as LM741
        A0_ = 100.0f;
        baseA0_ = 100.0f;

        // Dominant pole at 100 Hz
        b1_ = expf(-6.283185307f * 100.0f / sampleRate_);
        oneMinusB1_ = 1.0f - b1_;

        // Slew rate: 0.3 V/µs = 300000 V/s (key LM308 characteristic)
        baseSlewRate_ = 300000.0f;
        slewLimitPerSample_ = baseSlewRate_ / sampleRate_;

        // Rail voltage: 9V battery minus ~1.5V dropout
        Vrail_ = 7.5f;

        // Input offset voltage: 2mV typical for LM308
        Vos_ = 0.002f;
        baseVos_ = 0.002f;

        // Default comp cap: 30pF (stock RAT value)
        compCapPF_ = 30.0f;

        // No aging or character by default
        age_ = 0.0f;
        character_ = 0.0f;

        openLoopState_ = 0.0f;
        prevOutput_ = 0.0f;
    }

    /// Set rail voltage for dying battery effect.
    /// Default 7.5V. Lower values produce saggy, compressed distortion.
    void setRailVoltage(float Vrail) {
        Vrail_ = std::max(Vrail, 0.1f);
    }

    /// Set compensation capacitor value in picofarads.
    /// Stock RAT = 30pF. Lower = faster slew (brighter), higher = slower (darker).
    /// Range: 10–100pF.
    void setCompCapPF(float pf) {
        compCapPF_ = std::clamp(pf, 10.0f, 100.0f);
        updateSlewRate();
    }

    /// Set aging amount (0–1).
    /// 0 = new chip, 1 = heavily aged.
    /// Reduces A0 by 30%, increases Vos 5×, slows slew by 40%.
    void setAge(float age) {
        age_ = std::clamp(age, 0.0f, 1.0f);
        // Interpolate parameters
        A0_ = baseA0_ * (1.0f - 0.3f * age_);
        Vos_ = baseVos_ * (1.0f + 4.0f * age_);
        updateSlewRate();
    }

    /// Set soft rail character (0–1).
    /// 0 = hard rail clamp (stock), 1 = gradual compression before clamp.
    void setCharacter(float character) {
        character_ = std::clamp(character, 0.0f, 1.0f);
    }

    [[nodiscard]] float process(float vDiff) noexcept {
        // 1. Add input offset voltage
        float vEff = vDiff + Vos_;

        // 2. Open-loop amplification
        float vAmplified = A0_ * vEff;

        // 3. Clamp to prevent float overflow before IIR filter
        vAmplified = std::clamp(vAmplified, -1e4f, 1e4f);

        // 4. Dominant pole IIR (100 Hz single pole)
        openLoopState_ = oneMinusB1_ * vAmplified + b1_ * openLoopState_;

        // 4b. Clamp IIR state to prevent accumulated drift
        openLoopState_ = std::clamp(openLoopState_, -2.0f * Vrail_, 2.0f * Vrail_);

        // 5. Slew rate limiting (THE defining LM308 characteristic)
        float delta = openLoopState_ - prevOutput_;
        delta = std::clamp(delta, -slewLimitPerSample_, slewLimitPerSample_);

        // 6. Apply slew-limited output
        float vOut = prevOutput_ + delta;

        // 7. Rail clamping — hard or soft depending on character_
        if (character_ > 0.001f) {
            // Soft rail: tanh-like compression before hard clamp
            // At character_=1, the soft zone extends 2V below the rail
            float softZone = character_ * 2.0f;
            float softRail = Vrail_ - softZone;
            if (vOut > softRail) {
                float excess = (vOut - softRail) / (softZone + 0.001f);
                vOut = softRail + softZone * (excess / (1.0f + fabsf(excess)));
            } else if (vOut < -softRail) {
                float excess = (vOut + softRail) / (softZone + 0.001f);
                vOut = -softRail + softZone * (excess / (1.0f + fabsf(excess)));
            }
            vOut = std::clamp(vOut, -Vrail_, Vrail_);
        } else {
            vOut = std::clamp(vOut, -Vrail_, Vrail_);
        }

        // 8. Store for next sample
        prevOutput_ = vOut;

        return vOut;
    }

    void reset() noexcept {
        openLoopState_ = 0.0f;
        prevOutput_ = 0.0f;
    }

private:
    void updateSlewRate() {
        // Slew rate scales inversely with comp cap: SR = baseSR × (30/compCap)
        // Aging slows slew by up to 40%
        float capFactor = 30.0f / compCapPF_;
        float ageFactor = 1.0f - 0.4f * age_;
        slewLimitPerSample_ = (baseSlewRate_ * capFactor * ageFactor) / sampleRate_;
    }

    float sampleRate_ = 48000.0f;
    float A0_ = 100.0f;
    float baseA0_ = 100.0f;
    float b1_ = 0.0f;
    float oneMinusB1_ = 1.0f;
    float baseSlewRate_ = 300000.0f;
    float slewLimitPerSample_ = 6.25f;
    float Vrail_ = 7.5f;
    float Vos_ = 0.002f;
    float baseVos_ = 0.002f;
    float compCapPF_ = 30.0f;
    float age_ = 0.0f;
    float character_ = 0.0f;
    float openLoopState_ = 0.0f;
    float prevOutput_ = 0.0f;
};
