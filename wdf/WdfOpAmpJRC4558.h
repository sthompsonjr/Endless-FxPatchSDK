#pragma once

#include <algorithm>
#include <cmath>

/// JRC4558 Dual Op-Amp open-loop signal model.
///
/// NOT a WDF element — a signal processing block that takes differential
/// input voltage (volts) and returns output voltage (volts).
///
/// The JRC4558 (Japan Radio Corporation RC4558) is the classic op-amp used
/// in the Ibanez TS808 Tubescreamer and many other overdrive pedals.
///
/// Physical vs model parameter derivation:
///
///   Parameter  | LM741 physical | LM741 model | JRC4558 physical | JRC4558 model
///   -----------|----------------|-------------|------------------|---------------
///   A0         | 200,000        | 100         | 100,000          | 50
///   fp         | 5 Hz           | 100 Hz      | 30 Hz            | 300 Hz
///   SR (V/s)   | 0.5 V/µs       | 500,000/sr  | ~0.32 V/µs       | 322,000/sr
///   Vrail      | 13 V           | 13 V        | 13 V             | 13 V
///   Vos        | 1 mV           | 0.001 V     | 2 mV             | 0.002 V
///
/// fp rationale: LM741 physical fp=5 Hz raised 20× to model fp=100 Hz.
///   JRC4558 physical fp=30 Hz is 6× higher than LM741 (30/5 = 6).
///   Applying same relative scaling: 100 Hz × 3 = 300 Hz model fp.
///   Model GBW = 50 × 300 = 15 kHz vs LM741's 10 kHz — JRC4558 has
///   higher small-signal bandwidth (brighter, more present in the mix).
///
/// A0 rationale: stability constraint A0 × β_max × (1-b1) < 2 must hold.
///   With fp=300 Hz at 48 kHz: b1 = exp(-2π*300/48000) ≈ 0.9611
///   (1-b1) ≈ 0.0389, β_max ≈ 0.94 for TS808 feedback network.
///   A0=100: 100 × 0.94 × 0.039 ≈ 3.7 > 2 → UNSTABLE.
///   A0=50:   50 × 0.94 × 0.039 ≈ 1.83 < 2 → stable. ✓
///   Lower A0 also correctly models harder clipping onset: virtual ground
///   approximation degrades earlier, producing asymmetric soft-knee at
///   high drive settings (characteristic TS808 "grit").
///
/// SR rationale: JRC4558 SR ≈ 1.49 V/µs normalized vs LM741 2.315 V/µs.
///   Ratio = 1.49/2.315 ≈ 0.644. Apply to LM741 model constant:
///   500,000 × 0.644 ≈ 322,000 V/s. This produces the JRC4558's
///   characteristic "rounder attack" — slower transient rise at large-
///   signal levels despite being faster at small-signal levels (GBW).
///   This SR/GBW paradox is physically real and sonically important.
///
/// Creative controls (beyond LM741 interface):
///   setAge(0–1):       Degrades A0 and fp → vintage worn character
///   setCharacter(0–1): Modulates Vos → harmonic asymmetry bias
///   setSlew(0.5–2.0):  Manual slew rate multiplier → attack shape
///
/// All coefficients are precomputed in init() — the per-sample path
/// uses only multiply, add, clamp (no expf/sinf at runtime).
class WdfOpAmpJRC4558 {
public:
    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;
        age_      = 0.0f;
        character_ = 0.0f;
        slewScale_ = 1.0f;
        recomputeCoeffs();
        openLoopState_ = 0.0f;
        prevOutput_    = 0.0f;
    }

    /// Set rail voltage for "dying battery" or lower-supply effect.
    /// Default 13V (from ±15V supply with ~2V dropout).
    void setRailVoltage(float Vrail) noexcept {
        Vrail_ = std::max(Vrail, 0.1f);
    }

    /// Set age/wear factor (0=new, 1=heavily aged).
    /// Reduces effective A0 by up to 50% and fp by up to 30%,
    /// producing a softer, more compressed vintage character.
    void setAge(float age) noexcept {
        age_ = std::clamp(age, 0.0f, 1.0f);
        recomputeCoeffs();
    }

    /// Set character/asymmetry bias (0=nominal, 1=maximum asymmetry).
    /// Increases Vos, introducing even-harmonic content and warm bias.
    void setCharacter(float character) noexcept {
        character_ = std::clamp(character, 0.0f, 1.0f);
        recomputeCoeffs();
    }

    /// Set slew rate multiplier (0.5=slower/rounder, 2.0=faster/sharper).
    /// Values below 1.0 increase the "rounder attack" character.
    void setSlew(float slew) noexcept {
        slewScale_ = std::clamp(slew, 0.1f, 4.0f);
        recomputeCoeffs();
    }

    [[nodiscard]] float process(float vDiff) noexcept {
        // 1. Add input offset voltage (includes character bias)
        float vEff = vDiff + Vos_;

        // 2. Open-loop amplification (age-modulated A0)
        float vAmplified = A0eff_ * vEff;

        // 3. Clamp to prevent float overflow before IIR filter
        vAmplified = std::clamp(vAmplified, -1e4f, 1e4f);

        // 4. Dominant pole IIR (300 Hz base, age-modulated)
        openLoopState_ = oneMinusB1_ * vAmplified + b1_ * openLoopState_;

        // 4b. Clamp IIR state to prevent accumulated drift
        openLoopState_ = std::clamp(openLoopState_, -2.0f * Vrail_, 2.0f * Vrail_);

        // 5. Slew rate limiting (slower than LM741 at large signals)
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
        prevOutput_    = 0.0f;
    }

private:
    void recomputeCoeffs() noexcept {
        // Age degrades open-loop gain (up to -50%) and dominant pole (up to -30%)
        float A0base = 50.0f;
        float fpBase = 300.0f;

        A0eff_ = A0base * (1.0f - 0.5f * age_);
        float fpEff = fpBase * (1.0f - 0.3f * age_);

        b1_          = expf(-6.283185307f * fpEff / sampleRate_);
        oneMinusB1_  = 1.0f - b1_;

        // Slew: base 322,000 V/s, scaled by slewScale_ and age (age → slower)
        float slewBase = 322000.0f * (1.0f - 0.4f * age_);
        slewLimitPerSample_ = slewBase * slewScale_ / sampleRate_;

        // Character increases Vos for asymmetric harmonic content
        Vos_ = 0.002f + character_ * 0.008f;
    }

    float sampleRate_        = 48000.0f;
    float A0eff_             = 50.0f;
    float b1_                = 0.0f;
    float oneMinusB1_        = 1.0f;
    float slewLimitPerSample_ = 0.0f;
    float Vrail_             = 13.0f;
    float Vos_               = 0.002f;
    float age_               = 0.0f;
    float character_         = 0.0f;
    float slewScale_         = 1.0f;
    float openLoopState_     = 0.0f;
    float prevOutput_        = 0.0f;
};
