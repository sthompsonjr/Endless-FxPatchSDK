#pragma once

#include "WdfNonlinear.h"
#include "WdfOpAmpLM308.h"
#include "../dsp/ParameterSmoother.h"
#include "../dsp/OnePoleFilter.h"
#include "../dsp/Saturation.h"
#include <algorithm>
#include <cmath>

/// ProCo RAT variant selector.
enum class RatVariant {
    Original,      ///< 1N914 single diode — asymmetric +0.6V / −rail
    TurboRat,      ///< LED single diode — asymmetric +1.6V / −rail
    YouDirtyRat,   ///< Antiparallel 1N914 — symmetric ±0.6V
    WhiteRat,      ///< No diodes — symmetric ±rail only
    GermaniumMod,  ///< Germanium single diode — asymmetric +0.3V / −rail
    kCount
};

/// ProCo RAT Distortion Pedal — complete circuit model.
///
/// Built around an LM308 op-amp in inverting configuration with:
///   R_in = 1kΩ input resistor
///   R_dist = 100Ω–1MΩ gain pot (logarithmic taper, Distortion knob)
///   C_feedback = 47pF feedback capacitor
///   D1 = variant-dependent diode clipping to ground
///
/// The RAT's signature sound comes from:
///   1. LM308's slow slew rate (0.3 V/µs) creating aggressive harmonics
///   2. Single-diode asymmetric clipping (Original variant)
///   3. High gain range (up to 1000:1 ratio)
///
/// 5 variants model different RAT pedal versions and popular mods.
///
/// Filter control is REVERSED from convention:
///   0.0 = bright (20kHz cutoff), 1.0 = dark (2kHz cutoff)
///   This matches the original RAT's "Filter" knob behavior.
class WdfRatCircuit {
public:
    void init(float sampleRate) {
        sampleRate_ = sampleRate;

        // Init LM308
        opamp_.init(sampleRate);

        // Init parameter smoothers (20ms)
        distortionSmoother_.init(sampleRate, 20.0f);
        distortionSmoother_.snapTo(0.5f);
        volumeSmoother_.init(sampleRate, 20.0f);
        volumeSmoother_.snapTo(0.5f);

        // Init IIR coefficients with default Rdist
        currentRdist_ = mapDistortionToRdist(0.5f);
        updateIIRCoeffs(currentRdist_);

        // Init diodes for all types — port resistance = Rdist
        singleDiode_ = WdfDiodeToGround::make1N914(currentRdist_);
        antiparallelDiodes_.init(1e-7f, 0.02585f, currentRdist_);

        // Init tone filter: two cascaded lowpass for −12dB/oct
        toneFilter1_.init(sampleRate);
        toneFilter1_.setType(OnePoleFilter::Type::Lowpass);
        toneFilter1_.setFrequency(20000.0f);
        toneFilter2_.init(sampleRate);
        toneFilter2_.setType(OnePoleFilter::Type::Lowpass);
        toneFilter2_.setFrequency(20000.0f);

        // DC blocking filter
        dcBlocker_.init(sampleRate);
        dcBlocker_.setType(OnePoleFilter::Type::DCBlock);

        // Precompute log ratio for distortion mapping
        logRdistRatio_ = logf(kRdistMin / kRdistMax);

        // Default variant
        applyVariant(RatVariant::Original);

        // Clear crossfade state
        crossfadeCounter_ = 0;
        crossfadeEnvelope_ = 1.0f;

        iirState_ = 0.0f;
        xPrev_ = 0.0f;
    }

    /// Set distortion amount (0–1). Log taper to Rdist: 1MΩ (clean) → 100Ω (max).
    void setDistortion(float distortion) {
        distortionSmoother_.setTarget(std::clamp(distortion, 0.0f, 1.0f));
    }

    /// Set filter cutoff (0–1). REVERSED: 0 = bright (20kHz), 1 = dark (2kHz).
    void setFilter(float filter) {
        filter = std::clamp(filter, 0.0f, 1.0f);
        // Reversed: 0→20kHz, 1→2kHz. freq = 20000 × 0.1^filter
        float freq = 20000.0f * powf(0.1f, filter);
        freq = std::min(freq, sampleRate_ * 0.45f);
        toneFilter1_.setFrequency(freq);
        toneFilter2_.setFrequency(freq);
    }

    /// Set output volume (0–1).
    void setVolume(float volume) {
        volumeSmoother_.setTarget(std::clamp(volume, 0.0f, 1.0f));
    }

    /// Switch RAT variant. Triggers a crossfade to prevent pops.
    void setVariant(RatVariant variant) {
        if (variant == currentVariant_) return;
        pendingVariant_ = variant;
        crossfadeCounter_ = kCrossfadeSamples * 2; // fade out + fade in
    }

    /// Set slew character via compensation capacitor.
    /// 0 = stock (30pF), 1 = fast (15pF), 2 = slow (60pF).
    void setSlewCharacter(int mode) {
        switch (mode) {
            case 0: opamp_.setCompCapPF(30.0f); break;
            case 1: opamp_.setCompCapPF(15.0f); break;
            case 2: opamp_.setCompCapPF(60.0f); break;
            default: break;
        }
    }

    /// Forward aging to LM308.
    void setAge(float age) {
        opamp_.setAge(age);
    }

    /// Get the current variant.
    [[nodiscard]] RatVariant getVariant() const { return currentVariant_; }

    [[nodiscard]] float process(float input) noexcept {
        // 1. Smooth parameters
        float dist = distortionSmoother_.process();
        float volume = volumeSmoother_.process();

        // 2. Update Rdist if changed significantly
        float Rdist = mapDistortionToRdist(dist);
        if (fabsf(Rdist - currentRdist_) / currentRdist_ > 0.001f) {
            updateIIRCoeffs(Rdist);
            currentRdist_ = Rdist;
            // Reinit diode port resistances
            reinitDiodes(Rdist);
        }

        // 3. Scale input to circuit voltage (~100mV guitar signal)
        float vIn = input * kInputScale;

        // 4. IIR closed-loop transfer: H(s) = -(Rdist/Rin) / (1 + s·Rdist·Cfeedback)
        float vIdeal = iirA0_ * (vIn + xPrev_) + iirB1_ * iirState_;
        xPrev_ = vIn;
        iirState_ = vIdeal;

        // 5. Diode clipping (variant-dependent)
        float vClipped = applyDiodeClipping(vIdeal);

        // 6. LM308 slew rate + rail clamping
        float vOut = opamp_.process(-vClipped);

        // 7. Scale back to normalized audio
        float out = vOut * kOutputScale;

        // 8. DC blocking
        out = dcBlocker_.process(out);

        // 9. Two cascaded lowpass filters (−12dB/oct)
        out = toneFilter1_.process(out);
        out = toneFilter2_.process(out);

        // 10. Volume
        out *= volume;

        // 11. Safety soft clip
        out = sat::softClip(out);

        // 12. Crossfade envelope (if switching variants)
        if (crossfadeCounter_ > 0) {
            out *= crossfadeEnvelope_;
            updateCrossfade();
        }

        return out;
    }

    void reset() noexcept {
        opamp_.reset();
        singleDiode_.reset();
        antiparallelDiodes_.reset();
        toneFilter1_.reset();
        toneFilter2_.reset();
        dcBlocker_.reset();
        distortionSmoother_.snapTo(0.5f);
        volumeSmoother_.snapTo(0.5f);
        iirState_ = 0.0f;
        xPrev_ = 0.0f;
        crossfadeCounter_ = 0;
        crossfadeEnvelope_ = 1.0f;
        currentRdist_ = mapDistortionToRdist(0.5f);
    }

    /// Access the LM308 for advanced parameter tweaking.
    WdfOpAmpLM308& opamp() { return opamp_; }

private:
    /// Map 0–1 distortion knob to Rdist (log taper).
    /// 0 → 1MΩ (clean), 1 → 100Ω (max distortion).
    float mapDistortionToRdist(float dist) const {
        return kRdistMax * expf(dist * logRdistRatio_);
    }

    void updateIIRCoeffs(float Rdist) {
        // Closed-loop gain and time constant
        float gain = Rdist / kRin;
        float tau = Rdist * kCfeedback;

        // Bilinear transform: H(s) = -gain / (1 + s*tau)
        float beta = 2.0f * sampleRate_ * tau;
        float invOnePlusBeta = 1.0f / (1.0f + beta);
        iirA0_ = -gain * invOnePlusBeta;
        iirB1_ = (beta - 1.0f) * invOnePlusBeta;
    }

    void reinitDiodes(float Rdist) {
        // Reinit single diode based on current variant type
        switch (currentVariant_) {
            case RatVariant::Original:
                singleDiode_ = WdfDiodeToGround::make1N914(Rdist);
                break;
            case RatVariant::TurboRat:
                singleDiode_ = WdfDiodeToGround::makeLED(Rdist);
                break;
            case RatVariant::GermaniumMod:
                singleDiode_ = WdfDiodeToGround::makeGermanium(Rdist);
                break;
            case RatVariant::YouDirtyRat:
                antiparallelDiodes_.init(1e-7f, 0.02585f, Rdist);
                break;
            case RatVariant::WhiteRat:
                break; // No diodes to reinit
            default:
                break;
        }
    }

    void applyVariant(RatVariant variant) {
        currentVariant_ = variant;
        switch (variant) {
            case RatVariant::Original:
                useSingleDiode_ = true;
                useNoDiode_ = false;
                singleDiode_ = WdfDiodeToGround::make1N914(currentRdist_);
                break;
            case RatVariant::TurboRat:
                useSingleDiode_ = true;
                useNoDiode_ = false;
                singleDiode_ = WdfDiodeToGround::makeLED(currentRdist_);
                break;
            case RatVariant::YouDirtyRat:
                useSingleDiode_ = false;
                useNoDiode_ = false;
                antiparallelDiodes_.init(1e-7f, 0.02585f, currentRdist_);
                break;
            case RatVariant::WhiteRat:
                useSingleDiode_ = false;
                useNoDiode_ = true;
                break;
            case RatVariant::GermaniumMod:
                useSingleDiode_ = true;
                useNoDiode_ = false;
                singleDiode_ = WdfDiodeToGround::makeGermanium(currentRdist_);
                break;
            default:
                break;
        }
    }

    [[nodiscard]] float applyDiodeClipping(float vIdeal) noexcept {
        if (useNoDiode_) {
            // WhiteRat: no diode clipping, just pass through
            return vIdeal;
        }
        if (useSingleDiode_) {
            // Asymmetric: single diode clips positive half only
            singleDiode_.port.a = 2.0f * vIdeal;
            singleDiode_.reflect();
            return singleDiode_.port.voltage();
        }
        // Antiparallel: symmetric clipping (YouDirtyRat)
        antiparallelDiodes_.port.a = 2.0f * vIdeal;
        antiparallelDiodes_.reflect();
        return antiparallelDiodes_.port.voltage();
    }

    void updateCrossfade() {
        crossfadeCounter_--;
        if (crossfadeCounter_ == kCrossfadeSamples) {
            // Midpoint: switch variant while envelope is at 0
            applyVariant(pendingVariant_);
        }
        if (crossfadeCounter_ > kCrossfadeSamples) {
            // Fading out
            crossfadeEnvelope_ = static_cast<float>(crossfadeCounter_ - kCrossfadeSamples)
                                 / static_cast<float>(kCrossfadeSamples);
        } else {
            // Fading in
            crossfadeEnvelope_ = 1.0f - static_cast<float>(crossfadeCounter_)
                                        / static_cast<float>(kCrossfadeSamples);
        }
    }

    // Circuit components
    WdfOpAmpLM308 opamp_;
    WdfDiodeToGround singleDiode_;
    WdfAntiparallelDiodes antiparallelDiodes_;
    ParameterSmoother distortionSmoother_;
    ParameterSmoother volumeSmoother_;
    OnePoleFilter toneFilter1_;
    OnePoleFilter toneFilter2_;
    OnePoleFilter dcBlocker_;

    // Circuit parameters
    float sampleRate_ = 48000.0f;
    float currentRdist_ = 10000.0f;
    float logRdistRatio_ = 0.0f;

    // RAT component values
    static constexpr float kRin = 1000.0f;          // 1kΩ input resistor
    static constexpr float kCfeedback = 47e-12f;    // 47pF feedback capacitor
    static constexpr float kRdistMin = 100.0f;      // max distortion position
    static constexpr float kRdistMax = 1000000.0f;  // min distortion (clean)

    // I/O scaling
    static constexpr float kInputScale = 0.1f;      // normalized → ~100mV
    static constexpr float kOutputScale = 2.5f;     // scale clipped signal to fill ±1

    // IIR filter state
    float iirA0_ = 0.0f;
    float iirB1_ = 0.0f;
    float iirState_ = 0.0f;
    float xPrev_ = 0.0f;

    // Variant state
    RatVariant currentVariant_ = RatVariant::Original;
    RatVariant pendingVariant_ = RatVariant::Original;
    bool useSingleDiode_ = true;
    bool useNoDiode_ = false;

    // Crossfade for variant switching (128 samples each direction)
    static constexpr int kCrossfadeSamples = 128;
    int crossfadeCounter_ = 0;
    float crossfadeEnvelope_ = 1.0f;
};
