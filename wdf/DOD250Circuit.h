#pragma once

#include "WdfInvertingStage.h"
#include "../dsp/ParameterSmoother.h"
#include "../dsp/OnePoleFilter.h"
#include "../dsp/Saturation.h"
#include <algorithm>
#include <cmath>

/// DOD 250 Overdrive/Preamp — complete circuit model.
///
/// Built around an LM741 op-amp in inverting configuration with:
///   R1 = 68kΩ input resistor
///   Rf = 4.7k–500kΩ gain pot (logarithmic taper)
///   C2 = 47nF feedback capacitor (HF rolloff)
///   1N914 silicon antiparallel clipping diodes in feedback loop
///
/// The gain knob controls Rf, which sets both the closed-loop gain
/// (Rf/R1) and the clipping threshold. At high gain, C2 rolls off
/// the op-amp gain at low frequencies, and the diodes dominate clipping.
/// At low gain, the circuit is nearly clean with gentle compression.
///
/// Enhancement controls beyond the original DOD 250:
///   - Tone: post-clipper lowpass filter (2kHz–20kHz)
///   - Level: output volume
class DOD250Circuit {
public:
    void init(float sampleRate) {
        sampleRate_ = sampleRate;

        // DOD 250 component values:
        // R1=68k, Rf=10k (mid default), C2=47nF, 1N914 silicon diodes
        constexpr float kR1 = 68000.0f;
        constexpr float kRfDefault = 10000.0f;
        constexpr float kC2 = 47e-9f;
        constexpr float kDiodeIs = 1e-7f;
        constexpr float kDiodeVt = 0.02585f;

        stage_.init(kR1, kRfDefault, kC2, kDiodeIs, kDiodeVt, sampleRate);

        // Parameter smoothers (20ms)
        gainSmoother_.init(sampleRate, 20.0f);
        gainSmoother_.snapTo(0.5f);
        levelSmoother_.init(sampleRate, 20.0f);
        levelSmoother_.snapTo(0.5f);

        // Tone filter: post-clipper lowpass
        toneFilter_.init(sampleRate);
        toneFilter_.setType(OnePoleFilter::Type::Lowpass);
        toneFilter_.setFrequency(8000.0f);

        // DC blocking filter
        dcBlocker_.init(sampleRate);
        dcBlocker_.setType(OnePoleFilter::Type::DCBlock);

        currentRf_ = kRfDefault;

        // Precompute log ratio for gain mapping
        logRfRatio_ = logf(kRfMin / kRfMax);
    }

    /// Set gain (0–1). Maps logarithmically to Rf: 500kΩ (clean) → 4.7kΩ (max).
    void setGain(float gain) {
        gainSmoother_.setTarget(std::clamp(gain, 0.0f, 1.0f));
    }

    /// Set output level (0–1).
    void setLevel(float level) {
        levelSmoother_.setTarget(std::clamp(level, 0.0f, 1.0f));
    }

    /// Set tone (0–1). Maps to post-clipper lowpass: 2kHz (dark) → 20kHz (bright).
    void setTone(float tone) {
        tone = std::clamp(tone, 0.0f, 1.0f);
        // Log mapping: 2kHz to 20kHz
        float freq = 2000.0f * powf(10.0f, tone);
        toneFilter_.setFrequency(std::min(freq, sampleRate_ * 0.45f));
    }

    [[nodiscard]] float process(float input) noexcept {
        // 1. Smooth parameters
        float g = gainSmoother_.process();
        float level = levelSmoother_.process();

        // 2. Update Rf if gain changed significantly
        float Rf = kRfMax * expf(g * logRfRatio_);
        if (fabsf(Rf - currentRf_) / currentRf_ > 0.001f) {
            stage_.setFeedbackResistance(Rf);
            currentRf_ = Rf;
        }

        // 3. Scale input to circuit voltage (~100mV guitar signal)
        float vIn = input * kInputScale;

        // 4. Process through WDF inverting stage + LM741
        float vOut = stage_.process(vIn);

        // 5. Scale back to normalized audio
        float out = vOut * kOutputScale;

        // 6. DC blocking
        out = dcBlocker_.process(out);

        // 7. Tone filter
        out = toneFilter_.process(out);

        // 8. Output level
        out *= level;

        // 9. Safety soft clip
        out = sat::softClip(out);

        return out;
    }

    void reset() noexcept {
        stage_.reset();
        toneFilter_.reset();
        dcBlocker_.reset();
        gainSmoother_.snapTo(0.5f);
        levelSmoother_.snapTo(0.5f);
        currentRf_ = 10000.0f;
    }

    /// Access the inverting stage for advanced parameter tweaking.
    WdfInvertingStage& stage() { return stage_; }

private:
    WdfInvertingStage stage_;
    ParameterSmoother gainSmoother_;
    ParameterSmoother levelSmoother_;
    OnePoleFilter toneFilter_;
    OnePoleFilter dcBlocker_;

    float sampleRate_ = 48000.0f;
    float currentRf_ = 10000.0f;
    float logRfRatio_ = 0.0f;

    // Gain pot range
    static constexpr float kRfMin = 4700.0f;     // max gain position
    static constexpr float kRfMax = 500000.0f;    // min gain position

    // I/O scaling
    static constexpr float kInputScale = 0.1f;    // normalized → ~100mV
    static constexpr float kOutputScale = 1.5f;    // clipped ~±0.6V → fill ±1.0
};
