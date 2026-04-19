#pragma once

#include "WdfInvertingStage.h"
#include "WdfBigMuffToneStack.h"
#include "BiquadFilter.h"
#include <algorithm>

// Op-Amp Big Muff Pi (EHX Version 4) — two-stage op-amp inverting clipper
// with anti-parallel 1N914 diode feedback and Big Muff tone stack.
//
// Signal flow:
//   Input -> [Stage 1: WdfInvertingStage] -> [BiquadFilter HP, AC coupling] ->
//            [Stage 2: WdfInvertingStage] -> [WdfBigMuffToneStack | bypass] ->
//            [volume scale] -> Output
//
// The sustain control scales the feedback resistance (R_fb) of both stages
// simultaneously, from R_fb_min to R_fb_max. Both clipping stages share the
// same R_fb at any instant.
//
// Tone bypass: when active, the tone stack is skipped, producing a wider,
// brighter sound (mod popularised on the V5 Violet Ram's Head).

namespace opamp_big_muff_params {

    // Common per-stage values
    static constexpr float R_in       = 10000.0f;   // Ω — input resistor (10kΩ)
    static constexpr float R_fb_min   = 1000.0f;    // Ω — sustain min (1kΩ, ~unity gain)
    static constexpr float R_fb_max   = 100000.0f;  // Ω — sustain max (100kΩ, max gain)
    static constexpr float C_fb       = 1e-10f;     // F — 100pF stability cap

    // 1N914 diode parameters (anti-parallel pair in feedback)
    static constexpr float diode_Is   = 2.52e-9f;   // A — saturation current
    static constexpr float diode_n    = 1.752f;     // — ideality factor (documented)
    static constexpr float diode_Vt   = 0.02585f;   // V — thermal voltage at 20°C

    // Inter-stage AC coupling (BiquadFilter highpass)
    static constexpr float C_couple   = 100e-9f;    // F — physical coupling cap
    static constexpr float fc_couple  = 159.15f;    // Hz — equivalent corner
    static constexpr float Q_couple   = 0.707f;     // Butterworth

    // Output DC block (models 100nF output coupling cap → ~16 Hz with 100k load).
    // Required because each LM741 stage contributes ~1 mV of input offset,
    // amplified by the closed-loop gain and partially passed by the tone stack.
    static constexpr float fc_dcblock = 20.0f;      // Hz
    static constexpr float Q_dcblock  = 0.707f;     // Butterworth

    // Documentation-only physical references
    static constexpr float V_supply   = 9.0f;       // V — pedal supply

} // namespace opamp_big_muff_params

class WdfOpAmpBigMuffCircuit {
public:
    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;

        stage1_.init(opamp_big_muff_params::R_in,
                     opamp_big_muff_params::R_fb_min,
                     opamp_big_muff_params::C_fb,
                     opamp_big_muff_params::diode_Is,
                     opamp_big_muff_params::diode_Vt,
                     sampleRate);
        stage2_.init(opamp_big_muff_params::R_in,
                     opamp_big_muff_params::R_fb_min,
                     opamp_big_muff_params::C_fb,
                     opamp_big_muff_params::diode_Is,
                     opamp_big_muff_params::diode_Vt,
                     sampleRate);

        acCoupling_.init(sampleRate);
        acCoupling_.setParameters(BiquadFilter::Type::Highpass,
                                  opamp_big_muff_params::fc_couple,
                                  opamp_big_muff_params::Q_couple);

        outputDcBlock_.init(sampleRate);
        outputDcBlock_.setParameters(BiquadFilter::Type::Highpass,
                                     opamp_big_muff_params::fc_dcblock,
                                     opamp_big_muff_params::Q_dcblock);

        toneStack_.init(sampleRate, WdfBigMuffToneStack::Variant::OpAmp);

        currentRfb_ = opamp_big_muff_params::R_fb_min;
    }

    void reset() noexcept {
        stage1_.reset();
        stage2_.reset();
        acCoupling_.reset();
        outputDcBlock_.reset();
        toneStack_.reset();
        currentRfb_ = opamp_big_muff_params::R_fb_min;
        // Pre-warm: each LM741 has a 1 mV input offset that gets amplified
        // and slowly bleeds out through the AC-coupling and output DC-block
        // highpass filters. Run silent samples until the chain has settled.
        for (int i = 0; i < 4096; ++i) {
            (void)processInternal(0.0f);
        }
    }

    [[nodiscard]] float process(float x) noexcept {
        return processInternal(x) * volume_;
    }

private:
    [[nodiscard]] float processInternal(float x) noexcept {
        const float y1  = stage1_.process(x);
        const float y1c = acCoupling_.process(y1);
        const float y2  = stage2_.process(y1c);
        const float toneOut = toneBypass_ ? y2 : toneStack_.process(y2);
        return outputDcBlock_.process(toneOut);
    }

public:

    void setSustain(float sustain) noexcept {
        const float s = std::clamp(sustain, 0.0f, 1.0f);
        currentRfb_ = opamp_big_muff_params::R_fb_min
                    + s * (opamp_big_muff_params::R_fb_max
                         - opamp_big_muff_params::R_fb_min);
        stage1_.setFeedbackResistance(currentRfb_);
        stage2_.setFeedbackResistance(currentRfb_);
    }

    void setTone(float tone) noexcept {
        toneStack_.setTone(std::clamp(tone, 0.0f, 1.0f));
    }

    void setVolume(float volume) noexcept {
        volume_ = std::clamp(volume, 0.0f, 1.0f);
    }

    void setToneBypass(bool bypass) noexcept {
        toneBypass_ = bypass;
    }

private:
    WdfInvertingStage   stage1_;
    WdfInvertingStage   stage2_;
    BiquadFilter        acCoupling_;
    BiquadFilter        outputDcBlock_;
    WdfBigMuffToneStack toneStack_;

    float sampleRate_ = 48000.0f;
    float volume_     = 0.7f;
    bool  toneBypass_ = false;
    float currentRfb_ = opamp_big_muff_params::R_fb_min;
};
