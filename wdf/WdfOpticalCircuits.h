#pragma once

#include "WdfPhotoresistor.h"
#include "WdfOnePort.h"
#include "WdfOta.h"
#include "../dsp/EnvelopeFollower.h"
#include "../dsp/OnePoleFilter.h"
#include "../dsp/ParameterSmoother.h"
#include "../dsp/Saturation.h"
#include <algorithm>
#include <cmath>

/// PC2ACircuit — Optical Leveling Amplifier (LA-2A / Effectrode PC-2A style).
///
/// Signal path:
///   Input → VCA (LDR shunt divider: Rldr/(Rref+Rldr)) → Makeup gain → Output
///
/// Voltage divider topology:
///   Vin ──[Rref]──┬── Vout
///                 │
///               [Rldr]
///                 │
///                GND
///
///   Vout = Vin × Rldr / (Rref + Rldr)
///   At rest (Rldr=R_dark=1MΩ):  ratio ≈ 0.91 — minimal attenuation
///   Compressed (Rldr=R_bright=200Ω): ratio ≈ 0.002 — heavy attenuation
///
/// Sidechain path:
///   Input → [optional HF highpass emphasis] → |envelope| → sat::softClip → light level → LDR
///
/// The LDR in the shunt position means: more light → lower Rldr → lower Vout → more attenuation.
/// This is correct compressor behavior: louder signal → more compression.
///
/// Controls:
///   peakReduction: 0.0–1.0 — sidechain sensitivity (how hard the LED is driven)
///   gain:          0.0–1.0 — manual makeup gain
///   hfEmphasis:   0.0–1.0 — sidechain treble weighting (0=flat, 1=treble-focused)
class PC2ACircuit {
public:
    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;

        WdfPhotoresistor::Params ldrParams;
        ldrParams.sampleRate = sampleRate;
        ldr_.init(ldrParams);

        rRef_.init(kRref);

        detector_.init(sampleRate);
        detector_.setAttackMs(0.1f);   // very fast attack — optical envelope tracks signal
        detector_.setReleaseMs(50.0f);
        detector_.setMode(EnvelopeFollower::Mode::Peak);

        hfEmphasisFilter_.init(sampleRate);
        hfEmphasisFilter_.setType(OnePoleFilter::Type::Highpass);
        hfEmphasisFilter_.setFrequency(2000.0f);

        peakReductionSmoother_.init(sampleRate, 20.0f);
        peakReductionSmoother_.snapTo(0.5f);
        gainSmoother_.init(sampleRate, 20.0f);
        gainSmoother_.snapTo(0.5f);
        hfSmoother_.init(sampleRate, 50.0f);
        hfSmoother_.snapTo(0.0f);
    }

    /// Peak reduction: 0.0–1.0 → sidechain gain scaling.
    void setPeakReduction(float peakReduction) noexcept {
        peakReductionSmoother_.setTarget(std::clamp(peakReduction, 0.0f, 1.0f));
    }

    /// Gain: 0.0–1.0 → makeup output gain (manual).
    void setGain(float gain) noexcept {
        gainSmoother_.setTarget(std::clamp(gain, 0.0f, 1.0f));
    }

    /// HF emphasis: 0.0–1.0 → sidechain highpass weighting.
    void setHFEmphasis(float hfEmphasis) noexcept {
        hfSmoother_.setTarget(std::clamp(hfEmphasis, 0.0f, 1.0f));
    }

    [[nodiscard]] float process(float input) noexcept {
        float peakReduction = peakReductionSmoother_.process();
        float gain          = gainSmoother_.process();
        float hfEmphasis    = hfSmoother_.process();

        // Sidechain: blend flat + highpass-emphasized signals
        float hfSide = hfEmphasisFilter_.process(input);
        float sidechain = input + hfEmphasis * (hfSide - input);

        // Envelope detection
        float envelope = detector_.process(sidechain);

        // Map envelope to light level
        float light = sidechainToLight(envelope, peakReduction);

        // Update LDR
        ldr_.setLightLevel(light);

        // Apply voltage divider attenuation (Rldr in shunt: more light → more attenuation)
        float out = applyAttenuation(input);

        // Makeup gain (0 = silent, 1 = unity × 2 for headroom)
        out *= gain * 2.0f;

        return std::clamp(out, -1.0f, 1.0f);
    }

    void processStereo(float inL, float inR, float& outL, float& outR) noexcept {
        float peakReduction = peakReductionSmoother_.process();
        float gain          = gainSmoother_.process();
        float hfEmphasis    = hfSmoother_.process();

        // Linked compression: use max of both channels for sidechain
        float sidechainL = inL + hfEmphasis * (hfEmphasisFilter_.process(inL) - inL);
        float sidechainR = inR;  // simplified: use L channel's HF filter for both

        float envL = detector_.process(sidechainL);
        // For linked stereo, use the peak of both channels
        float monoEnv = std::max(fabsf(inL), fabsf(inR));
        (void)envL;
        (void)sidechainR;

        // Use mono envelope for linked GR
        float light = sidechainToLight(monoEnv, peakReduction);
        ldr_.setLightLevel(light);

        // Shunt divider: Rldr / (Rref + Rldr) — more light → lower Rldr → more attenuation
        float Rldr = ldr_.getCurrentResistance();
        float ratio = Rldr / (kRref + Rldr);
        float makeup = gain * 2.0f;

        outL = std::clamp(inL * ratio * makeup, -1.0f, 1.0f);
        outR = std::clamp(inR * ratio * makeup, -1.0f, 1.0f);
    }

    void reset() noexcept {
        ldr_.reset();
        detector_.reset();
        hfEmphasisFilter_.reset();
        peakReductionSmoother_.snapTo(0.5f);
        gainSmoother_.snapTo(0.5f);
        hfSmoother_.snapTo(0.0f);
    }

    /// Gain reduction in dB (negative value = attenuation applied).
    /// Uses shunt-divider formula: ratio = Rldr / (Rref + Rldr).
    [[nodiscard]] float getGainReductionDb() const noexcept {
        float Rldr = ldr_.getCurrentResistance();
        float ratio = std::max(Rldr / (kRref + Rldr), 1e-6f);
        return 20.0f * log10f(ratio);
    }

    /// LDR inertia: 0.0 (fully recovered) to 1.0 (fully driven).
    [[nodiscard]] float getLdrInertia() const noexcept {
        return ldr_.getInertia();
    }

private:
    WdfPhotoresistor ldr_;
    WdfResistor      rRef_;  // kept for circuit documentation; value used in kRref constant

    EnvelopeFollower  detector_;
    OnePoleFilter     hfEmphasisFilter_;
    ParameterSmoother peakReductionSmoother_;
    ParameterSmoother gainSmoother_;
    ParameterSmoother hfSmoother_;

    float sampleRate_ = 48000.0f;

    static constexpr float kRref = 100000.0f;  // 100kΩ series resistor

    /// Maps peak reduction control + sidechain level → LED light level.
    [[nodiscard]] float sidechainToLight(float envelope, float peakReduction) const noexcept {
        float driven = envelope * peakReduction * 5.0f;
        float light = fabsf(sat::softClip(driven));
        return std::clamp(light, 0.0f, 1.0f);
    }

    /// Shunt voltage divider: Vout = Vin × Rldr / (Rref + Rldr).
    /// More light → lower Rldr → lower Vout → more attenuation (compressor behavior).
    [[nodiscard]] float applyAttenuation(float input) const noexcept {
        float Rldr = ldr_.getCurrentResistance();
        float ratio = Rldr / (kRref + Rldr);
        return input * ratio;
    }
};

// ============================================================

/// OpticalLevelerCircuit — Single-Knob Optical Leveler.
///
/// A simplified optical compressor inspired by the original LA-2A "set it and
/// forget it" philosophy: one knob adjusts compression intensity, and the
/// program-dependent LDR response handles the character.
///
/// Threshold maps to peak reduction. Makeup gain is automatically adjusted
/// inversely — more compression gives slightly more makeup gain.
class OpticalLevelerCircuit {
public:
    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;
        pc2a_.init(sampleRate);
        pc2a_.setGain(0.5f);  // default center gain
    }

    /// Threshold: 0.0 = gentle limiting, 1.0 = aggressive compression.
    void setThreshold(float threshold) noexcept {
        float t = std::clamp(threshold, 0.0f, 1.0f);
        pc2a_.setPeakReduction(t);
        // More compression → slightly more makeup to keep overall level
        pc2a_.setGain(0.5f + t * 0.15f);
    }

    [[nodiscard]] float process(float input) noexcept {
        return pc2a_.process(input);
    }

    void reset() noexcept {
        pc2a_.reset();
        pc2a_.setGain(0.5f);
    }

private:
    PC2ACircuit pc2a_;
    float sampleRate_ = 48000.0f;
};

// ============================================================

/// HybridOptOtaCircuit — Optical Detector + OTA Gain Element.
///
/// Uses WdfPhotoresistor's slowly-changing resistance to control WdfOta's
/// Iabc via a current mirror (log-linear mapping). The optical detector
/// provides program-dependent time constants; the OTA provides precise,
/// low-noise gain control.
///
/// The LDR resistance maps to Iabc as:
///   norm  = (R_dark - Rldr) / (R_dark - R_bright)  // 0=dark, 1=bright
///   Iabc  = Iabc_max × (Iabc_min/Iabc_max)^norm     // log-linear
///
/// This creates a VCA gain proportional to how much the LDR has been
/// illuminated, with the same program-dependent timing as the optical model.
class HybridOptOtaCircuit {
public:
    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;

        WdfPhotoresistor::Params ldrParams;
        ldrParams.sampleRate = sampleRate;
        ldr_.init(ldrParams);

        ota_.init(kIabcMax, kIabcMin, sampleRate);

        detector_.init(sampleRate);
        detector_.setAttackMs(5.0f);
        detector_.setReleaseMs(50.0f);
        detector_.setMode(EnvelopeFollower::Mode::Peak);

        sensitivitySmoother_.init(sampleRate, 20.0f);
        sensitivitySmoother_.snapTo(0.5f);
        gainSmoother_.init(sampleRate, 20.0f);
        gainSmoother_.snapTo(0.5f);

        tauFast_    = ldr_params::tau_fast;
        tauRelease_ = ldr_params::tau_release;
    }

    void setSensitivity(float sensitivity) noexcept {
        sensitivitySmoother_.setTarget(std::clamp(sensitivity, 0.0f, 1.0f));
    }

    void setMakeupGain(float gain) noexcept {
        gainSmoother_.setTarget(std::clamp(gain, 0.0f, 1.0f));
    }

    /// Attack: 0.0=fast (5ms), 1.0=slow (50ms). Modifies LDR tau_fast.
    void setAttack(float attack) noexcept {
        tauFast_ = 0.005f + std::clamp(attack, 0.0f, 1.0f) * 0.045f;
        rebuildLdr();
    }

    /// Release: 0.0=short (200ms), 1.0=long (2000ms). Modifies LDR tau_release.
    void setRelease(float release) noexcept {
        tauRelease_ = 0.2f + std::clamp(release, 0.0f, 1.0f) * 1.8f;
        rebuildLdr();
    }

    [[nodiscard]] float process(float input) noexcept {
        float sensitivity = sensitivitySmoother_.process();
        float makeup      = gainSmoother_.process();

        // Sidechain
        float envelope = detector_.process(input);
        float light    = sat::softClip(fabsf(envelope) * sensitivity * 5.0f);
        light = std::clamp(light, 0.0f, 1.0f);

        // Update LDR
        ldr_.setLightLevel(light);

        // Map LDR resistance to OTA Iabc
        float Iabc = resistanceToIabc(ldr_.getCurrentResistance());
        ota_.setIabc(Iabc);

        // Apply VCA then makeup
        float out = ota_.process(input);
        return std::clamp(out * makeup * 2.0f, -1.0f, 1.0f);
    }

    void reset() noexcept {
        ldr_.reset();
        ota_.reset();
        detector_.reset();
        sensitivitySmoother_.snapTo(0.5f);
        gainSmoother_.snapTo(0.5f);
    }

private:
    WdfPhotoresistor  ldr_;
    WdfOta            ota_;
    EnvelopeFollower  detector_;
    ParameterSmoother sensitivitySmoother_;
    ParameterSmoother gainSmoother_;
    float sampleRate_  = 48000.0f;
    float tauFast_     = ldr_params::tau_fast;
    float tauRelease_  = ldr_params::tau_release;

    static constexpr float kIabcMax = 1.0f;
    static constexpr float kIabcMin = 0.01f;

    /// Maps LDR resistance to OTA Iabc (log-linear).
    /// R_dark (1MΩ) → Iabc_max (full gain, no compression)
    /// R_bright (200Ω) → Iabc_min (maximum gain reduction)
    [[nodiscard]] float resistanceToIabc(float Rldr) const noexcept {
        float range = ldr_params::R_dark - ldr_params::R_bright;
        if (range < 1.0f) return kIabcMax;
        float norm = (ldr_params::R_dark - Rldr) / range;
        norm = std::clamp(norm, 0.0f, 1.0f);
        // Log-linear: Iabc = Iabc_max * (Iabc_min/Iabc_max)^norm
        float logRatio = logf(kIabcMin / kIabcMax);
        return kIabcMax * expf(logRatio * norm);
    }

    /// Rebuild LDR with updated tau values (preserves current state via init).
    void rebuildLdr() noexcept {
        WdfPhotoresistor::Params params;
        params.sampleRate   = sampleRate_;
        params.tau_fast     = tauFast_;
        params.tau_release  = tauRelease_;
        // Preserve other defaults
        float prevLight = ldr_.getCurrentLightLevel();
        ldr_.init(params);
        // Snap to previous light level so state is not lost abruptly
        // (approximate — full state reset would require saving R_fast/R_slow)
        if (prevLight > 0.0f) {
            ldr_.setLightLevel(prevLight);
        }
    }
};
