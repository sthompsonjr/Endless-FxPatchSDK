#pragma once

#include "WdfOta.h"
#include "dsp/EnvelopeFollower.h"
#include "dsp/ParameterSmoother.h"
#include <algorithm>
#include <cmath>

/// DynacompCircuit — MXR Dynacomp-style OTA compressor.
///
/// Topology:
///   Input → Envelope Detector (sidechain)
///           ↓
///         Iabc = f(envelope, sensitivity)
///           ↓
///   Input → OTA VCA (gain controlled by Iabc) → Output (× makeup gain)
///
/// The Dynacomp uses a fixed-time-constant envelope follower driving an OTA.
/// Unlike the optical compressor, attack and release times are fixed by RC
/// networks in the hardware and do not vary with program material.
///
/// This fixed-time-constant behavior (attack ~2ms, release ~50ms) produces
/// a punchy, "snap"-heavy character — useful for comparison with the
/// program-dependent optical LDR model.
///
/// Parameter range:
///   sensitivity: 0.0 (gentle) — 1.0 (aggressive)
///   output:      0.0 (silent) — 1.0 (unity/makeup)
class DynacompCircuit {
public:
    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;

        // OTA: full gain when Iabc=1.0, max compression when Iabc=0.01
        ota_.init(1.0f, 0.01f, sampleRate);

        // Envelope detector: fixed Dynacomp-style time constants
        detector_.init(sampleRate);
        detector_.setAttackMs(2.0f);    // fast attack — catches transients
        detector_.setReleaseMs(50.0f);  // short release — pumpy character
        detector_.setMode(EnvelopeFollower::Mode::Peak);

        sensitivitySmoother_.init(sampleRate, 20.0f);
        sensitivitySmoother_.snapTo(0.5f);
        outputSmoother_.init(sampleRate, 20.0f);
        outputSmoother_.snapTo(0.5f);

        currentGainReduction_ = 0.0f;
    }

    void setSensitivity(float sensitivity) noexcept {
        sensitivitySmoother_.setTarget(std::clamp(sensitivity, 0.0f, 1.0f));
    }

    void setOutput(float output) noexcept {
        outputSmoother_.setTarget(std::clamp(output, 0.0f, 1.0f));
    }

    [[nodiscard]] float process(float input) noexcept {
        float sensitivity = sensitivitySmoother_.process();
        float makeup = outputSmoother_.process();

        // Sidechain: envelope detection
        float envelope = detector_.process(input);

        // Map envelope + sensitivity → Iabc
        // At sensitivity=0: very little compression (Iabc stays near max)
        // At sensitivity=1: strong compression (Iabc drops to min for loud signals)
        float driven = envelope * sensitivity * 8.0f; // scale to useful range
        float compression = std::clamp(driven, 0.0f, 1.0f);

        // Iabc: 1.0 (no compression) → 0.01 (max compression)
        float Iabc = 1.0f - compression * 0.99f;
        ota_.setIabc(Iabc);

        // Track gain reduction for diagnostics
        currentGainReduction_ = ota_.getGain();

        // Apply VCA then makeup gain
        float out = ota_.process(input);
        return out * makeup;
    }

    void reset() noexcept {
        ota_.reset();
        detector_.reset();
        sensitivitySmoother_.snapTo(0.5f);
        outputSmoother_.snapTo(0.5f);
        currentGainReduction_ = 0.0f;
    }

    /// Gain reduction in dB (negative value means attenuation).
    [[nodiscard]] float getGainReductionDb() const noexcept {
        float gain = std::max(currentGainReduction_, 1e-6f);
        return 20.0f * log10f(gain);
    }

private:
    WdfOta ota_;
    EnvelopeFollower detector_;
    ParameterSmoother sensitivitySmoother_;
    ParameterSmoother outputSmoother_;
    float sampleRate_           = 48000.0f;
    float currentGainReduction_ = 1.0f;
};
