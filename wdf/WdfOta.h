#pragma once

#include "dsp/Saturation.h"
#include <algorithm>
#include <cmath>

/// OTA (Operational Transconductance Amplifier) VCA model.
///
/// The OTA is a voltage-controlled current source whose transconductance gm
/// is proportional to the bias current Iabc:
///   Iout = gm * Vdiff,  gm = Iabc / (2 * Vt)   (Vt = 26mV thermal voltage)
///
/// For an audio VCA: Vout = Rin * Iout = Rin * gm * Vin
/// Normalized gain = Iabc / Iabc_max (linear mapping to 0..1 gain range).
///
/// Used in:
///   - DynacompCircuit (MXR Dynacomp style) — Iabc driven by envelope detector
///   - HybridOptOtaCircuit — Iabc driven by LDR resistance mapping
///
/// Soft-clipping the output prevents instability at extreme Iabc values.
class WdfOta {
public:
    /// Iabc_max: bias current at full gain (no compression), e.g. 1.0f (normalized)
    /// Iabc_min: bias current at maximum compression, e.g. 0.01f
    void init(float Iabc_max, float Iabc_min, float /*sampleRate*/) noexcept {
        Iabc_max_ = std::max(Iabc_max, 1e-6f);
        Iabc_min_ = std::clamp(Iabc_min, 1e-6f, Iabc_max_);
        currentIabc_ = Iabc_max_;
        currentGain_ = 1.0f;
    }

    /// Set OTA bias current — determines VCA gain.
    /// Iabc range: [Iabc_min, Iabc_max] → gain range: [Iabc_min/Iabc_max, 1.0]
    void setIabc(float Iabc) noexcept {
        currentIabc_ = std::clamp(Iabc, Iabc_min_, Iabc_max_);
        currentGain_ = currentIabc_ / Iabc_max_;
    }

    /// Apply VCA gain to input sample.
    [[nodiscard]] float process(float input) noexcept {
        return sat::softClip(input * currentGain_);
    }

    /// Current gain factor (0..1).
    [[nodiscard]] float getGain() const noexcept { return currentGain_; }

    /// Current Iabc value.
    [[nodiscard]] float getIabc() const noexcept { return currentIabc_; }

    void reset() noexcept {
        currentIabc_ = Iabc_max_;
        currentGain_ = 1.0f;
    }

private:
    float Iabc_max_    = 1.0f;
    float Iabc_min_    = 0.01f;
    float currentIabc_ = 1.0f;
    float currentGain_ = 1.0f;
};
