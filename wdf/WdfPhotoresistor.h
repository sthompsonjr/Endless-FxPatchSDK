#pragma once

#include "WdfPort.h"
#include "../dsp/Saturation.h"
#include <algorithm>
#include <cmath>

/// Physical constants for cadmium sulfide LDR (e.g. Silonex NSL-32SR3, Vactrol VTL5C3).
namespace ldr_params {
    static constexpr float R_dark         = 1e6f;    // 1MΩ — resistance in darkness
    static constexpr float R_bright       = 200.0f;  // 200Ω — resistance at full illumination
    static constexpr float tau_fast       = 0.010f;  // 10ms — fast attack component
    static constexpr float tau_slow       = 0.300f;  // 300ms — slow component (attack + release)
    static constexpr float tau_release    = 1.0f;    // 1000ms — release (slow component only)
    static constexpr float w_fast_attack  = 0.7f;    // during attack: 70% fast, 30% slow
} // namespace ldr_params

/// Light-dependent resistor (LDR) modeled with two-time-constant temporal response.
///
/// The WDF representation is a one-port resistor whose resistance (port.Rp)
/// changes over time according to the light level set by the LED driver.
///
/// IMPORTANT: Because port.Rp changes every sample, any adaptor tree containing
/// a WdfPhotoresistor must rebuild adaptor Rp values before the scatter step.
/// See WdfOpticalCircuits for the correct per-sample update sequence.
///
/// The LDR's defining characteristic is asymmetric attack/release:
///   Attack: resistance falls with a fast component (10ms) and slow component (300ms)
///   Release: resistance rises only via the slow component (1000ms release)
/// This models the photochemical trap-state behavior of cadmium sulfide LDRs.
class WdfPhotoresistor {
public:
    WdfPort port;   // port.Rp = current LDR resistance — updated each sample

    struct Params {
        float R_dark        = ldr_params::R_dark;
        float R_bright      = ldr_params::R_bright;
        float tau_fast      = ldr_params::tau_fast;
        float tau_slow      = ldr_params::tau_slow;
        float tau_release   = ldr_params::tau_release;
        float w_fast_attack = ldr_params::w_fast_attack;
        float sampleRate    = 48000.0f;
    };

    void init(const Params& params) noexcept {
        p = params;
        alpha_fast    = tauToAlpha(p.tau_fast,    p.sampleRate);
        alpha_slow    = tauToAlpha(p.tau_slow,    p.sampleRate);
        alpha_release = tauToAlpha(p.tau_release, p.sampleRate);
        R_fast        = p.R_dark;
        R_slow        = p.R_dark;
        currentLight  = 0.0f;
        attacking     = false;
        port.Rp       = p.R_dark;
        port.reset();
    }

    /// Set light level (0.0=dark, 1.0=full illumination) — called each sample.
    /// Internally updates R_fast, R_slow, and port.Rp via two-time-constant model.
    void setLightLevel(float lightLevel) noexcept {
        lightLevel   = std::clamp(lightLevel, 0.0f, 1.0f);
        currentLight = lightLevel;

        float R_target = lightToResistance(lightLevel);

        // Attack/release direction based on whether the target resistance is below
        // (brighter than) the slow component's current state.
        // This is stable for oscillating signals: a held sine wave is "attacking"
        // as long as the LED drives the LDR brighter than its current slow state.
        // Using momentary delta (R_target < currentLight) would flip every cycle.
        attacking = (R_target < R_slow);

        // Update fast and slow IIR states independently
        R_fast += alpha_fast * (R_target - R_fast);
        R_slow += (attacking ? alpha_slow : alpha_release) * (R_target - R_slow);

        // Blend based on direction:
        float R_ldr;
        if (attacking) {
            // Attack: fast component dominates (LDRs respond quickly to increasing light)
            R_ldr = p.w_fast_attack * R_fast + (1.0f - p.w_fast_attack) * R_slow;
        } else {
            // Release: only slow component (LDRs release slowly — trap state emptying)
            R_ldr = R_slow;
        }

        // Clamp to physical range
        port.Rp = std::clamp(R_ldr, p.R_bright, p.R_dark);
    }

    /// WDF reflect: LDR acts as resistor (b = 0), but port.Rp is dynamic.
    void reflect() noexcept {
        port.b = 0.0f;
    }

    void reset() noexcept {
        R_fast        = p.R_dark;
        R_slow        = p.R_dark;
        currentLight  = 0.0f;
        attacking     = false;
        port.Rp       = p.R_dark;
        port.reset();
    }

    [[nodiscard]] float getCurrentResistance() const noexcept { return port.Rp; }
    [[nodiscard]] float getCurrentLightLevel() const noexcept { return currentLight; }

    /// Returns true if currently in attack phase (resistance falling).
    [[nodiscard]] bool isAttacking() const noexcept { return attacking; }

    /// Program dependency diagnostic: returns "inertia" — a measure of how hard the LDR
    /// has been driven recently. Higher inertia → slower release.
    /// Computed as: inertia = (R_dark - R_slow) / (R_dark - R_bright)
    /// Range: 0.0 (fully recovered / dark) to 1.0 (fully driven / bright)
    [[nodiscard]] float getInertia() const noexcept {
        float range = p.R_dark - p.R_bright;
        if (range < 1.0f) return 0.0f;
        return std::clamp((p.R_dark - R_slow) / range, 0.0f, 1.0f);
    }

private:
    Params p;
    float R_fast        = 0.0f;   // fast-component resistance state
    float R_slow        = 0.0f;   // slow-component resistance state
    float currentLight  = 0.0f;   // current light level
    bool  attacking     = false;

    // IIR coefficients (precomputed from time constants)
    float alpha_fast    = 0.0f;
    float alpha_slow    = 0.0f;
    float alpha_release = 0.0f;

    /// Log-linear mapping from light level to resistance.
    /// lightLevel=0 → R_dark  (maximum resistance, dark)
    /// lightLevel=1 → R_bright (minimum resistance, fully lit)
    [[nodiscard]] float lightToResistance(float lightLevel) const noexcept {
        float logRatio = logf(p.R_bright / p.R_dark);
        return p.R_dark * expf(logRatio * lightLevel);
    }

    /// Precompute IIR coefficient from time constant:
    /// alpha = 1 - exp(-1 / (tau × sampleRate))
    [[nodiscard]] static float tauToAlpha(float tau, float sr) noexcept {
        return 1.0f - expf(-1.0f / (tau * sr));
    }
};
