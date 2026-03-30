#pragma once

#include "WdfPort.h"
#include <cmath>
#include <algorithm>

// WdfMosfetFamily — parametric N-channel enhancement MOSFET, soft-clipper model.
// Models the drain-source channel as a voltage-controlled resistor with threshold.
//
// Rds formula:
//   Vgs <= Vt:  Rds = 1e6 Ω  (device off, open circuit approximation)
//   Vgs >  Vt:  Rds = Rds_on * Vt / (Vgs - Vt)   [APPROXIMATION — see below]
//
// APPROXIMATION: MOSFET triode Rds; valid for Vgs slightly above Vt in the
// triode (linear) region. Not accurate for large-signal saturation region
// (Vds >> Vgs - Vt). Derivation: from the standard triode Id equation
//   Id = µnCox(W/L) * [(Vgs-Vt)*Vds - Vds²/2]
// in the limit Vds << Vgs-Vt, Id ≈ Vds/Rds where Rds = 1/(µnCox(W/L)*(Vgs-Vt)).
// Normalizing with Rds_on at Vgs=2*Vt gives Rds(Vgs) = Rds_on*Vt/(Vgs-Vt).
// Correct limiting behavior: Rds → ∞ at Vgs → Vt+, Rds = Rds_on at Vgs = 2*Vt.
//
// Primary use case: gate fixed at a bias voltage, drain in signal path.
// When signal voltage causes Vgs to exceed Vt, device conducts and
// soft-clips the signal. Softer knee than a diode clipper.
//
// Use: init(Vt, Rds_on), then setGateVoltage(Vgs) (once at init for static bias,
// or per-sample for dynamic use), then reflect() in the adaptor tree.
class WdfMosfetFamily {
public:
    WdfPort port;   // Drain-Source port. port.Rp = Rds(Vgs).

    WdfMosfetFamily() = default;

    // Initialize device parameters.
    // vt:      threshold voltage (V). Device off below this gate voltage.
    // rds_on:  on-resistance (Ω) at Vgs = 2*Vt. From datasheet.
    void init(float vt, float rds_on) noexcept {
        Vt     = std::max(vt,     0.1f);
        Rds_on = std::max(rds_on, 0.1f);
    }

    // Set gate-to-source voltage. Updates port.Rp = Rds(Vgs).
    // For a static soft-clipper, call once at init with the bias voltage.
    // For dynamic use, call per-sample.
    void setGateVoltage(float Vgs) noexcept {
        currentRds = computeRds(Vgs);
        port.Rp    = currentRds;
    }

    // Compute Rds for given Vgs without updating port.Rp. Used for testing.
    // APPROXIMATION: MOSFET triode Rds; valid for Vgs slightly above Vt;
    // not accurate for large-signal saturation region.
    [[nodiscard]] float computeRds(float Vgs) const noexcept {
        if (Vgs <= Vt) return kRdsOff;
        float headroom = Vgs - Vt;
        if (headroom < 1.0e-4f) return kRdsOff;
        float Rds = Rds_on * Vt / headroom;
        return std::clamp(Rds, kRdsMin, kRdsOff);
    }

    // Audio-path. Triode model: b = 0 (resistor model — all energy absorbed).
    // MUST be noexcept.
    void reflect() noexcept {
        port.b = 0.0f;
    }

    void reset() noexcept {
        setGateVoltage(0.0f);   // puts device in off state
        port.a = 0.0f;
        port.b = 0.0f;
    }

    [[nodiscard]] float getCurrentRds() const noexcept { return currentRds; }
    // Returns true (non-zero) when device is conducting (Rds < 1e5 Ω)
    [[nodiscard]] float isOn() const noexcept { return currentRds < 1.0e5f ? 1.0f : 0.0f; }

private:
    float Vt         = 1.5f;
    float Rds_on     = 50.0f;
    float currentRds = 1.0e6f;  // default: off state

    static constexpr float kRdsOff = 1.0e6f;
    static constexpr float kRdsMin = 1.0f;    // lower bound — prevents short-circuit
};

// ──────────────────────────────────────────────────────────────────────────────
// mosfet_params namespace — named device parameter structs for WdfMosfetFamily.
// ──────────────────────────────────────────────────────────────────────────────
namespace mosfet_params {

    // Source: NXP/Fairchild 2N7000 N-channel enhancement MOSFET datasheet.
    // Used in: modern pedal clipper stages, switching.
    struct N2N7000 {
        static constexpr float Vt     = 1.5f;   // threshold voltage, V
        static constexpr float Rds_on = 50.0f;  // on-resistance at Vgs=2*Vt, Ω
    };

    // Source: Infineon BS170 datasheet. Faster switching, lower Rds_on.
    struct Bs170 {
        static constexpr float Vt     = 2.0f;
        static constexpr float Rds_on = 20.0f;
    };

} // namespace mosfet_params
