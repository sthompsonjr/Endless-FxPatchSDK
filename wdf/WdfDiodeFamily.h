#pragma once

#include "WdfPort.h"
#include "LambertW.h"
#include <cmath>
#include <algorithm>

/// WdfDiodeFamily — parametric single diode, Lambert W closed-form solver.
///
/// Place at the WDF tree root. Caller sets port.Rp before calling reflect().
/// Accepts saturation current Is and ideality factor n, covering any
/// Shockley-law diode (silicon, germanium, Schottky, LED).
///
/// Models: I(V) = Is * (exp(V/(n*Vt)) - 1)  where Vt = 25.85e-3 V
///
/// Analytical solution (Fettweis/Werner formulation with ideality factor):
///   V = n*Vt * W0( (Is*Rp)/(n*Vt) * exp((a + Is*Rp)/(n*Vt)) ) - Is*Rp
///   b = 2*V - a
///
/// Note: port.Rp is set by the adaptor tree, not by init().
/// Memoryless: lastV/lastI are diagnostic only, not state.
class WdfDiodeFamily {
public:
    WdfPort port;   // Public: caller sets port.Rp and port.a; reflect() writes port.b.

    WdfDiodeFamily() = default;

    /// Initialize diode parameters. Must be called before reflect().
    /// is:      saturation current (A). Use diode_params namespace.
    /// ideality: ideality factor (dimensionless, 1.0–2.0). Use diode_params namespace.
    /// Note: port.Rp is NOT set here. It is set by the adaptor tree after init().
    void init(float is, float ideality) noexcept;

    /// Audio-path. Reads port.a and port.Rp; writes port.b.
    /// MUST be noexcept. MUST NOT allocate. Uses single-precision math only.
    void reflect() noexcept;

    /// Returns computed node voltage from last reflect() call.
    [[nodiscard]] float voltage() const noexcept { return lastV; }
    /// Returns computed current from last reflect() call.
    [[nodiscard]] float current() const noexcept { return lastI; }

    void reset() noexcept;

private:
    float Is    = 2.52e-9f;  // saturation current (A)
    float n     = 1.752f;    // ideality factor
    float lastV = 0.0f;      // last node voltage (diagnostic)
    float lastI = 0.0f;      // last current (diagnostic)
};

inline void WdfDiodeFamily::init(float is, float ideality) noexcept {
    Is = std::max(is, 1.0e-20f);
    n  = ideality;
    // port.Rp intentionally not set here; caller (adaptor tree) is responsible.
}

inline void WdfDiodeFamily::reflect() noexcept {
    // Thermal voltage at 300 K = 25.85 mV (matches diode_physics::Vt).
    constexpr float kVt = 25.85e-3f;

    const float Rp      = port.Rp;
    const float a       = port.a;
    const float Is_safe = std::max(Is, 1.0e-20f);
    const float nVt     = n * kVt;
    const float RIs     = Is_safe * Rp;

    // Clamp exponent argument to prevent float overflow (expf overflows above ~88).
    const float arg_exp = std::clamp((a + RIs) / nVt, -80.0f, 85.0f);
    const float lw_arg  = (RIs / nVt) * expf(arg_exp);

    // Correct form (matches WdfDiodeToGround derivation):
    //   V = a + Is*Rp - n*Vt * W0( (Is*Rp)/(n*Vt) * exp((a + Is*Rp)/(n*Vt)) )
    // Note: prompt Section 5.3 has the sign inverted relative to the correct formula.
    // This form is verified by WdfDiodeToGround which uses identical algebra.
    const float lw = math::lambertW0(lw_arg);
    float V = a + RIs - nVt * lw;

    // Guard: handle non-finite results only.
    // For physical audio inputs (|a| <= ~3 V), the formula naturally produces
    // bounded V in [0, a] for a > 0. A fixed voltage clamp is omitted because
    // LEDs have Vfwd > 0.9 V with the given diode_params (e.g. RedLed ≈ 0.96 V).
    // For pathological inputs the exp argument is clamped to 85.0f above, which
    // limits the damage without introducing a spurious hard voltage ceiling.
    if (!std::isfinite(V)) {
        V = (a >= 0.0f) ? 0.0f : 0.0f;
    }

    lastI  = Is_safe * (expf(std::clamp(V / nVt, -80.0f, 80.0f)) - 1.0f);
    port.b = 2.0f * V - a;
    lastV  = V;
}

inline void WdfDiodeFamily::reset() noexcept {
    lastV = 0.0f;
    lastI = 0.0f;
    // port.Rp intentionally not reset; preserve adaptor tree assignment.
}
