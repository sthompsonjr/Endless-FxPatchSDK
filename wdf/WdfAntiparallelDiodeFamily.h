#pragma once

#include "WdfPort.h"
#include <cmath>
#include <algorithm>

/// WdfAntiparallelDiodeFamily — parametric matched antiparallel diode pair, 1D NR solver.
///
/// Place at the WDF tree root. Caller sets port.Rp before calling reflect().
/// Two matched diodes in opposite polarity: net current = 2*Is*sinh(V/(n*Vt)).
///
/// Port equation (transcendental, no closed form):
///   V + 2*Rp*Is*sinh(V/(n*Vt)) = a
///
/// Solved by 1D Newton-Raphson. f'(V) >= 1 for all V (cosh >= 1), so the function
/// is strictly monotone and NR is guaranteed to converge from any starting point.
/// Warm-started from lastV to reduce iteration count to ~3 in steady-state audio.
///
/// Note: port.Rp is set by the adaptor tree, not by init().
class WdfAntiparallelDiodeFamily {
public:
    WdfPort port;   // Public: caller sets port.Rp and port.a; reflect() writes port.b.

    WdfAntiparallelDiodeFamily() = default;

    /// Initialize diode parameters. Must be called before reflect().
    /// is:      saturation current per diode (A). Matched pair assumed.
    /// ideality: ideality factor per diode. Matched pair assumed.
    /// Note: port.Rp is NOT set here. It is set by the adaptor tree after init().
    void init(float is, float ideality) noexcept;

    /// Audio-path. Reads port.a and port.Rp; writes port.b.
    /// Solves: V + 2*Rp*Is*sinh(V/(n*Vt)) = a  via 1D Newton-Raphson.
    /// MUST be noexcept. MUST NOT allocate. Uses single-precision math only.
    void reflect() noexcept;

    [[nodiscard]] float voltage() const noexcept { return lastV; }
    [[nodiscard]] float current() const noexcept { return lastI; }  // net current

    /// Returns iteration count from last reflect() call. Use in test harness.
    [[nodiscard]] int lastIterations() const noexcept { return iterCount; }

    void reset() noexcept;

private:
    float Is       = 2.52e-9f;  // saturation current per diode (A)
    float n        = 1.752f;    // ideality factor
    float lastV    = 0.0f;      // warm-start and diagnostic
    float lastI    = 0.0f;      // net current diagnostic
    int   iterCount = 0;        // NR iteration count from last reflect()
};

inline void WdfAntiparallelDiodeFamily::init(float is, float ideality) noexcept {
    Is = std::max(is, 1.0e-20f);
    n  = ideality;
    // port.Rp intentionally not set here; caller (adaptor tree) is responsible.
}

inline void WdfAntiparallelDiodeFamily::reflect() noexcept {
    // Thermal voltage at 300 K = 25.85 mV (matches diode_physics::Vt).
    constexpr float kVt = 25.85e-3f;

    const float Rp              = port.Rp;
    const float a               = port.a;
    const float nVt             = n * kVt;
    const float twoRpIs         = 2.0f * Rp * Is;
    const float twoRpIs_over_nVt = twoRpIs / nVt;

    // NR parameters
    constexpr float kMaxStep = 0.1f;   // V per iteration; limits overshoot at extreme inputs
    constexpr float kVclamp  = 0.5f;   // V; clamps sinh/cosh argument to prevent float overflow
    constexpr int   kMaxIter = 10;
    constexpr float kTol     = 1.0e-6f;

    float V = lastV;  // warm start from previous sample

    int iter = 0;
    for (; iter < kMaxIter; ++iter) {
        const float Vc  = std::clamp(V, -kVclamp, kVclamp);  // overflow guard for sinh/cosh
        const float sv  = sinhf(Vc / nVt);
        const float cv  = coshf(Vc / nVt);
        const float fv  = V + twoRpIs * sv - a;              // residual (uses unclamped V)
        const float dfv = 1.0f + twoRpIs_over_nVt * cv;      // derivative (always >= 1)
        const float step = std::clamp(fv / dfv, -kMaxStep, kMaxStep);
        V -= step;
        if (fabsf(step) < kTol) { ++iter; break; }
    }

    iterCount = iter;
    lastV     = V;
    lastI     = 2.0f * Is * sinhf(std::clamp(V, -kVclamp, kVclamp) / nVt);
    port.b    = 2.0f * V - a;
}

inline void WdfAntiparallelDiodeFamily::reset() noexcept {
    lastV     = 0.0f;
    lastI     = 0.0f;
    iterCount = 0;
    // port.Rp intentionally not reset; preserve adaptor tree assignment.
}
