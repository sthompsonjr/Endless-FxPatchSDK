#pragma once

#include <cmath>

/// Lambert W function approximation for WDF diode modeling.
///
/// The Lambert W function solves w * exp(w) = x for w.
/// It has two real branches:
///   W0 (principal branch): x >= -1/e, W0 >= -1
///   W-1 (negative branch): -1/e <= x < 0, W-1 <= -1
///
/// Used to analytically solve the diode equation I = Is*(exp(V/Vt) - 1)
/// within the WDF framework, avoiding Newton-Raphson iteration for diodes.
namespace math {

/// Principal branch W0(x), valid for x >= -1/e ≈ -0.3679.
/// Uses Fritsch-Shafer-Crowley initial approximation + Halley refinement.
[[nodiscard]] inline float lambertW0(float x) noexcept {
    // Guard: below valid range
    constexpr float kInvE = 0.36787944f; // 1/e
    if (x < -kInvE) return -1.0f;
    if (x == 0.0f) return 0.0f;

    float w;

    if (x < 1.0f) {
        // Near-zero region: use Padé approximant seed
        // W(x) ≈ x * (1 - x) / (1 + x) is rough but good enough to seed Halley
        float p = sqrtf(2.0f * (2.718281828f * x + 1.0f));
        w = -1.0f + p * (1.0f + p * (-0.333333f + p * 0.152778f));
    } else if (x < 100.0f) {
        // Medium range: log-based seed
        float lx = logf(x);
        float llx = logf(lx + 1e-10f);
        w = lx - llx + llx / (lx + 1.0f);
    } else {
        // Large x: asymptotic seed
        float lx = logf(x);
        float llx = logf(lx);
        w = lx - llx + llx / lx;
    }

    // Halley's method refinement (cubic convergence)
    // w_new = w - (w*e^w - x) / (e^w*(w+1) - (w+2)*(w*e^w - x)/(2*w+2))
    for (int i = 0; i < 4; ++i) {
        float ew = expf(w);
        float wew = w * ew;
        float diff = wew - x;
        if (fabsf(diff) < 1e-6f) break;
        float wp1 = w + 1.0f;
        if (fabsf(wp1) < 1e-10f) wp1 = 1e-10f;
        float denom = ew * wp1 - (w + 2.0f) * diff / (2.0f * wp1);
        if (fabsf(denom) < 1e-12f) break;
        w -= diff / denom;
    }

    return w;
}

/// Negative branch W-1(x), valid for -1/e <= x < 0.
/// Returns values <= -1.
[[nodiscard]] inline float lambertWn1(float x) noexcept {
    constexpr float kInvE = 0.36787944f;
    if (x < -kInvE) return -1.0f;
    if (x >= 0.0f) return -1.0f; // out of domain

    // Seed for negative branch
    float lx = logf(-x);
    float llx = logf(-lx + 1e-10f);
    float w = lx - llx;

    // Halley refinement
    for (int i = 0; i < 6; ++i) {
        float ew = expf(w);
        float wew = w * ew;
        float diff = wew - x;
        if (fabsf(diff) < 1e-6f) break;
        float wp1 = w + 1.0f;
        if (fabsf(wp1) < 1e-10f) wp1 = -1e-10f;
        float denom = ew * wp1 - (w + 2.0f) * diff / (2.0f * wp1);
        if (fabsf(denom) < 1e-12f) break;
        w -= diff / denom;
    }

    return w;
}

} // namespace math
