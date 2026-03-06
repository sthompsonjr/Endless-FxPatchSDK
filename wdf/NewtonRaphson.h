#pragma once

#include <algorithm>
#include <cmath>

namespace math {

struct NRResult {
    float value;
    bool converged;
    int iterations;
};

/// Newton-Raphson solver: find x such that func(x) = 0.
///
/// @param x0       Initial guess (warm-start with previous sample's solution)
/// @param func     Callable returning f(x)
/// @param dfunc    Callable returning f'(x)
/// @param maxIter  Maximum iterations (default 8, typical convergence 3-5)
/// @param tol      Convergence tolerance (default 1e-5f)
///
/// Guarantees: never returns NaN or Inf. If derivative is near-zero,
/// returns current best estimate with converged=false.
template<typename F, typename dF>
[[nodiscard]] NRResult newtonRaphson(
    float x0, F func, dF dfunc,
    int maxIter = 8, float tol = 1e-5f) noexcept
{
    float x = x0;
    bool conv = false;
    int iter = 0;

    for (; iter < maxIter; ++iter) {
        float fx = func(x);
        if (fabsf(fx) < tol) {
            conv = true;
            break;
        }
        float dfx = dfunc(x);
        if (fabsf(dfx) < 1e-12f) {
            break; // near-zero derivative, bail with current x
        }
        float delta = fx / dfx;
        delta = std::clamp(delta, -10.0f, 10.0f);
        x -= delta;
    }

    return { x, conv, iter };
}

/// Halley's method: cubic convergence variant of Newton-Raphson.
/// Particularly good for Lambert W refinement.
///
/// @param x0       Initial guess
/// @param func     Callable returning f(x)
/// @param dfunc    Callable returning f'(x)
/// @param d2func   Callable returning f''(x)
/// @param maxIter  Maximum iterations (default 6)
/// @param tol      Convergence tolerance
template<typename F, typename dF, typename d2F>
[[nodiscard]] NRResult halley(
    float x0, F func, dF dfunc, d2F d2func,
    int maxIter = 6, float tol = 1e-5f) noexcept
{
    float x = x0;
    bool conv = false;
    int iter = 0;

    for (; iter < maxIter; ++iter) {
        float fx = func(x);
        if (fabsf(fx) < tol) {
            conv = true;
            break;
        }
        float dfx = dfunc(x);
        if (fabsf(dfx) < 1e-12f) break;

        float d2fx = d2func(x);
        float denom = 2.0f * dfx * dfx - fx * d2fx;
        if (fabsf(denom) < 1e-12f) {
            // Fall back to plain Newton step
            float delta = fx / dfx;
            delta = std::clamp(delta, -10.0f, 10.0f);
            x -= delta;
        } else {
            float delta = 2.0f * fx * dfx / denom;
            delta = std::clamp(delta, -10.0f, 10.0f);
            x -= delta;
        }
    }

    return { x, conv, iter };
}

} // namespace math
