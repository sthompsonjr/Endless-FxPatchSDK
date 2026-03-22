#pragma once

#include <cmath>

namespace interp {

[[nodiscard]] inline constexpr float linear(float a, float b, float t) {
    return a + t * (b - a);
}

[[nodiscard]] inline constexpr float hermite(float xm1, float x0, float x1, float x2, float t) {
    float c0 = x0;
    float c1 = 0.5f * (x1 - xm1);
    float c2 = xm1 - 2.5f * x0 + 2.0f * x1 - 0.5f * x2;
    float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
    return ((c3 * t + c2) * t + c1) * t + c0;
}

[[nodiscard]] inline float cosine(float a, float b, float t) {
    float ct = (1.0f - cosf(t * 3.14159265f)) * 0.5f;
    return a + ct * (b - a);
}

} // namespace interp
