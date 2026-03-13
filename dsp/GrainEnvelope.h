#pragma once

#include <cmath>
#include <algorithm>

namespace grain {

enum class EnvelopeShape { Hann, Tukey, Trapezoid };

[[nodiscard]] inline float hann(float phase) {
    return 0.5f * (1.0f - cosf(phase * 6.283185307f));
}

[[nodiscard]] inline float tukey(float phase, float alpha) {
    if (alpha <= 0.0f) return 1.0f;
    if (alpha >= 1.0f) return hann(phase);

    float halfAlpha = alpha * 0.5f;
    if (phase < halfAlpha) {
        return 0.5f * (1.0f - cosf(phase / halfAlpha * 3.14159265f));
    } else if (phase > 1.0f - halfAlpha) {
        return 0.5f * (1.0f - cosf((1.0f - phase) / halfAlpha * 3.14159265f));
    }
    return 1.0f;
}

[[nodiscard]] inline float trapezoid(float phase, float rampFraction) {
    rampFraction = std::clamp(rampFraction, 0.0001f, 0.5f);
    if (phase < rampFraction) {
        return phase / rampFraction;
    } else if (phase > 1.0f - rampFraction) {
        return (1.0f - phase) / rampFraction;
    }
    return 1.0f;
}

[[nodiscard]] inline float gaussian(float phase, float sigma) {
    float x = (phase - 0.5f) / sigma;
    return std::clamp(expf(-0.5f * x * x), 0.0f, 1.0f);
}

} // namespace grain
