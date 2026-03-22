#pragma once

#include <algorithm>
#include <cmath>

namespace sat {

[[nodiscard]] inline float softClip(float x) {
    // Padé approximant for tanh, clamped to (-1, 1) for large inputs
    float x2 = x * x;
    float result = x * (27.0f + x2) / (27.0f + 9.0f * x2);
    return std::clamp(result, -1.0f, 1.0f);
}

[[nodiscard]] inline float softClipCubic(float x) {
    x = std::clamp(x, -1.0f, 1.0f);
    return x - x * x * x / 3.0f;
}

[[nodiscard]] inline float hardClip(float x) {
    return std::clamp(x, -1.0f, 1.0f);
}

[[nodiscard]] inline float fold(float x, float thresh) {
    while (x > thresh || x < -thresh) {
        if (x > thresh) {
            x = 2.0f * thresh - x;
        }
        if (x < -thresh) {
            x = -2.0f * thresh - x;
        }
    }
    return x;
}

[[nodiscard]] inline float drive(float x, float amount) {
    return softClip(x * (1.0f + amount * 15.0f));
}

} // namespace sat
