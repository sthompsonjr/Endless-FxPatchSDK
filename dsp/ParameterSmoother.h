#pragma once

#include <cmath>

class ParameterSmoother {
public:
    void init(float sampleRate, float smoothingTimeMs) {
        coeff_ = expf(-6.283185307f / (smoothingTimeMs * 0.001f * sampleRate));
        oneMinusCoeff_ = 1.0f - coeff_;
    }

    void setTarget(float target) {
        target_ = target;
    }

    [[nodiscard]] float process() {
        current_ += oneMinusCoeff_ * (target_ - current_);
        return current_;
    }

    [[nodiscard]] float getCurrentValue() const {
        return current_;
    }

    void snapTo(float value) {
        current_ = value;
        target_ = value;
    }

private:
    float coeff_ = 0.0f;
    float oneMinusCoeff_ = 1.0f;
    float current_ = 0.0f;
    float target_ = 0.0f;
};
