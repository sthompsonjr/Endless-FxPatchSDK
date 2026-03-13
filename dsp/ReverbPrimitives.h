#pragma once

#include "CircularBuffer.h"
#include <cmath>

template<size_t Size>
class CombFilter {
public:
    void init(int delaySamples, float feedback, float damping) {
        delaySamples_ = delaySamples;
        feedback_ = feedback;
        damping_ = damping;
        filterState_ = 0.0f;
        buffer_.reset();
    }

    [[nodiscard]] float process(float input) {
        float delayed = buffer_.read(delaySamples_);
        // One-pole lowpass on feedback path for damping
        filterState_ = delayed * (1.0f - damping_) + filterState_ * damping_;
        buffer_.write(input + filterState_ * feedback_);
        return delayed;
    }

    void setFeedback(float feedback) {
        feedback_ = feedback;
    }

    void setDamping(float damping) {
        damping_ = damping;
    }

    void reset() {
        buffer_.reset();
        filterState_ = 0.0f;
    }

private:
    CircularBuffer<float, Size> buffer_;
    int delaySamples_ = 0;
    float feedback_ = 0.0f;
    float damping_ = 0.0f;
    float filterState_ = 0.0f;
};

namespace fdn {

inline void hadamard4(float& a, float& b, float& c, float& d) {
    float w = a + b;
    float x = a - b;
    float y = c + d;
    float z = c - d;
    a = (w + y) * 0.5f;
    b = (x + z) * 0.5f;
    c = (w - y) * 0.5f;
    d = (x - z) * 0.5f;
}

} // namespace fdn
