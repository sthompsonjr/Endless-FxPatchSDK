#pragma once

#include "CircularBuffer.h"

template<size_t Size>
class AllpassDelay {
public:
    void init(int delaySamples, float coefficient) {
        delaySamples_ = delaySamples;
        coefficient_ = coefficient;
        buffer_.reset();
    }

    [[nodiscard]] float process(float input) {
        float delayed = buffer_.read(delaySamples_);
        float w = input + coefficient_ * delayed;
        float output = -coefficient_ * w + delayed;
        buffer_.write(w);
        return output;
    }

    void reset() {
        buffer_.reset();
    }

private:
    CircularBuffer<float, Size> buffer_;
    int delaySamples_ = 0;
    float coefficient_ = 0.0f;
};
