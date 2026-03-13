#pragma once

#include <cmath>

class OnePoleFilter {
public:
    enum class Type { Lowpass, Highpass, DCBlock };

    void init(float sampleRate) {
        sampleRate_ = sampleRate;
        type_ = Type::Lowpass;
        setFrequency(1000.0f);
    }

    void setFrequency(float hz) {
        freq_ = hz;
        b1_ = expf(-6.283185307f * freq_ / sampleRate_);
        a0_ = 1.0f - b1_;
    }

    void setType(Type type) {
        type_ = type;
        if (type_ == Type::DCBlock) {
            b1_ = expf(-6.283185307f * 10.0f / sampleRate_);
            a0_ = 1.0f - b1_;
        }
    }

    [[nodiscard]] float process(float input) {
        float output = 0.0f;
        switch (type_) {
            case Type::Lowpass:
                y1_ = a0_ * input + b1_ * y1_;
                output = y1_;
                break;
            case Type::Highpass:
                // H(z) = (1 - z^-1) / (1 - b1*z^-1), unity gain at Nyquist
                y1_ = (input - x1_) + b1_ * y1_;
                output = y1_;
                x1_ = input;
                break;
            case Type::DCBlock:
                // H(z) = (1 - z^-1) / (1 - b1*z^-1), 10Hz cutoff
                y1_ = (input - x1_) + b1_ * y1_;
                output = y1_;
                x1_ = input;
                break;
        }
        return output;
    }

    void reset() {
        y1_ = 0.0f;
        x1_ = 0.0f;
    }

private:
    float sampleRate_ = 48000.0f;
    float freq_ = 1000.0f;
    float b1_ = 0.0f;
    float a0_ = 1.0f;
    float y1_ = 0.0f;
    float x1_ = 0.0f;
    Type type_ = Type::Lowpass;
};
