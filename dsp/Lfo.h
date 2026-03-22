#pragma once

#include <cmath>
#include <cstdint>

class Lfo {
public:
    enum class Shape { Sine, Triangle, Saw, ReverseSaw, Square, SampleHold };

    void init(float sampleRate) {
        sampleRate_ = sampleRate;
        phase_ = 0.0f;
        phaseInc_ = 0.0f;
        shape_ = Shape::Sine;
        phaseOffset_ = 0.0f;
        shValue_ = 0.0f;
        lcgState_ = 123456789u;
        prevPhase_ = 0.0f;
    }

    void setFrequency(float hz) {
        phaseInc_ = hz / sampleRate_;
    }

    void setShape(Shape shape) {
        shape_ = shape;
    }

    void setPhaseOffset(float phase) {
        phaseOffset_ = phase;
    }

    void reset() {
        phase_ = 0.0f;
        prevPhase_ = 0.0f;
    }

    [[nodiscard]] float process() {
        prevPhase_ = phase_;
        phase_ += phaseInc_;
        if (phase_ >= 1.0f) {
            phase_ -= 1.0f;
            // Generate new S&H value on phase wrap
            lcgState_ = lcgState_ * 1664525u + 1013904223u;
            shValue_ = static_cast<float>(lcgState_) / 4294967296.0f * 2.0f - 1.0f;
        }

        float p = phase_ + phaseOffset_;
        if (p >= 1.0f) p -= 1.0f;
        if (p < 0.0f) p += 1.0f;

        return computeShape(p);
    }

    [[nodiscard]] float processUnipolar() {
        return process() * 0.5f + 0.5f;
    }

private:
    [[nodiscard]] float computeShape(float p) const {
        switch (shape_) {
            case Shape::Sine:
                return sinf(p * 6.283185307f);
            case Shape::Triangle:
                return 1.0f - fabsf(p * 4.0f - 2.0f);
            case Shape::Saw:
                return p * 2.0f - 1.0f;
            case Shape::ReverseSaw:
                return 1.0f - p * 2.0f;
            case Shape::Square:
                return p < 0.5f ? 1.0f : -1.0f;
            case Shape::SampleHold:
                return shValue_;
        }
        return 0.0f;
    }

    float sampleRate_ = 48000.0f;
    float phase_ = 0.0f;
    float prevPhase_ = 0.0f;
    float phaseInc_ = 0.0f;
    float phaseOffset_ = 0.0f;
    float shValue_ = 0.0f;
    uint32_t lcgState_ = 123456789u;
    Shape shape_ = Shape::Sine;
};
