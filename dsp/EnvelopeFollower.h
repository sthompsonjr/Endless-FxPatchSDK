#pragma once

#include <cmath>

class EnvelopeFollower {
public:
    enum class Mode { Peak, RMS };

    void init(float sampleRate) {
        sampleRate_ = sampleRate;
        setAttackMs(10.0f);
        setReleaseMs(100.0f);
        mode_ = Mode::Peak;
        envelope_ = 0.0f;
    }

    void setAttackMs(float ms) {
        attackCoeff_ = expf(-1.0f / (ms * 0.001f * sampleRate_));
    }

    void setReleaseMs(float ms) {
        releaseCoeff_ = expf(-1.0f / (ms * 0.001f * sampleRate_));
    }

    void setMode(Mode mode) {
        mode_ = mode;
    }

    [[nodiscard]] float process(float input) {
        float detect;
        if (mode_ == Mode::Peak) {
            detect = fabsf(input);
        } else {
            detect = input * input;
        }

        if (detect > envelope_) {
            envelope_ = attackCoeff_ * envelope_ + (1.0f - attackCoeff_) * detect;
        } else {
            envelope_ = releaseCoeff_ * envelope_ + (1.0f - releaseCoeff_) * detect;
        }

        if (mode_ == Mode::RMS) {
            return sqrtf(envelope_);
        }
        return envelope_;
    }

    [[nodiscard]] float getLevel() const {
        if (mode_ == Mode::RMS) {
            return sqrtf(envelope_);
        }
        return envelope_;
    }

    void reset() {
        envelope_ = 0.0f;
    }

private:
    float sampleRate_ = 48000.0f;
    float attackCoeff_ = 0.0f;
    float releaseCoeff_ = 0.0f;
    float envelope_ = 0.0f;
    Mode mode_ = Mode::Peak;
};
