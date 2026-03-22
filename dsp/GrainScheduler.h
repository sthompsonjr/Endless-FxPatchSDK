#pragma once

#include "GrainEnvelope.h"
#include "Interpolation.h"
#include <array>
#include <cmath>
#include <cstdint>

template<int MaxGrains>
class GrainScheduler {
public:
    struct GrainParams {
        float position;       // read position in source buffer (0.0–1.0 normalized)
        float pitch;          // playback ratio (1.0 = normal, 2.0 = octave up)
        float durationMs;     // grain length in milliseconds
        float pan;            // -1.0 (left) to 1.0 (right)
        float amplitude;      // 0.0–1.0
        grain::EnvelopeShape shape;
    };

    void init(float sampleRate) {
        sampleRate_ = sampleRate;
        reset();
    }

    void trigger(const GrainParams& params, const float* sourceBuffer, int sourceLength) {
        int slot = findFreeSlot();
        Grain& g = grains_[slot];
        g.active = true;
        g.phase = 0.0f;
        g.phaseInc = 1.0f / (params.durationMs * 0.001f * sampleRate_);
        g.readPos = params.position * static_cast<float>(sourceLength);
        g.readInc = params.pitch;
        g.sourceBuffer = sourceBuffer;
        g.sourceLength = sourceLength;
        g.amplitude = params.amplitude;
        g.shape = params.shape;
        // Constant-power pan
        float panAngle = params.pan * 0.785398163f + 0.785398163f; // pan * pi/4 + pi/4
        g.leftGain = cosf(panAngle);
        g.rightGain = sinf(panAngle);
    }

    void process(float& outLeft, float& outRight) {
        for (int i = 0; i < MaxGrains; ++i) {
            Grain& g = grains_[i];
            if (!g.active) continue;

            // Read source with Hermite interpolation
            float sample = readSourceHermite(g);

            // Apply envelope
            float env = 0.0f;
            switch (g.shape) {
                case grain::EnvelopeShape::Hann:
                    env = grain::hann(g.phase);
                    break;
                case grain::EnvelopeShape::Tukey:
                    env = grain::tukey(g.phase, 0.5f);
                    break;
                case grain::EnvelopeShape::Trapezoid:
                    env = grain::trapezoid(g.phase, 0.25f);
                    break;
            }

            sample *= env * g.amplitude;

            outLeft += sample * g.leftGain;
            outRight += sample * g.rightGain;

            // Advance grain
            g.phase += g.phaseInc;
            g.readPos += g.readInc;

            if (g.phase >= 1.0f) {
                g.active = false;
            }
        }
    }

    [[nodiscard]] int activeGrainCount() const {
        int count = 0;
        for (int i = 0; i < MaxGrains; ++i) {
            if (grains_[i].active) ++count;
        }
        return count;
    }

    void reset() {
        for (int i = 0; i < MaxGrains; ++i) {
            grains_[i].active = false;
        }
    }

private:
    struct Grain {
        bool active = false;
        float phase = 0.0f;
        float phaseInc = 0.0f;
        float readPos = 0.0f;
        float readInc = 0.0f;
        const float* sourceBuffer = nullptr;
        int sourceLength = 0;
        float amplitude = 0.0f;
        float leftGain = 0.0f;
        float rightGain = 0.0f;
        grain::EnvelopeShape shape = grain::EnvelopeShape::Hann;
    };

    int findFreeSlot() {
        // Find inactive grain
        for (int i = 0; i < MaxGrains; ++i) {
            if (!grains_[i].active) return i;
        }
        // All active — steal oldest (highest phase progress)
        int oldest = 0;
        float maxPhase = 0.0f;
        for (int i = 0; i < MaxGrains; ++i) {
            if (grains_[i].phase > maxPhase) {
                maxPhase = grains_[i].phase;
                oldest = i;
            }
        }
        return oldest;
    }

    float readSourceHermite(const Grain& g) const {
        int idx = static_cast<int>(g.readPos);
        float frac = g.readPos - static_cast<float>(idx);
        int len = g.sourceLength;

        auto safeRead = [&](int i) -> float {
            if (i < 0) i = 0;
            if (i >= len) i = len - 1;
            return g.sourceBuffer[i];
        };

        return interp::hermite(
            safeRead(idx - 1),
            safeRead(idx),
            safeRead(idx + 1),
            safeRead(idx + 2),
            frac
        );
    }

    std::array<Grain, MaxGrains> grains_{};
    float sampleRate_ = 48000.0f;
};
