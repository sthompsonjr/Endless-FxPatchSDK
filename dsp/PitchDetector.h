#pragma once

#include "CircularBuffer.h"
#include "OnePoleFilter.h"
#include <cmath>

namespace pitch_constants {
    // 1024-sample window at 48 kHz = ~21.3 ms — covers E2 (82 Hz, period ~585 samples)
    constexpr int window_size = 1024;
    constexpr int hop_size = 128;  // run detection every 128 samples

    // Guitar pitch range: E2 (~82 Hz) to above E6 (~1318 Hz)
    constexpr float min_frequency_hz = 80.0f;
    constexpr float max_frequency_hz = 1400.0f;

    // Confidence threshold (0.0–1.0)
    constexpr float min_confidence = 0.5f;

    // Coarse-to-fine search parameters
    constexpr int coarse_step = 4;
    constexpr int fine_range = 4;
}

class PitchDetector {
public:
    PitchDetector() noexcept = default;

    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;
        minLag_ = static_cast<int>(sampleRate_ / pitch_constants::max_frequency_hz);
        maxLag_ = static_cast<int>(sampleRate_ / pitch_constants::min_frequency_hz);
        if (maxLag_ >= pitch_constants::window_size) {
            maxLag_ = pitch_constants::window_size - 1;
        }
        if (minLag_ < 1) minLag_ = 1;

        m_buffer.reset();
        m_inputFilter.init(sampleRate_);
        m_inputFilter.setType(OnePoleFilter::Type::DCBlock);

        samplesSinceLastDetection_ = 0;
        lastPitch_ = 0.0f;
        confidence_ = 0.0f;
    }

    // Feed one sample; returns true when a new pitch estimate has been computed
    bool process(float input) noexcept {
        float filtered = m_inputFilter.process(input);
        m_buffer.write(filtered);
        ++samplesSinceLastDetection_;

        if (samplesSinceLastDetection_ >= pitch_constants::hop_size) {
            samplesSinceLastDetection_ = 0;
            int bestLag = findPitchPeriod();
            if (bestLag > 0) {
                lastPitch_ = sampleRate_ / static_cast<float>(bestLag);
                float bestAmdf = computeAMDF(bestLag);
                confidence_ = estimateConfidence(bestAmdf);
            }
            return true;
        }
        return false;
    }

    float getPitch()      const noexcept { return lastPitch_; }
    float getConfidence() const noexcept { return confidence_; }

    bool isPitchValid() const noexcept {
        return confidence_ > pitch_constants::min_confidence;
    }

    void reset() noexcept {
        m_buffer.reset();
        m_inputFilter.reset();
        samplesSinceLastDetection_ = 0;
        lastPitch_ = 0.0f;
        confidence_ = 0.0f;
    }

private:
    float sampleRate_ = 48000.0f;
    int minLag_ = 0;
    int maxLag_ = 0;

    CircularBuffer<float, pitch_constants::window_size> m_buffer;
    OnePoleFilter m_inputFilter;

    int samplesSinceLastDetection_ = 0;
    float lastPitch_ = 0.0f;
    float confidence_ = 0.0f;
    float meanAmdf_ = 1.0f;  // updated each detection cycle for confidence

    // Maximum coarse-pass entries: ceil((maxLag-minLag)/coarse_step) + 1
    // At 48 kHz: (600-34)/4 + 1 = 142; keep a safe compile-time cap.
    static constexpr int kMaxCoarseEntries = 192;

    // Find the best pitch lag using coarse-to-fine AMDF search.
    //
    // Strategy: YIN-inspired — find the FIRST local minimum in normalized
    // AMDF below a threshold.  This naturally prefers shorter periods (the
    // fundamental) over harmonic multiples of the period.
    int findPitchPeriod() noexcept {
        // ── Pass 1: compute all coarse AMDF values ───────────────────────────
        float coarseD[kMaxCoarseEntries];
        int nCoarse = 0;

        for (int lag = minLag_; lag <= maxLag_ && nCoarse < kMaxCoarseEntries;
             lag += pitch_constants::coarse_step) {
            coarseD[nCoarse++] = computeAMDF(lag);
        }
        if (nCoarse == 0) return minLag_;

        // ── Compute global mean (used for normalization) ─────────────────────
        float sum = 0.0f;
        int globalMinIdx = 0;
        for (int i = 0; i < nCoarse; ++i) {
            sum += coarseD[i];
            if (coarseD[i] < coarseD[globalMinIdx]) globalMinIdx = i;
        }
        float mean = sum / static_cast<float>(nCoarse);
        if (mean < 1e-9f) mean = 1.0f;
        meanAmdf_ = mean;

        // ── Pass 2: first local minimum with normalised AMDF < threshold ─────
        // A "local minimum" at index i means coarseD[i-1] > coarseD[i] < coarseD[i+1].
        // Comparing d/mean to the threshold avoids dependence on signal amplitude.
        constexpr float kThreshold = 0.25f;
        int bestIdx = -1;

        for (int i = 1; i + 1 < nCoarse; ++i) {
            if (coarseD[i] < coarseD[i - 1] && coarseD[i] < coarseD[i + 1]) {
                if (coarseD[i] / mean < kThreshold) {
                    bestIdx = i;
                    break;
                }
            }
        }

        // Fallback: use global minimum if no below-threshold local min found
        if (bestIdx < 0) bestIdx = globalMinIdx;

        int coarseLag = minLag_ + bestIdx * pitch_constants::coarse_step;

        // ── Fine pass: search ±fine_range around the chosen coarse lag ────────
        float bestFineAmdf = coarseD[bestIdx];
        int   bestFineLag  = coarseLag;

        int fStart = coarseLag - pitch_constants::fine_range;
        int fEnd   = coarseLag + pitch_constants::fine_range;
        if (fStart < minLag_) fStart = minLag_;
        if (fEnd   > maxLag_) fEnd   = maxLag_;

        for (int lag = fStart; lag <= fEnd; ++lag) {
            float d = computeAMDF(lag);
            if (d < bestFineAmdf) {
                bestFineAmdf = d;
                bestFineLag  = lag;
            }
        }

        return bestFineLag;
    }

    // Average Magnitude Difference Function for one lag value.
    // D(τ) = (1/(N-τ)) × Σ|x[n] - x[n-τ]|
    float computeAMDF(int lag) noexcept {
        const int count = pitch_constants::window_size - lag;
        if (count <= 0) return 1e30f;

        float sumAbs = 0.0f;
        for (int n = 0; n < count; ++n) {
            float diff = m_buffer.read(n) - m_buffer.read(n + lag);
            sumAbs += (diff < 0.0f) ? -diff : diff;
        }
        return sumAbs / static_cast<float>(count);
    }

    // Confidence = 1 − (bestAmdf / meanAmdf).
    // Relies on meanAmdf_ being set by the most recent findPitchPeriod() call.
    float estimateConfidence(float bestAmdf) noexcept {
        if (meanAmdf_ < 1e-9f) return 0.0f;
        float conf = 1.0f - (bestAmdf / meanAmdf_);
        if (conf < 0.0f) conf = 0.0f;
        if (conf > 1.0f) conf = 1.0f;
        return conf;
    }
};
