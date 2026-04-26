#pragma once

// EH-7850 Feedback Loop
// Component source: EH-7850 SPICE/component reference (project upload)
//   Section 8: C14=1µF, R31=1MΩ, R36=240kΩ, C15=220nF
//   HF rolloff: ~3kHz (each repeat progressively darker)
//   Mid-band attenuation: 0.24× (−12.4 dB)
//   Loop gain at kRunawayGain=0.92: 0.92 × 0.24 = 0.221 (stable)
// Polarity inversion: wet path has odd inversions vs dry (ref Section 7)
//   Applied here so polarity logic is in one place.
//
// CYCLE BUDGET (528 MHz / 48 kHz = 11,000 cycles/sample available):
//   DmmFeedbackEq WDF:       ~150–250 cycles/sample
//   Gain scalar + softClip:  ~20–40  cycles/sample
//   ParameterSmoother:       ~15–25  cycles/sample
//   Total DmmFeedbackLoop:   ~185–315 cycles/sample (1.7–2.9% of budget)

#include "wdf/DmmCircuits.h"
#include "dsp/Saturation.h"
#include "dsp/ParameterSmoother.h"

struct DmmFeedbackLoop {
    DmmFeedbackEq     feedbackEq;
    ParameterSmoother feedbackSmooth;  // smoothMs = 20.0f

    float feedbackState   = 0.0f;
    float currentFdbkGain = 0.0f;
    bool  isRunawayMode   = false;

    static constexpr float kRunawayGain = DmmFeedbackEq::kRunawayGain; // 0.92f
    static constexpr float kMaxFeedback = 1.0f;

    void init(float sampleRate) noexcept {
        feedbackEq.init(sampleRate);
        feedbackSmooth.init(sampleRate, 20.0f);
        feedbackSmooth.snapTo(0.0f);
    }

    void setFeedbackKnob(float v) noexcept {
        // v in [0,1]; log-taper via squaring
        float target = isRunawayMode
                     ? kRunawayGain
                     : v * v * kMaxFeedback;
        feedbackSmooth.setTarget(target);
    }

    void setRunawayMode(bool active) noexcept {
        isRunawayMode = active;
        if (active)
            feedbackSmooth.setTarget(kRunawayGain);
        // On deactivate, next setFeedbackKnob() call restores pot position
    }

    // Returns the feedback contribution to be summed at chain input.
    // Call AFTER chain output is available.
    float getFeedbackContribution() const noexcept {
        return feedbackState;
    }

    // Call once per sample with the wet chain output (post-expander,
    // pre-output-buffer). Updates feedbackState for next sample.
    void update(float wetSample) noexcept {
        currentFdbkGain = feedbackSmooth.process();
        // Polarity inversion: wet has odd inversions vs dry (ref Section 7)
        float inverted  = -wetSample;
        // Scale by feedback gain
        float scaled    = inverted * currentFdbkGain;
        // Process through passive EQ (darkens with each repeat)
        float equalized = feedbackEq.process(scaled);
        // Soft-clip to prevent explosion from gain recovery stack-up
        feedbackState   = sat::softClip(equalized);
    }

    void reset() noexcept {
        feedbackEq.reset();
        feedbackState   = 0.0f;
        currentFdbkGain = 0.0f;
    }
};
