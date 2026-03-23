#pragma once

/*
 * SoftFocusCircuit — Yamaha SPX500 "Soft Focus" shimmer/pitch reverb
 *
 * API ADAPTATIONS from prompt design (recorded per Step 2):
 *
 * 1. CircularBuffer: All template sizes must be power-of-2.
 *    AP1(347)->512, AP2(463)->512, AP3(521)->1024, AP4(613)->1024,
 *    Comb1-4(1583-1993)->2048, CompDelay(1920)->2048.
 *    Actual delay lengths passed to init()/read().
 *
 * 2. CircularBuffer methods: write() not push(), read(int) not readAt(int),
 *    readHermite(float) for fractional reads. No readAt() exists.
 *
 * 3. CombFilter has built-in damping one-pole LP on feedback path.
 *    init(delaySamples, feedback, damping). No separate OnePoleFilter per comb.
 *    Damping coeff for 4kHz @ 48kHz: exp(-2*pi*4000/48000) ~ 0.5924f.
 *
 * 4. GrainScheduler: NOT a streaming process(input) API.
 *    Requires external source buffer + manual trigger() calls.
 *    Solution: 131072-float recording buffer from working memory.
 *    Grains triggered every kGrainHopSamples with position/pitch/pan params.
 *
 * 5. AllpassDelay: init(delaySamples, coefficient). No setGain().
 *
 * 6. Lfo: init(sampleRate), setFrequency(hz), setShape(Shape::Sine).
 *    process() returns bipolar [-1,1].
 *
 * 7. fdn::hadamard4(a,b,c,d) modifies in-place by reference.
 */

#include "CircularBuffer.h"
#include "AllpassDelay.h"
#include "ReverbPrimitives.h"
#include "GrainScheduler.h"
#include "GrainEnvelope.h"
#include "Lfo.h"
#include "ParameterSmoother.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace soft_focus {

    // Grain parameters
    constexpr int   kGrainSizeSamples    = 1920;      // 40ms at 48kHz
    constexpr int   kGrainHopSamples     = 960;       // 50% overlap
    constexpr int   kMaxGrains           = 4;
    constexpr int   kDryCompDelaySamples = 1920;

    // Recording buffer for grain source
    constexpr int   kRecBufSize          = 131072;    // ~2.7s at 48kHz
    constexpr int   kRecSafetyMargin     = 8192;      // wrap margin

    // Pitch
    constexpr float kMaxDetuneCents      = 15.0f;
    constexpr float kMaxModDepthCents    = 12.0f;
    constexpr float kPitchApproxCoeff    = 5.776e-4f; // ln(2)/1200

    // LFO
    constexpr float kLfoRateMin          = 0.15f;
    constexpr float kLfoRateRange        = 1.35f;

    // Reverb: allpass delay lengths (mutually prime)
    constexpr int   kAP1 = 347;
    constexpr int   kAP2 = 463;
    constexpr int   kAP3 = 521;
    constexpr int   kAP4 = 613;
    constexpr float kApGain = 0.7f;

    // Reverb: comb delay lengths (mutually prime)
    constexpr int   kC1 = 1583;
    constexpr int   kC2 = 1687;
    constexpr int   kC3 = 1801;
    constexpr int   kC4 = 1993;

    // Reverb: decay
    constexpr float kDecayMin            = 0.3f;
    constexpr float kDecayRange          = 3.7f;
    constexpr float kCombFeedbackCeil    = 0.9999f;

    // Reverb: damping coeff for 4kHz LP @ 48kHz
    // d = exp(-2*pi*4000/48000) ~ 0.5924f
    constexpr float kCombDamping         = 0.5924f;

    // Mix
    constexpr float kMaxWetMix           = 0.75f;
    constexpr float kGrainReverbBlend    = 0.7071f;   // 1/sqrt(2)
    constexpr float kGrainMix            = 0.5f;
    constexpr float kSymphonicMix        = 0.5f;

    // Symphonic doubler delay
    constexpr float kSymphonicBaseMs     = 2.5f;
    constexpr float kSymphonicDepthMs    = 1.5f;
    constexpr int   kSymphonicBufSize    = 256;

    // Post-reverb parallel delay taps
    constexpr int   kTap1Samples         = 12000;     // 250ms
    constexpr int   kTap2Samples         = 18240;     // 380ms
    constexpr float kTapDirect           = 0.5f;
    constexpr float kTap1Mix             = 0.3f;
    constexpr float kTap2Mix             = 0.2f;
    constexpr int   kTapBufSize          = 32768;

    // Smoothing
    constexpr float kSmoothTimeMs        = 20.0f;

} // namespace soft_focus

class SoftFocusCircuit {
public:
    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;

        // LFO
        lfo_.init(sampleRate);
        lfo_.setShape(Lfo::Shape::Sine);
        lfo_.setFrequency(soft_focus::kLfoRateMin);

        // Allpass diffusers (power-of-2 template, actual delay in init)
        ap1_.init(soft_focus::kAP1, soft_focus::kApGain);
        ap2_.init(soft_focus::kAP2, soft_focus::kApGain);
        ap3_.init(soft_focus::kAP3, soft_focus::kApGain);
        ap4_.init(soft_focus::kAP4, soft_focus::kApGain);

        // Comb filters (built-in damping)
        comb1_.init(soft_focus::kC1, 0.5f, soft_focus::kCombDamping);
        comb2_.init(soft_focus::kC2, 0.5f, soft_focus::kCombDamping);
        comb3_.init(soft_focus::kC3, 0.5f, soft_focus::kCombDamping);
        comb4_.init(soft_focus::kC4, 0.5f, soft_focus::kCombDamping);

        // Grain schedulers
        grainUp_.init(sampleRate);
        grainDown_.init(sampleRate);

        // Parameter smoothers
        reverbSmoother_.init(sampleRate, soft_focus::kSmoothTimeMs);
        pitchSmoother_.init(sampleRate, soft_focus::kSmoothTimeMs);
        modSmoother_.init(sampleRate, soft_focus::kSmoothTimeMs);

        // Initial parameter targets
        targetDecayTime_ = 0.3f + 0.5f * soft_focus::kDecayRange;
        targetWetMix_    = 0.5f * soft_focus::kMaxWetMix;
        targetBaseCents_ = 0.5f * soft_focus::kMaxDetuneCents;

        reverbSmoother_.setTarget(targetWetMix_);
        pitchSmoother_.setTarget(targetBaseCents_);
        modSmoother_.setTarget(0.3f);

        recomputeCombFeedback();

        // Reset buffers
        compDelay_.reset();
        sympBufUp_.reset();
        sympBufDown_.reset();
        tapBuf_.reset();
    }

    void assignWorkingBuffer(float* buf, int size) noexcept {
        (void)size;
        recBuf_ = buf;
        recWritePos_ = 0;
        hopCounter_ = 0;
        std::memset(recBuf_, 0, soft_focus::kRecBufSize * sizeof(float));
    }

    void setReverbAmount(float knob0) noexcept {
        targetDecayTime_ = soft_focus::kDecayMin + knob0 * soft_focus::kDecayRange;
        targetWetMix_    = knob0 * soft_focus::kMaxWetMix;
        reverbSmoother_.setTarget(targetWetMix_);
        recomputeCombFeedback();
    }

    void setPitchAmount(float knob1) noexcept {
        targetBaseCents_ = knob1 * soft_focus::kMaxDetuneCents;
        pitchSmoother_.setTarget(targetBaseCents_);
    }

    void setModIntensity(float knob2) noexcept {
        float rate = soft_focus::kLfoRateMin + knob2 * soft_focus::kLfoRateRange;
        lfo_.setFrequency(rate);
        modSmoother_.setTarget(knob2);
    }

    void setDryMute(bool mute) noexcept { dryMuted_ = mute; }
    void setFreeze(bool frozen) noexcept { frozen_ = frozen; }

    /// Gate the grain schedulers entirely.
    /// When disabled, GrainScheduler::trigger() and ::process() are not called,
    /// saving ~600 cycles/sample. The recording buffer continues to fill so that
    /// grains play from recent audio immediately when voices are re-enabled.
    void setVoicesEnabled(bool enabled) noexcept { voicesEnabled_ = enabled; }

    void process(float inL, float inR,
                 float& outL, float& outR) noexcept {
        const float input = (inL + inR) * 0.5f;

        // Advance smoothers
        const float smoothedWet   = reverbSmoother_.process();
        const float smoothedCents = pitchSmoother_.process();
        const float smoothedMod   = modSmoother_.process();

        // Single LFO tick — shared by pitch and Symphonic
        const float lfoOut   = lfo_.process();
        const float modDepth = smoothedMod * soft_focus::kMaxModDepthCents;
        const float dUp      =  smoothedCents + lfoOut * modDepth;
        const float dDown    = -smoothedCents - lfoOut * modDepth;

        // Write input to recording buffer for grain source
        if (recBuf_ != nullptr) {
            recBuf_[recWritePos_] = input;
            recWritePos_++;

            // Wrap: shift recent data to start, reset grains
            if (recWritePos_ >= soft_focus::kRecBufSize - soft_focus::kRecSafetyMargin) {
                std::memmove(recBuf_,
                             recBuf_ + recWritePos_ - soft_focus::kRecSafetyMargin,
                             soft_focus::kRecSafetyMargin * sizeof(float));
                recWritePos_ = soft_focus::kRecSafetyMargin;
                grainUp_.reset();
                grainDown_.reset();
                hopCounter_ = 0;
            }

            // Trigger grains at hop intervals — gated by voicesEnabled_
            hopCounter_++;
            if (hopCounter_ >= soft_focus::kGrainHopSamples) {
                hopCounter_ = 0;
                if (voicesEnabled_) {
                    triggerGrains(dUp, dDown);
                }
            }
        }

        // Grain pitch voices — gated by voicesEnabled_
        float grainL = 0.0f, grainR = 0.0f;
        if (voicesEnabled_) {
            grainUp_.process(grainL, grainR);
            float grainDownL = 0.0f, grainDownR = 0.0f;
            grainDown_.process(grainDownL, grainDownR);
            grainL += grainDownL;
            grainR += grainDownR;
        }

        // Symphonic doubler layer
        sympBufUp_.write(input);
        sympBufDown_.write(input);
        const float lfoModMs      = lfoOut * soft_focus::kSymphonicDepthMs;
        const float delayUpSamp   = (soft_focus::kSymphonicBaseMs + lfoModMs)
                                    * sampleRate_ * 0.001f;
        const float delayDownSamp = (soft_focus::kSymphonicBaseMs - lfoModMs)
                                    * sampleRate_ * 0.001f;
        // Clamp to valid range to avoid reading before buffer start
        const float sympDelayUpClamped   = std::max(1.0f, delayUpSamp);
        const float sympDelayDownClamped = std::max(1.0f, delayDownSamp);
        const float sympL = sympBufUp_.readHermite(sympDelayUpClamped);
        const float sympR = sympBufDown_.readHermite(sympDelayDownClamped);

        // Blend grain and Symphonic (unity-sum split)
        const float wetL = soft_focus::kGrainMix * grainL
                         + soft_focus::kSymphonicMix * sympL;
        const float wetR = soft_focus::kGrainMix * grainR
                         + soft_focus::kSymphonicMix * sympR;

        // Dry compensation delay
        compDelay_.write(input);
        const float dry = dryMuted_ ? 0.0f
                                    : compDelay_.read(soft_focus::kDryCompDelaySamples);

        // FDN reverb + post-reverb taps
        const float fdnIn    = frozen_ ? 0.0f : input;
        const float reverbOut = processFDN(fdnIn);

        // Output mix
        const float dryMix   = 1.0f - smoothedWet * 0.5f;
        const float wetBlend = smoothedWet * soft_focus::kGrainReverbBlend;
        outL = dryMix * dry + wetBlend * wetL + wetBlend * reverbOut;
        outR = dryMix * dry + wetBlend * wetR + wetBlend * reverbOut;
    }

    void reset() noexcept {
        ap1_.reset(); ap2_.reset(); ap3_.reset(); ap4_.reset();
        comb1_.reset(); comb2_.reset(); comb3_.reset(); comb4_.reset();
        compDelay_.reset();
        sympBufUp_.reset(); sympBufDown_.reset();
        tapBuf_.reset();
        grainUp_.reset(); grainDown_.reset();
        lfo_.reset();
        recWritePos_ = 0;
        hopCounter_ = 0;
        if (recBuf_ != nullptr) {
            std::memset(recBuf_, 0, soft_focus::kRecBufSize * sizeof(float));
        }
    }

    [[nodiscard]] float getCombFeedback(int idx) const noexcept {
        if (idx < 0 || idx > 3) return 0.0f;
        return combFb_[idx];
    }

    [[nodiscard]] static float centsToRatio(float cents) noexcept {
        // Approximation: r ~= 1.0f + cents * ln(2)/1200
        // Valid for |cents| <= 27. Error at 27 cents: 0.009%.
        return 1.0f + cents * soft_focus::kPitchApproxCoeff;
    }

private:
    float sampleRate_   = 48000.0f;
    bool  dryMuted_     = false;
    bool  frozen_       = false;
    bool  voicesEnabled_ = true;  // grain gate — false skips trigger() + process()

    // Recording buffer (from working memory)
    float* recBuf_    = nullptr;
    int recWritePos_  = 0;
    int hopCounter_   = 0;

    // Dry compensation delay
    CircularBuffer<float, 2048> compDelay_;

    // Grain schedulers
    GrainScheduler<soft_focus::kMaxGrains> grainUp_;
    GrainScheduler<soft_focus::kMaxGrains> grainDown_;

    // Shared LFO
    Lfo lfo_;

    // Symphonic doubler delay buffers
    CircularBuffer<float, soft_focus::kSymphonicBufSize> sympBufUp_;
    CircularBuffer<float, soft_focus::kSymphonicBufSize> sympBufDown_;

    // FDN allpass diffusers (power-of-2 template sizes)
    AllpassDelay<512> ap1_;
    AllpassDelay<512> ap2_;
    AllpassDelay<1024> ap3_;
    AllpassDelay<1024> ap4_;

    // FDN comb filters (power-of-2 template, built-in damping)
    CombFilter<2048> comb1_;
    CombFilter<2048> comb2_;
    CombFilter<2048> comb3_;
    CombFilter<2048> comb4_;

    // Comb feedback coefficients
    float combFb_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // Post-reverb tap buffer
    CircularBuffer<float, soft_focus::kTapBufSize> tapBuf_;

    // Parameter smoothers
    ParameterSmoother reverbSmoother_;
    ParameterSmoother pitchSmoother_;
    ParameterSmoother modSmoother_;

    // Cached targets
    float targetDecayTime_ = 1.0f;
    float targetWetMix_    = 0.5f;
    float targetBaseCents_ = 0.0f;

    void recomputeCombFeedback() noexcept {
        constexpr int delays[4] = {
            soft_focus::kC1, soft_focus::kC2,
            soft_focus::kC3, soft_focus::kC4
        };
        for (int i = 0; i < 4; ++i) {
            float g = powf(10.0f, -3.0f * static_cast<float>(delays[i])
                                  / (targetDecayTime_ * sampleRate_));
            combFb_[i] = std::min(g, soft_focus::kCombFeedbackCeil);
        }
        comb1_.setFeedback(combFb_[0]);
        comb2_.setFeedback(combFb_[1]);
        comb3_.setFeedback(combFb_[2]);
        comb4_.setFeedback(combFb_[3]);
    }

    void triggerGrains(float dUp, float dDown) noexcept {
        if (recBuf_ == nullptr || recWritePos_ < soft_focus::kGrainSizeSamples) return;

        float pos = static_cast<float>(recWritePos_ - soft_focus::kGrainSizeSamples)
                  / static_cast<float>(recWritePos_);
        if (pos < 0.0f) pos = 0.0f;

        typename GrainScheduler<soft_focus::kMaxGrains>::GrainParams upParams;
        upParams.position   = pos;
        upParams.pitch      = centsToRatio(dUp);
        upParams.durationMs = 40.0f;
        upParams.pan        = -1.0f; // left
        upParams.amplitude  = 1.0f;
        upParams.shape      = grain::EnvelopeShape::Hann;
        grainUp_.trigger(upParams, recBuf_, recWritePos_);

        typename GrainScheduler<soft_focus::kMaxGrains>::GrainParams downParams;
        downParams.position   = pos;
        downParams.pitch      = centsToRatio(dDown);
        downParams.durationMs = 40.0f;
        downParams.pan        = 1.0f; // right
        downParams.amplitude  = 1.0f;
        downParams.shape      = grain::EnvelopeShape::Hann;
        grainDown_.trigger(downParams, recBuf_, recWritePos_);
    }

    [[nodiscard]] float processFDN(float x) noexcept {
        // Stage 1: input diffusion (4 allpass in series)
        float d = ap1_.process(x);
        d = ap2_.process(d);
        d = ap3_.process(d);
        d = ap4_.process(d);

        // Stage 2: parallel comb bank (damping is internal to CombFilter)
        float c1 = comb1_.process(d);
        float c2 = comb2_.process(d);
        float c3 = comb3_.process(d);
        float c4 = comb4_.process(d);

        // Stage 3: Hadamard 4x4 mixing (in-place)
        fdn::hadamard4(c1, c2, c3, c4);

        // Stage 4: FDN mono sum
        const float fdnOut = (c1 + c2 + c3 + c4) * 0.25f;

        // Stage 5: Post-reverb parallel delay taps
        tapBuf_.write(fdnOut);
        const float tap1 = tapBuf_.read(soft_focus::kTap1Samples);
        const float tap2 = tapBuf_.read(soft_focus::kTap2Samples);

        return soft_focus::kTapDirect * fdnOut
             + soft_focus::kTap1Mix  * tap1
             + soft_focus::kTap2Mix  * tap2;
    }
};
