/*
 * PatchImpl_Shoegaze.cpp — Big Muff Pi (WDF) → Soft Focus (FDN shimmer reverb)
 *                           combined effect.
 *
 * Chains BigMuffCircuit into SoftFocusCircuit. A hold footswitch reverses
 * the chain order (Soft Focus → Big Muff). A tap footswitch cycles the Big
 * Muff transistor variant: Triangle → Ram's Head → NYC.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  PRE-FLIGHT API FINDINGS (Phase 0)
 * ═══════════════════════════════════════════════════════════════════════
 *
 * WdfBigMuffCircuit.h (actual API):
 *   - Variant enum values: RamsHead(0), CivilWar(1), Triangle(2)
 *     NYC did NOT exist → added in this PR: NYC(3)
 *   - Public methods: init(float sampleRate, Variant = RamsHead)
 *                     process(float) -> float   [mono in / mono out]
 *                     setSustain(float 0-1)
 *                     setTone(float 0-1)
 *                     setVolume(float 0-1)
 *                     setVariant(Variant)
 *   - applyVariant() sets curIs_/curVt_/curHFE_ then calls initBjts()
 *   - No setTransistorParams() — NYC added as a new case in applyVariant()
 *
 * SoftFocusCircuit.h (actual API — differs significantly from prompt design):
 *   - process(float inL, float inR, float& outL, float& outR)
 *     Internally: input = (inL + inR) * 0.5f → stereo output
 *   - setReverbAmount(float 0-1): decay 0.3+knob*3.7 s, wet mix 0-75%
 *     → used for Depth Zone A+B+C (no separate feedback control available)
 *   - setPitchAmount(float 0-1): detune 0-15 cents per voice
 *     → used for Modulation Zone A (detune sweep)
 *   - setModIntensity(float 0-1): LFO rate 0.15-1.5 Hz, depth 0-12 cents (coupled)
 *     → used for Modulation Zones B+C (LFO ramp and extreme)
 *   - setDryMute(bool): mutes the dry (compensation-delayed) signal
 *   - setFreeze(bool): gates reverb input
 *   - NO separate setDecay(), setVoiceMix(), setFeedback(), setLfoRate(), setLfoDepth()
 *   - setVoicesEnabled(bool): ADDED in this PR — gates GrainScheduler::trigger()
 *     and ::process() for ~600 cycles/sample saving when voices off
 *   - assignWorkingBuffer(float* buf, int size): for grain recording buffer
 *   - Working buffer required: soft_focus::kRecBufSize = 131072 floats
 *
 * ADAPTATIONS FROM PROMPT DESIGN:
 *   1. Depth Zone C (0.67-1.0): No delay feedback control exposed. This zone
 *      holds reverb at maximum. Prompt's "near-oscillation" effect is not
 *      achievable without API changes to SoftFocusCircuit. Documented limitation.
 *   2. Modulation LFO rate range: 0.15-1.5 Hz actual (prompt expected 0.3-4.0 Hz).
 *      Max "extreme" modulation at knob2=1.0 uses modIntensity=1.0 which gives
 *      LFO rate 1.5 Hz, depth 12 cents.
 *   3. Voice mix: SoftFocusCircuit has no voiceMix control. Voices are on or off.
 *      Smooth fade-in is achieved by ramping pitchAmount from 0 in Zone A.
 *   4. "100% wet" (prompt): SoftFocusCircuit mixes dry + wet internally. The dry
 *      path carries the fuzz signal (40ms compensation delay) which IS part of
 *      the shoegaze sound. setDryMute(false) is used — fuzz present at all settings.
 *   5. NYC input coupling cap (150nF): hardcoded constant in WdfBigMuffCircuit,
 *      not reconfigurable. Documented limitation.
 *   6. Extra mod depth (>1.0 via external pitch modulation): not implemented
 *      because GrainScheduler pitch is set via triggerGrains() which is private
 *      in SoftFocusCircuit, and adding per-sample pitch modulation would require
 *      further API changes. Zone C "extreme" uses setModIntensity(1.0) which
 *      reaches the circuit's natural maximum.
 *
 * ═══════════════════════════════════════════════════════════════════════
 */

#include "sdk/Patch.h"
#include "wdf/WdfBigMuffCircuit.h"
#include "dsp/SoftFocusCircuit.h"
#include "dsp/ParameterSmoother.h"
#include "dsp/OnePoleFilter.h"
#include "dsp/AllpassDelay.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// Named constants
// ─────────────────────────────────────────────────────────────────────────────
namespace shoegaze {

    // Variant cycle: Triangle → RamsHead → NYC → Triangle
    // Indices into variantTable_
    static constexpr int kNumVariants = 3;

    // ── Fuzz knob (Knob 0) mapping ──────────────────────────────────────
    constexpr float kFuzzZoneBoundary     = 0.67f;

    constexpr float kSustainMin           = 0.10f;   // barely clipping
    constexpr float kSustainMax           = 1.00f;   // full fuzz
    constexpr float kToneBright           = 0.72f;   // treble-forward overdrive
    constexpr float kToneMidScoop         = 0.45f;   // classic mid-scoop entry
    constexpr float kToneDark             = 0.18f;   // deep bass, minimal treble

    // ── Depth knob (Knob 1) mapping ─────────────────────────────────────
    constexpr float kDepthZone1           = 0.33f;   // end of reverb-only zone
    constexpr float kDepthZone2           = 0.67f;   // end of voices zone

    // Mapped to setReverbAmount(0-1): 0→decay 0.3s, 1→decay 4.0s
    constexpr float kReverbAmountMin      = 0.0f;    // shortest tail
    constexpr float kReverbAmountMid      = 0.5f;    // medium tail (~2.15s)
    constexpr float kReverbAmountMax      = 1.0f;    // near-infinite tail (~4.0s)

    // ── Modulation knob (Knob 2) mapping ────────────────────────────────
    constexpr float kModZone1             = 0.33f;   // end of detune-only zone
    constexpr float kModZone2             = 0.67f;   // end of LFO ramp zone

    // ── Chain switching ─────────────────────────────────────────────────
    // kChainFadeSamples = 20ms @ 48kHz total (10ms down, 10ms up)
    constexpr int   kChainFadeSamples     = 960;     // 20ms @ 48kHz
    constexpr int   kChainFadeHalf        = kChainFadeSamples / 2;

    // ── Variant fade ─────────────────────────────────────────────────────
    constexpr int   kVariantFadeSamples   = 480;     // 10ms @ 48kHz
    constexpr int   kVariantFadeHalf      = kVariantFadeSamples / 2;

    // ── Decorrelation allpass (reversed chain stereo recovery) ───────────
    // 1.3ms @ 48kHz = 62.4 → 62 samples, AllpassDelay<128> template
    constexpr int   kDecorrDelaySamples   = 62;
    constexpr float kDecorrCoeff          = 0.5f;

    // ── LED flash interval for reversed chain mode (samples per half-period)
    // 2 Hz flash → 0.5s period → 24000 samples half-period
    constexpr uint32_t kFlashHalfPeriod   = 24000u;

} // namespace shoegaze

// ─────────────────────────────────────────────────────────────────────────────
// PatchImpl
// ─────────────────────────────────────────────────────────────────────────────
class PatchImpl : public Patch {
public:
    void init() override {
        // Reset knob values to defaults (important for re-init on singleton instances)
        knobFuzz_  = 0.5f;
        knobDepth_ = 0.3f;
        knobMod_   = 0.0f;

        // Sub-circuits
        muff_.init(static_cast<float>(kSampleRate), bigmuff::Variant::RamsHead);
        muff_.setSustain(0.5f);
        muff_.setTone(0.5f);
        muff_.setVolume(1.0f);

        softFocus_.init(static_cast<float>(kSampleRate));
        softFocus_.setReverbAmount(0.5f);
        softFocus_.setPitchAmount(0.5f);
        softFocus_.setModIntensity(0.0f);
        // Voices start disabled (knob1 default maps to Zone A)
        softFocus_.setVoicesEnabled(false);

        // Decorrelation allpass (used in reversed chain)
        decorrAllpass_.init(shoegaze::kDecorrDelaySamples, shoegaze::kDecorrCoeff);

        // DC blocking filters (output stage)
        dcBlockL_.init(static_cast<float>(kSampleRate));
        dcBlockL_.setType(OnePoleFilter::Type::DCBlock);
        dcBlockL_.reset();   // OnePoleFilter::init() does not clear y1_/x1_
        dcBlockR_.init(static_cast<float>(kSampleRate));
        dcBlockR_.setType(OnePoleFilter::Type::DCBlock);
        dcBlockR_.reset();

        // Parameter smoothers
        // 30ms for fuzz knob (sustain, tone)
        knob0Smoother_.init(static_cast<float>(kSampleRate), 30.0f);
        knob0Smoother_.snapTo(knobFuzz_);
        // 50ms for depth knob (reverb amount — longer to prevent clicks on feedback steps)
        knob1Smoother_.init(static_cast<float>(kSampleRate), 50.0f);
        knob1Smoother_.snapTo(knobDepth_);
        // 30ms for modulation knob
        knob2Smoother_.init(static_cast<float>(kSampleRate), 30.0f);
        knob2Smoother_.snapTo(knobMod_);

        // Pre-warm BigMuffCircuit's tone-stack filters and output DC block.
        // BigMuffCircuit::warmupPortStates() pre-sets the inter-stage HP filters
        // and WDF port voltages to DC steady state, but NOT toneLp_, toneHp_, or
        // the output dcBlock_. Without warm-up those filters produce a large DC
        // transient (~0.45f) on the first samples, which would contaminate the
        // SoftFocus reverb tail. 5000 silence samples > 5× dcBlock_ time constant
        // (τ ≈ 764 samples at 10 Hz cutoff), settling output to < 0.001.
        //
        // Critically: set tone/sustain from the current knobFuzz_ value BEFORE the
        // warmup so the DC block settles to the same operating point that the first
        // control-rate update will push. If tone changes between warmup and first
        // update, the DC block sees a step change and produces a transient.
        {
            float initSustain, initTone;
            if (knobFuzz_ <= shoegaze::kFuzzZoneBoundary) {
                const float t = knobFuzz_ / shoegaze::kFuzzZoneBoundary;
                initSustain = shoegaze::kSustainMin
                            + (shoegaze::kSustainMax - shoegaze::kSustainMin) * t;
                initTone    = shoegaze::kToneBright
                            - (shoegaze::kToneBright - shoegaze::kToneMidScoop) * t;
            } else {
                const float t = (knobFuzz_ - shoegaze::kFuzzZoneBoundary)
                              / (1.0f - shoegaze::kFuzzZoneBoundary);
                initSustain = shoegaze::kSustainMax;
                initTone    = shoegaze::kToneMidScoop
                            - (shoegaze::kToneMidScoop - shoegaze::kToneDark) * t;
            }
            muff_.setSustain(initSustain);
            muff_.setTone(initTone);
            muff_.setVolume(1.0f);
            // 15000 samples = ~19.6τ at 10 Hz: residual < 3.3e-9 × initial DC
            for (int i = 0; i < 15000; ++i) {
                (void)muff_.process(0.0f);
            }
        }

        // State
        chainReversed_          = false;
        currentVariantIdx_      = 1;  // default: RamsHead
        chainFadeCounter_       = 0;
        chainFadeSwapped_       = false;
        pendingChainSwap_       = false;
        variantFadeCounter_     = 0;
        variantFadeSwapped_     = false;
        pendingVariantIdx_      = -1;
        sampleCounter_          = 0;
        voicesEnabled_          = false;
    }

    void setWorkingBuffer(std::span<float, kWorkingBufferSize> buffer) override {
        // BigMuffCircuit has no working buffer requirement (all DTCM state).
        // SoftFocusCircuit needs kRecBufSize (131072) floats for the grain
        // recording buffer. Pass the full working buffer — it has ample space.
        softFocus_.assignWorkingBuffer(buffer.data(), kWorkingBufferSize);
    }

    void processAudio(std::span<float> left, std::span<float> right) noexcept override {
        for (std::size_t n = 0; n < left.size(); ++n) {
            // 1. Mono input sum
            const float mono = (left[n] + right[n]) * 0.5f;

            // 2. Advance all smoothers (every sample)
            const float smoothKnob0 = knob0Smoother_.process();
            const float smoothKnob1 = knob1Smoother_.process();
            const float smoothKnob2 = knob2Smoother_.process();

            // 3. Control-rate parameter push (every 32 samples)
            //    Smoothers run every sample above; sub-circuit setters are
            //    decimated to 32-sample intervals (~0.67ms) to amortize the
            //    cost of WDF adaptor recalculation and filter coefficient updates.
            if ((sampleCounter_ & 0x1Fu) == 0u) {
                // ── Fuzz parameters (Knob 0 piecewise mapping) ─────────────
                float sustain, tone;
                if (smoothKnob0 <= shoegaze::kFuzzZoneBoundary) {
                    const float t = smoothKnob0 / shoegaze::kFuzzZoneBoundary;
                    sustain = shoegaze::kSustainMin + (shoegaze::kSustainMax - shoegaze::kSustainMin) * t;
                    tone    = shoegaze::kToneBright  - (shoegaze::kToneBright - shoegaze::kToneMidScoop) * t;
                } else {
                    const float t = (smoothKnob0 - shoegaze::kFuzzZoneBoundary)
                                  / (1.0f - shoegaze::kFuzzZoneBoundary);
                    sustain = shoegaze::kSustainMax;
                    tone    = shoegaze::kToneMidScoop - (shoegaze::kToneMidScoop - shoegaze::kToneDark) * t;
                }
                muff_.setSustain(sustain);
                muff_.setTone(tone);
                muff_.setVolume(1.0f);

                // ── Reverb/voices (Knob 1 three-zone mapping) ─────────────
                // Zone A (0→0.33): reverb 0→0.5, voices off
                // Zone B (0.33→0.67): reverb 0.5→1.0, voices on
                // Zone C (0.67→1.0): reverb held at 1.0, voices on
                //   Adaptation: no delay feedback control in SoftFocusCircuit.
                //   Zone C is "max reverb + voices" without additional oscillation.
                float reverbAmount;
                bool  newVoicesEnabled;
                if (smoothKnob1 <= shoegaze::kDepthZone1) {
                    const float t   = smoothKnob1 / shoegaze::kDepthZone1;
                    reverbAmount    = shoegaze::kReverbAmountMin
                                    + (shoegaze::kReverbAmountMid - shoegaze::kReverbAmountMin) * t;
                    newVoicesEnabled = false;
                } else if (smoothKnob1 <= shoegaze::kDepthZone2) {
                    const float t   = (smoothKnob1 - shoegaze::kDepthZone1)
                                    / (shoegaze::kDepthZone2 - shoegaze::kDepthZone1);
                    reverbAmount    = shoegaze::kReverbAmountMid
                                    + (shoegaze::kReverbAmountMax - shoegaze::kReverbAmountMid) * t;
                    newVoicesEnabled = true;
                } else {
                    reverbAmount    = shoegaze::kReverbAmountMax;
                    newVoicesEnabled = true;
                }
                softFocus_.setReverbAmount(reverbAmount);
                if (newVoicesEnabled != voicesEnabled_) {
                    voicesEnabled_ = newVoicesEnabled;
                    softFocus_.setVoicesEnabled(voicesEnabled_);
                }

                // ── Modulation (Knob 2 three-zone mapping) ─────────────────
                // Zone A (0→0.33): pitch detune ramps 0→max, no LFO
                // Zone B (0.33→0.67): max detune, LFO fades in (rate+depth coupled)
                // Zone C (0.67→1.0): max detune, LFO ramps to maximum intensity
                //   Adaptation: LFO rate/depth are coupled via setModIntensity.
                //   Actual range: 0.15-1.5 Hz rate, 0-12 cents depth (vs prompt's
                //   0.3-4.0 Hz and 0-50 cents). Zone C "extreme" = modIntensity 1.0.
                float pitchAmount, modIntensity;
                if (smoothKnob2 <= shoegaze::kModZone1) {
                    const float t = smoothKnob2 / shoegaze::kModZone1;
                    pitchAmount   = t;      // 0→1 maps to 0→15 cents
                    modIntensity  = 0.0f;
                } else if (smoothKnob2 <= shoegaze::kModZone2) {
                    const float t = (smoothKnob2 - shoegaze::kModZone1)
                                  / (shoegaze::kModZone2 - shoegaze::kModZone1);
                    pitchAmount  = 1.0f;
                    modIntensity = 0.5f * t;   // 0→0.5
                } else {
                    const float t = (smoothKnob2 - shoegaze::kModZone2)
                                  / (1.0f - shoegaze::kModZone2);
                    pitchAmount  = 1.0f;
                    modIntensity = 0.5f + 0.5f * t;   // 0.5→1.0
                }
                softFocus_.setPitchAmount(pitchAmount);
                softFocus_.setModIntensity(modIntensity);
            }
            sampleCounter_++;

            // 4. Compute crossfade envelope (chain order crossfade)
            float chainEnv = 1.0f;
            if (chainFadeCounter_ > 0) {
                --chainFadeCounter_;
                if (chainFadeCounter_ > shoegaze::kChainFadeHalf) {
                    // Fading out: from 1.0→0.0 over the first half
                    chainEnv = static_cast<float>(chainFadeCounter_ - shoegaze::kChainFadeHalf)
                             / static_cast<float>(shoegaze::kChainFadeHalf);
                } else {
                    // Swap chain at midpoint (once)
                    if (!chainFadeSwapped_) {
                        chainFadeSwapped_ = true;
                        chainReversed_ = !chainReversed_;
                    }
                    // Fading in: from 0.0→1.0 over the second half
                    chainEnv = 1.0f - static_cast<float>(chainFadeCounter_)
                                    / static_cast<float>(shoegaze::kChainFadeHalf);
                }
            }

            // 5. Compute variant fade envelope
            float variantEnv = 1.0f;
            if (variantFadeCounter_ > 0) {
                --variantFadeCounter_;
                if (variantFadeCounter_ > shoegaze::kVariantFadeHalf) {
                    variantEnv = static_cast<float>(variantFadeCounter_ - shoegaze::kVariantFadeHalf)
                               / static_cast<float>(shoegaze::kVariantFadeHalf);
                } else {
                    // Apply variant at midpoint (once)
                    if (!variantFadeSwapped_ && pendingVariantIdx_ >= 0) {
                        variantFadeSwapped_ = true;
                        currentVariantIdx_  = pendingVariantIdx_;
                        muff_.setVariant(variantTable_[currentVariantIdx_]);
                        pendingVariantIdx_  = -1;
                    }
                    variantEnv = 1.0f - static_cast<float>(variantFadeCounter_)
                                      / static_cast<float>(shoegaze::kVariantFadeHalf);
                }
            }

            const float envelope = chainEnv * variantEnv;

            // 6. Process signal chain
            float outL, outR;
            if (!chainReversed_) {
                // DEFAULT: BigMuff → SoftFocus
                // mono fuzz → passed as both channels to SoftFocus (it sums to mono internally)
                const float fuzzed = muff_.process(mono);
                softFocus_.process(fuzzed, fuzzed, outL, outR);
            } else {
                // REVERSED: SoftFocus → sum → BigMuff → decorrelation stereo
                float sfL, sfR;
                softFocus_.process(mono, mono, sfL, sfR);
                const float sfMono = (sfL + sfR) * 0.5f;
                const float fuzzed = muff_.process(sfMono);
                outL = fuzzed;
                outR = decorrAllpass_.process(fuzzed);
            }

            // 7. Apply envelope
            outL *= envelope;
            outR *= envelope;

            // 8. DC block
            outL = dcBlockL_.process(outL);
            outR = dcBlockR_.process(outR);

            // 9. Write output
            left[n]  = outL;
            right[n] = outR;
        }
    }

    ParameterMetadata getParameterMetadata(int paramIdx) override {
        switch (paramIdx) {
            case 0: return ParameterMetadata{ 0.0f, 1.0f, 0.5f };  // Fuzz
            case 1: return ParameterMetadata{ 0.0f, 1.0f, 0.3f };  // Depth/Width
            case 2: return ParameterMetadata{ 0.0f, 1.0f, 0.0f };  // Modulation
            default: return ParameterMetadata{ 0.0f, 1.0f, 0.0f };
        }
    }

    void setParamValue(int paramIdx, float value) override {
        switch (paramIdx) {
            case 0:
                knobFuzz_  = value;
                knob0Smoother_.setTarget(value);
                break;
            case 1:
                knobDepth_ = value;
                knob1Smoother_.setTarget(value);
                break;
            case 2:
                knobMod_   = value;
                knob2Smoother_.setTarget(value);
                break;
            default:
                break;
        }
    }

    bool isParamEnabled(int paramIdx, ParamSource source) override {
        switch (source) {
            case ParamSource::kParamSourceKnob:
                return paramIdx < 3;
            case ParamSource::kParamSourceExpression:
                // Expression pedal not mapped for this effect.
                // Future extension: map to Fuzz tone sweep (paramIdx 0).
                return false;
            default:
                return false;
        }
    }

    void handleAction(int actionIdx) override {
        if (actionIdx == static_cast<int>(endless::ActionId::kLeftFootSwitchPress)) {
            // TAP: cycle variant Triangle → RamsHead → NYC → Triangle
            if (variantFadeCounter_ == 0) {  // not already fading
                pendingVariantIdx_  = (currentVariantIdx_ + 1) % shoegaze::kNumVariants;
                variantFadeCounter_ = shoegaze::kVariantFadeSamples;
                variantFadeSwapped_ = false;
            }
        }
        else if (actionIdx == static_cast<int>(endless::ActionId::kLeftFootSwitchHold)) {
            // HOLD: toggle chain order with 20ms crossfade
            if (chainFadeCounter_ == 0) {   // not already crossfading
                chainFadeCounter_ = shoegaze::kChainFadeSamples;
                chainFadeSwapped_ = false;
                pendingChainSwap_ = true;
            }
        }
    }

    Color getStateLedColor() override {
        // Base color per variant
        Color variantColor;
        switch (currentVariantIdx_) {
            case 0:  variantColor = chainReversed_ ? Color::kDimYellow    : Color::kLightYellow; break;
            case 1:  variantColor = chainReversed_ ? Color::kDarkRed      : Color::kMagenta;     break;
            case 2:  variantColor = chainReversed_ ? Color::kDimGreen     : Color::kLightGreen;  break;
            default: variantColor = Color::kDimWhite; break;
        }
        return variantColor;
    }

private:
    // ── Variant table ────────────────────────────────────────────────────
    // Triangle(idx=0), RamsHead(idx=1), NYC(idx=2)
    static constexpr bigmuff::Variant variantTable_[shoegaze::kNumVariants] = {
        bigmuff::Variant::Triangle,
        bigmuff::Variant::RamsHead,
        bigmuff::Variant::NYC
    };

    // ── Sub-circuits ─────────────────────────────────────────────────────
    BigMuffCircuit   muff_;
    SoftFocusCircuit softFocus_;

    // ── Decorrelation allpass (reversed chain stereo recovery) ────────────
    // Allpass delay: ~1.3ms at 48kHz ≈ 62 samples, coefficient 0.5
    AllpassDelay<128> decorrAllpass_;

    // ── DC blocking (post-processing) ────────────────────────────────────
    OnePoleFilter dcBlockL_;
    OnePoleFilter dcBlockR_;

    // ── Raw knob smoothers ───────────────────────────────────────────────
    ParameterSmoother knob0Smoother_;  // Fuzz
    ParameterSmoother knob1Smoother_;  // Depth (50ms — prevent clicks on reverb steps)
    ParameterSmoother knob2Smoother_;  // Modulation

    // ── State ────────────────────────────────────────────────────────────
    bool     chainReversed_      = false;
    int      currentVariantIdx_  = 1;    // 0=Triangle, 1=RamsHead, 2=NYC
    int      pendingVariantIdx_  = -1;

    // Chain crossfade
    int      chainFadeCounter_   = 0;
    bool     chainFadeSwapped_   = false;
    bool     pendingChainSwap_   = false;  // unused but retained for clarity

    // Variant fade
    int      variantFadeCounter_ = 0;
    bool     variantFadeSwapped_ = false;

    // Optimization state
    uint32_t sampleCounter_      = 0u;
    bool     voicesEnabled_      = false;

    // Raw knob values (stored for reference / init state)
    float    knobFuzz_   = 0.5f;
    float    knobDepth_  = 0.3f;
    float    knobMod_    = 0.0f;
};

static PatchImpl patch;

Patch* Patch::getInstance() {
    return &patch;
}
