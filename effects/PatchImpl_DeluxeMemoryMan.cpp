/*
 * PatchImpl_DeluxeMemoryMan.cpp — EHX Deluxe Memory Man (EH-7850) skeleton
 *
 * Knob 0 (left):   Delay time   [0,1] → 82–550 ms linear
 * Knob 1 (center): Feedback     log taper (v*v); self-oscillation near 80%
 * Knob 2 (right):  Chorus/Vib   <0.5 = Chorus 3 Hz LFO, >=0.5 = Vibrato 6 Hz LFO
 * Left hold:       Runaway mode (toggles self-oscillation clamp)
 *
 * Session 3a:  class skeleton, init(), setWorkingBuffer(), stub processAudio(),
 *              stub getStateLedColor(). No audio processing logic.
 * Session 3aa: API audit — corrected processAudio stub, parameter defaults,
 *              setParamValue storage pattern.
 * Session 3b:  full per-sample signal chain, dry/wet mix, vibrato dry-mute,
 *              control-rate dispatch (once per block), LED pulse counter.
 *              getStateLedColor() three-state implementation (3c merged into 3b).
 *              HANDOFF: skip session 3c → proceed to session 3d (test harness).
 */

#include "sdk/Patch.h"
#include "wdf/DmmDelayCircuit.h"

// SESSION 3aa VERIFIED API FACTS (do not remove — session 3b depends on these)
// setWorkingBuffer : std::span<float, kWorkingBufferSize> — stores .data() + size
// processAudio     : left + right dynamic spans; left.size() == right.size()
// setParamValue    : indices 0/1/2 stored in paramDelay_/paramFeedback_/paramMode_
//                    circuit setters NOT called here — called at control rate in processAudio
// handleAction     : kLeftFootSwitchHold TOGGLES isRunaway_ (no hold-release event in API,
//                    confirmed from PatchImpl_OpticalComp.cpp)
//                    kLeftFootSwitchPress: no action
// getStateLedColor : returns Color enum; ledPulseCounter_ incremented in processAudio (session 3b)
// Control rate     : per-sample counter in processAudio (see PatchImpl_PowerPuff.cpp pattern);
//                    session 3b decides divider value
// Stereo handling  : DMM is mono; write wet sample to BOTH left[i] and right[i]
//                    (confirmed from PatchImpl_PowerPuff.cpp: left[i]=y; right[i]=y)

class PatchImpl : public Patch
{
public:
    // -------------------------------------------------------------------------
    // Platform constants (528 MHz measured clock, not marketed 720 MHz)
    // -------------------------------------------------------------------------
    static constexpr float kClockHz                     = 528'000'000.0f;
    static constexpr float kSampleRateF                 = 48'000.0f;
    static constexpr float kCyclesPerSamp               = kClockHz / kSampleRateF; // 11,000
    static constexpr float kCycleFailPct                = 0.60f; // 6,600 cycles hard fail
    static constexpr float kCycleWarnPct                = 0.40f; // 4,400 cycles warning
    static constexpr int   kRunawayLedHalfPeriodSamples = 12'000; // 250 ms @ 48 kHz

    // -------------------------------------------------------------------------
    // Patch interface
    // -------------------------------------------------------------------------

    void init() override
    {
        circuit_.init(static_cast<float>(kSampleRate), workingBuffer_, workingBufferSize_);
    }

    void setWorkingBuffer(std::span<float, kWorkingBufferSize> buffer) override
    {
        workingBuffer_     = buffer.data();
        workingBufferSize_ = static_cast<size_t>(kWorkingBufferSize);
    }

    void processAudio(std::span<float> audioBufferLeft,
                      std::span<float> audioBufferRight) noexcept override
    {
        // Cycle budget: DmmDelayCircuit ~1,275–2,295 cycles/sample (measured headroom >8,700)
        // Clock: 528 MHz, available: 11,000 cycles/sample at 48 kHz
        // Control-rate overhead amortized: applied once per block (before sample loop)

        // Apply parameters once per block (SoftFocus pattern: dispatch outside sample loop).
        circuit_.setDelayKnob(paramDelay_);
        circuit_.setFeedbackKnob(paramFeedback_);
        circuit_.setModeKnob(paramMode_);
        isVibrato_ = (paramMode_ >= 0.5f);

        for (auto leftIt = audioBufferLeft.begin(), rightIt = audioBufferRight.begin();
             leftIt != audioBufferLeft.end();
             ++leftIt, ++rightIt)
        {
            const float dry = *leftIt;
            const float wet = circuit_.process(dry);

            // Vibrato mode: hard mute dry (EH-7850 hardware behavior).
            // Chorus mode: 50/50 fixed ratio — not user-adjustable per original hardware.
            const float out = isVibrato_ ? wet : (0.5f * dry + 0.5f * wet);

            *leftIt  = out;
            *rightIt = out;

            // Runaway LED pulse: alternates kRed/kDarkRed every 250 ms (12,000 samples).
            if (++ledPulseCounter_ >= kRunawayLedHalfPeriodSamples) {
                ledPulseCounter_ = 0;
                ledPulseState_   = !ledPulseState_;
            }
        }
    }

    ParameterMetadata getParameterMetadata(int paramIdx) override
    {
        switch (paramIdx)
        {
            case 0: return ParameterMetadata{0.0f, 1.0f, 0.3f};  // Delay time (~222ms)
            case 1: return ParameterMetadata{0.0f, 1.0f, 0.25f}; // Feedback (modest repeats)
            case 2: return ParameterMetadata{0.0f, 1.0f, 0.0f}; // Chorus/Vib
            default: return ParameterMetadata{0.0f, 1.0f, 0.5f};
        }
    }

    void setParamValue(int paramIdx, float value) override
    {
        // Store only — circuit setters are called at control rate inside processAudio()
        // (session 3b). isVibrato_ is derived state kept in sync here for session 3b to read.
        switch (paramIdx)
        {
            case 0: paramDelay_    = value; break;
            case 1: paramFeedback_ = value; break;
            case 2:
                paramMode_ = value;
                isVibrato_ = (value >= 0.5f);
                break;
            default: break;
        }
    }

    bool isParamEnabled(int paramIdx, ParamSource source) override
    {
        if (source == ParamSource::kParamSourceKnob)
            return paramIdx >= 0 && paramIdx <= 2;
        return false;
    }

    void handleAction(int actionIdx) override
    {
        if (actionIdx == static_cast<int>(endless::ActionId::kLeftFootSwitchHold))
        {
            isRunaway_ = !isRunaway_;
            circuit_.setRunawayMode(isRunaway_);
            ledPulseCounter_ = 0;
            ledPulseState_   = false;
        }
        // kLeftFootSwitchPress: no action for DMM
    }

    Color getStateLedColor() noexcept override
    {
        if (isRunaway_) {
            return ledPulseState_ ? Color::kRed : Color::kDarkRed;
        }
        if (isVibrato_) {
            return Color::kLightBlueColor;
        }
        return Color::kLightYellow;
    }

private:
    DmmDelayCircuit circuit_;

    float*  workingBuffer_     = nullptr;
    size_t  workingBufferSize_ = 0;

    // Stored knob values — consumed by processAudio() at control rate (session 3b)
    float   paramDelay_    = 0.3f;   // knob 0: delay time [0,1]
    float   paramFeedback_ = 0.25f;  // knob 1: feedback [0,1]
    float   paramMode_     = 0.0f;   // knob 2: chorus/vibrato [0,1]

    bool    isRunaway_       = false;
    bool    isVibrato_       = false;
    int     ledPulseCounter_ = 0;
    bool    ledPulseState_   = false;
};

static PatchImpl patch;

Patch* Patch::getInstance()
{
    return &patch;
}
