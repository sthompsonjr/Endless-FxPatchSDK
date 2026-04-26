/*
 * PatchImpl_DeluxeMemoryMan.cpp — EHX Deluxe Memory Man (EH-7850) skeleton
 *
 * Knob 0 (left):   Delay time   [0,1] → 82–550 ms linear
 * Knob 1 (center): Feedback     log taper (v*v); self-oscillation near 80%
 * Knob 2 (right):  Chorus/Vib   <0.5 = Chorus 3 Hz LFO, >=0.5 = Vibrato 6 Hz LFO
 * Left hold:       Runaway mode (toggles self-oscillation clamp)
 *
 * Session 3a: class skeleton, init(), setWorkingBuffer(), stub processAudio(),
 *             stub getStateLedColor(). No audio processing logic.
 * Session 3b: replace stub processAudio() with full per-sample signal chain.
 */

#include "sdk/Patch.h"
#include "wdf/DmmDelayCircuit.h"

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
        // Session 3a stub: pass input to output unchanged.
        // Session 3b replaces this with the full DMM signal chain.
        (void)audioBufferLeft;
        (void)audioBufferRight;
    }

    ParameterMetadata getParameterMetadata(int paramIdx) override
    {
        switch (paramIdx)
        {
            case 0: return ParameterMetadata{0.0f, 1.0f, 0.5f}; // Delay time
            case 1: return ParameterMetadata{0.0f, 1.0f, 0.3f}; // Feedback
            case 2: return ParameterMetadata{0.0f, 1.0f, 0.0f}; // Chorus/Vib
            default: return ParameterMetadata{0.0f, 1.0f, 0.5f};
        }
    }

    void setParamValue(int paramIdx, float value) override
    {
        switch (paramIdx)
        {
            case 0:
                circuit_.setDelayKnob(value);
                break;
            case 1:
                circuit_.setFeedbackKnob(value);
                break;
            case 2:
                isVibrato_ = (value >= 0.5f);
                circuit_.setModeKnob(value);
                break;
            default:
                break;
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

    Color getStateLedColor() override
    {
        // Session 3a stub: warm amber for chorus mode (default).
        // Session 3b: kLightYellow=chorus, kDimCobalt=vibrato, kRed pulsing=runaway.
        return Color::kLightYellow;
    }

private:
    DmmDelayCircuit circuit_;

    float*  workingBuffer_     = nullptr;
    size_t  workingBufferSize_ = 0;

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
