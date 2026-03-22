/*
 * PatchImpl_SoftFocus.cpp — Yamaha SPX500 "Soft Focus" shimmer reverb
 *
 * Knob 0 (left):   Reverb Amount — decay 0.3s-4.0s, wet mix 0-75%
 * Knob 1 (center): Pitch Amount  — base detune 0-15 cents per voice
 * Knob 2 (right):  Mod Intensity — coupled LFO rate + depth + Symphonic
 *
 * Expression pedal: Real-time wet mix swell (overrides knob 0 wet depth)
 * Left footswitch tap:  Toggle Pure Shimmer (mutes dry signal)
 * Left footswitch hold: Toggle Freeze (gates reverb input)
 *
 * LED: DimBlue (normal), Magenta (pure shimmer), LightBlue (freeze)
 */

#include "sdk/Patch.h"
#include "dsp/SoftFocusCircuit.h"

class PatchImpl : public Patch
{
  public:
    void init() override
    {
        circuit_.init(static_cast<float>(kSampleRate));
        circuit_.setReverbAmount(0.5f);
        circuit_.setPitchAmount(0.5f);
        circuit_.setModIntensity(0.3f);
    }

    void setWorkingBuffer(std::span<float, kWorkingBufferSize> buffer) override
    {
        circuit_.assignWorkingBuffer(buffer.data(), kWorkingBufferSize);
    }

    void processAudio(std::span<float> audioBufferLeft,
                      std::span<float> audioBufferRight) override
    {
        for (auto leftIt = audioBufferLeft.begin(), rightIt = audioBufferRight.begin();
             leftIt != audioBufferLeft.end();
             ++leftIt, ++rightIt)
        {
            float outL, outR;
            circuit_.process(*leftIt, *rightIt, outL, outR);
            *leftIt = outL;
            *rightIt = outR;
        }
    }

    ParameterMetadata getParameterMetadata(int paramIdx) override
    {
        switch (paramIdx)
        {
            case 0: return ParameterMetadata{0.0f, 1.0f, 0.5f};  // Reverb Amount
            case 1: return ParameterMetadata{0.0f, 1.0f, 0.5f};  // Pitch Amount
            case 2: return ParameterMetadata{0.0f, 1.0f, 0.3f};  // Mod Intensity
            default: return ParameterMetadata{0.0f, 1.0f, 0.5f};
        }
    }

    void setParamValue(int idx, float value) override
    {
        switch (idx)
        {
            case 0:
                knob0_ = value;
                circuit_.setReverbAmount(value);
                break;
            case 1:
                circuit_.setPitchAmount(value);
                break;
            case 2:
                circuit_.setModIntensity(value);
                break;
            default:
                break;
        }
    }

    bool isParamEnabled(int paramIdx, ParamSource source) override
    {
        switch (source)
        {
            case ParamSource::kParamSourceKnob:
                return paramIdx < 3;
            case ParamSource::kParamSourceExpression:
                // Expression pedal controls wet mix swell (same as knob 0)
                return paramIdx == 0;
            default:
                return false;
        }
    }

    void handleAction(int idx) override
    {
        if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchPress))
        {
            // Tap: toggle Pure Shimmer (mutes dry signal)
            pureShimmerMode_ = !pureShimmerMode_;
            circuit_.setDryMute(pureShimmerMode_);
        }
        else if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchHold))
        {
            // Hold: toggle Freeze (gates reverb input, tail sustains)
            freezeMode_ = !freezeMode_;
            circuit_.setFreeze(freezeMode_);
        }
    }

    Color getStateLedColor() override
    {
        if (freezeMode_) return Color::kLightBlueColor;
        if (pureShimmerMode_) return Color::kMagenta;
        return Color::kDimBlue;
    }

  private:
    SoftFocusCircuit circuit_;
    float knob0_ = 0.5f;
    bool pureShimmerMode_ = false;
    bool freezeMode_ = false;
};

static PatchImpl patch;

Patch* Patch::getInstance()
{
    return &patch;
}
