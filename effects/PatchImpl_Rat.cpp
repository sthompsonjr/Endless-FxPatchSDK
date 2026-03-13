#include "sdk/Patch.h"
#include "wdf/WdfRatCircuit.h"
#include "dsp/Saturation.h"
#include <cmath>

class PatchImpl : public Patch
{
  public:
    void init() override
    {
        circuit_.init(static_cast<float>(kSampleRate));
        circuit_.setVolume(0.5f);
        circuit_.setDistortion(0.5f);
        circuit_.setFilter(0.5f);
    }

    void setWorkingBuffer(std::span<float, kWorkingBufferSize> buffer) override
    {
        (void)buffer;
    }

    void processAudio(std::span<float> audioBufferLeft,
                      std::span<float> audioBufferRight) override
    {
        for (auto leftIt = audioBufferLeft.begin(), rightIt = audioBufferRight.begin();
             leftIt != audioBufferLeft.end();
             ++leftIt, ++rightIt)
        {
            float out = circuit_.process(*leftIt);
            *leftIt = out;
            *rightIt = out;
        }
    }

    ParameterMetadata getParameterMetadata(int paramIdx) override
    {
        switch (paramIdx)
        {
            case 0: // Distortion
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
            case 1: // Filter (REVERSED: 0=bright, 1=dark)
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
            case 2: // Volume
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
            default:
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
        }
    }

    void setParamValue(int idx, float value) override
    {
        switch (idx)
        {
            case 0:
                circuit_.setDistortion(value);
                break;
            case 1:
                // Filter: 0 = bright (20kHz), 1 = dark (2kHz)
                circuit_.setFilter(value);
                break;
            case 2:
                circuit_.setVolume(value);
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
                return paramIdx == 0; // Expression pedal controls Distortion

            default:
                return false;
        }
    }

    void handleAction(int idx) override
    {
        if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchPress))
        {
            // Tap: cycle through RAT variants
            int next = (static_cast<int>(currentVariant_) + 1)
                       % static_cast<int>(RatVariant::kCount);
            currentVariant_ = static_cast<RatVariant>(next);
            circuit_.setVariant(currentVariant_);
        }
        else if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchHold))
        {
            // Hold: cycle slew character (Stock → Fast → Slow)
            slewMode_ = (slewMode_ + 1) % 3;
            circuit_.setSlewCharacter(slewMode_);
        }
    }

    Color getStateLedColor() override
    {
        switch (currentVariant_)
        {
            case RatVariant::Original:     return Color::kRed;
            case RatVariant::TurboRat:     return Color::kLightYellow;
            case RatVariant::YouDirtyRat:  return Color::kDarkLime;
            case RatVariant::WhiteRat:     return Color::kDimWhite;
            case RatVariant::GermaniumMod: return Color::kMagenta;
            default:                       return Color::kRed;
        }
    }

  private:
    WdfRatCircuit circuit_;
    RatVariant currentVariant_ = RatVariant::Original;
    int slewMode_ = 0;
};

static PatchImpl patch;

Patch* Patch::getInstance()
{
    return &patch;
}
