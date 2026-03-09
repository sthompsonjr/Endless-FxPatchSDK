#include "Patch.h"
#include "../wdf/DOD250Circuit.h"
#include "../dsp/ParameterSmoother.h"
#include "../dsp/Saturation.h"
#include <cmath>

class PatchImpl : public Patch
{
  public:
    void init() override
    {
        circuit_.init(static_cast<float>(kSampleRate));
        circuit_.setLevel(1.0f); // Hardware handles effect level

        boostSmoother_.init(static_cast<float>(kSampleRate), 20.0f);
        boostSmoother_.snapTo(1.0f); // Unity gain (no boost)
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

            // Apply boost (always runs smoother for click-free transitions)
            float gain = boostSmoother_.process();
            out *= gain;
            out = sat::softClip(out);

            *leftIt = out;
            *rightIt = out;
        }
    }

    ParameterMetadata getParameterMetadata(int paramIdx) override
    {
        switch (paramIdx)
        {
            case 0: // Gain
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
            case 1: // Tone
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
            case 2: // Boost Level
                return ParameterMetadata{ 0.0f, 1.0f, 0.0f };
            default:
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
        }
    }

    void setParamValue(int idx, float value) override
    {
        switch (idx)
        {
            case 0:
                circuit_.setGain(value);
                break;
            case 1:
                circuit_.setTone(value);
                break;
            case 2:
                // Boost Level: 0–1 maps to 0–20dB (1x to 10x linear gain)
                boostGain_ = powf(10.0f, value);
                if (boostEnabled_)
                {
                    boostSmoother_.setTarget(boostGain_);
                }
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
                return paramIdx == 2; // Expression pedal controls Boost Level

            default:
                return false;
        }
    }

    void handleAction(int idx) override
    {
        if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchPress))
        {
            boostEnabled_ = !boostEnabled_;
            boostSmoother_.setTarget(boostEnabled_ ? boostGain_ : 1.0f);
        }
    }

    Color getStateLedColor() override
    {
        return boostEnabled_ ? Color::kRed : Color::kDarkRed;
    }

  private:
    DOD250Circuit circuit_;
    ParameterSmoother boostSmoother_;
    float boostGain_ = 1.0f;
    bool boostEnabled_ = false;
};

static PatchImpl patch;

Patch* Patch::getInstance()
{
    return &patch;
}
