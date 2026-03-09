#include "Patch.h"
#include "../wdf/DOD250Circuit.h"

class PatchImpl : public Patch
{
  public:
    void init() override
    {
        circuit_.init(static_cast<float>(kSampleRate));
    }

    void setWorkingBuffer(std::span<float, kWorkingBufferSize> buffer) override
    {
        // DOD 250 is purely real-time — no delay lines or looper memory needed.
        (void)buffer;
    }

    void processAudio(std::span<float> audioBufferLeft,
                      std::span<float> audioBufferRight) override
    {
        for (auto leftIt = audioBufferLeft.begin(), rightIt = audioBufferRight.begin();
             leftIt != audioBufferLeft.end();
             ++leftIt, ++rightIt)
        {
            if (bypassed_)
            {
                // True bypass: pass through unmodified
                continue;
            }

            // Mono processing through DOD 250 circuit
            float out = circuit_.process(*leftIt);
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
            case 2: // Level
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
                circuit_.setGain(value);
                break;
            case 1:
                circuit_.setTone(value);
                break;
            case 2:
                circuit_.setLevel(value);
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
                return paramIdx == 2; // Expression pedal controls Level

            default:
                return false;
        }
    }

    void handleAction(int idx) override
    {
        if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchPress))
        {
            bypassed_ = !bypassed_;
        }
    }

    Color getStateLedColor() override
    {
        return bypassed_ ? Color::kDimWhite : Color::kDarkRed;
    }

  private:
    DOD250Circuit circuit_;
    bool bypassed_ = false;
};

static PatchImpl patch;

Patch* Patch::getInstance()
{
    return &patch;
}
