#include "sdk/Patch.h"

#include <cmath>

class PatchImpl : public Patch
{
  public:
    void setWorkingBuffer(std::span<float, kWorkingBufferSize> /* buffer */) override {}

    void processAudio(std::span<float> audioBufferLeft, std::span<float> audioBufferRight) override
    {
        auto values = std::pow(2.0f, (1.0f - paramValue) * 4.0f + 5.0f);
        auto valuesInv = 1.0f / values;

        for (auto leftIt = audioBufferLeft.begin(), rightIt = audioBufferRight.begin();
             leftIt != audioBufferLeft.end();
             ++leftIt, ++rightIt)
        {
            *leftIt = std::round(*leftIt * values) * valuesInv;
            *rightIt = std::round(*rightIt * values) * valuesInv;
        }
    }

    ParameterMetadata getParameterMetadata(int /* paramIdx */) override
    {
        return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
    }

    bool isParamEnabled(int paramIdx, ParamSource source) override
    {
        switch (source)
        {
            case ParamSource::kParamSourceKnob:
                return paramIdx < 3 ? true : false;

            case ParamSource::kParamSourceExpression:
                return paramIdx == 0 ? true : false;

            default:
                return false;
        }
    }

    Color getStateLedColor() override { return state ? Color::kBlue : Color::kDimBlue; }

    void setParamValue(int idx, float value) override
    {
        if (idx == 0)
        {
            paramValue = value;
        }
    }

    void init() override {}

    void handleAction(int /* idx */) override { state = !state; }

  private:
    bool state = false;
    float paramValue = 0.5f;
};

static PatchImpl patch;

Patch* Patch::getInstance()
{
    return &patch;
}
