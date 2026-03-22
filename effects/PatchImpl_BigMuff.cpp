#include "sdk/Patch.h"
#include "wdf/WdfBigMuffCircuit.h"
#include "dsp/ParameterSmoother.h"
#include <cmath>

class PatchImpl : public Patch
{
  public:
    void init() override
    {
        circuit_.init(static_cast<float>(kSampleRate), bigmuff::Variant::RamsHead);
        circuit_.setSustain(0.5f);
        circuit_.setTone(0.5f);
        circuit_.setVolume(1.0f);

        sustainSmoother_.init(static_cast<float>(kSampleRate), 50.0f);
        sustainSmoother_.snapTo(0.5f);
        toneSmoother_.init(static_cast<float>(kSampleRate), 30.0f);
        toneSmoother_.snapTo(0.5f);
        volumeSmoother_.init(static_cast<float>(kSampleRate), 20.0f);
        volumeSmoother_.snapTo(1.0f);
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
            // Smooth parameters per-sample for click-free transitions
            circuit_.setSustain(sustainSmoother_.process());
            circuit_.setTone(toneSmoother_.process());
            circuit_.setVolume(volumeSmoother_.process());

            // Mono sum input
            const float in = 0.5f * (*leftIt + *rightIt);

            // Apply variant fade for click-free variant switching
            float fadeMult = 1.0f;
            if (fadeSamplesRemaining_ > 0) {
                --fadeSamplesRemaining_;
                if (fadeSamplesRemaining_ > kFadeHalf_) {
                    // Fading out
                    fadeMult = static_cast<float>(fadeSamplesRemaining_ - kFadeHalf_)
                               / static_cast<float>(kFadeHalf_);
                } else {
                    // Switch variant at midpoint
                    if (fadeSamplesRemaining_ == kFadeHalf_ - 1) {
                        circuit_.setVariant(pendingVariant_);
                    }
                    // Fading in
                    fadeMult = 1.0f - static_cast<float>(fadeSamplesRemaining_)
                                      / static_cast<float>(kFadeHalf_);
                }
            }

            const float out = circuit_.process(in) * fadeMult;
            *leftIt = out;
            *rightIt = out;
        }
    }

    ParameterMetadata getParameterMetadata(int paramIdx) override
    {
        switch (paramIdx)
        {
            case 0: return ParameterMetadata{ 0.0f, 1.0f, 0.5f }; // Sustain
            case 1: return ParameterMetadata{ 0.0f, 1.0f, 0.5f }; // Tone
            case 2: return ParameterMetadata{ 0.0f, 1.0f, 1.0f }; // Volume
            default: return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
        }
    }

    void setParamValue(int idx, float value) override
    {
        switch (idx)
        {
            case 0: sustainSmoother_.setTarget(value); break;
            case 1: toneSmoother_.setTarget(value); break;
            case 2: volumeSmoother_.setTarget(value); break;
            default: break;
        }
    }

    bool isParamEnabled(int paramIdx, ParamSource source) override
    {
        switch (source)
        {
            case ParamSource::kParamSourceKnob:       return paramIdx < 3;
            case ParamSource::kParamSourceExpression: return paramIdx == 2;
            default:                                  return false;
        }
    }

    void handleAction(int idx) override
    {
        if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchPress))
        {
            // Cycle through variants with a brief fade to avoid clicks
            const int next = (static_cast<int>(currentVariant_) + 1) % 3;
            pendingVariant_ = static_cast<bigmuff::Variant>(next);
            fadeSamplesRemaining_ = kFadeSamples_;
        }
        else if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchHold))
        {
            // Hold: toggle overdrive (boost sustain to max temporarily)
            overdriveOn_ = !overdriveOn_;
            sustainSmoother_.setTarget(overdriveOn_ ? 1.0f
                                                    : sustainTarget_);
        }
    }

    Color getStateLedColor() override
    {
        if (overdriveOn_) {
            switch (currentVariant_) {
                case bigmuff::Variant::RamsHead: return Color::kRed;
                case bigmuff::Variant::CivilWar: return Color::kLightYellow;
                case bigmuff::Variant::Triangle:  return Color::kDimWhite;
            }
        }
        switch (currentVariant_) {
            case bigmuff::Variant::RamsHead: return Color::kMagenta;
            case bigmuff::Variant::CivilWar: return Color::kDarkRed;
            case bigmuff::Variant::Triangle:  return Color::kLightYellow;
        }
        return Color::kMagenta;
    }

  private:
    static constexpr int kFadeSamples_ = 960;  // 20ms @ 48kHz
    static constexpr int kFadeHalf_    = kFadeSamples_ / 2;

    BigMuffCircuit circuit_;
    ParameterSmoother sustainSmoother_;
    ParameterSmoother toneSmoother_;
    ParameterSmoother volumeSmoother_;

    bigmuff::Variant currentVariant_ = bigmuff::Variant::RamsHead;
    bigmuff::Variant pendingVariant_ = bigmuff::Variant::RamsHead;
    int fadeSamplesRemaining_ = 0;

    float sustainTarget_ = 0.5f;
    bool overdriveOn_    = false;
};

static PatchImpl patch;

Patch* Patch::getInstance()
{
    return &patch;
}
