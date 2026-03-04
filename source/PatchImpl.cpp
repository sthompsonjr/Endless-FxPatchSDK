#include "Patch.h"
#include "dsp/DelayLine.h"
#include "dsp/Looper.h"

class PatchImpl : public Patch
{
  public:
    void setWorkingBuffer(std::span<float, kWorkingBufferSize> buffer) override
    {
        // Partition the working buffer:
        // First 96000 floats = 2 seconds of delay at 48kHz
        // Remaining floats = looper storage (~24s stereo)
        constexpr std::size_t kDelaySize = 96000;

        delay_.init(buffer.subspan(0, kDelaySize));
        looper_.init(buffer.subspan(kDelaySize, kWorkingBufferSize - kDelaySize));
    }

    void processAudio(std::span<float> audioBufferLeft,
                      std::span<float> audioBufferRight) override
    {
        auto delaySamples =
            static_cast<std::size_t>(delayTime_ * static_cast<float>(kSampleRate));

        for (auto leftIt = audioBufferLeft.begin(), rightIt = audioBufferRight.begin();
             leftIt != audioBufferLeft.end();
             ++leftIt, ++rightIt)
        {
            // Delay with feedback on both channels
            float delayedL =
                delay_.process(*leftIt + feedback_ * delay_.read(delaySamples), delaySamples);
            *leftIt = *leftIt * (1.0f - delayMix_) + delayedL * delayMix_;

            // Looper on both channels
            looper_.process(*leftIt, *rightIt, looperMix_);
        }
    }

    ParameterMetadata getParameterMetadata(int paramIdx) override
    {
        switch (paramIdx)
        {
            case 0: // Delay time: 0–2 seconds
                return ParameterMetadata{ 0.0f, 2.0f, 0.3f };
            case 1: // Feedback: 0–0.95
                return ParameterMetadata{ 0.0f, 0.95f, 0.4f };
            case 2: // Looper mix: 0–1
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
            default:
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
        }
    }

    bool isParamEnabled(int paramIdx, ParamSource source) override
    {
        switch (source)
        {
            case ParamSource::kParamSourceKnob:
                return paramIdx < 3;

            case ParamSource::kParamSourceExpression:
                return paramIdx == 2;

            default:
                return false;
        }
    }

    Color getStateLedColor() override
    {
        switch (looper_.getState())
        {
            case endless::dsp::Looper::State::kRecording:
                return Color::kRed;
            case endless::dsp::Looper::State::kPlaying:
                return Color::kLightGreen;
            default:
                return Color::kDimBlue;
        }
    }

    void setParamValue(int idx, float value) override
    {
        switch (idx)
        {
            case 0:
                delayTime_ = value;
                break;
            case 1:
                feedback_ = value;
                break;
            case 2:
                looperMix_ = value;
                break;
            default:
                break;
        }
    }

    void init() override {}

    void handleAction(int idx) override
    {
        if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchPress))
        {
            looper_.toggle();
        }
    }

  private:
    endless::dsp::DelayLine delay_;
    endless::dsp::Looper looper_;

    float delayTime_ = 0.3f;
    float feedback_ = 0.4f;
    float delayMix_ = 0.5f;
    float looperMix_ = 0.5f;
};

static PatchImpl patch;

Patch* Patch::getInstance()
{
    return &patch;
}
