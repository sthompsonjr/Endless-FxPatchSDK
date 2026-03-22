#pragma once

#include <cstddef>
#include <span>

namespace endless::dsp
{

/// A stereo record/playback looper that operates on externally provided memory.
/// The buffer is split in half for left and right channels.
/// State cycles: Idle -> Recording -> Playing -> Idle via toggle().
class Looper
{
  public:
    enum class State
    {
        kIdle,
        kRecording,
        kPlaying,
    };

    /// Attach this looper to a pre-allocated region of memory.
    /// The buffer is split in half: first half = left, second half = right.
    void init(std::span<float> buffer)
    {
        auto halfSize = buffer.size() / 2;
        leftBuffer_ = buffer.subspan(0, halfSize);
        rightBuffer_ = buffer.subspan(halfSize, halfSize);
        state_ = State::kIdle;
        loopLength_ = 0;
        playPos_ = 0;
        recPos_ = 0;
    }

    /// Cycle the looper state: Idle -> Recording -> Playing -> Idle.
    /// Call this from handleAction() on footswitch press.
    void toggle()
    {
        switch (state_)
        {
            case State::kIdle:
                state_ = State::kRecording;
                recPos_ = 0;
                break;

            case State::kRecording:
                loopLength_ = recPos_;
                playPos_ = 0;
                state_ = State::kPlaying;
                break;

            case State::kPlaying:
                state_ = State::kIdle;
                loopLength_ = 0;
                break;
        }
    }

    /// Process one stereo sample pair in place.
    /// Recording: stores the input. Playing: crossfade-mixes loop with input.
    /// @param mix  Wet/dry (0.0 = fully dry, 1.0 = fully wet).
    void process(float& inLeft, float& inRight, float mix)
    {
        switch (state_)
        {
            case State::kIdle:
                break;

            case State::kRecording:
                if (recPos_ < leftBuffer_.size())
                {
                    leftBuffer_[recPos_] = inLeft;
                    rightBuffer_[recPos_] = inRight;
                    recPos_++;
                }
                else
                {
                    // Buffer full — auto-transition to playback
                    loopLength_ = recPos_;
                    playPos_ = 0;
                    state_ = State::kPlaying;
                }
                break;

            case State::kPlaying:
                if (loopLength_ > 0)
                {
                    float loopL = leftBuffer_[playPos_];
                    float loopR = rightBuffer_[playPos_];
                    inLeft = inLeft * (1.0f - mix) + loopL * mix;
                    inRight = inRight * (1.0f - mix) + loopR * mix;
                    playPos_++;
                    if (playPos_ >= loopLength_)
                    {
                        playPos_ = 0;
                    }
                }
                break;
        }
    }

    [[nodiscard]] State getState() const { return state_; }
    [[nodiscard]] std::size_t getLoopLength() const { return loopLength_; }
    [[nodiscard]] std::size_t maxLength() const { return leftBuffer_.size(); }

  private:
    std::span<float> leftBuffer_{};
    std::span<float> rightBuffer_{};
    State state_ = State::kIdle;
    std::size_t loopLength_ = 0;
    std::size_t playPos_ = 0;
    std::size_t recPos_ = 0;
};

} // namespace endless::dsp
