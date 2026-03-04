#pragma once

#include <cstddef>
#include <span>

namespace endless::dsp
{

/// A circular-buffer delay line that operates on externally provided memory.
/// Attach it to a region of the working buffer via init() before use.
class DelayLine
{
  public:
    /// Attach this delay line to a pre-allocated region of memory.
    /// The buffer is zero-filled to avoid garbage on first read.
    void init(std::span<float> buffer)
    {
        buffer_ = buffer;
        writePos_ = 0;
        for (auto& s : buffer_)
        {
            s = 0.0f;
        }
    }

    /// Write one sample into the delay line and advance the write head.
    void write(float sample)
    {
        buffer_[writePos_] = sample;
        writePos_++;
        if (writePos_ >= buffer_.size())
        {
            writePos_ = 0;
        }
    }

    /// Read a sample at `delaySamples` behind the write head.
    /// Clamped to buffer size - 1.
    [[nodiscard]] float read(std::size_t delaySamples) const
    {
        if (delaySamples >= buffer_.size())
        {
            delaySamples = buffer_.size() - 1;
        }
        std::size_t readPos =
            (writePos_ + buffer_.size() - delaySamples) % buffer_.size();
        return buffer_[readPos];
    }

    /// Read with fractional delay using linear interpolation.
    /// Useful for modulated delays (chorus, flanger).
    [[nodiscard]] float readInterpolated(float delaySamples) const
    {
        if (delaySamples < 0.0f)
        {
            delaySamples = 0.0f;
        }
        auto maxDelay = static_cast<float>(buffer_.size() - 1);
        if (delaySamples > maxDelay)
        {
            delaySamples = maxDelay;
        }

        auto intPart = static_cast<std::size_t>(delaySamples);
        float frac = delaySamples - static_cast<float>(intPart);

        float s0 = read(intPart);
        float s1 = read(intPart + 1);
        return s0 + frac * (s1 - s0);
    }

    /// Write a sample and return the delayed output in one call.
    /// Reads first, then writes — correct order for feedback topologies.
    float process(float input, std::size_t delaySamples)
    {
        float output = read(delaySamples);
        write(input);
        return output;
    }

    /// Returns the capacity (max delay in samples).
    [[nodiscard]] std::size_t capacity() const { return buffer_.size(); }

  private:
    std::span<float> buffer_{};
    std::size_t writePos_ = 0;
};

} // namespace endless::dsp
