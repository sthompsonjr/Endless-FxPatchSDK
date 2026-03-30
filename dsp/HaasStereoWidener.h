#ifndef DSP_HAAS_STEREO_WIDENER_H
#define DSP_HAAS_STEREO_WIDENER_H

#include "CircularBuffer.h"
#include "OnePoleFilter.h"
#include <algorithm>

namespace haas_constants {
    constexpr float min_delay_ms       = 5.0f;
    constexpr float max_delay_ms       = 40.0f;
    constexpr float default_delay_ms   = 15.0f;
    constexpr float min_feedback       = 0.0f;
    constexpr float max_feedback       = 0.7f;
    constexpr float default_feedback   = 0.4f;
    constexpr float polarity_normal    = 1.0f;
    constexpr float polarity_inverted  = -1.0f;
    constexpr float diffusion_cutoff_hz = 10000.0f;
}

// Haas (precedence) effect stereo widener
// Creates perceived spatial width using 5-40 ms cross-delays
// Mono-compatible when polarity is inverted
template <int MaxDelaySamples = 2048>  // 42.6 ms @ 48 kHz
class HaasStereoWidener {
    static_assert((MaxDelaySamples & (MaxDelaySamples - 1)) == 0,
                  "MaxDelaySamples must be a power of 2");

public:
    HaasStereoWidener() noexcept = default;

    void init(float sampleRate) noexcept {
        m_sampleRate = std::clamp(sampleRate, 8000.0f, 192000.0f);
        m_delayL.reset();
        m_delayR.reset();
        m_diffusionL.init(m_sampleRate);
        m_diffusionL.setType(OnePoleFilter::Type::Lowpass);
        m_diffusionL.setFrequency(haas_constants::diffusion_cutoff_hz);
        m_diffusionR.init(m_sampleRate);
        m_diffusionR.setType(OnePoleFilter::Type::Lowpass);
        m_diffusionR.setFrequency(haas_constants::diffusion_cutoff_hz);
        updateDelaySamples();
    }

    void setDelayMs(float delay_ms) noexcept {
        m_delayMs = std::clamp(delay_ms,
                               haas_constants::min_delay_ms,
                               haas_constants::max_delay_ms);
        updateDelaySamples();
    }

    void setFeedback(float feedback) noexcept {
        m_feedback = std::clamp(feedback,
                                haas_constants::min_feedback,
                                haas_constants::max_feedback);
    }

    void setInvertPolarity(bool invert) noexcept {
        m_invertPolarity = invert;
    }

    void setDiffusionEnabled(bool enabled) noexcept {
        m_diffusionEnabled = enabled;
    }

    void process(float inputL, float inputR,
                 float& outputL, float& outputR) noexcept {
        m_delayL.write(inputL);
        m_delayR.write(inputR);

        float delayedL = m_delayL.readLinear(m_delaySamples);
        float delayedR = m_delayR.readLinear(m_delaySamples);

        if (m_diffusionEnabled) {
            delayedL = m_diffusionL.process(delayedL);
            delayedR = m_diffusionR.process(delayedR);
        }

        const float polarity = m_invertPolarity ? -1.0f : 1.0f;

        outputL = inputL + m_feedback * polarity * delayedR;
        outputR = inputR + m_feedback * polarity * delayedL;
    }

    void reset() noexcept {
        m_delayL.reset();
        m_delayR.reset();
        m_diffusionL.reset();
        m_diffusionR.reset();
    }

    float getDelayMs()        const noexcept { return m_delayMs; }
    float getFeedback()       const noexcept { return m_feedback; }
    bool  isInvertedPolarity() const noexcept { return m_invertPolarity; }
    bool  isDiffusionEnabled() const noexcept { return m_diffusionEnabled; }

private:
    float m_sampleRate      = 48000.0f;
    float m_delayMs         = haas_constants::default_delay_ms;
    float m_delaySamples    = 0.0f;
    float m_feedback        = haas_constants::default_feedback;
    bool  m_invertPolarity  = true;
    bool  m_diffusionEnabled = false;

    CircularBuffer<float, MaxDelaySamples> m_delayL;
    CircularBuffer<float, MaxDelaySamples> m_delayR;

    OnePoleFilter m_diffusionL;
    OnePoleFilter m_diffusionR;

    void updateDelaySamples() noexcept {
        m_delaySamples = (m_delayMs / 1000.0f) * m_sampleRate;
        m_delaySamples = std::clamp(m_delaySamples,
                                    1.0f,
                                    static_cast<float>(MaxDelaySamples - 1));
    }
};

#endif // DSP_HAAS_STEREO_WIDENER_H
