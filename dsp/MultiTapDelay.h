#pragma once

#include "CircularBuffer.h"
#include "OnePoleFilter.h"
#include <array>
#include <cmath>

namespace multitap_constants {
    constexpr int max_taps = 4;
    constexpr float min_bpm = 30.0f;
    constexpr float max_bpm = 300.0f;
    constexpr float default_bpm = 120.0f;

    // Note divisions (as fractions of whole note)
    constexpr float whole_note      = 1.0f;
    constexpr float half_note       = 0.5f;
    constexpr float quarter_note    = 0.25f;
    constexpr float eighth_note     = 0.125f;
    constexpr float sixteenth_note  = 0.0625f;
    constexpr float dotted_quarter  = 0.375f;
    constexpr float dotted_eighth   = 0.1875f;
    constexpr float triplet_quarter = 1.0f / 6.0f;
    constexpr float triplet_eighth  = 1.0f / 12.0f;

    constexpr float min_feedback     = 0.0f;
    constexpr float max_feedback     = 0.9f;  // stability margin
    constexpr float min_filter_cutoff = 500.0f;
    constexpr float max_filter_cutoff = 20000.0f;
}

// MaxDelaySamples must be a power of 2 (CircularBuffer constraint).
// Default 131072 = 2^17 ≈ 2.73 s @ 48 kHz (nearest power-of-2 above 2 s).
template <int MaxDelaySamples = 131072>
class MultiTapDelay {
    static_assert((MaxDelaySamples & (MaxDelaySamples - 1)) == 0,
                  "MultiTapDelay MaxDelaySamples must be a power of 2");
    static_assert(MaxDelaySamples > 1, "MultiTapDelay MaxDelaySamples must be > 1");

public:
    struct TapConfig {
        float note_division    = multitap_constants::quarter_note;
        float feedback         = 0.5f;
        float filter_cutoff_hz = 8000.0f;
        bool  filter_enabled   = true;
        bool  ping_pong        = false;  // L→R→L... cross-routing
    };

    MultiTapDelay() noexcept = default;

    void init(float sampleRate) noexcept;
    void setBpm(float bpm) noexcept;
    void setTapConfig(int tap_index, const TapConfig& config) noexcept;
    void process(float inputL, float inputR,
                 float& outputL, float& outputR) noexcept;
    void reset() noexcept;

    float getBpm()     const noexcept { return m_bpm; }
    int   getNumTaps() const noexcept { return multitap_constants::max_taps; }

private:
    float m_sampleRate = 48000.0f;
    float m_bpm        = multitap_constants::default_bpm;

    std::array<TapConfig, multitap_constants::max_taps> m_tapConfigs{};
    std::array<float,     multitap_constants::max_taps> m_tapDelaySamples{};

    CircularBuffer<float, MaxDelaySamples> m_bufferL;
    CircularBuffer<float, MaxDelaySamples> m_bufferR;

    std::array<OnePoleFilter, multitap_constants::max_taps> m_filtersL{};
    std::array<OnePoleFilter, multitap_constants::max_taps> m_filtersR{};

    void  updateTapDelays() noexcept;
    float noteDivisionToSamples(float division) const noexcept;
};

// ─────────────────────────────────────────────────────────────────────────────
// Implementation
// ─────────────────────────────────────────────────────────────────────────────

template <int MaxDelaySamples>
void MultiTapDelay<MaxDelaySamples>::init(float sampleRate) noexcept {
    m_sampleRate = sampleRate;
    m_bufferL.reset();
    m_bufferR.reset();

    for (int i = 0; i < multitap_constants::max_taps; ++i) {
        m_filtersL[i].init(sampleRate);
        m_filtersR[i].init(sampleRate);
        m_filtersL[i].setFrequency(m_tapConfigs[i].filter_cutoff_hz);
        m_filtersR[i].setFrequency(m_tapConfigs[i].filter_cutoff_hz);
    }

    updateTapDelays();
}

template <int MaxDelaySamples>
void MultiTapDelay<MaxDelaySamples>::setBpm(float bpm) noexcept {
    if (bpm < multitap_constants::min_bpm) bpm = multitap_constants::min_bpm;
    if (bpm > multitap_constants::max_bpm) bpm = multitap_constants::max_bpm;
    m_bpm = bpm;
    updateTapDelays();
}

template <int MaxDelaySamples>
void MultiTapDelay<MaxDelaySamples>::setTapConfig(int tap_index,
                                                   const TapConfig& config) noexcept {
    if (tap_index < 0 || tap_index >= multitap_constants::max_taps) return;

    m_tapConfigs[tap_index] = config;

    // Clamp feedback to stability range
    float& fb = m_tapConfigs[tap_index].feedback;
    if (fb < multitap_constants::min_feedback) fb = multitap_constants::min_feedback;
    if (fb > multitap_constants::max_feedback) fb = multitap_constants::max_feedback;

    // Update filter coefficients for this tap
    m_filtersL[tap_index].setFrequency(config.filter_cutoff_hz);
    m_filtersR[tap_index].setFrequency(config.filter_cutoff_hz);

    // Update delay time for this tap
    m_tapDelaySamples[tap_index] = noteDivisionToSamples(config.note_division);
}

template <int MaxDelaySamples>
void MultiTapDelay<MaxDelaySamples>::process(float inputL, float inputR,
                                              float& outputL, float& outputR) noexcept {
    float fbL = 0.0f;
    float fbR = 0.0f;
    outputL   = inputL;
    outputR   = inputR;

    for (int i = 0; i < multitap_constants::max_taps; ++i) {
        float delayedL = m_bufferL.readLinear(m_tapDelaySamples[i]);
        float delayedR = m_bufferR.readLinear(m_tapDelaySamples[i]);

        float filteredL = m_tapConfigs[i].filter_enabled
                              ? m_filtersL[i].process(delayedL)
                              : delayedL;
        float filteredR = m_tapConfigs[i].filter_enabled
                              ? m_filtersR[i].process(delayedR)
                              : delayedR;

        if (m_tapConfigs[i].ping_pong) {
            // Cross-route: L channel delay → R output, R channel delay → L output
            fbR    += m_tapConfigs[i].feedback * filteredL;
            fbL    += m_tapConfigs[i].feedback * filteredR;
            outputL += filteredR;
            outputR += filteredL;
        } else {
            fbL    += m_tapConfigs[i].feedback * filteredL;
            fbR    += m_tapConfigs[i].feedback * filteredR;
            outputL += filteredL;
            outputR += filteredR;
        }
    }

    // Write dry input plus accumulated feedback into the shared delay lines
    m_bufferL.write(inputL + fbL);
    m_bufferR.write(inputR + fbR);
}

template <int MaxDelaySamples>
void MultiTapDelay<MaxDelaySamples>::reset() noexcept {
    m_bufferL.reset();
    m_bufferR.reset();
    for (int i = 0; i < multitap_constants::max_taps; ++i) {
        m_filtersL[i].reset();
        m_filtersR[i].reset();
    }
}

template <int MaxDelaySamples>
void MultiTapDelay<MaxDelaySamples>::updateTapDelays() noexcept {
    for (int i = 0; i < multitap_constants::max_taps; ++i) {
        m_tapDelaySamples[i] = noteDivisionToSamples(m_tapConfigs[i].note_division);
    }
}

template <int MaxDelaySamples>
float MultiTapDelay<MaxDelaySamples>::noteDivisionToSamples(float division) const noexcept {
    // note_division is a fraction of a whole note (e.g. quarter_note = 0.25).
    // seconds_per_whole_note = 4 * (60 / BPM)
    // delay_samples = seconds_per_whole_note * sampleRate * note_division
    // Subtract 1 to account for the read-before-write ordering in process(),
    // so that the caller-specified division maps to an exact sample delay.
    float samples = 4.0f * (60.0f / m_bpm) * m_sampleRate * division - 1.0f;
    const float maxDelay = static_cast<float>(MaxDelaySamples - 2);
    if (samples < 0.0f)     samples = 0.0f;
    if (samples > maxDelay) samples = maxDelay;
    return samples;
}
