#ifndef DSP_STATE_VARIABLE_FILTER_H
#define DSP_STATE_VARIABLE_FILTER_H

#include <cmath>
#include <algorithm>
#include "Saturation.h"

namespace svf_constants {
    constexpr float pi = 3.14159265358979323846f;
    constexpr float min_cutoff_hz = 10.0f;
    constexpr float max_cutoff_ratio = 0.45f;
    constexpr float min_q = 0.5f;
    constexpr float max_q = 50.0f;
    constexpr float default_q = 0.707f;
    constexpr float default_saturation_amount = 1.0f;
    constexpr float min_saturation = 0.1f;
    constexpr float max_saturation = 4.0f;
}

// Topology II state-variable filter
// Provides simultaneous lowpass, highpass, bandpass, and notch outputs
// with voltage-controllable cutoff and resonance
class StateVariableFilter {
public:
    StateVariableFilter() noexcept = default;

    // Initialize with sample rate
    // Must be called before processing
    void init(float sampleRate) noexcept {
        m_sampleRate = std::clamp(sampleRate, 8000.0f, 192000.0f);
        reset();
        updateCoefficients();
    }

    // Set cutoff frequency in Hz
    // Valid range: [min_cutoff_hz, max_cutoff_ratio × sampleRate]
    // Clamped automatically
    void setCutoff(float cutoff_hz) noexcept {
        const float max_cutoff = svf_constants::max_cutoff_ratio * m_sampleRate;
        m_cutoff = std::clamp(cutoff_hz, svf_constants::min_cutoff_hz, max_cutoff);
        updateCoefficients();
    }

    // Set resonance (Q factor)
    // Valid range: [min_q, max_q]
    // Q = 0.707 is Butterworth (maximally flat)
    // Q > 10 produces self-oscillation at cutoff
    void setResonance(float q) noexcept {
        m_q = std::clamp(q, svf_constants::min_q, svf_constants::max_q);
        updateCoefficients();
    }

    // Enable/disable tanh saturation on resonance feedback
    // Prevents harsh clipping at extreme Q values
    // Adds ~80 cycles/sample when enabled
    void setSaturationEnabled(bool enabled) noexcept {
        m_saturationEnabled = enabled;
    }

    // Set saturation amount (alpha parameter)
    // Higher values = earlier saturation onset
    // Valid range: [min_saturation, max_saturation]
    void setSaturationAmount(float amount) noexcept {
        m_saturationAmount = std::clamp(amount,
                                        svf_constants::min_saturation,
                                        svf_constants::max_saturation);
    }

    // Process one sample, returns lowpass output
    // Use getLastHighpass(), getLastBandpass(), getLastNotch() for other outputs
    float process(float input) noexcept {
        // Apply optional saturation to state2 (resonance feedback limiting)
        const float s2 = m_saturationEnabled
            ? sat::softClip(m_state2 * m_saturationAmount) / m_saturationAmount
            : m_state2;

        // Highpass: solve feedback loop analytically (no delay-free loop)
        const float hp = (input - m_k * s2 - m_state1) * m_denom;

        // Bandpass: forward from highpass
        const float bp = m_g * hp + s2;

        // Lowpass: forward from bandpass
        const float lp = m_g * bp + m_state1;

        // Notch: LP + HP gives the correct null at f0 in Topology II.
        // Note: input - k*bp differs from lp+hp in this discretisation, so
        // we use the algebraic identity directly.
        const float notch = lp + hp;

        // State update
        m_state1 += 2.0f * m_g * bp;
        m_state2 += 2.0f * m_g * hp;

        // Cache outputs
        m_lastLowpass  = lp;
        m_lastHighpass = hp;
        m_lastBandpass = bp;
        m_lastNotch    = notch;

        return lp;
    }

    // Get last computed outputs (from most recent process() call)
    float getLastLowpass()  const noexcept { return m_lastLowpass; }
    float getLastHighpass() const noexcept { return m_lastHighpass; }
    float getLastBandpass() const noexcept { return m_lastBandpass; }
    float getLastNotch()    const noexcept { return m_lastNotch; }

    // Reset filter state (clear integrators)
    void reset() noexcept {
        m_state1      = 0.0f;
        m_state2      = 0.0f;
        m_lastLowpass  = 0.0f;
        m_lastHighpass = 0.0f;
        m_lastBandpass = 0.0f;
        m_lastNotch    = 0.0f;
    }

    // Get current parameter values
    float getCutoff()          const noexcept { return m_cutoff; }
    float getResonance()       const noexcept { return m_q; }
    bool  isSaturationEnabled() const noexcept { return m_saturationEnabled; }

private:
    // Sample rate
    float m_sampleRate = 48000.0f;

    // User parameters
    float m_cutoff    = 1000.0f;
    float m_q         = svf_constants::default_q;
    bool  m_saturationEnabled  = false;
    float m_saturationAmount   = svf_constants::default_saturation_amount;

    // Precomputed coefficients (updated when cutoff/Q changes)
    float m_g     = 0.0f;   // tan(π·f₀/fs), frequency warp term
    float m_k     = 0.0f;   // 1/Q, damping coefficient
    float m_denom = 0.0f;   // 1 / (1 + g·k + g²), reciprocal for efficiency

    // Integrator states (hot-path variables)
    float m_state1 = 0.0f;  // lowpass integrator state
    float m_state2 = 0.0f;  // bandpass integrator state

    // Last computed outputs (cached for getLastXXX() methods)
    float m_lastLowpass  = 0.0f;
    float m_lastHighpass = 0.0f;
    float m_lastBandpass = 0.0f;
    float m_lastNotch    = 0.0f;

    // Update precomputed coefficients
    // Called whenever cutoff or Q changes
    void updateCoefficients() noexcept {
        m_k = 1.0f / m_q;
        // Topology II stability condition: g < k = 1/Q.
        // When g >= k the poles leave the unit circle and the filter diverges.
        // Clamp g to 0.999*k to stay within the stable region.
        const float g_raw = std::tan(svf_constants::pi * m_cutoff / m_sampleRate);
        m_g = std::min(g_raw, m_k * 0.999f);
        const float g2 = m_g * m_g;
        m_denom = 1.0f / (1.0f + m_g * m_k + g2);
    }
};

#endif // DSP_STATE_VARIABLE_FILTER_H
