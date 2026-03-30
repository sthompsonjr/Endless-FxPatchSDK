#ifndef DSP_BIQUAD_FILTER_H
#define DSP_BIQUAD_FILTER_H

#include <cmath>
#include <algorithm>
#include <array>

namespace biquad_constants {
    constexpr float pi = 3.14159265358979323846f;
    constexpr float two_pi = 2.0f * pi;
    constexpr float min_frequency_hz = 10.0f;
    constexpr float max_frequency_ratio = 0.49f;
    constexpr float min_q = 0.1f;
    constexpr float max_q = 100.0f;
    constexpr float default_q = 0.707f;
    constexpr float min_db_gain = -24.0f;
    constexpr float max_db_gain = 24.0f;
    constexpr float db_to_linear_scale = 1.0f / 40.0f;
}

// Second-order IIR filter (biquad)
// Supports: lowpass, highpass, bandpass, notch, allpass, peaking, shelves
// Direct Form II Transposed for numerical stability
class BiquadFilter {
public:
    // Filter types
    enum class Type {
        Lowpass,
        Highpass,
        Bandpass,
        Notch,
        Allpass,
        Peaking,    // parametric EQ boost/cut
        LowShelf,   // low-frequency shelf
        HighShelf   // high-frequency shelf
    };

    BiquadFilter() noexcept = default;

    // Initialize with sample rate
    void init(float sampleRate) noexcept;

    // Set filter type and parameters
    // For lowpass/highpass/bandpass/notch/allpass: use setParameters(type, freq, Q)
    // For peaking/shelves: use setParameters(type, freq, Q, dBgain)
    void setParameters(Type type, float freq_hz, float Q) noexcept;
    void setParameters(Type type, float freq_hz, float Q, float dBgain) noexcept;

    // Process one sample
    float process(float input) noexcept;

    // Reset filter state
    void reset() noexcept;

    // Get current parameters
    Type getType() const noexcept { return m_type; }
    float getFrequency() const noexcept { return m_frequency; }
    float getQ() const noexcept { return m_q; }
    float getGain() const noexcept { return m_dbGain; }

    // Get coefficients (for debugging/inspection)
    std::array<float, 5> getCoefficients() const noexcept {
        return {m_b0, m_b1, m_b2, m_a1, m_a2};
    }

private:
    // Sample rate
    float m_sampleRate = 48000.0f;

    // User parameters
    Type m_type = Type::Lowpass;
    float m_frequency = 1000.0f;              // Hz
    float m_q = biquad_constants::default_q;
    float m_dbGain = 0.0f;                    // dB (for peaking/shelf only)

    // Biquad coefficients (Direct Form II Transposed)
    // Transfer function: H(z) = (b0 + b1·z⁻¹ + b2·z⁻²) / (1 + a1·z⁻¹ + a2·z⁻²)
    // Note: a1, a2 are stored with sign flip for Direct Form II
    float m_b0 = 1.0f;
    float m_b1 = 0.0f;
    float m_b2 = 0.0f;
    float m_a1 = 0.0f;
    float m_a2 = 0.0f;

    // State variables (delay line)
    float m_s1 = 0.0f;  // state 1
    float m_s2 = 0.0f;  // state 2

    // Update coefficients based on current parameters
    void updateCoefficients() noexcept;
};

inline void BiquadFilter::init(float sampleRate) noexcept {
    m_sampleRate = std::clamp(sampleRate, 8000.0f, 192000.0f);
    m_s1 = 0.0f;
    m_s2 = 0.0f;
    updateCoefficients();
}

inline void BiquadFilter::setParameters(Type type, float freq_hz, float Q) noexcept {
    m_type = type;
    m_frequency = std::clamp(freq_hz,
                              biquad_constants::min_frequency_hz,
                              biquad_constants::max_frequency_ratio * m_sampleRate);
    m_q = std::clamp(Q, biquad_constants::min_q, biquad_constants::max_q);
    updateCoefficients();
}

inline void BiquadFilter::setParameters(Type type, float freq_hz, float Q, float dBgain) noexcept {
    m_type = type;
    m_frequency = std::clamp(freq_hz,
                              biquad_constants::min_frequency_hz,
                              biquad_constants::max_frequency_ratio * m_sampleRate);
    m_q = std::clamp(Q, biquad_constants::min_q, biquad_constants::max_q);
    m_dbGain = std::clamp(dBgain, biquad_constants::min_db_gain, biquad_constants::max_db_gain);
    updateCoefficients();
}

inline float BiquadFilter::process(float input) noexcept {
    const float output = m_b0 * input + m_s1;
    m_s1 = m_b1 * input + m_a1 * output + m_s2;
    m_s2 = m_b2 * input + m_a2 * output;
    return output;
}

inline void BiquadFilter::reset() noexcept {
    m_s1 = 0.0f;
    m_s2 = 0.0f;
}

inline void BiquadFilter::updateCoefficients() noexcept {
    const float omega0 = biquad_constants::two_pi * m_frequency / m_sampleRate;
    const float cos_omega0 = std::cos(omega0);
    const float sin_omega0 = std::sin(omega0);
    const float alpha = sin_omega0 / (2.0f * m_q);

    float b0_raw, b1_raw, b2_raw, a0_raw, a1_raw, a2_raw;

    switch (m_type) {
        case Type::Lowpass:
            b0_raw = (1.0f - cos_omega0) / 2.0f;
            b1_raw = 1.0f - cos_omega0;
            b2_raw = (1.0f - cos_omega0) / 2.0f;
            a0_raw = 1.0f + alpha;
            a1_raw = -2.0f * cos_omega0;
            a2_raw = 1.0f - alpha;
            break;

        case Type::Highpass:
            b0_raw = (1.0f + cos_omega0) / 2.0f;
            b1_raw = -(1.0f + cos_omega0);
            b2_raw = (1.0f + cos_omega0) / 2.0f;
            a0_raw = 1.0f + alpha;
            a1_raw = -2.0f * cos_omega0;
            a2_raw = 1.0f - alpha;
            break;

        case Type::Bandpass:
            b0_raw = alpha;
            b1_raw = 0.0f;
            b2_raw = -alpha;
            a0_raw = 1.0f + alpha;
            a1_raw = -2.0f * cos_omega0;
            a2_raw = 1.0f - alpha;
            break;

        case Type::Notch:
            b0_raw = 1.0f;
            b1_raw = -2.0f * cos_omega0;
            b2_raw = 1.0f;
            a0_raw = 1.0f + alpha;
            a1_raw = -2.0f * cos_omega0;
            a2_raw = 1.0f - alpha;
            break;

        case Type::Allpass:
            b0_raw = 1.0f - alpha;
            b1_raw = -2.0f * cos_omega0;
            b2_raw = 1.0f + alpha;
            a0_raw = 1.0f + alpha;
            a1_raw = -2.0f * cos_omega0;
            a2_raw = 1.0f - alpha;
            break;

        case Type::Peaking: {
            const float A = std::pow(10.0f, m_dbGain * biquad_constants::db_to_linear_scale);
            b0_raw = 1.0f + alpha * A;
            b1_raw = -2.0f * cos_omega0;
            b2_raw = 1.0f - alpha * A;
            a0_raw = 1.0f + alpha / A;
            a1_raw = -2.0f * cos_omega0;
            a2_raw = 1.0f - alpha / A;
            break;
        }

        case Type::LowShelf: {
            const float A = std::pow(10.0f, m_dbGain * biquad_constants::db_to_linear_scale);
            const float beta = std::sqrt(A) / m_q;
            b0_raw = A * ((A + 1.0f) - (A - 1.0f) * cos_omega0 + beta * sin_omega0);
            b1_raw = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cos_omega0);
            b2_raw = A * ((A + 1.0f) - (A - 1.0f) * cos_omega0 - beta * sin_omega0);
            a0_raw = (A + 1.0f) + (A - 1.0f) * cos_omega0 + beta * sin_omega0;
            a1_raw = -2.0f * ((A - 1.0f) + (A + 1.0f) * cos_omega0);
            a2_raw = (A + 1.0f) + (A - 1.0f) * cos_omega0 - beta * sin_omega0;
            break;
        }

        case Type::HighShelf: {
            const float A = std::pow(10.0f, m_dbGain * biquad_constants::db_to_linear_scale);
            const float beta = std::sqrt(A) / m_q;
            b0_raw = A * ((A + 1.0f) + (A - 1.0f) * cos_omega0 + beta * sin_omega0);
            b1_raw = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cos_omega0);
            b2_raw = A * ((A + 1.0f) + (A - 1.0f) * cos_omega0 - beta * sin_omega0);
            a0_raw = (A + 1.0f) - (A - 1.0f) * cos_omega0 + beta * sin_omega0;
            a1_raw = 2.0f * ((A - 1.0f) - (A + 1.0f) * cos_omega0);
            a2_raw = (A + 1.0f) - (A - 1.0f) * cos_omega0 - beta * sin_omega0;
            break;
        }

        default:
            m_b0 = 1.0f; m_b1 = 0.0f; m_b2 = 0.0f;
            m_a1 = 0.0f; m_a2 = 0.0f;
            return;
    }

    // Normalize by a0
    const float a0_inv = 1.0f / a0_raw;
    m_b0 = b0_raw * a0_inv;
    m_b1 = b1_raw * a0_inv;
    m_b2 = b2_raw * a0_inv;
    m_a1 = -a1_raw * a0_inv;  // sign flip for Direct Form II
    m_a2 = -a2_raw * a0_inv;
}

#endif // DSP_BIQUAD_FILTER_H
