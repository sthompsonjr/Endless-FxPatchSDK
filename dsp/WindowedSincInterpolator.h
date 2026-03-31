#pragma once

// WindowedSincInterpolator — Bandlimited fractional delay interpolation
//
// Implements Shannon-Whittaker reconstruction using a 16-tap Blackman-Harris-
// windowed sinc kernel. Achieves >90 dB stopband rejection at the cost of
// ~760 cycles per read (vs ~80 for Hermite). Use only for critical delay
// paths where transparent delay modulation is required (chorus/flanger LFO
// modulation, pitch-shifter grain reads). Do NOT use for reverb diffusion
// taps or simple delay echoes where Hermite suffices.
//
// Usage:
//   WindowedSincInterpolator::init();   // call once at startup
//   float sample = WindowedSincInterpolator::interpolate(
//       buffer.data(), Size, delaySamples, writeHead);
//
// The read_index parameter must be the CircularBuffer write-head position
// (next-write slot), consistent with CircularBuffer::writeHead_.

#include <array>
#include <cmath>

namespace sinc_constants {
    constexpr int   num_taps = 16;
    constexpr int   lut_size = 1024;  // fractional resolution
    constexpr float pi       = 3.14159265358979323846f;

    // 4-term Blackman-Harris window coefficients (>92 dB stopband rejection)
    constexpr float bh_a0 = 0.35875f;
    constexpr float bh_a1 = 0.48829f;
    constexpr float bh_a2 = 0.14128f;
    constexpr float bh_a3 = 0.01168f;
}

class WindowedSincInterpolator {
public:
    // Pre-compute the windowed sinc LUT. Must be called once before any
    // interpolate() call (e.g., in your effect's init() method).
    static void init() noexcept {
        if (s_initialized) return;

        for (int frac_idx = 0; frac_idx < sinc_constants::lut_size; ++frac_idx) {
            const float frac = static_cast<float>(frac_idx)
                             / static_cast<float>(sinc_constants::lut_size);

            // Build raw windowed-sinc coefficients for this fractional position.
            // t = distance from the current sinc center to tap n.
            // Window is evaluated at fixed tap positions (periodic BH, period = num_taps)
            // so that the center tap (tap 8) has window value = 1.0.
            float norm = 0.0f;
            for (int tap = 0; tap < sinc_constants::num_taps; ++tap) {
                const float t = static_cast<float>(tap) - 8.0f + frac;
                const float sinc_val = (t == 0.0f)
                    ? 1.0f
                    : std::sin(sinc_constants::pi * t) / (sinc_constants::pi * t);
                s_sincLUT[frac_idx][tap] = sinc_val * blackmanHarris(tap);
                norm += s_sincLUT[frac_idx][tap];
            }

            // Normalise to unity DC gain for every fractional position.
            if (norm > 1e-10f) {
                const float inv_norm = 1.0f / norm;
                for (int tap = 0; tap < sinc_constants::num_taps; ++tap)
                    s_sincLUT[frac_idx][tap] *= inv_norm;
            }
        }

        s_initialized = true;
    }

    // Interpolate a fractional-delay sample from a power-of-2 circular buffer.
    //
    // buffer       - pointer to circular buffer storage
    // buffer_size  - number of elements (must be a power of 2)
    // delay_samples - fractional delay >= 8.0f (headroom for the leading taps)
    // read_index   - write-head position (next-write slot = CircularBuffer::writeHead_)
    //
    // Returns the reconstructed sample at the requested fractional delay.
    template <typename T>
    [[nodiscard]] static float interpolate(const T*  buffer,
                                           int       buffer_size,
                                           float     delay_samples,
                                           int       read_index) noexcept {
        const int   mask      = buffer_size - 1;
        const int   int_delay = static_cast<int>(delay_samples);
        const float frac      = delay_samples - static_cast<float>(int_delay);

        // Map fractional part to LUT row; clamp to guard against fp edge cases.
        int lut_idx = static_cast<int>(frac * static_cast<float>(sinc_constants::lut_size));
        if (lut_idx >= sinc_constants::lut_size) lut_idx = sinc_constants::lut_size - 1;

        const auto& coeffs = s_sincLUT[lut_idx];

        // Tap 8 reads the integer-delay sample, consistent with
        //   CircularBuffer::read(int_delay) = buffer[(writeHead_ - 1 - int_delay) & mask]
        // base = index of tap-0 = (read_index - 1 - int_delay - 8).
        const int base = read_index - 1 - int_delay - 8;

        float sum = 0.0f;
        for (int tap = 0; tap < sinc_constants::num_taps; ++tap)
            sum += static_cast<float>(buffer[(base + tap) & mask]) * coeffs[tap];

        return sum;
    }

private:
    // 4-term Blackman-Harris window, DFT-even (symmetric) form:
    //   w[n] = a0 - a1·cos(2πn/(N-1)) + a2·cos(4πn/(N-1)) - a3·cos(6πn/(N-1))
    // Using N-1 in the denominator makes the 16-point window symmetric around
    // tap 7.5 (i.e. w[k] = w[15-k]), which is necessary for the windowed-sinc
    // kernel to cancel the Nyquist component at frac = 0.5.
    [[nodiscard]] static float blackmanHarris(int n) noexcept {
        const float phase = 2.0f * sinc_constants::pi * static_cast<float>(n)
                          / static_cast<float>(sinc_constants::num_taps - 1);
        return sinc_constants::bh_a0
             - sinc_constants::bh_a1 * std::cos(phase)
             + sinc_constants::bh_a2 * std::cos(2.0f * phase)
             - sinc_constants::bh_a3 * std::cos(3.0f * phase);
    }

    inline static bool s_initialized = false;

    // LUT: s_sincLUT[frac_idx][tap] — 1024 fractional positions × 16 taps
    // Memory: 1024 × 16 × 4 bytes = 64 KB (static BSS allocation)
    inline static std::array<std::array<float, sinc_constants::num_taps>,
                             sinc_constants::lut_size> s_sincLUT{};
};
