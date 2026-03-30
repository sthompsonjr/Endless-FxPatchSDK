// tests/biquad_test.cpp
#include "dsp/BiquadFilter.h"
#include <cmath>
#include <cstdio>
#include <cassert>
#include <chrono>

constexpr float sample_rate = 48000.0f;
constexpr float pi = 3.14159265358979323846f;

float sine_wave(float freq_hz, int sample_index, float fs) {
    return std::sin(2.0f * pi * freq_hz * static_cast<float>(sample_index) / fs);
}

float measure_rms(BiquadFilter& filter, float freq_hz, int num_samples) {
    float sum_sq = 0.0f;
    for (int i = 0; i < num_samples; ++i) {
        float input = sine_wave(freq_hz, i, sample_rate);
        float output = filter.process(input);
        sum_sq += output * output;
    }
    return std::sqrt(sum_sq / static_cast<float>(num_samples));
}

void test_dc_response() {
    printf("Test: DC response (lowpass passes DC, highpass blocks DC)\n");

    BiquadFilter lpf, hpf;
    lpf.init(sample_rate);
    hpf.init(sample_rate);

    lpf.setParameters(BiquadFilter::Type::Lowpass, 1000.0f, 0.707f);
    hpf.setParameters(BiquadFilter::Type::Highpass, 1000.0f, 0.707f);

    // Feed DC (constant 1.0)
    for (int i = 0; i < 1000; ++i) {
        lpf.process(1.0f);
        hpf.process(1.0f);
    }

    float lpf_dc = lpf.process(1.0f);
    float hpf_dc = hpf.process(1.0f);

    assert(lpf_dc > 0.99f && lpf_dc < 1.01f);   // lowpass passes DC
    assert(hpf_dc > -0.01f && hpf_dc < 0.01f);  // highpass blocks DC

    printf("  PASS Lowpass DC gain: %.4f (expected ~1.0)\n", lpf_dc);
    printf("  PASS Highpass DC gain: %.4f (expected ~0.0)\n", hpf_dc);
}

void test_nyquist_response() {
    printf("Test: Nyquist response (highpass passes Nyquist, lowpass blocks)\n");

    BiquadFilter lpf, hpf;
    lpf.init(sample_rate);
    hpf.init(sample_rate);

    lpf.setParameters(BiquadFilter::Type::Lowpass, 1000.0f, 0.707f);
    hpf.setParameters(BiquadFilter::Type::Highpass, 1000.0f, 0.707f);

    // Nyquist = fs/2 = 24 kHz
    // Alternate +1, -1 (Nyquist sine wave)
    float lpf_sum = 0.0f, hpf_sum = 0.0f;
    for (int i = 0; i < 100; ++i) {
        float nyquist_sample = (i % 2 == 0) ? 1.0f : -1.0f;
        lpf_sum += std::abs(lpf.process(nyquist_sample));
        hpf_sum += std::abs(hpf.process(nyquist_sample));
    }
    lpf_sum /= 100.0f;
    hpf_sum /= 100.0f;

    assert(lpf_sum < 0.1f);    // lowpass blocks Nyquist
    assert(hpf_sum > 0.5f);    // highpass passes Nyquist

    printf("  PASS Lowpass Nyquist gain: %.4f (expected ~0.0)\n", lpf_sum);
    printf("  PASS Highpass Nyquist gain: %.4f (expected >0.5)\n", hpf_sum);
}

void test_cutoff_frequency() {
    printf("Test: Cutoff frequency accuracy (f0 = 1000 Hz, Q = 0.707)\n");

    BiquadFilter filter;
    filter.init(sample_rate);
    filter.setParameters(BiquadFilter::Type::Lowpass, 1000.0f, 0.707f);

    // Measure RMS at cutoff frequency (unit-amplitude sine input).
    // Butterworth: amplitude gain at f0 = -3 dB = 0.707, so output RMS = 0.707/sqrt(2) ~= 0.5.
    float rms = measure_rms(filter, 1000.0f, 4800);

    assert(rms > 0.45f && rms < 0.57f);

    printf("  PASS Lowpass RMS at f0: %.4f (expected ~0.5, i.e. -3 dB amplitude)\n", rms);
}

void test_notch_depth() {
    printf("Test: Notch filter null depth (f0 = 1000 Hz, Q = 10)\n");

    BiquadFilter filter;
    filter.init(sample_rate);
    filter.setParameters(BiquadFilter::Type::Notch, 1000.0f, 10.0f);

    // Warm up to steady state — Q=10 notch has long settling time (~300 samples)
    constexpr int warmup = 9600;
    for (int i = 0; i < warmup; ++i)
        filter.process(sine_wave(1000.0f, i, sample_rate));

    // Measure RMS at notch frequency in steady state
    float sum_sq = 0.0f;
    constexpr int measure_n = 4800;
    for (int i = 0; i < measure_n; ++i) {
        float out = filter.process(sine_wave(1000.0f, warmup + i, sample_rate));
        sum_sq += out * out;
    }
    float rms = std::sqrt(sum_sq / static_cast<float>(measure_n));

    // High-Q notch should have deep null in steady state (< 0.01)
    assert(rms < 0.01f);

    printf("  PASS Notch depth at f0: %.6f (expected ~0.0)\n", rms);
}

void test_peaking_eq_gain() {
    printf("Test: Peaking EQ gain (f0 = 1000 Hz, Q = 2, +12 dB)\n");

    BiquadFilter filter;
    filter.init(sample_rate);
    filter.setParameters(BiquadFilter::Type::Peaking, 1000.0f, 2.0f, 12.0f);

    // Measure RMS at peak frequency.
    // +12 dB amplitude gain = 10^(12/20) ~= 3.98, output RMS ~= 3.98/sqrt(2) ~= 2.81.
    float rms = measure_rms(filter, 1000.0f, 4800);

    assert(rms > 2.4f && rms < 3.2f);

    printf("  PASS Peaking RMS at f0: %.4f (expected ~2.81 for +12 dB)\n", rms);
}

void test_lowshelf_response() {
    printf("Test: Low-shelf filter (f0 = 200 Hz, Q = 0.707, +6 dB)\n");

    BiquadFilter filter;
    filter.init(sample_rate);
    filter.setParameters(BiquadFilter::Type::LowShelf, 200.0f, 0.707f, 6.0f);

    // Measure gain at DC (should be ~+6 dB = 2.0x amplitude)
    for (int i = 0; i < 1000; ++i) filter.process(1.0f);
    float dc_gain = filter.process(1.0f);

    // Measure RMS at high frequency (0 dB shelf region, amplitude ~1.0, RMS ~0.707)
    filter.reset();
    float hf_rms = measure_rms(filter, 10000.0f, 4800);

    assert(dc_gain > 1.8f && dc_gain < 2.2f);   // +6 dB ~= 2.0 amplitude
    assert(hf_rms > 0.60f && hf_rms < 0.80f);   // 0 dB: amplitude ~1.0, RMS ~0.707

    printf("  PASS Low-shelf DC gain: %.4f (expected ~2.0 for +6 dB)\n", dc_gain);
    printf("  PASS Low-shelf HF RMS: %.4f (expected ~0.707)\n", hf_rms);
}

void test_stability() {
    printf("Test: Stability at extreme parameters\n");

    BiquadFilter filter;
    filter.init(sample_rate);

    // Test 1: Very high Q
    filter.setParameters(BiquadFilter::Type::Bandpass, 1000.0f, 100.0f);
    for (int i = 0; i < 1000; ++i) {
        float out = filter.process(sine_wave(1000.0f, i, sample_rate));
        assert(std::isfinite(out));
    }
    printf("  PASS High Q (100) stable\n");

    // Test 2: Near-Nyquist frequency
    filter.reset();
    filter.setParameters(BiquadFilter::Type::Lowpass, 20000.0f, 0.707f);
    for (int i = 0; i < 1000; ++i) {
        float out = filter.process(sine_wave(5000.0f, i, sample_rate));
        assert(std::isfinite(out));
    }
    printf("  PASS High cutoff (20 kHz) stable\n");
}

void test_performance() {
    printf("Test: Performance measurement\n");

    BiquadFilter filter;
    filter.init(sample_rate);
    filter.setParameters(BiquadFilter::Type::Peaking, 1000.0f, 5.0f, 6.0f);

    constexpr int num_samples = 480000;

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_samples; ++i) {
        filter.process(sine_wave(440.0f, i, sample_rate));
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    float ns_per_sample = static_cast<float>(duration_ns) / static_cast<float>(num_samples);

    printf("  PASS Time per sample: %.2f ns\n", ns_per_sample);
    printf("       ARM estimate: ~%.0f cycles at 720 MHz\n", ns_per_sample / 1.39f);
}

int main() {
    printf("=== BiquadFilter Test Suite ===\n\n");

    test_dc_response();
    test_nyquist_response();
    test_cutoff_frequency();
    test_notch_depth();
    test_peaking_eq_gain();
    test_lowshelf_response();
    test_stability();
    test_performance();

    printf("\n=== All tests passed ===\n");
    return 0;
}
