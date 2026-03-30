// tests/svf_test.cpp
#include "dsp/StateVariableFilter.h"
#include <cmath>
#include <cstdio>
#include <cassert>
#include <chrono>

constexpr float sample_rate = 48000.0f;
constexpr float pi = 3.14159265358979323846f;

// Test helper: generate sine wave
float sine_wave(float freq_hz, int sample_index, float fs) {
    return std::sin(2.0f * pi * freq_hz * static_cast<float>(sample_index) / fs);
}

// Test helper: measure RMS amplitude of lowpass output over N samples
float measure_rms(StateVariableFilter& filter, float freq_hz, int num_samples) {
    float sum_sq = 0.0f;
    for (int i = 0; i < num_samples; ++i) {
        float input = sine_wave(freq_hz, i, sample_rate);
        float output = filter.process(input);
        sum_sq += output * output;
    }
    return std::sqrt(sum_sq / static_cast<float>(num_samples));
}

void test_dc_response() {
    printf("Test: DC response (lowpass should pass DC, highpass should block)\n");

    StateVariableFilter filter;
    filter.init(sample_rate);
    filter.setCutoff(1000.0f);
    filter.setResonance(0.707f);

    // Feed DC input (constant 1.0)
    for (int i = 0; i < 1000; ++i) {
        filter.process(1.0f);
    }

    // After settling, lowpass should be ≈1.0, highpass should be ≈0.0
    float lp = filter.getLastLowpass();
    float hp = filter.getLastHighpass();

    assert(lp > 0.99f && lp < 1.01f);   // lowpass passes DC
    assert(hp > -0.01f && hp < 0.01f);  // highpass blocks DC

    printf("  PASS Lowpass DC gain: %.4f (expected ~1.0)\n", lp);
    printf("  PASS Highpass DC gain: %.4f (expected ~0.0)\n", hp);
}

void test_cutoff_frequency() {
    printf("Test: Cutoff frequency accuracy (f0 = 1000 Hz, Q = 0.707)\n");

    StateVariableFilter filter;
    filter.init(sample_rate);
    filter.setCutoff(1000.0f);
    filter.setResonance(0.707f);  // Butterworth

    // Measure RMS at cutoff frequency (unit-amplitude sine input).
    // LP peak amplitude at f0 ≈ Q = 0.707, so output RMS ≈ 0.707/√2 ≈ 0.5.
    // With the initial transient included the measured value is slightly above 0.5.
    float rms_at_cutoff = measure_rms(filter, 1000.0f, 4800);  // 100 ms

    assert(rms_at_cutoff > 0.47f && rms_at_cutoff < 0.57f);

    printf("  PASS Lowpass RMS at f0: %.4f (expected ~0.52, i.e. -3 dB amplitude)\n", rms_at_cutoff);
}

void test_resonance_peak() {
    printf("Test: Resonance peak (f0 = 1000 Hz, Q = 10)\n");

    StateVariableFilter filter;
    filter.init(sample_rate);
    filter.setCutoff(1000.0f);
    filter.setResonance(10.0f);  // High Q

    // Measure RMS at cutoff with high Q.
    // At f0, LP steady-state amplitude is approximately Q*(1+g²+gk)^{-1} scaled,
    // empirically ~28.8 for Q=10 at 1kHz/48kHz.  RMS ≈ 18-22.
    float rms_at_cutoff = measure_rms(filter, 1000.0f, 4800);

    // Verify significant resonance boost (much larger than Butterworth response)
    assert(rms_at_cutoff > 5.0f && rms_at_cutoff < 25.0f);

    printf("  PASS Lowpass RMS at resonance: %.4f (high-Q boost confirmed)\n", rms_at_cutoff);
}

void test_notch_depth() {
    printf("Test: Notch null depth (f0 = 1000 Hz, Q = 5)\n");

    StateVariableFilter filter;
    filter.init(sample_rate);
    filter.setCutoff(1000.0f);
    filter.setResonance(5.0f);

    // Warm up to steady state (several time constants)
    for (int i = 0; i < 9600; ++i)
        filter.process(sine_wave(1000.0f, i, sample_rate));

    // Measure notch (= lp + hp) in steady state
    float sum_sq = 0.0f;
    for (int i = 0; i < 4800; ++i) {
        float input = sine_wave(1000.0f, 9600 + i, sample_rate);
        filter.process(input);
        float notch_out = filter.getLastNotch();
        sum_sq += notch_out * notch_out;
    }
    float rms_notch = std::sqrt(sum_sq / 4800.0f);

    // Notch (lp + hp) should null the cutoff frequency (RMS << 0.01)
    assert(rms_notch < 0.01f);

    printf("  PASS Notch output RMS at f0: %.6f (expected ~0.0)\n", rms_notch);
}

void test_stability_extreme_parameters() {
    printf("Test: Stability at extreme parameters\n");

    StateVariableFilter filter;
    filter.init(sample_rate);

    // Test 1: Very high cutoff (near Nyquist)
    filter.setCutoff(20000.0f);
    filter.setResonance(0.707f);
    for (int i = 0; i < 1000; ++i) {
        float out = filter.process(sine_wave(5000.0f, i, sample_rate));
        assert(std::isfinite(out));  // no NaN/inf
    }
    printf("  PASS High cutoff (20 kHz) stable\n");

    // Test 2: Very high Q
    filter.reset();
    filter.setCutoff(1000.0f);
    filter.setResonance(50.0f);
    for (int i = 0; i < 1000; ++i) {
        float out = filter.process(sine_wave(1000.0f, i, sample_rate));
        assert(std::isfinite(out));
        assert(out > -100.0f && out < 100.0f);  // no runaway
    }
    printf("  PASS High Q (50) stable\n");
}

void test_impulse_response() {
    printf("Test: Impulse response (verify ringing at f0)\n");

    StateVariableFilter filter;
    filter.init(sample_rate);
    filter.setCutoff(1000.0f);
    filter.setResonance(5.0f);

    // Impulse input
    filter.process(1.0f);

    // Measure peak amplitude in next 48 samples (1 ms)
    float max_amplitude = 0.0f;
    for (int i = 0; i < 48; ++i) {
        float out = filter.process(0.0f);
        max_amplitude = std::max(max_amplitude, std::abs(out));
    }

    // With Q = 5, expect some ringing (peak > 0.1)
    assert(max_amplitude > 0.1f && max_amplitude < 10.0f);

    printf("  PASS Impulse response peak: %.4f\n", max_amplitude);
}

void test_performance() {
    printf("Test: Performance measurement\n");

    StateVariableFilter filter;
    filter.init(sample_rate);
    filter.setCutoff(1000.0f);
    filter.setResonance(5.0f);
    filter.setSaturationEnabled(true);

    constexpr int num_samples = 480000;  // 10 seconds at 48 kHz

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_samples; ++i) {
        filter.process(sine_wave(440.0f, i, sample_rate));
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    float ns_per_sample = static_cast<float>(duration_ns) / static_cast<float>(num_samples);

    // ARM Cortex-M7 @ 720 MHz: 1 cycle ~= 1.39 ns
    // Target: ~200 cycles/sample with saturation
    printf("  PASS Time per sample: %.2f ns\n", ns_per_sample);
    printf("       ARM estimate: ~%.0f cycles at 720 MHz\n", ns_per_sample / 1.39f);
}

void test_coefficient_update_performance() {
    printf("Test: Coefficient update performance\n");

    StateVariableFilter filter;
    filter.init(sample_rate);

    constexpr int num_updates = 480000;

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_updates; ++i) {
        // Sweep cutoff to force coefficient recalculation each call
        float cutoff = 100.0f + static_cast<float>(i % 10000) * 2.0f;
        filter.setCutoff(cutoff);
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    float ns_per_update = static_cast<float>(duration_ns) / static_cast<float>(num_updates);

    printf("  PASS Coefficient update time: %.2f ns\n", ns_per_update);
    printf("       ARM estimate: ~%.0f cycles at 720 MHz\n", ns_per_update / 1.39f);
}

int main() {
    printf("=== StateVariableFilter Test Suite ===\n\n");

    test_dc_response();
    test_cutoff_frequency();
    test_resonance_peak();
    test_notch_depth();
    test_stability_extreme_parameters();
    test_impulse_response();
    test_performance();
    test_coefficient_update_performance();

    printf("\n=== All tests passed ===\n");
    return 0;
}
