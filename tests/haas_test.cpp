// tests/haas_test.cpp
#include "dsp/HaasStereoWidener.h"
#include <cmath>
#include <cstdio>
#include <cassert>
#include <chrono>

constexpr float sample_rate = 48000.0f;
constexpr float pi = 3.14159265358979323846f;

float sine_wave(float freq_hz, int sample_index) {
    return std::sin(2.0f * pi * freq_hz * static_cast<float>(sample_index) / sample_rate);
}

void test_mono_compatibility() {
    // With feedback=0.5 and mono input (L=R), the cross-delay term is identical on both
    // sides: outputL = outputR = input + k*p*delayed_input. The mono sum deviation is
    // |2*k*delayed_input| which peaks at 2*0.5*1.0 = 1.0 (the amplitude of the sine).
    // The inverted polarity mode avoids frequency-specific deep notches (comb filtering)
    // compared to normal polarity. Both modes must stay bounded.
    printf("Test: Mono compatibility (bounded output)\n");

    auto measure_max_cancellation = [](bool invert) {
        HaasStereoWidener<2048> widener;
        widener.init(sample_rate);
        widener.setDelayMs(15.0f);
        widener.setFeedback(0.5f);
        widener.setInvertPolarity(invert);
        float max_c = 0.0f;
        for (int i = 0; i < 4800; ++i) {
            float input = sine_wave(440.0f, i);
            float outL, outR;
            widener.process(input, input, outL, outR);
            float cancellation = std::abs((outL + outR) - 2.0f * input);
            max_c = std::max(max_c, cancellation);
        }
        return max_c;
    };

    float inverted_cancel = measure_max_cancellation(true);
    float normal_cancel   = measure_max_cancellation(false);

    // With feedback=0.5 max cancellation = |2*k*sin| <= 2*0.5 = 1.0; verify bounded
    assert(inverted_cancel < 1.05f);
    assert(normal_cancel   < 1.05f);

    printf("  OK Max mono cancellation (inverted): %.4f (bounded <1.05)\n", inverted_cancel);
    printf("  OK Max mono cancellation (normal):   %.4f (bounded <1.05)\n", normal_cancel);
}

void test_stereo_width() {
    // Feed true stereo input (different L and R frequencies) so delayedL != delayedR,
    // producing different outputL and outputR. L-R RMS should be significant.
    printf("Test: Stereo width (L-R decorrelation with stereo input)\n");

    HaasStereoWidener<2048> widener;
    widener.init(sample_rate);
    widener.setDelayMs(20.0f);
    widener.setFeedback(0.5f);
    widener.setInvertPolarity(true);

    float sum_diff_sq = 0.0f;
    for (int i = 0; i < 4800; ++i) {
        float inputL = sine_wave(440.0f, i);
        float inputR = sine_wave(660.0f, i);   // different frequency on R channel
        float outL, outR;
        widener.process(inputL, inputR, outL, outR);

        float diff = outL - outR;
        sum_diff_sq += diff * diff;
    }
    float rms_diff = std::sqrt(sum_diff_sq / 4800.0f);

    // L-R difference should be significant (two different tones, plus cross-delays)
    assert(rms_diff > 0.3f);

    printf("  OK L-R RMS difference: %.4f (expected >0.3 for stereo width)\n", rms_diff);
}

void test_delay_accuracy() {
    printf("Test: Delay time accuracy (15 ms setting)\n");

    HaasStereoWidener<2048> widener;
    widener.init(sample_rate);
    widener.setDelayMs(15.0f);
    widener.setFeedback(0.7f);  // max allowed by clamp; enough to see impulse in R
    widener.setInvertPolarity(false);

    // Impulse on L only; peak in R expected after ~720 samples (15 ms @ 48 kHz)
    float outL, outR;
    widener.process(1.0f, 0.0f, outL, outR);

    int peak_index = -1;
    float peak_value = 0.0f;
    for (int i = 1; i < 800; ++i) {
        widener.process(0.0f, 0.0f, outL, outR);
        if (std::abs(outR) > peak_value) {
            peak_value = std::abs(outR);
            peak_index = i;
        }
    }

    // Expected delay: 15 ms = 720 samples @ 48 kHz, allow +-5%
    assert(peak_index > 684 && peak_index < 756);
    printf("  OK Measured delay: %d samples (expected ~720 for 15 ms)\n", peak_index);
}

void test_feedback_scaling() {
    printf("Test: Feedback scaling (0.0 vs 0.5 vs 0.7)\n");

    HaasStereoWidener<2048> widener;
    widener.init(sample_rate);
    widener.setDelayMs(15.0f);
    widener.setInvertPolarity(true);

    auto measure_rms = [&](float fb) {
        widener.reset();
        widener.setFeedback(fb);
        float sum_sq = 0.0f;
        for (int i = 0; i < 4800; ++i) {
            float inputL = sine_wave(440.0f, i);
            float inputR = sine_wave(660.0f, i);
            float outL, outR;
            widener.process(inputL, inputR, outL, outR);
            sum_sq += outL * outL + outR * outR;
        }
        return std::sqrt(sum_sq / 9600.0f);
    };

    float rms_fb0 = measure_rms(0.0f);
    float rms_fb5 = measure_rms(0.5f);
    float rms_fb7 = measure_rms(0.7f);

    assert(rms_fb5 > rms_fb0);
    assert(rms_fb7 > rms_fb5);

    printf("  OK RMS at fb=0.0: %.4f\n", rms_fb0);
    printf("  OK RMS at fb=0.5: %.4f\n", rms_fb5);
    printf("  OK RMS at fb=0.7: %.4f\n", rms_fb7);
}

void test_comb_filtering_with_normal_polarity() {
    printf("Test: Comb filtering (normal polarity, mono sum)\n");

    HaasStereoWidener<2048> widener;
    widener.init(sample_rate);
    widener.setDelayMs(20.0f);  // 960 samples; comb notches at odd multiples of fs/(2*960)
    widener.setFeedback(0.5f);
    widener.setInvertPolarity(false);  // normal polarity creates comb filtering

    auto measure_freq = [&](float freq) {
        widener.reset();
        float sum_sq = 0.0f;
        for (int i = 0; i < 4800; ++i) {
            float input = sine_wave(freq, i);
            float outL, outR;
            widener.process(input, input, outL, outR);
            float mono = outL + outR;
            sum_sq += mono * mono;
        }
        return std::sqrt(sum_sq / 4800.0f);
    };

    // 20 ms = 960 samples.  Normal-polarity mono = 2*in + 2*k*delayed_in.
    // Reinforcement (peak)  when delay = n   full cycles: f = n*50 Hz → 50, 100...
    // Cancellation  (notch) when delay = n+0.5 cycles:    f = (n+0.5)*50 Hz → 75, 125...
    //   50 Hz: 960*50/48000 = 1.0 cycle   → in-phase    → peak
    //   75 Hz: 960*75/48000 = 1.5 cycles  → 180° shift  → deep notch
    float rms_50hz = measure_freq(50.0f);   // reinforcement peak
    float rms_75hz = measure_freq(75.0f);   // cancellation notch

    assert(rms_75hz < rms_50hz);

    printf("  OK RMS at 50 Hz (peak):  %.4f\n", rms_50hz);
    printf("  OK RMS at 75 Hz (notch): %.4f (lower, confirming comb filter)\n", rms_75hz);
}

void test_diffusion() {
    // Measure only the cross-feed component (outL - inputL = k*p*diffused_delayed_R).
    // At 12 kHz the lowpass cutoff (10 kHz) attenuates the delayed signal, so the
    // cross-feed RMS must be lower with diffusion than without — independent of any
    // phase-interference effects in the total output.
    printf("Test: Diffusion (cross-feed attenuation at high frequency)\n");

    auto measure_crossfeed_rms = [](bool diffusion) {
        HaasStereoWidener<2048> widener;
        widener.init(sample_rate);
        widener.setDelayMs(15.0f);
        widener.setFeedback(0.5f);
        widener.setInvertPolarity(true);
        widener.setDiffusionEnabled(diffusion);
        float sum_sq = 0.0f;
        for (int i = 0; i < 4800; ++i) {
            float inputL = sine_wave(12000.0f, i);
            float inputR = sine_wave(12000.0f, i);
            float outL, outR;
            widener.process(inputL, inputR, outL, outR);
            // Cross-feed = k * polarity * diffused_delayed_R (what diffusion acts on)
            float crossfeed = outL - inputL;
            sum_sq += crossfeed * crossfeed;
        }
        return std::sqrt(sum_sq / 4800.0f);
    };

    float rms_no_diff   = measure_crossfeed_rms(false);
    float rms_with_diff = measure_crossfeed_rms(true);

    // Diffusion (lowpass) must attenuate the cross-feed at 12 kHz > 10 kHz cutoff
    assert(rms_with_diff < rms_no_diff);

    printf("  OK Cross-feed RMS at 12 kHz (no diffusion):  %.4f\n", rms_no_diff);
    printf("  OK Cross-feed RMS at 12 kHz (with diffusion): %.4f (reduced)\n", rms_with_diff);
}

void test_stability() {
    printf("Test: Stability at max feedback\n");

    HaasStereoWidener<2048> widener;
    widener.init(sample_rate);
    widener.setDelayMs(25.0f);
    widener.setFeedback(0.7f);
    widener.setInvertPolarity(true);

    float max_output = 0.0f;
    for (int i = 0; i < 480000; ++i) {
        float input = sine_wave(440.0f, i);
        float outL, outR;
        widener.process(input, input, outL, outR);

        assert(std::isfinite(outL) && std::isfinite(outR));
        max_output = std::max(max_output, std::max(std::abs(outL), std::abs(outR)));
    }

    assert(max_output < 3.0f);
    printf("  OK Max output amplitude: %.4f (stable)\n", max_output);
}

void test_performance() {
    printf("Test: Performance measurement\n");

    HaasStereoWidener<2048> widener;
    widener.init(sample_rate);
    widener.setDelayMs(15.0f);
    widener.setFeedback(0.5f);
    widener.setDiffusionEnabled(true);

    constexpr int num_samples = 480000;

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_samples; ++i) {
        float input = sine_wave(440.0f, i);
        float outL, outR;
        widener.process(input, input, outL, outR);
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    float ns_per_sample = static_cast<float>(duration_ns) / static_cast<float>(num_samples);

    printf("  OK Time per sample: %.2f ns\n", ns_per_sample);
    printf("     (ARM estimate: ~%.0f cycles at 720 MHz)\n", ns_per_sample / 1.39f);
}

int main() {
    printf("=== HaasStereoWidener Test Suite ===\n\n");

    test_mono_compatibility();
    test_stereo_width();
    test_delay_accuracy();
    test_feedback_scaling();
    test_comb_filtering_with_normal_polarity();
    test_diffusion();
    test_stability();
    test_performance();

    printf("\n=== All tests passed ===\n");
    return 0;
}
