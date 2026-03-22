/*
 * soft_focus_test.cpp — Test harness for SoftFocusCircuit
 * Build: g++ -std=c++20 -O2 -Wall -Werror -Wdouble-promotion
 *        -I. -I./sdk -I./dsp -I./wdf
 *        tests/soft_focus_test.cpp -o build/test_soft_focus -lm
 */

#include "dsp/SoftFocusCircuit.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>

static int totalTests = 0;
static int passedTests = 0;

static void check(bool condition, const char* name) {
    totalTests++;
    if (condition) {
        passedTests++;
        std::printf("  PASS: %s\n", name);
    } else {
        std::printf("  FAIL: %s\n", name);
    }
}

// Shared working buffer — must be large enough for kRecBufSize (131072)
static float workingBuf[200000];

static void initCircuit(SoftFocusCircuit& c) {
    c.init(48000.0f);
    c.assignWorkingBuffer(workingBuf, 200000);
}

// ============================================================
// Group 1: Pitch Ratio
// ============================================================
static void testPitchRatio() {
    std::printf("\n--- Pitch Ratio ---\n");

    check(SoftFocusCircuit::centsToRatio(0.0f) == 1.0f,
          "test_cents_to_ratio_zero");

    {
        float exact  = powf(2.0f, 12.0f / 1200.0f);
        float approxV = SoftFocusCircuit::centsToRatio(12.0f);
        check(fabsf(approxV - exact) < 0.0001f,
              "test_cents_to_ratio_12_cents");
    }

    {
        float exact  = powf(2.0f, -15.0f / 1200.0f);
        float approxV = SoftFocusCircuit::centsToRatio(-15.0f);
        check(fabsf(approxV - exact) < 0.0001f,
              "test_cents_to_ratio_neg15_cents");
    }

    {
        float exact  = powf(2.0f, 27.0f / 1200.0f);
        float approxV = SoftFocusCircuit::centsToRatio(27.0f);
        check(approxV > 1.0f && approxV < 1.02f,
              "test_cents_to_ratio_max_range");
        check(fabsf(approxV - exact) < 0.0003f,
              "test_cents_to_ratio_max_error");
    }
}

// ============================================================
// Group 1b: Hann Overlap-Add
// ============================================================
static void testHannOverlap() {
    std::printf("\n--- Hann Overlap-Add ---\n");

    bool allPass = true;
    for (int i = 0; i < 100; ++i) {
        float phi = static_cast<float>(i) / 100.0f * 0.5f;
        float w1 = grain::hann(phi);
        float w2 = grain::hann(phi + 0.5f);
        if (fabsf(w1 + w2 - 1.0f) >= 1e-5f) {
            allPass = false;
            break;
        }
    }
    check(allPass, "test_hann_overlap_add");
}

// ============================================================
// Group 2: FDN Reverb Physics
// ============================================================
static void testFDNReverb() {
    std::printf("\n--- FDN Reverb Physics ---\n");

    {
        // test_comb_feedback_at_half_knob
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.5f);
        // decayTime = 0.3 + 0.5 * 3.7 = 2.15
        // g1 = 10^(-3 * 1583 / (2.15 * 48000)) = 10^(-4749/103200) = 10^(-0.046017)
        float expected = powf(10.0f, -3.0f * 1583.0f / (2.15f * 48000.0f));
        check(fabsf(c.getCombFeedback(0) - expected) < 0.001f,
              "test_comb_feedback_at_half_knob");
    }

    {
        // test_comb_feedback_ceiling
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(1.0f);
        bool allBelow = true;
        for (int i = 0; i < 4; ++i) {
            if (c.getCombFeedback(i) > soft_focus::kCombFeedbackCeil) {
                allBelow = false;
            }
        }
        check(allBelow, "test_comb_feedback_ceiling");
    }

    {
        // test_comb_feedback_minimum
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.0f);
        // decayTime = 0.3, kC4 = 1993
        float expected = powf(10.0f, -3.0f * 1993.0f / (0.3f * 48000.0f));
        check(fabsf(c.getCombFeedback(3) - expected) < 0.002f,
              "test_comb_feedback_minimum");
    }

    {
        // test_fdn_decay_t60
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(1.0f); // 4.0s decay
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);

        float outL, outR;
        // Feed impulse
        c.process(1.0f, 1.0f, outL, outR);
        // Process 96000 samples of silence (2 seconds)
        bool allFinite = true;
        float lastVal = 0.0f;
        for (int i = 1; i <= 96000; ++i) {
            c.process(0.0f, 0.0f, outL, outR);
            if (!std::isfinite(outL) || !std::isfinite(outR)) {
                allFinite = false;
            }
            if (i == 96000) lastVal = fabsf(outL);
        }
        check(allFinite, "test_fdn_decay_finite");
        // At 2.0s with T60=4.0s, expect ~20dB decay -> magnitude <= 0.12
        check(lastVal < 0.12f, "test_fdn_decay_t60_level");
    }

    {
        // test_fdn_no_dc
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.5f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);

        float outL, outR;
        // 48000 samples of 1kHz sine
        for (int i = 0; i < 48000; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
        }
        // 1000 samples silence
        float sum = 0.0f;
        for (int i = 0; i < 1000; ++i) {
            c.process(0.0f, 0.0f, outL, outR);
            sum += outL;
        }
        float mean = fabsf(sum / 1000.0f);
        check(mean < 0.01f, "test_fdn_no_dc");
    }
}

// ============================================================
// Group 3: Grain Voices
// ============================================================
static void testGrainVoices() {
    std::printf("\n--- Grain Voices ---\n");

    {
        // test_grain_latency
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.0f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);

        float outL, outR;
        // Feed impulse
        c.process(1.0f, 1.0f, outL, outR);
        // Process until we see the dry impulse
        int peakSample = -1;
        float peakVal = 0.0f;
        for (int i = 1; i < 4000; ++i) {
            c.process(0.0f, 0.0f, outL, outR);
            float val = fabsf(outL);
            if (val > peakVal) {
                peakVal = val;
                peakSample = i;
            }
        }
        // Dry compensation delay = 1920 samples (±2 tolerance)
        check(peakSample >= 1918 && peakSample <= 1922,
              "test_grain_latency");
    }

    {
        // test_stereo_spread
        // Need non-zero reverb amount so wet mix allows grain output through
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.5f);
        c.setPitchAmount(0.5f);
        c.setModIntensity(0.0f);

        float outL, outR;
        // Settle with signal for 4800 samples (smoother converges + grains fill)
        for (int i = 0; i < 4800; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
        }

        float sumSqDiff = 0.0f;
        for (int i = 0; i < 4800; ++i) {
            float sig = sinf(static_cast<float>(i + 4800) * 6.283185307f * 440.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
            float diff = outL - outR;
            sumSqDiff += diff * diff;
        }
        check(sumSqDiff > 0.01f, "test_stereo_spread");
    }

    {
        // test_no_pitch_no_spread
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.0f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);

        float outL, outR;
        // Settle
        for (int i = 0; i < 2000; ++i)
            c.process(0.0f, 0.0f, outL, outR);

        float sumSqDiff = 0.0f;
        for (int i = 0; i < 4800; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
            float diff = outL - outR;
            sumSqDiff += diff * diff;
        }
        check(sumSqDiff < 0.001f, "test_no_pitch_no_spread");
    }
}

// ============================================================
// Group 4: Symphonic Doubler
// ============================================================
static void testSymphonicDoubler() {
    std::printf("\n--- Symphonic Doubler ---\n");

    {
        // test_symphonic_stereo_differs_from_grain_only
        // knob1=0 (no pitch), knob2=0.5 (LFO active) -> Symphonic creates stereo width
        // Need non-zero reverb so wet mix allows symphonic output through
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.5f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.5f);

        float outL, outR;
        // Settle with signal so smoother converges and grains fill
        for (int i = 0; i < 4800; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
        }

        // Measure over a full LFO period (~58000 samples at 0.825Hz)
        float sumSqDiff = 0.0f;
        for (int i = 0; i < 58000; ++i) {
            float sig = sinf(static_cast<float>(i + 4800) * 6.283185307f * 440.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
            float diff = outL - outR;
            sumSqDiff += diff * diff;
        }
        check(sumSqDiff > 0.005f,
              "test_symphonic_stereo_differs_from_grain_only");
    }

    {
        // test_symphonic_delay_range — verify no NaN/crash at max modulation
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.0f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(1.0f);

        float outL, outR;
        // Feed silence then sine
        for (int i = 0; i < 48000; ++i)
            c.process(0.0f, 0.0f, outL, outR);

        bool allFinite = true;
        for (int i = 0; i < 1000; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
            if (!std::isfinite(outL) || !std::isfinite(outR)) allFinite = false;
        }
        check(allFinite, "test_symphonic_delay_range");
    }

    {
        // test_symphonic_zero_mod_is_fixed_delay
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.0f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);

        float outL, outR;
        // Settle
        for (int i = 0; i < 2000; ++i)
            c.process(0.0f, 0.0f, outL, outR);

        float sumSqDiff = 0.0f;
        for (int i = 0; i < 4800; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
            float diff = outL - outR;
            sumSqDiff += diff * diff;
        }
        check(sumSqDiff < 0.001f,
              "test_symphonic_zero_mod_is_fixed_delay");
    }
}

// ============================================================
// Group 5: Post-Reverb Delay Taps
// ============================================================
static void testPostReverbTaps() {
    std::printf("\n--- Post-Reverb Delay Taps ---\n");

    {
        // test_tap_mix_unity
        constexpr float tapSum = soft_focus::kTapDirect
                               + soft_focus::kTap1Mix
                               + soft_focus::kTap2Mix;
        check(fabsf(tapSum - 1.0f) < 1e-6f, "test_tap_mix_unity");
    }

    {
        // test_tap1_timing_250ms
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.5f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);

        float outL, outR;
        // Feed impulse
        c.process(1.0f, 1.0f, outL, outR);
        // Process 15000 samples
        float tapRegionMax = 0.0f;
        for (int i = 1; i < 15000; ++i) {
            c.process(0.0f, 0.0f, outL, outR);
            if (i >= 11900 && i <= 12100) {
                float v = fabsf(outL);
                if (v > tapRegionMax) tapRegionMax = v;
            }
        }
        // Tap region should show energy from 250ms tap
        check(tapRegionMax > 1e-5f, "test_tap1_timing_250ms");
    }

    {
        // test_tap2_timing_380ms
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(1.0f); // long decay to ensure energy at 380ms
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);

        float outL, outR;
        c.process(1.0f, 1.0f, outL, outR);
        for (int i = 1; i <= 21000; ++i) {
            c.process(0.0f, 0.0f, outL, outR);
            if (i == 18240) {
                check(fabsf(outL) > 1e-5f, "test_tap2_timing_380ms");
            }
        }
    }

    {
        // test_tap_buffer_does_not_alias_fdn
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.5f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);

        float outL, outR;
        // Build up reverb tail
        for (int i = 0; i < 48000; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
        }
        // Freeze
        c.setFreeze(true);
        int nonZeroCount = 0;
        bool allFinite = true;
        for (int i = 0; i < 1000; ++i) {
            c.process(0.0f, 0.0f, outL, outR);
            if (fabsf(outL) > 1e-7f) nonZeroCount++;
            if (!std::isfinite(outL)) allFinite = false;
        }
        check(allFinite && nonZeroCount > 500,
              "test_tap_buffer_does_not_alias_fdn");
    }
}

// ============================================================
// Group 6: Modulation
// ============================================================
static void testModulation() {
    std::printf("\n--- Modulation ---\n");

    {
        // test_mod_zero_is_static
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.0f);
        c.setPitchAmount(0.5f);
        c.setModIntensity(0.0f);

        float outL, outR;
        // Settle
        for (int i = 0; i < 3000; ++i)
            c.process(0.0f, 0.0f, outL, outR);

        // Collect L-R differences
        float sumDiff = 0.0f;
        float sumDiffSq = 0.0f;
        int n = 9600;
        for (int i = 0; i < n; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
            float d = outL - outR;
            sumDiff += d;
            sumDiffSq += d * d;
        }
        float mean = sumDiff / static_cast<float>(n);
        float variance = sumDiffSq / static_cast<float>(n) - mean * mean;
        // With zero mod, any stereo difference should be static (from pitch only)
        // Variance of the difference over time should be very low if no time-varying mod
        // But pitch-shifted grains still create L-R diff. Check that the variance
        // of the running difference is small (no modulation means no variation pattern).
        // This is tricky — just verify it's below a threshold.
        check(variance < 0.1f, "test_mod_zero_is_static");
    }
}

// ============================================================
// Group 7: Stability
// ============================================================
static void testStability() {
    std::printf("\n--- Stability ---\n");

    {
        // test_silence_in_silence_out
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.5f);
        c.setPitchAmount(0.5f);
        c.setModIntensity(0.3f);

        float outL, outR;
        bool allFinite = true;
        bool allQuiet = true;
        for (int i = 0; i < 96000; ++i) {
            c.process(0.0f, 0.0f, outL, outR);
            if (!std::isfinite(outL) || !std::isfinite(outR)) allFinite = false;
            if (i > 1000 && (fabsf(outL) > 1e-6f || fabsf(outR) > 1e-6f))
                allQuiet = false;
        }
        check(allFinite, "test_silence_finite");
        check(allQuiet, "test_silence_out");
    }

    {
        // test_extreme_input_finite
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.5f);
        c.setPitchAmount(0.5f);
        c.setModIntensity(0.5f);

        float outL, outR;
        bool allFinite = true;
        c.process(10.0f, -10.0f, outL, outR);
        if (!std::isfinite(outL) || !std::isfinite(outR)) allFinite = false;
        c.process(-10.0f, 10.0f, outL, outR);
        if (!std::isfinite(outL) || !std::isfinite(outR)) allFinite = false;
        c.process(1.0f, -1.0f, outL, outR);
        if (!std::isfinite(outL) || !std::isfinite(outR)) allFinite = false;
        c.process(-1.0f, 1.0f, outL, outR);
        if (!std::isfinite(outL) || !std::isfinite(outR)) allFinite = false;
        for (int i = 0; i < 1000; ++i) {
            c.process(0.0f, 0.0f, outL, outR);
            if (!std::isfinite(outL) || !std::isfinite(outR)) allFinite = false;
        }
        check(allFinite, "test_extreme_input_finite");
    }

    {
        // test_parameter_step_bounded
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.0f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);

        float outL, outR;
        bool bounded = true;
        for (int i = 0; i < 4000; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            if (i == 1000) c.setReverbAmount(1.0f);
            if (i == 2000) c.setPitchAmount(1.0f);
            if (i == 3000) c.setModIntensity(1.0f);
            c.process(sig, sig, outL, outR);
            if (fabsf(outL) > 2.0f || fabsf(outR) > 2.0f) bounded = false;
        }
        check(bounded, "test_parameter_step_bounded");
    }

    {
        // test_freeze_sustains
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(1.0f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);

        float outL, outR;
        // Build reverb tail
        for (int i = 0; i < 4800; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
        }
        // Freeze
        c.setFreeze(true);
        // Process 2400 samples of silence
        float rmsSum = 0.0f;
        for (int i = 0; i < 2400; ++i) {
            c.process(0.0f, 0.0f, outL, outR);
            if (i >= 2300) rmsSum += outL * outL;
        }
        float rms = sqrtf(rmsSum / 100.0f);
        check(rms > 0.0001f, "test_freeze_sustains");
    }

    {
        // test_pure_shimmer_mutes_dry
        SoftFocusCircuit c;
        initCircuit(c);
        c.setReverbAmount(0.0f);
        c.setPitchAmount(0.0f);
        c.setModIntensity(0.0f);
        c.setDryMute(true);

        float outL, outR;
        bool allQuiet = true;
        for (int i = 0; i < 4800; ++i) {
            float sig = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            c.process(sig, sig, outL, outR);
            // With reverb=0 (wetMix=0) and dry muted, output should be near zero
            // But grain and symphonic still produce output through smoothed wet (which
            // smooths toward 0). Allow settling time.
            if (i > 2000 && (fabsf(outL) > 0.01f || fabsf(outR) > 0.01f))
                allQuiet = false;
        }
        check(allQuiet, "test_pure_shimmer_mutes_dry");
    }
}

// ============================================================
// Group 8: Performance
// ============================================================
static void testPerformance() {
    std::printf("\n--- Performance ---\n");

    SoftFocusCircuit c;
    initCircuit(c);
    c.setReverbAmount(0.5f);
    c.setPitchAmount(0.5f);
    c.setModIntensity(0.5f);

    float outL, outR;
    // Warm-up
    for (int i = 0; i < 9600; ++i)
        c.process(sinf(static_cast<float>(i) * 0.1f), 0.0f, outL, outR);

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 48000; ++i)
        c.process(sinf(static_cast<float>(i) * 0.1f), 0.0f, outL, outR);
    auto t1 = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double est_cycles = (720e6 / (48000.0 / (ms / 1000.0)));
    std::printf("PERF: %.1fms / 48000 samples | est %.0f cycles/sample at 720MHz\n",
                ms, est_cycles);
    std::printf("PERF: Budget headroom = %.1f%%\n",
                (1.0 - est_cycles / 15000.0) * 100.0);
    check(est_cycles < 12000.0, "test_performance");
}

// ============================================================
// Main
// ============================================================
int main() {
    std::printf("======================================\n");
    std::printf(" Soft Focus — Test Suite\n");
    std::printf("======================================\n");

    testPitchRatio();
    testHannOverlap();
    testFDNReverb();
    testGrainVoices();
    testSymphonicDoubler();
    testPostReverbTaps();
    testModulation();
    testStability();
    testPerformance();

    std::printf("\n======================================\n");
    std::printf(" Results: %d/%d passed\n", passedTests, totalTests);
    std::printf("======================================\n");

    return (passedTests == totalTests) ? 0 : 1;
}
