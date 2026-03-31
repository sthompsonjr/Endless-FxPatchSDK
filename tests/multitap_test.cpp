// tests/multitap_test.cpp — MultiTapDelay test harness
#include "dsp/MultiTapDelay.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>

constexpr float kSR = 48000.0f;
constexpr float kPi = 3.14159265358979323846f;

static int s_total  = 0;
static int s_passed = 0;

static void check(bool condition, const char* name) {
    ++s_total;
    if (condition) {
        ++s_passed;
        std::printf("  PASS: %s\n", name);
    } else {
        std::printf("  FAIL: %s\n", name);
    }
}

static bool approx(float a, float b, float eps = 0.001f) {
    return fabsf(a - b) < eps;
}

static float sine(float freq_hz, int n) {
    return sinf(2.0f * kPi * freq_hz * static_cast<float>(n) / kSR);
}

// Helper: compute expected delay in samples for BPM + note_division constant
// (mirrors noteDivisionToSamples; +1 because -1 is subtracted inside)
static float expectedDelay(float bpm, float division) {
    return 4.0f * (60.0f / bpm) * kSR * division;
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. BPM-to-samples conversion accuracy
//
//   300 BPM + sixteenth note (0.0625):
//     expected = 4 × (60/300) × 48000 × 0.0625 = 4 × 0.2 × 48000 × 0.0625 = 2400 samples
//   noteDivisionToSamples subtracts 1 (read-before-write ordering), so the
//   impulse re-appears exactly at sample 2400 in the output loop.
// ─────────────────────────────────────────────────────────────────────────────
static void test_bpm_to_samples() {
    std::printf("\n--- BPM-to-samples conversion ---\n");

    // Analytical formula checks (no DSP run needed)
    float q120  = expectedDelay(120.0f, multitap_constants::quarter_note);
    float de120 = expectedDelay(120.0f, multitap_constants::dotted_eighth);
    check(approx(q120,  24000.0f, 0.5f), "quarter note @ 120 BPM = 24000 samples");
    check(approx(de120, 18000.0f, 0.5f), "dotted eighth @ 120 BPM = 18000 samples");

    // Runtime impulse test: 300 BPM + 1/16 → 2400 samples
    // Buffer 4096 (next power-of-2 above 2400)
    constexpr int kExpected = 2400;
    MultiTapDelay<4096> delay;
    delay.init(kSR);
    delay.setBpm(300.0f);

    // Tap 0: sixteenth note, no feedback, no filter
    MultiTapDelay<4096>::TapConfig cfg;
    cfg.note_division  = multitap_constants::sixteenth_note;
    cfg.feedback       = 0.0f;
    cfg.filter_enabled = false;
    // Taps 1-3: same division, no feedback (shared buffer still contributes amplitude)
    for (int i = 0; i < multitap_constants::max_taps; ++i)
        delay.setTapConfig(i, cfg);

    // Inject impulse
    float outL, outR;
    delay.process(1.0f, 0.0f, outL, outR);

    int   peak_index = -1;
    float peak_val   = 0.0f;
    for (int i = 1; i <= kExpected + 20; ++i) {
        delay.process(0.0f, 0.0f, outL, outR);
        if (fabsf(outL) > peak_val) {
            peak_val   = fabsf(outL);
            peak_index = i;
        }
    }

    // Allow ±2 samples tolerance
    check(peak_index >= kExpected - 2 && peak_index <= kExpected + 2,
          "300 BPM + 1/16: impulse peak within ±2 samples of 2400");
    std::printf("    expected peak at sample %d, measured at %d\n", kExpected, peak_index);
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Per-tap feedback independence
//    Tap with higher feedback accumulates more energy after the impulse
//    (second and further echoes at multiples of the delay time).
// ─────────────────────────────────────────────────────────────────────────────
static void test_feedback_independence() {
    std::printf("\n--- Per-tap feedback independence ---\n");

    // 300 BPM + 1/16 = 2400 samples delay.
    // With feedback, second echo appears at ~4800.
    // Buffer 8192 (next power-of-2 above 4800).
    constexpr int kRun = 5500;

    auto measure_energy = [](float fb) -> float {
        MultiTapDelay<8192> delay;
        delay.init(kSR);
        delay.setBpm(300.0f);

        MultiTapDelay<8192>::TapConfig cfg;
        cfg.note_division  = multitap_constants::sixteenth_note;
        cfg.feedback       = fb;
        cfg.filter_enabled = false;
        cfg.ping_pong      = false;

        // Only tap 0 varies; taps 1-3 have zero feedback (but same division
        // so they still produce output and inject the impulse)
        MultiTapDelay<8192>::TapConfig silent;
        silent.note_division  = multitap_constants::sixteenth_note;
        silent.feedback       = 0.0f;
        silent.filter_enabled = false;

        delay.setTapConfig(0, cfg);
        delay.setTapConfig(1, silent);
        delay.setTapConfig(2, silent);
        delay.setTapConfig(3, silent);

        float outL, outR;
        delay.process(1.0f, 0.0f, outL, outR);  // impulse

        float energy = 0.0f;
        for (int i = 1; i < kRun; ++i) {
            delay.process(0.0f, 0.0f, outL, outR);
            energy += outL * outL;
        }
        return energy;
    };

    float e0 = measure_energy(0.0f);
    float e5 = measure_energy(0.5f);
    float e8 = measure_energy(0.8f);

    check(e5 > e0, "feedback=0.5 produces more energy than feedback=0.0");
    check(e8 > e5, "feedback=0.8 produces more energy than feedback=0.5");
    std::printf("    energy: fb=0.0=%.2f  fb=0.5=%.2f  fb=0.8=%.2f\n",
                static_cast<double>(e0), static_cast<double>(e5), static_cast<double>(e8));
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Filter cutoff per tap
//    Measure the amplitude of the first echo using an impulse.
//    500 Hz lowpass attenuates 4 kHz (the OnePoleFilter's a0 = 1-b1 << 1),
//    20 kHz lowpass passes 4 kHz (a0 ≈ 1).
// ─────────────────────────────────────────────────────────────────────────────
static void test_filter_cutoff_per_tap() {
    std::printf("\n--- Filter cutoff per tap ---\n");

    // Helper: measure peak echo amplitude after impulse through a specific cutoff
    constexpr int kDelay = 2400;  // 300 BPM + 1/16

    auto echo_peak = [](float cutoff_hz) -> float {
        MultiTapDelay<4096> delay;
        delay.init(kSR);
        delay.setBpm(300.0f);

        MultiTapDelay<4096>::TapConfig cfg;
        cfg.note_division    = multitap_constants::sixteenth_note;
        cfg.feedback         = 0.0f;
        cfg.filter_enabled   = true;
        cfg.filter_cutoff_hz = cutoff_hz;
        cfg.ping_pong        = false;
        for (int i = 0; i < multitap_constants::max_taps; ++i)
            delay.setTapConfig(i, cfg);

        float outL, outR;
        delay.process(1.0f, 0.0f, outL, outR);  // impulse

        // Advance to just past the expected echo
        float peak = 0.0f;
        for (int i = 1; i <= kDelay + 5; ++i) {
            delay.process(0.0f, 0.0f, outL, outR);
            if (fabsf(outL) > peak) peak = fabsf(outL);
        }
        return peak;
    };

    float peak_low  = echo_peak(multitap_constants::min_filter_cutoff);  // 500 Hz
    float peak_high = echo_peak(multitap_constants::max_filter_cutoff);  // 20 kHz

    check(peak_low < peak_high,
          "500 Hz cutoff attenuates echo amplitude vs 20 kHz cutoff");
    std::printf("    echo peak: 500 Hz cutoff=%.4f  20kHz cutoff=%.4f\n",
                static_cast<double>(peak_low), static_cast<double>(peak_high));
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Ping-pong routing verification
//    L-only impulse → echo appears in R output, not (or less) in L.
// ─────────────────────────────────────────────────────────────────────────────
static void test_ping_pong_routing() {
    std::printf("\n--- Ping-pong routing ---\n");

    constexpr int kDelay = 2400;
    MultiTapDelay<4096> delay;
    delay.init(kSR);
    delay.setBpm(300.0f);

    // All 4 taps ping-pong so L buffer routes entirely to R output and vice versa.
    // With only L input, all delayed L signal goes to R output and delayed R (=0) to L.
    MultiTapDelay<4096>::TapConfig pp;
    pp.note_division  = multitap_constants::sixteenth_note;
    pp.feedback       = 0.0f;
    pp.filter_enabled = false;
    pp.ping_pong      = true;

    for (int i = 0; i < multitap_constants::max_taps; ++i)
        delay.setTapConfig(i, pp);

    // Inject impulse on L only (R = 0)
    float outL, outR;
    delay.process(1.0f, 0.0f, outL, outR);

    float peak_L = 0.0f, peak_R = 0.0f;
    for (int i = 1; i <= kDelay + 20; ++i) {
        delay.process(0.0f, 0.0f, outL, outR);
        if (fabsf(outL) > peak_L) peak_L = fabsf(outL);
        if (fabsf(outR) > peak_R) peak_R = fabsf(outR);
    }

    // L impulse cross-routes to R output; L sees R (which was silent) → small
    check(peak_R > 0.5f,  "ping-pong: L impulse produces R echo");
    check(peak_R > peak_L, "ping-pong: R echo dominates over L echo for L-only input");
    std::printf("    peak_L=%.4f  peak_R=%.4f\n",
                static_cast<double>(peak_L), static_cast<double>(peak_R));
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Stability at max feedback
//    Single tap with max feedback (0.9); three remaining taps silent.
//    Total loop gain = 0.9 < 1 → stable.
//    Run 10 seconds; output must stay finite and bounded.
// ─────────────────────────────────────────────────────────────────────────────
static void test_stability() {
    std::printf("\n--- Stability at max feedback ---\n");

    // 300 BPM + quarter_note = 4×0.2×48000×0.25 = 960 samples; buffer 1024
    MultiTapDelay<1024> delay;
    delay.init(kSR);
    delay.setBpm(300.0f);

    MultiTapDelay<1024>::TapConfig active;
    active.note_division    = multitap_constants::quarter_note;
    active.feedback         = multitap_constants::max_feedback;  // 0.9
    active.filter_enabled   = true;
    active.filter_cutoff_hz = 8000.0f;
    active.ping_pong        = false;

    MultiTapDelay<1024>::TapConfig silent;
    silent.note_division    = multitap_constants::quarter_note;
    silent.feedback         = 0.0f;
    silent.filter_enabled   = false;

    delay.setTapConfig(0, active);
    delay.setTapConfig(1, silent);
    delay.setTapConfig(2, silent);
    delay.setTapConfig(3, silent);

    float max_out   = 0.0f;
    float outL, outR;
    bool  all_finite = true;
    constexpr int kSamples = static_cast<int>(kSR) * 10;  // 10 seconds

    for (int i = 0; i < kSamples; ++i) {
        float in = sine(440.0f, i);
        delay.process(in, in, outL, outR);
        if (!std::isfinite(outL) || !std::isfinite(outR)) { all_finite = false; break; }
        float m = fabsf(outL) > fabsf(outR) ? fabsf(outL) : fabsf(outR);
        if (m > max_out) max_out = m;
    }

    check(all_finite,        "stability: all output samples finite");
    check(max_out < 50.0f,   "stability: output bounded (< 50.0 amplitude)");
    std::printf("    max output amplitude: %.4f\n", static_cast<double>(max_out));
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Tempo change response
//    300 BPM + 1/16 = 2400 samples; 150 BPM + 1/16 = 4800 samples.
// ─────────────────────────────────────────────────────────────────────────────
static void test_tempo_change() {
    std::printf("\n--- Tempo change response ---\n");

    // Need buffer ≥ 4800; use 8192
    MultiTapDelay<8192> delay;
    delay.init(kSR);

    MultiTapDelay<8192>::TapConfig cfg;
    cfg.note_division  = multitap_constants::sixteenth_note;
    cfg.feedback       = 0.0f;
    cfg.filter_enabled = false;
    for (int i = 0; i < multitap_constants::max_taps; ++i)
        delay.setTapConfig(i, cfg);

    auto find_peak = [&](float bpm, int max_samples) -> int {
        delay.reset();
        delay.setBpm(bpm);
        float outL, outR;
        delay.process(1.0f, 0.0f, outL, outR);
        int peak_idx = -1; float peak_val = 0.0f;
        for (int i = 1; i <= max_samples; ++i) {
            delay.process(0.0f, 0.0f, outL, outR);
            if (fabsf(outL) > peak_val) { peak_val = fabsf(outL); peak_idx = i; }
        }
        return peak_idx;
    };

    int peak1 = find_peak(300.0f, 2500);
    int peak2 = find_peak(150.0f, 5000);

    check(peak1 >= 2398 && peak1 <= 2402, "300 BPM: echo near sample 2400");
    check(peak2 >= 4798 && peak2 <= 4802, "150 BPM: echo near sample 4800");
    check(peak2 > peak1, "slower BPM produces longer delay");
    std::printf("    peak @ 300 BPM: sample %d  @ 150 BPM: sample %d\n", peak1, peak2);
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. Performance measurement
// ─────────────────────────────────────────────────────────────────────────────
static void test_performance() {
    std::printf("\n--- Performance ---\n");

    MultiTapDelay<8192> delay;
    delay.init(kSR);
    delay.setBpm(120.0f);

    MultiTapDelay<8192>::TapConfig cfg;
    cfg.note_division    = multitap_constants::eighth_note;
    cfg.feedback         = 0.5f;
    cfg.filter_enabled   = true;
    cfg.filter_cutoff_hz = 8000.0f;
    for (int i = 0; i < multitap_constants::max_taps; ++i) {
        cfg.ping_pong = (i % 2 == 1);
        delay.setTapConfig(i, cfg);
    }

    constexpr int kSamples = 480000;
    float outL, outR;
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < kSamples; ++i) {
        float in = sine(440.0f, i);
        delay.process(in, in, outL, outR);
    }
    auto t1 = std::chrono::high_resolution_clock::now();

    float ns = static_cast<float>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    float ns_per = ns / static_cast<float>(kSamples);

    std::printf("  OK  Time per sample:     %.2f ns\n", static_cast<double>(ns_per));
    std::printf("      ARM est. @ 720 MHz:  ~%.0f cycles\n",
                static_cast<double>(ns_per / 1.39f));

    check(ns_per < 1000.0f, "performance: < 1000 ns/sample on host");
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    std::printf("=== MultiTapDelay Test Suite ===\n");

    test_bpm_to_samples();
    test_feedback_independence();
    test_filter_cutoff_per_tap();
    test_ping_pong_routing();
    test_stability();
    test_tempo_change();
    test_performance();

    std::printf("\n==============================\n");
    std::printf(" Results: %d/%d passed\n", s_passed, s_total);
    std::printf("==============================\n");

    if (s_passed != s_total) {
        std::printf("SOME TESTS FAILED\n");
        return 1;
    }
    std::printf("All tests passed\n");
    return 0;
}
