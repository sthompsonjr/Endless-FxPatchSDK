/*
 * test_TriPhaseLfo.cpp — Session 2: TriPhaseLfo DSP primitive unit tests
 * Tridimension (TSC + BOSS DC-2) — Session 2 of 6.
 *
 * Build:
 *   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra \
 *       -DENDLESS_DESKTOP_BUILD \
 *       -I. -I./sdk -I./wdf -I./dsp \
 *       tests/test_TriPhaseLfo.cpp -o test_triphase -lm && ./test_triphase
 *
 * Tests:
 *   A. SPACING          — 3 sine taps at 0, 1/3, 2/3; zero-crossing phase
 *                         estimate; pairwise offsets = 1/3 period ±0.005,
 *                         peak-to-peak offset variation < 0.001 (coherence
 *                         invariant under default drift + jitter).
 *   B. DC2 MODE         — 2 triangle taps at 0, 0.5; tri(x)+tri(x+0.5)=0,
 *                         |tap0 + tap1| < 1e-4 every sample for 5 s.
 *   C. BLEND CONTINUITY — sweep one tap's blend m 0→1 over 2 s; no
 *                         sample step exceeds 4*(2*pi*f/fs) + 0.02.
 *   D. RANGE            — all 25 shape pairs, 30 s soak each, jitter 0.001;
 *                         every tap within [-1, 1].
 *   E. MICRO-BENCH      — 1,000,000 tick() calls, 3 active taps;
 *                         cycles/sample at 528 MHz (report only).
 *
 * TriPhaseLfo findings (session 2):
 *   init(fs) defaults identical to AnalogLfo (drift 0.13 Hz @ 1.2%,
 *   secondary 0.07 Hz @ 0.6%, jitter 0.0003, LCG seed 7919).
 *   tick() advances the single master accumulator once; tap(i) reads the
 *   cached bipolar output. All taps share one accumulator, so pairwise
 *   phase differences are drift/jitter-invariant by construction.
 */

#include "dsp/TriPhaseLfo.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>

// ---------------------------------------------------------------------------
// Clock and rate constants — 528 MHz actual / 48 kHz
// (pattern from tests/test_DeluxeMemoryMan_wdf.cpp)
// ---------------------------------------------------------------------------
static constexpr float kClockHz     = 528'000'000.0f;
static constexpr float kSampleRateF = 48'000.0f;
static constexpr int   kSampleRateI = 48'000;

// ---------------------------------------------------------------------------
// PASS/FAIL tracking — pattern from tests/test_DeluxeMemoryMan_wdf.cpp
// ---------------------------------------------------------------------------
static int totalTests  = 0;
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

using Lfo3 = TriPhaseLfo<4>;
using ShapeE = Lfo3::Shape;

// ---------------------------------------------------------------------------
// Test A: SPACING — tap coherence invariant
//
// 1 Hz sine, taps at 0, 1/3, 2/3, drift and jitter at init() defaults,
// 10 s at 48 kHz. Rising zero crossings of the three taps interleave at
// exactly 1/3-cycle intervals of the master phase, so the merged crossing
// sequence has one event per 1/3 cycle. Crossing times are refined by
// linear interpolation between the bracketing samples.
//
// Offset estimator: for merged-sequence gap g[k] (a pairwise offset between
// two phase-adjacent taps), the fraction of the local period is
//     frac[k] = g[k] / (g[k-1] + g[k] + g[k+1])
// The three-gap denominator is a full period CENTERED on gap k, which
// cancels the first-order within-cycle frequency slope caused by the drift
// oscillators (a one-sided period estimate leaves an error of ~drift-slope/9
// ≈ 0.0014, which would swamp the 0.001 peak-to-peak bound; the centered
// estimate leaves only second-order curvature, ~1e-4).
//
// P1: every frac within 0.005 of 1/3 (pairwise offsets = one third period).
// P2: max(frac) - min(frac) < 0.001 (coherence invariant: offsets do not
//     wander even though the drift oscillators sweep the frequency ±1.8%).
// ---------------------------------------------------------------------------
static void runTestA_Spacing() {
    std::printf("\n--- Test A: SPACING (coherence invariant) ---\n");

    Lfo3 lfo;
    lfo.init(kSampleRateF);
    lfo.setFrequency(1.0f);
    lfo.setShapes(ShapeE::Sine, ShapeE::Sine);
    lfo.setActiveTaps(3);
    lfo.setTapPhase(0, 0.0f);
    lfo.setTapPhase(1, 1.0f / 3.0f);
    lfo.setTapPhase(2, 2.0f / 3.0f);
    // drift + jitter left at init() defaults per spec

    constexpr int kNumSamples = 10 * kSampleRateI;  // 10 s
    constexpr int kMaxEvents  = 64;                 // ~31 expected at 1 Hz
    float evtTime[kMaxEvents];                      // sample units, fractional
    int   numEvents = 0;

    float prev[3];
    lfo.tick();
    for (int t = 0; t < 3; ++t) prev[t] = lfo.tap(t);

    for (int n = 1; n < kNumSamples; ++n) {
        lfo.tick();
        for (int t = 0; t < 3; ++t) {
            const float cur = lfo.tap(t);
            if (prev[t] < 0.0f && cur >= 0.0f && numEvents < kMaxEvents) {
                // Linear interpolation for sub-sample crossing time.
                // Taps are 1/3 period (16k samples) apart, so at most one
                // tap crosses per sample and the event list stays sorted.
                const float frac = prev[t] / (prev[t] - cur);
                evtTime[numEvents++] = static_cast<float>(n - 1) + frac;
            }
            prev[t] = cur;
        }
    }

    std::printf("  INFO: %d rising crossings collected (expect ~30)\n", numEvents);

    bool  enoughEvents = numEvents >= 12;  // need >= 10 interior fractions
    float maxDev  = 0.0f;
    float minFrac = 1.0f, maxFrac = 0.0f;
    int   numFracs = 0;

    if (enoughEvents) {
        // Gaps g[k] = evtTime[k+1] - evtTime[k], k in [0, numEvents-2].
        // Interior fractions use gaps k-1, k, k+1.
        for (int k = 1; k + 2 < numEvents; ++k) {
            const float gPrev = evtTime[k]     - evtTime[k - 1];
            const float g     = evtTime[k + 1] - evtTime[k];
            const float gNext = evtTime[k + 2] - evtTime[k + 1];
            const float f     = g / (gPrev + g + gNext);
            const float dev   = fabsf(f - 1.0f / 3.0f);
            if (dev > maxDev)  maxDev  = dev;
            if (f < minFrac)   minFrac = f;
            if (f > maxFrac)   maxFrac = f;
            ++numFracs;
        }
    }
    const float peakToPeak = maxFrac - minFrac;

    std::printf("  INFO: %d offset fractions; max |offset - 1/3| = %.6f "
                "(tolerance 0.005)\n", numFracs, maxDev);
    std::printf("  INFO: offset peak-to-peak variation = %.6f "
                "(tolerance 0.001)\n", peakToPeak);

    check(enoughEvents, "spacing: enough zero crossings collected");
    check(enoughEvents && maxDev < 0.005f,
          "spacing: P1 pairwise offsets within 0.005 of 1/3 period");
    check(enoughEvents && peakToPeak < 0.001f,
          "spacing: P2 offset peak-to-peak variation < 0.001 (coherence)");
}

// ---------------------------------------------------------------------------
// Test B: DC2 MODE — triangle antiphase ("motionless") identity
//
// 2 taps at 0 and 0.5, Triangle/Triangle (blend endpoints identical, so any
// blend value yields a pure triangle; 0.3 / 0.8 chosen arbitrarily).
// Identity: tri(x) + tri(x + 0.5) = 0 for all x, so the two taps must sum
// to zero every sample — this is the DC-2's constant-average delay property.
// Drift and jitter stay at defaults: the identity is structural (single
// accumulator), not statistical.
//
// P1: |tap(0) + tap(1)| < 1e-4 for every sample over 5 s.
// ---------------------------------------------------------------------------
static void runTestB_Dc2Mode() {
    std::printf("\n--- Test B: DC2 MODE (triangle antiphase identity) ---\n");

    Lfo3 lfo;
    lfo.init(kSampleRateF);
    lfo.setFrequency(1.7f);  // arbitrary; identity holds at any rate
    lfo.setShapes(ShapeE::Triangle, ShapeE::Triangle);
    lfo.setActiveTaps(2);
    lfo.setTapPhase(0, 0.0f);
    lfo.setTapPhase(1, 0.5f);
    lfo.setTapBlend(0, 0.3f);  // "any blend": endpoints identical
    lfo.setTapBlend(1, 0.8f);

    constexpr int kNumSamples = 5 * kSampleRateI;  // 5 s
    float maxSum = 0.0f;

    for (int n = 0; n < kNumSamples; ++n) {
        lfo.tick();
        const float s = fabsf(lfo.tap(0) + lfo.tap(1));
        if (s > maxSum) maxSum = s;
    }

    std::printf("  INFO: max |tap0 + tap1| = %.3e (tolerance 1e-4)\n", maxSum);
    check(maxSum < 1e-4f, "dc2_mode: P1 |tap0 + tap1| < 1e-4 every sample");
}

// ---------------------------------------------------------------------------
// Test C: BLEND CONTINUITY
//
// Sine/Triangle endpoints (both continuous), 2 Hz, 3 active taps. Tap 0's
// blend m sweeps linearly 0 → 1 over 2 s while ticking. The bound
//     4 * (2*pi*f/fs) + 0.02
// covers the natural per-sample slope of any continuous shape at f, plus
// slack for jitter; a blend-induced discontinuity (e.g. from recomputing
// the tap against a stale phase) would exceed it.
//
// P1: no sample-to-sample step of tap 0 exceeds the bound.
// ---------------------------------------------------------------------------
static void runTestC_BlendContinuity() {
    std::printf("\n--- Test C: BLEND CONTINUITY ---\n");

    constexpr float kFreq  = 2.0f;
    const float kBound = 4.0f * (6.283185307f * kFreq / kSampleRateF) + 0.02f;

    Lfo3 lfo;
    lfo.init(kSampleRateF);
    lfo.setFrequency(kFreq);
    lfo.setShapes(ShapeE::Sine, ShapeE::Triangle);
    lfo.setActiveTaps(3);
    lfo.setTapPhase(0, 0.0f);
    lfo.setTapPhase(1, 1.0f / 3.0f);
    lfo.setTapPhase(2, 2.0f / 3.0f);
    lfo.setTapBlend(0, 0.0f);

    constexpr int kNumSamples = 2 * kSampleRateI;  // 2 s sweep

    lfo.tick();
    float prev    = lfo.tap(0);
    float maxStep = 0.0f;

    for (int n = 1; n < kNumSamples; ++n) {
        const float m = static_cast<float>(n) / static_cast<float>(kNumSamples - 1);
        lfo.setTapBlend(0, m);
        lfo.tick();
        const float cur  = lfo.tap(0);
        const float step = fabsf(cur - prev);
        if (step > maxStep) maxStep = step;
        prev = cur;
    }

    std::printf("  INFO: max step = %.6f, bound = %.6f\n", maxStep, kBound);
    check(maxStep <= kBound, "blend_continuity: P1 no step exceeds bound");
}

// ---------------------------------------------------------------------------
// Test D: RANGE — 30 s soak per shape pair, jitter 0.001
//
// All 25 (shapeA, shapeB) pairs, 3 taps at 0/1/3/2/3, blends 0.0/0.5/1.0
// (covers both endpoints and the midpoint of the convex blend).
// P1: every tap output within [-1, 1] every sample (1e-6 float-rounding
// headroom on the convex blend a + (b-a)*m).
// ---------------------------------------------------------------------------
static void runTestD_Range() {
    std::printf("\n--- Test D: RANGE (30 s soak, all shape pairs) ---\n");

    static const ShapeE kShapes[5] = {
        ShapeE::Sine, ShapeE::Triangle, ShapeE::Saw,
        ShapeE::ReverseSaw, ShapeE::Square
    };
    static const char* kNames[5] = {
        "Sine", "Triangle", "Saw", "ReverseSaw", "Square"
    };

    constexpr int   kNumSamples = 30 * kSampleRateI;  // 30 s per pair
    constexpr float kLimit      = 1.0f + 1e-6f;

    bool  allOk   = true;
    float globalMax = 0.0f;

    for (int ia = 0; ia < 5; ++ia) {
        for (int ib = 0; ib < 5; ++ib) {
            Lfo3 lfo;
            lfo.init(kSampleRateF);
            lfo.setFrequency(3.7f);
            lfo.setShapes(kShapes[ia], kShapes[ib]);
            lfo.setJitterAmount(0.001f);
            lfo.setActiveTaps(3);
            lfo.setTapPhase(0, 0.0f);
            lfo.setTapPhase(1, 1.0f / 3.0f);
            lfo.setTapPhase(2, 2.0f / 3.0f);
            lfo.setTapBlend(0, 0.0f);
            lfo.setTapBlend(1, 0.5f);
            lfo.setTapBlend(2, 1.0f);

            float pairMax = 0.0f;
            for (int n = 0; n < kNumSamples; ++n) {
                lfo.tick();
                for (int t = 0; t < 3; ++t) {
                    const float mag = fabsf(lfo.tap(t));
                    if (mag > pairMax) pairMax = mag;
                }
            }

            if (pairMax > globalMax) globalMax = pairMax;
            if (pairMax > kLimit) {
                std::printf("  FAIL detail: %s/%s max |tap| = %.7f\n",
                            kNames[ia], kNames[ib], pairMax);
                allOk = false;
            }
        }
    }

    std::printf("  INFO: 25 shape pairs x 30 s, global max |tap| = %.7f\n",
                globalMax);
    check(allOk, "range: P1 all taps within [-1, 1] for every shape pair");
}

// ---------------------------------------------------------------------------
// Test E: MICRO-BENCH — report only, no threshold
// 1,000,000 tick() calls, 3 active taps, Sine/Triangle mid-blend.
// Warmup pass prevents cold-start cache bias; volatile sink prevents
// dead-store elimination (pattern from test_DeluxeMemoryMan_wdf.cpp Test 4).
// ---------------------------------------------------------------------------
static void runTestE_MicroBench() {
    std::printf("\n--- Test E: MICRO-BENCH (report only) ---\n");

    Lfo3 lfo;
    lfo.init(kSampleRateF);
    lfo.setFrequency(1.0f);
    lfo.setShapes(ShapeE::Sine, ShapeE::Triangle);
    lfo.setActiveTaps(3);
    lfo.setTapPhase(0, 0.0f);
    lfo.setTapPhase(1, 1.0f / 3.0f);
    lfo.setTapPhase(2, 2.0f / 3.0f);
    lfo.setTapBlend(0, 0.5f);
    lfo.setTapBlend(1, 0.5f);
    lfo.setTapBlend(2, 0.5f);

    // Warmup
    for (int i = 0; i < 48'000; ++i) lfo.tick();

    constexpr int kIterations = 1'000'000;
    volatile float sink = 0.0f;
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        lfo.tick();
        sink = lfo.tap(0) + lfo.tap(1) + lfo.tap(2);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    (void)sink;

    const auto elapsed_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    const double meanCycles = static_cast<double>(elapsed_ns)
                              * static_cast<double>(kClockHz)
                              / 1'000'000'000.0
                              / static_cast<double>(kIterations);
    const double budgetPct = meanCycles / 11'000.0 * 100.0;

    std::printf("  Iterations:         %d (3 active taps)\n", kIterations);
    std::printf("  Elapsed:            %lld ns\n",
                static_cast<long long>(elapsed_ns));
    std::printf("  Mean cycles/sample: %.1f  (528 MHz assumed)\n", meanCycles);
    std::printf("  Budget consumed:    %.2f%%  (11,000 cycles @ 48 kHz)\n",
                budgetPct);
    check(true, "micro_bench: measured and reported (no threshold)");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
    std::printf("==========================================\n");
    std::printf(" TriPhaseLfo — DSP Primitive Test Suite\n");
    std::printf(" Tridimension Session 2: Tests A-E\n");
    std::printf("==========================================\n");

    // Snapshot counters around each test to derive per-test bool for summary
    // (pattern from test_DeluxeMemoryMan_wdf.cpp).
    int c0, p0;

    c0 = totalTests; p0 = passedTests;
    runTestA_Spacing();
    const bool tA = (passedTests - p0) == (totalTests - c0);

    c0 = totalTests; p0 = passedTests;
    runTestB_Dc2Mode();
    const bool tB = (passedTests - p0) == (totalTests - c0);

    c0 = totalTests; p0 = passedTests;
    runTestC_BlendContinuity();
    const bool tC = (passedTests - p0) == (totalTests - c0);

    c0 = totalTests; p0 = passedTests;
    runTestD_Range();
    const bool tD = (passedTests - p0) == (totalTests - c0);

    c0 = totalTests; p0 = passedTests;
    runTestE_MicroBench();
    const bool tE = (passedTests - p0) == (totalTests - c0);

    std::printf("\n=== TRIPHASELFO TEST HARNESS SUMMARY ===\n");
    std::printf("Test A (Spacing):          %s\n", tA ? "PASS" : "FAIL");
    std::printf("Test B (DC2 Mode):         %s\n", tB ? "PASS" : "FAIL");
    std::printf("Test C (Blend Continuity): %s\n", tC ? "PASS" : "FAIL");
    std::printf("Test D (Range):            %s\n", tD ? "PASS" : "FAIL");
    std::printf("Test E (Micro-bench):      %s\n", tE ? "PASS" : "FAIL");
    std::printf("Checks: %d/%d passed\n", passedTests, totalTests);
    const bool allPassed = tA && tB && tC && tD && tE;
    std::printf("Overall: %s\n", allPassed ? "PASS" : "FAIL");
    return allPassed ? 0 : 1;
}
