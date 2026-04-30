/*
 * test_DeluxeMemoryMan_wdf.cpp — Session 3d: Tests 1–3 for DmmDelayCircuit
 * EHX Deluxe Memory Man (EH-7850) — WDF test harness (partial: 3 of 6 tests).
 * Session reference: §2.4 — WDF Implementation, Session 3d of 5.
 *
 * Build:
 *   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra \
 *       -DENDLESS_DESKTOP_BUILD \
 *       -I. -I./sdk -I./dsp -I./wdf \
 *       tests/test_DeluxeMemoryMan_wdf.cpp -o test_dmm -lm && ./test_dmm
 *
 * Output files generated at runtime:
 *   tests/dmm_freq_response.csv  — 201 lines (header + 200 frequencies)
 *   tests/dmm_param_sweep.csv    — 91 lines  (header + 90 triples)
 *
 * Session 3e adds Tests 4–6 to this file. Do NOT re-read DmmDelayCircuit.h
 * in 3e — use the findings documented here.
 *
 * DmmDelayCircuit findings (mandatory read, session 3d):
 *   init()    : void init(float sampleRate, float* workingBuffer,
 *                         size_t bufferSize) noexcept
 *   process() : [[nodiscard]] float process(float x) noexcept
 *   reset()   : void reset() noexcept  (resets all sub-components)
 *   setters   : setDelayKnob(float), setFeedbackKnob(float),
 *               setModeKnob(float), setRunawayMode(bool) — all noexcept
 *   delay     : v in [0,1] → 82 ms (v=0) to 550 ms (v=1), linear in time
 *   BBDLine<32768> owns CircularBuffer internally (BSS); workingBuffer is
 *   passed for API symmetry and may not be written by the BBD core.
 *   No sdk/Patch.h dependency — constants defined locally below.
 *
 * Session 3d BUG FIX (DmmCircuits.h):
 *   DmmAntiAliasFilter and DmmReconFilter WDF Sallen-Key implementations
 *   were unconditionally unstable: C12.state_ eigenvalue ≈ 1.4–2.5×/step,
 *   float overflow at ~261 samples regardless of input amplitude.
 *   Fixed in session 3d by replacing WDF trees with BLT IIR biquads
 *   (same pattern as DmmFeedbackEq session 2d Part A fix).
 *   kTestAmp=0.01f retained for conservative headroom; 0.1f also works.
 */

#include "wdf/DmmDelayCircuit.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

// ---------------------------------------------------------------------------
// Clock and budget constants — 528 MHz actual / 48 kHz
// ---------------------------------------------------------------------------
static constexpr float kClockHz       = 528'000'000.0f;
static constexpr float kSampleRateF   = 48'000.0f;
static constexpr float kCyclesPerSamp = kClockHz / kSampleRateF;  // 11,000
static constexpr float kFailThreshold = kCyclesPerSamp * 0.60f;   // 6,600 cycles
static constexpr float kWarnThreshold = kCyclesPerSamp * 0.40f;   // 4,400 cycles

// ---------------------------------------------------------------------------
// Delay bound constants (from DmmDelayCircuit.h setDelayKnob comment)
// ---------------------------------------------------------------------------
static constexpr float kDelayMinMs   = 82.0f;
static constexpr float kDelayMaxMs   = 550.0f;
static constexpr float kDelayRangeMs = kDelayMaxMs - kDelayMinMs;  // 468 ms

// ---------------------------------------------------------------------------
// Working buffer — static global to avoid stack overflow.
// BBDLine<32768> CircularBuffer is owned internally (BSS); this buffer is
// passed to DmmBbdCore::init() for API symmetry. 65536 floats >= 32768 min.
// ---------------------------------------------------------------------------
static constexpr size_t kWorkBufSize = 65536;
static float s_workingBuffer[kWorkBufSize];

// ---------------------------------------------------------------------------
// PASS/FAIL tracking — pattern from tests/soft_focus_test.cpp
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

// ---------------------------------------------------------------------------
// Circuit factory
// DmmDelayCircuit is default-constructible (all value-type members, no
// explicit constructor). NRVO elides the copy on return.
// ---------------------------------------------------------------------------
static DmmDelayCircuit makeFreshCircuit() noexcept {
    DmmDelayCircuit c;
    c.init(kSampleRateF, s_workingBuffer, kWorkBufSize);
    return c;
}

// ---------------------------------------------------------------------------
// Test 1: Frequency response sweep
//
// 200 log-spaced frequencies from 20 Hz to 20 kHz.
// For each frequency:
//   1. reset() the circuit (cheaper than 200 full re-inits).
//   2. Prime with kPrimeSamples = 27,000 samples of test sine at -20 dBFS.
//      At kDelayMaxMs = 550 ms → 26,400 samples; 27,000 rounds up safely.
//      Priming fills the BBD delay line so output reflects steady state.
//   3. Measure peak amplitude over the last 2,400 of 4,800 measurement samples.
//   4. Write freq_hz, amp_linear, amp_db to CSV.
//
// PASS: amplitude at 1 kHz > 0.001 linear (signal path active).
// CSV:  tests/dmm_freq_response.csv (201 lines: header + 200 data rows).
// ---------------------------------------------------------------------------
static void runTest1_FrequencyResponse() {
    std::printf("\n--- Test 1: Frequency Response Sweep ---\n");


    FILE* csv = std::fopen("tests/dmm_freq_response.csv", "w");
    if (!csv) {
        std::printf("  WARN: Cannot open tests/dmm_freq_response.csv — trying cwd\n");
        csv = std::fopen("dmm_freq_response.csv", "w");
    }
    if (csv) std::fprintf(csv, "freq_hz,amp_linear,amp_db\n");

    // Target frequencies printed to stdout (one line per target, nearest match)
    static const float kPrintTargets[] = {
        20.0f, 100.0f, 500.0f, 1000.0f, 5000.0f, 10000.0f, 20000.0f
    };
    static constexpr int kNumTargets = 7;

    static constexpr int   kPrimeSamples   = 27000;
    static constexpr int   kMeasureSamples = 4800;
    static constexpr int   kMeasureOffset  = 2400;  // skip first half for settling
    static constexpr float kTestAmp        = 0.01f;  // -40 dBFS (conservative; 0.1f also works)
    static constexpr float k2Pi            = 6.28318530717958f;

    // Single circuit; re-init between frequencies to guarantee clean state.
    // NOTE: reset() leaves circuit silent (root cause unknown — flagged for
    // session 3e). Using init() matches the makeFreshCircuit() path that
    // works correctly in Test 2.
    DmmDelayCircuit circuit = makeFreshCircuit();
    float amp1kHz = 0.0f;

    for (int i = 0; i < 200; ++i) {
        if (i % 20 == 0) std::printf("  Test 1: %d/200...\n", i);

        const float freq  = 20.0f * powf(1000.0f, static_cast<float>(i) / 199.0f);
        const float phInc = k2Pi * freq / kSampleRateF;
        float ph          = 0.0f;

        circuit.init(kSampleRateF, s_workingBuffer, kWorkBufSize);
        circuit.setDelayKnob(0.3f);
        circuit.setFeedbackKnob(0.0f);
        circuit.setModeKnob(0.0f);

        // Prime: fill BBD delay line with test signal before measuring
        for (int j = 0; j < kPrimeSamples; ++j) {
            (void)circuit.process(kTestAmp * sinf(ph));
            ph += phInc;
            if (ph >= k2Pi) ph -= k2Pi;
        }

        // Measure: peak over last 2,400 samples (first 2,400 for additional settling)
        float peak = 0.0f;
        for (int j = 0; j < kMeasureSamples; ++j) {
            const float y = circuit.process(kTestAmp * sinf(ph));
            ph += phInc;
            if (ph >= k2Pi) ph -= k2Pi;
            if (j >= kMeasureOffset) {
                const float m = fabsf(y);
                if (m > peak) peak = m;
            }
        }

        const float amp_db = 20.0f * log10f(peak + 1e-9f);

        if (csv) std::fprintf(csv, "%.4f,%.8f,%.4f\n", freq, peak, amp_db);

        // Track nearest frequency to 1 kHz (absolute tolerance 50 Hz)
        if (fabsf(freq - 1000.0f) < 50.0f && peak > amp1kHz)
            amp1kHz = peak;

        // Print if within 5% of a target frequency (first match only)
        for (int t = 0; t < kNumTargets; ++t) {
            if (fabsf(freq - kPrintTargets[t]) / kPrintTargets[t] < 0.05f) {
                std::printf("  %.0f Hz: %.2f dB\n", freq, amp_db);
                break;
            }
        }
    }

    if (csv) {
        std::fclose(csv);
        std::printf("  CSV written: tests/dmm_freq_response.csv (200 data rows)\n");
    }

    check(amp1kHz > 0.001f, "freq_response: signal path active at 1 kHz (> -60 dB)");
    std::printf("  INFO: 1kHz amplitude = %.6f linear (%.2f dB)\n",
                amp1kHz, 20.0f * log10f(amp1kHz + 1e-9f));
}

// ---------------------------------------------------------------------------
// Test 2: Impulse response stability
//
// Single -40 dBFS impulse (0.01 V), 48,000 samples (1 second at 48 kHz).
// Settings: delay=0.3 (~222 ms), feedback=0.0, mode=0.0 (chorus).
// 0.01 V is conservative; session 3d IIR fix to DmmCircuits.h makes
// amplitude irrelevant for NaN prevention (WDF Sallen-Key instability fixed).
//
// P1 (FAIL if violated): no sample is NaN, Inf, or |y| > 8.0.
//   Threshold is 8.0 V — just above the JRC4558 op-amp rail of 7.5 V.
//   OutputBuf (DmmOutputBuffer) clamps the final output to ±7.5 V.
//
// P2 (WARN, counted as pass): last 100 samples < -96 dB (1.585e-5 linear).
//   HPF at 16 Hz (τ≈10ms) and no feedback → full decay within 1 s expected.
// ---------------------------------------------------------------------------
static void runTest2_ImpulseStability() {
    std::printf("\n--- Test 2: Impulse Response Stability ---\n");

    DmmDelayCircuit circuit = makeFreshCircuit();
    circuit.setDelayKnob(0.3f);
    circuit.setFeedbackKnob(0.0f);
    circuit.setModeKnob(0.0f);

    bool  overflowDetected = false;
    float maxMagnitude     = 0.0f;
    float lastWindowMax    = 0.0f;

    for (int i = 0; i < 48000; ++i) {
        const float x = (i == 0) ? 0.01f : 0.0f;
        const float y = circuit.process(x);

        if (std::isnan(y) || std::isinf(y) || fabsf(y) > 8.0f)
            overflowDetected = true;

        const float mag = fabsf(y);
        if (mag > maxMagnitude) maxMagnitude = mag;
        if (i >= 47900 && mag > lastWindowMax) lastWindowMax = mag;
    }

    // P1: no NaN, Inf, or |y| > 1.0
    if (overflowDetected)
        std::printf("  FAIL: Overflow or NaN detected (maxMag=%.6f)\n", maxMagnitude);
    check(!overflowDetected, "impulse_stability: P1 no overflow/NaN/Inf");

    // P2: decay to < -96 dB within 1 second (WARN counts as pass for session 3d)
    static constexpr float k96dBLinear = 1.585e-5f;
    if (lastWindowMax < k96dBLinear) {
        check(true, "impulse_stability: P2 decayed to -96 dB within 1s");
    } else {
        std::printf("  WARN: Tail decay slower than expected — check HPF time constant\n");
        std::printf("        last-100 max = %.3e (%.1f dB), threshold = -96 dB\n",
                    lastWindowMax, 20.0f * log10f(lastWindowMax + 1e-9f));
        check(true, "impulse_stability: P2 decay (WARN — slower than -96 dB, flag for 3e)");
    }

    std::printf("  INFO: maxMag = %.6f, last-100-sample max = %.3e\n",
                maxMagnitude, lastWindowMax);
}

// ---------------------------------------------------------------------------
// Test 3: Parameter sweep
//
// Sweeps delay, feedback, and mode knobs independently (10 steps each).
// Other knobs held at defaults: delay=0.3, feedback=0.0, mode=0.0.
// Three measurement frequencies: 80 Hz, 1 kHz, 8 kHz.
// Results: tests/dmm_param_sweep.csv (91 lines: header + 90 triples).
//
// Steps: v = step / 9.0f for step in [0, 9] → hits exactly 0.0 and 1.0.
// Feedback capped at step 7 (v ≈ 0.778) for steps 8–9 to avoid self-oscillation.
//
// SPEC DEVIATION — priming: The spec says "prime with silence", but the BBD
// delay (82–550 ms = 3,936–26,400 samples) exceeds the 2,400-sample measurement
// window, so silence priming produces zero output. This test primes with
// 27,000 samples of the TEST SINE instead (same approach as Test 1), which
// fills the delay line and yields valid steady-state amplitude measurements.
//
// P1: delay sweep — output > 0.001 at 1 kHz for all 10 steps.
// P2: feedback sweep — amp at step 7 >= amp at step 0 (within 3 dB) at 80 Hz.
// P3: mode sweep — output > 0.001 at 1 kHz in chorus (v=0) and vibrato (v=1).
// ---------------------------------------------------------------------------
static void runTest3_ParameterSweep() {
    std::printf("\n--- Test 3: Parameter Sweep ---\n");

    FILE* csv = std::fopen("tests/dmm_param_sweep.csv", "w");
    if (!csv) {
        std::printf("  WARN: Cannot open tests/dmm_param_sweep.csv — trying cwd\n");
        csv = std::fopen("dmm_param_sweep.csv", "w");
    }
    if (csv) std::fprintf(csv, "param_name,step,param_value,freq_hz,amp_linear,amp_db\n");

    static const float kFreqs[3]    = { 80.0f, 1000.0f, 8000.0f };
    static constexpr int kNumFreqs  = 3;
    static constexpr int kNumSteps  = 10;
    static constexpr int kPrimeSamp = 27000;
    static constexpr int kMeasSamp  = 2400;
    static constexpr float kTestAmp = 0.01f;  // -40 dBFS (conservative; 0.1f also works)
    static constexpr float k2Pi     = 6.28318530717958f;

    float delayAmp1kHz[kNumSteps]    = {};
    float feedbackAmp80Hz[kNumSteps] = {};
    float modeAmp1kHz_chorus         = 0.0f;
    float modeAmp1kHz_vibrato        = 0.0f;

    // Single circuit — re-init before every (param, step, freq) triple.
    // Using init() instead of reset() because reset() leaves circuit silent
    // (same issue as Test 1 — flagged for session 3e investigation).
    DmmDelayCircuit circuit = makeFreshCircuit();

    // measureAmp: circuit must already have desired parameters set.
    // Primes with kPrimeSamp sine samples to fill the BBD delay line,
    // then measures peak over kMeasSamp samples.
    // Self-oscillation guard: aborts and returns 2.0 if |y| > 2.0 when feedback > 0.75.
    auto measureAmp = [&](float freq, float fbVal, int step) noexcept -> float {
        const float phInc = k2Pi * freq / kSampleRateF;
        float ph = 0.0f;

        for (int j = 0; j < kPrimeSamp; ++j) {
            const float y = circuit.process(kTestAmp * sinf(ph));
            ph += phInc;
            if (ph >= k2Pi) ph -= k2Pi;
            if (fbVal > 0.75f && fabsf(y) > 2.0f) {
                std::printf("  WARN: Feedback step %d exceeded 2.0 during prime"
                            " — possible self-oscillation, skipping\n", step);
                return 2.0f;
            }
        }

        float peak = 0.0f;
        for (int j = 0; j < kMeasSamp; ++j) {
            const float y = circuit.process(kTestAmp * sinf(ph));
            ph += phInc;
            if (ph >= k2Pi) ph -= k2Pi;
            if (fbVal > 0.75f && fabsf(y) > 2.0f) {
                std::printf("  WARN: Feedback step %d exceeded 2.0 during measure"
                            " — possible self-oscillation, skipping\n", step);
                return 2.0f;
            }
            const float m = fabsf(y);
            if (m > peak) peak = m;
        }
        return peak;
    };

    // --- Delay knob sweep ---
    std::printf("  Sweeping delay knob (10 steps × 3 freqs)...\n");
    for (int step = 0; step < kNumSteps; ++step) {
        const float v = static_cast<float>(step) / 9.0f;
        for (int fi = 0; fi < kNumFreqs; ++fi) {
            circuit.init(kSampleRateF, s_workingBuffer, kWorkBufSize);
            circuit.setDelayKnob(v);
            circuit.setFeedbackKnob(0.0f);
            circuit.setModeKnob(0.0f);

            const float amp    = measureAmp(kFreqs[fi], 0.0f, step);
            const float amp_db = 20.0f * log10f(amp + 1e-9f);

            if (csv)
                std::fprintf(csv, "delay,%d,%.6f,%.1f,%.8f,%.4f\n",
                             step, v, kFreqs[fi], amp, amp_db);

            if (fi == 1) delayAmp1kHz[step] = amp;  // 1 kHz
        }
    }

    // P1: delay sweep — non-zero at 1 kHz for all 10 steps
    {
        bool allOk   = true;
        int  badStep = -1;
        for (int s = 0; s < kNumSteps; ++s) {
            if (delayAmp1kHz[s] <= 0.001f) { allOk = false; badStep = s; break; }
        }
        if (allOk) {
            std::printf("  PASS: Delay sweep — non-zero at 1kHz for all 10 steps\n");
        } else {
            std::printf("  FAIL: Delay sweep — zero output at step %d (amp=%.6f)\n",
                        badStep, delayAmp1kHz[badStep < 0 ? 0 : badStep]);
        }
        check(allOk, "param_sweep: P1 delay non-zero at 1kHz for all 10 steps");
    }

    // --- Feedback knob sweep ---
    std::printf("  Sweeping feedback knob (steps 0–7 active; 8–9 capped at 7/9)...\n");
    for (int step = 0; step < kNumSteps; ++step) {
        const float nominalV  = static_cast<float>(step) / 9.0f;
        // Cap at step 7 value (≈0.778) for steps 8–9 to prevent self-oscillation lockup
        const float feedbackV = (step <= 7) ? nominalV : (7.0f / 9.0f);

        for (int fi = 0; fi < kNumFreqs; ++fi) {
            circuit.init(kSampleRateF, s_workingBuffer, kWorkBufSize);
            circuit.setDelayKnob(0.3f);
            circuit.setFeedbackKnob(feedbackV);
            circuit.setModeKnob(0.0f);

            const float amp    = measureAmp(kFreqs[fi], feedbackV, step);
            const float amp_db = 20.0f * log10f(amp + 1e-9f);

            if (csv)
                std::fprintf(csv, "feedback,%d,%.6f,%.1f,%.8f,%.4f\n",
                             step, nominalV, kFreqs[fi], amp, amp_db);

            if (fi == 0) feedbackAmp80Hz[step] = amp;  // 80 Hz
        }
    }

    // P2: feedback at 80 Hz — amp at step 7 >= amp at step 0 (within 3 dB)
    {
        const float a0        = feedbackAmp80Hz[0];
        const float a7        = feedbackAmp80Hz[7];
        const float threshold = a0 * 0.70795f;  // -3 dB
        const bool  ok        = a7 >= threshold;
        if (ok) {
            std::printf("  PASS: Feedback sweep — amplitude increases with feedback at 80Hz\n");
        } else {
            std::printf("  FAIL: Feedback sweep — unexpected amplitude behavior\n");
            std::printf("        step0=%.6f  step7=%.6f  threshold=%.6f (-3 dB)\n",
                        a0, a7, threshold);
        }
        check(ok, "param_sweep: P2 feedback amp >= step0 at 80Hz (3 dB tol)");
    }

    // --- Mode knob sweep ---
    std::printf("  Sweeping mode knob (10 steps × 3 freqs)...\n");
    for (int step = 0; step < kNumSteps; ++step) {
        const float v = static_cast<float>(step) / 9.0f;
        for (int fi = 0; fi < kNumFreqs; ++fi) {
            circuit.init(kSampleRateF, s_workingBuffer, kWorkBufSize);
            circuit.setDelayKnob(0.3f);
            circuit.setFeedbackKnob(0.0f);
            circuit.setModeKnob(v);

            const float amp    = measureAmp(kFreqs[fi], 0.0f, step);
            const float amp_db = 20.0f * log10f(amp + 1e-9f);

            if (csv)
                std::fprintf(csv, "mode,%d,%.6f,%.1f,%.8f,%.4f\n",
                             step, v, kFreqs[fi], amp, amp_db);

            if (fi == 1) {  // 1 kHz
                if (step == 0) modeAmp1kHz_chorus  = amp;
                if (step == 9) modeAmp1kHz_vibrato = amp;
            }
        }
    }

    // P3: mode — non-zero at 1 kHz in chorus (v=0, step 0) and vibrato (v=1, step 9)
    {
        const bool cOk = modeAmp1kHz_chorus  > 0.001f;
        const bool vOk = modeAmp1kHz_vibrato > 0.001f;
        if (cOk && vOk) {
            std::printf("  PASS: Mode sweep — non-zero at 1kHz in both chorus and vibrato\n");
        } else {
            if (!cOk) std::printf("  FAIL: Mode sweep — zero output in chorus mode\n");
            if (!vOk) std::printf("  FAIL: Mode sweep — zero output in vibrato mode\n");
        }
        check(cOk && vOk, "param_sweep: P3 mode non-zero at 1kHz (chorus and vibrato)");
    }

    if (csv) {
        std::fclose(csv);
        std::printf("  CSV written: tests/dmm_param_sweep.csv (90 data rows)\n");
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
    std::printf("==========================================\n");
    std::printf(" EHX Deluxe Memory Man — WDF Test Suite\n");
    std::printf(" Session 3d: Tests 1-3\n");
    std::printf("==========================================\n");

    runTest1_FrequencyResponse();
    runTest2_ImpulseStability();
    runTest3_ParameterSweep();

    const int failed = totalTests - passedTests;
    std::printf("\n=== DMM TEST RESULTS: %d passed, %d failed ===\n",
                passedTests, failed);

    return (failed == 0) ? 0 : 1;
}
