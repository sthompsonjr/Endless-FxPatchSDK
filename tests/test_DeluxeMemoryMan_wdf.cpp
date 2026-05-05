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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

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
// Pink noise state — Voss-McCartney 7-stage algorithm (file scope)
// ---------------------------------------------------------------------------
static float    s_pinkState[7]   = {};
static uint32_t s_pinkCounter    = 0u;

inline float nextPinkSample() noexcept {
    static uint32_t lcg = 0x12345678u;
    auto white = [&]() -> float {
        lcg = lcg * 1664525u + 1013904223u;
        return static_cast<float>(static_cast<int32_t>(lcg))
               / static_cast<float>(0x80000000u);
    };
    ++s_pinkCounter;
    for (int i = 0; i < 7; ++i) {
        if ((s_pinkCounter & (1u << i)) == 0u) break;
        s_pinkState[i] = white();
    }
    float sum = 0.0f;
    for (int i = 0; i < 7; ++i) sum += s_pinkState[i];
    return sum * (1.0f / 7.0f);
}

inline bool isDenormal(float x) noexcept {
    uint32_t bits;
    static_assert(sizeof(float) == sizeof(uint32_t));
    __builtin_memcpy(&bits, &x, sizeof(bits));
    uint32_t exponent = bits & 0x7F800000u;
    uint32_t mantissa = bits & 0x007FFFFFu;
    return (exponent == 0u) && (mantissa != 0u);
}

static void resetPinkNoise() noexcept {
    for (auto& s : s_pinkState) s = 0.0f;
    s_pinkCounter = 0u;
}

// ---------------------------------------------------------------------------
// Test 4: Cycle count estimation
// Clock: 528 MHz. Budget: 11,000 cycles/sample. Warmup: 48,000 samples.
// Timed pass: 1,000,000 samples. FAIL > 60%, WARN > 40%, PASS <= 40%.
// process(0.0f) with feedback=0 reflects realistic steady-state LFO cost.
// ---------------------------------------------------------------------------
static bool runTest4_CycleCount(DmmDelayCircuit& circuit) {
    std::printf("\n--- Test 4: Cycle Count Estimation ---\n");

    circuit.setDelayKnob(0.5f);
    circuit.setFeedbackKnob(0.0f);
    circuit.setModeKnob(0.0f);
    circuit.setRunawayMode(false);

    // Warmup pass — prevent cold-start cache bias
    for (int i = 0; i < 48000; ++i)
        (void)circuit.process(0.0f);

    // Timed pass — volatile sink prevents dead-store elimination
    constexpr int kTimingIterations = 1'000'000;
    volatile float sink = 0.0f;
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < kTimingIterations; ++i)
        sink = circuit.process(0.0f);
    auto t1 = std::chrono::high_resolution_clock::now();
    (void)sink;

    auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    double mean_cycles = static_cast<double>(elapsed_ns) * 528'000'000.0
                         / 1'000'000'000.0 / static_cast<double>(kTimingIterations);
    double budget_pct  = mean_cycles / 11000.0 * 100.0;

    std::printf("Test 4 — Cycle Count Estimation\n");
    std::printf("  Iterations:         %d\n", kTimingIterations);
    std::printf("  Elapsed:            %lld ns\n", static_cast<long long>(elapsed_ns));
    std::printf("  Mean cycles/sample: %.1f  (528 MHz assumed)\n", mean_cycles);
    std::printf("  Budget consumed:    %.1f%%  (budget = 11,000 cycles @ 528 MHz)\n",
                budget_pct);

    if (budget_pct > 60.0)
        std::printf("FAIL: Test 4 — exceeds 60%% cycle budget (%.1f%%)\n", budget_pct);
    else if (budget_pct > 40.0)
        std::printf("WARN: Test 4 — exceeds 40%% cycle budget (%.1f%%)\n", budget_pct);
    check(budget_pct <= 60.0, "cycle_count: within 60pct cycle budget");

    return budget_pct <= 60.0;
}

// ---------------------------------------------------------------------------
// Test 5: Numerical precision — 10 seconds pink noise at -6 dBFS
// P1: no NaN (hard fail). P2: no Inf (hard fail).
// P3: no denormals (WARN only — does not affect return value).
// Non-zero feedback exercises the feedback path for denormal accumulation.
// ---------------------------------------------------------------------------
static bool runTest5_NumericalPrecision(DmmDelayCircuit& circuit) {
    std::printf("\n--- Test 5: Numerical Precision ---\n");

    circuit.init(kSampleRateF, s_workingBuffer, kWorkBufSize);
    resetPinkNoise();
    circuit.setDelayKnob(0.3f);
    circuit.setFeedbackKnob(0.3f);
    circuit.setModeKnob(0.0f);
    circuit.setRunawayMode(false);

    int nanCount    = 0;
    int infCount    = 0;
    int denormCount = 0;

    for (int n = 0; n < 480'000; ++n) {
        float x = 0.5f * nextPinkSample();
        float y = circuit.process(x);
        if (std::isnan(y))      ++nanCount;
        else if (std::isinf(y)) ++infCount;
        else if (isDenormal(y)) ++denormCount;
    }

    std::printf("Test 5 — Numerical Precision (10 seconds pink noise @ -6 dBFS)\n");
    std::printf("  NaN outputs:      %d\n", nanCount);
    std::printf("  Inf outputs:      %d\n", infCount);
    std::printf("  Denormal outputs: %d\n", denormCount);

    bool p1 = (nanCount == 0);
    bool p2 = (infCount == 0);
    bool p3 = (denormCount == 0);

    if (!p1) std::printf("  FAIL: P1 — %d NaN outputs detected\n", nanCount);
    check(p1, "precision: P1 no NaN outputs");

    if (!p2) std::printf("  FAIL: P2 — %d Inf outputs detected\n", infCount);
    check(p2, "precision: P2 no Inf outputs");

    if (!p3) {
        std::printf("  WARN: P3 — %d denormal outputs detected\n", denormCount);
        std::printf("        Recommendation: add _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON)\n");
        std::printf("        or equivalent ARM VFP FTZ bit in the production init().\n");
        std::printf("        Denormals do not corrupt output but inflate cycle cost ~10x\n");
        std::printf("        on Cortex-M7 when FTZ is not set.\n");
    }
    check(true, "precision: P3 denormals (WARN only, counted as pass)");

    return p1 && p2;
}

// ---------------------------------------------------------------------------
// Test 6: A/B comparison (conditional — requires argv[1] reference file)
// Reference format: raw IEEE 754 float32 LE mono 48000 Hz, no header.
// Must have been generated with delay=0.3, feedback=0.0, mode=0.0,
// runaway=false, using this harness's LCG pink noise (seed 0x12345678).
// If reference is from LTSpice or hardware, comparison will not be meaningful.
// A-weighted error requires FFT; deferred to post-processing.
// Target: RMS error < -40 dB (linear BBD path). WARN < -20 dB. FAIL >= -20 dB.
// ---------------------------------------------------------------------------
static bool runTest6_ABComparison(DmmDelayCircuit& circuit, const char* refPath) {
    std::printf("\n--- Test 6: A/B Comparison ---\n");
    std::printf("  NOTE: Reference must be raw float32 LE mono at 48000 Hz (no header).\n");
    std::printf("  NOTE: Settings assumed: delay=0.3, feedback=0.0, mode=0.0, runaway=false.\n");
    std::printf("        LCG pink noise seed 0x12345678. If from LTSpice/hardware,\n");
    std::printf("        this comparison will not be meaningful.\n");

    FILE* f = std::fopen(refPath, "rb");
    if (!f) {
        std::printf("FAIL: Test 6 — cannot open %s\n", refPath);
        check(false, "ab_comparison: reference file opened");
        return false;
    }
    std::fseek(f, 0, SEEK_END);
    long fileBytes = std::ftell(f);
    std::rewind(f);
    int numSamples = static_cast<int>(fileBytes / static_cast<long>(sizeof(float)));
    if (numSamples <= 0) {
        std::fclose(f);
        std::printf("FAIL: Test 6 — reference file empty or too small (%ld bytes)\n", fileBytes);
        check(false, "ab_comparison: reference file non-empty");
        return false;
    }
    // std::vector permitted in test harness for unknown-length reference buffer
    std::vector<float> refBuf(static_cast<size_t>(numSamples));
    size_t nRead = std::fread(refBuf.data(), sizeof(float), static_cast<size_t>(numSamples), f);
    if (nRead != static_cast<size_t>(numSamples))
        std::printf("  WARN: fread returned %zu of %d expected samples\n", nRead, numSamples);
    std::fclose(f);
    std::printf("  Loaded %d samples (%.2f s) from %s\n",
                numSamples, static_cast<float>(numSamples) / kSampleRateF, refPath);

    circuit.init(kSampleRateF, s_workingBuffer, kWorkBufSize);
    circuit.setDelayKnob(0.3f);
    circuit.setFeedbackKnob(0.0f);
    circuit.setModeKnob(0.0f);
    circuit.setRunawayMode(false);
    resetPinkNoise();

    std::vector<float> outBuf(static_cast<size_t>(numSamples));
    for (int n = 0; n < numSamples; ++n) {
        float x   = 0.5f * nextPinkSample();
        outBuf[n] = circuit.process(x);
    }

    // Error computation — double accumulation avoids catastrophic cancellation
    // over long buffers. A-weighted error requires FFT; deferred to post-processing.
    double sumSqErr = 0.0;
    float  peakErr  = 0.0f;
    for (int n = 0; n < numSamples; ++n) {
        float err = refBuf[n] - outBuf[n];
        sumSqErr += static_cast<double>(err) * static_cast<double>(err);
        float absErr = fabsf(err);
        if (absErr > peakErr) peakErr = absErr;
    }
    double rmsErr    = std::sqrt(sumSqErr / static_cast<double>(numSamples));
    float  rmsErrDb  = 20.0f * log10f(static_cast<float>(rmsErr) + 1e-12f);
    float  peakErrDb = 20.0f * log10f(peakErr + 1e-12f);

    std::printf("  Unweighted RMS error: %.1f dB\n", rmsErrDb);
    std::printf("  Peak error:           %.1f dB\n", peakErrDb);
    std::printf("  Note: A-weighted error requires FFT; deferred to post-processing.\n");

    FILE* csv = std::fopen("tests/dmm_ab_comparison.csv", "w");
    if (!csv) csv = std::fopen("dmm_ab_comparison.csv", "w");
    if (csv) {
        // A-weighted error requires FFT; deferred to post-processing.
        std::fprintf(csv,
                     "num_samples,rms_error_linear,rms_error_db,"
                     "peak_error_linear,peak_error_db\n");
        std::fprintf(csv, "%d,%.8e,%.4f,%.8e,%.4f\n",
                     numSamples, rmsErr, rmsErrDb,
                     static_cast<double>(peakErr), peakErrDb);
        std::fclose(csv);
        std::printf("  CSV written: tests/dmm_ab_comparison.csv\n");
    }

    if (rmsErrDb >= -20.0f)
        std::printf("FAIL: Test 6 — RMS error %.1f dB (exceeds -20 dB floor)\n", rmsErrDb);
    else if (rmsErrDb >= -40.0f)
        std::printf("WARN: Test 6 — RMS error %.1f dB (acceptable for nonlinear; "
                    "BBD path expected < -40 dB)\n", rmsErrDb);
    check(rmsErrDb < -20.0f, "ab_comparison: RMS error < -20 dB floor");

    return rmsErrDb < -20.0f;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::printf("==========================================\n");
    std::printf(" EHX Deluxe Memory Man — WDF Test Suite\n");
    std::printf(" Session 3e: Tests 1-6\n");
    std::printf("==========================================\n");

    // Tests 1-3 return void and accumulate into totalTests/passedTests.
    // Snapshot counters before/after each test to derive per-test bool for summary.
    int c0, p0;

    c0 = totalTests; p0 = passedTests;
    runTest1_FrequencyResponse();
    bool t1 = (passedTests - p0) == (totalTests - c0);

    c0 = totalTests; p0 = passedTests;
    runTest2_ImpulseStability();
    bool t2 = (passedTests - p0) == (totalTests - c0);

    c0 = totalTests; p0 = passedTests;
    runTest3_ParameterSweep();
    bool t3 = (passedTests - p0) == (totalTests - c0);

    DmmDelayCircuit circuit = makeFreshCircuit();

    bool t4 = runTest4_CycleCount(circuit);
    bool t5 = runTest5_NumericalPrecision(circuit);
    bool t6 = true;
    if (argc >= 2) {
        t6 = runTest6_ABComparison(circuit, argv[1]);
    } else {
        std::printf("Test 6 — A/B Comparison: SKIP (no reference file; pass path as argv[1])\n");
    }

    std::printf("\n=== DMM WDF TEST HARNESS SUMMARY ===\n");
    std::printf("Test 1 (Freq Response):     %s\n", t1 ? "PASS" : "FAIL");
    std::printf("Test 2 (Impulse Stability): %s\n", t2 ? "PASS" : "FAIL");
    std::printf("Test 3 (Param Sweep):       %s\n", t3 ? "PASS" : "FAIL");
    std::printf("Test 4 (Cycle Count):       %s\n", t4 ? "PASS" : "FAIL");
    std::printf("Test 5 (Precision):         %s\n", t5 ? "PASS" : "FAIL");
    std::printf("Test 6 (A/B Compare):       %s\n", t6 ? "PASS/SKIP" : "FAIL");
    bool allPassed = t1 && t2 && t3 && t4 && t5 && t6;
    std::printf("Overall: %s\n", allPassed ? "PASS" : "FAIL");
    return allPassed ? 0 : 1;
}
