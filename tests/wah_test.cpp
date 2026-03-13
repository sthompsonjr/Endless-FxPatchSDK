// Wah pedal test suite -- CryBabyCircuit and AutoWahCircuit.
// Build: g++ -std=c++20 -O2 -Wall -Werror -Wdouble-promotion -I.. tests/wah_test.cpp -o wah_test -lm

#include "wdf/WdfWahCircuit.h"
#include "dsp/Saturation.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>

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

// ============================================================
// DFT helper — measures energy in a frequency band.
// Uses a simple DFT over N samples (no heap, fixed-size array).
// ============================================================

// Compute energy in [fLow, fHigh] Hz from a 512-sample buffer.
// Uses a cosine-weighted sum rather than full DFT for efficiency.
static float bandEnergy(const float* buf, int n, float fLow, float fHigh, float sr) {
    float energy = 0.0f;
    int kLow  = static_cast<int>(fLow  * static_cast<float>(n) / sr);
    int kHigh = static_cast<int>(fHigh * static_cast<float>(n) / sr);
    kLow  = kLow  < 1 ? 1 : kLow;
    kHigh = kHigh > n / 2 ? n / 2 : kHigh;

    for (int k = kLow; k <= kHigh; ++k) {
        float re = 0.0f;
        float im = 0.0f;
        float twoPiKoverN = 6.283185307f * static_cast<float>(k) / static_cast<float>(n);
        for (int i = 0; i < n; ++i) {
            float fi = static_cast<float>(i);
            re += buf[i] * cosf(twoPiKoverN * fi);
            im += buf[i] * sinf(twoPiKoverN * fi);
        }
        energy += re * re + im * im;
    }
    return energy;
}

// ============================================================
// LCG noise generator (no heap, inline)
// ============================================================
static float nextNoise(uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return static_cast<float>(static_cast<int32_t>(state)) / 2147483648.0f;
}

// ============================================================
// Test 1: Bandpass peak sweeps upward as sweep increases
// ============================================================
static void testBandpassPeakSweep() {
    std::printf("\n--- Bandpass Peak Sweep ---\n");

    constexpr int kBufLen = 512;
    float buf[kBufLen];
    constexpr float kSr = 48000.0f;

    // For each of 5 sweep positions, measure energy in 3 bands:
    //   low:  100–600 Hz
    //   mid:  600–1500 Hz
    //   high: 1500–5000 Hz
    // The band with maximum energy should increase with sweep position.

    float sweeps[] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
    int prevPeakBand = -1;
    bool sweepIncreasing = true;

    for (float sweep : sweeps) {
        CryBabyCircuit wah;
        wah.init(kSr);
        wah.setSweep(sweep);
        wah.setOutputLevel(5.0f); // boost output for better SNR in test

        // Settle the smoother (sweep takes ~10ms=480 samples to reach target)
        uint32_t rng = 12345u;
        for (int i = 0; i < 960; ++i) {
            (void)wah.process(nextNoise(rng) * 0.5f);
        }

        // Collect output
        for (int i = 0; i < kBufLen; ++i) {
            buf[i] = wah.process(nextNoise(rng) * 0.5f);
        }

        float eLow  = bandEnergy(buf, kBufLen, 100.0f,  600.0f,  kSr);
        float eMid  = bandEnergy(buf, kBufLen, 600.0f,  1500.0f, kSr);
        float eHigh = bandEnergy(buf, kBufLen, 1500.0f, 5000.0f, kSr);

        int peakBand = 0;
        if (eMid > eLow  && eMid >= eHigh) peakBand = 1;
        if (eHigh > eLow && eHigh > eMid)  peakBand = 2;

        if (peakBand < prevPeakBand) sweepIncreasing = false;
        prevPeakBand = peakBand;
    }

    check(sweepIncreasing, "Sweep: peak band moves upward as sweep increases");
}

// ============================================================
// Test 2: Higher resonance → sharper peak
// ============================================================
static void testResonancePeak() {
    std::printf("\n--- Resonance Peak ---\n");

    constexpr int kBufLen = 512;
    float buf[kBufLen];
    constexpr float kSr = 48000.0f;

    auto measurePeakEnergy = [&](float resonance) {
        CryBabyCircuit wah;
        wah.init(kSr);
        wah.setSweep(0.5f);
        wah.setResonance(resonance);
        wah.setOutputLevel(5.0f);

        uint32_t rng = 99999u;
        // Settle smoothers and resonance
        for (int i = 0; i < 2000; ++i) {
            (void)wah.process(nextNoise(rng) * 0.5f);
        }
        for (int i = 0; i < kBufLen; ++i) {
            buf[i] = wah.process(nextNoise(rng) * 0.5f);
        }
        // Energy in a narrow band around center (~1 kHz)
        return bandEnergy(buf, kBufLen, 700.0f, 1500.0f, kSr);
    };

    float energyLowQ  = measurePeakEnergy(0.0f);
    float energyHighQ = measurePeakEnergy(0.8f);

    check(energyHighQ > energyLowQ,
          "Resonance: higher Q gives more energy at center frequency");
}

// ============================================================
// Test 3: Frequency range — sweep=0 is low, sweep=1 is high
// ============================================================
static void testFrequencyRange() {
    std::printf("\n--- Frequency Range ---\n");

    constexpr int kBufLen = 512;
    float buf[kBufLen];
    constexpr float kSr = 48000.0f;

    auto measureBands = [&](float sweep, float& outLow, float& outHigh) {
        CryBabyCircuit wah;
        wah.init(kSr);
        wah.setSweep(sweep);
        wah.setOutputLevel(5.0f);

        uint32_t rng = 77777u;
        for (int i = 0; i < 960; ++i) {
            (void)wah.process(nextNoise(rng) * 0.5f);
        }
        for (int i = 0; i < kBufLen; ++i) {
            buf[i] = wah.process(nextNoise(rng) * 0.5f);
        }
        outLow  = bandEnergy(buf, kBufLen, 100.0f,  600.0f, kSr);
        outHigh = bandEnergy(buf, kBufLen, 1000.0f, 5000.0f, kSr);
    };

    float lowAtHeel, highAtHeel, lowAtToe, highAtToe;
    measureBands(0.0f, lowAtHeel, highAtHeel);
    measureBands(1.0f, lowAtToe,  highAtToe);

    check(lowAtHeel > highAtHeel,
          "Freq range: sweep=0 (heel) has more energy below 600Hz than above 1kHz");
    check(highAtToe > lowAtToe,
          "Freq range: sweep=1 (toe) has more energy above 1kHz than below 600Hz");
}

// ============================================================
// Test 4: Expression pedal smoothing — no click on step change
// ============================================================
static void testSweepSmoothing() {
    std::printf("\n--- Sweep Smoothing ---\n");

    constexpr float kSr = 48000.0f;
    CryBabyCircuit wah;
    wah.init(kSr);
    wah.setSweep(0.0f);
    wah.setOutputLevel(5.0f);

    uint32_t rng = 55555u;
    // Settle at sweep=0
    for (int i = 0; i < 960; ++i) {
        (void)wah.process(nextNoise(rng) * 0.5f);
    }

    // Step to sweep=1 and check for no large single-sample jumps
    wah.setSweep(1.0f);
    float prev = wah.process(nextNoise(rng) * 0.5f);
    float maxJump = 0.0f;
    for (int i = 1; i < 480; ++i) { // 10ms = 480 samples at 48kHz
        float out = wah.process(nextNoise(rng) * 0.5f);
        float jump = fabsf(out - prev);
        if (jump > maxJump) maxJump = jump;
        prev = out;
    }

    check(maxJump < 0.1f, "Sweep smoothing: no click on step change (max jump < 0.1)");
}

// ============================================================
// Test 5: Auto-wah trigger — sweep opens on loud transient
// ============================================================
static void testAutoWahTrigger() {
    std::printf("\n--- Auto-Wah Trigger ---\n");

    constexpr float kSr = 48000.0f;
    AutoWahCircuit wah;
    wah.init(kSr);
    wah.setMode(AutoWahCircuit::Mode::Envelope);
    wah.setSensitivity(0.8f);
    wah.setOutputLevel(5.0f);

    // 4800 silent samples
    float silentRms = 0.0f;
    for (int i = 0; i < 4800; ++i) {
        float out = wah.process(0.0f, 0.5f);
        silentRms += out * out;
    }
    silentRms = sqrtf(silentRms / 4800.0f);

    // 4800 loud samples (amplitude 0.7)
    uint32_t rng = 33333u;
    float loudRms = 0.0f;
    for (int i = 0; i < 4800; ++i) {
        float in  = nextNoise(rng) * 0.7f;
        float out = wah.process(in, 0.5f);
        loudRms += out * out;
    }
    loudRms = sqrtf(loudRms / 4800.0f);

    check(loudRms > silentRms,
          "Auto-wah: more output energy when loud transient opens the wah");
}

// ============================================================
// Test 6: Self-oscillation at high resonance
// ============================================================
static void testSelfOscillation() {
    std::printf("\n--- Self-Oscillation ---\n");

    constexpr float kSr = 48000.0f;
    CryBabyCircuit wah;
    wah.init(kSr);
    wah.setSweep(0.5f);
    wah.setResonance(1.0f);
    wah.setOutputLevel(2.0f);

    // Allow resonance smoother to settle
    for (int i = 0; i < 2400; ++i) {
        (void)wah.process(0.0f);
    }

    // Collect 48000 samples with zero input
    float sumSq = 0.0f;
    bool bounded = true;
    for (int i = 0; i < 48000; ++i) {
        float out = wah.process(0.0f);
        sumSq += out * out;
        if (fabsf(out) > 1.0f) {
            bounded = false;
        }
    }
    float rms = sqrtf(sumSq / 48000.0f);

    check(rms > 0.001f, "Self-oscillation: nonzero energy with zero input at high resonance");
    check(bounded,      "Self-oscillation: output bounded to ±1.0f");
}

// ============================================================
// Test 7: Stability under silence (no self-oscillation at normal Q)
// ============================================================
static void testStabilityUnderSilence() {
    std::printf("\n--- Stability Under Silence ---\n");

    constexpr float kSr = 48000.0f;

    auto testMode = [&](AutoWahCircuit::Mode mode, const char* name) {
        AutoWahCircuit wah;
        wah.init(kSr);
        wah.setMode(mode);
        wah.setResonance(0.5f); // below self-oscillation threshold
        wah.setOutputLevel(2.0f);

        // Run 96000 samples of silence
        for (int i = 0; i < 96000; ++i) {
            (void)wah.process(0.0f, 0.5f);
        }

        // Measure last 1000 samples
        float rms = 0.0f;
        for (int i = 0; i < 1000; ++i) {
            float out = wah.process(0.0f, 0.5f);
            rms += out * out;
        }
        rms = sqrtf(rms / 1000.0f);

        check(rms < 0.01f, name);
    };

    testMode(AutoWahCircuit::Mode::Expression, "Stability: Expression mode settles under silence");
    testMode(AutoWahCircuit::Mode::Envelope,   "Stability: Envelope mode settles under silence");
    testMode(AutoWahCircuit::Mode::Lfo,        "Stability: Lfo mode settles under silence");
    testMode(AutoWahCircuit::Mode::EnvLfo,     "Stability: EnvLfo mode settles under silence");
}

// ============================================================
// Test 8: No NaN propagation
// ============================================================
static void testNoNaN() {
    std::printf("\n--- No NaN Propagation ---\n");

    constexpr float kSr = 48000.0f;

    struct Corner { float sweep; float resonance; };
    Corner corners[] = {
        { 0.0f, 0.0f }, { 1.0f, 0.0f },
        { 0.0f, 1.0f }, { 1.0f, 1.0f }
    };

    bool allFinite = true;
    for (auto [sweep, resonance] : corners) {
        CryBabyCircuit wah;
        wah.init(kSr);
        wah.setSweep(sweep);
        wah.setResonance(resonance);
        wah.setOutputLevel(1.0f);

        // Alternate ±1 input
        for (int i = 0; i < 24000; ++i) {
            float in  = (i & 1) ? 1.0f : -1.0f;
            float out = wah.process(in);
            if (!std::isfinite(out)) {
                allFinite = false;
                break;
            }
        }
        if (!allFinite) break;
    }

    check(allFinite, "No NaN: all outputs finite at (sweep, resonance) extremes");
}

// ============================================================
// Test 9: Performance
// ============================================================
static void testPerformance() {
    std::printf("\n--- Performance ---\n");

    constexpr int kSamples = 100000;
    constexpr float kSr = 48000.0f;

    uint32_t rng = 11111u;
    float inputs[256];
    for (int i = 0; i < 256; ++i) {
        inputs[i] = nextNoise(rng) * 0.5f;
    }

    // CryBabyCircuit
    {
        CryBabyCircuit wah;
        wah.init(kSr);
        wah.setSweep(0.5f);

        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) {
            dummy = wah.process(inputs[i & 255]);
        }
        auto end = std::chrono::high_resolution_clock::now();
        (void)dummy;

        long long ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::printf("  INFO: CryBabyCircuit: %lld ns/sample\n",
                    static_cast<long long>(ns / kSamples));
        check(true, "CryBabyCircuit performance measured");
    }

    // AutoWahCircuit (Envelope mode)
    {
        AutoWahCircuit wah;
        wah.init(kSr);
        wah.setMode(AutoWahCircuit::Mode::Envelope);
        wah.setSensitivity(0.7f);

        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) {
            dummy = wah.process(inputs[i & 255], 0.5f);
        }
        auto end = std::chrono::high_resolution_clock::now();
        (void)dummy;

        long long ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::printf("  INFO: AutoWahCircuit (Envelope): %lld ns/sample\n",
                    static_cast<long long>(ns / kSamples));
        check(true, "AutoWahCircuit performance measured");
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    std::printf("=== Wah Pedal Test Suite ===\n");

    testBandpassPeakSweep();
    testResonancePeak();
    testFrequencyRange();
    testSweepSmoothing();
    testAutoWahTrigger();
    testSelfOscillation();
    testStabilityUnderSilence();
    testNoNaN();
    testPerformance();

    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);
    return (passedTests == totalTests) ? 0 : 1;
}
