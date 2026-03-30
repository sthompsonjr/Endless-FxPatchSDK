#include "dsp/ReverbPrimitives.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <chrono>

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

static bool approx(float a, float b, float eps = 0.001f) {
    return fabsf(a - b) < eps;
}

static float l2norm(const float* x, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; ++i) sum += x[i] * x[i];
    return sqrtf(sum);
}

// ============================================================
// hadamard8 tests
// ============================================================
static void testHadamard8() {
    std::printf("\n--- hadamard8 ---\n");

    // Test 1: Losslessness — unit impulse e0
    {
        float x[8] = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        float normIn = l2norm(x, 8);
        fdn::hadamard8(x);
        float normOut = l2norm(x, 8);
        check(approx(normOut, normIn, 1e-5f),
              "hadamard8 lossless unit impulse e0");
    }

    // Test 2: Losslessness — unit impulse e3
    {
        float x[8] = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        float normIn = l2norm(x, 8);
        fdn::hadamard8(x);
        float normOut = l2norm(x, 8);
        check(approx(normOut, normIn, 1e-5f),
              "hadamard8 lossless unit impulse e3");
    }

    // Test 3: Losslessness — non-uniform input
    {
        float x[8] = { 0.1f, 0.3f, 0.5f, 0.7f, 0.2f, 0.4f, 0.6f, 0.8f };
        float normIn = l2norm(x, 8);
        fdn::hadamard8(x);
        float normOut = l2norm(x, 8);
        check(approx(normOut, normIn, 1e-5f),
              "hadamard8 lossless non-uniform input");
    }

    // Test 4: Involutory — H8 * H8 = I
    {
        float x[8]        = { 0.1f, -0.3f, 0.5f, -0.7f, 0.2f, -0.4f, 0.6f, -0.8f };
        float original[8] = { 0.1f, -0.3f, 0.5f, -0.7f, 0.2f, -0.4f, 0.6f, -0.8f };
        fdn::hadamard8(x);
        fdn::hadamard8(x);
        bool ok = true;
        for (int i = 0; i < 8; ++i) {
            if (!approx(x[i], original[i], 1e-4f)) ok = false;
        }
        check(ok, "hadamard8 involutory (H8*H8 = I)");
    }

    // Test 5: DC input — all ones -> [sqrt(8), 0, 0, ..., 0]
    {
        float x[8] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        fdn::hadamard8(x);
        float expected0 = sqrtf(8.0f);
        bool ok = approx(x[0], expected0, 1e-5f);
        for (int i = 1; i < 8; ++i) {
            if (!approx(x[i], 0.0f, 1e-5f)) ok = false;
        }
        check(ok, "hadamard8 DC input -> sqrt(8) in output[0]");
    }

    // Test 6: Alternating input — energy concentrates in one bin
    {
        float x[8] = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f };
        fdn::hadamard8(x);
        float expected = sqrtf(8.0f);
        float maxVal = 0.0f;
        int maxIdx = -1;
        for (int i = 0; i < 8; ++i) {
            if (fabsf(x[i]) > maxVal) { maxVal = fabsf(x[i]); maxIdx = i; }
        }
        bool ok = approx(maxVal, expected, 1e-5f);
        for (int i = 0; i < 8; ++i) {
            if (i != maxIdx && !approx(x[i], 0.0f, 1e-5f)) ok = false;
        }
        check(ok, "hadamard8 alternating input concentrates energy");
    }

    // Test 7: Max magnitude bounded by L2 norm + all outputs finite
    {
        float x[8] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        fdn::hadamard8(x);
        bool ok = true;
        for (int i = 0; i < 8; ++i) {
            if (fabsf(x[i]) > sqrtf(8.0f) + 1e-4f) ok = false;
            if (!std::isfinite(x[i])) ok = false;
        }
        check(ok, "hadamard8 max magnitude bounded and finite");
    }

    // Test 8: Structural consistency with H4 decomposition
    // When x[4..7] = 0, output[0..3] == output[4..7]
    {
        float x[8] = { 0.5f, -0.3f, 0.8f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f };
        fdn::hadamard8(x);
        bool ok = true;
        for (int i = 0; i < 4; ++i) {
            if (!approx(x[i], x[i + 4], 1e-5f)) ok = false;
        }
        check(ok, "hadamard8 matches H4 structural decomposition");
    }

    // Test 9: Iterated stability — 10,000 applications, energy preserved
    {
        float x[8] = { 0.5f, 0.5f, 0.5f, 0.5f, -0.5f, -0.5f, -0.5f, -0.5f };
        float normInit = l2norm(x, 8);
        for (int i = 0; i < 8; ++i) x[i] /= normInit;

        for (int iter = 0; iter < 10000; ++iter) {
            fdn::hadamard8(x);
        }
        float normFinal = l2norm(x, 8);
        check(fabsf(normFinal - 1.0f) < 1e-3f,
              "hadamard8 iterated stability (10k iterations)");
    }

    // Test 10: Performance benchmark (informational, no assertion)
    {
        float x[8] = { 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f };
        constexpr int ITERATIONS = 1000000;
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < ITERATIONS; ++i) {
            fdn::hadamard8(x);
            x[0] += 1e-20f;  // prevent dead-code elimination
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
        double nsPerCall = ns / ITERATIONS;
        std::printf("  INFO: hadamard8 performance: %.2f ns/call\n", nsPerCall);
        check(true, "hadamard8 performance benchmark (informational)");
    }
}

// ============================================================
// ModulatedAllpassDelay tests
// ============================================================

// Helper: run N samples through a ModulatedAllpassDelay, return last output.
template<size_t S>
static float runModAllpass(ModulatedAllpassDelay<S>& ap, float inputDC, int numSamples) {
    float out = 0.0f;
    for (int i = 0; i < numSamples; ++i)
        out = ap.process(inputDC);
    return out;
}

static void testModulatedAllpassDelay() {
    std::printf("\n--- ModulatedAllpassDelay ---\n");

    // Test 1: Static mode — DC gain converges to 1.0
    // H(z=1) = (1-g)/(1-g) = 1 for any |g|<1.
    {
        constexpr size_t S = 512;
        ModulatedAllpassDelay<S> ap;
        ap.init(100.0f, 0.6f, 0.0f, 0.0f, 48000.0f, 0.0f);
        const float output = runModAllpass(ap, 1.0f, 2000);
        check(approx(output, 1.0f, 1e-3f),
              "ModulatedAllpassDelay static DC gain = 1.0");
    }

    // Test 2: Unity magnitude (allpass property) at multiple frequencies
    // |H(e^jw)| = 1 for all w. Verify via RMS ratio after settling.
    {
        constexpr size_t S = 512;
        const float sr = 48000.0f;
        const float test_freqs[] = { 100.0f, 1000.0f, 5000.0f, 10000.0f };
        bool allOk = true;

        for (float freq : test_freqs) {
            ModulatedAllpassDelay<S> ap;
            ap.init(47.0f, 0.6f, 0.0f, 0.0f, sr, 0.0f);

            const float phInc = freq / sr;
            float ph = 0.0f;

            // Settle (10000 samples for low-frequency group delay convergence)
            for (int i = 0; i < 10000; ++i) {
                (void)ap.process(sinf(kTwoPi * ph));
                ph += phInc;
                if (ph >= 1.0f) ph -= 1.0f;
            }

            // Measure
            float rmsIn = 0.0f, rmsOut = 0.0f;
            constexpr int MEASURE = 4800; // 10 cycles of 100Hz at 48kHz
            for (int i = 0; i < MEASURE; ++i) {
                const float s = sinf(kTwoPi * ph);
                const float o = ap.process(s);
                rmsIn  += s * s;
                rmsOut += o * o;
                ph += phInc;
                if (ph >= 1.0f) ph -= 1.0f;
            }
            rmsIn  = sqrtf(rmsIn  / MEASURE);
            rmsOut = sqrtf(rmsOut / MEASURE);
            const float ratio = rmsOut / (rmsIn + 1e-10f);
            std::printf("    %.0fHz: RMS ratio=%.6f\n", static_cast<double>(freq), static_cast<double>(ratio));
            if (fabsf(ratio - 1.0f) >= 0.02f) allOk = false;
        }
        check(allOk, "ModulatedAllpassDelay unity magnitude at 100/1k/5k/10kHz");
    }

    // Test 3: Stability over 100k samples with modulation active
    // No NaN/Inf, output magnitude < 10.0f at all times.
    {
        constexpr size_t S = 512;
        ModulatedAllpassDelay<S> ap;
        ap.init(200.0f, 0.65f, 1.2f, 8.0f, 48000.0f, 0.25f);

        const float f1 = 432.0f / 48000.0f;
        const float f2 = 1337.0f / 48000.0f;
        const float f3 = 7777.0f / 48000.0f;
        float ph1 = 0.0f, ph2 = 0.0f, ph3 = 0.0f;
        bool ok = true;

        for (int i = 0; i < 100000; ++i) {
            const float in = (sinf(kTwoPi * ph1) +
                              sinf(kTwoPi * ph2) +
                              sinf(kTwoPi * ph3)) / 3.0f;
            const float out = ap.process(in);
            if (!std::isfinite(out) || fabsf(out) >= 10.0f) { ok = false; break; }
            ph1 += f1; if (ph1 >= 1.0f) ph1 -= 1.0f;
            ph2 += f2; if (ph2 >= 1.0f) ph2 -= 1.0f;
            ph3 += f3; if (ph3 >= 1.0f) ph3 -= 1.0f;
        }
        check(ok, "ModulatedAllpassDelay stability 100k samples with modulation");
    }

    // Test 4: LFO phase offset affects impulse peak timing
    // phase=0 → D=center, phase=0.25 → D=center+depth
    {
        constexpr size_t S = 512;
        const float sr = 48000.0f;
        const float center = 100.0f;
        const float depth  = 20.0f;

        ModulatedAllpassDelay<S> apA;
        apA.init(center, 0.6f, 0.0f, depth, sr, 0.0f);   // frozen at phase=0, D=center
        ModulatedAllpassDelay<S> apB;
        apB.init(center, 0.6f, 0.0f, depth, sr, 0.25f);  // frozen at phase=0.25, D=center+depth

        constexpr int LEN = 512;
        float outA[LEN], outB[LEN];
        outA[0] = apA.process(1.0f);
        outB[0] = apB.process(1.0f);
        for (int i = 1; i < LEN; ++i) {
            outA[i] = apA.process(0.0f);
            outB[i] = apB.process(0.0f);
        }

        int peakA = 0, peakB = 0;
        float maxA = 0.0f, maxB = 0.0f;
        for (int i = 0; i < LEN; ++i) {
            if (fabsf(outA[i]) > maxA) { maxA = fabsf(outA[i]); peakA = i; }
            if (fabsf(outB[i]) > maxB) { maxB = fabsf(outB[i]); peakB = i; }
        }

        bool ok = (abs(peakA - (int)center) <= 2) &&
                  (abs(peakB - (int)(center + depth)) <= 2);
        std::printf("    peakA=%d (expect~%d), peakB=%d (expect~%d)\n",
                    peakA, (int)center, peakB, (int)(center + depth));
        check(ok, "ModulatedAllpassDelay LFO phase offset affects impulse timing");
    }

    // Test 5: setRate() changes LFO without discontinuity
    {
        constexpr size_t S = 512;
        ModulatedAllpassDelay<S> ap;
        ap.init(200.0f, 0.6f, 1.0f, 10.0f, 48000.0f, 0.0f);
        bool ok = true;

        for (int i = 0; i < 1000; ++i) {
            const float out = ap.process(0.5f * sinf(kTwoPi * i * 440.0f / 48000.0f));
            if (!std::isfinite(out) || fabsf(out) >= 10.0f) { ok = false; break; }
        }
        ap.setRate(2.0f, 48000.0f);
        for (int i = 0; i < 1000; ++i) {
            const float out = ap.process(0.5f * sinf(kTwoPi * i * 440.0f / 48000.0f));
            if (!std::isfinite(out) || fabsf(out) >= 10.0f) { ok = false; break; }
        }
        check(ok, "ModulatedAllpassDelay setRate continuity");
    }

    // Test 6: Delay clamping prevents buffer overread with extreme depth
    {
        constexpr size_t S = 256;
        ModulatedAllpassDelay<S> ap;
        ap.init(200.0f, 0.6f, 10.0f, 100.0f, 48000.0f, 0.0f);
        bool ok = true;

        for (int i = 0; i < 10000; ++i) {
            const float out = ap.process(0.3f);
            if (!std::isfinite(out)) { ok = false; break; }
        }
        check(ok, "ModulatedAllpassDelay delay clamping prevents overread");
    }

    // Test 7: reset() clears buffer — silence in → silence out
    {
        constexpr size_t S = 512;
        ModulatedAllpassDelay<S> ap;
        ap.init(100.0f, 0.6f, 1.0f, 5.0f, 48000.0f, 0.0f);

        for (int i = 0; i < 5000; ++i)
            (void)ap.process(sinf(kTwoPi * i * 440.0f / 48000.0f));

        ap.reset();
        bool ok = true;
        for (int i = 0; i < 200; ++i) {
            const float out = ap.process(0.0f);
            if (!approx(out, 0.0f, 1e-6f)) { ok = false; break; }
        }
        check(ok, "ModulatedAllpassDelay reset clears buffer");
    }

    // Test 8: Performance benchmark (informational)
    {
        constexpr size_t S = 512;
        ModulatedAllpassDelay<S> ap;
        ap.init(200.0f, 0.6f, 1.5f, 8.0f, 48000.0f, 0.0f);

        constexpr int ITERATIONS = 1000000;
        float acc = 0.0f;

        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < ITERATIONS; ++i) {
            acc += ap.process(acc * 1e-6f + 0.001f);
        }
        auto t1 = std::chrono::high_resolution_clock::now();

        double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
        double nsPerCall = ns / ITERATIONS;
        (void)acc;
        std::printf("  INFO: ModulatedAllpassDelay performance: %.2f ns/call (acc=%.4f)\n",
                    nsPerCall, static_cast<double>(acc));
        check(true, "ModulatedAllpassDelay performance benchmark (informational)");
    }
}

// ============================================================
// BitCrusher tests
// ============================================================
static void testBitCrusher() {
    std::printf("\n--- BitCrusher ---\n");

    // Test 1: 12-bit quantization — known values
    {
        BitCrusher bc;
        bc.init(12);

        check(bc.process(0.0f) == 0.0f,
              "BitCrusher 12-bit: 0.0 reproduced exactly");
        check(bc.process(1.0f) == 1.0f,
              "BitCrusher 12-bit: 1.0 reproduced exactly");
        check(bc.process(0.5f) == 0.5f,
              "BitCrusher 12-bit: 0.5 reproduced exactly");

        // Δ/2 above a level: round(1.5) = 2 → 2/2048
        const float inputAbove = (1.0f / 2048.0f) + (1.0f / 4096.0f);
        const float expected   = 2.0f / 2048.0f;
        check(approx(bc.process(inputAbove), expected, 1e-7f),
              "BitCrusher 12-bit: round-half-up behavior");
    }

    // Test 2: SQNR matches theory — 12-bit
    {
        BitCrusher bc;
        bc.init(12);
        const float phaseInc = 1000.0f / 48000.0f;

        constexpr int N = 48000;
        float phase = 0.0f;
        float sigPow = 0.0f, noisePow = 0.0f;

        for (int i = 0; i < N; ++i) {
            const float x = sinf(kTwoPi * phase);
            const float q = bc.process(x);
            sigPow   += x * x;
            noisePow += (q - x) * (q - x);
            phase += phaseInc;
            if (phase >= 1.0f) phase -= 1.0f;
        }

        const float sqnr_dB = 10.0f * log10f(sigPow / (noisePow + 1e-30f));
        const float theoretical = 6.02f * 12.0f + 1.76f;

        std::printf("    12-bit SQNR: measured=%.1f dB, theoretical=%.1f dB\n",
                    static_cast<double>(sqnr_dB), static_cast<double>(theoretical));
        check(fabsf(sqnr_dB - theoretical) < 3.0f,
              "BitCrusher 12-bit SQNR within 3dB of 74.0dB");
    }

    // Test 3: SQNR matches theory — 8-bit
    {
        BitCrusher bc;
        bc.init(8);
        const float phaseInc = 1000.0f / 48000.0f;

        constexpr int N = 48000;
        float phase = 0.0f;
        float sigPow = 0.0f, noisePow = 0.0f;

        for (int i = 0; i < N; ++i) {
            const float x = sinf(kTwoPi * phase);
            const float q = bc.process(x);
            sigPow   += x * x;
            noisePow += (q - x) * (q - x);
            phase += phaseInc;
            if (phase >= 1.0f) phase -= 1.0f;
        }

        const float sqnr_dB = 10.0f * log10f(sigPow / (noisePow + 1e-30f));
        const float theoretical = 6.02f * 8.0f + 1.76f;

        std::printf("    8-bit SQNR: measured=%.1f dB, theoretical=%.1f dB\n",
                    static_cast<double>(sqnr_dB), static_cast<double>(theoretical));
        check(fabsf(sqnr_dB - theoretical) < 3.0f,
              "BitCrusher 8-bit SQNR within 3dB of 49.9dB");
    }

    // Test 4: Noise shaping shifts noise to high frequencies
    {
        constexpr int N = 48000;

        BitCrusher bcRound, bcShaped;
        bcRound.init(12, BitCrusher::Mode::Round);
        bcShaped.init(12, BitCrusher::Mode::NoiseShape);

        const float phaseInc = 1000.0f / 48000.0f;
        float phase = 0.0f;

        float errDiffRound = 0.0f, errDiffShaped = 0.0f;
        float errAccRound  = 0.0f, errAccShaped  = 0.0f;
        float prevErrR = 0.0f, prevErrS = 0.0f;

        for (int i = 0; i < N; ++i) {
            const float x  = sinf(kTwoPi * phase);
            const float eR = bcRound.process(x) - x;
            const float eS = bcShaped.process(x) - x;

            errDiffRound  += (eR - prevErrR) * (eR - prevErrR);
            errDiffShaped += (eS - prevErrS) * (eS - prevErrS);
            errAccRound   += eR * eR;
            errAccShaped  += eS * eS;
            prevErrR = eR;
            prevErrS = eS;

            phase += phaseInc;
            if (phase >= 1.0f) phase -= 1.0f;
        }

        const float ratio = errAccShaped / (errAccRound + 1e-30f);
        std::printf("    noise shaping: diffRatio=%.2f, totalPowerRatio=%.2f\n",
                    static_cast<double>(errDiffShaped / (errDiffRound + 1e-10f)),
                    static_cast<double>(ratio));
        check(errDiffShaped > errDiffRound,
              "BitCrusher noise shaping increases HF noise content");
        check(ratio > 0.5f && ratio < 2.5f,
              "BitCrusher noise shaping total power within expected range");
    }

    // Test 5: Input clamping
    {
        BitCrusher bc;
        bc.init(12);

        check(bc.process(1.5f) == 1.0f,
              "BitCrusher clamps input > 1.0 to 1.0");
        check(bc.process(-1.5f) == -1.0f,
              "BitCrusher clamps input < -1.0 to -1.0");
        check(bc.process(100.0f) == 1.0f,
              "BitCrusher clamps large positive input");
        check(std::isfinite(bc.process(1e10f)),
              "BitCrusher: extremely large input stays finite");

        bc.init(12, BitCrusher::Mode::NoiseShape);
        check(bc.process(2.0f) == 1.0f,
              "BitCrusher (NoiseShape) clamps input > 1.0");
        check(std::isfinite(bc.process(-2.0f)),
              "BitCrusher (NoiseShape) -2.0 stays finite");
    }

    // Test 6: reset() clears error accumulator
    {
        BitCrusher bcR, bcS;
        bcR.init(12, BitCrusher::Mode::Round);
        bcS.init(12, BitCrusher::Mode::NoiseShape);

        for (int i = 0; i < 100; ++i)
            (void)bcS.process(0.3f);

        bcS.reset();

        const float testInput = 0.7f;
        const float outR = bcR.process(testInput);
        const float outS = bcS.process(testInput);

        check(outR == outS,
              "BitCrusher reset: NoiseShape first output matches Round");
    }

    // Test 7: setBits() changes step size
    {
        BitCrusher bc;
        bc.init(12);

        const float out12 = bc.process(0.51f);
        // 12-bit: round(0.51 * 2048) = round(1044.48) = 1044 → 1044/2048
        check(approx(out12, 1044.0f / 2048.0f, 1e-6f),
              "BitCrusher 12-bit quantization of 0.51");

        bc.setBits(8);
        const float out8 = bc.process(0.51f);
        // 8-bit: round(0.51 * 128) = round(65.28) = 65 → 65/128
        check(approx(out8, 65.0f / 128.0f, 1e-6f),
              "BitCrusher 8-bit quantization of 0.51 after setBits(8)");

        std::printf("    setBits: 12-bit=%.9f, 8-bit=%.9f\n",
                    static_cast<double>(out12), static_cast<double>(out8));
    }

    // Test 8: Performance benchmark (informational)
    {
        BitCrusher bc;
        bc.init(12);

        constexpr int ITERATIONS = 10000000;
        float acc = 0.001f;

        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < ITERATIONS; ++i) {
            acc = bc.process(acc * 0.9999f + 0.0001f);
        }
        auto t1 = std::chrono::high_resolution_clock::now();

        double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
        double nsPerCall = ns / ITERATIONS;
        (void)acc;
        std::printf("  INFO: BitCrusher performance: %.2f ns/call (target < 41 ns @ 720 MHz)\n",
                    nsPerCall);
        check(true, "BitCrusher performance benchmark (informational)");
    }
}

// ════════════════════════════════════════════════════════════════════════════
// FdnReverb<8> Tests
// ════════════════════════════════════════════════════════════════════════════

// Working buffer for FdnReverb<8> delay lines: 29,696 floats + headroom.
static float gTestWorkingBuffer[32768];

static bool nearlyEqual(float a, float b, float eps) {
    return fabsf(a - b) < eps;
}

static void test_fdnreverb8_t60_decay()
{
    FdnReverb<8> fdn;
    fdn.init(48000.0f, gTestWorkingBuffer, 32768);
    fdn.setT60(1.0f);
    fdn.setMix(1.0f);

    constexpr int SR = 48000;
    float outL, outR;

    // Inject impulse; measure early energy (first 48 samples = 1ms)
    fdn.process(1.0f, outL, outR);
    float earlyEnergy = outL * outL + outR * outR;
    for (int i = 1; i < 48; ++i) {
        fdn.process(0.0f, outL, outR);
        earlyEnergy += outL * outL + outR * outR;
    }

    // Run to 1 second minus the already-processed samples
    for (int i = 48; i < SR - 48; ++i)
        fdn.process(0.0f, outL, outR);

    // Measure late energy (last 48 samples)
    float lateEnergy = 0.0f;
    for (int i = 0; i < 48; ++i) {
        fdn.process(0.0f, outL, outR);
        lateEnergy += outL * outL + outR * outR;
    }

    // Ratio must be < 3.16e-6 (-55dB energy tolerance, T60=1s target is -60dB)
    const float ratio = lateEnergy / (earlyEnergy + 1e-30f);
    assert(ratio < 3.16e-6f &&
           "FdnReverb<8>: T60=1s decay ratio exceeds -55dB after 1 second");
    printf("  PASS: test_fdnreverb8_t60_decay  (energy ratio=%.2e, target < 3.16e-6)\n", (double)ratio);
    check(true, "FdnReverb<8> T60=1s energy decays >= 55dB in 1 second");
}

static void test_fdnreverb8_stability_sustained_input()
{
    FdnReverb<8> fdn;
    fdn.init(48000.0f, gTestWorkingBuffer, 32768);
    fdn.setT60(5.0f);
    fdn.setMix(1.0f);

    const float phaseInc = 440.0f / 48000.0f;
    float phase = 0.0f;
    float outL, outR;
    bool ok = true;

    for (int i = 0; i < 48000 * 10; ++i) {
        const float in = sinf(kTwoPi * phase);
        fdn.process(in, outL, outR);
        if (!std::isfinite(outL) || !std::isfinite(outR) ||
            fabsf(outL) >= 10.0f || fabsf(outR) >= 10.0f) {
            ok = false;
            break;
        }
        phase += phaseInc;
        if (phase >= 1.0f) phase -= 1.0f;
    }
    check(ok, "FdnReverb<8> stability: 10s sine, no runaway or NaN (T60=5s)");
}

static void test_fdnreverb8_dry_passthrough()
{
    FdnReverb<8> fdn;
    fdn.init(48000.0f, gTestWorkingBuffer, 32768);
    fdn.setMix(0.0f);

    float outL, outR;
    // Allow smoother to converge: 20ms time constant needs ~1200 samples for <0.1% residual
    for (int i = 0; i < 1200; ++i)
        fdn.process(0.0f, outL, outR);

    bool ok = true;
    for (int i = 0; i < 100; ++i) {
        fdn.process(0.5f, outL, outR);
        if (!nearlyEqual(outL, 0.5f, 0.01f) || !nearlyEqual(outR, 0.5f, 0.01f)) {
            ok = false;
            break;
        }
    }
    check(ok, "FdnReverb<8> dry passthrough: output = input at mix=0");
}

static void test_fdnreverb8_stereo_decorrelation()
{
    FdnReverb<8> fdn;
    fdn.init(48000.0f, gTestWorkingBuffer, 32768);
    fdn.setT60(3.0f);
    fdn.setMix(1.0f);

    float outL, outR;
    fdn.process(1.0f, outL, outR);  // impulse

    constexpr int MEAS = 4800;
    float bufL[MEAS], bufR[MEAS];
    for (int i = 0; i < MEAS; ++i)
        fdn.process(0.0f, bufL[i], bufR[i]);

    // Pearson correlation coefficient
    float sumL = 0.0f, sumR = 0.0f, sumLL = 0.0f, sumRR = 0.0f, sumLR = 0.0f;
    for (int i = 0; i < MEAS; ++i) {
        sumL  += bufL[i];
        sumR  += bufR[i];
        sumLL += bufL[i] * bufL[i];
        sumRR += bufR[i] * bufR[i];
        sumLR += bufL[i] * bufR[i];
    }
    const float meanL = sumL / MEAS, meanR = sumR / MEAS;
    const float cov   = sumLR / MEAS - meanL * meanR;
    const float stdL  = sqrtf(sumLL / MEAS - meanL * meanL + 1e-30f);
    const float stdR  = sqrtf(sumRR / MEAS - meanR * meanR + 1e-30f);
    const float r     = fabsf(cov / (stdL * stdR));

    std::printf("  INFO: stereo correlation r=%.3f (target < 0.9)\n", (double)r);
    check(r < 0.9f, "FdnReverb<8> stereo decorrelation: L/R correlation < 0.9");
}

static void test_fdnreverb8_reset_clears_state()
{
    FdnReverb<8> fdn;
    fdn.init(48000.0f, gTestWorkingBuffer, 32768);
    fdn.setT60(10.0f);
    fdn.setMix(1.0f);

    float outL, outR;
    // Build up reverb tail
    for (int i = 0; i < 24000; ++i)
        fdn.process(sinf(kTwoPi * (float)i * 440.0f / 48000.0f), outL, outR);

    fdn.reset();

    bool ok = true;
    for (int i = 0; i < 200; ++i) {
        fdn.process(0.0f, outL, outR);
        if (!nearlyEqual(outL, 0.0f, 1e-5f) || !nearlyEqual(outR, 0.0f, 1e-5f)) {
            ok = false;
            break;
        }
    }
    check(ok, "FdnReverb<8> reset: output is silent after reset()");
}

static void test_fdnreverb8_setT60_affects_decay()
{
    auto measureDecayEnergy = [](float t60, int measureAt) -> float {
        static float localBuf[32768];
        FdnReverb<8> fdn;
        fdn.init(48000.0f, localBuf, 32768);
        fdn.setT60(t60);
        fdn.setMix(1.0f);

        float outL, outR;
        fdn.process(1.0f, outL, outR);  // impulse

        float energy = 0.0f;
        for (int i = 1; i < measureAt; ++i) {
            fdn.process(0.0f, outL, outR);
            if (i >= measureAt - 48)
                energy += outL * outL + outR * outR;
        }
        return energy;
    };

    const float energyShort = measureDecayEnergy(0.5f, 24000);
    const float energyLong  = measureDecayEnergy(5.0f, 24000);

    std::printf("  INFO: T60 decay: short=%.2e, long=%.2e\n",
                (double)energyShort, (double)energyLong);
    check(energyShort < energyLong,
          "FdnReverb<8> setT60: shorter T60 produces faster decay");
}

static void test_fdnreverb8_performance()
{
    static float perfBuf[32768];
    FdnReverb<8> fdn;
    fdn.init(48000.0f, perfBuf, 32768);
    fdn.setT60(2.0f);

    constexpr int ITERATIONS = 100000;
    float outL = 0.0f, outR = 0.0f;
    float phase = 0.0f;

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < ITERATIONS; ++i) {
        fdn.process(sinf(kTwoPi * phase), outL, outR);
        phase += 440.0f / 48000.0f;
        if (phase >= 1.0f) phase -= 1.0f;
    }
    auto t1 = std::chrono::high_resolution_clock::now();

    double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
    double ns_per_call = ns / ITERATIONS;
    (void)outL; (void)outR;

    std::printf("  INFO: FdnReverb<8> performance: %.1f ns/call (outL=%.4f)\n",
                ns_per_call, (double)outL);
    check(true, "FdnReverb<8> performance benchmark (informational)");
}

static void testFdnReverb8() {
    std::printf("\n--- FdnReverb<8> ---\n");
    test_fdnreverb8_t60_decay();
    test_fdnreverb8_stability_sustained_input();
    test_fdnreverb8_dry_passthrough();
    test_fdnreverb8_stereo_decorrelation();
    test_fdnreverb8_reset_clears_state();
    test_fdnreverb8_setT60_affects_decay();
    test_fdnreverb8_performance();
    printf("  All FdnReverb<8> tests complete.\n");
}

int main() {
    std::printf("=== ReverbPrimitives Test Suite ===\n");
    testHadamard8();
    testModulatedAllpassDelay();
    testBitCrusher();
    testFdnReverb8();
    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);
    return (passedTests == totalTests) ? 0 : 1;
}
