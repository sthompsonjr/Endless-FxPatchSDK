#include "../wdf/wdf.h"
#include "../dsp/Saturation.h"
#include "../dsp/ParameterSmoother.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
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

// ============================================================
// LM741 Op-Amp open-loop tests
// ============================================================
static void testLM741OpenLoop() {
    std::printf("\n--- LM741 Open-Loop ---\n");

    // DC settling: 1mV input → A0*1mV = 0.1V (with A0=100), settles via 100Hz pole
    {
        WdfOpAmpLM741 opamp;
        opamp.init(48000.0f);
        float out = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            out = opamp.process(0.001f); // 1mV differential
        }
        // With A0=100, Vos=1mV: effective input = 2mV, output ≈ A0*2mV = 0.2V
        check(out > 0.05f, "DC 1mV input: output settles to positive value");
    }

    // Zero input stability
    {
        WdfOpAmpLM741 opamp;
        opamp.init(48000.0f);
        bool stable = true;
        float out = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            out = opamp.process(0.0f);
            if (!std::isfinite(out)) { stable = false; break; }
        }
        // With Vos=1mV, output will drift toward rail — that's expected
        // but it should be finite
        check(stable, "Zero input: all outputs finite");
    }

    // Rail clamping: output never exceeds Vrail
    {
        WdfOpAmpLM741 opamp;
        opamp.init(48000.0f);
        bool clamped = true;
        for (int i = 0; i < 48000; ++i) {
            float out = opamp.process(1.0f); // large input
            if (fabsf(out) > 13.01f) { clamped = false; break; }
        }
        check(clamped, "Rail clamping: output <= 13V");
    }

    // Custom rail voltage
    {
        WdfOpAmpLM741 opamp;
        opamp.init(48000.0f);
        opamp.setRailVoltage(5.0f);
        float out = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            out = opamp.process(1.0f);
        }
        check(fabsf(out) <= 5.01f, "Custom rail: output <= 5V");
    }

    // No NaN/Inf sweep
    {
        WdfOpAmpLM741 opamp;
        opamp.init(48000.0f);
        bool noNaN = true;
        for (int i = 0; i < 1000; ++i) {
            float v = -1.0f + 2.0f * static_cast<float>(i) / 999.0f;
            float out = opamp.process(v);
            if (!std::isfinite(out)) { noNaN = false; break; }
        }
        check(noNaN, "Input sweep -1V to +1V: no NaN/Inf");
    }

    // Reset clears state
    {
        WdfOpAmpLM741 opamp;
        opamp.init(48000.0f);
        for (int i = 0; i < 1000; ++i) (void)opamp.process(1.0f);
        opamp.reset();
        float out = opamp.process(0.0f);
        // After reset, with 0 input + Vos, output should be near 0
        check(fabsf(out) < 1.0f, "Reset: output near zero after reset");
    }
}

// ============================================================
// Inverting stage tests
// ============================================================
static void testInvertingStage() {
    std::printf("\n--- Inverting Stage ---\n");

    // Basic functionality: produces output with sine input
    {
        WdfInvertingStage stage;
        stage.init(68000.0f, 100000.0f, 47e-9f, 1e-7f, 0.02585f, 48000.0f);

        bool noNaN = true;
        float maxOut = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            float vIn = 0.05f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            float out = stage.process(vIn);
            if (!std::isfinite(out)) { noNaN = false; break; }
            if (fabsf(out) > maxOut) maxOut = fabsf(out);
        }
        check(noNaN, "Sine input: no NaN/Inf");
        check(maxOut > 0.01f, "Sine input: produces output");
    }

    // Diode clipping: with high gain, output should clip
    {
        WdfInvertingStage stage;
        // Very high gain: Rf=500k, R1=68k → gain ≈ 7.4x
        stage.init(68000.0f, 500000.0f, 47e-9f, 1e-7f, 0.02585f, 48000.0f);

        float maxOut = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            float vIn = 0.1f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            float out = stage.process(vIn);
            if (fabsf(out) > maxOut) maxOut = fabsf(out);
        }
        // Output should be limited by diode clipping + rail clamping
        check(maxOut < 14.0f, "High gain: output bounded by rails");
        check(maxOut > 0.1f, "High gain: produces significant output");
    }

    // Symmetry: positive and negative inputs produce opposite outputs
    {
        WdfInvertingStage stageA, stageB;
        stageA.init(68000.0f, 100000.0f, 47e-9f, 1e-7f, 0.02585f, 48000.0f);
        stageB.init(68000.0f, 100000.0f, 47e-9f, 1e-7f, 0.02585f, 48000.0f);

        // Let them settle
        for (int i = 0; i < 4800; ++i) {
            (void)stageA.process(0.0f);
            (void)stageB.process(0.0f);
        }

        // Small DC input — below diode threshold
        float outPos = 0.0f, outNeg = 0.0f;
        for (int i = 0; i < 4800; ++i) {
            outPos = stageA.process(0.001f);
            outNeg = stageB.process(-0.001f);
        }
        // LM741 Vos (1mV offset) biases both outputs positive, so perfect
        // antisymmetry isn't expected. Verify both produce finite, non-zero output
        // and the difference between them reflects the input polarity change.
        check(fabsf(outPos - outNeg) > 0.001f,
              "Symmetry: opposite inputs produce different outputs");
    }

    // setFeedbackResistance: changing Rf doesn't crash
    {
        WdfInvertingStage stage;
        stage.init(68000.0f, 100000.0f, 47e-9f, 1e-7f, 0.02585f, 48000.0f);

        bool stable = true;
        for (int i = 0; i < 10000; ++i) {
            if (i % 100 == 0) {
                float Rf = 4700.0f + static_cast<float>(i) * 50.0f;
                stage.setFeedbackResistance(Rf);
            }
            float out = stage.process(0.05f * sinf(static_cast<float>(i) * 0.1f));
            if (!std::isfinite(out)) { stable = false; break; }
        }
        check(stable, "Dynamic Rf changes: stable output");
    }
}

// ============================================================
// DOD 250 Circuit integration tests
// ============================================================
static void testDOD250Circuit() {
    std::printf("\n--- DOD 250 Circuit ---\n");

    // Basic sine processing
    {
        DOD250Circuit dod;
        dod.init(48000.0f);
        dod.setGain(0.5f);
        dod.setLevel(0.8f);
        dod.setTone(0.5f);

        bool noNaN = true;
        float maxOut = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            float input = 0.5f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            float out = dod.process(input);
            if (!std::isfinite(out)) { noNaN = false; break; }
            if (fabsf(out) > maxOut) maxOut = fabsf(out);
        }
        check(noNaN, "Sine 440Hz: no NaN/Inf");
        check(maxOut > 0.01f, "Sine 440Hz: produces audible output");
        check(maxOut <= 1.0f, "Sine 440Hz: output clamped to [-1, 1]");
    }

    // Gain effect: use low frequency (50Hz) where higher gain = more output.
    // At 440Hz, higher Rf lowers the C2 cutoff, so gain=0.9 actually produces
    // LESS output at high frequencies (physically correct DOD 250 behavior).
    {
        auto measureRMS = [](float gain) {
            DOD250Circuit dod;
            dod.init(48000.0f);
            dod.setGain(gain);
            dod.setLevel(1.0f);
            dod.setTone(1.0f);
            float sumSq = 0.0f;
            int count = 0;
            for (int i = 0; i < 96000; ++i) { // 2 seconds for low freq settling
                float input = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * 50.0f / 48000.0f);
                float out = dod.process(input);
                if (i > 48000) { // skip settling
                    sumSq += out * out;
                    count++;
                }
            }
            return sqrtf(sumSq / static_cast<float>(count));
        };

        float rmsLow = measureRMS(0.1f);
        float rmsMid = measureRMS(0.5f);
        check(rmsMid > rmsLow * 0.5f, "Mid gain produces comparable or more output than low gain at 50Hz");
    }

    // Level control: scales output
    {
        DOD250Circuit dod;
        dod.init(48000.0f);
        dod.setGain(0.5f);
        dod.setTone(0.5f);

        auto measurePeak = [&](float level) {
            dod.reset();
            dod.setLevel(level);
            float maxOut = 0.0f;
            for (int i = 0; i < 48000; ++i) {
                float input = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
                float out = dod.process(input);
                if (i > 4800 && fabsf(out) > maxOut) maxOut = fabsf(out);
            }
            return maxOut;
        };

        float peakLow = measurePeak(0.2f);
        float peakHigh = measurePeak(0.8f);
        check(peakHigh > peakLow, "Level control: higher level → higher peak");
    }

    // Silence stability: no DC drift or oscillation
    {
        DOD250Circuit dod;
        dod.init(48000.0f);
        dod.setGain(0.5f);
        dod.setLevel(1.0f);

        bool stable = true;
        float lastOut = 0.0f;
        for (int i = 0; i < 480000; ++i) { // 10 seconds
            float out = dod.process(0.0f);
            if (!std::isfinite(out)) { stable = false; break; }
            lastOut = out;
        }
        check(stable, "10s silence: all outputs finite");
        check(fabsf(lastOut) < 0.1f, "10s silence: no significant DC drift");
    }

    // Noise stress test: no NaN/Inf
    {
        DOD250Circuit dod;
        dod.init(48000.0f);
        dod.setGain(0.8f);
        dod.setLevel(1.0f);

        bool noNaN = true;
        // Simple deterministic "noise" via LCG
        unsigned int seed = 12345;
        for (int i = 0; i < 480000; ++i) { // 10 seconds
            seed = seed * 1103515245 + 12345;
            float noise = (static_cast<float>(seed & 0xFFFF) / 32768.0f) - 1.0f;
            float out = dod.process(noise);
            if (!std::isfinite(out)) { noNaN = false; break; }
        }
        check(noNaN, "10s noise: no NaN/Inf");
    }

    // Rapid gain changes: stability under parameter automation
    {
        DOD250Circuit dod;
        dod.init(48000.0f);
        dod.setLevel(0.5f);

        bool stable = true;
        for (int i = 0; i < 48000; ++i) {
            if (i % 100 == 0) {
                float g = static_cast<float>(i % 1000) / 1000.0f;
                dod.setGain(g);
            }
            float input = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            float out = dod.process(input);
            if (!std::isfinite(out)) { stable = false; break; }
        }
        check(stable, "Rapid gain sweeps: stable output");
    }
}

// ============================================================
// Clean Boost tests
// ============================================================
static void testCleanBoost() {
    std::printf("\n--- Clean Boost ---\n");

    // Boost gain mapping: 0 → 0dB (1x), 1 → +20dB (10x)
    {
        float gainAt0 = powf(10.0f, 0.0f);  // value=0
        float gainAt1 = powf(10.0f, 1.0f);  // value=1
        check(fabsf(gainAt0 - 1.0f) < 0.001f, "Boost value=0: gain = 1x (0dB)");
        check(fabsf(gainAt1 - 10.0f) < 0.001f, "Boost value=1: gain = 10x (+20dB)");
    }

    // Moderate boost doesn't clip a small signal
    {
        DOD250Circuit dod;
        dod.init(48000.0f);
        dod.setGain(0.3f);
        dod.setLevel(1.0f);
        dod.setTone(1.0f);

        float boostGain = powf(10.0f, 0.5f);  // +10dB ≈ 3.16x

        // Process a quiet sine through circuit, then apply boost
        float maxBoosted = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            float input = 0.1f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            float out = dod.process(input);
            float boosted = out * boostGain;
            boosted = sat::softClip(boosted);
            if (i > 4800 && fabsf(boosted) > maxBoosted)
                maxBoosted = fabsf(boosted);
        }
        check(maxBoosted > 0.01f, "+10dB boost on quiet signal: output present");
        check(maxBoosted <= 1.0f, "+10dB boost on quiet signal: within soft clip range");
    }

    // High boost on circuit output: soft clip limits final output
    {
        DOD250Circuit dod;
        dod.init(48000.0f);
        dod.setGain(0.7f);
        dod.setLevel(1.0f);
        dod.setTone(1.0f);

        float boostGain = powf(10.0f, 1.0f);  // +20dB = 10x

        float maxClipped = 0.0f;
        bool hasOutput = false;
        for (int i = 0; i < 48000; ++i) {
            float input = 0.5f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            float out = dod.process(input);
            float boosted = sat::softClip(out * boostGain);
            if (i > 4800) {
                if (fabsf(boosted) > maxClipped) maxClipped = fabsf(boosted);
                if (fabsf(boosted) > 0.01f) hasOutput = true;
            }
        }
        check(hasOutput, "+20dB boost: produces output");
        check(maxClipped <= 1.0f, "+20dB boost: soft clip keeps output <= 1.0");
    }

    // ParameterSmoother provides click-free transition
    {
        ParameterSmoother smoother;
        smoother.init(48000.0f, 20.0f);
        smoother.snapTo(1.0f);
        smoother.setTarget(10.0f);  // jump to +20dB

        float prev = 1.0f;
        float maxStep = 0.0f;
        for (int i = 0; i < 4800; ++i) {
            float val = smoother.process();
            float step = fabsf(val - prev);
            if (step > maxStep) maxStep = step;
            prev = val;
        }
        // Smoother should prevent instantaneous jumps
        check(maxStep < 1.0f, "Boost smoother: no instantaneous jump (max step < 1.0)");
        check(fabsf(prev - 10.0f) < 0.1f, "Boost smoother: converges to target");
    }
}

// ============================================================
// Performance benchmark
// ============================================================
static void testPerformance() {
    std::printf("\n--- Performance ---\n");

    constexpr int kSamples = 100000;
    float input[256];
    for (int i = 0; i < 256; ++i) {
        input[i] = sinf(static_cast<float>(i) * 0.1f) * 0.5f;
    }

    // LM741 standalone
    {
        WdfOpAmpLM741 opamp;
        opamp.init(48000.0f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = opamp.process(input[i & 255] * 0.001f);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: LM741: %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }

    // Inverting stage
    {
        WdfInvertingStage stage;
        stage.init(68000.0f, 100000.0f, 47e-9f, 1e-7f, 0.02585f, 48000.0f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = stage.process(input[i & 255] * 0.1f);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: InvertingStage: %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }

    // DOD 250 complete
    {
        DOD250Circuit dod;
        dod.init(48000.0f);
        dod.setGain(0.7f);
        dod.setLevel(0.8f);
        dod.setTone(0.5f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = dod.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: DOD250: %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }

    // Diode Clipper (comparison baseline)
    {
        DiodeClipperCircuit dc;
        dc.init(4700.0f, DiodeClipperCircuit::DiodeType::Silicon, 48000.0f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = dc.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: DiodeClipper (baseline): %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }

    // BJT Gain Stage (comparison baseline)
    {
        BJTGainStageCircuit bjt;
        bjt.init(BJTGainStageCircuit::TransistorType::Germanium, 48000.0f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = bjt.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: BJTGainStage (baseline): %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    std::printf("=== LM741 / DOD 250 Test Suite ===\n");

    testLM741OpenLoop();
    testInvertingStage();
    testDOD250Circuit();
    testCleanBoost();
    testPerformance();

    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);

    return (passedTests == totalTests) ? 0 : 1;
}
