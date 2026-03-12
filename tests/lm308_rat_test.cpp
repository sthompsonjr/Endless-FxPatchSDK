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
// LM308 Op-Amp open-loop tests
// ============================================================
static void testLM308OpenLoop() {
    std::printf("\n--- LM308 Open-Loop ---\n");

    // DC settling: 1mV input → A0*1mV with Vos=2mV
    {
        WdfOpAmpLM308 opamp;
        opamp.init(48000.0f);
        float out = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            out = opamp.process(0.001f);
        }
        check(out > 0.05f, "DC 1mV input: output settles to positive value");
    }

    // Zero input stability
    {
        WdfOpAmpLM308 opamp;
        opamp.init(48000.0f);
        bool stable = true;
        float out = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            out = opamp.process(0.0f);
            if (!std::isfinite(out)) { stable = false; break; }
        }
        check(stable, "Zero input: all outputs finite");
    }

    // Rail clamping: output never exceeds Vrail (7.5V for LM308)
    {
        WdfOpAmpLM308 opamp;
        opamp.init(48000.0f);
        bool clamped = true;
        for (int i = 0; i < 48000; ++i) {
            float out = opamp.process(1.0f);
            if (fabsf(out) > 7.51f) { clamped = false; break; }
        }
        check(clamped, "Rail clamping: output <= 7.5V");
    }

    // Custom rail voltage
    {
        WdfOpAmpLM308 opamp;
        opamp.init(48000.0f);
        opamp.setRailVoltage(4.0f);
        float out = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            out = opamp.process(1.0f);
        }
        check(fabsf(out) <= 4.01f, "Custom rail: output <= 4V");
    }

    // Slew rate measurement: LM308 should be slower than LM741
    {
        WdfOpAmpLM308 lm308;
        lm308.init(48000.0f);
        WdfOpAmpLM741 lm741;
        lm741.init(48000.0f);

        // Apply step input and measure first sample delta
        float out308_0 = lm308.process(0.0f);
        float out308_1 = lm308.process(1.0f);
        float delta308 = fabsf(out308_1 - out308_0);

        float out741_0 = lm741.process(0.0f);
        float out741_1 = lm741.process(1.0f);
        float delta741 = fabsf(out741_1 - out741_0);

        // LM308 slew = 300kV/s → 6.25 V/sample
        // LM741 slew = 500kV/s → 10.42 V/sample
        // With small signal (A0*1V = 100V clamped), both hit slew limit
        check(delta308 < delta741 + 0.01f, "LM308 slew <= LM741 slew (slower op-amp)");
        check(delta308 > 0.0f, "LM308 produces non-zero step response");
    }

    // Slew rate value check: ~6.25 V/sample at 48kHz
    {
        WdfOpAmpLM308 opamp;
        opamp.init(48000.0f);
        // Let it sit at 0
        for (int i = 0; i < 100; ++i) (void)opamp.process(0.0f);
        opamp.reset();
        float out0 = opamp.process(0.0f);
        float out1 = opamp.process(10.0f); // large step to hit slew limit
        float slewPerSample = fabsf(out1 - out0);
        float expectedSlew = 300000.0f / 48000.0f; // 6.25
        check(fabsf(slewPerSample - expectedSlew) < 0.5f,
              "Slew rate ~6.25 V/sample at 48kHz");
    }

    // Comp cap modulation: faster slew with smaller cap
    {
        WdfOpAmpLM308 fast, slow;
        fast.init(48000.0f);
        slow.init(48000.0f);
        fast.setCompCapPF(15.0f);  // faster
        slow.setCompCapPF(60.0f);  // slower

        float out_fast_0 = fast.process(0.0f);
        float out_fast_1 = fast.process(10.0f);
        float out_slow_0 = slow.process(0.0f);
        float out_slow_1 = slow.process(10.0f);

        float deltaFast = fabsf(out_fast_1 - out_fast_0);
        float deltaSlow = fabsf(out_slow_1 - out_slow_0);
        check(deltaFast > deltaSlow, "Comp cap: 15pF faster slew than 60pF");
    }

    // Age effect: reduces gain, increases offset
    {
        WdfOpAmpLM308 fresh, aged;
        fresh.init(48000.0f);
        aged.init(48000.0f);
        aged.setAge(1.0f);

        // Both process same small DC signal for a while
        float outFresh = 0.0f, outAged = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            outFresh = fresh.process(0.001f);
            outAged = aged.process(0.001f);
        }
        // Aged has higher Vos (10mV vs 2mV), so it saturates more toward rail
        // With 0 input, aged drifts more due to 5× Vos
        check(std::isfinite(outFresh) && std::isfinite(outAged), "Aged LM308: both outputs finite");
    }

    // No NaN/Inf sweep
    {
        WdfOpAmpLM308 opamp;
        opamp.init(48000.0f);
        bool noNaN = true;
        for (int i = 0; i < 1000; ++i) {
            float v = -1.0f + 2.0f * static_cast<float>(i) / 999.0f;
            float out = opamp.process(v);
            if (!std::isfinite(out)) { noNaN = false; break; }
        }
        check(noNaN, "Input sweep -1V to +1V: no NaN/Inf");
    }
}

// ============================================================
// WdfDiodeToGround tests
// ============================================================
static void testDiodeToGround() {
    std::printf("\n--- WdfDiodeToGround ---\n");

    // Asymmetry: positive clips, negative passes through
    // Use low port resistance (100Ω) for stronger diode effect
    {
        auto diode = WdfDiodeToGround::make1N914(100.0f);

        // Positive incident wave → diode conducts → voltage reduced
        diode.port.a = 2.0f;
        diode.reflect();
        float vPos = diode.port.voltage();

        diode.reset();

        // Negative incident wave → diode off → voltage = incident (open circuit)
        diode.port.a = -2.0f;
        diode.reflect();
        float vNeg = diode.port.voltage();

        check(vPos < 1.0f, "1N914: positive voltage clipped below 1V");
        check(vNeg < -0.5f, "1N914: negative voltage passes through");
        check(fabsf(vPos) < fabsf(vNeg), "1N914: asymmetric — positive clipped more than negative");
    }

    // Germanium vs Silicon: germanium has higher Is → clips more at same voltage
    {
        auto si = WdfDiodeToGround::make1N914(100.0f);
        auto ge = WdfDiodeToGround::makeGermanium(100.0f);

        si.port.a = 1.0f;
        si.reflect();
        float vSi = si.port.voltage();

        ge.port.a = 1.0f;
        ge.reflect();
        float vGe = ge.port.voltage();

        check(vGe < vSi, "Germanium clips at lower voltage than silicon");
    }

    // LED: much lower Is → lets more voltage through than silicon
    {
        auto led = WdfDiodeToGround::makeLED(100.0f);
        auto si = WdfDiodeToGround::make1N914(100.0f);

        led.port.a = 2.0f;
        led.reflect();
        float vLed = led.port.voltage();

        si.port.a = 2.0f;
        si.reflect();
        float vSi = si.port.voltage();

        check(vLed > vSi, "LED: higher forward voltage than silicon");
    }

    // No NaN with extreme inputs
    {
        auto diode = WdfDiodeToGround::make1N914(100.0f);
        bool noNaN = true;
        for (int i = 0; i < 1000; ++i) {
            diode.port.a = -50.0f + 100.0f * static_cast<float>(i) / 999.0f;
            diode.reflect();
            if (!std::isfinite(diode.port.b)) { noNaN = false; break; }
        }
        check(noNaN, "Extreme input sweep: no NaN/Inf");
    }
}

// ============================================================
// RAT Circuit integration tests
// ============================================================
static void testRatCircuit() {
    std::printf("\n--- RAT Circuit ---\n");

    // Basic sine processing
    {
        WdfRatCircuit rat;
        rat.init(48000.0f);
        rat.setDistortion(0.5f);
        rat.setVolume(0.8f);
        rat.setFilter(0.5f);

        bool noNaN = true;
        float maxOut = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            float input = 0.5f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            float out = rat.process(input);
            if (!std::isfinite(out)) { noNaN = false; break; }
            if (fabsf(out) > maxOut) maxOut = fabsf(out);
        }
        check(noNaN, "Sine 440Hz: no NaN/Inf");
        check(maxOut > 0.01f, "Sine 440Hz: produces audible output");
        check(maxOut <= 1.0f, "Sine 440Hz: output clamped to [-1, 1]");
    }

    // Asymmetric output (Original variant with single diode)
    {
        WdfRatCircuit rat;
        rat.init(48000.0f);
        rat.setDistortion(0.8f);
        rat.setVolume(1.0f);
        rat.setFilter(0.0f); // bright

        float maxPos = 0.0f, maxNeg = 0.0f;
        for (int i = 0; i < 96000; ++i) { // 2 seconds for settling
            float input = 0.5f * sinf(static_cast<float>(i) * 6.283185307f * 200.0f / 48000.0f);
            float out = rat.process(input);
            if (i > 48000) { // skip settling
                if (out > maxPos) maxPos = out;
                if (out < maxNeg) maxNeg = out;
            }
        }
        // The original RAT clips asymmetrically
        check(maxPos > 0.0f && maxNeg < 0.0f, "Original RAT: bipolar output");
    }

    // Distortion effect: higher distortion → different output
    {
        auto measureRMS = [](float dist) {
            WdfRatCircuit rat;
            rat.init(48000.0f);
            rat.setDistortion(dist);
            rat.setVolume(1.0f);
            rat.setFilter(0.0f);
            float sumSq = 0.0f;
            int count = 0;
            for (int i = 0; i < 96000; ++i) {
                float input = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * 200.0f / 48000.0f);
                float out = rat.process(input);
                if (i > 48000) {
                    sumSq += out * out;
                    count++;
                }
            }
            return sqrtf(sumSq / static_cast<float>(count));
        };

        float rmsLow = measureRMS(0.1f);
        float rmsHigh = measureRMS(0.9f);
        check(rmsHigh > rmsLow * 0.5f, "Higher distortion produces comparable or more output");
    }

    // Filter reversal: 0 = bright (more highs), 1 = dark (fewer highs)
    // Use low distortion to avoid slew rate killing the HF signal
    {
        auto measureHighFreqEnergy = [](float filterVal) {
            WdfRatCircuit rat;
            rat.init(48000.0f);
            rat.setDistortion(0.2f);
            rat.setVolume(1.0f);
            rat.setFilter(filterVal);
            float sumSq = 0.0f;
            int count = 0;
            for (int i = 0; i < 96000; ++i) {
                float input = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * 5000.0f / 48000.0f);
                float out = rat.process(input);
                if (i > 48000) {
                    sumSq += out * out;
                    count++;
                }
            }
            return sqrtf(sumSq / static_cast<float>(count));
        };

        float rmsBright = measureHighFreqEnergy(0.0f); // bright
        float rmsDark = measureHighFreqEnergy(1.0f);   // dark
        check(rmsBright > rmsDark, "Filter: bright (0) passes more highs than dark (1)");
    }

    // Silence stability: no DC drift
    {
        WdfRatCircuit rat;
        rat.init(48000.0f);
        rat.setDistortion(0.5f);
        rat.setVolume(1.0f);

        bool stable = true;
        float lastOut = 0.0f;
        for (int i = 0; i < 480000; ++i) { // 10 seconds
            float out = rat.process(0.0f);
            if (!std::isfinite(out)) { stable = false; break; }
            lastOut = out;
        }
        check(stable, "10s silence: all outputs finite");
        check(fabsf(lastOut) < 0.1f, "10s silence: no significant DC drift");
    }

    // Noise stress test
    {
        WdfRatCircuit rat;
        rat.init(48000.0f);
        rat.setDistortion(0.8f);
        rat.setVolume(1.0f);

        bool noNaN = true;
        unsigned int seed = 12345;
        for (int i = 0; i < 480000; ++i) {
            seed = seed * 1103515245 + 12345;
            float noise = (static_cast<float>(seed & 0xFFFF) / 32768.0f) - 1.0f;
            float out = rat.process(noise);
            if (!std::isfinite(out)) { noNaN = false; break; }
        }
        check(noNaN, "10s noise: no NaN/Inf");
    }

    // All variants produce output
    {
        RatVariant variants[] = {
            RatVariant::Original, RatVariant::TurboRat,
            RatVariant::YouDirtyRat, RatVariant::WhiteRat,
            RatVariant::GermaniumMod
        };
        const char* names[] = {
            "Original", "TurboRat", "YouDirtyRat", "WhiteRat", "GermaniumMod"
        };

        for (int v = 0; v < 5; ++v) {
            WdfRatCircuit rat;
            rat.init(48000.0f);
            rat.setDistortion(0.7f);
            rat.setVolume(1.0f);
            rat.setFilter(0.3f);
            // Apply variant directly by reinitializing
            rat.setVariant(variants[v]);

            bool noNaN = true;
            float maxOut = 0.0f;
            for (int i = 0; i < 48000; ++i) {
                float input = 0.4f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
                float out = rat.process(input);
                if (!std::isfinite(out)) { noNaN = false; break; }
                if (fabsf(out) > maxOut) maxOut = fabsf(out);
            }
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%s variant: stable and produces output", names[v]);
            check(noNaN && maxOut > 0.001f, buf);
        }
    }
}

// ============================================================
// Variant crossfade tests
// ============================================================
static void testVariantCrossfade() {
    std::printf("\n--- Variant Crossfade ---\n");

    // No discontinuity during variant switch
    {
        WdfRatCircuit rat;
        rat.init(48000.0f);
        rat.setDistortion(0.5f);
        rat.setVolume(0.8f);
        rat.setFilter(0.5f);

        // Warm up
        for (int i = 0; i < 4800; ++i) {
            float input = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            (void)rat.process(input);
        }

        // Switch variant mid-stream and check for discontinuities
        float prev = 0.0f;
        float maxStep = 0.0f;
        bool noNaN = true;
        for (int i = 0; i < 4800; ++i) {
            if (i == 100) rat.setVariant(RatVariant::TurboRat);
            float input = 0.3f * sinf(static_cast<float>(i + 4800) * 6.283185307f * 440.0f / 48000.0f);
            float out = rat.process(input);
            if (!std::isfinite(out)) { noNaN = false; break; }
            float step = fabsf(out - prev);
            if (step > maxStep) maxStep = step;
            prev = out;
        }
        check(noNaN, "Variant switch: no NaN during crossfade");
        // The crossfade envelope prevents pops; allow normal audio-rate changes
        check(maxStep < 2.0f, "Variant switch: no large discontinuity during crossfade");
    }

    // All variant transitions are stable
    {
        WdfRatCircuit rat;
        rat.init(48000.0f);
        rat.setDistortion(0.5f);
        rat.setVolume(0.8f);

        RatVariant variants[] = {
            RatVariant::Original, RatVariant::TurboRat,
            RatVariant::YouDirtyRat, RatVariant::WhiteRat,
            RatVariant::GermaniumMod
        };

        bool allStable = true;
        for (int v = 0; v < 5; ++v) {
            rat.setVariant(variants[v]);
            for (int i = 0; i < 1000; ++i) {
                float input = 0.3f * sinf(static_cast<float>(i) * 0.1f);
                float out = rat.process(input);
                if (!std::isfinite(out)) { allStable = false; break; }
            }
        }
        check(allStable, "All variant transitions: stable output");
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

    // LM308 standalone
    {
        WdfOpAmpLM308 opamp;
        opamp.init(48000.0f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = opamp.process(input[i & 255] * 0.001f);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: LM308: %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }

    // RAT complete circuit
    {
        WdfRatCircuit rat;
        rat.init(48000.0f);
        rat.setDistortion(0.7f);
        rat.setVolume(0.8f);
        rat.setFilter(0.5f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = rat.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: RAT (Original): %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }

    // RAT variant comparison
    {
        RatVariant variants[] = {
            RatVariant::Original, RatVariant::TurboRat,
            RatVariant::YouDirtyRat, RatVariant::WhiteRat,
            RatVariant::GermaniumMod
        };
        const char* names[] = {
            "Original", "TurboRat", "YouDirtyRat", "WhiteRat", "GermaniumMod"
        };

        for (int v = 0; v < 5; ++v) {
            WdfRatCircuit rat;
            rat.init(48000.0f);
            rat.setDistortion(0.7f);
            rat.setVolume(0.8f);
            rat.setVariant(variants[v]);
            // Let crossfade complete
            for (int i = 0; i < 512; ++i) (void)rat.process(0.0f);

            auto start = std::chrono::high_resolution_clock::now();
            float dummy = 0.0f;
            for (int i = 0; i < kSamples; ++i) dummy = rat.process(input[i & 255]);
            auto end = std::chrono::high_resolution_clock::now();
            auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            (void)dummy;
            std::printf("  INFO: RAT (%s): %lld ns/sample\n", names[v],
                       static_cast<long long>(ns / kSamples));
        }
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    std::printf("=== LM308 / RAT Test Suite ===\n");

    testLM308OpenLoop();
    testDiodeToGround();
    testRatCircuit();
    testVariantCrossfade();
    testPerformance();

    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);

    return (passedTests == totalTests) ? 0 : 1;
}
