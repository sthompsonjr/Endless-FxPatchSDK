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
// JRC4558 open-loop tests
// ============================================================
static void testJRC4558OpenLoop() {
    std::printf("\n--- JRC4558 Open-Loop ---\n");

    // Dominant pole faster than LM741 (small-signal, low amplitude)
    // JRC4558 fp=300 Hz vs LM741 fp=100 Hz → JRC4558 settles faster
    // Use small input to stay in small-signal (linear) regime
    {
        WdfOpAmpLM741 lm741;
        lm741.init(48000.0f);
        WdfOpAmpJRC4558 jrc;
        jrc.init(48000.0f);

        // Ramp input 0→0.001V (keep slew well below limit at tiny amplitudes)
        float jrcOut50 = 0.0f, lmOut50 = 0.0f;
        for (int i = 0; i < 1000; ++i) {
            float vin = 0.001f * static_cast<float>(i) / 999.0f;
            float jrcOut = jrc.process(vin);
            float lmOut  = lm741.process(vin);
            if (i == 50) { jrcOut50 = jrcOut; lmOut50 = lmOut; }
        }
        // At sample 50, JRC4558 should have higher output (faster pole)
        check(jrcOut50 > lmOut50,
              "Dominant pole: JRC4558 responds faster than LM741 at sample 50");
    }

    // Slew rate: JRC4558 SR = 322,000 V/s < LM741 SR = 500,000 V/s.
    // At large-signal transitions the output delta per sample is slew-limited.
    // Paradox: JRC4558 has higher GBW (small-signal bandwidth) but slower SR
    // (large-signal rate limit). At low input levels JRC4558 tracks faster;
    // at large-signal steps LM741 slews faster (322k < 500k V/s).
    // Use a ±10V square wave to force the slew-limited regime.
    {
        WdfOpAmpJRC4558 jrc; jrc.init(48000.0f);
        WdfOpAmpLM741   lm;  lm.init(48000.0f);

        float jrcMaxSlew = 0.0f, lmMaxSlew = 0.0f;
        float jrcPrev    = 0.0f, lmPrev    = 0.0f;

        for (int i = 0; i < 96000; ++i) {
            float in = (i % 4800 < 2400) ? 10.0f : -10.0f;
            float jo = jrc.process(in);
            float lo = lm.process(in);
            float jd = fabsf(jo - jrcPrev);
            float ld = fabsf(lo - lmPrev);
            if (jd > jrcMaxSlew) jrcMaxSlew = jd;
            if (ld > lmMaxSlew)  lmMaxSlew  = ld;
            jrcPrev = jo; lmPrev = lo;
        }

        // JRC4558 peak output slew < LM741 peak output slew (322k vs 500k V/s)
        check(jrcMaxSlew < lmMaxSlew,
              "Slew rate: JRC4558 peak output slew < LM741 (large-signal regime, 322k vs 500k V/s)");
    }

    // Rail clamping: output never exceeds ±13V
    {
        WdfOpAmpJRC4558 jrc;
        jrc.init(48000.0f);
        bool clamped = true;
        for (int i = 0; i < 48000; ++i) {
            float out = jrc.process(1.0f);
            if (fabsf(out) > 13.01f) { clamped = false; break; }
        }
        check(clamped, "Rail clamping: output <= 13V");
    }

    // Reset clears state
    {
        WdfOpAmpJRC4558 jrc;
        jrc.init(48000.0f);
        for (int i = 0; i < 1000; ++i) (void)jrc.process(1.0f);
        jrc.reset();
        float out = jrc.process(0.0f);
        check(fabsf(out) < 1.0f, "Reset: output near zero after reset");
    }

    // Creative methods: setAge
    {
        WdfOpAmpJRC4558 jrc;
        jrc.init(48000.0f);
        // Run fresh
        float outFresh = 0.0f;
        for (int i = 0; i < 1000; ++i) outFresh = jrc.process(0.05f);

        jrc.reset();
        jrc.setAge(0.8f);
        float outAged = 0.0f;
        for (int i = 0; i < 1000; ++i) outAged = jrc.process(0.05f);

        check(std::isfinite(outAged), "setAge: no NaN/Inf");
        check(fabsf(outAged - outFresh) > 0.0001f, "setAge: changes output character");
    }

    // Creative methods: setSlew
    {
        WdfOpAmpJRC4558 jrc;
        jrc.init(48000.0f);
        jrc.setSlew(0.3f);  // very slow slew
        bool finite = true;
        for (int i = 0; i < 1000; ++i) {
            float out = jrc.process(0.1f);
            if (!std::isfinite(out)) { finite = false; break; }
        }
        check(finite, "setSlew(0.3): no NaN/Inf");
    }

    // Creative methods: setCharacter
    {
        WdfOpAmpJRC4558 jrc;
        jrc.init(48000.0f);
        jrc.setCharacter(0.5f);
        bool finite = true;
        for (int i = 0; i < 1000; ++i) {
            float out = jrc.process(0.0f);
            if (!std::isfinite(out)) { finite = false; break; }
        }
        check(finite, "setCharacter(0.5): no NaN/Inf");
    }

    // No NaN/Inf sweep
    {
        WdfOpAmpJRC4558 jrc;
        jrc.init(48000.0f);
        bool noNaN = true;
        for (int i = 0; i < 1000; ++i) {
            float v = -1.0f + 2.0f * static_cast<float>(i) / 999.0f;
            float out = jrc.process(v);
            if (!std::isfinite(out)) { noNaN = false; break; }
        }
        check(noNaN, "Input sweep -1V to +1V: no NaN/Inf");
    }
}

// ============================================================
// TS808 Circuit tests
// ============================================================
static void testTS808Circuit() {
    std::printf("\n--- TS808 Circuit ---\n");

    // Helper: measure peak amplitude of output for a sine input after settling
    auto measurePeak = [](TS808Circuit& ts, float freq, float amp, int totalSamples, int settleAt) {
        float peak = 0.0f;
        for (int i = 0; i < totalSamples; ++i) {
            float in = amp * sinf(static_cast<float>(i) * 6.283185307f * freq / 48000.0f);
            float out = ts.process(in);
            if (i >= settleAt && fabsf(out) > peak) peak = fabsf(out);
        }
        return peak;
    };

    // Mid hump (key test): input highpass at 720 Hz creates structural mid emphasis.
    // At low drive (IIR pole ~2500 Hz, above the midrange), the gain is nearly flat
    // from 0 to 2500 Hz, so the input HP dominates: 800 Hz passes the HP much
    // better than 200 Hz, producing significantly more output.
    // This is the fundamental TS structural character, NOT an EQ choice.
    {
        auto measure = [&](float freq) {
            TS808Circuit ts;
            ts.init(48000.0f);
            ts.setDrive(0.08f);   // low drive → IIR pole ~2100 Hz, flat in midrange
            ts.setTone(0.5f);
            ts.setLevel(0.8f);
            return measurePeak(ts, freq, 0.3f, 96000, 48000);
        };

        float amp200  = measure(200.0f);
        float amp800  = measure(800.0f);

        check(amp800 > amp200,
              "Mid hump: at low drive, 800 Hz output > 200 Hz (input HP at 720 Hz)");
        check(amp200 > 0.0001f, "Mid hump: 200 Hz still produces some output (not zero)");
    }

    // IIR lowpass gain structure: the feedback capacitor creates a lowpass in the
    // closed-loop gain. At high drive (large Rf), the IIR pole moves to very low
    // frequencies, giving enormous gain at bass frequencies but rolling off treble.
    // Net result at drive=0.9: 100 Hz produces significantly more output than 5 kHz
    // because the IIR amplifies bass far more than treble.
    {
        auto measurePeakPP = [](float freq, float drive) {
            TS808Circuit ts;
            ts.init(48000.0f);
            ts.setDrive(drive);
            ts.setTone(0.5f);
            ts.setLevel(1.0f);
            float posMax = 0.0f, negMin = 0.0f;
            for (int i = 0; i < 96000; ++i) {
                float in = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * freq / 48000.0f);
                float out = ts.process(in);
                if (i >= 48000) {
                    if (out > posMax) posMax = out;
                    if (out < negMin) negMin = out;
                }
            }
            return posMax - negMin;
        };

        float pp100Hz = measurePeakPP(100.0f, 0.9f);
        float pp5kHz  = measurePeakPP(5000.0f, 0.9f);

        // At drive=0.9, IIR pole ≈ 13 Hz — 100 Hz is above it and sees high gain
        // (partially through input HP) → clips and produces audible output.
        // 5 kHz sees extremely low IIR gain → much less output.
        check(pp100Hz > pp5kHz,
              "IIR lowpass gain: at high drive, 100 Hz output > 5 kHz (IIR pole at ~13 Hz)");
    }

    // Drive range: low drive → less clipped output, high drive → more
    {
        auto measureRMS = [](float drive) {
            TS808Circuit ts;
            ts.init(48000.0f);
            ts.setDrive(drive);
            ts.setTone(0.5f);
            ts.setLevel(1.0f);
            float sumSq = 0.0f;
            int count = 0;
            for (int i = 0; i < 96000; ++i) {
                float in = 0.1f * sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f);
                float out = ts.process(in);
                if (i >= 48000) { sumSq += out * out; count++; }
            }
            return sqrtf(sumSq / static_cast<float>(count));
        };

        float rmsLow  = measureRMS(0.1f);
        float rmsHigh = measureRMS(0.9f);
        check(rmsHigh > rmsLow,
              "Drive range: higher drive → more output (more clipping/gain)");
    }

    // Tone control: Tone=0.1 (dark) vs Tone=0.9 (bright) on white noise
    {
        auto measureBandRatio = [](float tone) {
            TS808Circuit ts;
            ts.init(48000.0f);
            ts.setDrive(0.5f);
            ts.setTone(tone);
            ts.setLevel(1.0f);

            // Separate output into bands using simple averaging approximation:
            // Process two fixed-frequency sinusoids to compare response
            float sumLow = 0.0f, sumHigh = 0.0f;
            for (int i = 0; i < 96000; ++i) {
                float in = 0.1f * sinf(static_cast<float>(i) * 6.283185307f * 400.0f / 48000.0f);
                float out = ts.process(in);
                if (i >= 48000) sumLow += out * out;
            }
            ts.reset();
            ts.setTone(tone);

            for (int i = 0; i < 96000; ++i) {
                float in = 0.1f * sinf(static_cast<float>(i) * 6.283185307f * 4000.0f / 48000.0f);
                float out = ts.process(in);
                if (i >= 48000) sumHigh += out * out;
            }

            return sumLow / (sumHigh + 1e-10f); // high ratio → more bass
        };

        float ratioDark   = measureBandRatio(0.1f);  // Tone=0.1 → more bass
        float ratioBright = measureBandRatio(0.9f);  // Tone=0.9 → more treble

        check(ratioDark > ratioBright,
              "Tone control: dark (0.1) has higher low/high ratio than bright (0.9)");
    }

    // Stability with silence: output settles to near zero
    {
        TS808Circuit ts;
        ts.init(48000.0f);
        ts.setDrive(0.5f);
        ts.setTone(0.5f);
        ts.setLevel(0.5f);

        bool stable = true;
        float lastOut = 0.0f;
        for (int i = 0; i < 96000; ++i) {
            float out = ts.process(0.0f);
            if (!std::isfinite(out)) { stable = false; break; }
            lastOut = out;
        }
        check(stable, "Silence stability: all outputs finite over 2s");
        check(fabsf(lastOut) < 0.01f, "Silence stability: settles to near zero");
    }

    // No NaN/Inf: random noise + rapid drive changes
    {
        TS808Circuit ts;
        ts.init(48000.0f);
        ts.setLevel(1.0f);

        bool noNaN = true;
        unsigned int seed = 42;
        for (int i = 0; i < 480000; ++i) {
            seed = seed * 1103515245u + 12345u;
            float noise = (static_cast<float>(seed & 0xFFFF) / 32768.0f) - 1.0f;
            if (i % 4800 == 0) {
                float d = static_cast<float>((i / 4800) % 10) / 10.0f;
                ts.setDrive(d);
            }
            float out = ts.process(noise);
            if (!std::isfinite(out)) { noNaN = false; break; }
        }
        check(noNaN, "10s noise + drive sweeps: no NaN/Inf");
    }
}

// ============================================================
// TS9 Circuit tests
// ============================================================
static void testTS9Circuit() {
    std::printf("\n--- TS9 Circuit ---\n");

    // TS808 vs TS9: outputs should be similar but not identical
    {
        TS9Circuit ts808mode, ts9mode;
        ts808mode.init(48000.0f, TS9Circuit::OutputBuffer::TS808_OpAmp);
        ts9mode.init(48000.0f,   TS9Circuit::OutputBuffer::TS9_Transistor);
        ts808mode.setDrive(0.5f); ts808mode.setTone(0.5f); ts808mode.setLevel(0.8f);
        ts9mode.setDrive(0.5f);   ts9mode.setTone(0.5f);   ts9mode.setLevel(0.8f);

        float maxDiff = 0.0f;
        float sumAB = 0.0f, sumA2 = 0.0f, sumB2 = 0.0f;
        int count = 0;
        for (int i = 0; i < 48000; ++i) {
            float in = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f);
            float a = ts808mode.process(in);
            float b = ts9mode.process(in);
            if (i >= 4800) {
                float diff = fabsf(a - b);
                if (diff > maxDiff) maxDiff = diff;
                sumAB += a * b;
                sumA2 += a * a;
                sumB2 += b * b;
                count++;
            }
        }
        float corr = sumAB / (sqrtf(sumA2 * sumB2) + 1e-10f);

        check(corr > 0.9f, "TS808 vs TS9: correlation > 0.9 (same core circuit)");
        check(maxDiff > 0.0001f, "TS808 vs TS9: outputs differ (different buffer)");
        (void)count;
    }

    // Both modes produce finite output
    {
        bool ts808ok = true, ts9ok = true;
        TS9Circuit ts808, ts9;
        ts808.init(48000.0f, TS9Circuit::OutputBuffer::TS808_OpAmp);
        ts9.init(48000.0f, TS9Circuit::OutputBuffer::TS9_Transistor);
        ts808.setDrive(0.7f); ts808.setTone(0.5f); ts808.setLevel(0.8f);
        ts9.setDrive(0.7f); ts9.setTone(0.5f); ts9.setLevel(0.8f);

        for (int i = 0; i < 48000; ++i) {
            float in = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
            if (!std::isfinite(ts808.process(in))) { ts808ok = false; break; }
            if (!std::isfinite(ts9.process(in)))   { ts9ok   = false; break; }
        }
        check(ts808ok, "TS9 TS808 mode: no NaN/Inf");
        check(ts9ok,   "TS9 Transistor mode: no NaN/Inf");
    }

    // setOutputBuffer changes behavior
    {
        TS9Circuit ts;
        ts.init(48000.0f, TS9Circuit::OutputBuffer::TS808_OpAmp);
        ts.setDrive(0.5f); ts.setTone(0.5f); ts.setLevel(0.8f);
        float out808 = 0.0f;
        for (int i = 0; i < 4800; ++i)
            out808 = ts.process(0.3f * sinf(static_cast<float>(i) * 0.131f));

        ts.setOutputBuffer(TS9Circuit::OutputBuffer::TS9_Transistor);
        ts.reset();
        float outTs9 = 0.0f;
        for (int i = 0; i < 4800; ++i)
            outTs9 = ts.process(0.3f * sinf(static_cast<float>(i) * 0.131f));

        check(std::isfinite(out808) && std::isfinite(outTs9),
              "OutputBuffer switch: both modes produce finite output");
    }
}

// ============================================================
// Klon Clip Stage tests
// ============================================================
static void testKlonClipStage() {
    std::printf("\n--- Klon Clip Stage ---\n");

    // Asymmetry (key test): mismatched diodes → different positive/negative peaks
    {
        KlonClipStage klon;
        klon.init(48000.0f);
        klon.setGain(0.5f);

        float posMax = 0.0f, negMin = 0.0f;
        for (int i = 0; i < 96000; ++i) {
            float in = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f);
            float out = klon.process(in);
            if (i >= 48000) {
                if (out > posMax) posMax = out;
                if (out < negMin) negMin = out;
            }
        }
        float diff = fabsf(posMax) - fabsf(negMin);
        check(fabsf(diff) > 0.001f,
              "Asymmetry: |positive peak| != |negative peak| (mismatched diodes)");
    }

    // No NaN/Inf
    {
        KlonClipStage klon;
        klon.init(48000.0f);
        klon.setGain(0.5f);
        bool finite = true;
        for (int i = 0; i < 48000; ++i) {
            float in = 0.3f * sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f);
            if (!std::isfinite(klon.process(in))) { finite = false; break; }
        }
        check(finite, "No NaN/Inf at 1kHz sine");
    }

    // Gain range: higher gain → more output energy (more clipping/drive)
    {
        auto measureRMS = [](float gain) {
            KlonClipStage klon;
            klon.init(48000.0f);
            klon.setGain(gain);
            float sumSq = 0.0f;
            int count = 0;
            for (int i = 0; i < 96000; ++i) {
                float in = 0.1f * sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f);
                float out = klon.process(in);
                if (i >= 48000) { sumSq += out * out; count++; }
            }
            return sqrtf(sumSq / static_cast<float>(count));
        };

        float rmsLow  = measureRMS(0.1f);
        float rmsHigh = measureRMS(0.9f);
        check(rmsHigh > rmsLow,
              "Gain range: higher gain → more output energy");
    }

    // Silence stability
    {
        KlonClipStage klon;
        klon.init(48000.0f);
        klon.setGain(0.5f);
        bool stable = true;
        float lastOut = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            float out = klon.process(0.0f);
            if (!std::isfinite(out)) { stable = false; break; }
            lastOut = out;
        }
        check(stable, "Silence: all outputs finite");
        check(fabsf(lastOut) < 0.01f, "Silence: settles to near zero");
    }
}

// ============================================================
// Performance benchmark
// ============================================================
static void testPerformance() {
    std::printf("\n--- Performance ---\n");

    constexpr int kSamples = 100000;
    float input[256];
    for (int i = 0; i < 256; ++i)
        input[i] = sinf(static_cast<float>(i) * 0.1f) * 0.3f;

    // JRC4558 standalone
    {
        WdfOpAmpJRC4558 jrc;
        jrc.init(48000.0f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i)
            dummy = jrc.process(input[i & 255] * 0.001f);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: JRC4558: %lld ns/sample\n",
                    static_cast<long long>(ns / kSamples));
    }

    // TS808Circuit complete
    {
        TS808Circuit ts;
        ts.init(48000.0f);
        ts.setDrive(0.7f);
        ts.setTone(0.5f);
        ts.setLevel(0.8f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i)
            dummy = ts.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        long long nsPerSample = ns / kSamples;
        std::printf("  INFO: TS808Circuit: %lld ns/sample (budget ~11000 ns)\n",
                    nsPerSample);
        check(nsPerSample < 12000LL,
              "TS808Circuit: performance within 12000 ns/sample budget");
    }

    // TS9Circuit
    {
        TS9Circuit ts;
        ts.init(48000.0f, TS9Circuit::OutputBuffer::TS9_Transistor);
        ts.setDrive(0.7f);
        ts.setTone(0.5f);
        ts.setLevel(0.8f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i)
            dummy = ts.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: TS9Circuit: %lld ns/sample\n",
                    static_cast<long long>(ns / kSamples));
    }

    // KlonClipStage
    {
        KlonClipStage klon;
        klon.init(48000.0f);
        klon.setGain(0.5f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i)
            dummy = klon.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: KlonClipStage: %lld ns/sample\n",
                    static_cast<long long>(ns / kSamples));
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    std::printf("=== JRC4558 / Tubescreamer Test Suite ===\n");

    testJRC4558OpenLoop();
    testTS808Circuit();
    testTS9Circuit();
    testKlonClipStage();
    testPerformance();

    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);

    return (passedTests == totalTests) ? 0 : 1;
}
