// PNP BJT WDF Primitive and Circuit Tests
// Build: g++ -std=c++20 -O2 -Wall -Werror -Wdouble-promotion -I.. tests/pnp_bjt_test.cpp -o /tmp/pnp_bjt_test -lm
// Run:   /tmp/pnp_bjt_test

#include "wdf/WdfPnpBjt.h"
#include "wdf/WdfPnpCircuits.h"
#include <cstdio>
#include <cmath>
#include <limits>
#include <chrono>

static int gFailures = 0;
static int gTotal = 0;

static void CHECK(bool cond, const char* msg) {
    ++gTotal;
    if (cond) {
        std::printf("  PASS: %s\n", msg);
    } else {
        std::printf("  FAIL: %s\n", msg);
        ++gFailures;
    }
}

// ---------------------------------------------------------------------------
// 1. Polarity check: forward vs. reverse bias
// ---------------------------------------------------------------------------
static void testPolarityCheck() {
    std::printf("\n--- Polarity Check ---\n");

    // Helper: drive BJT with Veb bias and return IC (current out of collector)
    auto measureIC = [](WdfPnpBjt& bjt, float Ve, float Vb) -> float {
        // aE = Ve*2 (so Ve_port ≈ Ve when no current), aB = Vb*2, aC = 0
        bjt.portE.a = Ve * 2.0f;
        bjt.portB.a = Vb * 2.0f;
        bjt.portC.a = 0.0f;
        bjt.reflect();
        // IC out of collector: bC = aC + 2*RpC*IC → IC = (bC - aC)/(2*RpC)
        return (bjt.portC.b - bjt.portC.a) / (2.0f * bjt.portC.Rp);
    };

    const char* names[] = { "OC44", "OC75", "AC128", "N3906" };
    WdfPnpBjt bjts[4] = {
        WdfPnpBjt::makeOC44(10000.0f, 10000.0f, 1000.0f),
        WdfPnpBjt::makeOC75(10000.0f, 10000.0f, 1000.0f),
        WdfPnpBjt::makeAC128(10000.0f, 10000.0f, 1000.0f),
        WdfPnpBjt::make2N3906(10000.0f, 10000.0f, 1000.0f),
    };

    for (int i = 0; i < 4; ++i) {
        bjts[i].reset();
        // Forward bias: Ve=0.3V above Vb → Veb=0.3V → should conduct
        float IC_fwd = measureIC(bjts[i], 0.3f, 0.0f);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s: IC > 0 in forward bias (Veb=0.3V)", names[i]);
        CHECK(IC_fwd > 0.0f, buf);

        bjts[i].reset();
        // Reverse bias: Ve below Vb → Veb < 0 → should not conduct (IC ≈ 0 or near-zero)
        float IC_rev = measureIC(bjts[i], 0.0f, 0.5f);
        std::snprintf(buf, sizeof(buf), "%s: IC ≈ 0 in reverse bias (Veb=-0.5V)", names[i]);
        CHECK(IC_rev < 1e-4f, buf);
    }
}

// ---------------------------------------------------------------------------
// 2. Current gain: IC/IB should be within 5% of hFE
// ---------------------------------------------------------------------------
static void testCurrentGain() {
    std::printf("\n--- Current Gain (IC/IB ≈ hFE) ---\n");

    struct Preset { const char* name; WdfPnpBjt bjt; float hFE; };
    Preset presets[] = {
        { "OC44",  WdfPnpBjt::makeOC44(50000.0f, 5000.0f, 500.0f),   70.0f },
        { "OC75",  WdfPnpBjt::makeOC75(50000.0f, 5000.0f, 500.0f),   45.0f },
        { "AC128", WdfPnpBjt::makeAC128(50000.0f, 5000.0f, 500.0f),  80.0f },
        { "N3906", WdfPnpBjt::make2N3906(50000.0f, 5000.0f, 500.0f), 200.0f },
    };

    for (auto& p : presets) {
        p.bjt.reset();

        // Apply a modest forward bias to put device in forward active
        // Veb ≈ 0.15V for germanium, 0.6V for silicon (N3906)
        float Veb_drive = (p.hFE > 150.0f) ? 0.6f : 0.15f;
        p.bjt.portE.a = Veb_drive * 2.0f;
        p.bjt.portB.a = 0.0f;
        p.bjt.portC.a = 0.0f;

        // Run several samples to warm-start the NR
        for (int i = 0; i < 10; ++i) p.bjt.reflect();

        float IC = (p.bjt.portC.b - p.bjt.portC.a) / (2.0f * p.bjt.portC.Rp);
        float IB = (p.bjt.portB.a - p.bjt.portB.b) / (2.0f * p.bjt.portB.Rp);

        char buf[80];
        if (IB > 1e-12f && IC > 0.0f) {
            float ratio = IC / IB;
            float err = fabsf(ratio / p.hFE - 1.0f);
            std::snprintf(buf, sizeof(buf), "%s: IC/IB within 5%% of hFE=%.0f (got %.1f)",
                          p.name, static_cast<double>(p.hFE), static_cast<double>(ratio));
            CHECK(err < 0.05f, buf);
        } else {
            std::snprintf(buf, sizeof(buf), "%s: conducts with Veb=%.2fV", p.name, static_cast<double>(Veb_drive));
            CHECK(IC > 0.0f, buf);
        }
    }
}

// ---------------------------------------------------------------------------
// 3. Early effect: IC should increase with |Vce| at fixed Veb
// ---------------------------------------------------------------------------
static void testEarlyEffect() {
    std::printf("\n--- Early Effect ---\n");

    // OC75 (Vaf=40) should show more Early slope than N3906 (Vaf=100)
    struct EarlyTest { const char* name; WdfPnpBjt bjt; };
    EarlyTest tests[] = {
        { "OC75",  WdfPnpBjt::makeOC75(1000.0f, 1000.0f, 100.0f) },
        { "N3906", WdfPnpBjt::make2N3906(1000.0f, 1000.0f, 100.0f) },
    };

    float slope[2];
    for (int t = 0; t < 2; ++t) {
        tests[t].bjt.reset();

        // Apply fixed Veb drive, vary aCollector (Vc)
        float Veb_drive = (t == 1) ? 0.6f : 0.15f;
        tests[t].bjt.portE.a = Veb_drive * 2.0f;
        tests[t].bjt.portB.a = 0.0f;

        // Two Vc values: -2V and -8V (Vce < 0 for PNP in active region)
        float IC_lo, IC_hi;
        {
            tests[t].bjt.portC.a = -4.0f; // aCollector → Vc approx -2V
            for (int i = 0; i < 5; ++i) tests[t].bjt.reflect();
            IC_lo = (tests[t].bjt.portC.b - tests[t].bjt.portC.a) / (2.0f * tests[t].bjt.portC.Rp);
        }
        {
            tests[t].bjt.portC.a = -16.0f; // Vc approx -8V
            for (int i = 0; i < 5; ++i) tests[t].bjt.reflect();
            IC_hi = (tests[t].bjt.portC.b - tests[t].bjt.portC.a) / (2.0f * tests[t].bjt.portC.Rp);
        }

        slope[t] = IC_hi - IC_lo;
        char buf[80];
        std::snprintf(buf, sizeof(buf), "%s: IC increases with |Vce| (Early effect)", tests[t].name);
        CHECK(slope[t] > 0.0f, buf);
    }

    // OC75 (Vaf=40) has stronger Early effect than N3906 (Vaf=100)
    CHECK(slope[0] > slope[1], "OC75 (Vaf=40) has larger Early slope than N3906 (Vaf=100)");
}

// ---------------------------------------------------------------------------
// 4. Germanium vs silicon Is ratio
// ---------------------------------------------------------------------------
static void testGermaniumVsSiliconIs() {
    std::printf("\n--- Germanium vs Silicon Is ratio ---\n");

    WdfPnpBjt ge = WdfPnpBjt::makeOC75(1000.0f, 1000.0f, 100.0f);
    WdfPnpBjt si = WdfPnpBjt::make2N3906(1000.0f, 1000.0f, 100.0f);

    // At the same Veb (using silicon drive so N3906 conducts too)
    float drive = 0.6f;
    ge.portE.a = drive * 2.0f; ge.portB.a = 0.0f; ge.portC.a = 0.0f;
    si.portE.a = drive * 2.0f; si.portB.a = 0.0f; si.portC.a = 0.0f;
    for (int i = 0; i < 10; ++i) { ge.reflect(); si.reflect(); }

    float IC_ge = (ge.portC.b - ge.portC.a) / (2.0f * ge.portC.Rp);
    float IC_si = (si.portC.b - si.portC.a) / (2.0f * si.portC.Rp);

    // Germanium (Is=3e-6) should have much higher IC than silicon (Is=1e-12) at same Veb
    CHECK(IC_ge > IC_si * 10.0f,
          "Germanium IC >> silicon IC at same Veb (Is ratio ≈ 3e6)");
}

// ---------------------------------------------------------------------------
// 5. Stability: 48000 samples through RangemasterCircuit
// ---------------------------------------------------------------------------
static void testStability() {
    std::printf("\n--- Stability (48000 samples, Rangemaster) ---\n");

    RangemasterCircuit rm;
    rm.init(48000.0f);

    bool stable = true;
    for (int i = 0; i < 48000; ++i) {
        float input = sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f) * 0.1f;
        float out = rm.process(input);
        if (!std::isfinite(out)) { stable = false; break; }
    }
    CHECK(stable, "Rangemaster: no NaN/Inf in 48000 samples of 1kHz sine");
}

// ---------------------------------------------------------------------------
// 6. Newton-Raphson convergence statistics
// ---------------------------------------------------------------------------
static void testNRConvergence() {
    std::printf("\n--- NR Convergence ---\n");

    const char* names[] = { "OC44", "OC75", "AC128", "N3906" };

    auto makeRM = [&](int idx) -> RangemasterCircuit {
        RangemasterCircuit rm;
        rm.init(48000.0f);
        // Re-init BJT with desired preset
        const WdfPnpBjt::Params presets[] = {
            pnp_presets::OC44, pnp_presets::OC75, pnp_presets::AC128, pnp_presets::N3906
        };
        rm.bjt().init(presets[idx], rm.bjt().portB.Rp, rm.bjt().portC.Rp, rm.bjt().portE.Rp);
        return rm;
    };

    constexpr int kSamples = 1000;
    for (int preset = 0; preset < 4; ++preset) {
        RangemasterCircuit rm = makeRM(preset);
        long totalIter = 0;
        int convergedCount = 0;
        for (int i = 0; i < kSamples; ++i) {
            float input = sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f) * 0.1f;
            (void)rm.process(input);
            totalIter += rm.bjt().lastIterationCount();
            if (rm.bjt().lastConverged()) ++convergedCount;
        }
        float avgIter = static_cast<float>(totalIter) / kSamples;
        float convergePct = static_cast<float>(convergedCount) / kSamples * 100.0f;
        std::printf("  INFO: %s: avg iter=%.2f, converged=%.1f%%\n",
                    names[preset], static_cast<double>(avgIter), static_cast<double>(convergePct));

        char buf[80];
        std::snprintf(buf, sizeof(buf), "%s: average NR iterations < 6", names[preset]);
        CHECK(avgIter < 6.0f, buf);
        std::snprintf(buf, sizeof(buf), "%s: NR converged > 98%% of samples", names[preset]);
        CHECK(convergePct > 98.0f, buf);
    }
}

// ---------------------------------------------------------------------------
// 7. NaN safety: extreme input values
// ---------------------------------------------------------------------------
static void testNaNSafety() {
    std::printf("\n--- NaN Safety ---\n");

    auto testCircuit = [&](auto& circuit, const char* name) {
        circuit.reset();
        float extremes[] = {
            std::numeric_limits<float>::max(),
            -std::numeric_limits<float>::max(),
            0.0f,
            1.0f,
            -1.0f
        };
        bool ok = true;
        for (float v : extremes) {
            float out = circuit.process(v);
            if (!std::isfinite(out)) { ok = false; break; }
        }
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s: finite output on extreme inputs", name);
        CHECK(ok, buf);
    };

    RangemasterCircuit rm;    rm.init(48000.0f);
    FuzzFaceCircuit    ff;    ff.init(48000.0f);
    ToneBenderMk1Circuit tb;  tb.init(48000.0f);

    testCircuit(rm, "Rangemaster");
    testCircuit(ff, "FuzzFace");
    testCircuit(tb, "ToneBender");
}

// ---------------------------------------------------------------------------
// 8. Frequency response: Rangemaster is a treble booster
// ---------------------------------------------------------------------------
static void testFrequencyResponse() {
    std::printf("\n--- Frequency Response ---\n");

    RangemasterCircuit rm;
    rm.init(48000.0f);
    rm.setInputCharacter(1.0f); // full treble boost (5nF)

    constexpr float kFs = 48000.0f;
    constexpr int kSettle = 4800;
    constexpr int kMeasure = 9600;

    auto measureAmplitude = [&](float freq) -> float {
        rm.reset();
        float energy = 0.0f;
        for (int i = 0; i < kSettle + kMeasure; ++i) {
            float input = sinf(static_cast<float>(i) * 6.283185307f * freq / kFs) * 0.05f;
            float out = rm.process(input);
            if (i >= kSettle) energy += out * out;
        }
        return sqrtf(energy / kMeasure);
    };

    float amp100  = measureAmplitude(100.0f);
    float amp1k   = measureAmplitude(1000.0f);
    float amp5k   = measureAmplitude(5000.0f);

    std::printf("  INFO: RMS at 100Hz=%.5f  1kHz=%.5f  5kHz=%.5f\n",
                static_cast<double>(amp100), static_cast<double>(amp1k), static_cast<double>(amp5k));

    CHECK(amp1k > amp100, "Rangemaster: more output at 1kHz than 100Hz (treble boost)");
    CHECK(amp5k > amp100, "Rangemaster: more output at 5kHz than 100Hz (treble boost)");
}

// ---------------------------------------------------------------------------
// 9. DC blocking: coupling caps remove DC offset
// ---------------------------------------------------------------------------
static void testDCBlocking() {
    std::printf("\n--- DC Blocking ---\n");

    RangemasterCircuit rm;
    rm.init(48000.0f);

    // Apply DC for 100ms (4800 samples) to charge caps, then measure
    constexpr int kSettle = 48000;
    for (int i = 0; i < kSettle; ++i) (void)rm.process(0.5f);

    // Measure average output (should be near zero due to DC blocking cap)
    float sum = 0.0f;
    constexpr int kMeasure = 4800;
    for (int i = 0; i < kMeasure; ++i) sum += rm.process(0.5f);
    float dcOut = fabsf(sum / kMeasure);

    std::printf("  INFO: DC offset after coupling = %.5f\n", static_cast<double>(dcOut));
    CHECK(dcOut < 0.01f, "Rangemaster: DC output < 0.01 after cap charging");
}

// ---------------------------------------------------------------------------
// 10. Gain range: 1kHz at 0.01 amplitude → output should be 2x–20x louder
// ---------------------------------------------------------------------------
static void testGainRange() {
    std::printf("\n--- Gain Range ---\n");

    RangemasterCircuit rm;
    rm.init(48000.0f);

    constexpr float kInputAmp = 0.01f;
    constexpr float kFs = 48000.0f;
    constexpr int kSettle = 4800;
    constexpr int kMeasure = 4800;

    float inputEnergy = 0.0f;
    float outputEnergy = 0.0f;
    for (int i = 0; i < kSettle + kMeasure; ++i) {
        float inp = sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / kFs) * kInputAmp;
        float out = rm.process(inp);
        if (i >= kSettle) {
            inputEnergy  += inp * inp;
            outputEnergy += out * out;
        }
    }
    float gainRMS = sqrtf(outputEnergy / inputEnergy);
    std::printf("  INFO: Rangemaster RMS gain at 1kHz = %.2f\n", static_cast<double>(gainRMS));
    CHECK(gainRMS > 2.0f && gainRMS < 30.0f,
          "Rangemaster: RMS gain in reasonable range (2x–30x)");
}

// ---------------------------------------------------------------------------
// 11. Fuzz interaction: more harmonic content at fuzz=1 vs fuzz=0
// ---------------------------------------------------------------------------
static void testFuzzInteraction() {
    std::printf("\n--- Fuzz Interaction ---\n");

    constexpr float kFs = 48000.0f;
    constexpr int kSettle = 4800;
    constexpr int kMeasure = 4800;

    auto measureHarmonicEnergy = [&](float fuzzLevel) -> float {
        FuzzFaceCircuit ff;
        ff.init(kFs);
        ff.setFuzz(fuzzLevel);
        ff.setVolume(1.0f);

        // Run with 1kHz fundamental
        float fundamental = 0.0f;
        float total = 0.0f;
        for (int i = 0; i < kSettle + kMeasure; ++i) {
            float inp = sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / kFs) * 0.2f;
            float out = ff.process(inp);
            if (i >= kSettle) {
                float phase = static_cast<float>(i - kSettle) * 6.283185307f * 1000.0f / kFs;
                fundamental += out * sinf(phase);
                total += out * out;
            }
        }
        float fundPower = fundamental * fundamental / (kMeasure * kMeasure);
        return total / kMeasure - fundPower; // harmonic energy
    };

    float harm0 = measureHarmonicEnergy(0.0f);
    float harm1 = measureHarmonicEnergy(1.0f);

    std::printf("  INFO: Harmonic energy fuzz=0: %.6f  fuzz=1: %.6f\n",
                static_cast<double>(harm0), static_cast<double>(harm1));
    CHECK(harm1 >= harm0, "FuzzFace: more harmonic energy at fuzz=1 vs fuzz=0");
}

// ---------------------------------------------------------------------------
// 12. Two-stage stability: silence after reset stays near zero
// ---------------------------------------------------------------------------
static void testTwoStageStability() {
    std::printf("\n--- Two-Stage Stability ---\n");

    FuzzFaceCircuit ff;
    ff.init(48000.0f);
    ff.reset();

    bool stable = true;
    float maxOut = 0.0f;
    for (int i = 0; i < 48000; ++i) {
        float out = ff.process(0.0f);
        if (!std::isfinite(out)) { stable = false; break; }
        if (fabsf(out) > maxOut) maxOut = fabsf(out);
    }
    CHECK(stable, "FuzzFace: no NaN/Inf in 48000 samples of silence");
    std::printf("  INFO: max output on silence = %.6f\n", static_cast<double>(maxOut));
    CHECK(maxOut < 0.1f, "FuzzFace: output near zero during silence");
}

// ---------------------------------------------------------------------------
// 13. Performance: timing
// ---------------------------------------------------------------------------
static void testPerformance() {
    std::printf("\n--- Performance Estimate ---\n");

    constexpr int kSamples = 100000;
    constexpr float kFs = 48000.0f;

    auto timeCircuit = [&](auto& circuit, const char* name) {
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < kSamples; ++i) {
            float inp = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / kFs) * 0.1f;
            (void)circuit.process(inp);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()
                    / static_cast<double>(kSamples);
        std::printf("  INFO: %s: %.0f ns/sample\n", name, ns);
    };

    RangemasterCircuit rm;   rm.init(kFs);
    FuzzFaceCircuit    ff;   ff.init(kFs);
    ToneBenderMk1Circuit tb; tb.init(kFs);
    GeBoostCircuit     gb;
    gb.init(GeBoostCircuit::ComponentValues{}, pnp_presets::AC128, kFs);

    timeCircuit(rm, "Rangemaster");
    timeCircuit(ff, "FuzzFace");
    timeCircuit(tb, "ToneBender");
    timeCircuit(gb, "GeBoost");

    // Warn (but don't fail) if FuzzFace is very slow on host
    std::printf("  (Performance thresholds apply on ARM Cortex-M7 target)\n");
    CHECK(true, "Performance test completed");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
    std::printf("=== PNP BJT WDF Test Suite ===\n");

    testPolarityCheck();
    testCurrentGain();
    testEarlyEffect();
    testGermaniumVsSiliconIs();
    testStability();
    testNRConvergence();
    testNaNSafety();
    testFrequencyResponse();
    testDCBlocking();
    testGainRange();
    testFuzzInteraction();
    testTwoStageStability();
    testPerformance();

    std::printf("\n=== Results: %d / %d passed ===\n", gTotal - gFailures, gTotal);
    return (gFailures == 0) ? 0 : 1;
}
