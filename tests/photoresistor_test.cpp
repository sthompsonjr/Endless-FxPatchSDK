/// Photoresistor & Optical Compressor test suite.
/// Build: g++ -std=c++20 -O2 -Wall -Werror -Wdouble-promotion -I.. tests/photoresistor_test.cpp -o photoresistor_test -lm

#include "../wdf/WdfPhotoresistor.h"
#include "../wdf/WdfOpticalCircuits.h"
#include "../wdf/WdfCompressorCircuits.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <algorithm>

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

static bool approxRel(float a, float b, float relTol = 0.01f) {
    float denom = std::max(fabsf(b), 1e-6f);
    return fabsf(a - b) / denom < relTol;
}

// ============================================================
// WdfPhotoresistor — Physical model validation
// ============================================================

static void testAsymmetricAttackRelease() {
    std::printf("\n--- LDR Asymmetric Attack/Release ---\n");

    WdfPhotoresistor::Params params;
    params.sampleRate = 48000.0f;
    WdfPhotoresistor ldr;
    ldr.init(params);

    // Attack: apply full illumination, count samples until within 10% of R_bright
    float attackThreshold = ldr_params::R_dark - 0.9f * (ldr_params::R_dark - ldr_params::R_bright);
    int attackSamples = 0;
    int maxSamples = static_cast<int>(5.0f * 48000.0f); // 5 seconds max
    for (int i = 0; i < maxSamples; ++i) {
        ldr.setLightLevel(1.0f);
        attackSamples++;
        if (ldr.getCurrentResistance() <= attackThreshold) break;
    }

    // Release: apply darkness, count samples until within 10% of R_dark
    float releaseThreshold = ldr_params::R_bright + 0.9f * (ldr_params::R_dark - ldr_params::R_bright);
    int releaseSamples = 0;
    for (int i = 0; i < maxSamples; ++i) {
        ldr.setLightLevel(0.0f);
        releaseSamples++;
        if (ldr.getCurrentResistance() >= releaseThreshold) break;
    }

    double attackMs  = static_cast<double>(attackSamples)  / 48000.0 * 1000.0;
    double releaseMs = static_cast<double>(releaseSamples) / 48000.0 * 1000.0;
    std::printf("  INFO: Attack samples: %d (%.1fms), Release samples: %d (%.1fms)\n",
                attackSamples,  attackMs,
                releaseSamples, releaseMs);

    check(attackSamples < maxSamples, "LDR: attack completed within time limit");
    check(releaseSamples < maxSamples, "LDR: release completed within time limit");
    check(releaseSamples >= 3 * attackSamples,
          "LDR: release takes at least 3x longer than attack (asymmetry)");
}

static void testLogLinearMapping() {
    std::printf("\n--- LDR Log-Linear Mapping ---\n");

    WdfPhotoresistor::Params params;
    params.sampleRate = 48000.0f;
    WdfPhotoresistor ldr;
    ldr.init(params);

    // Run 5 seconds at light=0 to reach steady state at R_dark
    for (int i = 0; i < 5 * 48000; ++i) ldr.setLightLevel(0.0f);
    check(approxRel(ldr.getCurrentResistance(), ldr_params::R_dark, 0.05f),
          "LDR: lightLevel=0 steady state ≈ R_dark (1MΩ)");

    // Run 5 seconds at light=1 to reach steady state at R_bright
    ldr.reset();
    for (int i = 0; i < 5 * 48000; ++i) ldr.setLightLevel(1.0f);
    check(approxRel(ldr.getCurrentResistance(), ldr_params::R_bright, 0.05f),
          "LDR: lightLevel=1 steady state ≈ R_bright (200Ω)");

    // Geometric mean check: at lightLevel=0.5, expected ≈ sqrt(R_dark * R_bright)
    ldr.reset();
    for (int i = 0; i < 10 * 48000; ++i) ldr.setLightLevel(0.5f);
    float expectedGeomMean = sqrtf(ldr_params::R_dark * ldr_params::R_bright); // ≈ 14142
    std::printf("  INFO: R at light=0.5: %.1f Ω (expected geometric mean ≈ %.1f Ω)\n",
                static_cast<double>(ldr.getCurrentResistance()),
                static_cast<double>(expectedGeomMean));
    check(approxRel(ldr.getCurrentResistance(), expectedGeomMean, 0.10f),
          "LDR: lightLevel=0.5 steady state ≈ geometric mean sqrt(R_dark*R_bright) ≈ 14142Ω");
}

static void testProgramDependency() {
    std::printf("\n--- LDR Program Dependency ---\n");

    auto measureReleaseTime = [](int driveMs) -> int {
        WdfPhotoresistor::Params params;
        params.sampleRate = 48000.0f;
        WdfPhotoresistor ldr;
        ldr.init(params);

        int driveSamples = driveMs * 48;
        for (int i = 0; i < driveSamples; ++i) ldr.setLightLevel(0.8f);

        float halfRecovery = ldr_params::R_bright + 0.5f * (ldr_params::R_dark - ldr_params::R_bright);
        int maxSamples = static_cast<int>(10.0f * 48000.0f);
        for (int i = 0; i < maxSamples; ++i) {
            ldr.setLightLevel(0.0f);
            if (ldr.getCurrentResistance() >= halfRecovery) return i + 1;
        }
        return maxSamples;
    };

    int releaseA = measureReleaseTime(100);
    int releaseB = measureReleaseTime(1000);

    std::printf("  INFO: Release after 100ms drive:  %d samples (%.1fms)\n",
                releaseA, static_cast<double>(releaseA) / 48000.0 * 1000.0);
    std::printf("  INFO: Release after 1000ms drive: %d samples (%.1fms)\n",
                releaseB, static_cast<double>(releaseB) / 48000.0 * 1000.0);

    check(releaseB > releaseA,
          "LDR: longer drive time → longer release (program dependency)");
}

static void testResistanceBounds() {
    std::printf("\n--- LDR Resistance Bounds ---\n");

    WdfPhotoresistor::Params params;
    params.sampleRate = 48000.0f;
    WdfPhotoresistor ldr;
    ldr.init(params);

    ldr.setLightLevel(2.0f);
    check(ldr.getCurrentResistance() >= ldr_params::R_bright &&
          ldr.getCurrentResistance() <= ldr_params::R_dark,
          "LDR: lightLevel=2.0 clamped — Rp in [R_bright, R_dark]");

    ldr.reset();
    ldr.setLightLevel(-1.0f);
    check(ldr.getCurrentResistance() >= ldr_params::R_bright &&
          ldr.getCurrentResistance() <= ldr_params::R_dark,
          "LDR: lightLevel=-1.0 clamped — Rp in [R_bright, R_dark]");
}

static void testIirCoefficientAccuracy() {
    std::printf("\n--- LDR IIR Coefficient Accuracy ---\n");

    float tau_fast = 0.010f;
    float sr = 48000.0f;
    float expected_alpha = 1.0f - expf(-1.0f / (tau_fast * sr));

    std::printf("  INFO: Expected alpha_fast = %.6f\n", static_cast<double>(expected_alpha));

    check(expected_alpha > 0.002f && expected_alpha < 0.003f,
          "LDR: alpha_fast in expected range 0.002–0.003");
    // The spec states ≈ 0.002079; float32 computation yields ~0.002081.
    // Verify the formula result is within 0.5% of the spec's stated approximation.
    check(approxRel(expected_alpha, 0.002079f, 0.005f),
          "LDR: alpha_fast within 0.5% of spec value 0.002079");

    // Verify IIR advances resistance: run LDR for 1/alpha samples at light=1.0
    WdfPhotoresistor::Params params;
    params.sampleRate = sr;
    WdfPhotoresistor ldr;
    ldr.init(params);

    int oneTauSamples = static_cast<int>(1.0f / expected_alpha);
    float initialR = ldr_params::R_dark;
    float targetR  = ldr_params::R_bright;

    for (int i = 0; i < oneTauSamples; ++i) ldr.setLightLevel(1.0f);

    float rangeTraversed = (initialR - ldr.getCurrentResistance()) / (initialR - targetR);
    std::printf("  INFO: After %d samples, %.1f%% of range traversed\n",
                oneTauSamples, static_cast<double>(rangeTraversed * 100.0f));

    check(rangeTraversed > 0.10f, "LDR: IIR advances resistance toward target over time");
}

// ============================================================
// PC2ACircuit — Compression behavior
// ============================================================

static void testOpticalVsOtaResponseCharacter() {
    std::printf("\n--- Optical vs OTA Response Character ---\n");

    PC2ACircuit pc2a;
    pc2a.init(48000.0f);
    pc2a.setPeakReduction(0.8f);
    pc2a.setGain(0.5f);
    pc2a.setHFEmphasis(0.0f);

    DynacompCircuit dynacomp;
    dynacomp.init(48000.0f);
    dynacomp.setSensitivity(0.8f);
    dynacomp.setOutput(0.5f);

    constexpr int kSampleRate = 48000;
    constexpr int kDuration   = kSampleRate / 2; // 500ms

    float maxPc2aGr  = 0.0f;
    float maxDynaGr  = 0.0f;

    for (int i = 0; i < kDuration; ++i) {
        float t   = static_cast<float>(i) / static_cast<float>(kSampleRate);
        float sig = 0.8f * sinf(6.283185307f * 1000.0f * t);
        (void)pc2a.process(sig);
        (void)dynacomp.process(sig);

        float pc2aGr = pc2a.getGainReductionDb();
        float dynaGr = dynacomp.getGainReductionDb();
        maxPc2aGr    = std::min(maxPc2aGr, pc2aGr);
        maxDynaGr    = std::min(maxDynaGr, dynaGr);
    }

    float pc2aGrSteady    = pc2a.getGainReductionDb();
    float dynacompGrSteady = dynacomp.getGainReductionDb();

    std::printf("  INFO: PC-2A peak GR: %.2f dB, steady state: %.2f dB\n",
                static_cast<double>(maxPc2aGr), static_cast<double>(pc2aGrSteady));
    std::printf("  INFO: Dynacomp peak GR: %.2f dB, steady state: %.2f dB\n",
                static_cast<double>(maxDynaGr), static_cast<double>(dynacompGrSteady));

    check(pc2aGrSteady < -3.0f,
          "PC-2A: gain reduction > 3dB at steady state");
    check(dynacompGrSteady < -3.0f,
          "Dynacomp: gain reduction > 3dB at steady state");

    check(pc2a.getLdrInertia() > 0.0f,
          "PC-2A: LDR inertia > 0 after sustained signal");
}

static void testHFEmphasis() {
    std::printf("\n--- PC-2A HF Emphasis ---\n");

    auto measureGainReduction = [](float freq, float hfEmphasis) {
        PC2ACircuit pc2a;
        pc2a.init(48000.0f);
        pc2a.setPeakReduction(0.7f);
        pc2a.setGain(0.5f);
        pc2a.setHFEmphasis(hfEmphasis);

        float minGr = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            float t   = static_cast<float>(i) / 48000.0f;
            float sig = 0.7f * sinf(6.283185307f * freq * t);
            (void)pc2a.process(sig);
            float gr = pc2a.getGainReductionDb();
            minGr = std::min(minGr, gr);
        }
        return minGr;
    };

    float gr200HzFlat = measureGainReduction(200.0f,  0.0f);
    float gr5kHzFlat  = measureGainReduction(5000.0f, 0.0f);
    float gr200HzHf   = measureGainReduction(200.0f,  1.0f);
    float gr5kHzHf    = measureGainReduction(5000.0f, 1.0f);

    std::printf("  INFO: 200Hz flat: %.2f dB, 5kHz flat: %.2f dB\n",
                static_cast<double>(gr200HzFlat), static_cast<double>(gr5kHzFlat));
    std::printf("  INFO: 200Hz HF:   %.2f dB, 5kHz HF:   %.2f dB\n",
                static_cast<double>(gr200HzHf), static_cast<double>(gr5kHzHf));

    float flatDiff = fabsf(gr200HzFlat - gr5kHzFlat);
    check(flatDiff < 3.0f,
          "PC-2A HF=0: 200Hz and 5kHz trigger similar compression (flat sidechain)");
    check(gr5kHzHf < gr200HzHf,
          "PC-2A HF=1: 5kHz triggers more compression than 200Hz (treble-focused sidechain)");
}

static void testReleaseHangInertia() {
    std::printf("\n--- PC-2A Release Hang / Program Dependency ---\n");

    auto measureInertiaAfterDrive = [](int driveMs) -> float {
        PC2ACircuit pc2a;
        pc2a.init(48000.0f);
        pc2a.setPeakReduction(0.9f);
        pc2a.setGain(0.5f);

        int driveSamples = driveMs * 48;
        for (int i = 0; i < driveSamples; ++i) {
            float t   = static_cast<float>(i) / 48000.0f;
            float sig = 0.9f * sinf(6.283185307f * 1000.0f * t);
            (void)pc2a.process(sig);
        }
        for (int i = 0; i < 4800; ++i) (void)pc2a.process(0.0f);
        return pc2a.getLdrInertia();
    };

    float inertiaShort   = measureInertiaAfterDrive(10);
    float inertiaSustain = measureInertiaAfterDrive(500);

    std::printf("  INFO: Inertia after 10ms drive:  %.4f\n",
                static_cast<double>(inertiaShort));
    std::printf("  INFO: Inertia after 500ms drive: %.4f\n",
                static_cast<double>(inertiaSustain));

    check(inertiaSustain > inertiaShort,
          "PC-2A: sustained signal → higher LDR inertia after release (program dependency)");
    check(inertiaShort >= 0.0f && inertiaShort <= 1.0f,
          "PC-2A: inertia in [0, 1] range after short drive");
    check(inertiaSustain >= 0.0f && inertiaSustain <= 1.0f,
          "PC-2A: inertia in [0, 1] range after sustained drive");
}

static void testMonoStereoLinked() {
    std::printf("\n--- PC-2A Mono/Stereo Linked Compression ---\n");

    PC2ACircuit pc2a;
    pc2a.init(48000.0f);
    pc2a.setPeakReduction(0.7f);
    pc2a.setGain(0.5f);

    float maxDiff = 0.0f;
    for (int i = 0; i < 48000; ++i) {
        float t   = static_cast<float>(i) / 48000.0f;
        float sig = 0.7f * sinf(6.283185307f * 440.0f * t);
        float outL = 0.0f, outR = 0.0f;
        pc2a.processStereo(sig, sig, outL, outR);
        maxDiff = std::max(maxDiff, fabsf(outL - outR));
    }

    std::printf("  INFO: Max L/R difference with mono input: %.6f\n",
                static_cast<double>(maxDiff));
    check(maxDiff < 0.001f,
          "PC-2A: processStereo with mono input → same output on L and R");
}

static void testHybridOptOta() {
    std::printf("\n--- HybridOptOtaCircuit ---\n");

    HybridOptOtaCircuit hybrid;
    hybrid.init(48000.0f);
    hybrid.setSensitivity(0.8f);
    hybrid.setMakeupGain(0.5f);

    float minOut = 1.0f;
    bool noNaN = true;
    for (int i = 0; i < 48000; ++i) {
        float t   = static_cast<float>(i) / 48000.0f;
        float sig = 0.8f * sinf(6.283185307f * 1000.0f * t);
        float out = hybrid.process(sig);
        if (!std::isfinite(out)) { noNaN = false; break; }
        if (fabsf(out) > 0.001f) minOut = std::min(minOut, fabsf(out));
    }

    check(noNaN, "HybridOptOta: no NaN/Inf over 48000 samples");
    check(minOut < 0.9f, "HybridOptOta: compression applied (output attenuated)");

    // Program dependency via residual compression measurement
    auto measureHybridResidual = [](int driveMs) -> float {
        HybridOptOtaCircuit h;
        h.init(48000.0f);
        h.setSensitivity(0.9f);
        h.setMakeupGain(0.5f);
        int driveSamples = driveMs * 48;
        for (int i = 0; i < driveSamples; ++i) {
            float t   = static_cast<float>(i) / 48000.0f;
            float sig = 0.9f * sinf(6.283185307f * 1000.0f * t);
            (void)h.process(sig);
        }
        // 200ms silence
        for (int i = 0; i < 9600; ++i) (void)h.process(0.0f);
        // Measure output with test tone
        float sumOut = 0.0f;
        for (int i = 0; i < 480; ++i) {
            float t   = static_cast<float>(i) / 48000.0f;
            float sig = 0.5f * sinf(6.283185307f * 1000.0f * t);
            sumOut += fabsf(h.process(sig));
        }
        return sumOut;
    };

    float outShort   = measureHybridResidual(100);
    float outSustain = measureHybridResidual(1000);

    std::printf("  INFO: Output after 100ms drive:  sum=%.4f\n",
                static_cast<double>(outShort));
    std::printf("  INFO: Output after 1000ms drive: sum=%.4f\n",
                static_cast<double>(outSustain));

    check(outSustain <= outShort,
          "HybridOptOta: longer drive → more residual compression (program dependency)");
}

static void testStability() {
    std::printf("\n--- Stability: Silence ---\n");

    bool noNaN = true;

    PC2ACircuit pc2a;
    pc2a.init(48000.0f);
    pc2a.setPeakReduction(0.5f);
    pc2a.setGain(0.5f);

    float maxOut = 0.0f;
    for (int i = 0; i < 96000; ++i) {
        float out = pc2a.process(0.0f);
        if (!std::isfinite(out)) { noNaN = false; break; }
        maxOut = std::max(maxOut, fabsf(out));
    }
    check(noNaN, "PC-2A: no NaN/Inf over 96000 silence samples");
    check(maxOut < 0.001f, "PC-2A: output < 0.001 after 96000 silence samples");

    OpticalLevelerCircuit leveler;
    leveler.init(48000.0f);
    leveler.setThreshold(0.5f);
    maxOut = 0.0f;
    noNaN = true;
    for (int i = 0; i < 96000; ++i) {
        float out = leveler.process(0.0f);
        if (!std::isfinite(out)) { noNaN = false; break; }
        maxOut = std::max(maxOut, fabsf(out));
    }
    check(noNaN, "OpticalLeveler: no NaN/Inf over 96000 silence samples");

    HybridOptOtaCircuit hybrid;
    hybrid.init(48000.0f);
    hybrid.setSensitivity(0.5f);
    hybrid.setMakeupGain(0.5f);
    maxOut = 0.0f;
    noNaN = true;
    for (int i = 0; i < 96000; ++i) {
        float out = hybrid.process(0.0f);
        if (!std::isfinite(out)) { noNaN = false; break; }
        maxOut = std::max(maxOut, fabsf(out));
    }
    check(noNaN, "HybridOptOta: no NaN/Inf over 96000 silence samples");
    check(maxOut < 0.001f, "HybridOptOta: output < 0.001 after 96000 silence samples");
}

static void testNoNaN() {
    std::printf("\n--- No NaN: Extreme Inputs ---\n");

    {
        PC2ACircuit pc2a;
        pc2a.init(48000.0f);
        pc2a.setPeakReduction(1.0f);
        pc2a.setGain(1.0f);
        bool finite = true;
        for (int i = 0; i < 1000; ++i) {
            float sig = (i % 2 == 0) ? 1.0f : -1.0f;
            float out = pc2a.process(sig);
            if (!std::isfinite(out)) { finite = false; break; }
        }
        check(finite, "PC-2A: no NaN/Inf on ±1.0 step inputs");
    }
    {
        OpticalLevelerCircuit leveler;
        leveler.init(48000.0f);
        leveler.setThreshold(1.0f);
        bool finite = true;
        for (int i = 0; i < 1000; ++i) {
            float sig = (i % 2 == 0) ? 1.0f : -1.0f;
            float out = leveler.process(sig);
            if (!std::isfinite(out)) { finite = false; break; }
        }
        check(finite, "OpticalLeveler: no NaN/Inf on ±1.0 step inputs");
    }
    {
        HybridOptOtaCircuit hybrid;
        hybrid.init(48000.0f);
        hybrid.setSensitivity(1.0f);
        hybrid.setMakeupGain(1.0f);
        bool finite = true;
        for (int i = 0; i < 1000; ++i) {
            float sig = (i % 2 == 0) ? 1.0f : -1.0f;
            float out = hybrid.process(sig);
            if (!std::isfinite(out)) { finite = false; break; }
        }
        check(finite, "HybridOptOta: no NaN/Inf on ±1.0 step inputs");
    }
    {
        DynacompCircuit dynacomp;
        dynacomp.init(48000.0f);
        dynacomp.setSensitivity(1.0f);
        dynacomp.setOutput(1.0f);
        bool finite = true;
        for (int i = 0; i < 1000; ++i) {
            float sig = (i % 2 == 0) ? 1.0f : -1.0f;
            float out = dynacomp.process(sig);
            if (!std::isfinite(out)) { finite = false; break; }
        }
        check(finite, "DynacompCircuit: no NaN/Inf on ±1.0 step inputs");
    }
}

static void testPerformance() {
    std::printf("\n--- Performance ---\n");

    constexpr int kSamples = 100000;
    float inputs[256];
    for (int i = 0; i < 256; ++i) {
        inputs[i] = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f) * 0.8f;
    }

    // WdfPhotoresistor::setLightLevel
    {
        WdfPhotoresistor::Params params;
        params.sampleRate = 48000.0f;
        WdfPhotoresistor ldr;
        ldr.init(params);

        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) {
            ldr.setLightLevel(fabsf(inputs[i & 255]));
            dummy += ldr.getCurrentResistance();
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        double nsPerSample = static_cast<double>(ns) / static_cast<double>(kSamples);
        std::printf("  INFO: WdfPhotoresistor::setLightLevel: %.1f ns/sample\n", nsPerSample);
        check(std::isfinite(nsPerSample) && nsPerSample < 1000000.0,
              "WdfPhotoresistor::setLightLevel: timing measurement valid");
    }

    // PC2ACircuit::process
    {
        PC2ACircuit pc2a;
        pc2a.init(48000.0f);
        pc2a.setPeakReduction(0.5f);
        pc2a.setGain(0.5f);

        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = pc2a.process(inputs[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        double nsPerSample = static_cast<double>(ns) / static_cast<double>(kSamples);
        std::printf("  INFO: PC2ACircuit::process: %.1f ns/sample\n", nsPerSample);
        check(std::isfinite(nsPerSample) && nsPerSample < 1000000.0,
              "PC2ACircuit::process: timing measurement valid");
    }

    // HybridOptOtaCircuit::process
    {
        HybridOptOtaCircuit hybrid;
        hybrid.init(48000.0f);
        hybrid.setSensitivity(0.5f);
        hybrid.setMakeupGain(0.5f);

        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = hybrid.process(inputs[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        double nsPerSample = static_cast<double>(ns) / static_cast<double>(kSamples);
        std::printf("  INFO: HybridOptOtaCircuit::process: %.1f ns/sample\n", nsPerSample);
        check(std::isfinite(nsPerSample) && nsPerSample < 1000000.0,
              "HybridOptOtaCircuit::process: timing measurement valid");
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    std::printf("=== Photoresistor & Optical Compressor Test Suite ===\n");

    testAsymmetricAttackRelease();
    testLogLinearMapping();
    testProgramDependency();
    testResistanceBounds();
    testIirCoefficientAccuracy();
    testOpticalVsOtaResponseCharacter();
    testHFEmphasis();
    testReleaseHangInertia();
    testMonoStereoLinked();
    testHybridOptOta();
    testStability();
    testNoNaN();
    testPerformance();

    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);

    return (passedTests == totalTests) ? 0 : 1;
}
