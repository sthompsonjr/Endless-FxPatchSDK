#include <cstdio>
#include <cmath>
#include <cassert>
#include "WdfOpAmpBigMuffCircuit.h"

// ── Test 1: Instantiation and silence ────────────────────────────────────

void test_opamp_bm_instantiation() {
    WdfOpAmpBigMuffCircuit c;
    c.init(48000.0f);
    c.setSustain(0.5f);
    c.setTone(0.5f);
    c.setVolume(1.0f);
    c.reset();
    // Zero input → zero (or near-zero) output after settling
    float lastOut = 0.0f;
    for (int i = 0; i < 500; ++i) {
        lastOut = c.process(0.0f);
    }
    assert(fabsf(lastOut) < 1e-4f &&
           "Op-Amp Big Muff: nonzero output for zero input after settling");
    printf("PASS: test_opamp_bm_instantiation\n");
}

// ── Test 2: Sustain increases output level (sub-clipping) ─────────────────

void test_opamp_bm_sustain_gain() {
    auto measureRMS = [](float sustain) {
        WdfOpAmpBigMuffCircuit c;
        c.init(48000.0f);
        c.setSustain(sustain);
        c.setTone(0.5f);
        c.setVolume(1.0f);
        c.reset();
        for (int i = 0; i < 500; ++i) (void)c.process(0.0f);

        float sum = 0.0f;
        const float sr = 48000.0f;
        for (int i = 0; i < 2400; ++i) {
            float x = 0.01f * sinf(2.0f * 3.14159265f * 440.0f * (float)i / sr);
            float y = c.process(x);
            sum += y * y;
        }
        return sqrtf(sum / 2400.0f);
    };

    float rmsLow  = measureRMS(0.1f);
    float rmsMid  = measureRMS(0.5f);
    float rmsHigh = measureRMS(0.9f);
    assert(rmsHigh >= rmsMid && rmsMid >= rmsLow &&
           "Op-Amp Big Muff: sustain does not monotonically increase gain");
    printf("PASS: test_opamp_bm_sustain_gain "
           "(low=%.5f mid=%.5f high=%.5f)\n", (double)rmsLow, (double)rmsMid, (double)rmsHigh);
}

// ── Test 3: Diode clipping engages at expected threshold ──────────────────

void test_opamp_bm_diode_clipping_threshold() {
    auto measurePeak = [](float inputAmp, float sustain) {
        WdfOpAmpBigMuffCircuit c;
        c.init(48000.0f);
        c.setSustain(sustain);
        c.setTone(0.5f);
        c.setVolume(1.0f);
        c.reset();
        for (int i = 0; i < 500; ++i) (void)c.process(0.0f);
        float peak = 0.0f;
        const float sr = 48000.0f;
        for (int i = 0; i < 4800; ++i) {
            float x = inputAmp * sinf(2.0f * 3.14159265f * 440.0f * (float)i / sr);
            float y = c.process(x);
            if (fabsf(y) > peak) peak = fabsf(y);
        }
        return peak;
    };

    float peak1 = measurePeak(0.1f, 0.5f);
    float peak2 = measurePeak(0.2f, 0.5f);
    float ratio = peak2 / (peak1 + 1e-8f);
    assert(ratio < 1.9f &&
           "Op-Amp Big Muff: diode clipping not engaging (ratio too close to linear)");
    printf("PASS: test_opamp_bm_diode_clipping_threshold "
           "(peak1=%.4f peak2=%.4f ratio=%.3f)\n",
           (double)peak1, (double)peak2, (double)ratio);
}

// ── Test 4: Tone bypass passes wider bandwidth ────────────────────────────

void test_opamp_bm_tone_bypass() {
    auto measureFreqEnergy = [](float freq, bool bypass) {
        WdfOpAmpBigMuffCircuit c;
        c.init(48000.0f);
        c.setSustain(0.5f);
        c.setTone(0.5f);
        c.setVolume(1.0f);
        c.setToneBypass(bypass);
        c.reset();
        for (int i = 0; i < 500; ++i) (void)c.process(0.0f);
        float sum = 0.0f;
        const float sr = 48000.0f;
        for (int i = 0; i < 4800; ++i) {
            float x = 0.1f * sinf(2.0f * 3.14159265f * freq * (float)i / sr);
            float y = c.process(x);
            sum += y * y;
        }
        return sum / 4800.0f;
    };

    float energy5k_normal = measureFreqEnergy(5000.0f, false);
    float energy5k_bypass = measureFreqEnergy(5000.0f, true);
    assert(energy5k_bypass >= energy5k_normal &&
           "Op-Amp Big Muff: tone bypass must pass >= energy vs tone stack at 5kHz");
    printf("PASS: test_opamp_bm_tone_bypass "
           "(normal=%.6f bypass=%.6f)\n",
           (double)energy5k_normal, (double)energy5k_bypass);
}

// ── Test 5: Numerical stability at clipping extremes ──────────────────────

void test_opamp_bm_stability_extreme_input() {
    WdfOpAmpBigMuffCircuit c;
    c.init(48000.0f);
    c.setSustain(1.0f);
    c.setTone(0.5f);
    c.setVolume(1.0f);
    c.reset();
    bool stable = true;
    for (int i = 0; i < 4800; ++i) {
        float x = (i % 2 == 0) ? 1.0f : -1.0f;
        float y = c.process(x);
        if (!std::isfinite(y)) { stable = false; break; }
    }
    assert(stable && "Op-Amp Big Muff: NaN or Inf at extreme input");
    printf("PASS: test_opamp_bm_stability_extreme_input\n");
}

// ── Test 6: LM741 slew rate creates characteristic soft transient ─────────

void test_opamp_bm_slew_rate_effect() {
    WdfOpAmpBigMuffCircuit c;
    c.init(48000.0f);
    c.setSustain(1.0f);
    c.setTone(0.5f);
    c.setVolume(1.0f);
    c.reset();
    for (int i = 0; i < 200; ++i) (void)c.process(0.0f);
    float firstStep = c.process(1.0f);
    float secondStep = c.process(1.0f);
    float tenthStep = 0.0f;
    for (int i = 2; i < 10; ++i) tenthStep = c.process(1.0f);
    assert(fabsf(firstStep) < fabsf(tenthStep) + 1e-4f &&
           "Op-Amp Big Muff: no slew-rate effect on step input");
    printf("PASS: test_opamp_bm_slew_rate_effect "
           "(first=%.4f second=%.4f tenth=%.4f)\n",
           (double)firstStep, (double)secondStep, (double)tenthStep);
}

// ── Main ──────────────────────────────────────────────────────────────────

int main() {
    test_opamp_bm_instantiation();
    test_opamp_bm_sustain_gain();
    test_opamp_bm_diode_clipping_threshold();
    test_opamp_bm_tone_bypass();
    test_opamp_bm_stability_extreme_input();
    test_opamp_bm_slew_rate_effect();
    printf("\nAll Op-Amp Big Muff tests passed.\n");
    return 0;
}
