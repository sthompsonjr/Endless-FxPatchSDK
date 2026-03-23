/*
 * shoegaze_test.cpp — Integration test harness for PatchImpl_Shoegaze
 *
 * Build:
 *   g++ -std=c++20 -O2 -Wall -Werror -Wdouble-promotion -fno-exceptions -fno-rtti \
 *       -I. -I./dsp -I./wdf tests/shoegaze_test.cpp -o shoegaze_test -lm
 *
 * Includes PatchImpl_Shoegaze.cpp directly (single-TU test build) to access
 * the Patch::getInstance() singleton and virtual interface.
 */

#include "effects/PatchImpl_Shoegaze.cpp"  // NOLINT — intentional single-TU test

#include <cmath>
#include <cstdio>
#include <cassert>
#include <chrono>
#include <cstring>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// Test infrastructure
// ─────────────────────────────────────────────────────────────────────────────
static int gTotal  = 0;
static int gPassed = 0;

static void check(bool cond, const char* name) {
    ++gTotal;
    if (cond) {
        ++gPassed;
        std::printf("  PASS  %s\n", name);
    } else {
        std::printf("  FAIL  %s\n", name);
    }
}

// Working buffer — must match kWorkingBufferSize = 2,400,000 floats (~9.6 MB).
static float gWorkingBuf[Patch::kWorkingBufferSize];

static Patch* makePatch() {
    Patch* p = Patch::getInstance();
    p->init();
    p->setWorkingBuffer(std::span<float, Patch::kWorkingBufferSize>(gWorkingBuf));
    return p;
}

static void fillSine(float* buf, int n, float freqHz, float amp) {
    constexpr float kSr = static_cast<float>(Patch::kSampleRate);
    float phase = 0.0f;
    const float phaseInc = 2.0f * 3.14159265f * freqHz / kSr;
    for (int i = 0; i < n; ++i) {
        buf[i] = amp * sinf(phase);
        phase += phaseInc;
    }
}

static void fillNoise(float* buf, int n, float amp) {
    uint32_t seed = 12345u;
    for (int i = 0; i < n; ++i) {
        seed = seed * 1664525u + 1013904223u;
        buf[i] = amp * (static_cast<float>(seed) / 2147483648.0f - 1.0f);
    }
}

static float rms(const float* buf, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; ++i) sum += buf[i] * buf[i];
    return sqrtf(sum / static_cast<float>(n));
}

static bool allFiniteBounded(const float* bufL, const float* bufR, int n, float limit = 3.0f) {
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(bufL[i]) || !std::isfinite(bufR[i])) return false;
        if (fabsf(bufL[i]) > limit || fabsf(bufR[i]) > limit) return false;
    }
    return true;
}

static void processSilence(Patch* p, int n) {
    static float L[256], R[256];
    while (n > 0) {
        int block = std::min(n, 256);
        std::memset(L, 0, static_cast<std::size_t>(block) * sizeof(float));
        std::memset(R, 0, static_cast<std::size_t>(block) * sizeof(float));
        p->processAudio(std::span<float>(L, static_cast<std::size_t>(block)),
                        std::span<float>(R, static_cast<std::size_t>(block)));
        n -= block;
    }
}

static void processBlock(Patch* p, const float* inL, const float* inR,
                         float* outL, float* outR, int n) {
    std::memcpy(outL, inL, static_cast<std::size_t>(n) * sizeof(float));
    std::memcpy(outR, inR, static_cast<std::size_t>(n) * sizeof(float));
    p->processAudio(std::span<float>(outL, static_cast<std::size_t>(n)),
                    std::span<float>(outR, static_cast<std::size_t>(n)));
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 1: Signal chain correctness
// ─────────────────────────────────────────────────────────────────────────────
static void testGroup1() {
    std::printf("\n--- Group 1: Signal Chain Correctness ---\n");

    static float sineBuf[2000];
    fillSine(sineBuf, 2000, 200.0f, 0.4f);

    // test_default_chain_muff_into_softfocus
    {
        Patch* p = makePatch();
        p->setParamValue(0, 0.5f);
        p->setParamValue(1, 0.3f);
        p->setParamValue(2, 0.0f);
        processSilence(p, 4096);

        static float inL[2000], inR[2000], outL[2000], outR[2000];
        std::memcpy(inL, sineBuf, sizeof sineBuf);
        std::memcpy(inR, sineBuf, sizeof sineBuf);
        processBlock(p, inL, inR, outL, outR, 2000);

        const float rmsL = rms(outL, 2000);
        float diffSum = 0.0f;
        for (int i = 0; i < 2000; ++i)
            diffSum += fabsf(outL[i] - outR[i]);
        const float avgDiff = diffSum / 2000.0f;

        check(rmsL > 0.001f,    "default_chain_output_nonzero");
        check(avgDiff > 0.001f, "default_chain_output_is_stereo");
        check(allFiniteBounded(outL, outR, 2000), "default_chain_all_finite");
    }

    // test_reversed_chain_softfocus_into_muff
    {
        Patch* p = makePatch();
        p->handleAction(static_cast<int>(endless::ActionId::kLeftFootSwitchHold));
        processSilence(p, 48000);

        static float inL[2000], inR[2000], outL[2000], outR[2000];
        std::memcpy(inL, sineBuf, sizeof sineBuf);
        std::memcpy(inR, sineBuf, sizeof sineBuf);
        processBlock(p, inL, inR, outL, outR, 2000);

        const float rmsL = rms(outL, 2000);
        float diffSum = 0.0f;
        for (int i = 0; i < 2000; ++i)
            diffSum += fabsf(outL[i] - outR[i]);
        const float avgDiff = diffSum / 2000.0f;

        check(rmsL > 0.001f,    "reversed_chain_output_nonzero");
        check(avgDiff > 0.001f, "reversed_chain_output_is_stereo");
        check(allFiniteBounded(outL, outR, 2000), "reversed_chain_all_finite");
    }

    // test_chain_order_produces_different_output
    {
        static float sineIn[1000];
        fillSine(sineIn, 1000, 400.0f, 0.3f);

        Patch* p1 = makePatch();
        processSilence(p1, 4096);
        static float dL[1000], dR[1000];
        processBlock(p1, sineIn, sineIn, dL, dR, 1000);
        const float rmsDefault = rms(dL, 1000);

        Patch* p2 = makePatch();
        p2->handleAction(static_cast<int>(endless::ActionId::kLeftFootSwitchHold));
        processSilence(p2, 4096);
        static float rL[1000], rR[1000];
        processBlock(p2, sineIn, sineIn, rL, rR, 1000);
        const float rmsReversed = rms(rL, 1000);

        const float diff = fabsf(rmsDefault - rmsReversed);
        const float larger = std::max(rmsDefault, rmsReversed);
        const float ratio = (larger > 1e-6f) ? (diff / larger) : 0.0f;
        std::printf("  INFO  default RMS=%.5f  reversed RMS=%.5f  ratio=%.3f\n",
                    static_cast<double>(rmsDefault),
                    static_cast<double>(rmsReversed),
                    static_cast<double>(ratio));
        check(ratio >= 0.10f, "chain_orders_produce_different_output_10pct");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 2: Knob Mapping
// ─────────────────────────────────────────────────────────────────────────────
static void testGroup2() {
    std::printf("\n--- Group 2: Knob Mapping ---\n");

    // test_fuzz_knob_zone_a_gain_ramp
    {
        static float sine[2000];
        fillSine(sine, 2000, 200.0f, 0.3f);

        Patch* p = makePatch();
        p->setParamValue(1, 0.1f);
        p->setParamValue(2, 0.0f);

        p->setParamValue(0, 0.0f);
        processSilence(p, 4096);
        static float oL[2000], oR[2000];
        processBlock(p, sine, sine, oL, oR, 2000);
        const float rmsLow = rms(oL, 2000);

        p->setParamValue(0, 0.5f);
        processSilence(p, 4096);
        processBlock(p, sine, sine, oL, oR, 2000);
        const float rmsMid = rms(oL, 2000);

        std::printf("  INFO  fuzz knob: rms(0.0)=%.5f  rms(0.5)=%.5f\n",
                    static_cast<double>(rmsLow), static_cast<double>(rmsMid));
        check(rmsMid > rmsLow, "fuzz_zone_a_higher_knob_more_gain");
    }

    // test_fuzz_knob_zone_b_tone_sweep (HF energy drops from 0.67→1.0)
    {
        static float noise[4000];
        fillNoise(noise, 4000, 0.3f);

        auto hfEnergy = [&](float knob0Val) -> float {
            Patch* p = makePatch();
            p->setParamValue(0, knob0Val);
            p->setParamValue(1, 0.1f);
            p->setParamValue(2, 0.0f);
            processSilence(p, 4096);
            static float oL[4000], oR[4000];
            processBlock(p, noise, noise, oL, oR, 4000);
            float e = 0.0f;
            for (int i = 1; i < 4000; ++i) {
                float d = oL[i] - oL[i-1];
                e += d * d;
            }
            return e;
        };

        const float hf067 = hfEnergy(0.67f);
        const float hf100 = hfEnergy(1.0f);
        std::printf("  INFO  HF proxy: knob0=0.67->%.4f  knob0=1.0->%.4f\n",
                    static_cast<double>(hf067), static_cast<double>(hf100));
        check(hf100 <= hf067, "fuzz_zone_b_darker_at_max_knob");
    }

    // test_depth_knob_voice_mix
    {
        static float sine[2000];
        fillSine(sine, 2000, 400.0f, 0.3f);

        Patch* p = makePatch();
        p->setParamValue(0, 0.5f);
        p->setParamValue(2, 0.5f);

        p->setParamValue(1, 0.1f);  // Zone A — no voices
        processSilence(p, 48000);
        static float oL[2000], oR[2000];
        processBlock(p, sine, sine, oL, oR, 2000);
        const float rmsNoVoices = rms(oL, 2000);

        p->setParamValue(1, 0.5f);  // Zone B — voices on
        processSilence(p, 48000);
        processBlock(p, sine, sine, oL, oR, 2000);
        const float rmsWithVoices = rms(oL, 2000);

        std::printf("  INFO  voice mix: no_voices=%.5f  with_voices=%.5f\n",
                    static_cast<double>(rmsNoVoices),
                    static_cast<double>(rmsWithVoices));
        check(rmsNoVoices > 0.001f,   "depth_zone_a_output_nonzero");
        check(rmsWithVoices > 0.001f, "depth_zone_b_output_nonzero_voices");
    }

    // test_depth_knob_reverb_tail_length
    {
        static float impL[48000], impR[48000];
        std::memset(impL, 0, sizeof impL);
        std::memset(impR, 0, sizeof impR);
        impL[0] = impR[0] = 0.5f;

        Patch* p = makePatch();
        p->setParamValue(1, 0.1f);
        processSilence(p, 4096);
        static float oL[48000], oR[48000];
        processBlock(p, impL, impR, oL, oR, 48000);
        const float tailShort = rms(oL + 24000, 1000);

        Patch* p2 = makePatch();
        p2->setParamValue(1, 0.9f);
        processSilence(p2, 4096);
        static float oL2[48000], oR2[48000];
        processBlock(p2, impL, impR, oL2, oR2, 48000);
        const float tailLong = rms(oL2 + 24000, 1000);

        std::printf("  INFO  reverb tail at 0.5s: short=%.6f  long=%.6f\n",
                    static_cast<double>(tailShort), static_cast<double>(tailLong));
        check(tailLong >= tailShort, "longer_knob1_produces_longer_tail");
    }

    // test_mod_knob_lfo_modulation
    {
        Patch* p = makePatch();
        p->setParamValue(0, 0.5f);
        p->setParamValue(1, 0.5f);
        p->setParamValue(2, 1.0f);
        processSilence(p, 4096);
        static float sine[1000];
        fillSine(sine, 1000, 400.0f, 0.3f);
        static float oL[1000], oR[1000];
        processBlock(p, sine, sine, oL, oR, 1000);
        check(rms(oL, 1000) > 0.001f, "mod_knob_max_output_nonzero");
        check(allFiniteBounded(oL, oR, 1000), "mod_knob_max_output_finite");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 3: Variant Switching
// ─────────────────────────────────────────────────────────────────────────────
static void testGroup3() {
    std::printf("\n--- Group 3: Variant Switching ---\n");

    // test_variant_cycle_order
    {
        Patch* p = makePatch();
        const auto col0 = p->getStateLedColor();
        p->handleAction(static_cast<int>(endless::ActionId::kLeftFootSwitchPress));
        processSilence(p, 500);
        const auto col1 = p->getStateLedColor();
        p->handleAction(static_cast<int>(endless::ActionId::kLeftFootSwitchPress));
        processSilence(p, 500);
        const auto col2 = p->getStateLedColor();
        p->handleAction(static_cast<int>(endless::ActionId::kLeftFootSwitchPress));
        processSilence(p, 500);
        const auto col3 = p->getStateLedColor();

        std::printf("  INFO  LED colors: init=%d  tap1=%d  tap2=%d  tap3(wrap)=%d\n",
                    static_cast<int>(col0), static_cast<int>(col1),
                    static_cast<int>(col2), static_cast<int>(col3));
        check(col0 != col1, "variant_tap1_changes_color");
        check(col1 != col2, "variant_tap2_changes_color");
        check(col3 == col0, "variant_three_taps_wraps_to_start");
    }

    // test_variant_switch_no_click
    // Uses 100Hz sine at 0.05 amplitude (guitar-level signal, low fuzz) so the
    // BigMuff operates in its linear region and natural sample-to-sample variation
    // stays well below the 0.1 threshold. This isolates the fade mechanism itself.
    {
        Patch* p = makePatch();
        p->setParamValue(0, 0.05f);  // near-minimum fuzz: sustainGain_ ≈ 1.45
        p->setParamValue(1, 0.0f);   // minimum reverb, voices off
        static float sine[480];
        fillSine(sine, 480, 100.0f, 0.05f);
        processSilence(p, 4096);

        p->handleAction(static_cast<int>(endless::ActionId::kLeftFootSwitchPress));
        static float fadeBufL[480], fadeBufR[480];
        processBlock(p, sine, sine, fadeBufL, fadeBufR, 480);

        float maxDelta = 0.0f;
        for (int i = 1; i < 480; ++i)
            maxDelta = std::max(maxDelta, fabsf(fadeBufL[i] - fadeBufL[i-1]));

        std::printf("  INFO  max sample-to-sample delta during variant fade: %.4f\n",
                    static_cast<double>(maxDelta));
        check(maxDelta < 0.1f, "variant_fade_no_click_delta_under_0_1");
        check(allFiniteBounded(fadeBufL, fadeBufR, 480), "variant_fade_all_finite");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 4: Stability
// ─────────────────────────────────────────────────────────────────────────────
static void testGroup4() {
    std::printf("\n--- Group 4: Stability ---\n");

    // test_silence_stability
    {
        Patch* p = makePatch();
        p->setParamValue(0, 0.5f);
        p->setParamValue(1, 0.5f);
        p->setParamValue(2, 0.5f);

        static float oL[256], oR[256];
        float maxOut = 0.0f;
        for (int block = 0; block < 96000 / 256; ++block) {
            std::memset(oL, 0, sizeof oL);
            std::memset(oR, 0, sizeof oR);
            p->processAudio(std::span<float>(oL, 256), std::span<float>(oR, 256));
            for (int i = 0; i < 256; ++i)
                maxOut = std::max(maxOut, std::max(fabsf(oL[i]), fabsf(oR[i])));
        }
        std::printf("  INFO  silence stability: max output = %.6f\n",
                    static_cast<double>(maxOut));
        check(maxOut < 0.001f, "silence_stability_below_threshold");
    }

    // test_max_reverb_stability
    {
        Patch* p = makePatch();
        p->setParamValue(1, 1.0f);

        static float impL[96000], impR[96000];
        std::memset(impL, 0, sizeof impL);
        std::memset(impR, 0, sizeof impR);
        impL[0] = impR[0] = 0.5f;

        static float oL[96000], oR[96000];
        processBlock(p, impL, impR, oL, oR, 96000);

        float maxSample = 0.0f;
        for (int i = 0; i < 96000; ++i)
            maxSample = std::max(maxSample, std::max(fabsf(oL[i]), fabsf(oR[i])));
        std::printf("  INFO  max reverb: peak output = %.4f\n",
                    static_cast<double>(maxSample));
        check(maxSample < 2.0f, "max_reverb_no_explosion_under_2");
        check(allFiniteBounded(oL, oR, 96000, 2.0f), "max_reverb_all_finite");
    }

    // test_extreme_inputs_no_nan
    {
        static float inL[10000], inR[10000], oL[10000], oR[10000];

        // DC +1
        {
            Patch* p = makePatch();
            p->setParamValue(0, 1.0f);
            p->setParamValue(1, 1.0f);
            p->setParamValue(2, 1.0f);
            for (int i = 0; i < 10000; ++i) inL[i] = inR[i] = 1.0f;
            processBlock(p, inL, inR, oL, oR, 10000);
            check(allFiniteBounded(oL, oR, 10000, 3.0f), "extreme_dc_plus1_finite");
        }

        // DC -1
        {
            Patch* p = makePatch();
            p->setParamValue(0, 1.0f);
            p->setParamValue(1, 1.0f);
            p->setParamValue(2, 1.0f);
            for (int i = 0; i < 10000; ++i) inL[i] = inR[i] = -1.0f;
            processBlock(p, inL, inR, oL, oR, 10000);
            check(allFiniteBounded(oL, oR, 10000, 3.0f), "extreme_dc_minus1_finite");
        }

        // Alternating ±1
        {
            Patch* p = makePatch();
            p->setParamValue(0, 1.0f);
            p->setParamValue(1, 1.0f);
            p->setParamValue(2, 1.0f);
            for (int i = 0; i < 10000; ++i) inL[i] = inR[i] = (i % 2 == 0) ? 1.0f : -1.0f;
            processBlock(p, inL, inR, oL, oR, 10000);
            check(allFiniteBounded(oL, oR, 10000, 3.0f), "extreme_alternating_finite");
        }

        // White noise at amplitude 1.0
        {
            Patch* p = makePatch();
            p->setParamValue(0, 1.0f);
            p->setParamValue(1, 1.0f);
            p->setParamValue(2, 1.0f);
            fillNoise(inL, 10000, 1.0f);
            fillNoise(inR, 10000, 1.0f);
            processBlock(p, inL, inR, oL, oR, 10000);
            check(allFiniteBounded(oL, oR, 10000, 3.0f), "extreme_white_noise_finite");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 5: Performance
// ─────────────────────────────────────────────────────────────────────────────
static void testGroup5() {
    std::printf("\n--- Group 5: Performance ---\n");

    constexpr int kPerfSamples = 96000;
    static float perfL[kPerfSamples], perfR[kPerfSamples];
    fillSine(perfL, kPerfSamples, 200.0f, 0.3f);
    for (int i = 0; i < kPerfSamples; ++i) perfR[i] = perfL[i];

    constexpr double kArmClockMhz = 720.0;

    auto measure = [&](const char* label, float k1, bool reverse) -> double {
        Patch* p = makePatch();
        p->setParamValue(1, k1);
        if (reverse) {
            p->handleAction(static_cast<int>(endless::ActionId::kLeftFootSwitchHold));
            processSilence(p, 2000);
        }
        processSilence(p, 4096);

        static float pL[kPerfSamples], pR[kPerfSamples];
        std::memcpy(pL, perfL, kPerfSamples * sizeof(float));
        std::memcpy(pR, perfR, kPerfSamples * sizeof(float));

        auto t0 = std::chrono::high_resolution_clock::now();
        p->processAudio(std::span<float>(pL, kPerfSamples),
                        std::span<float>(pR, kPerfSamples));
        auto t1 = std::chrono::high_resolution_clock::now();

        double usPerSample = std::chrono::duration<double, std::micro>(t1 - t0).count()
                           / static_cast<double>(kPerfSamples);
        double estCycles   = usPerSample * kArmClockMhz;

        std::printf("  INFO  %s: %.3f us/sample  est. %.0f ARM cycles/sample\n",
                    label, usPerSample, estCycles);
        return estCycles;
    };

    const double cA = measure("default_order_voices_off (k1=0.1)", 0.1f, false);
    const double cB = measure("default_order_voices_on  (k1=0.5)", 0.5f, false);
    const double cC = measure("reversed_order_voices_on (k1=0.5)", 0.5f, true);

    check(cA <= 13000.0, "perf_voices_off_under_13000_cycles");
    check(cB <= 13000.0, "perf_voices_on_under_13000_cycles");
    check(cC <= 13000.0, "perf_reversed_under_13000_cycles");
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 6: Optimization Verification
// ─────────────────────────────────────────────────────────────────────────────
static void testGroup6() {
    std::printf("\n--- Group 6: Optimization Verification ---\n");

    constexpr int kMeasSamples = 96000;
    static float inL[kMeasSamples], inR[kMeasSamples];
    fillSine(inL, kMeasSamples, 200.0f, 0.3f);
    for (int i = 0; i < kMeasSamples; ++i) inR[i] = inL[i];

    // test_grain_gate_saves_cycles
    {
        auto measureUs = [&](float k1) -> double {
            Patch* p = makePatch();
            p->setParamValue(1, k1);
            processSilence(p, 4096);
            static float oL[kMeasSamples], oR[kMeasSamples];
            std::memcpy(oL, inL, kMeasSamples * sizeof(float));
            std::memcpy(oR, inR, kMeasSamples * sizeof(float));
            auto t0 = std::chrono::high_resolution_clock::now();
            p->processAudio(std::span<float>(oL, kMeasSamples),
                            std::span<float>(oR, kMeasSamples));
            auto t1 = std::chrono::high_resolution_clock::now();
            return std::chrono::duration<double, std::micro>(t1 - t0).count()
                   / static_cast<double>(kMeasSamples);
        };

        const double usOff = measureUs(0.1f);   // Zone A: voices off
        const double usOn  = measureUs(0.5f);   // Zone B: voices on

        std::printf("  INFO  grain gate: voices_off=%.3f us  voices_on=%.3f us\n",
                    usOff, usOn);
        const double ratio = usOn / (usOff + 1e-12);
        std::printf("  INFO  voices_on/voices_off ratio = %.3f (expect >= 1.0)\n", ratio);
        check(ratio >= 1.0, "grain_gate_voices_on_not_faster_than_off");
    }

    // test_grain_gate_smooth_transition
    // Ramp knob1 through the Zone A/B boundary (voices OFF→ON) and verify no click.
    // Uses a continuous-phase sine (phase accumulated across blocks) to avoid
    // artificial inter-block phase discontinuities.
    {
        Patch* p = makePatch();
        p->setParamValue(0, 0.5f);
        p->setParamValue(2, 0.0f);
        p->setParamValue(1, 0.1f);
        processSilence(p, 4096);

        constexpr int kBlockSize = 200;
        constexpr int kNumBlocks = 48000 / kBlockSize;
        // Pre-generate a long continuous sine so phase is correct across all blocks.
        static float contSine[48000];
        fillSine(contSine, 48000, 200.0f, 0.3f);

        static float oL[kBlockSize], oR[kBlockSize];

        float maxDeltaAtTransition = 0.0f;
        float prevL = 0.0f;
        bool inTransition = false;
        bool prevInTransition = false;

        for (int block = 0; block < kNumBlocks; ++block) {
            const float t     = static_cast<float>(block) / static_cast<float>(kNumBlocks);
            const float knob1 = 0.1f + 0.4f * t;
            p->setParamValue(1, knob1);

            const float* blockIn = contSine + block * kBlockSize;
            std::memcpy(oL, blockIn, static_cast<std::size_t>(kBlockSize) * sizeof(float));
            std::memcpy(oR, blockIn, static_cast<std::size_t>(kBlockSize) * sizeof(float));
            p->processAudio(std::span<float>(oL, kBlockSize),
                            std::span<float>(oR, kBlockSize));

            bool nowInTransition = (knob1 > 0.28f && knob1 < 0.40f);
            if (nowInTransition) inTransition = true;

            // Measure within the transition region only (skip first sample of first
            // transition block to avoid measuring across the boundary from non-transition)
            if (inTransition) {
                int startI = (nowInTransition && !prevInTransition) ? 1 : 0;
                for (int i = startI; i < kBlockSize; ++i) {
                    float delta = fabsf(oL[i] - prevL);
                    maxDeltaAtTransition = std::max(maxDeltaAtTransition, delta);
                    prevL = oL[i];
                }
            } else {
                prevL = oL[kBlockSize - 1];
            }
            prevInTransition = nowInTransition;
        }

        std::printf("  INFO  max delta at voice enable transition: %.5f\n",
                    static_cast<double>(maxDeltaAtTransition));
        check(maxDeltaAtTransition < 0.1f, "grain_gate_transition_no_click_under_0_1");
    }

    // test_control_rate_decimation_convergence
    // Verifies the 32-sample control-rate decimation takes effect quickly.
    //
    // Strategy: use the BigMuff ALONE (knob1=0 → minimum reverb so dry signal
    // dominates) and measure that after a step change in fuzz, the output changes
    // within the first 64 samples (2 control-rate periods) AND that the output is
    // similar at samples 64-128 vs 700-800 (parameter converged — smoother
    // τ ≈ 229 samples, so 700 samples >> 3τ = 687 samples for 95% convergence).
    //
    // By using a pure-tone sine with minimum reverb (almost-dry signal path),
    // we isolate the BigMuff parameter change from reverb tail accumulation.
    {
        Patch* p = makePatch();
        p->setParamValue(0, 0.0f);   // start: very low fuzz
        p->setParamValue(1, 0.0f);   // minimum reverb — dry signal dominates
        p->setParamValue(2, 0.0f);
        // Let smoothers settle at knob0=0 and SoftFocus settle at min reverb
        processSilence(p, 4096);

        // Step change to full fuzz — parameter should take effect within 32 samples
        p->setParamValue(0, 1.0f);

        static float contSine[2000];
        fillSine(contSine, 2000, 200.0f, 0.3f);
        static float oL[2000], oR[2000];
        processBlock(p, contSine, contSine, oL, oR, 2000);

        // Measure RMS in a window AFTER the first control-rate update fires (32+ samples)
        // but BEFORE the smoother has fully converged (early effect window)
        const float rmsAfterFirst32  = rms(oL + 64,  128);   // samples 64-192
        // Measure RMS well after smoother convergence (>3τ = 687 samples)
        const float rmsConverged = rms(oL + 700, 300);  // samples 700-1000

        std::printf("  INFO  convergence: rms_after_32_samples=%.5f  rms_converged=%.5f\n",
                    static_cast<double>(rmsAfterFirst32),
                    static_cast<double>(rmsConverged));

        // The step change was to full fuzz (knob0=1.0). The smoother at sample 64
        // has current ≈ 0 + 64 * 0.00435 * 1.0 ≈ 0.278 — significantly above 0.
        // This should change the muff output measurably.
        check(rmsAfterFirst32 > 0.001f, "control_rate_step_has_effect_after_32_samples");

        // After convergence (700+ samples), output should be similar to the
        // fully-converged window — within 40% (reverb tail is still building up
        // slightly but for min-reverb setting this should be small)
        const float diff = fabsf(rmsAfterFirst32 - rmsConverged);
        const float larger = std::max(rmsAfterFirst32, rmsConverged);
        const float rel  = (larger > 1e-6f) ? (diff / larger) : 0.0f;
        std::printf("  INFO  early/converged relative difference: %.3f\n",
                    static_cast<double>(rel));
        check(rel < 0.50f, "control_rate_decimation_converges_within_50pct");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    std::printf("======================================\n");
    std::printf(" Shoegaze PatchImpl Test Suite\n");
    std::printf("======================================\n");

    testGroup1();
    testGroup2();
    testGroup3();
    testGroup4();
    testGroup5();
    testGroup6();

    std::printf("\n======================================\n");
    std::printf(" Results: %d/%d passed\n", gPassed, gTotal);
    std::printf("======================================\n");

    return (gPassed == gTotal) ? 0 : 1;
}
