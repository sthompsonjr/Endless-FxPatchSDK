/// big_muff_test.cpp — Test harness for BigMuffCircuit
///
/// Compile:
///   g++ -std=c++20 -I. -I./dsp -I./wdf -fno-exceptions -fno-rtti
///       -Wdouble-promotion -Werror -O2 -o build/test_big_muff
///       tests/big_muff_test.cpp -lm
///
/// Run:  ./build/test_big_muff

#include "wdf/WdfBigMuffCircuit.h"

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cassert>
#include <cstdint>
#include <chrono>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// Test infrastructure
// ─────────────────────────────────────────────────────────────────────────────
static int gTotal  = 0;
static int gPassed = 0;
static int gFailed = 0;

static void check(bool condition, const char* name, const char* detail = nullptr) {
    ++gTotal;
    if (condition) {
        ++gPassed;
        std::printf("  PASS  %s\n", name);
    } else {
        ++gFailed;
        if (detail)
            std::printf("  FAIL  %s  [%s]\n", name, detail);
        else
            std::printf("  FAIL  %s\n", name);
    }
}

static char gDetailBuf[256];

// Cast to double explicitly to satisfy -Wdouble-promotion in snprintf/printf.
#define FD(x) static_cast<double>(x)

#define ASSERT_NEAR(val, expected, tol, name) \
    do { \
        float _v = (val); float _e = (expected); float _t = (tol); \
        bool _ok = (fabsf(_v - _e) <= _t); \
        std::snprintf(gDetailBuf, sizeof(gDetailBuf), \
            "got=%.6f expected=%.6f tol=%.6f", FD(_v), FD(_e), FD(_t)); \
        check(_ok, name, gDetailBuf); \
    } while(0)

#define ASSERT_GT(val, lo, name) \
    do { \
        float _v = (val); float _l = (lo); \
        bool _ok = (_v > _l); \
        std::snprintf(gDetailBuf, sizeof(gDetailBuf), "got=%.6f must_be>%.6f", FD(_v), FD(_l)); \
        check(_ok, name, gDetailBuf); \
    } while(0)

#define ASSERT_LT(val, hi, name) \
    do { \
        float _v = (val); float _h = (hi); \
        bool _ok = (_v < _h); \
        std::snprintf(gDetailBuf, sizeof(gDetailBuf), "got=%.6f must_be<%.6f", FD(_v), FD(_h)); \
        check(_ok, name, gDetailBuf); \
    } while(0)

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float kSR = 48000.0f;

/// Discard-safe process call for warm-up / settling loops.
static void processWarmup(BigMuffCircuit& c, float in) noexcept {
    [[maybe_unused]] float out = c.process(in);
}

/// Settle a circuit to DC operating point with silence input.
static void settle(BigMuffCircuit& c, int nSamples = 5000) {
    for (int i = 0; i < nSamples; ++i)
        processWarmup(c, 0.0f);
}

/// LCG pseudo-random in [-1, 1].
static float lcgRand(uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return static_cast<float>(static_cast<int32_t>(state)) / 2147483648.0f;
}

/// Single-bin DFT magnitude at given frequency over a buffer.
static float dftBinMag(const float* buf, int N, float freqHz, float sr) {
    double re = 0.0, im = 0.0;
    const double omega = 6.283185307 * static_cast<double>(freqHz) / static_cast<double>(sr);
    for (int i = 0; i < N; ++i) {
        re += static_cast<double>(buf[i]) * std::cos(omega * static_cast<double>(i));
        im += static_cast<double>(buf[i]) * std::sin(omega * static_cast<double>(i));
    }
    return static_cast<float>(std::sqrt(re * re + im * im) * 2.0 / static_cast<double>(N));
}

/// Compute RMS of a buffer.
static float rms(const float* buf, int N) {
    double sum = 0.0;
    for (int i = 0; i < N; ++i) sum += static_cast<double>(buf[i]) * static_cast<double>(buf[i]);
    return static_cast<float>(std::sqrt(sum / static_cast<double>(N)));
}

/// Collect N samples of a sine at given amplitude/freq into buf[].
static void collectSine(BigMuffCircuit& c, float* buf, int N,
                        float ampIn, float freqHz, float sr) {
    const double omega = 6.283185307 * static_cast<double>(freqHz) / static_cast<double>(sr);
    for (int i = 0; i < 2000; ++i)   // warm-up / DC settling
        processWarmup(c, ampIn * static_cast<float>(std::sin(omega * static_cast<double>(i))));
    for (int i = 0; i < N; ++i)
        buf[i] = c.process(ampIn * static_cast<float>(
                               std::sin(omega * static_cast<double>(i + 2000))));
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 1: DC Operating Points
// ─────────────────────────────────────────────────────────────────────────────
static void test_dc_operating_points() {
    std::printf("\n--- Group 1: DC Operating Points ---\n");

    BigMuffCircuit c;
    c.init(kSR, bigmuff::Variant::RamsHead);
    c.setSustain(0.5f);
    settle(c, 10000);

    const float Vc1 = c.getQ1CollectorVoltage();
    const float Vc2 = c.getQ2CollectorVoltage();
    const float Vc3 = c.getQ3CollectorVoltage();

    std::printf("  INFO  Q1 Vc=%.4f V  Q2 Vc=%.4f V  Q3 Vc=%.4f V\n",
                FD(Vc1), FD(Vc2), FD(Vc3));

    // Q1: no diodes. Physical voltage = Vcc - R_c*Ic, expected mid-supply range.
    check(std::isfinite(Vc1) && Vc1 > 0.5f && Vc1 < 8.5f,
          "q1_dc_collector_in_valid_range",
          "Q1 collector must be 0.5–8.5V (mid-supply, forward-active)");

    // Q2, Q3: clipping stages — diodes are modelled in the inter-stage path,
    // so DC collector voltage is set by bias divider (mid-supply, ~3–5 V).
    std::snprintf(gDetailBuf, sizeof(gDetailBuf), "Vc2=%.4f V (must be 0.5..8.5 V)", FD(Vc2));
    check(Vc2 > 0.5f && Vc2 < 8.5f, "q2_dc_collector_in_valid_range", gDetailBuf);

    std::snprintf(gDetailBuf, sizeof(gDetailBuf), "Vc3=%.4f V (must be 0.5..8.5 V)", FD(Vc3));
    check(Vc3 > 0.5f && Vc3 < 8.5f, "q3_dc_collector_in_valid_range", gDetailBuf);
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 2: Signal Path Correctness
// ─────────────────────────────────────────────────────────────────────────────
static void test_tone_stack() {
    std::printf("\n--- Group 2: Tone Stack Mid-Scoop ---\n");

    constexpr int N = 4096;
    static float bufMid[N], bufLP[N], bufHP[N];

    // 300 Hz test: mid-scoop is centered around 300 Hz (above bass fc, below treble fc)
    {
        BigMuffCircuit c;
        c.init(kSR, bigmuff::Variant::RamsHead);
        c.setSustain(0.0f); c.setVolume(1.0f); c.setTone(0.5f);
        collectSine(c, bufMid, N, 0.1f, 300.0f, kSR);
    }
    {
        BigMuffCircuit c;
        c.init(kSR, bigmuff::Variant::RamsHead);
        c.setSustain(0.0f); c.setVolume(1.0f); c.setTone(0.0f);
        collectSine(c, bufLP, N, 0.1f, 300.0f, kSR);
    }
    {
        BigMuffCircuit c;
        c.init(kSR, bigmuff::Variant::RamsHead);
        c.setSustain(0.0f); c.setVolume(1.0f); c.setTone(1.0f);
        collectSine(c, bufHP, N, 0.1f, 300.0f, kSR);
    }

    const float mag300_mid = dftBinMag(bufMid, N, 300.0f, kSR);
    const float mag300_lp  = dftBinMag(bufLP,  N, 300.0f, kSR);
    const float mag300_hp  = dftBinMag(bufHP,  N, 300.0f, kSR);

    std::printf("  INFO  300Hz: tone=0.5->%.5f  tone=0.0->%.5f  tone=1.0->%.5f\n",
                FD(mag300_mid), FD(mag300_lp), FD(mag300_hp));

    // At tone=0.5 (mid-scoop), 300 Hz should be more attenuated than at tone=0.0 (LP mode)
    if (mag300_lp > 1e-6f) {
        const float ratio = mag300_mid / mag300_lp;
        std::snprintf(gDetailBuf, sizeof(gDetailBuf),
            "300Hz mid/LP=%.3f (must be <0.95 for mid-scoop effect)", FD(ratio));
        check(ratio < 0.95f, "mid_scoop_300hz_attenuated_vs_lp", gDetailBuf);
    } else {
        check(false, "mid_scoop_lp_nonzero", "LP output at 300Hz near zero");
    }

    // At tone=1.0 (HP), 300 Hz output might be higher because HP path has
    // gain that doesn't fully attenuate 300 Hz. This test confirms HP differs from LP.
    if (mag300_lp > 1e-6f && mag300_hp > 1e-6f) {
        const float diffRatio = fabsf(mag300_hp - mag300_lp) / mag300_lp;
        std::snprintf(gDetailBuf, sizeof(gDetailBuf),
            "300Hz HP/LP diff=%.3f", FD(diffRatio));
        check(diffRatio > 0.02f, "hp_and_lp_differ_at_300hz", gDetailBuf);
    }

    // LP test: 50 Hz should pass LP more than HP
    {
        static float b50lp[N], b50hp[N];
        BigMuffCircuit cLP, cHP;
        cLP.init(kSR); cLP.setSustain(0.0f); cLP.setTone(0.0f);
        cHP.init(kSR); cHP.setSustain(0.0f); cHP.setTone(1.0f);
        collectSine(cLP, b50lp, N, 0.1f, 50.0f, kSR);
        collectSine(cHP, b50hp, N, 0.1f, 50.0f, kSR);
        const float m50lp = dftBinMag(b50lp, N, 50.0f, kSR);
        const float m50hp = dftBinMag(b50hp, N, 50.0f, kSR);
        std::printf("  INFO  50Hz: tone=0.0->%.5f  tone=1.0->%.5f\n", FD(m50lp), FD(m50hp));
        if (m50hp > 1e-8f) {
            std::snprintf(gDetailBuf, sizeof(gDetailBuf), "LP/HP=%.2fdB at 50Hz",
                FD(20.0f * log10f(m50lp / (m50hp + 1e-12f))));
            check(m50lp >= m50hp * 1.2f, "lp_shelf_passes_bass_50hz", gDetailBuf);
        }
    }

    // HP test: 4 kHz should pass HP more than LP
    {
        static float b4klp[N], b4khp[N];
        BigMuffCircuit cLP, cHP;
        cLP.init(kSR); cLP.setSustain(0.0f); cLP.setTone(0.0f);
        cHP.init(kSR); cHP.setSustain(0.0f); cHP.setTone(1.0f);
        collectSine(cLP, b4klp, N, 0.1f, 4000.0f, kSR);
        collectSine(cHP, b4khp, N, 0.1f, 4000.0f, kSR);
        const float m4lp = dftBinMag(b4klp, N, 4000.0f, kSR);
        const float m4hp = dftBinMag(b4khp, N, 4000.0f, kSR);
        std::printf("  INFO  4kHz: tone=0.0->%.5f  tone=1.0->%.5f\n", FD(m4lp), FD(m4hp));
        if (m4lp > 1e-8f) {
            std::snprintf(gDetailBuf, sizeof(gDetailBuf), "HP/LP=%.2fdB at 4kHz",
                FD(20.0f * log10f(m4hp / (m4lp + 1e-12f))));
            check(m4hp >= m4lp * 1.2f, "hp_shelf_passes_treble_4khz", gDetailBuf);
        }
    }
}

static void test_symmetric_clipping() {
    std::printf("\n--- Group 2: Symmetric Clipping ---\n");

    constexpr int N = 8192;
    static float buf[N];

    BigMuffCircuit c;
    c.init(kSR, bigmuff::Variant::RamsHead);
    c.setSustain(0.8f);
    c.setTone(0.5f);
    c.setVolume(1.0f);
    settle(c, 3000);

    const double omega = 6.283185307 * 1000.0 / static_cast<double>(kSR);
    for (int i = 0; i < 1000; ++i)
        processWarmup(c, 0.8f * static_cast<float>(std::sin(omega * static_cast<double>(i))));
    for (int i = 0; i < N; ++i)
        buf[i] = c.process(0.8f * static_cast<float>(
                               std::sin(omega * static_cast<double>(i + 1000))));

    const float fund = dftBinMag(buf, N, 1000.0f, kSR);
    const float h2   = dftBinMag(buf, N, 2000.0f, kSR);
    const float h3   = dftBinMag(buf, N, 3000.0f, kSR);

    std::printf("  INFO  fund=%.5f  2nd=%.5f  3rd=%.5f\n", FD(fund), FD(h2), FD(h3));
    if (fund > 1e-5f) {
        std::printf("  INFO  2nd/fund=%.4f  3rd/fund=%.4f\n",
                    FD(h2/fund), FD(h3/fund));
    }

    // Anti-parallel diodes → odd-harmonic clipping: 3rd harmonic must be present
    if (fund > 1e-5f) {
        std::snprintf(gDetailBuf, sizeof(gDetailBuf),
            "3rd/fund=%.4f must be >0.01", FD(h3/fund));
        check(h3 / fund > 0.01f, "symmetric_clipping_3rd_harmonic_present", gDetailBuf);

        // 2nd harmonic should not dominate over 3rd (symmetric clipping = odd harmonics)
        std::snprintf(gDetailBuf, sizeof(gDetailBuf),
            "2nd=%.5f  3rd=%.5f", FD(h2), FD(h3));
        check(h2 < h3 * 3.0f || h2 < fund * 0.1f,
              "symmetric_clipping_not_dominated_by_2nd", gDetailBuf);
    } else {
        check(false, "symmetric_clipping_has_output", "fundamental near zero");
    }
}

static void test_higher_sustain_more_thd() {
    std::printf("\n--- Group 2: Higher Sustain → More THD ---\n");

    constexpr int N = 8192;
    static float bufLow[N], bufHigh[N];
    const double omega = 6.283185307 * 1000.0 / static_cast<double>(kSR);

    auto doRun = [&](float sust, float* buf) {
        BigMuffCircuit c;
        c.init(kSR);
        c.setSustain(sust);
        c.setTone(0.5f);
        c.setVolume(1.0f);
        settle(c, 3000);
        for (int i = 0; i < 1000; ++i)
            processWarmup(c, 0.3f * static_cast<float>(
                              std::sin(omega * static_cast<double>(i))));
        for (int i = 0; i < N; ++i)
            buf[i] = c.process(0.3f * static_cast<float>(
                                   std::sin(omega * static_cast<double>(i + 1000))));
    };

    doRun(0.0f, bufLow);
    doRun(0.7f, bufHigh);

    auto thdRatio = [&](const float* buf) -> float {
        float fund = dftBinMag(buf, N, 1000.0f, kSR);
        if (fund < 1e-7f) return 0.0f;
        float h2 = dftBinMag(buf, N, 2000.0f, kSR);
        float h3 = dftBinMag(buf, N, 3000.0f, kSR);
        float h4 = dftBinMag(buf, N, 4000.0f, kSR);
        float h5 = dftBinMag(buf, N, 5000.0f, kSR);
        return (h2 + h3 + h4 + h5) / fund;
    };

    const float thdLow  = thdRatio(bufLow);
    const float thdHigh = thdRatio(bufHigh);

    std::printf("  INFO  THD ratio: sustain=0.0->%.4f  sustain=0.7->%.4f\n",
                FD(thdLow), FD(thdHigh));

    std::snprintf(gDetailBuf, sizeof(gDetailBuf),
        "high/low THD ratio=%.2f (>2.0 expected)", FD(thdHigh / (thdLow + 1e-6f)));
    check(thdHigh > thdLow * 2.0f, "higher_sustain_more_thd", gDetailBuf);
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 3: Numerical Stability
// ─────────────────────────────────────────────────────────────────────────────
static void test_no_nan_white_noise() {
    std::printf("\n--- Group 3: No NaN / White Noise ---\n");

    BigMuffCircuit c;
    c.init(kSR, bigmuff::Variant::RamsHead);
    c.setSustain(1.0f);
    c.setTone(0.5f);
    c.setVolume(1.0f);

    uint32_t rng = 0xDEADBEEFu;
    bool allFinite = true;

    for (int i = 0; i < 96000; ++i) {
        const float out = c.process(lcgRand(rng));
        if (!std::isfinite(out)) allFinite = false;
    }

    check(allFinite, "no_nan_or_inf_on_white_noise");
}

static void test_silence_output() {
    std::printf("\n--- Group 3: Silence Output ---\n");

    BigMuffCircuit c;
    c.init(kSR, bigmuff::Variant::RamsHead);
    c.setSustain(0.5f);
    c.setVolume(1.0f);

    settle(c, 5000);  // DC settling

    double sumSq = 0.0;
    for (int i = 0; i < 96000; ++i) {
        const float out = c.process(0.0f);
        sumSq += static_cast<double>(out) * static_cast<double>(out);
    }
    const float outRms = static_cast<float>(std::sqrt(sumSq / 96000.0));
    std::printf("  INFO  Silence output RMS = %.2e\n", FD(outRms));

    std::snprintf(gDetailBuf, sizeof(gDetailBuf), "rms=%.2e (must be <1e-4)", FD(outRms));
    check(outRms < 1e-4f, "silence_output_below_threshold", gDetailBuf);
}

static void test_step_response_stability() {
    std::printf("\n--- Group 3: Step Response ---\n");

    BigMuffCircuit c;
    c.init(kSR, bigmuff::Variant::RamsHead);
    c.setSustain(0.5f);
    c.setVolume(1.0f);
    settle(c, 1000);

    float peakDuringStep = 0.0f;
    bool allFinite = true;
    for (int i = 0; i < 1000; ++i) {
        const float out = c.process(0.5f);
        if (!std::isfinite(out)) allFinite = false;
        peakDuringStep = std::max(peakDuringStep, fabsf(out));
    }
    check(allFinite, "step_response_all_finite");

    // dcBlock_ is 10 Hz → τ≈768 samples; need 5000 samples to settle to <0.05
    float maxAfter = 0.0f;
    for (int i = 0; i < 5000; ++i) {
        const float out = c.process(0.0f);
        if (i > 4500) maxAfter = std::max(maxAfter, fabsf(out));
    }

    std::printf("  INFO  Peak during step=%.4f  maxAfter 5000 samples=%.6f\n",
                FD(peakDuringStep), FD(maxAfter));

    std::snprintf(gDetailBuf, sizeof(gDetailBuf), "peak=%.4f", FD(peakDuringStep));
    check(peakDuringStep < 2.0f, "step_output_bounded_to_2", gDetailBuf);

    std::snprintf(gDetailBuf, sizeof(gDetailBuf), "maxAfter=%.6f", FD(maxAfter));
    check(maxAfter < 0.05f, "step_response_settles", gDetailBuf);
}

static void test_extreme_inputs_no_nan() {
    std::printf("\n--- Group 3: Extreme Inputs ---\n");

    BigMuffCircuit c;
    c.init(kSR, bigmuff::Variant::RamsHead);
    c.setSustain(0.5f);

    bool allFinite = true;
    for (int i = 0; i < 50; ++i) {
        if (!std::isfinite(c.process(1.0f)))  allFinite = false;
        if (!std::isfinite(c.process(-1.0f))) allFinite = false;
    }
    check(allFinite, "extreme_inputs_no_nan");
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 4: Variant Consistency
// ─────────────────────────────────────────────────────────────────────────────
static void test_variant_switching_no_nan() {
    std::printf("\n--- Group 4: Variant Switching ---\n");

    BigMuffCircuit c;
    c.init(kSR, bigmuff::Variant::RamsHead);
    c.setSustain(0.5f);
    c.setVolume(1.0f);

    const double omega = 6.283185307 * 1000.0 / static_cast<double>(kSR);
    bool allOk = true;

    for (int i = 0; i < 3000; ++i) {
        const float out = c.process(0.5f * static_cast<float>(
                                        std::sin(omega * static_cast<double>(i))));
        if (!std::isfinite(out) || fabsf(out) > 2.0f) allOk = false;
    }
    c.setVariant(bigmuff::Variant::CivilWar);
    for (int i = 3000; i < 6000; ++i) {
        const float out = c.process(0.5f * static_cast<float>(
                                        std::sin(omega * static_cast<double>(i))));
        if (!std::isfinite(out) || fabsf(out) > 2.0f) allOk = false;
    }
    c.setVariant(bigmuff::Variant::Triangle);
    for (int i = 6000; i < 9000; ++i) {
        const float out = c.process(0.5f * static_cast<float>(
                                        std::sin(omega * static_cast<double>(i))));
        if (!std::isfinite(out) || fabsf(out) > 2.0f) allOk = false;
    }
    check(allOk, "variant_switching_no_nan_no_clipping");
}

static void test_civil_war_vs_rams_head() {
    std::printf("\n--- Group 4: CivilWar vs RamsHead ---\n");

    constexpr int N = 4096;
    static float bufRH[N], bufCW[N];
    const double omega = 6.283185307 * 1000.0 / static_cast<double>(kSR);

    {
        BigMuffCircuit c;
        c.init(kSR, bigmuff::Variant::RamsHead);
        c.setSustain(0.7f); c.setTone(0.5f);
        settle(c, 3000);
        for (int i = 0; i < N; ++i)
            bufRH[i] = c.process(0.5f * static_cast<float>(
                                     std::sin(omega * static_cast<double>(i))));
    }
    {
        BigMuffCircuit c;
        c.init(kSR, bigmuff::Variant::CivilWar);
        c.setSustain(0.7f); c.setTone(0.5f);
        settle(c, 3000);
        for (int i = 0; i < N; ++i)
            bufCW[i] = c.process(0.5f * static_cast<float>(
                                     std::sin(omega * static_cast<double>(i))));
    }

    const float rmsRH = rms(bufRH, N);
    const float rmsCW = rms(bufCW, N);

    std::printf("  INFO  RamsHead RMS=%.5f  CivilWar RMS=%.5f\n", FD(rmsRH), FD(rmsCW));
    check(rmsRH > 1e-5f, "rams_head_produces_output");
    check(rmsCW > 1e-5f, "civil_war_produces_output");

    const float diff = fabsf(rmsRH - rmsCW) / (rmsRH + rmsCW + 1e-9f);
    std::snprintf(gDetailBuf, sizeof(gDetailBuf),
        "norm_diff=%.4f (>0.001 expected)", FD(diff));
    check(diff > 0.001f, "civil_war_differs_from_rams_head", gDetailBuf);
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 5: Performance
// ─────────────────────────────────────────────────────────────────────────────
static void test_performance() {
    std::printf("\n--- Group 5: Performance ---\n");

    BigMuffCircuit c;
    c.init(kSR, bigmuff::Variant::RamsHead);
    c.setSustain(0.5f);
    c.setVolume(1.0f);
    settle(c, 1000);

    constexpr int kMeas = 96000;
    const double omega = 6.283185307 * 1000.0 / static_cast<double>(kSR);

    auto t0 = std::chrono::high_resolution_clock::now();
    double sink = 0.0;
    for (int i = 0; i < kMeas; ++i)
        sink += static_cast<double>(c.process(0.2f * static_cast<float>(
                                          std::sin(omega * static_cast<double>(i)))));
    auto t1 = std::chrono::high_resolution_clock::now();
    (void)sink;

    const double elapsedUs =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
    const double usPerSample  = elapsedUs / static_cast<double>(kMeas);
    const double estCycles    = usPerSample * 720.0; // ARM @ 720 MHz

    std::printf("  INFO  Wall-clock: %.3f µs/sample\n", usPerSample);
    std::printf("  INFO  Est. ARM cycles/sample @ 720 MHz: %.0f\n", estCycles);
    std::printf("  INFO  Target ≤9000 cycles (flag at >12000)\n");

    if (estCycles > 12000.0)
        std::printf("  FLAG  Estimated cycles > 12000! Review for optimization.\n");

    check(usPerSample < 5000.0, "process_completes_in_reasonable_time");
}

// ─────────────────────────────────────────────────────────────────────────────
// Group 6: NYC Variant (2N5088) Tests
// ─────────────────────────────────────────────────────────────────────────────

static void test_nyc_variant_instantiation() {
    std::printf("\n--- Group 6: NYC Variant Instantiation ---\n");

    BigMuffCircuit circuit;
    circuit.init(48000.0f);
    circuit.setVariant(bigmuff::Variant::NYC);
    circuit.setSustain(0.5f);
    circuit.setTone(0.5f);
    circuit.reset();
    // Must not crash — if we get here, it passes
    check(true, "nyc_variant_instantiation_no_crash");
}

static void test_nyc_variant_dc_blocking() {
    std::printf("\n--- Group 6: NYC Variant DC Blocking ---\n");
    // DC input must produce near-zero DC output (AC-coupled circuit).
    // All inter-stage coupling caps and the output DC block filter remove DC
    // by design. After sufficient settling, output converges toward 0.
    //
    // Settling budget: the output DC block filter has a 10 Hz corner (τ≈764
    // samples). The input HP filter corner is 17 Hz (τ≈449 samples). Combined
    // 5000-sample run (≈6.5 time constants of the DC block) reduces the DC
    // transient to <0.001V before output scaling.

    BigMuffCircuit circuit;
    circuit.init(48000.0f);
    circuit.setVariant(bigmuff::Variant::NYC);
    circuit.setSustain(0.5f);
    circuit.setTone(0.5f);
    circuit.reset();

    // Process 5000 samples of DC input = 0.5f (DC bias)
    float lastOut = 0.0f;
    for (int i = 0; i < 5000; ++i)
        lastOut = circuit.process(0.5f);

    std::snprintf(gDetailBuf, sizeof(gDetailBuf),
        "|lastOut|=%.5f (must be <0.05)", FD(fabsf(lastOut)));
    check(fabsf(lastOut) < 0.05f, "nyc_variant_dc_blocked_by_coupling_caps", gDetailBuf);
}

static void test_nyc_variant_sine_produces_output() {
    std::printf("\n--- Group 6: NYC Variant Sine Output ---\n");
    // A 440 Hz sine at moderate gain should produce nonzero, bounded output.

    BigMuffCircuit circuit;
    circuit.init(48000.0f);
    circuit.setVariant(bigmuff::Variant::NYC);
    circuit.setSustain(0.7f);
    circuit.setTone(0.5f);
    circuit.reset();

    float maxOut = 0.0f;
    const float freq = 440.0f;
    const float sr   = 48000.0f;
    for (int i = 0; i < 2400; ++i) {
        float x = 0.3f * sinf(2.0f * 3.14159265f * freq * static_cast<float>(i) / sr);
        float y = circuit.process(x);
        if (fabsf(y) > maxOut) maxOut = fabsf(y);
    }

    std::snprintf(gDetailBuf, sizeof(gDetailBuf), "peak=%.4f (must be >0.001)", FD(maxOut));
    check(maxOut > 0.001f, "nyc_variant_sine_nonzero_output", gDetailBuf);

    std::snprintf(gDetailBuf, sizeof(gDetailBuf), "peak=%.4f (must be <2.0)", FD(maxOut));
    check(maxOut < 2.0f,   "nyc_variant_sine_bounded_output", gDetailBuf);

    std::printf("  INFO  NYC 440Hz peak=%.4f\n", FD(maxOut));
}

static void test_nyc_variant_sustain_increases_gain() {
    std::printf("\n--- Group 6: NYC Variant Sustain vs. Gain ---\n");
    // Higher sustain → higher output RMS for sub-clipping input.
    // Sustain pot controls loop gain; increasing it raises amplitude before clipping onset.
    //
    // Amplitude 0.002f is chosen to keep the signal sub-clipping at all stages
    // for both sustain levels (0.2 and 0.8). At 0.05f, Q3 saturates for both
    // sustain levels, making the comparison ambiguous.
    //
    // 5000-sample warmup lets the output DC block filter (10 Hz corner, τ≈764
    // samples) fully settle so the RMS accumulation reflects signal amplitude
    // rather than DC transient.

    BigMuffCircuit circuit;
    circuit.init(48000.0f);
    circuit.setVariant(bigmuff::Variant::NYC);
    circuit.setTone(0.5f);

    auto measureRMS = [&](float sustain) -> float {
        circuit.setSustain(sustain);
        circuit.reset();
        const float freq = 440.0f;
        const float sr   = 48000.0f;
        // Warmup: let dcBlock_ settle before measuring
        for (int i = 0; i < 5000; ++i)
            processWarmup(circuit,
                0.002f * sinf(2.0f * 3.14159265f * freq * static_cast<float>(i) / sr));
        // Accumulate RMS (continue sine phase from i=5000)
        float sum = 0.0f;
        for (int i = 0; i < 4800; ++i) {
            float x = 0.002f * sinf(2.0f * 3.14159265f * freq *
                                    static_cast<float>(i + 5000) / sr);
            float y = circuit.process(x);
            sum += y * y;
        }
        return sqrtf(sum / 4800.0f);
    };

    const float rmsLow  = measureRMS(0.2f);
    const float rmsHigh = measureRMS(0.8f);

    std::printf("  INFO  NYC sustain RMS: low(0.2)=%.5f  high(0.8)=%.5f\n",
                FD(rmsLow), FD(rmsHigh));
    std::snprintf(gDetailBuf, sizeof(gDetailBuf),
        "rmsLow=%.5f rmsHigh=%.5f (high must exceed low)", FD(rmsLow), FD(rmsHigh));
    check(rmsHigh > rmsLow, "nyc_variant_higher_sustain_higher_rms", gDetailBuf);
}

static void test_nyc_variant_independent_of_other_variants() {
    std::printf("\n--- Group 6: NYC Variant State Independence ---\n");
    // Switching to NYC and back to RamsHead with reset must reproduce identical output.
    // Verifies variant switch only changes BJT parameters; WDF state is properly reset.

    BigMuffCircuit circuit;
    circuit.init(48000.0f);
    circuit.setSustain(0.5f);
    circuit.setTone(0.5f);

    const float freq = 440.0f;
    const float sr   = 48000.0f;

    // Capture 100 samples from RamsHead
    circuit.setVariant(bigmuff::Variant::RamsHead);
    circuit.reset();
    static float ramsHead[100];
    for (int i = 0; i < 100; ++i)
        ramsHead[i] = circuit.process(0.1f * sinf(2.0f * 3.14159265f * freq * static_cast<float>(i) / sr));

    // Switch to NYC, run 100 samples to dirty up state
    circuit.setVariant(bigmuff::Variant::NYC);
    circuit.reset();
    for (int i = 0; i < 100; ++i)
        processWarmup(circuit, 0.1f * sinf(2.0f * 3.14159265f * freq * static_cast<float>(i) / sr));

    // Switch back to RamsHead, reset, capture again
    circuit.setVariant(bigmuff::Variant::RamsHead);
    circuit.reset();
    static float ramsHead2[100];
    for (int i = 0; i < 100; ++i)
        ramsHead2[i] = circuit.process(0.1f * sinf(2.0f * 3.14159265f * freq * static_cast<float>(i) / sr));

    // Both runs must be identical within float epsilon
    bool allMatch = true;
    int  firstMismatch = -1;
    for (int i = 0; i < 100; ++i) {
        if (fabsf(ramsHead[i] - ramsHead2[i]) >= 1e-6f) {
            allMatch = false;
            if (firstMismatch < 0) firstMismatch = i;
        }
    }

    if (firstMismatch >= 0) {
        std::snprintf(gDetailBuf, sizeof(gDetailBuf),
            "first mismatch at sample %d: %.8f vs %.8f",
            firstMismatch,
            FD(ramsHead[firstMismatch]), FD(ramsHead2[firstMismatch]));
    } else {
        std::snprintf(gDetailBuf, sizeof(gDetailBuf), "all 100 samples match within 1e-6");
    }
    check(allMatch, "nyc_variant_independent_of_rams_head_state", gDetailBuf);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    std::printf("======================================\n");
    std::printf(" BigMuffCircuit Test Suite\n");
    std::printf("======================================\n");

    test_dc_operating_points();
    test_tone_stack();
    test_symmetric_clipping();
    test_higher_sustain_more_thd();
    test_no_nan_white_noise();
    test_silence_output();
    test_step_response_stability();
    test_extreme_inputs_no_nan();
    test_variant_switching_no_nan();
    test_civil_war_vs_rams_head();
    test_performance();

    // NYC variant tests
    test_nyc_variant_instantiation();
    test_nyc_variant_dc_blocking();
    test_nyc_variant_sine_produces_output();
    test_nyc_variant_sustain_increases_gain();
    test_nyc_variant_independent_of_other_variants();

    std::printf("\n======================================\n");
    std::printf(" Results: %d/%d passed", gPassed, gTotal);
    if (gFailed > 0)
        std::printf("  (%d FAILED)", gFailed);
    std::printf("\n======================================\n");

    return gFailed > 0 ? 1 : 0;
}
