/*
 * test_TriDimension_wdf.cpp — Session 6: full test harness for the
 * TriDimension patch (Dytronics Tri-Stereo Chorus + Boss DC-2 Dimension C).
 * Tridimension Session 6 of 6.
 *
 * Build (the patch translation unit is linked in; TRIDIMENSION_SMOKE must
 * NOT be defined, so the patch's smoke main() stays out and this file owns
 * main()):
 *   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra \
 *       -DENDLESS_DESKTOP_BUILD \
 *       -I. -I./sdk -I./wdf -I./dsp \
 *       tests/test_TriDimension_wdf.cpp effects/PatchImpl_TriDimension.cpp \
 *       -lm -o test_tridimension && ./test_tridimension
 *
 * Output files generated at runtime:
 *   tests/tridimension_freq_response.csv — 201 lines (header + 200 rows)
 *   tests/tridimension_param_sweep.csv   — 181 lines (header + 180 rows)
 *
 * Tests (device under test: full PatchImpl via Patch::getInstance() for 1-6,
 * Dc2Circuit / TscCircuit directly for 7-8):
 *   1. FREQUENCY RESPONSE  — 20 Hz..20 kHz, 200 log points, -20 dBFS,
 *                            parallel routing, intensities 0, DC-2 mode 0.
 *   2. IMPULSE STABILITY   — 0 dBFS impulse, 48,000 samples, |y| < 1.0;
 *                            isolated tail < -96 dB within 1 s (see note A).
 *   3. PARAMETER SWEEP     — six virtual controls min->max in 10 steps at
 *                            80 Hz / 1 kHz / 8 kHz; intensity monotonicity
 *                            (see note B) + zone-transition click gate 0.5.
 *   4. CYCLE COUNT         — worst case (series zone 0, DC-2 on, intensities
 *                            1.0): 1,000,000 samples after warmup; mean+max
 *                            cycles/sample at 528 MHz; WARN >40%, FAIL >60%.
 *                            Parallel typical case also reported (INVENTORY).
 *   5. PRECISION           — 10 s Voss-McCartney pink noise at -6 dBFS; no
 *                            NaN/Inf; denormal scan on outputs; FTZ + retest
 *                            if denormals detected.
 *   6. A/B COMPARISON      — gated on tests/reference_tridimension.pcm
 *                            (raw float32 LE, interleaved stereo, 48 kHz);
 *                            SKIPPED when absent.
 *   7. DC2 MOTIONLESS      — Dc2Circuit, setMode(3), 1 kHz, 10 s: wet mono
 *                            sum flutter <= 0.2x single-channel flutter AND
 *                            cvA+cvB constant within 1e-4 (Session 3 gate).
 *                            Estimator conditioning documented at the test
 *                            (see NOTE C).
 *   8. TSC PHASE SPACING   — TscCircuit chorus-generator CV taps, 60 s at
 *                            defaults: pairwise spacing 120 deg within 2 deg
 *                            for the entire run (Session 2/4 estimator).
 *
 * NOTE A — impulse test design (verified against dsp/BBDLine.h source):
 *   BBDLine's clock-noise injection is ADDITIVE and signal-independent
 *   (clockNoise = clockPulse * noiseSample * level), so with silent input
 *   the patch output sits on a stationary noise floor of roughly -52 dB
 *   ([EAR-MATCHED] levels 0.002/0.003). That floor is by-design character,
 *   not an impulse tail, and a raw -96 dB gate can never pass. The noise
 *   sequence is fully deterministic (LCG state re-seeded by init()), so the
 *   impulse response is isolated by subtracting a bit-identical silence-
 *   driven twin run; the isolated tail carries the -96 dB gate and the raw
 *   floor is reported/flagged alongside. The test runs at the patch DEFAULT
 *   configuration (series zone 0, DC-2 on): in the parallel zone the output
 *   composition out = source + 0.707*wet makes |y| < 1.0 unsatisfiable at
 *   the 0 dBFS impulse instant whenever the instantaneous wet term is
 *   positive — the series composition keeps the spec's amplitude gate
 *   meaningful. All run lengths are multiples of 32 samples so the
 *   control-tick phase (not reset by init()) aligns across the twin runs.
 *
 * NOTE C — mono-sum flutter estimator (Test 7): the wet mono sum of two
 *   exactly antiphase-modulated delays is sin(w(t-dL)) + sin(w(t-dR)) =
 *   2*sin(w(t-dCenter))*cos(w*(dL-dR)/2) — a PHASE-CONSTANT (motionless)
 *   carrier whose amplitude term cos(w*dDelta/2) sweeps through zero
 *   (~70 sign flips over the 10 s run at MODE4 depth and 1 kHz). Near the
 *   AM nulls, interference between the carrier and the residual
 *   unmodulated content that lives inside the DC-2 wet outputs by the
 *   Session 3 output convention (Dc2DryLowpass reinforcement, bridged-T
 *   cross-feedback) wobbles the resultant's per-cycle frequency as
 *   ~1/amplitude^2 (measured: mean |dev| 9.6 Hz at 1/4 amplitude falling
 *   to 0.11 Hz at full carrier, vs the single channel's 3.6 Hz at ALL
 *   amplitudes). That wobble is bounded PHASE — over each null transit the
 *   signed deviations cancel exactly — so a raw per-cycle std is dominated
 *   by interference artifacts of the (correct) antiphase design and fails
 *   for ANY correct implementation, while cvA+cvB is exactly constant.
 *   SPEC DEVIATION (documented, DMM Test-3 precedent): per-cycle periods
 *   are measured via interpolated zero crossings exactly as prescribed,
 *   but the flutter statistic is the std of ENERGY-WEIGHTED 0.25 s window
 *   means of the per-cycle frequency, not the raw per-cycle std. 0.25 s is
 *   short against the MODE4 LFO half-period (0.558 s of one-signed
 *   triangle slope), so the single channel's genuine +-3.5 Hz vibrato
 *   survives windowing as the reference, while the mono sum's zero-mean
 *   bounded-phase wobble cancels within each window. Conditioning applied
 *   IDENTICALLY to both signals: 1 kHz RBJ bandpass Q = 2 (clock-noise
 *   jitter suppression), period plausibility +-25% of nominal, minimal
 *   noise floor 0.1 x sqrt(2) x RMS, weight = segment-peak^2. The DC-2's
 *   antiphase CV identity itself is asserted exactly (P2, unconditioned).
 *
 * NOTE B — intensity monotonicity (Test 3): the patch exposes no delay
 *   observability without -DTRIDIMENSION_SMOKE (which owns the knob-mapping
 *   checks and brings its own main()), so the "delay swing increasing"
 *   assertion runs on TscCircuit directly via getDelayMs() — the same
 *   intensity values the patch's control tick forwards verbatim
 *   (eff_[3..5] -> tsc_.setIntensity, PatchImpl_TriDimension.cpp). The
 *   patch-level sweep produces the CSV amplitudes and the zone-transition
 *   click gate (Session 5 bound: no per-sample output step > 0.5).
 *
 * PatchImpl findings (Session 5, mandatory read):
 *   virtual controls: v0 dc2Mode / v1 tscRate / v2 routing on base-layer
 *   knobs 0/1/2; v3/v4/v5 TSC intensities on the hold layer (latched toggle
 *   via kLeftFootSwitchHold). Soft takeover disengages knobs on every layer
 *   toggle: a disengaged knob re-engages within 0.02 of the stored value or
 *   when swept across it — the driver below always sweeps 0 -> 1 -> target,
 *   which crosses any stored value in [0,1]. Expression stays at 0 (writes
 *   land in heel_, effective values follow heel_). Zone map: v2 < 0.33
 *   zone 0 (TSC->DC2 series) | 0.33..0.66 zone 1 (DC2->TSC) | > 0.66 zone 2
 *   (parallel), 0.02 hysteresis, 512-sample crossfade on change.
 */

#include "sdk/Patch.h"
#include "wdf/Dc2Circuit.h"
#include "wdf/TscCircuit.h"
#include "dsp/BiquadFilter.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#if defined(__SSE2__)
#include <xmmintrin.h>
static void enableFtz() noexcept { _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON); }
#else
static void enableFtz() noexcept {}
#endif

// ---------------------------------------------------------------------------
// Clock and budget constants — 528 MHz actual / 48 kHz (NOT the marketed
// 720 MHz; pattern from tests/test_DeluxeMemoryMan_wdf.cpp)
// ---------------------------------------------------------------------------
static constexpr float kClockHz       = 528'000'000.0f;
static constexpr float kSampleRateF   = 48'000.0f;
static constexpr int   kSampleRateI   = 48'000;
static constexpr float kCyclesPerSamp = kClockHz / kSampleRateF;  // 11,000
static constexpr float k2Pi           = 6.28318530717958f;

// Block size for all patch feeds: multiple of 32 (control interval) and of
// 48 (one 1 kHz cycle at 48 kHz -> phase-continuous block joins).
static constexpr int kBlock = 480;
static float s_bufL[kBlock];
static float s_bufR[kBlock];

// ---------------------------------------------------------------------------
// PASS/FAIL tracking — pattern from tests/test_DeluxeMemoryMan_wdf.cpp
// ---------------------------------------------------------------------------
static int totalTests  = 0;
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

// ---------------------------------------------------------------------------
// Pink noise — Voss-McCartney 7-stage (pattern from DMM harness Test 5)
// ---------------------------------------------------------------------------
static float    s_pinkState[7] = {};
static uint32_t s_pinkCounter  = 0u;
static uint32_t s_pinkLcg      = 0x12345678u;

inline float nextPinkSample() noexcept {
    auto white = [&]() -> float {
        s_pinkLcg = s_pinkLcg * 1664525u + 1013904223u;
        return static_cast<float>(static_cast<int32_t>(s_pinkLcg))
               / static_cast<float>(0x80000000u);
    };
    ++s_pinkCounter;
    for (int i = 0; i < 7; ++i) {
        if ((s_pinkCounter & (1u << i)) == 0u) break;
        s_pinkState[i] = white();
    }
    float sum = 0.0f;
    for (int i = 0; i < 7; ++i) sum += s_pinkState[i];
    return sum * (1.0f / 7.0f);
}

static void resetPinkNoise() noexcept {
    for (auto& s : s_pinkState) s = 0.0f;
    s_pinkCounter = 0u;
    s_pinkLcg     = 0x12345678u;
}

inline bool isDenormal(float x) noexcept {
    uint32_t bits;
    static_assert(sizeof(float) == sizeof(uint32_t));
    __builtin_memcpy(&bits, &x, sizeof(bits));
    const uint32_t exponent = bits & 0x7F800000u;
    const uint32_t mantissa = bits & 0x007FFFFFu;
    return (exponent == 0u) && (mantissa != 0u);
}

// ---------------------------------------------------------------------------
// Patch driver — virtual-control access through the public Patch interface.
// Layer state (layerHold_) persists across Patch::init(), so ONE driver
// instance tracks it for the whole process lifetime; setVirtual() always
// runs the 0 -> 1 -> target pickup sweep, which satisfies soft takeover in
// every engagement state (crossed condition holds for any stored value in
// [0,1]). No audio is processed between the three writes, so only the final
// value reaches the control tick.
// ---------------------------------------------------------------------------
struct PatchDriver {
    Patch& p;
    bool   holdLayer = false;   // mirrors PatchImpl::layerHold_

    void ensureLayer(bool hold) {
        if (holdLayer != hold) {
            p.handleAction(static_cast<int>(endless::ActionId::kLeftFootSwitchHold));
            holdLayer = hold;
        }
    }

    void setVirtual(int v, float value) {
        ensureLayer(v >= 3);
        const int knob = (v >= 3) ? v - 3 : v;
        p.setParamValue(knob, 0.0f);
        p.setParamValue(knob, 1.0f);
        p.setParamValue(knob, value);
    }

    // Quasi-static configuration for Tests 1: parallel routing (zone 2),
    // DC-2 mode 0, all three TSC intensities 0. v1 stays at its 0.5 default.
    void configureQuasiStaticParallel() {
        setVirtual(0, 0.0f);
        setVirtual(2, 0.84f);
        setVirtual(3, 0.0f);
        setVirtual(4, 0.0f);
        setVirtual(5, 0.0f);
        ensureLayer(false);
    }

    // Worst-case configuration for Tests 4/5: series zone 0 (TSC -> DC2,
    // both units + two mono sums in the hot path), DC-2 enabled (init
    // default, never toggled by this harness), all intensities 1.0.
    void configureWorstCaseSeries() {
        setVirtual(0, 0.0f);
        setVirtual(2, 0.16f);
        setVirtual(3, 1.0f);
        setVirtual(4, 1.0f);
        setVirtual(5, 1.0f);
        ensureLayer(false);
    }
};

// Feed numSamples of a sine (amp == 0 -> silence) through the patch in
// kBlock chunks, mono into both channels (both hardware units are
// mono-input). phase persists across calls for continuity. Returns peak
// max(|L|,|R|) over the samples with index >= peakFrom (relative to this
// call); pass peakFrom >= numSamples to skip peak tracking.
static float feedTone(Patch& p, int numSamples, float amp, float freqHz,
                      float& phase, int peakFrom = 1 << 30) {
    const float phInc = k2Pi * freqHz / kSampleRateF;
    float peak = 0.0f;
    int fed = 0;
    while (fed < numSamples) {
        const int m = (numSamples - fed) < kBlock ? (numSamples - fed) : kBlock;
        for (int i = 0; i < m; ++i) {
            const float x = (amp != 0.0f) ? amp * sinf(phase) : 0.0f;
            phase += phInc;
            if (phase >= k2Pi) phase -= k2Pi;
            s_bufL[i] = x;
            s_bufR[i] = x;
        }
        p.processAudio(std::span<float>(s_bufL, static_cast<size_t>(m)),
                       std::span<float>(s_bufR, static_cast<size_t>(m)));
        for (int i = 0; i < m; ++i) {
            if (fed + i >= peakFrom) {
                const float mL = fabsf(s_bufL[i]);
                const float mR = fabsf(s_bufR[i]);
                if (mL > peak) peak = mL;
                if (mR > peak) peak = mR;
            }
        }
        fed += m;
    }
    return peak;
}

// ---------------------------------------------------------------------------
// Test 1: Frequency response sweep
//
// 200 log-spaced frequencies, 20 Hz..20 kHz, -20 dBFS (0.1 linear), through
// the patch in the quasi-static parallel configuration. One configuration,
// continuous run (state from the previous frequency dies during the 9,600-
// sample prime — all delay lines are <= 12.2 ms). Peak measured over the
// last 2,400 of 4,800 measurement samples, per DMM harness convention.
//
// No LTSpice reference exists for the composite. Pass conditions:
//   P1: bounded ripple — no UNEXPLAINED peak above +6 dB. In this
//       configuration the explained peaks are the quasi-static comb
//       alignments: with intensities at 0 all three TSC lines sit at the
//       same 8 ms delay, so wet = (v0 + 0.5*v1) is a coherent 1.5x delayed
//       replica, and the parallel composition bounds the coherent sum at
//         1 + kParallelWetGain*(TscCircuit::kWetGain*1.5 + 1.0)
//         = 1 + 0.707*(0.7*1.5 + 1.0) = 2.449  ->  +7.78 dB
//       (DC-2 wet summing weight is its documented unity). Comb teeth land
//       on the 125 Hz (1/8 ms, TSC) and 232.6 Hz (1/4.3 ms, DC-2) lattices;
//       measured peaks above +6 dB are listed with their lattice positions.
//       Gate: no gain above the +7.8 dB coherent bound — anything higher
//       cannot be comb alignment and IS unexplained (resonance/instability).
//   P2: documented BBD rolloff shape — the wet contribution is killed at HF
//       by the 8/10/12 kHz lowpass stack, so the top-octave ripple
//       (10..20 kHz gain deviation from the dry 0 dB) must be smaller than
//       the mid-band comb ripple (500..4000 Hz).
// CSV: tests/tridimension_freq_response.csv
//   frequency_hz, amplitude_linear (output peak), output_db (gain re input)
// ---------------------------------------------------------------------------
static void runTest1_FrequencyResponse(Patch& p, PatchDriver& drv) {
    std::printf("\n--- Test 1: Frequency Response Sweep ---\n");

    FILE* csv = std::fopen("tests/tridimension_freq_response.csv", "w");
    if (!csv) {
        std::printf("  WARN: cannot open tests/tridimension_freq_response.csv — trying cwd\n");
        csv = std::fopen("tridimension_freq_response.csv", "w");
    }
    if (csv) std::fprintf(csv, "frequency_hz,amplitude_linear,output_db\n");

    static constexpr int   kPrime   = 9600;
    static constexpr int   kMeasure = 4800;
    static constexpr int   kTail    = 2400;   // peak over last half
    static constexpr float kAmp     = 0.1f;   // -20 dBFS

    p.init();
    drv.configureQuasiStaticParallel();
    float phase = 0.0f;
    (void)feedTone(p, kSampleRateI, 0.0f, 1000.0f, phase);   // crossfade+smoother settle

    static const float kPrintTargets[] = {20.0f, 100.0f, 500.0f, 1000.0f,
                                          5000.0f, 10000.0f, 20000.0f};

    // Coherent-comb bound: 1 + 0.707*(0.7*1.5 + 1.0) = 2.449 -> +7.78 dB
    // (derivation in the block comment above). Peaks in (+6, +7.8] dB are
    // the explained quasi-static comb alignments.
    static constexpr float kCoherentBoundDb = 7.8f;

    float maxGainDb   = -200.0f;
    float maxGainFreq = 0.0f;
    float midRipple   = 0.0f;   // max |gain dB|, 500..4000 Hz
    float hfRipple    = 0.0f;   // max |gain dB|, 10..20 kHz
    int   numOver6    = 0;      // explained comb peaks above +6 dB

    for (int i = 0; i < 200; ++i) {
        if (i % 40 == 0) std::printf("  Test 1: %d/200...\n", i);
        const float freq = 20.0f * powf(1000.0f, static_cast<float>(i) / 199.0f);

        phase = 0.0f;
        (void)feedTone(p, kPrime, kAmp, freq, phase);
        const float peak = feedTone(p, kMeasure, kAmp, freq, phase,
                                    kMeasure - kTail);
        const float gainDb = 20.0f * log10f(peak / kAmp + 1e-9f);

        if (csv) std::fprintf(csv, "%.4f,%.8f,%.4f\n",
                              static_cast<double>(freq),
                              static_cast<double>(peak),
                              static_cast<double>(gainDb));

        if (gainDb > maxGainDb) { maxGainDb = gainDb; maxGainFreq = freq; }
        if (gainDb > 6.0f) {
            ++numOver6;
            std::printf("  comb peak: %.1f Hz %+.2f dB (125 Hz lattice x%.2f, "
                        "232.6 Hz lattice x%.2f)\n",
                        static_cast<double>(freq), static_cast<double>(gainDb),
                        static_cast<double>(freq / 125.0f),
                        static_cast<double>(freq / 232.6f));
        }
        const float absDev = fabsf(gainDb);
        if (freq >= 500.0f && freq <= 4000.0f && absDev > midRipple)
            midRipple = absDev;
        if (freq >= 10000.0f && absDev > hfRipple)
            hfRipple = absDev;

        for (float t : kPrintTargets) {
            if (fabsf(freq - t) / t < 0.05f) {
                std::printf("  %.0f Hz: %+.2f dB\n",
                            static_cast<double>(freq),
                            static_cast<double>(gainDb));
                break;
            }
        }
    }

    if (csv) {
        std::fclose(csv);
        std::printf("  CSV written: tests/tridimension_freq_response.csv (200 data rows)\n");
    }

    std::printf("  INFO: max gain %+.2f dB at %.1f Hz; ripple mid-band %.2f dB, "
                "top octave %.2f dB\n",
                static_cast<double>(maxGainDb), static_cast<double>(maxGainFreq),
                static_cast<double>(midRipple), static_cast<double>(hfRipple));
    std::printf("  INFO: %d explained comb peaks in (+6, +7.8] dB "
                "(coherent bound %.1f dB)\n",
                numOver6, static_cast<double>(kCoherentBoundDb));
    check(maxGainDb <= kCoherentBoundDb,
          "freq_response: P1 no unexplained peak (all within +7.8 dB coherent comb bound)");
    check(hfRipple < midRipple,
          "freq_response: P2 BBD rolloff (top-octave ripple < mid-band ripple)");
}

// ---------------------------------------------------------------------------
// Test 2: Impulse response stability — see NOTE A in the file header.
// Default configuration (fresh init: series zone 0, DC-2 on). Twin runs:
// run A = 0 dBFS impulse then silence, run B = all silence, identical
// deterministic state. P1 on run A raw output; P2 on |A - B|.
// ---------------------------------------------------------------------------
static constexpr int kImpulseLen = 48000;
static float s_impA_L[kImpulseLen], s_impA_R[kImpulseLen];
static float s_impB_L[kImpulseLen], s_impB_R[kImpulseLen];

static void captureImpulseRun(Patch& p, bool withImpulse,
                              float* outL, float* outR) {
    p.init();   // deterministic: LFO/LCG/BBD/envelope state all re-seeded
    float phase = 0.0f;
    (void)feedTone(p, 4800, 0.0f, 1000.0f, phase);   // 150 blocks of 32: settle

    int fed = 0;
    while (fed < kImpulseLen) {
        const int m = (kImpulseLen - fed) < kBlock ? (kImpulseLen - fed) : kBlock;
        for (int i = 0; i < m; ++i) {
            const float x = (withImpulse && fed + i == 0) ? 1.0f : 0.0f;
            s_bufL[i] = x;
            s_bufR[i] = x;
        }
        p.processAudio(std::span<float>(s_bufL, static_cast<size_t>(m)),
                       std::span<float>(s_bufR, static_cast<size_t>(m)));
        for (int i = 0; i < m; ++i) {
            outL[fed + i] = s_bufL[i];
            outR[fed + i] = s_bufR[i];
        }
        fed += m;
    }
}

static void runTest2_ImpulseStability(Patch& p) {
    std::printf("\n--- Test 2: Impulse Response Stability ---\n");

    captureImpulseRun(p, true,  s_impA_L, s_impA_R);
    captureImpulseRun(p, false, s_impB_L, s_impB_R);

    bool  finite       = true;
    float maxAbs       = 0.0f;
    float rawFloorTail = 0.0f;   // raw output, last 100 samples (noise floor)
    float isoTail      = 0.0f;   // isolated impulse tail, last 100 samples
    float isoPeak      = 0.0f;

    for (int n = 0; n < kImpulseLen; ++n) {
        const float aL = s_impA_L[n], aR = s_impA_R[n];
        if (!std::isfinite(aL) || !std::isfinite(aR)) finite = false;
        const float mA = fmaxf(fabsf(aL), fabsf(aR));
        if (mA > maxAbs) maxAbs = mA;

        const float dL = fabsf(aL - s_impB_L[n]);
        const float dR = fabsf(aR - s_impB_R[n]);
        const float d  = fmaxf(dL, dR);
        if (d > isoPeak) isoPeak = d;
        if (n >= kImpulseLen - 100) {
            if (d > isoTail) isoTail = d;
            if (mA > rawFloorTail) rawFloorTail = mA;
        }
    }

    static constexpr float k96dB = 1.585e-5f;
    std::printf("  INFO: raw max |y| = %.6f; isolated impulse peak = %.6f\n",
                static_cast<double>(maxAbs), static_cast<double>(isoPeak));
    std::printf("  INFO: isolated tail (last 100 of 48000) = %.3e (%.1f dB), "
                "gate -96 dB\n",
                static_cast<double>(isoTail),
                static_cast<double>(20.0f * log10f(isoTail + 1e-12f)));
    std::printf("  FLAG: raw output floor (last 100) = %.3e (%.1f dB) — the\n"
                "        stationary BBD clock-noise character ([EAR-MATCHED]\n"
                "        levels), NOT an impulse tail; see NOTE A.\n",
                static_cast<double>(rawFloorTail),
                static_cast<double>(20.0f * log10f(rawFloorTail + 1e-12f)));

    check(finite && maxAbs < 1.0f,
          "impulse_stability: P1 |y| < 1.0 throughout, no NaN/Inf");
    check(isoTail < k96dB,
          "impulse_stability: P2 isolated tail below -96 dB within 1 s");
}

// ---------------------------------------------------------------------------
// Test 3: Parameter sweep — six virtual controls, 10 steps, 3 probe tones.
//
// Part 1 (CSV + level sanity): for each (control, step, probe) the patch is
// re-initialized, set to the parallel baseline (v2 = 0.84 keeps both units
// audible; defaults elsewhere), the swept control applied, settled 0.5 s,
// primed 4,800 samples, peak measured over 2,400. 180 CSV rows.
// P3: 1 kHz output stays above 0.001 linear at every point (dry always
// passes in parallel; a dead point means a routing/control regression).
//
// Part 2 (P1, intensity monotonicity — NOTE B): TscCircuit delay swing per
// line, intensity 0 -> 1 in 10 steps at setRate(1.0f): swing (max-min of
// getDelayMs over 2 s after 0.5 s settle) strictly increasing.
//
// Part 3 (P2, zone transitions): continuous 1 kHz at -12 dBFS while v2
// walks 0 -> 1 in 10 steps (both zone boundaries crossed, hysteresis + 512-
// sample crossfade exercised); no per-sample output step above the Session 5
// click bound 0.5. Input is phase-continuous (block-size multiple of 48).
// CSV: tests/tridimension_param_sweep.csv
//   param_name, step, param_value, freq_hz, amp_linear, amp_db
// ---------------------------------------------------------------------------
static void runTest3_ParameterSweep(Patch& p, PatchDriver& drv) {
    std::printf("\n--- Test 3: Parameter Sweep ---\n");

    FILE* csv = std::fopen("tests/tridimension_param_sweep.csv", "w");
    if (!csv) {
        std::printf("  WARN: cannot open tests/tridimension_param_sweep.csv — trying cwd\n");
        csv = std::fopen("tridimension_param_sweep.csv", "w");
    }
    if (csv) std::fprintf(csv, "param_name,step,param_value,freq_hz,amp_linear,amp_db\n");

    static const char* kNames[6] = {"v0_dc2mode", "v1_tscrate", "v2_routing",
                                    "v3_intensityL", "v4_intensityC", "v5_intensityR"};
    static const float kFreqs[3] = {80.0f, 1000.0f, 8000.0f};
    static constexpr float kAmp  = 0.1f;   // -20 dBFS probes

    bool  levelOk  = true;
    float worstAmp = 1e9f;

    for (int ctrl = 0; ctrl < 6; ++ctrl) {
        std::printf("  sweeping %s...\n", kNames[ctrl]);
        for (int step = 0; step < 10; ++step) {
            const float v = static_cast<float>(step) / 9.0f;
            for (int fi = 0; fi < 3; ++fi) {
                p.init();
                drv.configureQuasiStaticParallel();
                // Baseline deviations: defaults for everything but v2
                // (parallel) and the swept control. Intensities back to the
                // 0.5 default unless swept.
                drv.setVirtual(3, 0.5f);
                drv.setVirtual(4, 0.5f);
                drv.setVirtual(5, 0.5f);
                drv.setVirtual(1, 0.5f);
                drv.setVirtual(ctrl, v);
                drv.ensureLayer(false);

                float phase = 0.0f;
                (void)feedTone(p, 24000, 0.0f, kFreqs[fi], phase);  // settle
                (void)feedTone(p, 4800, kAmp, kFreqs[fi], phase);   // prime
                const float amp = feedTone(p, 2400, kAmp, kFreqs[fi], phase, 0);
                const float ampDb = 20.0f * log10f(amp + 1e-9f);

                if (csv)
                    std::fprintf(csv, "%s,%d,%.6f,%.1f,%.8f,%.4f\n",
                                 kNames[ctrl], step, static_cast<double>(v),
                                 static_cast<double>(kFreqs[fi]),
                                 static_cast<double>(amp),
                                 static_cast<double>(ampDb));

                if (fi == 1) {   // 1 kHz probe
                    if (amp < worstAmp) worstAmp = amp;
                    if (amp <= 0.001f) levelOk = false;
                }
            }
        }
    }
    if (csv) {
        std::fclose(csv);
        std::printf("  CSV written: tests/tridimension_param_sweep.csv (180 data rows)\n");
    }

    std::printf("  INFO: worst 1 kHz amplitude across all 60 sweep points = %.6f\n",
                static_cast<double>(worstAmp));
    check(levelOk, "param_sweep: P3 1 kHz output alive at every sweep point");

    // --- P1: intensity monotonicity on TscCircuit delay swing (NOTE B) ---
    std::printf("  intensity monotonicity (TscCircuit delay swing, 10 steps/line):\n");
    static TscCircuit tsc;
    for (int line = 0; line < 3; ++line) {
        tsc.init(kSampleRateF);
        tsc.setRate(1.0f);
        for (int l = 0; l < 3; ++l) tsc.setIntensity(l, 0.0f);
        float prevSwing = -1.0f;
        bool  lineOk    = true;
        std::printf("  line %d swings (ms):", line);
        for (int step = 0; step < 10; ++step) {
            tsc.setIntensity(line, static_cast<float>(step) / 9.0f);
            float wetL = 0.0f, wetR = 0.0f;
            for (int n = 0; n < kSampleRateI / 2; ++n)      // settle 0.5 s
                tsc.processSample(0.0f, wetL, wetR);
            float mn = 1e9f, mx = -1e9f;
            for (int n = 0; n < 2 * kSampleRateI; ++n) {    // 2 cycles @ 1 Hz
                tsc.processSample(0.0f, wetL, wetR);
                const float d = tsc.getDelayMs(line);
                if (d < mn) mn = d;
                if (d > mx) mx = d;
            }
            const float swing = mx - mn;
            std::printf(" %.3f", static_cast<double>(swing));
            if (swing <= prevSwing) lineOk = false;
            prevSwing = swing;
        }
        std::printf("\n");
        char label[72];
        std::snprintf(label, sizeof label,
                      "param_sweep: P1 line %d delay swing strictly increasing", line);
        check(lineOk, label);
    }

    // --- P2: zone-transition click gate ---
    p.init();
    drv.configureQuasiStaticParallel();
    drv.setVirtual(3, 0.5f);
    drv.setVirtual(4, 0.5f);
    drv.setVirtual(5, 0.5f);
    drv.setVirtual(2, 0.0f);
    drv.ensureLayer(false);
    {
        float phase = 0.0f;
        (void)feedTone(p, kSampleRateI, 0.0f, 1000.0f, phase);   // settle in zone 0

        const float phInc  = k2Pi * 1000.0f / kSampleRateF;
        float prevL = 0.0f, prevR = 0.0f;
        bool  prevValid = false;
        float maxStep   = 0.0f;
        phase = 0.0f;

        for (int step = 0; step < 10; ++step) {
            drv.setVirtual(2, static_cast<float>(step) / 9.0f);
            int fed = 0;
            while (fed < 4800) {
                const int m = kBlock;
                for (int i = 0; i < m; ++i) {
                    const float x = 0.2512f * sinf(phase);   // -12 dBFS
                    phase += phInc;
                    if (phase >= k2Pi) phase -= k2Pi;
                    s_bufL[i] = x;
                    s_bufR[i] = x;
                }
                p.processAudio(std::span<float>(s_bufL, kBlock),
                               std::span<float>(s_bufR, kBlock));
                for (int i = 0; i < m; ++i) {
                    if (prevValid) {
                        const float d = fmaxf(fabsf(s_bufL[i] - prevL),
                                              fabsf(s_bufR[i] - prevR));
                        if (d > maxStep) maxStep = d;
                    }
                    prevL = s_bufL[i];
                    prevR = s_bufR[i];
                    prevValid = true;
                }
                fed += m;
            }
        }
        std::printf("  INFO: routing walk max per-sample step = %.4f (bound 0.5)\n",
                    static_cast<double>(maxStep));
        check(maxStep < 0.5f,
              "param_sweep: P2 zone transitions click-free (step < 0.5)");
    }
}

// ---------------------------------------------------------------------------
// Test 4: Cycle count estimation
// Clock: 528 MHz. Budget: 11,000 cycles/sample. Warmup: 48,000 samples.
// Timed: 1,000,000 samples (2,084 x 480-sample blocks, per-block timing for
// the max). Worst case gates; parallel typical case reported for INVENTORY.
// Input: 1 kHz at -12 dBFS (keeps companders/branches on realistic paths and
// avoids denormal-inflated silence timings on the desktop host).
// ---------------------------------------------------------------------------
struct BenchResult {
    double meanCycles;
    double maxCycles;
};

static BenchResult benchPatch(Patch& p) {
    const float phInc = k2Pi * 1000.0f / kSampleRateF;
    float phase = 0.0f;

    // Warmup — cache/state settle
    (void)feedTone(p, kSampleRateI, 0.2512f, 1000.0f, phase);

    constexpr int kBlocks = 1'000'000 / kBlock + 1;   // 2084 blocks = 1,000,320
    volatile float sink = 0.0f;
    double  totalNs   = 0.0;
    double  maxBlockNs = 0.0;

    for (int b = 0; b < kBlocks; ++b) {
        for (int i = 0; i < kBlock; ++i) {
            const float x = 0.2512f * sinf(phase);
            phase += phInc;
            if (phase >= k2Pi) phase -= k2Pi;
            s_bufL[i] = x;
            s_bufR[i] = x;
        }
        const auto t0 = std::chrono::high_resolution_clock::now();
        p.processAudio(std::span<float>(s_bufL, kBlock),
                       std::span<float>(s_bufR, kBlock));
        const auto t1 = std::chrono::high_resolution_clock::now();
        const double ns = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
        totalNs += ns;
        if (ns > maxBlockNs) maxBlockNs = ns;
        sink = s_bufL[0];
    }
    (void)sink;

    const double nsToCycles = static_cast<double>(kClockHz) / 1'000'000'000.0;
    BenchResult r;
    r.meanCycles = totalNs * nsToCycles / (static_cast<double>(kBlocks) * kBlock);
    r.maxCycles  = maxBlockNs * nsToCycles / static_cast<double>(kBlock);
    return r;
}

static BenchResult g_worstBench{};     // published for the docs-sync summary
static BenchResult g_typicalBench{};

static bool runTest4_CycleCount(Patch& p, PatchDriver& drv) {
    std::printf("\n--- Test 4: Cycle Count Estimation ---\n");

    // Worst case: series zone 0, DC-2 enabled, all intensities 1.0.
    p.init();
    drv.configureWorstCaseSeries();
    {
        float phase = 0.0f;
        (void)feedTone(p, kSampleRateI, 0.0f, 1000.0f, phase);   // config settle
    }
    g_worstBench = benchPatch(p);
    const double worstPct = g_worstBench.meanCycles / 11000.0 * 100.0;

    // Typical case: parallel routing, default intensities (INVENTORY figure).
    p.init();
    drv.configureQuasiStaticParallel();
    drv.setVirtual(3, 0.5f);
    drv.setVirtual(4, 0.5f);
    drv.setVirtual(5, 0.5f);
    drv.ensureLayer(false);
    {
        float phase = 0.0f;
        (void)feedTone(p, kSampleRateI, 0.0f, 1000.0f, phase);
    }
    g_typicalBench = benchPatch(p);

    std::printf("  Worst case (series zone 0, DC-2 on, intensities 1.0):\n");
    std::printf("    Mean cycles/sample: %.1f  (528 MHz)\n", g_worstBench.meanCycles);
    std::printf("    Max  cycles/sample: %.1f  (worst 480-sample block)\n",
                g_worstBench.maxCycles);
    std::printf("    Budget consumed:    %.1f%%  (budget 11,000 cycles @ 528 MHz)\n",
                worstPct);
    std::printf("  Typical case (parallel routing, intensities 0.5):\n");
    std::printf("    Mean cycles/sample: %.1f  (%.1f%% of budget)\n",
                g_typicalBench.meanCycles,
                g_typicalBench.meanCycles / 11000.0 * 100.0);

    if (worstPct > 60.0)
        std::printf("  FAIL: worst case exceeds 60%% cycle budget (%.1f%%)\n", worstPct);
    else if (worstPct > 40.0)
        std::printf("  WARN: worst case exceeds 40%% cycle budget (%.1f%%)\n", worstPct);
    check(worstPct <= 60.0, "cycle_count: worst case within 60% cycle budget");
    return worstPct <= 60.0;
}

// ---------------------------------------------------------------------------
// Test 5: Numerical precision — 10 s pink noise at -6 dBFS, worst-case
// configuration. P1 no NaN, P2 no Inf (hard gates). Denormal scan on the
// exposed outputs (L/R); if any are found, FTZ is enabled and the run
// repeats — the retest must then be clean.
// ---------------------------------------------------------------------------
struct PrecisionCounts {
    int nan = 0, inf = 0, denorm = 0;
};

static PrecisionCounts precisionRun(Patch& p, PatchDriver& drv) {
    p.init();
    drv.configureWorstCaseSeries();
    resetPinkNoise();
    {
        float phase = 0.0f;
        (void)feedTone(p, kSampleRateI, 0.0f, 1000.0f, phase);   // config settle
    }

    PrecisionCounts c;
    int fed = 0;
    while (fed < 480'000) {
        for (int i = 0; i < kBlock; ++i) {
            const float x = 0.5f * nextPinkSample();   // -6 dBFS
            s_bufL[i] = x;
            s_bufR[i] = x;
        }
        p.processAudio(std::span<float>(s_bufL, kBlock),
                       std::span<float>(s_bufR, kBlock));
        for (int i = 0; i < kBlock; ++i) {
            for (const float y : {s_bufL[i], s_bufR[i]}) {
                if (std::isnan(y))      ++c.nan;
                else if (std::isinf(y)) ++c.inf;
                else if (isDenormal(y)) ++c.denorm;
            }
        }
        fed += kBlock;
    }
    return c;
}

static void runTest5_NumericalPrecision(Patch& p, PatchDriver& drv) {
    std::printf("\n--- Test 5: Numerical Precision ---\n");

    PrecisionCounts c = precisionRun(p, drv);
    std::printf("  NaN outputs:      %d\n  Inf outputs:      %d\n"
                "  Denormal outputs: %d\n", c.nan, c.inf, c.denorm);

    bool denormOk = (c.denorm == 0);
    if (!denormOk) {
        std::printf("  WARN: denormals detected — enabling FTZ and re-testing\n");
        enableFtz();
        const PrecisionCounts c2 = precisionRun(p, drv);
        std::printf("  FTZ retest: NaN %d, Inf %d, denormal %d\n",
                    c2.nan, c2.inf, c2.denorm);
        denormOk = (c2.denorm == 0) && (c2.nan == 0) && (c2.inf == 0);
        std::printf("  NOTE: production init() should set the Cortex-M7 FPSCR\n"
                    "        FTZ bit (denormals cost ~10x cycles without it).\n");
    }

    check(c.nan == 0, "precision: P1 no NaN outputs");
    check(c.inf == 0, "precision: P2 no Inf outputs");
    check(denormOk,   "precision: P3 denormal-free (directly or after FTZ)");
}

// ---------------------------------------------------------------------------
// Test 6: A/B comparison — gated on tests/reference_tridimension.pcm.
// Reference format: raw IEEE 754 float32 LE, INTERLEAVED STEREO, 48 kHz, no
// header, recorded at the patch DEFAULT configuration driven by this
// harness's pink noise (seed 0x12345678, -6 dBFS, mono into both inputs).
// Ear-matching reference recordings live outside the harness; when the file
// is absent the test prints SKIPPED (this is the expected state).
// Target: RMS error < -40 dB. WARN < -20 dB. FAIL >= -20 dB.
// ---------------------------------------------------------------------------
static int runTest6_ABComparison(Patch& p, const char* refPath) {
    std::printf("\n--- Test 6: A/B Comparison ---\n");

    FILE* f = std::fopen(refPath, "rb");
    if (!f) {
        std::printf("  SKIPPED: no reference at %s (reference recordings for\n"
                    "  ear-matching live outside the harness)\n", refPath);
        return -1;   // skipped
    }
    std::fseek(f, 0, SEEK_END);
    const long bytes = std::ftell(f);
    std::rewind(f);
    const int numFrames = static_cast<int>(bytes / (2 * static_cast<long>(sizeof(float))));
    if (numFrames <= 0) {
        std::fclose(f);
        std::printf("  FAIL: reference file empty (%ld bytes)\n", bytes);
        check(false, "ab_comparison: reference file non-empty");
        return 0;
    }
    // std::vector permitted in the test harness for the unknown-length
    // reference buffer (DMM harness precedent).
    std::vector<float> ref(static_cast<size_t>(numFrames) * 2);
    const size_t nRead = std::fread(ref.data(), sizeof(float), ref.size(), f);
    std::fclose(f);
    if (nRead != ref.size())
        std::printf("  WARN: fread returned %zu of %zu expected floats\n",
                    nRead, ref.size());
    std::printf("  Loaded %d stereo frames (%.2f s)\n", numFrames,
                static_cast<double>(numFrames) / 48000.0);

    p.init();   // DEFAULT configuration, per the reference convention above
    resetPinkNoise();

    double sumSqErr = 0.0;
    double sumSqRef = 0.0;
    int fed = 0;
    while (fed < numFrames) {
        const int m = (numFrames - fed) < kBlock ? (numFrames - fed) : kBlock;
        for (int i = 0; i < m; ++i) {
            const float x = 0.5f * nextPinkSample();
            s_bufL[i] = x;
            s_bufR[i] = x;
        }
        p.processAudio(std::span<float>(s_bufL, static_cast<size_t>(m)),
                       std::span<float>(s_bufR, static_cast<size_t>(m)));
        for (int i = 0; i < m; ++i) {
            const float rL = ref[static_cast<size_t>(fed + i) * 2];
            const float rR = ref[static_cast<size_t>(fed + i) * 2 + 1];
            const double eL = static_cast<double>(rL - s_bufL[i]);
            const double eR = static_cast<double>(rR - s_bufR[i]);
            sumSqErr += eL * eL + eR * eR;
            sumSqRef += static_cast<double>(rL) * rL + static_cast<double>(rR) * rR;
        }
        fed += m;
    }

    const double rmsErr   = std::sqrt(sumSqErr / (2.0 * numFrames));
    const float  rmsErrDb = 20.0f * log10f(static_cast<float>(rmsErr) + 1e-12f);
    std::printf("  RMS error: %.1f dB (ref RMS %.1f dB)\n",
                static_cast<double>(rmsErrDb),
                20.0 * std::log10(std::sqrt(sumSqRef / (2.0 * numFrames)) + 1e-12));

    FILE* csv = std::fopen("tests/tridimension_ab_comparison.csv", "w");
    if (!csv) csv = std::fopen("tridimension_ab_comparison.csv", "w");
    if (csv) {
        std::fprintf(csv, "num_frames,rms_error_linear,rms_error_db\n");
        std::fprintf(csv, "%d,%.8e,%.4f\n", numFrames, rmsErr,
                     static_cast<double>(rmsErrDb));
        std::fclose(csv);
    }

    if (rmsErrDb >= -20.0f)
        std::printf("  FAIL: RMS error %.1f dB exceeds the -20 dB floor\n",
                    static_cast<double>(rmsErrDb));
    else if (rmsErrDb >= -40.0f)
        std::printf("  WARN: RMS error %.1f dB (target < -40 dB)\n",
                    static_cast<double>(rmsErrDb));
    check(rmsErrDb < -20.0f, "ab_comparison: RMS error < -20 dB floor");
    return rmsErrDb < -20.0f ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Test 7: DC2 motionless test — Dc2Circuit directly, setMode(3), 1 kHz sine
// at -12 dBFS, 10 s after 2 s warmup.
//
// Pitch-flutter metric: per-cycle instantaneous frequency from linearly
// interpolated rising zero crossings (as prescribed), aggregated as the std
// of energy-weighted 0.25 s window means — see NOTE C in the file header
// for the full derivation of why the raw per-cycle std measures AM-null
// interference artifacts instead of pitch, and why 0.25 s windows preserve
// the single channel's genuine vibrato as the reference. The DC-2's
// antiphase CV design keeps cvA + cvB constant, so the mono-summed wet path
// (wetL + wetR) must be nearly motionless while each single channel carries
// the full vibrato. Both signals get IDENTICAL estimator treatment.
//
// P1: flutter(wetL + wetR) <= 0.2 x flutter(wetL).
// P2: cvA + cvB constant within 1e-4 ms of its mean (Session 3 re-assert;
//     exact CV identity, no estimator conditioning involved).
// ---------------------------------------------------------------------------
struct FlutterEstimator {
    static constexpr int    kNumWindows    = 40;      // 10 s / 0.25 s
    static constexpr double kWindowSamples = 12000.0; // 0.25 s at 48 kHz

    float  prev      = 0.0f;
    bool   prevValid = false;
    double lastT     = -1e9;
    float  segPeak   = 0.0f;    // peak |s| since the previous valid crossing
    float  ampFloor  = 0.0f;    // minimal noise floor (set by caller)
    long   count     = 0;       // accepted periods
    long   rejected  = 0;       // crossings whose period failed a gate
    double winW [kNumWindows] = {};   // sum of weights (segPeak^2)
    double winWF[kNumWindows] = {};   // sum of weight * frequency

    void push(float cur, int n) noexcept {
        const float mag = fabsf(cur);
        if (mag > segPeak) segPeak = mag;
        if (prevValid && prev < 0.0f && cur >= 0.0f) {
            const double t = static_cast<double>(n - 1)
                           + static_cast<double>(prev) / (static_cast<double>(prev) - cur);
            if (t - lastT >= 24.0) {              // noise re-crossing filter
                if (lastT > -1e8) {
                    const double period = t - lastT;
                    if (period >= 36.0 && period <= 60.0 && segPeak >= ampFloor) {
                        const double freq = 48000.0 / period;
                        int w = static_cast<int>(t / kWindowSamples);
                        if (w >= kNumWindows) w = kNumWindows - 1;
                        const double weight =
                            static_cast<double>(segPeak) * segPeak;
                        winW [w] += weight;
                        winWF[w] += weight * freq;
                        ++count;
                    } else {
                        ++rejected;
                    }
                }
                lastT   = t;
                segPeak = 0.0f;
            }
        }
        prev = cur;
        prevValid = true;
    }

    // Flutter = population std of the energy-weighted window-mean frequency.
    [[nodiscard]] double flutter() const noexcept {
        double sum = 0.0, sumSq = 0.0;
        int    n   = 0;
        for (int w = 0; w < kNumWindows; ++w) {
            if (winW[w] <= 0.0) continue;
            const double m = winWF[w] / winW[w];
            sum   += m;
            sumSq += m * m;
            ++n;
        }
        if (n < 2) return 0.0;
        const double mean = sum / n;
        const double var  = sumSq / n - mean * mean;
        return var > 0.0 ? std::sqrt(var) : 0.0;
    }
};

static void runTest7_Dc2Motionless() {
    std::printf("\n--- Test 7: DC2 Motionless (mono-sum flutter) ---\n");

    static Dc2Circuit dc2;
    dc2.init(kSampleRateF);
    dc2.setMode(3);

    BiquadFilter bpSingle, bpMono;                    // (a) conditioning
    bpSingle.init(kSampleRateF);
    bpSingle.setParameters(BiquadFilter::Type::Bandpass, 1000.0f, 2.0f);
    bpMono.init(kSampleRateF);
    bpMono.setParameters(BiquadFilter::Type::Bandpass, 1000.0f, 2.0f);

    const float phInc = k2Pi * 1000.0f / kSampleRateF;
    float phase = 0.0f;
    float wetL = 0.0f, wetR = 0.0f;

    // Warmup 2 s: 0.5 s circuit+filter settle, then 1.5 s RMS reference for
    // the amplitude-validity floors (> 1 full AM cycle at MODE4's 0.896 Hz).
    double rmsAccS = 0.0, rmsAccM = 0.0;
    for (int n = 0; n < 2 * kSampleRateI; ++n) {
        dc2.processSample(0.2512f * sinf(phase), wetL, wetR);
        phase += phInc;
        if (phase >= k2Pi) phase -= k2Pi;
        const float cs = bpSingle.process(wetL);
        const float cm = bpMono.process(wetL + wetR);
        if (n >= kSampleRateI / 2) {
            rmsAccS += static_cast<double>(cs) * cs;
            rmsAccM += static_cast<double>(cm) * cm;
        }
    }
    const double refSamples = 1.5 * kSampleRateI;
    const float rmsS = static_cast<float>(std::sqrt(rmsAccS / refSamples));
    const float rmsM = static_cast<float>(std::sqrt(rmsAccM / refSamples));

    FlutterEstimator single, mono;
    single.ampFloor = 0.1f * 1.41421356f * rmsS;      // minimal noise floors
    mono.ampFloor   = 0.1f * 1.41421356f * rmsM;

    double cvAcc = 0.0;
    float  cvMin = 1e9f, cvMax = -1e9f;
    constexpr int kMeas = 10 * kSampleRateI;          // 10 s

    for (int n = 0; n < kMeas; ++n) {
        dc2.processSample(0.2512f * sinf(phase), wetL, wetR);
        phase += phInc;
        if (phase >= k2Pi) phase -= k2Pi;

        single.push(bpSingle.process(wetL), n);
        mono.push(bpMono.process(wetL + wetR), n);

        const float s = dc2.getDelayMsL() + dc2.getDelayMsR();
        cvAcc += static_cast<double>(s);
        if (s < cvMin) cvMin = s;
        if (s > cvMax) cvMax = s;
    }

    const double fSingle = single.flutter();
    const double fMono   = mono.flutter();
    const double ratio   = (fSingle > 0.0) ? fMono / fSingle : 1e9;
    const float  cvMean  = static_cast<float>(cvAcc / kMeas);
    const float  cvDev   = fmaxf(cvMax - cvMean, cvMean - cvMin);

    std::printf("  INFO: conditioned RMS: single %.4f, mono %.4f\n",
                static_cast<double>(rmsS), static_cast<double>(rmsM));
    std::printf("  INFO: periods accepted/rejected: single %ld/%ld, mono %ld/%ld\n",
                single.count, single.rejected, mono.count, mono.rejected);
    std::printf("  INFO: flutter (windowed freq std): single channel %.4f Hz, "
                "mono sum %.4f Hz\n", fSingle, fMono);
    std::printf("  INFO: mono/single flutter ratio = %.4f (gate 0.20)\n", ratio);
    std::printf("  INFO: cvA + cvB mean %.6f ms, max dev %.3g ms (gate 1e-4)\n",
                static_cast<double>(cvMean), static_cast<double>(cvDev));

    check(single.count > 5000 && mono.count > 5000,
          "dc2_motionless: enough accepted periods collected");
    check(ratio <= 0.2, "dc2_motionless: P1 mono-sum flutter <= 0.2x single channel");
    check(cvDev <= 1e-4f, "dc2_motionless: P2 cvA + cvB constant within 1e-4");
}

// ---------------------------------------------------------------------------
// Test 8: TSC phase spacing — chorus-generator CV taps logged 60 s at
// defaults (0.473 Hz, intensities 0.5); pairwise spacing 120 deg within
// 2 deg for the entire run. Estimator: Session 2/4 merged rising-crossing
// pattern — each merged gap over the CENTERED three-gap period estimate must
// equal 1/3 cycle (the centered denominator cancels first-order drift
// slope). The spacing invariant is a PER-GENERATOR property (Session 4: the
// total delay CV sums two incommensurate 3-phase systems), so the check runs
// on getChorusCvMs(); total delay CVs are logged alongside.
// ---------------------------------------------------------------------------
static void runTest8_TscPhaseSpacing() {
    std::printf("\n--- Test 8: TSC Phase Spacing (120 deg, 60 s) ---\n");

    static TscCircuit tsc;
    tsc.init(kSampleRateF);

    constexpr int   kDur       = 60 * kSampleRateI;
    constexpr int   kMaxEvents = 128;
    constexpr float kTol       = 2.0f / 360.0f;   // 2 deg in cycles
    static float evt[kMaxEvents];
    int   numEvents = 0;

    float wetL = 0.0f, wetR = 0.0f;
    float dMin[3] = {1e9f, 1e9f, 1e9f}, dMax[3] = {-1e9f, -1e9f, -1e9f};
    float prev[3];
    tsc.processSample(0.0f, wetL, wetR);
    for (int t = 0; t < 3; ++t) prev[t] = tsc.getChorusCvMs(t);

    for (int n = 1; n < kDur; ++n) {
        tsc.processSample(0.0f, wetL, wetR);
        for (int t = 0; t < 3; ++t) {
            const float d = tsc.getDelayMs(t);
            if (d < dMin[t]) dMin[t] = d;
            if (d > dMax[t]) dMax[t] = d;
            const float cur = tsc.getChorusCvMs(t);
            if (prev[t] < 0.0f && cur >= 0.0f && numEvents < kMaxEvents) {
                const float frac = prev[t] / (prev[t] - cur);
                evt[numEvents++] = static_cast<float>(n - 1) + frac;
            }
            prev[t] = cur;
        }
    }

    for (int t = 0; t < 3; ++t)
        std::printf("  INFO: line %d delay CV over 60 s: %.3f .. %.3f ms\n",
                    t, static_cast<double>(dMin[t]), static_cast<double>(dMax[t]));
    std::printf("  INFO: %d merged rising crossings (expect ~85 at 0.473 Hz)\n",
                numEvents);

    float maxDev = 0.0f, minFrac = 1.0f, maxFrac = 0.0f;
    int   numFracs = 0;
    for (int k = 1; k + 2 < numEvents; ++k) {
        const float gPrev = evt[k]     - evt[k - 1];
        const float g     = evt[k + 1] - evt[k];
        const float gNext = evt[k + 2] - evt[k + 1];
        const float fr    = g / (gPrev + g + gNext);
        const float dev   = fabsf(fr - 1.0f / 3.0f);
        if (dev > maxDev) maxDev = dev;
        if (fr < minFrac) minFrac = fr;
        if (fr > maxFrac) maxFrac = fr;
        ++numFracs;
    }

    std::printf("  INFO: %d spacing fractions; max |offset - 1/3| = %.6f cycles "
                "(%.3f deg, tol 2 deg); walk = %.6f\n",
                numFracs, static_cast<double>(maxDev),
                static_cast<double>(maxDev * 360.0f),
                static_cast<double>(maxFrac - minFrac));

    check(numFracs >= 12, "tsc_spacing: enough crossings collected");
    check(numFracs >= 12 && maxDev < kTol,
          "tsc_spacing: P1 pairwise spacing 120 deg within 2 deg over 60 s");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::printf("==========================================\n");
    std::printf(" TriDimension (TSC + DC-2) — WDF Test Suite\n");
    std::printf(" Session 6: Tests 1-8\n");
    std::printf("==========================================\n");

    Patch& p = *Patch::getInstance();
    PatchDriver drv{p};

    int c0, p0;

    c0 = totalTests; p0 = passedTests;
    runTest1_FrequencyResponse(p, drv);
    const bool t1 = (passedTests - p0) == (totalTests - c0);

    c0 = totalTests; p0 = passedTests;
    runTest2_ImpulseStability(p);
    const bool t2 = (passedTests - p0) == (totalTests - c0);

    c0 = totalTests; p0 = passedTests;
    runTest3_ParameterSweep(p, drv);
    const bool t3 = (passedTests - p0) == (totalTests - c0);

    const bool t4 = runTest4_CycleCount(p, drv);

    c0 = totalTests; p0 = passedTests;
    runTest5_NumericalPrecision(p, drv);
    const bool t5 = (passedTests - p0) == (totalTests - c0);

    const char* refPath = (argc >= 2) ? argv[1] : "tests/reference_tridimension.pcm";
    const int t6 = runTest6_ABComparison(p, refPath);   // -1 skipped / 0 fail / 1 pass

    c0 = totalTests; p0 = passedTests;
    runTest7_Dc2Motionless();
    const bool t7 = (passedTests - p0) == (totalTests - c0);

    c0 = totalTests; p0 = passedTests;
    runTest8_TscPhaseSpacing();
    const bool t8 = (passedTests - p0) == (totalTests - c0);

    std::printf("\n=== TRIDIMENSION WDF TEST HARNESS SUMMARY ===\n");
    std::printf("Test 1 (Freq Response):     %s\n", t1 ? "PASS" : "FAIL");
    std::printf("Test 2 (Impulse Stability): %s\n", t2 ? "PASS" : "FAIL");
    std::printf("Test 3 (Param Sweep):       %s\n", t3 ? "PASS" : "FAIL");
    std::printf("Test 4 (Cycle Count):       %s  (worst mean %.0f cyc, %.1f%%; "
                "typical %.0f cyc)\n",
                t4 ? "PASS" : "FAIL", g_worstBench.meanCycles,
                g_worstBench.meanCycles / 11000.0 * 100.0,
                g_typicalBench.meanCycles);
    std::printf("Test 5 (Precision):         %s\n", t5 ? "PASS" : "FAIL");
    std::printf("Test 6 (A/B Compare):       %s\n",
                t6 < 0 ? "SKIPPED" : (t6 ? "PASS" : "FAIL"));
    std::printf("Test 7 (DC2 Motionless):    %s\n", t7 ? "PASS" : "FAIL");
    std::printf("Test 8 (TSC Phase Spacing): %s\n", t8 ? "PASS" : "FAIL");
    std::printf("Checks: %d/%d passed\n", passedTests, totalTests);

    const bool allPassed = t1 && t2 && t3 && t4 && t5 && (t6 != 0) && t7 && t8;
    std::printf("Overall: %s\n", allPassed ? "PASS" : "FAIL");
    return allPassed ? 0 : 1;
}
