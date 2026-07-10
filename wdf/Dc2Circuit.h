#pragma once

// Boss DC-2 Dimension C — Complete circuit assembly (Session 3 of 6)
//
// Component source: reference/DC2_component_reference.md (value authority;
// Aion FX Blueshift redraw transcription). Every numeric constant below
// cites a designator from that document. Equation sources:
//   reference/philipsne570ne571sa571applicationnotes.pdf (AN174) — NE570
//     gain-computer equations and CRECT attack/release datapoints
//   reference/MN3207.pdf — 1024-stage BBD, delay = 512/fCP, fCP 10–200 kHz
//   reference/panasonicmn3102.pdf — clock driver, fCP = fOSC/2
//
// SIGNAL CHAIN (mono in -> stereo wet out; per ref doc §1–§12):
//   in -> Dc2InputBuffer -> Dc2PreEmphasis (dry rail, inverted)
//      -> Ne570Compressor (IC2 half 1 + D3/D4 zener clamp) -> Dc2AntiAliasLpf
//      -> split:  BBD channel C (IC9/IC8, CV = LFO tap 1) -> Dc2ReconLpf
//                   -> Ne570Expander (IC11 half 2) -> mixer IC12A -> wetL
//                 BBD channel B (IC3/IC4, CV = LFO tap 0) -> Dc2ReconLpf
//                   -> Ne570Expander (IC11 half 1) -> mixer IC12B -> wetR
//   dry taps: dry rail -> Dc2DryLowpass -> both mixers (R87/R88);
//             dry rail -> mixers via R71/R92 — factored out as kDryGain
//             (see OUTPUT CONVENTION); cross-feedback bridged-T between
//             the two mixer outputs (one-sample loop delay, Valimaki et
//             al., DAFX 2011 §5.3 — DmmDelayCircuit precedent).
//
// TOPOLOGY NOTE — SHARED COMPRESSOR / ANTI-ALIAS FILTER: the transcription
// (ref doc §3 "single, shared by both channels" and §4 "single, shared")
// shows ONE NE570 compressor half (IC2 half 1; half 2 is unity-tied,
// unused) and ONE Q5 anti-alias ladder, splitting to the two BBD channels
// at the Q5 emitter bus (R62/R29). The session prompt's flow sketch says
// "per channel: NE570 compressor -> AA" — the transcription is the value
// authority, so ONE shared compressor + AA is implemented. The expanders
// ARE per channel (IC11 halves 1 and 2), as transcribed. Total NE570
// gain-computer halves modeled: 3 (1 compressor + 2 expanders).
//
// POLARITY: hardware inversion count to each mixer summing node —
//   dry rail: IC1B (x1, odd)   wet: IC1B + NE570 comp + NE570 exp (x3, odd)
// so dry and wet arrive at the mixers with the SAME sign, and the mixers
// invert once more. The behavioral NE570 blocks below are non-inverting
// (their two hardware inversions cancel pairwise), and Dc2PreEmphasis /
// Dc2OutputMixer carry their physical signs — every relative polarity in
// the model therefore matches the hardware, and OUT_L/OUT_R are in-phase
// with the dry input (ref doc "Session-1 modeling decisions" #6).
//
// OUTPUT CONVENTION (Session 5 interface shape): processSample() returns
// WET-ONLY outputs. The R71/R92 direct-dry mixer path is exactly flat in
// the digital model (Dc2PreEmphasis and the mixer Zf shelf are exact
// inverses at matched prewarp — see Dc2Filters.h), so it reduces to the
// scalar kDryGain = (R9||R10)/R71 = 0.75188. The caller composes:
//   outL = in * Dc2Circuit::kDryGain + wetL   (same for R)
// The frequency-shaping dry contributions (Dc2DryLowpass low-end
// reinforcement via R87/R88, and dry circulating through the bridged-T
// cross-feedback) cannot be reduced to a scalar and are computed INSIDE
// processSample as part of the wet outputs. The cross-feedback taps the
// opposite channel's TOTAL output, which is reconstructed internally as
// wet + kDryGain*in. The only dry-path approximation is skipping the
// 3.35 Hz input-buffer HPF for the scalar dry (error <= 0.012 dB at
// 20 Hz, ref doc "decisions" #3/#4 convention).
//
// 1N914 / 2N5087 NOTE: per the transcription the only 1N914/2N5087
// devices in the wet path are in the clock servo (D6/D7, Q7/Q8 — absorbed
// into the clock/LFO derivation below) and the mono-sense switch (D5 —
// not modeled, stereo target). The expander output networks contain no
// nonlinear devices; their couplings are sub-audio (ref doc §10,
// "decisions" #4) and are modeled as unity.
//
// CYCLE BUDGET (528 MHz / 48 kHz = 11,000 cycles/sample; whole-patch
// WARN > 4,400, FAIL > 6,600):
//   TriPhaseLfo<2> tick        ~100–200  (2 sinf drift + 2 triangle taps)
//   Session-1 filters (stereo) ~290–505  (see Dc2Filters.h)
//   Ne570Compressor            ~100–200  (env follower + 1 sqrtf)
//   Ne570Expander (x2)         ~160–300  (env follower + mul, no sqrt)
//   BBDLine<1024> (x2)         ~300–500  (Hermite read + clock noise + LP)
//   setDelayMs (x2), totals    ~30–60
//   ─────────────────────────────────────
//   TOTAL ESTIMATED            ~980–1,765 cycles/sample (9–16% of budget)
//   Under WARN threshold; headroom for the Session 5 TSC voice + patch.
// FAST MATH: exactly ONE sqrtf per sample (compressor gain computer).
// Cortex-M7 VSQRT.F32 = 14 cycles — exact sqrtf is used; substitute an
// approximation behind WDF_USE_FAST_MATH only if the Session 6 benchmark
// demands it (none provided yet, per session rule).
//
// Desktop compile gate (single command line, wrapped for readability):
//   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra
//       -DENDLESS_DESKTOP_BUILD -I. -I./sdk -I./wdf -I./dsp
//       tests/dc2_circuit_test.cpp -lm -o dc2_circuit_test

#include "wdf/Dc2Filters.h"
#include "dsp/BBDLine.h"
#include "dsp/EnvelopeFollower.h"
#include "dsp/TriPhaseLfo.h"
#include "dsp/Saturation.h"

#include <array>
#include <cmath>

// ---------------------------------------------------------------------------
// NE570 behavioral gain computers (AN174; DmmCompander.h precedent).
//
// Gain cell physics (AN174 p.769): IB = 140 uA, internal rectifier R = 10k,
// 2:1 companding. The rectifier produces a full-wave AVERAGE of the input;
// gain is proportional to that average (expander) or its inverse square
// root (compressor: expander cell in the op-amp feedback loop). Modeled
// per the DMM precedent as envelope + algebraic gain — no Shockley-level
// modeling.
//
// TIME CONSTANTS (AN174 "Attack and Decay Time", p.771): rectifier filter
// tau = 10k x CRECT; the CCITT datapoint CRECT = 2 uF gives ~3 ms attack
// and ~13.5 ms release, i.e. 1.5 ms/uF attack and 6.75 ms/uF release.
// DC-2 CRECT values (all 0.47 uF tantalum, ref doc §3/§10):
//   compressor  C10 = 0.47 uF -> attack 1.5 x 0.47  = 0.705 ms
//                                release 6.75 x 0.47 = 3.1725 ms
//   expander C  C48 = 0.47 uF -> same 0.705 / 3.1725 ms
//   expander B  C47 = 0.47 uF -> same 0.705 / 3.1725 ms
// The 4.5:1 attack/release asymmetry is the NE570 rectifier's signature
// and is preserved exactly (project rule from the DMM build: NEVER
// equalize compander timing). Compressor and expanders happen to share
// CRECT = 0.47 uF in this circuit, so their time constants match each
// other — that is transcribed fact, not equalization.
//
// ENVELOPE: EnvelopeFollower in Peak mode is |x| into a one-pole with
// independent attack/release — exactly the AN174 full-wave rectifier +
// CRECT filter (fast charge through the rectifier, slow discharge through
// the internal 10k). The detector is homogeneous of degree 1 (scaling the
// input scales the envelope), which makes compressor->expander identity
// hold for ANY envRef in steady state; envRef only sets the compander
// unity level.
//
// envRef [CALIBRATED]: 0.1f — unity compressor gain at env = 0.1
// (~ -18 dBFS sine), matching the DmmCompander kVrefInt convention.
// Self-test measured residual (compressor->expander, 1 kHz sine):
// |error| <= 0.004 dB across -40..0 dBFS (gate: 0.5 dB).
//
// ZENER CLAMP: D3/D4 (1N5225 3.0V back-to-back) + C16 clamp the
// compressor output before the BBDs (ref doc §3 — overload protection,
// same role as AN174 Fig. 20 D3/D4). Physical clamp ~±3.6 V maps to
// ±1.0f normalized full scale [CALIBRATED]; sat::hardClip clamps at ±1.
// Steady-state compressor output at 0 dBFS input is ~0.35f, so the clamp
// engages only on transients that outrun the 0.705 ms attack — the
// zeners' hardware role.
// ---------------------------------------------------------------------------
struct Ne570Compressor {
    // [DTCM] per-sample state (envelope accumulator inside)
    EnvelopeFollower follower;

    static constexpr float kCrectUf      = 0.47f;               // C10
    static constexpr float kCompAttackMs  = 1.5f  * kCrectUf;   // 0.705 ms
    static constexpr float kCompReleaseMs = 6.75f * kCrectUf;   // 3.1725 ms
    static constexpr float kEnvRef  = 0.1f;    // [CALIBRATED] see block comment
    static constexpr float kEps     = 1e-6f;   // rectifier floor (AN174: bias
                                               // current limits range to ~60 dB)
    static constexpr float kGainMin = 0.1f;    // DmmCompander clamp convention
    static constexpr float kGainMax = 10.0f;

    void init(float sampleRate) noexcept {
        follower.init(sampleRate);
        follower.setMode(EnvelopeFollower::Mode::Peak);   // |x| full-wave detect
        follower.setAttackMs(kCompAttackMs);
        follower.setReleaseMs(kCompReleaseMs);
    }

    [[nodiscard]] float process(float x) noexcept {
        float env = follower.process(x);
        if (__builtin_expect(env < kEps, 0)) env = kEps;
        // compressorGain = 1/sqrt(env/envRef); exact sqrtf (VSQRT.F32, 14 cyc)
        float gain = std::sqrt(kEnvRef / env);
        if (__builtin_expect(gain > kGainMax, 0)) gain = kGainMax;
        if (__builtin_expect(gain < kGainMin, 0)) gain = kGainMin;
        // D3/D4 zener clamp at the compressor output (see block comment)
        return sat::hardClip(x * gain);
    }

    void reset() noexcept { follower.reset(); }
};

struct Ne570Expander {
    // [DTCM] per-sample state
    EnvelopeFollower follower;

    static constexpr float kCrectUf     = 0.47f;               // C47 / C48
    static constexpr float kExpAttackMs  = 1.5f  * kCrectUf;   // 0.705 ms
    static constexpr float kExpReleaseMs = 6.75f * kCrectUf;   // 3.1725 ms
    static constexpr float kEnvRef  = Ne570Compressor::kEnvRef;
    static constexpr float kEps     = Ne570Compressor::kEps;
    static constexpr float kGainMin = Ne570Compressor::kGainMin;
    static constexpr float kGainMax = Ne570Compressor::kGainMax;

    void init(float sampleRate) noexcept {
        follower.init(sampleRate);
        follower.setMode(EnvelopeFollower::Mode::Peak);
        follower.setAttackMs(kExpAttackMs);
        follower.setReleaseMs(kExpReleaseMs);
    }

    [[nodiscard]] float process(float x) noexcept {
        // AN174 basic expandor: rectifier and dG cell both tied to the INPUT
        // (IC11 pins 2+3 / 14+15 tied, ref doc §10); gain = env/envRef.
        float env = follower.process(x);
        if (__builtin_expect(env < kEps, 0)) env = kEps;
        float gain = env / kEnvRef;
        if (__builtin_expect(gain > kGainMax, 0)) gain = kGainMax;
        if (__builtin_expect(gain < kGainMin, 0)) gain = kGainMin;
        return x * gain;
    }

    void reset() noexcept { follower.reset(); }
};

// ---------------------------------------------------------------------------
// Mode presets — derived from the transcribed TL022 LFO (ref doc §7) and
// the MODE1..4 switch resistor selections, plus the clock servo (§6/§8).
//
// LFO RATE (standard op-amp triangle oscillator, ElectroSmash CE-2 form):
// comparator IC7A flips when the node-T superposition cancels:
//   (Vtri - VA/2)/R33 + (Vsq - VA/2)/R35 = 0  =>  A_tri = (R33/R35)*A_sq
// (R34 100k to VA/2 carries no current at the flip point; C37 10n is a
// transition speed-up — neither affects the frequency.) Integrator IC7B
// slope = A_sq/(R_int*C29), R_int = R36_eff + R37. Half period =
// 2*A_tri/slope, so A_sq cancels:
//   f = R35 / (4 * R33 * R_int * C29)
// MODE1 (all off):    R_int = 330k + 180k        = 510k   -> f = 0.3161 Hz
// MODE2 (SPDT):       rate side untouched                 -> f = 0.3161 Hz
// MODE3 (R98 82k):    R_int = (330k||82k) + 180k = 245.7k -> f = 0.6561 Hz
// MODE4 (R36 short):  R_int = 180k               -> f = 0.8955 Hz
//
// CV DEPTH: triangle amplitude A_tri = (R33/R35)*A_sq = 0.33*A_sq with
// A_sq ~= 2.1 V (TL022 swing ~1.3 V from each rail on the 6.8 V VA supply,
// about VA/2) -> A_tri ~= 0.693 V. IC5B inverting amp: CV_B amplitude =
// A_tri * R45/(R38_eff + R39); IC6A re-inverts so CV_C = -CV_B (§8) —
// realized here as the LFO tap pair at phase 0.0 / 0.5 (triangle
// antiphase identity tri(x) + tri(x+0.5) = 0, TriPhaseLfo header).
// MODE1: R38_eff = 22k         -> 0.693*47/55    = 0.592 V
// MODE2: R38 shorted           -> 0.693*47/33    = 0.987 V
// MODE3: R38||R97 470k = 21.0k -> 0.693*47/54.0  = 0.603 V
// MODE4: R38||R99 220k = 20.0k -> 0.693*47/53    = 0.614 V
//
// CLOCK / DELAY MAPPING (MN3102 + MN3207 datasheets, servo §6):
//   MN3102: fCP = fOSC/2 (two-phase divider).
//   MN3207: delay = (1024/2)/fCP = 512/fCP; fCP permitted 10–200 kHz
//           => delay envelope 2.56–51.2 ms.
// Servo ramp current (Q8 2N5087 current source):
//   VB(Q8) = VA*R58/(R56+R58) = 6.8*10/11.5 = 5.91 V; VEB = 0.65 V
//   I = (VA - VB - VEB)/R55 = (6.8 - 5.91 - 0.65)/8.2k ~= 29 uA
// Comparator threshold sensitivity to CV (R54/R49 divider with R50
// hysteresis): k_cv = (R49||R50)/(R54 + R49||R50) = 4.489k/10.089k = 0.445.
// Sawtooth VCO: t_cycle = C35 * dV/I with dV = k_cv*CV + offsets, so
//   delay = 1024 * t_cycle  and
//   d(delay)/d(CV) = 1024 * (C35/I) * k_cv
//                  = 1024 * (100pF/29uA) * 0.445 = 1.57 ms/V.
// CENTER DELAY [CALIBRATED 4.3 ms]: at CV = VA/2 = 3.4 V the threshold is
// k_cv*3.4 = 1.51 V (+ ~0.14 V hysteresis with the output high, - I*R57 =
// 0.20 V sense offset); the ramp span depends on the D7/OX3 reset clamp
// level, which the transcription bounds only to ~±0.3 V — giving a derived
// center window of ~3.0–4.4 ms. 4.3 ms is chosen inside that window so
// that the full MODE2 swing stays within the MN3207 fCP envelope
// (min delay 2.75 ms -> fCP 186 kHz < 200 kHz) and sits in the
// low-single-digit-millisecond region of the community measurement
// record. Sanity anchor satisfied — no STOP condition.
//
// depthMs = CV amplitude x 1.57 ms/V:
//   MODE1: 0.93 ms   MODE2: 1.55 ms   MODE3: 0.95 ms   MODE4: 0.97 ms
// Resulting modulation intensity (peak pitch deviation, 4*depth*rate):
//   MODE1 2.0 / MODE2 3.4 / MODE3 4.3 / MODE4 6.0 cents — monotonically
// increasing 1->4, matching the hardware's intensity-ordered mode buttons;
// modes 1,2 slower and 3,4 faster per the community record (Axe-Fx match
// near 0.3 Hz is the MODE1/2 rate). Required ordering holds — no STOP.
// ---------------------------------------------------------------------------
struct Dc2ModePreset {
    float rateHz;
    float depthMs;
};

// ---------------------------------------------------------------------------
// Dc2Circuit — complete DC-2, mono in / stereo wet out.
// ---------------------------------------------------------------------------
class Dc2Circuit {
public:
    // Derived per-mode table (derivation in the block comment above).
    static constexpr std::array<Dc2ModePreset, 4> kModeTable{{
        {0.316f, 0.93f},   // MODE1
        {0.316f, 1.55f},   // MODE2
        {0.656f, 0.95f},   // MODE3
        {0.896f, 0.97f},   // MODE4
    }};

    // Clock-servo center delay (see CLOCK / DELAY MAPPING above).
    static constexpr float kCenterDelayMs = 4.3f;

    // Required qualitative outcome: modes 1,2 slower than modes 3,4.
    static_assert(kModeTable[0].rateHz < kModeTable[2].rateHz &&
                  kModeTable[0].rateHz < kModeTable[3].rateHz &&
                  kModeTable[1].rateHz < kModeTable[2].rateHz &&
                  kModeTable[1].rateHz < kModeTable[3].rateHz,
                  "DC-2 mode ordering violated: modes 1,2 must be slower");
    // Full modulation swing must respect the MN3207 fCP 10-200 kHz envelope
    // (delay 2.56-51.2 ms) in every mode.
    static_assert(kCenterDelayMs - kModeTable[1].depthMs >= 2.56f &&
                  kCenterDelayMs + kModeTable[1].depthMs <= 51.2f &&
                  kModeTable[0].depthMs <= kModeTable[1].depthMs &&
                  kModeTable[2].depthMs <= kModeTable[1].depthMs &&
                  kModeTable[3].depthMs <= kModeTable[1].depthMs,
                  "DC-2 delay swing outside MN3207 clock envelope");

    // Transcribed mixer ratios (ref doc §12; see OUTPUT CONVENTION):
    //   direct dry path, net flat: kDryGain = (R9||R10)/R71
    //     = 24812.03/33000 = 0.75188  (pre-emphasis k0 x mixer R75/R71;
    //     the shelf pair cancels exactly at matched prewarp)
    //   wet summing weight per side: R75/R82 = R76/R85 = 47k/47k = 1.0 —
    //     "unity wet summing" confirmed from the transcription. Already
    //     applied INSIDE processSample (it is the mixer's kGainWet);
    //     exposed for documentation/composition only.
    static constexpr float kDryGain =
        (Dc2Constants::kR10 * Dc2Constants::kR9
         / (Dc2Constants::kR10 + Dc2Constants::kR9)) / Dc2Constants::kR71;
    static constexpr float kWetGainL = Dc2OutputMixer::kGainWet;  // 1.0
    static constexpr float kWetGainR = Dc2OutputMixer::kGainWet;  // 1.0

    void init(float sampleRate) noexcept {
        inputBuf_.init(sampleRate);
        preEmph_.init(sampleRate);
        comp_.init(sampleRate);
        aaLpf_.init(sampleRate);

        bbdC_.init(sampleRate);
        bbdB_.init(sampleRate);
        // Real NE570 compander pair is modeled EXPLICITLY outside the BBD
        // (Ne570Compressor/Ne570Expander above) — DMM precedent; do not
        // double-compand.
        bbdC_.setCompanderAmount(0.0f);
        bbdB_.setCompanderAmount(0.0f);
        bbdC_.setClockNoiseLevel(0.002f);   // [EAR-MATCHED]
        bbdB_.setClockNoiseLevel(0.002f);
        // BBDLine's fixed internal 8 kHz reconstruction one-pole coexists
        // with the explicit Dc2ReconLpf (17.9 kHz cubic): both are kept —
        // project rule preserves fixed BBD/filter interaction cutoffs for
        // aliasing character.

        reconC_.init(sampleRate);
        reconB_.init(sampleRate);
        expC_.init(sampleRate);
        expB_.init(sampleRate);
        dryLf_.init(sampleRate);
        mixL_.init(sampleRate);
        mixR_.init(sampleRate);

        // One master oscillator, two antiphase triangle taps (ref doc §8:
        // IC6A inverts CV_C relative to CV_B == 0.5-cycle tap offset).
        // Drift/jitter stay at AnalogLfo defaults [EAR-MATCHED: the
        // hardware LFO is a real oscillator; keep the drift engine on].
        lfo_.init(sampleRate);
        lfo_.setActiveTaps(2);
        lfo_.setShapes(TriPhaseLfo<2>::Shape::Triangle,
                       TriPhaseLfo<2>::Shape::Triangle);
        lfo_.setTapBlend(0, 0.0f);
        lfo_.setTapBlend(1, 0.0f);
        lfo_.setTapPhase(0, 0.0f);    // channel C (OUT_L)
        lfo_.setTapPhase(1, 0.5f);    // channel B (OUT_R), antiphase CV

        prevTotalL_ = 0.0f;
        prevTotalR_ = 0.0f;
        delayMsC_ = kCenterDelayMs;
        delayMsB_ = kCenterDelayMs;
        setMode(0);
    }

    // [CONTROL-RATE] Applies the derived per-mode (rate, depth) pair.
    // Hardware mode buttons switch the LFO resistor networks abruptly;
    // no smoothing is added (mode changes are rare, and the depth change
    // reaches the delay through the triangle's continuous motion).
    void setMode(int mode) noexcept {
        if (mode < 0) mode = 0;
        if (mode > 3) mode = 3;
        mode_ = mode;
        lfo_.setFrequency(kModeTable[static_cast<size_t>(mode)].rateHz);
        depthMs_ = kModeTable[static_cast<size_t>(mode)].depthMs;
    }

    [[nodiscard]] int getMode() const noexcept { return mode_; }

    // Wet-only stereo outputs; caller composes in*kDryGain + wet.
    void processSample(float in, float& wetL, float& wetR) noexcept {
        // --- Modulation: one tick, two antiphase taps -> delay CVs ---
        lfo_.tick();
        const float dC = kCenterDelayMs + depthMs_ * lfo_.tap(0);
        const float dB = kCenterDelayMs + depthMs_ * lfo_.tap(1);
        delayMsC_ = dC;
        delayMsB_ = dB;
        bbdC_.setDelayMs(dC);
        bbdB_.setDelayMs(dB);

        // --- Shared front end: buffer -> pre-emphasis (dry rail) ---
        const float dry = preEmph_.process(inputBuf_.process(in));

        // --- Shared compressor (+ zener clamp) and anti-alias LPF ---
        const float aa = aaLpf_.process(comp_.process(dry));

        // --- Per-channel BBD -> reconstruction -> expander ---
        const float wetC = expC_.process(reconC_.process(bbdC_.process(aa)));
        const float wetB = expB_.process(reconB_.process(bbdB_.process(aa)));

        // --- Dry low-frequency reinforcement (Q2, feeds both mixers) ---
        const float lf = dryLf_.process(dry);

        // --- Output mixers: dry-rail input factored out as kDryGain;
        //     cross-feedback takes the opposite TOTAL output, previous
        //     sample (one-sample loop delay) ---
        const float outL = mixL_.process(0.0f, lf, wetC, prevTotalR_);
        const float outR = mixR_.process(0.0f, lf, wetB, prevTotalL_);
        prevTotalL_ = outL + kDryGain * in;
        prevTotalR_ = outR + kDryGain * in;

        wetL = outL;
        wetR = outR;
    }

    void reset() noexcept {
        inputBuf_.reset();
        preEmph_.reset();
        comp_.reset();
        aaLpf_.reset();
        bbdC_.reset();
        bbdB_.reset();
        reconC_.reset();
        reconB_.reset();
        expC_.reset();
        expB_.reset();
        dryLf_.reset();
        mixL_.reset();
        mixR_.reset();
        lfo_.reset();
        prevTotalL_ = 0.0f;
        prevTotalR_ = 0.0f;
        delayMsC_ = kCenterDelayMs;
        delayMsB_ = kCenterDelayMs;
    }

    // Instantaneous delay CVs (ms) as of the last processSample() —
    // observability for the motionless-CV self-test.
    [[nodiscard]] float getDelayMsL() const noexcept { return delayMsC_; }
    [[nodiscard]] float getDelayMsR() const noexcept { return delayMsB_; }

private:
    // [DTCM] hot path: every member below is read/written per sample.
    Dc2InputBuffer   inputBuf_;
    Dc2PreEmphasis   preEmph_;
    Ne570Compressor  comp_;       // IC2 half 1 (shared)
    Dc2AntiAliasLpf  aaLpf_;      // Q5 (shared)
    BBDLine<1024>    bbdC_;       // IC9 MN3207 (channel C -> OUT_L)
    BBDLine<1024>    bbdB_;       // IC3 MN3207 (channel B -> OUT_R)
    Dc2ReconLpf      reconC_;     // Q10
    Dc2ReconLpf      reconB_;     // Q4
    Ne570Expander    expC_;       // IC11 half 2
    Ne570Expander    expB_;       // IC11 half 1
    Dc2DryLowpass    dryLf_;      // Q2
    Dc2OutputMixer   mixL_;       // IC12A
    Dc2OutputMixer   mixR_;       // IC12B
    TriPhaseLfo<2>   lfo_;        // IC7 TL022 (+ IC5B/IC6A CV scaling)

    // [DTCM] cross-feedback loop state (previous-sample TOTAL outputs)
    float prevTotalL_ = 0.0f;
    float prevTotalR_ = 0.0f;

    // [DTCM] last delay CVs (ms) — written per sample, read by tests
    float delayMsC_ = kCenterDelayMs;
    float delayMsB_ = kCenterDelayMs;

    // [CONTROL-RATE] mode state, written only by setMode()
    int   mode_    = 0;
    float depthMs_ = kModeTable[0].depthMs;
};

// ---------------------------------------------------------------------------
// Self-test (session 3 verification steps). Build via the compile gate at
// the top of this file with -DDC2_CIRCUIT_SELFTEST (tests/dc2_circuit_test.cpp).
// ---------------------------------------------------------------------------
#ifdef DC2_CIRCUIT_SELFTEST
#include <cstdio>

namespace dc2_circuit_selftest {

inline int g_passed = 0, g_failed = 0;

inline void verdict(bool ok, const char* what) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    (ok ? g_passed : g_failed)++;
}

// 1) Compander identity: sine bursts through compressor -> expander
//    (no BBD); level match within 0.5 dB at every level.
inline void testCompanderIdentity() {
    constexpr float kFs = 48000.0f;
    constexpr int   kPrime = 24000;   // 0.5 s settle (release 3.17 ms)
    constexpr int   kMeas  = 24000;
    const float levelsDb[6] = {-40.0f, -30.0f, -20.0f, -12.0f, -6.0f, 0.0f};

    std::printf("Compander identity (1 kHz sine, gate 0.5 dB):\n");
    std::printf("  %8s %12s %12s %10s\n", "in dBFS", "in RMS", "out RMS", "err dB");
    bool allOk = true;
    float worst = 0.0f;
    for (float db : levelsDb) {
        Ne570Compressor comp;  comp.init(kFs);
        Ne570Expander   expd;  expd.init(kFs);
        const float amp = std::pow(10.0f, db / 20.0f);
        const float w = 6.2831853f * 1000.0f / kFs;
        for (int i = 0; i < kPrime; ++i)
            (void)expd.process(comp.process(amp * std::sin(w * static_cast<float>(i))));
        float si = 0.0f, so = 0.0f;
        for (int i = 0; i < kMeas; ++i) {
            const float x = amp * std::sin(w * static_cast<float>(kPrime + i));
            const float y = expd.process(comp.process(x));
            si += x * x;
            so += y * y;
        }
        const float inRms  = std::sqrt(si / static_cast<float>(kMeas));
        const float outRms = std::sqrt(so / static_cast<float>(kMeas));
        const float errDb  = 20.0f * std::log10(outRms / inRms);
        std::printf("  %8.1f %12.6f %12.6f %+10.3f\n", db, inRms, outRms, errDb);
        if (std::fabs(errDb) > worst) worst = std::fabs(errDb);
        if (std::fabs(errDb) > 0.5f) allOk = false;
    }
    std::printf("  worst |err| = %.3f dB\n", worst);
    verdict(allOk, "compander identity within 0.5 dB, -40..0 dBFS");
}

// 2) Motionless CV: cvA + cvB constant within 1e-4 of its mean over 5 s.
inline void testMotionlessCv() {
    constexpr float kFs = 48000.0f;
    constexpr int   kN  = 5 * 48000;
    static Dc2Circuit c;
    c.init(kFs);
    c.setMode(1);   // deepest modulation (MODE2)
    float wetL = 0.0f, wetR = 0.0f;
    double acc = 0.0;
    float minS = 1e9f, maxS = -1e9f;
    for (int i = 0; i < kN; ++i) {
        c.processSample(0.0f, wetL, wetR);
        const float s = c.getDelayMsL() + c.getDelayMsR();
        acc += static_cast<double>(s);
        if (s < minS) minS = s;
        if (s > maxS) maxS = s;
    }
    const float mean = static_cast<float>(acc / static_cast<double>(kN));
    const float dev  = std::fmax(maxS - mean, mean - minS);
    std::printf("Motionless CV (5 s, MODE2): sum mean %.6f ms, max dev %.3g ms\n",
                mean, static_cast<double>(dev));
    verdict(dev <= 1e-4f, "cvA + cvB constant within 1e-4 of mean");
}

// 3) Mode table print + ordering assertion (also enforced at compile time).
inline void testModeTable() {
    std::printf("Mode table (derived):\n");
    for (int m = 0; m < 4; ++m) {
        const auto& p = Dc2Circuit::kModeTable[static_cast<size_t>(m)];
        std::printf("  MODE%d: rate %.3f Hz  depth %.2f ms  (delay %.2f..%.2f ms)\n",
                    m + 1, static_cast<double>(p.rateHz), static_cast<double>(p.depthMs),
                    static_cast<double>(Dc2Circuit::kCenterDelayMs - p.depthMs),
                    static_cast<double>(Dc2Circuit::kCenterDelayMs + p.depthMs));
    }
    const auto& t = Dc2Circuit::kModeTable;
    verdict(t[0].rateHz < t[2].rateHz && t[0].rateHz < t[3].rateHz &&
            t[1].rateHz < t[2].rateHz && t[1].rateHz < t[3].rateHz,
            "modes 1,2 slower than modes 3,4");
}

// 4) Audio sanity: 2 s of 1 kHz at -12 dBFS per mode; finite outputs,
//    wet RMS in -30..0 dBFS, mono sum shows no gross cancellation.
inline void testAudioSanity() {
    constexpr float kFs   = 48000.0f;
    constexpr int   kWarm = 24000;            // 0.5 s
    constexpr int   kMeas = 72000;            // 1.5 s
    constexpr float kAmp  = 0.2512f;          // -12 dBFS
    const float w = 6.2831853f * 1000.0f / kFs;

    static Dc2Circuit c;
    for (int m = 0; m < 4; ++m) {
        c.init(kFs);
        c.setMode(m);
        float wetL = 0.0f, wetR = 0.0f;
        bool finite = true;
        float sl = 0.0f, sr = 0.0f, ss = 0.0f;
        for (int i = 0; i < kWarm + kMeas; ++i) {
            const float x = kAmp * std::sin(w * static_cast<float>(i));
            c.processSample(x, wetL, wetR);
            if (__builtin_expect(!std::isfinite(wetL) || !std::isfinite(wetR), 0)) {
                finite = false;
                break;
            }
            if (i >= kWarm) {
                sl += wetL * wetL;
                sr += wetR * wetR;
                const float s = wetL + wetR;
                ss += s * s;
            }
        }
        const float rmsL = std::sqrt(sl / static_cast<float>(kMeas));
        const float rmsR = std::sqrt(sr / static_cast<float>(kMeas));
        const float rmsS = std::sqrt(ss / static_cast<float>(kMeas));
        const float dbL = 20.0f * std::log10(rmsL + 1e-12f);
        const float dbR = 20.0f * std::log10(rmsR + 1e-12f);
        const float avg = 0.5f * (rmsL + rmsR);
        std::printf("MODE%d: wetL %.1f dBFS  wetR %.1f dBFS  monoSum/avg %.2f\n",
                    m + 1, static_cast<double>(dbL), static_cast<double>(dbR),
                    static_cast<double>(rmsS / avg));
        char label[64];
        std::snprintf(label, sizeof label, "MODE%d finite, RMS in band, in-phase", m + 1);
        verdict(finite &&
                dbL >= -30.0f && dbL <= 0.0f &&
                dbR >= -30.0f && dbR <= 0.0f &&
                rmsS >= 0.7f * avg,
                label);
    }
}

}  // namespace dc2_circuit_selftest

int main() {
    using namespace dc2_circuit_selftest;
    testCompanderIdentity();
    testMotionlessCv();
    testModeTable();
    testAudioSanity();
    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
#endif  // DC2_CIRCUIT_SELFTEST
