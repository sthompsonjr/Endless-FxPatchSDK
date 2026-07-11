#pragma once

// Dytronics/Songbird Tri-Stereo Chorus (CS-5 class) — voice assembly
// (Tridimension Session 4 of 6).
//
// WHY THIS FILE IS MOSTLY DSP, NOT WDF: no public schematic exists for the
// Tri-Stereo Chorus (unlike the DC-2, which has the Aion FX Blueshift
// transcription). The BBD cores are clocked discrete-time systems and are
// modeled behaviorally by dsp/BBDLine.h per project convention (DMM and DC-2
// precedents). This session is therefore BEHAVIOR-FIRST: the only honestly
// scopeable analog network is the input coupling/pad (series C into shunt R,
// built from wdf/ one-ports below); the filters are biquads with H(s)
// derivation comments (Session 1 convention); everything else is DSP by
// design. Every numeric constant carries a provenance tag:
//   [DATASHEET]   — a published figure (UAD/Softube DTSC manual,
//                   reference/Dytronics_TriStereo_Chorus_Manual.pdf)
//   [JH-DERIVED]  — structural analog from the Jurgen Haible Triple Chorus
//                   (reference/JH__Triple_Chorus.pdf): two 3-phase modulation
//                   generators (slow "Chorus" + fast "Vibrato"), 0/120/240 deg
//                   outputs summed per BBD clock VCO; 2nd-order 12 kHz input
//                   anti-alias precedent (Solina lineage, JH p.3)
//   [EAR-MATCHED] — starting point to be tuned against the Italo De Angelis
//                   reference recordings (Session 6)
//
// SIGNAL CHAIN (mono in -> stereo wet out):
//   in -> TscInputCoupling (WDF: series C 100n -> shunt R 50k, [JH-DERIVED])
//      -> input anti-alias LPF, 2nd order @ 12 kHz [JH-DERIVED]
//      -> split to three BBD lines (0=L, 1=C, 2=R), BBDLine<1024> each:
//           delay_i = kCenterDelayMs + cv_i   (cv derivation below)
//      -> per-line reconstruction LPF, 2nd order @ 10 kHz [JH-DERIVED class
//         value; EAR-MATCH tune flag]
//   Output mix (LOCKED DECISION, community-documented BBD1+BBD2 left /
//   BBD2+BBD3 right scheme — center voice split equally):
//     wetL = v0 + 0.5*v1        wetR = v2 + 0.5*v1
//   Caller composes: out = in * kDryGain + wet * kWetGain (Session 5 wiring;
//   same wet-only output convention as wdf/Dc2Circuit.h).
//
// MODULATION (JH Triple Chorus structure, TriPhaseLfo realization):
//   Two independent TriPhaseLfo<3> generators, each with taps at 0, 1/3, 2/3
//   (0/120/240 deg):
//     chorusGen  — shape endpoints Sine/Triangle, per-line blend m_i,
//                  rate = setRate() (manual rate range 0.03..7.45 Hz
//                  [DATASHEET — UAD manual figure for the modeled CS-5
//                  behavior, manual p.7/p.8])
//     vibratoGen — Sine/Sine, fixed 5.5 Hz [EAR-MATCHED start; the DTSC
//                  Preset mode is "a slowly sweeping sine LFO with a swifter
//                  added sine vibrato part", manual p.7]
//   Per-line clock CV (ms), evaluated per sample:
//     cv_i = depth_i * blend(sine, triangle, m_i)|chorus tap i
//          + kVibDepthMs * vibrato tap i
//   INTENSITY MAPPING [EAR-MATCHED]:
//     depth_i = intensity_i * kMaxDepthMs;   m_i = intensity_i
//   Rationale: the hardware intensity knobs blend LFO waveforms per line,
//   producing morphing non-sine sweeps (Italo De Angelis' description of the
//   unit). Mapping the waveform morph m_i to the SAME knob as the depth
//   reproduces the "more intensity = stranger waveform" behavior with one
//   control per line.
//   Both generators keep the AnalogLfo-default drift/jitter engine
//   (TriPhaseLfo init defaults): the two generators drift INDEPENDENTLY,
//   matching two physical oscillator cores on the board, while each
//   generator's three taps stay 120 deg apart by construction (single master
//   accumulator — see dsp/TriPhaseLfo.h header).
//
// DELAY-LINE REGION [EAR-MATCHED; anchor: BBD chorus norms and the
// MN30xx-class parts documented in the unit]: center delay 8.0 ms, max
// chorus swing +/-4.0 ms at full intensity (+/-0.15 ms vibrato on top).
// BBDLine<1024> gives (1024-2)/48000 = 21.29 ms max at 48 kHz — ample
// headroom (static-asserted below).
//
// BBD warmth: setCompanderAmount(0.15f) on each line [EAR-MATCHED]. The
// TSC's internal noise-reduction scheme is undocumented, so BBDLine's
// generic soft-clip compander stands in at low amount — this is an explicit
// modeling ASSUMPTION, not a transcription. Clock noise 0.003 [EAR-MATCHED].
// BBDLine's fixed internal 8 kHz reconstruction one-pole coexists with the
// explicit 10 kHz recon biquad — both kept, per the project rule preserving
// fixed BBD/filter interaction cutoffs for aliasing character (DC-2
// precedent, wdf/Dc2Circuit.h).
//
// CYCLE BUDGET (528 MHz / 48 kHz = 11,000 cycles/sample; whole-patch
// WARN > 4,400, FAIL > 6,600):
//   TriPhaseLfo<3> tick x2      ~600-1000  (~13 sinf-class shape/drift evals)
//   BBDLine<1024> x3            ~450-750   (Hermite read + clock noise + LP)
//   recon biquad x3             ~90-180
//   input AA biquad             ~30-60
//   TscInputCoupling (WDF)      ~40-70
//   smoothers + tap blends x3   ~60-100
//   setDelayMs x3               ~30-60
//   output mix                  ~10
//   ------------------------------------------------
//   TOTAL ESTIMATED             ~1,310-2,230 cycles/sample (12-20% of budget)
//   Under WARN; composes with Dc2Circuit (~980-1,765) inside the Session 5
//   patch with headroom.
// The processSample body is branch-free (all branches live inside the
// components, which carry their own __builtin_expect hints — see
// TriPhaseLfo::tick); no per-sample transcendentals are added here, so
// WDF_USE_FAST_MATH is not needed in this file.
//
// Desktop compile gate (single command line, wrapped for readability):
//   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra
//       -DENDLESS_DESKTOP_BUILD -I. -I./sdk -I./wdf -I./dsp
//       tests/tsc_circuit_test.cpp -lm -o tsc_circuit_test

#include "wdf/WdfOnePort.h"
#include "dsp/TriPhaseLfo.h"
#include "dsp/BBDLine.h"
#include "dsp/BiquadFilter.h"
#include "dsp/ParameterSmoother.h"

#include <array>
#include <cmath>

// ---------------------------------------------------------------------------
// TscInputCoupling — WDF input coupling and pad network.
//
// [JH-DERIVED] The JH Triple Chorus input stage is AC-coupled ("The audio
// signals are centered to 0V ... due to AC coupling of inputs and outputs",
// JH p.4) with a recommended input level potentiometer of "10k log ... 50k
// log" (JH p.4, Bill of Materials note). Modeled as:
//
//   Vin --(Rs 1k)--- C 100n ---+--- out
//                              |
//                             R 50k
//                              |
//                             GND
//
//   Rs   = 1 kOhm  [JH-DERIVED: line-level driving-stage output impedance;
//                   same convention as RCLowpassCircuit / DiodeClipperCircuit
//                   in wdf/WdfCircuits.h]
//   C    = 100 nF  [JH-DERIVED class value: standard film coupling value in
//                   the JH design lineage; chosen so the corner lands sub-bass]
//   R    = 50 kOhm [JH-DERIVED: upper of the JH-recommended 10k..50k input
//                   level pot, wiper at maximum = full shunt to ground]
//
// H(s) derivation (voltage divider, output across R):
//   H(s) = R / (Rs + R + 1/(s*C)) = k * s*tau / (1 + s*tau)
//     tau = C*(Rs+R) = 100e-9 * 51e3 = 5.10 ms -> fc = 1/(2*pi*tau) = 31.21 Hz
//     k   = R/(Rs+R) = 50/51 = 0.98039  (pad: -0.17 dB)
//   First-order highpass: blocks DC/sub-bass, passband loss 0.17 dB.
//
// WDF structure: the circuit is one series loop {Thevenin source (Vs, Rs),
// C, R}, realized as a single CLOSED 3-port series junction (Fettweis 1986):
//   b_k = a_k - (2*R_k / sum_j R_j) * sum_j a_j     (a_j = element-reflected
//   waves into the junction, b_k = waves scattered back to the elements)
// with the per-element wave laws taken from wdf/WdfOnePort.h:
//   capacitor: Rp = 1/(2*C*fs), b[n] = a[n-1]   (WdfCapacitor law)
//   resistor:  Rp = R,          b    = 0        (WdfResistor law)
//   Thevenin source, ADAPTED (Rp = Rs): V = Vs + Rs*I_in =>
//     b = V - Rs*I_in = Vs  (reflection-FREE; derived from WdfPort.h's
//     a = V + Rp*I / b = V - Rp*I convention)
// The junction is written out flat on WdfPort variables instead of going
// through WdfSeriesAdaptor2 + element reflect() calls, for two verified
// reasons (both reproduce as NaN blowup in the self-test if ignored):
//   1. WdfResistiveVoltageSource::reflect() applies the IDEAL-source law
//      b = 2*Vs - a; the adapted Thevenin law is b = Vs. The spurious -a
//      term re-injects the incident wave with unit gain.
//   2. WdfCapacitor::reflect() emits b and consumes port.a in the same
//      call, so under the one-reflect-per-sample adaptor pattern the emitted
//      wave is a[n-2], not a[n-1] — an extra z^-1 in the loop.
//   In this single-loop root configuration the two effects make the wave
//   recursion unstable (companion-matrix eigenvalue |lambda| ~ 1.7). The
//   closed junction below is passive scattering + passive element laws,
//   hence unconditionally stable, and realizes the EXACT bilinear transform
//   of H(s) (no explicit prewarp needed at a 31 Hz corner; verified against
//   the analytic magnitude in self-test 0).
// ---------------------------------------------------------------------------
struct TscInputCoupling {
    static constexpr float kRsource = 1000.0f;   // [JH-DERIVED] see above
    static constexpr float kCin     = 100e-9f;   // [JH-DERIVED] see above
    static constexpr float kRshunt  = 50e3f;     // [JH-DERIVED] see above

    // [DTCM] hot path: one wave port per element + capacitor wave memory
    WdfPort srcPort_;   // Thevenin source (Rp = Rs)
    WdfPort capPort_;   // series coupling capacitor
    WdfPort resPort_;   // shunt resistor (output element)
    float   capState_ = 0.0f;

    // [CONTROL-RATE] junction scattering coefficients 2*Rk/sumR
    float kS_ = 0.0f, kC_ = 0.0f, kR_ = 0.0f;

    void init(float sampleRate) noexcept {
        srcPort_.Rp = kRsource;
        capPort_.Rp = 1.0f / (2.0f * kCin * sampleRate);   // WdfCapacitor law
        resPort_.Rp = kRshunt;
        const float sumR = srcPort_.Rp + capPort_.Rp + resPort_.Rp;
        kS_ = 2.0f * srcPort_.Rp / sumR;
        kC_ = 2.0f * capPort_.Rp / sumR;
        kR_ = 2.0f * resPort_.Rp / sumR;
        reset();
    }

    [[nodiscard]] float process(float in) noexcept {
        // Element reflections into the junction (laws cited above).
        srcPort_.b = in;          // adapted Thevenin source: b = Vs
        capPort_.b = capState_;   // capacitor: b[n] = a[n-1]
        resPort_.b = 0.0f;        // resistor: pure absorption
        // Closed series junction scatter.
        const float sum = srcPort_.b + capPort_.b + resPort_.b;
        srcPort_.a = srcPort_.b - kS_ * sum;
        capPort_.a = capPort_.b - kC_ * sum;
        resPort_.a = resPort_.b - kR_ * sum;
        capState_  = capPort_.a;  // capacitor consumes its incident wave
        // Output = voltage across the shunt R. The series-loop port
        // orientation sums all three port voltages to zero around the loop,
        // so the divider-referenced output is the NEGATIVE of the resistor's
        // loop-oriented port voltage (sign verified against the analog
        // step response: v_out(0+) = +R/(R+Rs)*Vs).
        return -resPort_.voltage();
    }

    void reset() noexcept {
        srcPort_.reset();
        capPort_.reset();
        resPort_.reset();
        capState_ = 0.0f;
    }
};

// ---------------------------------------------------------------------------
// TscCircuit — complete Tri-Stereo Chorus voice, mono in / stereo wet out.
// Interface shape mirrors wdf/Dc2Circuit.h exactly (init / processSample(in,
// wetL, wetR) / kDryGain / kWetGain / reset) so Session 5 composes both
// units symmetrically.
// ---------------------------------------------------------------------------
class TscCircuit {
public:
    // Output composition constants (Session 5 convention, Dc2Circuit shape):
    //   out = in * kDryGain + wet * kWetGain
    static constexpr float kDryGain = 1.0f;   // [EAR-MATCHED]
    static constexpr float kWetGain = 0.7f;   // [EAR-MATCHED starting point]

    // Manual LFO rate range [DATASHEET — UAD manual figure for the modeled
    // CS-5 behavior: "The rate of the manual LFO ranges from 0.03 Hz to
    // 7.45 Hz", manual p.7].
    static constexpr float kRateMinHz = 0.03f;   // [DATASHEET]
    static constexpr float kRateMaxHz = 7.45f;   // [DATASHEET]
    // Default rate: geometric mean of the [DATASHEET] range endpoints,
    // sqrt(0.03 * 7.45) = 0.473 Hz — a mid-travel knob on a log-taper pot.
    static constexpr float kDefaultRateHz = 0.473f;   // [DATASHEET-derived]

    // Delay-line region [EAR-MATCHED; anchor: BBD chorus norms and the
    // MN30xx-class parts documented in the unit — see file header].
    static constexpr float kCenterDelayMs = 8.0f;   // [EAR-MATCHED]
    static constexpr float kMaxDepthMs    = 4.0f;   // [EAR-MATCHED] full-intensity swing

    // Fixed vibrato generator [EAR-MATCHED start — DTSC Preset mode's
    // "swifter added sine vibrato part" (manual p.7); rate and amount are
    // static in hardware Preset mode].
    static constexpr float kVibRateHz  = 5.5f;    // [EAR-MATCHED start]
    static constexpr float kVibDepthMs = 0.15f;   // [EAR-MATCHED] ms equivalent

    // Center voice split (LOCKED DECISION): community-documented
    // BBD1+BBD2 left / BBD2+BBD3 right scheme — center split equally.
    static constexpr float kCenterSplit = 0.5f;

    // BBD warmth / noise [EAR-MATCHED — see file header ASSUMPTION note].
    static constexpr float kCompanderAmount = 0.15f;   // [EAR-MATCHED]
    static constexpr float kClockNoiseLevel = 0.003f;  // [EAR-MATCHED]

    // Intensity smoothing: the hardware controls are continuous pots; a
    // stepped digital intensity would step depth_i * tap(i) directly into
    // the delay CV (audible pitch click). 20 ms is the library's standard
    // pot-smoothing floor (dsp/ParameterSmoother.h, "typical 20-50 ms").
    static constexpr float kIntensitySmoothMs = 20.0f;   // [EAR-MATCHED]
    // Default intensity: mid-travel knobs [EAR-MATCHED default].
    static constexpr float kDefaultIntensity = 0.5f;     // [EAR-MATCHED]

    // Full modulation swing must fit BBDLine<1024> at 48 kHz:
    // Hermite-safe delay range is [1, 1022] samples = [0.021, 21.29] ms.
    // 1 ms bottom margin keeps the read pointer clear of the write head.
    static constexpr float kMaxSwingMs = kMaxDepthMs + kVibDepthMs;   // 4.15
    static_assert(kCenterDelayMs - kMaxSwingMs >= 1.0f,
                  "TSC delay swing under-runs the BBD line");
    static_assert(kCenterDelayMs + kMaxSwingMs <= 21.29f,
                  "TSC delay swing over-runs BBDLine<1024> at 48 kHz");

    void init(float sampleRate) noexcept {
        coupling_.init(sampleRate);

        // Input anti-alias LPF [JH-DERIVED]: "the signal ... runs thru a
        // second order 12kHz filter before it goes into the BBDs. This acts
        // as an anti-aliasing filter" (JH p.3, Solina lineage; JH's own
        // build adds a 16 kHz option — the 12 kHz Solina value is used).
        // H(s) = w0^2 / (s^2 + (w0/Q)*s + w0^2), w0 = 2*pi*12000.
        // Q = 0.7071 (Butterworth): JH specifies order and corner, not
        // alignment — maximally-flat assumed [EAR-MATCHED tune flag].
        // Discretized by dsp/BiquadFilter.h (RBJ bilinear, prewarped at f0).
        inputAa_.init(sampleRate);
        inputAa_.setParameters(BiquadFilter::Type::Lowpass,
                               kInputAaHz, kFilterQ);

        for (size_t i = 0; i < 3; ++i) {
            bbd_[i].init(sampleRate);
            bbd_[i].setCompanderAmount(kCompanderAmount);
            bbd_[i].setClockNoiseLevel(kClockNoiseLevel);

            // Reconstruction LPF [JH-DERIVED class value; EAR-MATCH tune
            // flag]: post-BBD reconstruction sits just below the input AA
            // corner in the JH/Solina lineage; 10 kHz, same H(s) form and
            // Butterworth alignment as the input AA above.
            recon_[i].init(sampleRate);
            recon_[i].setParameters(BiquadFilter::Type::Lowpass,
                                    kReconHz, kFilterQ);

            intensitySm_[i].init(sampleRate, kIntensitySmoothMs);
            intensitySm_[i].snapTo(kDefaultIntensity);

            chorusCvMs_[i] = 0.0f;
            vibCvMs_[i]    = 0.0f;
            delayMs_[i]    = kCenterDelayMs;
        }

        // Two independent 3-phase generators (JH structure). Drift/jitter
        // stay at TriPhaseLfo init defaults (== AnalogLfo defaults): the
        // generators are two separate physical oscillator cores and drift
        // independently of each other; within each generator the three taps
        // share one master accumulator, so their 120 deg spacing is
        // drift/jitter-invariant (dsp/TriPhaseLfo.h header).
        chorusGen_.init(sampleRate);
        chorusGen_.setShapes(TriPhaseLfo<3>::Shape::Sine,
                             TriPhaseLfo<3>::Shape::Triangle);
        chorusGen_.setActiveTaps(3);
        vibratoGen_.init(sampleRate);
        vibratoGen_.setShapes(TriPhaseLfo<3>::Shape::Sine,
                              TriPhaseLfo<3>::Shape::Sine);
        vibratoGen_.setActiveTaps(3);
        vibratoGen_.setFrequency(kVibRateHz);
        for (int i = 0; i < 3; ++i) {
            // Taps at 0, 1/3, 2/3 cycles = 0/120/240 deg on BOTH generators;
            // line i sums chorus tap i and vibrato tap i (JH: "BBD1 sees ...
            // Chorus 0 deg and Vibrato 0 deg", etc.).
            const float ph = static_cast<float>(i) / 3.0f;
            chorusGen_.setTapPhase(i, ph);
            vibratoGen_.setTapPhase(i, ph);
            chorusGen_.setTapBlend(i, kDefaultIntensity);
            vibratoGen_.setTapBlend(i, 0.0f);   // Sine/Sine: blend is a no-op
        }

        rateHz_ = kDefaultRateHz;
        chorusGen_.setFrequency(rateHz_);
    }

    // [CONTROL-RATE] Manual chorus rate, clamped to the [DATASHEET] range.
    // Applied directly (no smoothing): a rate step only changes the phase
    // increment — the master phase stays continuous, so the delay CV cannot
    // jump (same argument as Dc2Circuit::setMode).
    void setRate(float hz) noexcept {
        if (hz < kRateMinHz) hz = kRateMinHz;
        if (hz > kRateMaxHz) hz = kRateMaxHz;
        rateHz_ = hz;
        chorusGen_.setFrequency(hz);
    }

    // [CONTROL-RATE] Per-line intensity (0=L, 1=C, 2=R; clamped to [0,1]).
    // Sets the smoother target only; depth_i AND morph m_i are derived from
    // the smoothed value per sample (see processSample) so knob steps reach
    // the delay CV click-free. Out-of-range line indices are ignored
    // (TriPhaseLfo setter convention).
    void setIntensity(int line, float v) noexcept {
        if (line < 0 || line > 2) return;
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        intensitySm_[static_cast<size_t>(line)].setTarget(v);
    }

    // Wet-only stereo outputs; caller composes in*kDryGain + wet*kWetGain.
    void processSample(float in, float& wetL, float& wetR) noexcept {
        // --- Modulation: smoothed intensity -> morph, then one tick each ---
        float m0 = intensitySm_[0].process();
        float m1 = intensitySm_[1].process();
        float m2 = intensitySm_[2].process();
        chorusGen_.setTapBlend(0, m0);
        chorusGen_.setTapBlend(1, m1);
        chorusGen_.setTapBlend(2, m2);
        chorusGen_.tick();
        vibratoGen_.tick();

        // Per-line clock CV: cv_i = depth_i * chorus tap i + vib * tap i,
        // depth_i = intensity_i * kMaxDepthMs (mapping rationale in header).
        const float depth0 = m0 * kMaxDepthMs;
        const float depth1 = m1 * kMaxDepthMs;
        const float depth2 = m2 * kMaxDepthMs;
        chorusCvMs_[0] = depth0 * chorusGen_.tap(0);
        chorusCvMs_[1] = depth1 * chorusGen_.tap(1);
        chorusCvMs_[2] = depth2 * chorusGen_.tap(2);
        vibCvMs_[0] = kVibDepthMs * vibratoGen_.tap(0);
        vibCvMs_[1] = kVibDepthMs * vibratoGen_.tap(1);
        vibCvMs_[2] = kVibDepthMs * vibratoGen_.tap(2);
        for (size_t i = 0; i < 3; ++i) {
            delayMs_[i] = kCenterDelayMs + chorusCvMs_[i] + vibCvMs_[i];
            bbd_[i].setDelayMs(delayMs_[i]);
        }

        // --- Audio path: coupling -> AA -> 3x (BBD -> recon) ---
        const float aa = inputAa_.process(coupling_.process(in));
        const float v0 = recon_[0].process(bbd_[0].process(aa));   // L line
        const float v1 = recon_[1].process(bbd_[1].process(aa));   // C line
        const float v2 = recon_[2].process(bbd_[2].process(aa));   // R line

        // --- Output mix: center voice split equally (locked decision) ---
        wetL = v0 + kCenterSplit * v1;
        wetR = v2 + kCenterSplit * v1;
    }

    void reset() noexcept {
        coupling_.reset();
        inputAa_.reset();
        for (size_t i = 0; i < 3; ++i) {
            bbd_[i].reset();
            recon_[i].reset();
            chorusCvMs_[i] = 0.0f;
            vibCvMs_[i]    = 0.0f;
            delayMs_[i]    = kCenterDelayMs;
        }
        chorusGen_.reset();
        vibratoGen_.reset();
        // Intensity smoothers are control state, not audio state — they keep
        // their current value/target across reset (Dc2Circuit keeps mode).
    }

    // [CONTROL-RATE] observability -------------------------------------------
    [[nodiscard]] float getRateHz() const noexcept { return rateHz_; }

    // Instantaneous per-line values as of the last processSample() —
    // observability for the self-tests (Dc2Circuit::getDelayMsL/R precedent).
    // The 120 deg spacing invariant is a PER-GENERATOR property (the total
    // CV sums two incommensurate 3-phase systems), so the generator
    // components are exposed separately. Index is the line (0=L, 1=C, 2=R),
    // NOT bounds-checked (test/diagnostic use).
    [[nodiscard]] float getDelayMs(int line) const noexcept {
        return delayMs_[static_cast<size_t>(line)];
    }
    [[nodiscard]] float getChorusCvMs(int line) const noexcept {
        return chorusCvMs_[static_cast<size_t>(line)];
    }
    [[nodiscard]] float getVibratoCvMs(int line) const noexcept {
        return vibCvMs_[static_cast<size_t>(line)];
    }

private:
    // Filter corners (derivations at the init() call sites above).
    static constexpr float kInputAaHz = 12000.0f;   // [JH-DERIVED]
    static constexpr float kReconHz   = 10000.0f;   // [JH-DERIVED class value; tune flag]
    static constexpr float kFilterQ   = 0.7071f;    // Butterworth [EAR-MATCHED tune flag]

    // [DTCM] hot path: every member below is read/written per sample.
    TscInputCoupling               coupling_;
    BiquadFilter                   inputAa_;
    std::array<BBDLine<1024>, 3>   bbd_;      // 0=L, 1=C, 2=R
    std::array<BiquadFilter, 3>    recon_;
    TriPhaseLfo<3>                 chorusGen_;
    TriPhaseLfo<3>                 vibratoGen_;
    std::array<ParameterSmoother, 3> intensitySm_;

    // [DTCM] last per-line CVs / delays (ms) — written per sample, read by tests
    std::array<float, 3> chorusCvMs_{};
    std::array<float, 3> vibCvMs_{};
    std::array<float, 3> delayMs_{kCenterDelayMs, kCenterDelayMs, kCenterDelayMs};

    // [CONTROL-RATE] rate state, written only by setRate()
    float rateHz_ = kDefaultRateHz;
};

// ---------------------------------------------------------------------------
// Self-test (Session 4 verification steps). Build via the compile gate at
// the top of this file with -DTSC_CIRCUIT_SELFTEST
// (tests/tsc_circuit_test.cpp).
// ---------------------------------------------------------------------------
#ifdef TSC_CIRCUIT_SELFTEST
#include <cstdio>

namespace tsc_circuit_selftest {

constexpr float kFs = 48000.0f;
constexpr int   kFsI = 48000;

inline int g_passed = 0, g_failed = 0;

inline void verdict(bool ok, const char* what) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    (ok ? g_passed : g_failed)++;
}

// 0) Input coupling vs analytic H(s) = k*s*tau/(1+s*tau) at 100/1k/8k Hz,
//    0.5 dB gate (Dc2Filters self-test convention). Verifies the inlined
//    WDF series-junction + linear-root equations empirically.
inline void testCouplingResponse() {
    std::printf("Input coupling vs analytic |H| (gate 0.5 dB):\n");
    constexpr float tau = TscInputCoupling::kCin
                        * (TscInputCoupling::kRsource + TscInputCoupling::kRshunt);
    constexpr float k   = TscInputCoupling::kRshunt
                        / (TscInputCoupling::kRsource + TscInputCoupling::kRshunt);
    const float freqs[3] = {100.0f, 1000.0f, 8000.0f};
    bool allOk = true;
    for (float f : freqs) {
        TscInputCoupling c;
        c.init(kFs);
        const float w = 6.2831853f * f / kFs;
        constexpr int kWarm = 9600, kMeas = 9600;
        for (int i = 0; i < kWarm; ++i)
            (void)c.process(0.1f * std::sin(w * static_cast<float>(i)));
        float si = 0.0f, so = 0.0f;
        for (int i = 0; i < kMeas; ++i) {
            const float x = 0.1f * std::sin(w * static_cast<float>(kWarm + i));
            const float y = c.process(x);
            si += x * x;
            so += y * y;
        }
        const float got = std::sqrt(so / si);
        const float wt  = 6.2831853f * f * tau;
        const float expect = k * wt / std::sqrt(1.0f + wt * wt);
        const float errDb = 20.0f * std::log10(got / expect);
        std::printf("  %7.0f Hz: got %.5f  expect %.5f  err %+.3f dB\n",
                    static_cast<double>(f), static_cast<double>(got),
                    static_cast<double>(expect), static_cast<double>(errDb));
        // NaN-robust: NaN must FAIL, not slip past a > comparison.
        if (!(std::fabs(errDb) <= 0.5f)) allOk = false;
    }
    verdict(allOk, "coupling matches analytic HPF within 0.5 dB");
}

// Shared spacing estimator (Session 2 test_TriPhaseLfo.cpp Test A pattern):
// merged rising-zero-crossing times of three 120-deg-spaced signals; each
// merged gap over the CENTERED three-gap period estimate must be 1/3
// (the centered denominator cancels first-order drift slope; see Session 2).
struct SpacingResult {
    int   numFracs   = 0;
    float maxDev     = 0.0f;   // max |frac - 1/3|
    float peakToPeak = 0.0f;   // spacing walk over the run
};

template <typename ReadTap>
SpacingResult measureSpacing(TscCircuit& c, ReadTap&& read,
                             int numSamples, int maxEvents,
                             float* evtBuf) {
    float wetL = 0.0f, wetR = 0.0f;
    int numEvents = 0;
    float prev[3];
    c.processSample(0.0f, wetL, wetR);
    for (int t = 0; t < 3; ++t) prev[t] = read(t);
    for (int n = 1; n < numSamples; ++n) {
        c.processSample(0.0f, wetL, wetR);
        for (int t = 0; t < 3; ++t) {
            const float cur = read(t);
            if (prev[t] < 0.0f && cur >= 0.0f && numEvents < maxEvents) {
                const float frac = prev[t] / (prev[t] - cur);
                evtBuf[numEvents++] = static_cast<float>(n - 1) + frac;
            }
            prev[t] = cur;
        }
    }
    SpacingResult r;
    if (numEvents < 12) return r;
    float minFrac = 1.0f, maxFrac = 0.0f;
    for (int kk = 1; kk + 2 < numEvents; ++kk) {
        const float gPrev = evtBuf[kk]     - evtBuf[kk - 1];
        const float g     = evtBuf[kk + 1] - evtBuf[kk];
        const float gNext = evtBuf[kk + 2] - evtBuf[kk + 1];
        const float fr    = g / (gPrev + g + gNext);
        const float dev   = std::fabs(fr - 1.0f / 3.0f);
        if (dev > r.maxDev) r.maxDev = dev;
        if (fr < minFrac)   minFrac = fr;
        if (fr > maxFrac)   maxFrac = fr;
        ++r.numFracs;
    }
    r.peakToPeak = maxFrac - minFrac;
    return r;
}

// 1) CV spacing at defaults, 60 s: pairwise spacing = 120 deg within 2 deg,
//    stable over the run (drift is common-mode within each generator).
//    The spacing invariant is PER GENERATOR: the total delay CV sums two
//    incommensurate 3-phase systems (chorus at 0.473 Hz, vibrato at 5.5 Hz),
//    and the vibrato term perturbs a total-CV crossing by up to
//    kVibDepthMs/(depth*2*pi) cycles ~ 2.1 deg even at full intensity — a
//    property of the (correct) sum, not of generator coherence. So the check
//    runs on each generator's CV component (exposed getters); the total
//    delay CVs are logged alongside. Tolerances: 2 deg = 0.005556 cycles
//    (P1), spacing walk < 1 deg = 0.002778 (P2, "stable over the run").
inline void testCvSpacing() {
    constexpr int   kDur       = 60 * kFsI;
    constexpr int   kMaxEvents = 128;
    constexpr float kTol120    = 2.0f / 360.0f;   // P1
    constexpr float kTolWalk   = 1.0f / 360.0f;   // P2
    static float evt[kMaxEvents];

    // Chorus generator, full 60 s at init() defaults (0.473 Hz -> ~85 events).
    static TscCircuit c;
    c.init(kFs);
    // Log the total delay CVs while measuring (spec: "logs the three CVs").
    float dMin[3] = {1e9f, 1e9f, 1e9f}, dMax[3] = {-1e9f, -1e9f, -1e9f};
    {
        // piggyback logging inside the read lambda's circuit stepping is not
        // possible (measureSpacing owns the loop), so log in a wrapper read.
        auto read = [&](int t) {
            const float d = c.getDelayMs(t);
            if (d < dMin[t]) dMin[t] = d;
            if (d > dMax[t]) dMax[t] = d;
            return c.getChorusCvMs(t);
        };
        const SpacingResult r =
            measureSpacing(c, read, kDur, kMaxEvents, evt);
        for (int t = 0; t < 3; ++t)
            std::printf("  delay CV line %d over 60 s: %.3f .. %.3f ms\n",
                        t, static_cast<double>(dMin[t]),
                        static_cast<double>(dMax[t]));
        std::printf("  chorus gen: %d fracs, max |off-1/3| = %.6f (tol %.6f), "
                    "walk = %.6f (tol %.6f)\n",
                    r.numFracs, static_cast<double>(r.maxDev),
                    static_cast<double>(kTol120),
                    static_cast<double>(r.peakToPeak),
                    static_cast<double>(kTolWalk));
        verdict(r.numFracs >= 12 && r.maxDev < kTol120,
                "chorus CV spacing 120 deg within 2 deg over 60 s");
        verdict(r.numFracs >= 12 && r.peakToPeak < kTolWalk,
                "chorus CV spacing stable (walk < 1 deg)");
    }

    // Vibrato generator, 60 s equivalent property at 5.5 Hz — 128 events
    // arrive in ~8 s; the run still covers many drift periods (0.13 Hz).
    c.init(kFs);
    {
        auto read = [&](int t) { return c.getVibratoCvMs(t); };
        const SpacingResult r =
            measureSpacing(c, read, 20 * kFsI, kMaxEvents, evt);
        std::printf("  vibrato gen: %d fracs, max |off-1/3| = %.6f, walk = %.6f\n",
                    r.numFracs, static_cast<double>(r.maxDev),
                    static_cast<double>(r.peakToPeak));
        verdict(r.numFracs >= 12 && r.maxDev < kTol120,
                "vibrato CV spacing 120 deg within 2 deg");
        verdict(r.numFracs >= 12 && r.peakToPeak < kTolWalk,
                "vibrato CV spacing stable (walk < 1 deg)");
    }
}

// 2) Intensity monotonicity: per line, sweep intensity 0 -> 1 in 10 steps;
//    measured delay swing (max - min of getDelayMs over 2 chorus cycles at
//    1 Hz, after a 0.5 s smoother settle) strictly increasing. The morph
//    m_i = intensity_i shrinks the blended waveform's peak near m = 0.5
//    (sine and triangle peak at different phases), but depth_i grows
//    linearly and dominates — measured, not assumed.
inline void testIntensityMonotonic() {
    static TscCircuit c;
    bool allOk = true;
    for (int line = 0; line < 3; ++line) {
        c.init(kFs);
        c.setRate(1.0f);
        for (int l = 0; l < 3; ++l) c.setIntensity(l, 0.0f);
        float prevSwing = -1.0f;
        bool lineOk = true;
        std::printf("  line %d swings (ms):", line);
        for (int step = 0; step <= 10; ++step) {
            c.setIntensity(line, static_cast<float>(step) * 0.1f);
            float wetL = 0.0f, wetR = 0.0f;
            for (int n = 0; n < kFsI / 2; ++n)          // settle 0.5 s
                c.processSample(0.0f, wetL, wetR);
            float mn = 1e9f, mx = -1e9f;
            for (int n = 0; n < 2 * kFsI; ++n) {        // 2 cycles @ 1 Hz
                c.processSample(0.0f, wetL, wetR);
                const float d = c.getDelayMs(line);
                if (d < mn) mn = d;
                if (d > mx) mx = d;
            }
            const float swing = mx - mn;
            std::printf(" %.3f", static_cast<double>(swing));
            if (swing <= prevSwing) lineOk = false;
            prevSwing = swing;
        }
        std::printf("\n");
        char label[64];
        std::snprintf(label, sizeof label,
                      "line %d delay swing strictly increasing", line);
        verdict(lineOk, label);
        allOk = allOk && lineOk;
    }
    (void)allOk;
}

// 3) Audio sanity.
//    a) 1 kHz sine, 2 s at defaults: no NaN/Inf; wetL and wetR both nonzero.
//    b) DC block: 1 s of +0.5 DC -> wet decays to the clock-noise floor
//       (verifies the WDF coupling network inside the full chain).
//    c) Center split symmetry: intensityC = 1, others 0 -> wetL equals wetR
//       within 0.1 dB. The residual L/R difference is the finite-time
//       average of the coherent v_side x v_center cross term (its long-run
//       limits are equal by tap symmetry: both side-center vibrato phase
//       differences are 120 deg). Averaging must span many chorus cycles for
//       the cross term to converge below 0.1 dB, so this check runs at
//       setRate(kRateMaxHz) (7.45 Hz -> ~149 cycles) for 20 s; the 2 s
//       defaults run in (a) covers the spec's headline duration.
inline void testAudioSanity() {
    const float w1k = 6.2831853f * 1000.0f / kFs;
    static TscCircuit c;

    // a) defaults, 2 s
    c.init(kFs);
    {
        bool finite = true;
        float sl = 0.0f, sr = 0.0f;
        float wetL = 0.0f, wetR = 0.0f;
        for (int n = 0; n < 2 * kFsI; ++n) {
            const float x = 0.25f * std::sin(w1k * static_cast<float>(n));
            c.processSample(x, wetL, wetR);
            if (__builtin_expect(!std::isfinite(wetL) || !std::isfinite(wetR), 0)) {
                finite = false;
                break;
            }
            sl += wetL * wetL;
            sr += wetR * wetR;
        }
        const float rmsL = std::sqrt(sl / static_cast<float>(2 * kFsI));
        const float rmsR = std::sqrt(sr / static_cast<float>(2 * kFsI));
        std::printf("  defaults: wetL RMS %.4f, wetR RMS %.4f\n",
                    static_cast<double>(rmsL), static_cast<double>(rmsR));
        verdict(finite, "1 kHz / 2 s: no NaN/Inf");
        verdict(rmsL > 1e-3f && rmsR > 1e-3f, "wetL and wetR both nonzero");
    }

    // b) DC block
    c.init(kFs);
    {
        float wetL = 0.0f, wetR = 0.0f;
        bool finite = true;
        for (int n = 0; n < kFsI; ++n) c.processSample(0.5f, wetL, wetR);
        float tailMax = 0.0f;
        for (int n = 0; n < kFsI / 10; ++n) {
            c.processSample(0.5f, wetL, wetR);
            const float m = std::fabs(wetL) + std::fabs(wetR);
            if (__builtin_expect(!std::isfinite(m), 0)) finite = false;
            if (m > tailMax) tailMax = m;
        }
        std::printf("  DC tail |wetL|+|wetR| max = %.5f "
                    "(clock-noise floor ~%.4f)\n",
                    static_cast<double>(tailMax),
                    static_cast<double>(3.0f * TscCircuit::kClockNoiseLevel));
        verdict(finite && tailMax < 0.02f, "DC input blocked by coupling network");
    }

    // c) center split symmetry
    c.init(kFs);
    c.setRate(TscCircuit::kRateMaxHz);
    c.setIntensity(0, 0.0f);
    c.setIntensity(1, 1.0f);
    c.setIntensity(2, 0.0f);
    {
        float wetL = 0.0f, wetR = 0.0f;
        for (int n = 0; n < kFsI / 2; ++n)              // settle smoothers
            c.processSample(0.0f, wetL, wetR);
        float sl = 0.0f, sr = 0.0f;
        constexpr int kMeas = 20 * kFsI;
        for (int n = 0; n < kMeas; ++n) {
            const float x = 0.25f * std::sin(w1k * static_cast<float>(n));
            c.processSample(x, wetL, wetR);
            sl += wetL * wetL;
            sr += wetR * wetR;
        }
        const float diffDb = 10.0f * std::log10(sl / sr);
        std::printf("  center-only: L/R power diff = %+.4f dB (gate 0.1)\n",
                    static_cast<double>(diffDb));
        verdict(std::fabs(diffDb) <= 0.1f,
                "intensityC=1: wetL equals wetR within 0.1 dB");
    }
}

// 4) Rate bounds: setRate(0.0f) and setRate(100.0f) clamp to the
//    [DATASHEET] range without misbehavior (finite output, delay CVs within
//    the static swing envelope).
inline void testRateBounds() {
    static TscCircuit c;
    const float probes[2]  = {0.0f, 100.0f};
    const float clamped[2] = {TscCircuit::kRateMinHz, TscCircuit::kRateMaxHz};
    for (int p = 0; p < 2; ++p) {
        c.init(kFs);
        c.setRate(probes[p]);
        const bool clampOk = c.getRateHz() == clamped[p];
        bool finite = true, inEnvelope = true;
        float wetL = 0.0f, wetR = 0.0f;
        const float w1k = 6.2831853f * 1000.0f / kFs;
        constexpr float lo = TscCircuit::kCenterDelayMs - TscCircuit::kMaxSwingMs - 1e-3f;
        constexpr float hi = TscCircuit::kCenterDelayMs + TscCircuit::kMaxSwingMs + 1e-3f;
        for (int n = 0; n < kFsI / 2; ++n) {
            const float x = 0.25f * std::sin(w1k * static_cast<float>(n));
            c.processSample(x, wetL, wetR);
            if (__builtin_expect(!std::isfinite(wetL) || !std::isfinite(wetR), 0))
                finite = false;
            for (int t = 0; t < 3; ++t) {
                const float d = c.getDelayMs(t);
                if (d < lo || d > hi) inEnvelope = false;
            }
        }
        char label[80];
        std::snprintf(label, sizeof label,
                      "setRate(%.1f) clamps to %.2f Hz, output sane",
                      static_cast<double>(probes[p]),
                      static_cast<double>(clamped[p]));
        verdict(clampOk && finite && inEnvelope, label);
    }
}

}  // namespace tsc_circuit_selftest

int main() {
    using namespace tsc_circuit_selftest;
    std::printf("=== TscCircuit self-test (Session 4) ===\n");
    testCouplingResponse();
    testCvSpacing();
    testIntensityMonotonic();
    testAudioSanity();
    testRateBounds();
    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
#endif  // TSC_CIRCUIT_SELFTEST
