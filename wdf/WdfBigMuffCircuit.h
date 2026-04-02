#pragma once

/// BigMuffCircuit — EHX Big Muff Pi, transistor-level WDF model.
///
/// Three NPN common-emitter stages:
///   Q1 (input boost) → Q2 (clipping) → Q3 (clipping) → tone → out
///
/// Each stage uses:
///   - WdfResistiveVoltageSource for base bias (Thevenin) and collector load
///   - WdfResistor for emitter degeneration
///   - WdfNpnBjt for the transistor (1D Newton-Raphson on Vbe)
///
/// Inter-stage AC coupling uses first-order HP filters (OnePoleFilter).
/// Anti-parallel diode clipping (1N914) is applied analytically between
/// stages using Lambert W, reproducing the Big Muff's symmetric soft-clip
/// character without the WDF parallel-adaptor instability that arises from
/// pairing two memoryless branches (no stable fixed point exists for that
/// topology — eigenvalue = 1 in the scatter/reflect map).
///
/// DC operating points are pre-computed and used to:
///   (a) Initialise WDF port.a values (eliminates Nyquist oscillation)
///   (b) Pre-warm inter-stage HP filters (eliminates DC-transient blow-up)

#include <cmath>
#include <algorithm>
#include "WdfPort.h"
#include "WdfOnePort.h"
#include "WdfAdaptors.h"
#include "WdfNonlinear.h"
#include "LambertW.h"
#include "dsp/OnePoleFilter.h"

// ─────────────────────────────────────────────────────────────────────────────
// Named constants
// ─────────────────────────────────────────────────────────────────────────────
namespace bigmuff {

    constexpr float Vsupply = 9.0f;

    // BC549C (Ram's Head / Triangle)
    namespace bc549c {
        constexpr float Is  = 2.04e-14f;
        constexpr float Vt  = 1.0f * 25.85e-3f;
        constexpr float hFE = 520.0f;
    }

    // 2N3904 (Civil War)
    namespace npn2n3904 {
        constexpr float Is  = 6.734e-15f;
        constexpr float Vt  = 1.259f * 25.85e-3f;
        constexpr float hFE = 416.0f;
    }

    // 2N5088 NPN — Central Semiconductor Corp.
    // Used in EHX Big Muff Pi NYC (Version 9, mid-1990s onward)
    // Source: Central Semiconductor LTspice model, rev. A, 2010-08-18
    //   IS=38.116E-15  BF=599.06  NF=1.0(default)  VAF=100
    // Simplified to (Is, Vt=NF*kT/q, hFE=BF) for WdfNpnBjt solver.
    namespace npn2n5088 {
        constexpr float Is  = 38.116e-15f;       // transport saturation current (IS)
        constexpr float Vt  = 1.0f * 25.85e-3f;  // NF=1.0 (default) × thermal voltage
        constexpr float hFE = 599.06f;            // forward beta (BF)
    }

    // 1N914 silicon diodes (inter-stage clipping)
    namespace diode_1n914 {
        constexpr float Is = 2.52e-9f;
        constexpr float Vt = 1.752f * 25.85e-3f;  // ≈ 0.04529 V
    }

    // Q1: input boost stage
    namespace q1 {
        constexpr float R_b1 = 1.2e6f;    // bias divider top, Ω
        constexpr float R_b2 = 100e3f;    // bias divider bot, Ω
        constexpr float R_c  = 47e3f;     // collector load, Ω
        constexpr float R_e  = 10e3f;     // emitter resistor, Ω
        constexpr float C_in = 100e-9f;   // input coupling cap, F
        // Thevenin bias
        constexpr float V_th = Vsupply * R_b2 / (R_b1 + R_b2); // ≈ 0.692 V
        constexpr float R_th = R_b1 * R_b2 / (R_b1 + R_b2);    // ≈ 92.3 kΩ
    }

    // Q2 / Q3: clipping stages (same component values)
    namespace q2 {
        constexpr float R_b_top  = 470e3f;
        constexpr float R_b_bot  = 56e3f;
        constexpr float R_c      = 8.2e3f;
        constexpr float R_e      = 470.0f;
        constexpr float C_couple = 100e-9f;
        constexpr float V_th = Vsupply * R_b_bot / (R_b_top + R_b_bot); // ≈ 0.958 V
        constexpr float R_th = R_b_top * R_b_bot / (R_b_top + R_b_bot); // ≈ 47.6 kΩ
    }
    namespace q3 = q2;

    // Tone stack corner frequencies
    namespace tone {
        constexpr float f_bass_hz   = 72.3f;
        constexpr float f_treble_hz = 1046.0f;
    }

    // Inter-stage AC coupling corner frequencies
    namespace coupling {
        constexpr float f_in_hz   = 1.0f / (6.283185f * q1::R_th * q1::C_in);     // ≈ 17 Hz
        constexpr float f_q1q2_hz = 1.0f / (6.283185f * q2::R_th * q2::C_couple); // ≈ 33 Hz
    }

    enum class Variant { RamsHead, CivilWar, Triangle, NYC };

} // namespace bigmuff

// ─────────────────────────────────────────────────────────────────────────────
// BigMuffCircuit
// ─────────────────────────────────────────────────────────────────────────────
class BigMuffCircuit {
public:
    BigMuffCircuit() = default;

    void init(float sampleRate,
              bigmuff::Variant variant = bigmuff::Variant::RamsHead) noexcept {
        sampleRate_ = sampleRate;
        variant_    = variant;
        sustain_    = 0.5f;
        toneBlend_  = 0.5f;
        volume_     = 1.0f;
        sustainGain_ = 1.0f + sustain_ * 9.0f;

        buildAdaptorTrees();
        applyVariant();   // sets BJT params, calls initBjts()

        // HP coupling filters — initialised BEFORE warmupPortStates() so they
        // can be pre-warmed to the DC steady state inside that function.
        inputHp_.init(sampleRate);
        inputHp_.setType(OnePoleFilter::Type::Highpass);
        inputHp_.setFrequency(bigmuff::coupling::f_in_hz);

        q1q2Hp_.init(sampleRate);
        q1q2Hp_.setType(OnePoleFilter::Type::Highpass);
        q1q2Hp_.setFrequency(bigmuff::coupling::f_q1q2_hz);

        q2q3Hp_.init(sampleRate);
        q2q3Hp_.setType(OnePoleFilter::Type::Highpass);
        q2q3Hp_.setFrequency(bigmuff::coupling::f_q1q2_hz);

        // Pre-set WDF port.a values AND pre-warm HP filters to DC steady state.
        warmupPortStates();

        // Tone stack
        toneLp_.init(sampleRate);
        toneLp_.setType(OnePoleFilter::Type::Lowpass);
        toneLp_.setFrequency(bigmuff::tone::f_bass_hz);

        toneHp_.init(sampleRate);
        toneHp_.setType(OnePoleFilter::Type::Highpass);
        toneHp_.setFrequency(bigmuff::tone::f_treble_hz);

        // DC block on output
        dcBlock_.init(sampleRate);
        dcBlock_.setType(OnePoleFilter::Type::DCBlock);
    }

    [[nodiscard]] float process(float in) noexcept {
        using namespace bigmuff;

        // Scale guitar signal: 10 mV nominal, boosted by sustain
        const float in_v = in * 0.01f * sustainGain_;

        // ── Stage helper: reflect BJT, clamp Ic to physical range, return Vc ─
        // Clamp prevents: NaN cascades when NR fails near saturation, and the
        // Nyquist oscillation inherent in WDF one-sample-delay feedback.
        // Fixed-point proof: col.port.a* = Vcc - Rpc*Ic  →  next reflect gives
        // col.port.b = Vcc + Rpc*Ic  →  portC.b = Vcc - Rpc*Ic = col.port.a* ✓
        auto runStage = [&](WdfNpnBjt& q,
                            WdfResistiveVoltageSource& base,
                            WdfResistiveVoltageSource& col,
                            WdfResistor& emit) -> float {
            base.reflect();
            col.reflect();
            emit.reflect();

            q.portB.a = base.port.b;
            q.portC.a = col.port.b;
            q.portE.a = emit.port.b;
            q.reflect();

            // Ic = (portC.a - portC.b) / (2*Rpc); clamp to [0, Vcc/Rpc]
            // The NR may compute Ic >> physical max; clamp before updating ports.
            const float Ic = std::clamp(
                (q.portC.a - q.portC.b) / (2.0f * col.port.Rp),
                0.0f, Vsupply / col.port.Rp);

            // Recompute base/emitter scattered waves using clamped Ic.
            // Using the NR's unclamped Ib would cause a Nyquist limit cycle
            // in the base WDF state whenever portB.a is large.
            const float Ib = Ic / curHFE_;
            const float Ie = Ic * (1.0f + 1.0f / curHFE_);
            base.port.a = q.portB.a - 2.0f * q.portB.Rp * Ib;
            emit.port.a = q.portE.a + 2.0f * q.portE.Rp * Ie;

            // Set col.port.a to the steady-state value for this Ic.
            // This is the unique fixed point of the WDF reflect/scatter loop.
            col.port.a = Vsupply - col.port.Rp * Ic;
            return Vsupply - col.port.Rp * Ic;  // physical collector voltage
        };

        // ── Q1: input boost ──────────────────────────────────────────────────
        const float in_ac = inputHp_.process(in_v);
        q1Base_.setVoltage(q1::V_th + in_ac);
        const float Vc1 = runStage(q1_, q1Base_, q1Collector_, q1Emitter_);

        // ── Q2: clipping stage ───────────────────────────────────────────────
        const float q2_in = applyDiodeClip(q1q2Hp_.process(Vc1));
        q2Base_.setVoltage(q2::V_th + q2_in);
        const float Vc2 = runStage(q2_, q2Base_, q2Collector_, q2Emitter_);

        // ── Q3: clipping stage ───────────────────────────────────────────────
        const float q3_in = applyDiodeClip(q2q3Hp_.process(Vc2));
        q3Base_.setVoltage(q3::V_th + q3_in);
        const float Vc3 = runStage(q3_, q3Base_, q3Collector_, q3Emitter_);

        // ── Tone stack ───────────────────────────────────────────────────────
        const float lpOut  = toneLp_.process(Vc3);
        const float hpOut  = toneHp_.process(Vc3);
        const float toned  = toneBlend_ * hpOut + (1.0f - toneBlend_) * lpOut;

        // ── DC block + output scale ──────────────────────────────────────────
        const float out = dcBlock_.process(toned);
        constexpr float kOutScale = 0.15f;
        return out * kOutScale * volume_;
    }

    void setSustain(float sustain) noexcept {
        sustain_     = std::clamp(sustain, 0.0f, 1.0f);
        sustainGain_ = 1.0f + sustain_ * 9.0f;
    }

    void setTone(float tone) noexcept {
        toneBlend_ = std::clamp(tone, 0.0f, 1.0f);
    }

    void setVolume(float volume) noexcept {
        volume_ = std::clamp(volume, 0.0f, 1.0f);
    }

    void setVariant(bigmuff::Variant variant) noexcept {
        variant_ = variant;
        applyVariant();
    }

    /// Reset all filter states and re-warm port states to DC steady state.
    /// Call after setVariant() to obtain a fully deterministic initial condition.
    ///
    /// OnePoleFilter::init() sets parameters but does NOT clear y1_/x1_ state.
    /// Each filter must have reset() called explicitly after init() to guarantee
    /// the same initial state regardless of prior variant or signal history.
    void reset() noexcept {
        inputHp_.init(sampleRate_);
        inputHp_.setType(OnePoleFilter::Type::Highpass);
        inputHp_.setFrequency(bigmuff::coupling::f_in_hz);
        inputHp_.reset();   // clear y1_, x1_ — init() does not do this

        q1q2Hp_.init(sampleRate_);
        q1q2Hp_.setType(OnePoleFilter::Type::Highpass);
        q1q2Hp_.setFrequency(bigmuff::coupling::f_q1q2_hz);
        q1q2Hp_.reset();   // warmupPortStates() will re-warm from zero

        q2q3Hp_.init(sampleRate_);
        q2q3Hp_.setType(OnePoleFilter::Type::Highpass);
        q2q3Hp_.setFrequency(bigmuff::coupling::f_q1q2_hz);
        q2q3Hp_.reset();   // warmupPortStates() will re-warm from zero

        toneLp_.init(sampleRate_);
        toneLp_.setType(OnePoleFilter::Type::Lowpass);
        toneLp_.setFrequency(bigmuff::tone::f_bass_hz);
        toneLp_.reset();

        toneHp_.init(sampleRate_);
        toneHp_.setType(OnePoleFilter::Type::Highpass);
        toneHp_.setFrequency(bigmuff::tone::f_treble_hz);
        toneHp_.reset();

        dcBlock_.init(sampleRate_);
        dcBlock_.setType(OnePoleFilter::Type::DCBlock);
        dcBlock_.reset();

        warmupPortStates();
    }

    // Accessor helpers used by tests.
    // col.port.a is updated to (Vcc - Rpc*Ic_clamped) each sample by runStage(),
    // so Vc = Vcc - Rpc*Ic = col.port.a directly.
    float getQ1CollectorVoltage() const noexcept { return q1Collector_.port.a; }
    float getQ2CollectorVoltage() const noexcept { return q2Collector_.port.a; }
    float getQ3CollectorVoltage() const noexcept { return q3Collector_.port.a; }

private:
    // ── WDF elements ─────────────────────────────────────────────────────────
    // Q1
    WdfResistiveVoltageSource q1Base_;      // Thevenin base: Vs=V_th, R=R_th
    WdfResistiveVoltageSource q1Collector_; // Vcc through R_c1
    WdfResistor               q1Emitter_;   // R_e1 to GND
    WdfNpnBjt                 q1_;

    // Q2
    WdfResistiveVoltageSource q2Base_;
    WdfResistiveVoltageSource q2Collector_; // Vcc through R_c2
    WdfResistor               q2Emitter_;
    WdfNpnBjt                 q2_;

    // Q3
    WdfResistiveVoltageSource q3Base_;
    WdfResistiveVoltageSource q3Collector_; // Vcc through R_c3
    WdfResistor               q3Emitter_;
    WdfNpnBjt                 q3_;

    // ── Filters ──────────────────────────────────────────────────────────────
    OnePoleFilter inputHp_;  // guitar signal → Q1 base
    OnePoleFilter q1q2Hp_;   // Q1 collector → Q2 base
    OnePoleFilter q2q3Hp_;   // Q2 collector → Q3 base
    OnePoleFilter toneLp_;
    OnePoleFilter toneHp_;
    OnePoleFilter dcBlock_;

    // ── State ─────────────────────────────────────────────────────────────────
    float sampleRate_   = 48000.0f;
    float sustain_      = 0.5f;
    float toneBlend_    = 0.5f;
    float volume_       = 1.0f;
    float sustainGain_  = 1.0f;
    bigmuff::Variant variant_ = bigmuff::Variant::RamsHead;

    float curIs_  = bigmuff::bc549c::Is;
    float curVt_  = bigmuff::bc549c::Vt;
    float curHFE_ = bigmuff::bc549c::hFE;

    // ── Inter-stage diode clipper ─────────────────────────────────────────────
    /// Anti-parallel 1N914 diode pair — analytical soft-clip using tanh.
    /// tanh gives odd-symmetric clipping (dominant odd harmonics = Big Muff
    /// character) and is numerically stable for all finite inputs.
    ///
    /// Scale factor: Vt * asinh(1) ≈ 0.0453 * 0.881 is the "knee" of the
    /// diode; we use V_f = 0.65 V as the practical forward voltage at which
    /// the clipper saturates.
    [[nodiscard]] static float applyDiodeClip(float V_in) noexcept {
        constexpr float Vf    = 0.65f;
        constexpr float invVf = 1.0f / Vf;
        return Vf * tanhf(V_in * invVf);
    }

    // ── DC steady-state solver ─────────────────────────────────────────────────
    /// Newton-Raphson for BJT collector current at DC steady state.
    ///
    /// Solves: f(Ic) = Vt*ln(Ic/Is) + RpE*(1+1/hFE)*Ic - V_bias = 0
    ///
    /// Derivation: at WDF steady state the base port.a_ss = V_bias - RpB*Ib
    /// and emitter port.b = 0 (WdfResistor), so portE.a = 0 always, giving
    /// Vbe_ss = portB.a - RpB*Ib - 0 - RpE*Ie = V_bias - RpB*Ib - RpE*Ie.
    /// Substituting Ib = Ic/hFE, Ie = Ic*(1+1/hFE) and rearranging yields
    /// the equation above (RpB*Ib terms cancel when Ic/hFE << Ic).
    [[nodiscard]] static float solveIcSteadyState(float Is, float Vt, float hFE,
                                                   float V_bias, float RpE) noexcept {
        const float alpha = RpE * (1.0f + 1.0f / hFE);
        float Ic = Is * expf(V_bias / (2.0f * Vt));
        Ic = std::clamp(Ic, 1e-12f, 0.1f);
        for (int i = 0; i < 20; ++i) {
            const float lnArg = std::max(Ic / Is, 1e-30f);
            const float f  = Vt * logf(lnArg) + alpha * Ic - V_bias;
            const float df = Vt / Ic + alpha;
            const float dIc = std::clamp(f / df, -Ic * 0.5f, Ic * 2.0f);
            Ic -= dIc;
            Ic = std::max(Ic, 1e-12f);
            if (fabsf(dIc / (Ic + 1e-12f)) < 1e-6f) break;
        }
        return Ic;
    }

    // ── Port state pre-warming ─────────────────────────────────────────────────
    /// Pre-set WDF port.a values to DC steady state, and pre-warm HP filters.
    ///
    /// WDF one-sample-delay mechanism: WdfResistiveVoltageSource.reflect() sets
    ///   port.b = 2*Vs - port.a
    /// then the BJT scatter sets port.a_new = portC.b (from BJT reflect).
    /// The fixed point is: port.a* = Vs - Rp*I_ss (collector source: Vcc, Ic).
    /// Initialising to this value eliminates the Nyquist oscillation that
    /// would otherwise appear on the first ~100 samples.
    ///
    /// HP filter pre-warming: at startup the HP filter state is zero. On sample 0
    /// with a DC input Vc_dc, the output = Vc_dc (full pass), which drives the
    /// next stage base to Vc_dc + V_bias >> supply → NaN/blow-up. Running 2000
    /// dummy passes settles the filter to < 0.02% of the initial spike.
    void warmupPortStates() noexcept {
        using namespace bigmuff;

        // Q1 DC
        const float Ic1 = solveIcSteadyState(curIs_, curVt_, curHFE_,
                                              q1::V_th, q1Emitter_.port.Rp);
        q1Base_.port.a      = q1::V_th  - q1Base_.port.Rp * (Ic1 / curHFE_);
        q1Collector_.port.a = Vsupply   - q1Collector_.port.Rp * Ic1;

        // Pre-warm Q1→Q2 HP filter with Q1 DC collector voltage
        const float Vc1_dc = Vsupply - q1Collector_.port.Rp * Ic1;
        for (int i = 0; i < 2000; ++i)
            (void)q1q2Hp_.process(Vc1_dc);

        // Q2/Q3 DC (same component values)
        const float Ic2 = solveIcSteadyState(curIs_, curVt_, curHFE_,
                                              q2::V_th, q2Emitter_.port.Rp);
        q2Base_.port.a      = q2::V_th - q2Base_.port.Rp * (Ic2 / curHFE_);
        q2Collector_.port.a = Vsupply  - q2Collector_.port.Rp * Ic2;

        q3Base_.port.a      = q3::V_th - q3Base_.port.Rp * (Ic2 / curHFE_);
        q3Collector_.port.a = Vsupply  - q3Collector_.port.Rp * Ic2;

        // Pre-warm Q2→Q3 HP filter with Q2 DC collector voltage
        // (at DC with no inter-stage signal, Q2 bias is V_th_q2 only)
        const float Vc2_dc = Vsupply - q2Collector_.port.Rp * Ic2;
        for (int i = 0; i < 2000; ++i)
            (void)q2q3Hp_.process(Vc2_dc);
    }

    // ── Adaptor tree construction ──────────────────────────────────────────────
    void buildAdaptorTrees() noexcept {
        using namespace bigmuff;

        // Q1
        q1Base_.init(q1::R_th);
        q1Base_.setVoltage(q1::V_th);
        q1Collector_.init(q1::R_c);
        q1Collector_.setVoltage(Vsupply);
        q1Emitter_.init(q1::R_e);

        // Q2
        q2Base_.init(q2::R_th);
        q2Base_.setVoltage(q2::V_th);
        q2Collector_.init(q2::R_c);
        q2Collector_.setVoltage(Vsupply);
        q2Emitter_.init(q2::R_e);

        // Q3
        q3Base_.init(q3::R_th);
        q3Base_.setVoltage(q3::V_th);
        q3Collector_.init(q3::R_c);
        q3Collector_.setVoltage(Vsupply);
        q3Emitter_.init(q3::R_e);
    }

    void initBjts() noexcept {
        q1_.init(curIs_, curVt_, curHFE_,
                 q1Base_.port.Rp, q1Collector_.port.Rp, q1Emitter_.port.Rp);
        q2_.init(curIs_, curVt_, curHFE_,
                 q2Base_.port.Rp, q2Collector_.port.Rp, q2Emitter_.port.Rp);
        q3_.init(curIs_, curVt_, curHFE_,
                 q3Base_.port.Rp, q3Collector_.port.Rp, q3Emitter_.port.Rp);
    }

    void applyVariant() noexcept {
        using namespace bigmuff;
        switch (variant_) {
            case Variant::RamsHead:
            case Variant::Triangle:
                curIs_ = bc549c::Is; curVt_ = bc549c::Vt; curHFE_ = bc549c::hFE;
                break;
            case Variant::CivilWar:
                curIs_ = npn2n3904::Is; curVt_ = npn2n3904::Vt; curHFE_ = npn2n3904::hFE;
                break;
            case Variant::NYC:
                // 2N5088 NPN — EHX Big Muff Pi NYC
                // Diode parameters unchanged: 1N914 antiparallel (same for all variants)
                curIs_ = npn2n5088::Is; curVt_ = npn2n5088::Vt; curHFE_ = npn2n5088::hFE;
                break;
        }
        initBjts();
    }
};
