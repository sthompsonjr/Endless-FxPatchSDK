#pragma once

// EHX Deluxe Memory Man (EH-7850) — Signal Conditioning Chain
// Session 1 of 3: Input/Output conditioning blocks (BBD not included)
//
// References:
//   EH-7850 schematic (Retro Electro archive, rev D)
//   Werner et al. "EUSIPCO 2016 — Wave Digital Filters: Revisited"
//   JRC4558D datasheet (Japan Radio Corp, Rev 5)
//
// Supply rail note:
//   EH-7850 runs from a single -15V supply with VREF at ~+7.52V (positive
//   ground topology). All voltages are normalized relative to VREF.
//   Op-amp rail swing is therefore ±7.52V → setRailVoltage(7.5f) on all
//   WdfOpAmpJRC4558 instances.
//
// RC4558 slew-rate note:
//   The JRC4558 model uses A0=50, fp=300Hz, SR=322000 V/s (conservative).
//   Closed-loop gain accuracy at 1kHz will be ~5-8% below the theoretical
//   Av = 1 + R4/R3 due to finite open-loop gain. This is a known model
//   limitation, not a circuit error.
//
// CYCLE BUDGET (528 MHz / 48 kHz = 11,000 cycles/sample):
//   DmmInputBuffer      ~350 cycles  (3 WDF leaves, 2 adaptors, 1 op-amp)
//   DmmAntiAliasFilter  ~550 cycles  (5 WDF leaves, 4 adaptors, 1 op-amp)
//   DmmReconFilter      ~550 cycles  (identical tree, +1 multiply)
//   DmmFeedbackEq       ~30–60 cycles (direct IIR biquad; WDF tree removed)
//   DmmOutputBuffer     ~150 cycles  (1 op-amp, no tree)
//   Total (all 5):    ~1,900 cycles  (~17% of budget)

#include "WdfOnePort.h"
#include "WdfAdaptors.h"
#include "WdfOpAmpJRC4558.h"
#include "dsp/ParameterSmoother.h"
#include <cmath>
#include <array>

// ---------------------------------------------------------------------------
// Block A1: DmmInputBuffer
//
// IC1a RC4558, non-inverting amplifier, Av = 1 + R4/R3 = 1 + 47k/22k ≈ 3.14×
//
// Schematic topology:
//   Vin → C1(100nF) → Node_in → [R2(100kΩ) to VREF || R1(2.2MΩ) to VREF]
//   V(Node_in) → op-amp V+
//   op-amp feedback: Vout → R4(47kΩ) → V- → R3(22kΩ) → VREF
//
// WDF tree:
//   Series(C1, Parallel(R2, R1))
//   Root port is driven as ideal voltage source (Thevenin injection).
//
// Port resistances:
//   C1:  Rp = 1/(2·100e-9·48000) = 104.167 Ω
//   R2:  Rp = 100,000 Ω
//   R1:  Rp = 2,200,000 Ω
//   Par: Rp = R2||R1 = 95,652 Ω
//   Ser: Rp = C1 + Par = 95,756 Ω  (root)
//
// Op-amp feedback:
//   β = R3/(R3+R4) = 22k/69k ≈ 0.31884
//   One-sample delay on V- (Werner 2016 §4.2 approximation for linear stages).
// ---------------------------------------------------------------------------
struct DmmInputBuffer {
    using InnerPar = WdfParallelAdaptor2<WdfResistor, WdfResistor>;
    using Tree     = WdfSeriesAdaptor2<WdfCapacitor, InnerPar>;

    void init(float sampleRate) noexcept {
        tree_.childA.init(100e-9f, sampleRate);   // C1
        tree_.childB.childA.init(100e3f);          // R2
        tree_.childB.childB.init(2.2e6f);          // R1
        tree_.childB.updatePortResistance();
        tree_.updatePortResistance();
        opamp_.init(sampleRate);
        opamp_.setRailVoltage(7.5f);
        vMinusPrev_ = 0.0f;
    }

    float process(float vin) noexcept {
        tree_.reflect();
        tree_.port.a = 2.0f * vin - tree_.port.b;
        tree_.scatter();
        tree_.childB.scatter();

        float vPlus = tree_.childB.port.voltage();
        float vOut  = opamp_.process(vPlus - vMinusPrev_);
        vMinusPrev_ = vOut * kFeedbackRatio;
        return vOut;
    }

    void reset() noexcept {
        tree_.reset();
        opamp_.reset();
        vMinusPrev_ = 0.0f;
    }

private:
    static constexpr float kFeedbackRatio = 22000.0f / 69000.0f; // R3/(R3+R4)

    Tree            tree_;
    WdfOpAmpJRC4558 opamp_;
    float           vMinusPrev_ = 0.0f;
};

// ---------------------------------------------------------------------------
// Block A3: DmmAntiAliasFilter
//
// IC3a RC4558 Sallen-Key LPF, fc ≈ 2.1 kHz, Q ≈ 0.5 (Butterworth-ish)
// Component values: R13=51kΩ, R14=16kΩ, C11=C12=2.7nF
//
// Sallen-Key topology (unity-gain):
//   Vin → R13 → Node_A → [C11 (to op-amp Vout) || R14 → C12 → VREF]
//   Op-amp: unity gain (V+ = C12 top = Node_B, Vout = V+)
//
// WDF tree:
//   Series(R13, Parallel(Series(C11, Vs), Series(R14, C12)))
//
//   Where Vs (WdfIdealVoltageSource) is set to vOutPrev_ each sample to
//   break the delay-free feedback loop through C11. This is the standard
//   WDF technique for op-amp output feedback injection into a capacitor
//   branch (Rp_Vs ≈ 1e-6 Ω, negligible effect on junction impedance).
//
// Port resistances (at 48kHz):
//   C11:  Rp = 1/(2·2.7e-9·48000) = 3,858 Ω
//   Vs:   Rp = 1e-6 Ω
//   C11Br: Rp = 3,858 + 1e-6 ≈ 3,858 Ω
//   R14:  Rp = 16,000 Ω
//   C12:  Rp = 3,858 Ω
//   NodeBBr: Rp = 16,000 + 3,858 = 19,858 Ω
//   NodeAPar: Rp = 3858||19858 = 3,196 Ω
//   R13:  Rp = 51,000 Ω
//   SkTree root: Rp = 51,000 + 3,196 = 54,196 Ω
// ---------------------------------------------------------------------------
struct DmmAntiAliasFilter {
    using NodeBBr  = WdfSeriesAdaptor2<WdfResistor, WdfCapacitor>;
    using C11Br    = WdfSeriesAdaptor2<WdfCapacitor, WdfIdealVoltageSource>;
    using NodeAPar = WdfParallelAdaptor2<C11Br, NodeBBr>;
    using SkTree   = WdfSeriesAdaptor2<WdfResistor, NodeAPar>;

    void init(float sampleRate) noexcept {
        tree_.childA.init(51000.0f);                               // R13
        tree_.childB.childA.childA.init(2.7e-9f, sampleRate);     // C11
        tree_.childB.childA.childB.init();                         // Vs (no args)
        tree_.childB.childB.childA.init(16000.0f);                 // R14
        tree_.childB.childB.childB.init(2.7e-9f, sampleRate);     // C12
        tree_.childB.childA.updatePortResistance();
        tree_.childB.childB.updatePortResistance();
        tree_.childB.updatePortResistance();
        tree_.updatePortResistance();
        opamp_.init(sampleRate);
        opamp_.setRailVoltage(7.5f);
        vOutPrev_ = 0.0f;
    }

    float process(float vin) noexcept {
        tree_.childB.childA.childB.setVoltage(vOutPrev_);

        tree_.reflect();
        tree_.port.a = 2.0f * vin - tree_.port.b;
        tree_.scatter();
        tree_.childB.scatter();
        tree_.childB.childA.scatter();  // update C11.port.a so its state evolves
        tree_.childB.childB.scatter();

        float vNodeB = tree_.childB.childB.childB.port.voltage();
        vOutPrev_ = opamp_.process(vNodeB - vOutPrev_);
        return vOutPrev_;
    }

    void reset() noexcept {
        tree_.reset();
        opamp_.reset();
        vOutPrev_ = 0.0f;
    }

private:
    SkTree          tree_;
    WdfOpAmpJRC4558 opamp_;
    float           vOutPrev_ = 0.0f;
};

// ---------------------------------------------------------------------------
// Block A5: DmmReconFilter
//
// IC5a RC4558 Sallen-Key LPF, fc ≈ 3.8 kHz (post-BBD reconstruction)
// Component values: R27=16kΩ, R28=15kΩ, C16=C17=2.7nF
//
// Identical Sallen-Key WDF topology as DmmAntiAliasFilter.
// fc is intentionally higher than the anti-alias filter — this is the
// correct EH-7850 "warmth asymmetry" behavior; do not equalize.
//
// Gain recovery:
//   The BBD output is attenuated ~1/4.545× by the BBD interface circuitry.
//   R29(39kΩ) and R30(11kΩ) form a non-inverting gain stage on IC5b that
//   follows this filter. That stage is modeled here as a scalar post-multiply:
//   kGainRecovery = 1 + 39k/11k ≈ 4.545
//   (The IC5b stage is linear and not separately WDF-modeled.)
//
// Port resistances (at 48kHz):
//   C16,C17: Rp = 3,858 Ω each (same as DmmAntiAliasFilter)
//   C16Br:   Rp = 3,858 Ω
//   NodeBBr: Rp = 15,000 + 3,858 = 18,858 Ω
//   NodeAPar: Rp = 3858||18858 = 3,107 Ω
//   R27:     Rp = 16,000 Ω
//   SkTree root: Rp = 16,000 + 3,107 = 19,107 Ω
// ---------------------------------------------------------------------------
struct DmmReconFilter {
    using NodeBBr  = WdfSeriesAdaptor2<WdfResistor, WdfCapacitor>;
    using C16Br    = WdfSeriesAdaptor2<WdfCapacitor, WdfIdealVoltageSource>;
    using NodeAPar = WdfParallelAdaptor2<C16Br, NodeBBr>;
    using SkTree   = WdfSeriesAdaptor2<WdfResistor, NodeAPar>;

    void init(float sampleRate) noexcept {
        tree_.childA.init(16000.0f);                               // R27
        tree_.childB.childA.childA.init(2.7e-9f, sampleRate);     // C16
        tree_.childB.childA.childB.init();                         // Vs
        tree_.childB.childB.childA.init(15000.0f);                 // R28
        tree_.childB.childB.childB.init(2.7e-9f, sampleRate);     // C17
        tree_.childB.childA.updatePortResistance();
        tree_.childB.childB.updatePortResistance();
        tree_.childB.updatePortResistance();
        tree_.updatePortResistance();
        opamp_.init(sampleRate);
        opamp_.setRailVoltage(7.5f);
        vOutPrev_ = 0.0f;
    }

    float process(float vin) noexcept {
        tree_.childB.childA.childB.setVoltage(vOutPrev_);

        tree_.reflect();
        tree_.port.a = 2.0f * vin - tree_.port.b;
        tree_.scatter();
        tree_.childB.scatter();
        tree_.childB.childA.scatter();  // update C16.port.a so its state evolves
        tree_.childB.childB.scatter();

        float vNodeB = tree_.childB.childB.childB.port.voltage();
        vOutPrev_ = opamp_.process(vNodeB - vOutPrev_);
        return vOutPrev_ * kGainRecovery;
    }

    void reset() noexcept {
        tree_.reset();
        opamp_.reset();
        vOutPrev_ = 0.0f;
    }

private:
    // Gain recovery for post-BBD IC5b stage: Av = 1 + R29/R30 = 1 + 39k/11k
    static constexpr float kGainRecovery = 1.0f + 39000.0f / 11000.0f;

    SkTree          tree_;
    WdfOpAmpJRC4558 opamp_;
    float           vOutPrev_ = 0.0f;
};

// ---------------------------------------------------------------------------
// Block B8: DmmFeedbackEq
//
// Passive RC feedback equalization / darkening network (no op-amp).
// Located in the feedback path; reduces high-frequency content on repeat.
//
// Schematic: Vin → C14 → R31(1MΩ) → Node_N2 → [C15 || R36(240kΩ)]
//
// BUG ANALYSIS — original WDF tree (session 1):
// ------------------------------------------------
// The original implementation used:
//   Series(C14, Series(R31, Parallel(C15, R36)))
//
// Per WDF first principles (Fettweis 1986; Yeh & Smith DAFx-08, Table 1):
//   WdfResistor reflected wave: b[n] = 0 always.
//
// In a WDF series adaptor with children A and B:
//   b_A = a_A - (2*Ra/(Ra+Rb)) * (a_A + a_B)
//   b_B = a_B - (2*Rb/(Ra+Rb)) * (a_A + a_B)
//
// When B = WdfResistor (R31), b_B = 0, so the adaptor's scattering
// reduces to b_A = -a_A (inverted passthrough). The downstream parallel
// junction (N2) therefore receives the full incident wave with no voltage
// division from R31. The 1MΩ vs 240kΩ ratio that should produce the
// ~−12 dB mid-band attenuation is algebraically cancelled by b_R31 = 0.
//
// A resistor in a WDF tree contributes its PORT RESISTANCE to adaptor
// impedance matching but contributes ZERO reflected energy. It is only
// meaningful as part of a source (WdfResistiveVoltageSource) or as
// an adaptor impedance, not as a standalone voltage-dividing element
// in the middle of a tree.
//
// FIX: Replace the WDF tree with a direct IIR biquad implementing the
// analog transfer function via bilinear transform.
//
// ANALOG TRANSFER FUNCTION DERIVATION:
// -------------------------------------
// Circuit: Vin → C14 → N1 → R31(1MΩ) → N2; N2 → C15 → GND; N2 → R36(240kΩ) → Vout
//
// Z_shunt(s) = Z_C15 || Z_R36 = R36 / (1 + s·τ2),  τ2 = C15·R36
// Z_series(s) = Z_C14 + Z_R31 = R31 + 1/(s·C14)
//
// H(s) = Z_shunt / (Z_series + Z_shunt)
//
// Multiplying numerator and denominator by s·C14·(1 + s·τ2):
//   Numerator:   s·C14·R36
//   Denominator: 1 + s·(τ2 + C14·R31 + C14·R36) + s²·(C14·R31·τ2)
//
// Time constants (in milliseconds — see BLT note below):
//   τ2       = C15·R36 = 0.0528 ms  → 1/(2π·τ2) ≈ 3.01 kHz HF rolloff
//   C14·R31  = 1.0    ms            → HPF zero near DC
//   C14·R36  = 0.24   ms            → numerator scale
//
// Coefficients:
//   a_s2 = C14·R31·τ2           = 0.0528
//   a_s1 = τ2 + C14·R31 + C14·R36 = 0.0528 + 1.0 + 0.24 = 1.2928
//   a_s0 = 1.0
//   b_s1 = C14·R36               = 0.24   (numerator s coefficient)
//
// H(s) = (0.24·s) / (0.0528·s² + 1.2928·s + 1)   [s in rad/ms = krad/s]
//
// Mid-band gain (well above HPF, below HF rolloff ≈ 3 kHz):
//   H → b_s1/a_s1 ... but more precisely the peak is near where
//   the reactive terms cancel; at 1 kHz the gain ≈ 0.18.
//   Expected resistive ratio: R36/(R31+R36) = 240k/1240k = 0.194.
//   Digital result ≈ 0.18 (reactive loading at 1 kHz shifts it slightly).
//
// BILINEAR TRANSFORM (Direct Form II Transposed):
// -----------------------------------------------
// NOTE: time constants above are in MILLISECONDS (τ2 = 0.0528 ms, not 0.0528 s).
//   The BLT parameter k must therefore match: k = 2·fs_kHz = 2·48 = 96 at 48 kHz.
//   Using k = 2·sampleRate (Hz) = 96000 would give near-zero audio-band gain
//   because the poles would map to z ≈ 1 with negligible bandwidth.
//
// Substitute s = k·(z−1)/(z+1), multiply through by (z+1)², collect z^{−n}:
//
//   Numerator:   b_s1·k·(z²−1)  →  b0 = +b_s1·k/D,  b1 = 0,  b2 = −b_s1·k/D
//     (b1 = 0 confirmed: (z−1)(z+1) = z²−1, coefficient of z^1 is zero)
//
//   Denominator scale D = a_s2·k² + a_s1·k + a_s0
//   a1 = 2·(a_s0 − a_s2·k²) / D       (z^{−1} term, negative → resonance below Nyquist)
//   a2 = (a_s2·k² − a_s1·k + a_s0) / D
//
// At fs = 48 kHz, k = 96:
//   D    = 0.0528·9216 + 1.2928·96 + 1 = 486.60 + 124.11 + 1 = 611.71
//   b0   =  0.24·96 / 611.71 ≈  0.03766
//   b2   = −0.03766
//   a1   = 2·(1 − 486.60) / 611.71 ≈ −1.5877  (negative → pole between DC and Nyquist)
//   a2   = (486.60 − 124.11 + 1) / 611.71 ≈  0.5942
//
// Poles at z = (−a1 ± √(a1²−4·a2))/2 = 0.9835 and 0.6044 — both real, |z|<1, stable.
//
// Frequency response verification at fs = 48 kHz:
//   f =    1 kHz : |H| ≈ 0.184  (target ≈ 0.18) ✓
//   f =   20 Hz  : |H| ≈ 0.030  (< 0.05 HPF confirmed) ✓
//   f =   10 kHz : |H| ≈ 0.068  (< 0.10 LPF confirmed) ✓
//
// Runaway guard: kRunawayGain = 0.92f — caller multiplies feedback signal
//   by this constant to prevent self-oscillation at high feedback settings.
// ---------------------------------------------------------------------------
struct DmmFeedbackEq {
    // Multiply feedback signal by this constant before passing to process()
    // to prevent self-oscillation at high feedback settings.
    static constexpr float kRunawayGain = 0.92f;

    // Biquad state (Direct Form II Transposed)
    float s1 = 0.0f;
    float s2 = 0.0f;

    // Coefficients computed by init() from bilinear transform of H(s) above
    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;
    float a1 = 0.0f, a2 = 0.0f;

    void init(float sampleRate) noexcept {
        // H(s) = 0.24·s / (0.0528·s² + 1.2928·s + 1), s in rad/ms (krad/s)
        // BLT: k = 2·fs_kHz  (NOT 2·fs_Hz — time constants are in milliseconds)
        const float k    = 2.0f * sampleRate / 1000.0f;  // 96.0 at 48 kHz
        const float k2   = k * k;

        // Analog H(s) coefficients (dimensionless after ms normalization)
        constexpr float b_s1 = 0.24f;    // numerator: s coefficient
        constexpr float a_s0 = 1.0f;     // denominator: constant
        constexpr float a_s1 = 1.2928f;  // denominator: s coefficient
        constexpr float a_s2 = 0.0528f;  // denominator: s² coefficient

        // Bilinear substitution denominator scale
        const float denom = a_s2 * k2 + a_s1 * k + a_s0;

        // Digital coefficients — bandpass form: zero at DC (z=1) and Nyquist (z=−1)
        // Numerator: b_s1·k·(1 − z^{−2}), so b1 = 0 and b2 = −b0
        b0 =  b_s1 * k / denom;
        b1 =  0.0f;
        b2 = -b_s1 * k / denom;

        // Denominator (stored with natural sign; process() uses minus):
        //   1 + a1·z^{−1} + a2·z^{−2}
        a1 = 2.0f * (a_s0 - a_s2 * k2) / denom;
        a2 = (a_s2 * k2 - a_s1 * k + a_s0) / denom;

        s1 = 0.0f;
        s2 = 0.0f;
    }

    float process(float x) noexcept {
        // Direct Form II Transposed:
        //   H(z) = (b0 + b1·z^{−1} + b2·z^{−2}) / (1 + a1·z^{−1} + a2·z^{−2})
        const float y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
        return y;
    }

    void reset() noexcept {
        s1 = 0.0f;
        s2 = 0.0f;
    }
};

// ---------------------------------------------------------------------------
// Block A9: DmmOutputBuffer
//
// IC9b RC4558 unity-gain follower (voltage follower / buffer).
// R58(100kΩ) is a shunt resistor to VREF at op-amp V+ — with infinite
// op-amp input impedance this produces V+ = vin with no voltage divider
// effect (the WDF tree is not needed; R58 is listed for completeness).
//
// Unity-gain approximation: closed-loop delay one-sample feedback (same
// technique as DmmInputBuffer with β=1.0).
// ---------------------------------------------------------------------------
struct DmmOutputBuffer {
    void init(float sampleRate) noexcept {
        opamp_.init(sampleRate);
        opamp_.setRailVoltage(7.5f);
        vOutPrev_ = 0.0f;
    }

    float process(float vin) noexcept {
        float vOut = opamp_.process(vin - vOutPrev_);
        vOutPrev_  = vOut;
        return vOut;
    }

    void reset() noexcept {
        opamp_.reset();
        vOutPrev_ = 0.0f;
    }

private:
    WdfOpAmpJRC4558 opamp_;
    float           vOutPrev_ = 0.0f;
};

// ---------------------------------------------------------------------------
// Self-test
// Compile: g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra
//          -DDMM_CIRCUITS_SELF_TEST -I. wdf/DmmCircuits.h -o /tmp/dmm_test
// Run:     /tmp/dmm_test
// ---------------------------------------------------------------------------
#ifdef DMM_CIRCUITS_SELF_TEST
#include <cstdio>
#include <cmath>

int main() {
    constexpr float kFs       = 48000.0f;
    constexpr int   kWarmup   = 1000;
    constexpr int   kMeasure  = 1000;
    constexpr float kFreq     = 1000.0f;
    constexpr float kAmp      = 0.1f;  // 100mV — well within ±7.5V rail

    int passed = 0;
    int failed = 0;

    auto check = [&](const char* name, float got, float expected, float tol) {
        float err = std::abs(got - expected) / (std::abs(expected) + 1e-9f);
        bool ok = err <= tol;
        std::printf("[%s] %s: got=%.4f  expected=%.4f  err=%.1f%%  tol=%.0f%%\n",
                    ok ? "PASS" : "FAIL", name, got, expected, err * 100.0f, tol * 100.0f);
        if (ok) ++passed; else ++failed;
    };

    // --- DmmInputBuffer ---
    {
        DmmInputBuffer blk;
        blk.init(kFs);
        float inRms  = 0.0f;
        float outRms = 0.0f;

        for (int i = 0; i < kWarmup; ++i) {
            float s = kAmp * std::sin(2.0f * 3.14159265f * kFreq * static_cast<float>(i) / kFs);
            blk.process(s);
        }
        float sumIn2 = 0.0f, sumOut2 = 0.0f;
        for (int i = 0; i < kMeasure; ++i) {
            float s = kAmp * std::sin(2.0f * 3.14159265f * kFreq * static_cast<float>(kWarmup + i) / kFs);
            sumIn2  += s * s;
            float o = blk.process(s);
            sumOut2 += o * o;
        }
        inRms  = std::sqrt(sumIn2  / kMeasure);
        outRms = std::sqrt(sumOut2 / kMeasure);
        float gain = outRms / (inRms + 1e-9f);
        // Theoretical: 3.14×; model achieves ~2.9–3.0× (finite A0=50 limit)
        check("DmmInputBuffer gain", gain, 3.14f, 0.15f);
    }

    // --- DmmAntiAliasFilter: passband gain ~1.0 at 500 Hz ---
    {
        DmmAntiAliasFilter blk;
        blk.init(kFs);
        constexpr float kFreqPB = 500.0f;

        for (int i = 0; i < kWarmup; ++i) {
            float s = kAmp * std::sin(2.0f * 3.14159265f * kFreqPB * static_cast<float>(i) / kFs);
            blk.process(s);
        }
        float sumIn2 = 0.0f, sumOut2 = 0.0f;
        for (int i = 0; i < kMeasure; ++i) {
            float s = kAmp * std::sin(2.0f * 3.14159265f * kFreqPB * static_cast<float>(kWarmup + i) / kFs);
            sumIn2  += s * s;
            float o = blk.process(s);
            sumOut2 += o * o;
        }
        float inRms  = std::sqrt(sumIn2  / kMeasure);
        float outRms = std::sqrt(sumOut2 / kMeasure);
        float gain   = outRms / (inRms + 1e-9f);
        check("DmmAntiAliasFilter passband gain @500Hz", gain, 1.0f, 0.15f);
    }

    // --- DmmAntiAliasFilter: stopband at 10 kHz must be < -12 dB (gain < 0.25) ---
    {
        DmmAntiAliasFilter blk;
        blk.init(kFs);
        constexpr float kFreqSB = 10000.0f;

        for (int i = 0; i < kWarmup; ++i) {
            float s = kAmp * std::sin(2.0f * 3.14159265f * kFreqSB * static_cast<float>(i) / kFs);
            blk.process(s);
        }
        float sumOut2 = 0.0f;
        for (int i = 0; i < kMeasure; ++i) {
            float s = kAmp * std::sin(2.0f * 3.14159265f * kFreqSB * static_cast<float>(kWarmup + i) / kFs);
            float o = blk.process(s);
            sumOut2 += o * o;
        }
        float outRms = std::sqrt(sumOut2 / kMeasure);
        float gain   = outRms / (kAmp * 0.7071f + 1e-9f);  // normalize vs input RMS
        bool ok = gain < 0.25f;
        std::printf("[%s] DmmAntiAliasFilter stopband @10kHz: gain=%.4f (need <0.25)\n",
                    ok ? "PASS" : "FAIL", gain);
        if (ok) ++passed; else ++failed;
    }

    // --- DmmReconFilter: output ≈ input × 4.545 at 500 Hz ---
    {
        DmmReconFilter blk;
        blk.init(kFs);
        constexpr float kFreqPB = 500.0f;

        for (int i = 0; i < kWarmup; ++i) {
            float s = kAmp * std::sin(2.0f * 3.14159265f * kFreqPB * static_cast<float>(i) / kFs);
            blk.process(s);
        }
        float sumIn2 = 0.0f, sumOut2 = 0.0f;
        for (int i = 0; i < kMeasure; ++i) {
            float s = kAmp * std::sin(2.0f * 3.14159265f * kFreqPB * static_cast<float>(kWarmup + i) / kFs);
            sumIn2  += s * s;
            float o = blk.process(s);
            sumOut2 += o * o;
        }
        float inRms  = std::sqrt(sumIn2  / kMeasure);
        float outRms = std::sqrt(sumOut2 / kMeasure);
        float gain   = outRms / (inRms + 1e-9f);
        check("DmmReconFilter gain @500Hz", gain, 4.545f, 0.15f);
    }

    // --- DmmFeedbackEq: IIR biquad —
    //     1 kHz ≈ 0.18, 20 Hz < 0.05, 10 kHz < 0.10, no NaN/Inf ---
    {
        // 1 kHz mid-band gain ≈ 0.18 (bilinear transform of H(s); see derivation)
        {
            DmmFeedbackEq blk;
            blk.init(kFs);

            for (int i = 0; i < kWarmup; ++i) {
                float s = kAmp * std::sin(2.0f * 3.14159265f * kFreq * static_cast<float>(i) / kFs);
                blk.process(s);
            }
            float sumIn2 = 0.0f, sumOut2 = 0.0f;
            for (int i = 0; i < kMeasure; ++i) {
                float s = kAmp * std::sin(2.0f * 3.14159265f * kFreq * static_cast<float>(kWarmup + i) / kFs);
                sumIn2  += s * s;
                float o = blk.process(s);
                sumOut2 += o * o;
            }
            float inRms  = std::sqrt(sumIn2  / kMeasure);
            float outRms = std::sqrt(sumOut2 / kMeasure);
            float gain   = outRms / (inRms + 1e-9f);
            check("DmmFeedbackEq gain @1kHz", gain, 0.18f, 0.12f);
        }
        // 20 Hz — strong HPF rolloff: gain must be < 0.05
        {
            constexpr float kFreqLF = 20.0f;
            DmmFeedbackEq blk;
            blk.init(kFs);
            for (int i = 0; i < kWarmup; ++i) {
                float s = kAmp * std::sin(2.0f * 3.14159265f * kFreqLF * static_cast<float>(i) / kFs);
                blk.process(s);
            }
            float sumIn2 = 0.0f, sumOut2 = 0.0f;
            for (int i = 0; i < kMeasure; ++i) {
                float s = kAmp * std::sin(2.0f * 3.14159265f * kFreqLF * static_cast<float>(kWarmup + i) / kFs);
                sumIn2  += s * s;
                float o = blk.process(s);
                sumOut2 += o * o;
            }
            float inRms  = std::sqrt(sumIn2  / kMeasure);
            float outRms = std::sqrt(sumOut2 / kMeasure);
            float gain   = outRms / (inRms + 1e-9f);
            bool ok = gain < 0.05f;
            std::printf("[%s] DmmFeedbackEq HPF @20Hz: gain=%.4f (need <0.05)\n",
                        ok ? "PASS" : "FAIL", gain);
            if (ok) ++passed; else ++failed;
        }
        // 10 kHz — HF rolloff: gain must be < 0.10
        {
            constexpr float kFreqHF = 10000.0f;
            DmmFeedbackEq blk;
            blk.init(kFs);
            for (int i = 0; i < kWarmup; ++i) {
                float s = kAmp * std::sin(2.0f * 3.14159265f * kFreqHF * static_cast<float>(i) / kFs);
                blk.process(s);
            }
            float sumIn2 = 0.0f, sumOut2 = 0.0f;
            for (int i = 0; i < kMeasure; ++i) {
                float s = kAmp * std::sin(2.0f * 3.14159265f * kFreqHF * static_cast<float>(kWarmup + i) / kFs);
                sumIn2  += s * s;
                float o = blk.process(s);
                sumOut2 += o * o;
            }
            float inRms  = std::sqrt(sumIn2  / kMeasure);
            float outRms = std::sqrt(sumOut2 / kMeasure);
            float gain   = outRms / (inRms + 1e-9f);
            bool ok = gain < 0.10f;
            std::printf("[%s] DmmFeedbackEq LPF @10kHz: gain=%.4f (need <0.10)\n",
                        ok ? "PASS" : "FAIL", gain);
            if (ok) ++passed; else ++failed;
        }
        // Numerical stability: 48000 samples of pink-noise-like input, no NaN/Inf
        {
            DmmFeedbackEq blk;
            blk.init(kFs);
            bool finite = true;
            float x = 0.31623f; // -10 dBFS seed
            for (int i = 0; i < 48000; ++i) {
                // Simple LFSR-ish float noise via bit twiddling on phase
                x = std::sin(x * 1.618033f + 0.5f) * 0.31623f;
                float o = blk.process(x);
                if (!std::isfinite(o)) { finite = false; break; }
            }
            std::printf("[%s] DmmFeedbackEq NaN/Inf check (48000 samples)\n",
                        finite ? "PASS" : "FAIL");
            if (finite) ++passed; else ++failed;
        }
    }

    // --- DmmOutputBuffer: unity gain ≈ 1.0 ---
    {
        DmmOutputBuffer blk;
        blk.init(kFs);

        for (int i = 0; i < kWarmup; ++i) {
            float s = kAmp * std::sin(2.0f * 3.14159265f * kFreq * static_cast<float>(i) / kFs);
            blk.process(s);
        }
        float sumIn2 = 0.0f, sumOut2 = 0.0f;
        for (int i = 0; i < kMeasure; ++i) {
            float s = kAmp * std::sin(2.0f * 3.14159265f * kFreq * static_cast<float>(kWarmup + i) / kFs);
            sumIn2  += s * s;
            float o = blk.process(s);
            sumOut2 += o * o;
        }
        float inRms  = std::sqrt(sumIn2  / kMeasure);
        float outRms = std::sqrt(sumOut2 / kMeasure);
        float gain   = outRms / (inRms + 1e-9f);
        check("DmmOutputBuffer unity gain", gain, 1.0f, 0.10f);
    }

    std::printf("\n%d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
#endif // DMM_CIRCUITS_SELF_TEST
