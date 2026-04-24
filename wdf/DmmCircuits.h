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
//   DmmFeedbackEq       ~300 cycles  (4 WDF leaves, 3 adaptors, no op-amp)
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
// Schematic: Vin → C14(1µF) → R31(1MΩ) → Node_N2 → [C15(220nF) || R36(240kΩ)]
//
// WDF tree:
//   Series(C14, Series(R31, Parallel(C15, R36)))
//
// Transfer function (approx):
//   HPF (C14+R31): f_H ≈ 1/(2π·1e6·1e-6) ≈ 0.16 Hz (DC block, effectively flat)
//   LPF (C15||R36): f_L ≈ 1/(2π·240k·220n) ≈ 3.0 Hz  → not a true LPF at audio
//   The actual EQ behavior is a complex bandpass/shelving shape; the WDF
//   accurately models the exact RC network without approximation.
//
// Output: V(N2) = voltage at Node_N2 (junction of R31 and C15||R36)
//
// Runaway guard: kRunawayGain = 0.92f — caller multiplies feedback signal
//   by this constant to prevent self-oscillation at high feedback settings.
//   Self-oscillation onset measured at ~80-85% pot travel on reference unit.
//
// Port resistances (at 48kHz):
//   C14:  Rp = 1/(2·1e-6·48000) = 10.417 Ω
//   R31:  Rp = 1,000,000 Ω
//   C15:  Rp = 1/(2·220e-9·48000) = 47.35 Ω
//   R36:  Rp = 240,000 Ω
//   N2Par: Rp = C15||R36 = 47.26 Ω
//   N1Ser: Rp = R31 + N2Par = 1,000,047 Ω
//   EqTree root: Rp = C14 + N1Ser = 1,000,057 Ω
// ---------------------------------------------------------------------------
struct DmmFeedbackEq {
    using N2Par  = WdfParallelAdaptor2<WdfCapacitor, WdfResistor>;
    using N1Ser  = WdfSeriesAdaptor2<WdfResistor, N2Par>;
    using EqTree = WdfSeriesAdaptor2<WdfCapacitor, N1Ser>;

    // Multiply feedback signal by this constant before passing to process()
    // to prevent self-oscillation at high feedback settings.
    static constexpr float kRunawayGain = 0.92f;

    void init(float sampleRate) noexcept {
        root_.childA.init(1e-6f, sampleRate);              // C14
        root_.childB.childA.init(1e6f);                    // R31
        root_.childB.childB.childA.init(220e-9f, sampleRate);  // C15
        root_.childB.childB.childB.init(240e3f);           // R36
        root_.childB.childB.updatePortResistance();
        root_.childB.updatePortResistance();
        root_.updatePortResistance();
    }

    float process(float vin) noexcept {
        root_.reflect();
        root_.port.a = 2.0f * vin - root_.port.b;
        root_.scatter();
        root_.childB.scatter();
        root_.childB.childB.scatter();  // update C15.port.a so its state evolves
        return root_.childB.childB.port.voltage();
    }

    void reset() noexcept {
        root_.reset();
    }

private:
    EqTree root_;
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

    // --- DmmFeedbackEq: mid-band attenuation at 1 kHz ≈ 0.24× (-12.4 dB) ---
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
        check("DmmFeedbackEq attenuation @1kHz", gain, 0.24f, 0.15f);
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
