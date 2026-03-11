#pragma once

#include "WdfPnpBjt.h"
#include "WdfOnePort.h"
#include "WdfAdaptors.h"
#include "../dsp/ParameterSmoother.h"
#include <algorithm>
#include <cmath>

/// Dallas Rangemaster Treble Booster — PNP OC44 common-emitter gain stage.
///
/// Original circuit (1966): single OC44 germanium PNP transistor,
/// 5nF input coupling cap for treble boost, 9V supply rail.
///
/// Topology:
///   Input → C_input(5nF) + R_bias_eff(8.7kΩ) → BJT base
///   Emitter: R_emitter(3.9kΩ) to +9V, bypassed by C_emitter(47μF)
///   Collector: R_collector(3.3kΩ) to GND (output taken here)
///
/// Component values derived from original schematic:
///   R_bias1 = 68kΩ, R_bias2 = 10kΩ → Thevenin R_eff ≈ 8717Ω
///   C_input = 5nF (sets treble-cut corner ~3.6kHz)
///   R_emitter = 3.9kΩ, C_emitter = 47μF (emitter bypass)
///   R_collector = 3.3kΩ
class RangemasterCircuit {
public:
    void init(float sampleRate) {
        sampleRate_ = sampleRate;

        // Base input network: C_input + R_bias_eff in series
        constexpr float kRbias1 = 68000.0f;
        constexpr float kRbias2 = 10000.0f;
        constexpr float kCinput = 5e-9f;  // 5nF
        float rBiasEff = (kRbias1 * kRbias2) / (kRbias1 + kRbias2); // ≈8717Ω

        baseInput_.childA.init(kCinput, sampleRate);
        baseInput_.childB.init(rBiasEff);
        baseInput_.updatePortResistance();

        // Emitter network: R_emitter to +9V, C_emitter bypass cap in parallel
        constexpr float kRemit = 3900.0f;   // 3.9kΩ
        constexpr float kCemit = 47e-6f;    // 47μF emitter bypass
        emitter_.childA.init(kRemit);
        emitter_.childA.setVoltage(9.0f);   // +9V supply
        emitter_.childB.init(kCemit, sampleRate);
        emitter_.updatePortResistance();

        // Collector load to GND
        constexpr float kRcoll = 3300.0f;  // 3.3kΩ
        collector_.init(kRcoll);
        collector_.setVoltage(0.0f);

        // OC44 transistor with port resistances from network
        bjt_ = WdfPnpBjt::makeOC44(baseInput_.port.Rp,
                                    collector_.port.Rp,
                                    emitter_.port.Rp);

        // DC blocking highpass ~10Hz
        dcBlockCoeff_ = expf(-6.283185307f * 10.0f / sampleRate);
        dcBlockState_ = 0.0f;
        dcBlockPrev_  = 0.0f;

        outputGain_ = 5.0f;
        capValue_   = kCinput;
    }

    /// Set treble character via input cap value.
    /// level=1.0 → 5nF (treble boost, authentic Rangemaster sound)
    /// level=0.0 → 1nF (reduced treble emphasis, flatter response)
    void setInputCharacter(float level) {
        float newCap = 1e-9f + std::clamp(level, 0.0f, 1.0f) * (5e-9f - 1e-9f);
        if (fabsf(newCap - capValue_) < 1e-12f) return;
        capValue_ = newCap;
        baseInput_.childA.init(newCap, sampleRate_);
        baseInput_.updatePortResistance();
        // Reinit BJT with updated base port impedance
        bjt_.portB.Rp = baseInput_.port.Rp;
    }

    [[nodiscard]] float process(float input) noexcept {
        // Scale guitar input to realistic circuit voltage (~10mV)
        baseInput_.childB.setVoltage(input * 0.01f);

        // Bottom-up reflect
        baseInput_.childA.reflect();
        baseInput_.childB.reflect();
        baseInput_.reflect();

        collector_.reflect();

        emitter_.childA.reflect();
        emitter_.childB.reflect();
        emitter_.reflect();

        // Set BJT incident waves
        bjt_.portB.a = baseInput_.port.b;
        bjt_.portC.a = collector_.port.b;
        bjt_.portE.a = emitter_.port.b;

        // Solve BJT (2D Newton-Raphson)
        bjt_.reflect();

        // Top-down scatter
        baseInput_.port.a = bjt_.portB.b;
        baseInput_.scatter();

        collector_.port.a = bjt_.portC.b;

        emitter_.port.a = bjt_.portE.b;
        emitter_.scatter();

        // Collector voltage with DC blocking
        float Vc = bjt_.portC.voltage();
        float out = (1.0f - dcBlockCoeff_) * (Vc - dcBlockPrev_) + dcBlockCoeff_ * dcBlockState_;
        dcBlockState_ = out;
        dcBlockPrev_  = Vc;

        return std::clamp(out * outputGain_, -1.0f, 1.0f);
    }

    void setOutputGain(float g) noexcept { outputGain_ = std::max(g, 0.0f); }

    void reset() noexcept {
        baseInput_.reset();
        emitter_.reset();
        collector_.reset();
        bjt_.reset();
        dcBlockState_ = 0.0f;
        dcBlockPrev_  = 0.0f;
    }

    WdfPnpBjt& bjt() noexcept { return bjt_; }

private:
    WdfSeriesAdaptor2<WdfCapacitor, WdfResistiveVoltageSource> baseInput_;
    WdfParallelAdaptor2<WdfResistiveVoltageSource, WdfCapacitor> emitter_;
    WdfResistiveVoltageSource collector_;
    WdfPnpBjt bjt_;

    float sampleRate_   = 48000.0f;
    float outputGain_   = 5.0f;
    float capValue_     = 5e-9f;
    float dcBlockCoeff_ = 0.0f;
    float dcBlockState_ = 0.0f;
    float dcBlockPrev_  = 0.0f;
};

/// Dallas Arbiter Fuzz Face — two-stage AC128 PNP germanium fuzz.
///
/// Two cascaded common-emitter stages. C2 (inter-stage coupling cap) provides
/// a one-sample delay between stages, preventing algebraic loops.
///
/// Q1 topology:
///   Input → C1(2.2μF) + R_fuzz(variable 0–1kΩ) → Q1 base
///   Q1 emitter: R2(470Ω) to GND (+9V supply at emitter for PNP)
///   Q1 collector: R3(8.2kΩ) to GND (reference). Output to Q2 via C2.
///
/// Q2 topology:
///   Q1 collector → C2(20μF) + R1_bias(33kΩ) → Q2 base
///   Q2 emitter: small R to GND (modeled as 100Ω)
///   Q2 collector: R4(100kΩ) to GND. Volume pot on output.
class FuzzFaceCircuit {
public:
    void init(float sampleRate) {
        sampleRate_ = sampleRate;

        // Q1 base input: C1(2.2μF) in series with input source (100kΩ + fuzz pot)
        q1Base_.childA.init(2.2e-6f, sampleRate);
        q1Base_.childB.init(100000.0f + fuzzR_);
        q1Base_.updatePortResistance();

        // Q1 emitter: R2=470Ω to 9V (PNP emitter goes to supply)
        q1Emitter_.init(470.0f);
        q1Emitter_.setVoltage(9.0f);

        // Q1 collector: R3=8.2kΩ to GND
        q1Collector_.init(8200.0f);
        q1Collector_.setVoltage(0.0f);

        q1_ = WdfPnpBjt::makeAC128(q1Base_.port.Rp,
                                    q1Collector_.port.Rp,
                                    q1Emitter_.port.Rp);

        // Q2 base: C2(20μF) + R1_bias(33kΩ) — cap provides inter-stage delay
        q2Base_.childA.init(20e-6f, sampleRate);
        q2Base_.childB.init(33000.0f);
        q2Base_.updatePortResistance();

        // Q2 emitter: 100Ω to 9V (very small degeneration)
        q2Emitter_.init(100.0f);
        q2Emitter_.setVoltage(9.0f);

        // Q2 collector: R4=100kΩ to GND
        q2Collector_.init(100000.0f);
        q2Collector_.setVoltage(0.0f);

        q2_ = WdfPnpBjt::makeAC128(q2Base_.port.Rp,
                                    q2Collector_.port.Rp,
                                    q2Emitter_.port.Rp);

        fuzzSmoother_.init(sampleRate, 20.0f);
        fuzzSmoother_.snapTo(0.5f);
        volumeSmoother_.init(sampleRate, 20.0f);
        volumeSmoother_.snapTo(0.5f);
        fuzzR_ = 500.0f;

        dcBlockCoeff_ = expf(-6.283185307f * 10.0f / sampleRate);
        dcBlockState_ = 0.0f;
        dcBlockPrev_  = 0.0f;
    }

    void setFuzz(float fuzz) noexcept {
        fuzzSmoother_.setTarget(std::clamp(fuzz, 0.0f, 1.0f));
    }

    void setVolume(float vol) noexcept {
        volumeSmoother_.setTarget(std::clamp(vol, 0.0f, 1.0f));
    }

    [[nodiscard]] float process(float input) noexcept {
        // Update fuzz (resistance in Q1 base path)
        float newFuzzR = fuzzSmoother_.process() * 1000.0f;
        if (fabsf(newFuzzR - fuzzR_) > 10.0f) {
            fuzzR_ = newFuzzR;
            // Update Q1 base source impedance to include fuzz pot
            q1Base_.childB.init(100000.0f + fuzzR_);
            q1Base_.updatePortResistance();
            q1_.portB.Rp = q1Base_.port.Rp;
        }

        // --- Stage 1: Q1 ---
        q1Base_.childB.setVoltage(input * 0.01f);
        q1Base_.childA.reflect();
        q1Base_.childB.reflect();
        q1Base_.reflect();

        q1Collector_.reflect();
        q1Emitter_.reflect();

        q1_.portB.a = q1Base_.port.b;
        q1_.portC.a = q1Collector_.port.b;
        q1_.portE.a = q1Emitter_.port.b;
        q1_.reflect();

        q1Base_.port.a = q1_.portB.b;
        q1Base_.scatter();
        q1Collector_.port.a = q1_.portC.b;
        q1Emitter_.port.a = q1_.portE.b;

        // Read Q1 collector voltage for inter-stage coupling
        float Vc1 = q1_.portC.voltage();

        // --- Stage 2: Q2 (via C2 cap providing one-sample delay) ---
        q2Base_.childB.setVoltage(Vc1 * 0.1f); // inter-stage level scaling
        q2Base_.childA.reflect();
        q2Base_.childB.reflect();
        q2Base_.reflect();

        q2Collector_.reflect();
        q2Emitter_.reflect();

        q2_.portB.a = q2Base_.port.b;
        q2_.portC.a = q2Collector_.port.b;
        q2_.portE.a = q2Emitter_.port.b;
        q2_.reflect();

        q2Base_.port.a = q2_.portB.b;
        q2Base_.scatter();
        q2Collector_.port.a = q2_.portC.b;
        q2Emitter_.port.a = q2_.portE.b;

        float Vc2 = q2_.portC.voltage();

        // DC blocking
        float out = (1.0f - dcBlockCoeff_) * (Vc2 - dcBlockPrev_) + dcBlockCoeff_ * dcBlockState_;
        dcBlockState_ = out;
        dcBlockPrev_  = Vc2;

        float vol = volumeSmoother_.process();
        return std::clamp(out * vol * 10.0f, -1.0f, 1.0f);
    }

    void reset() noexcept {
        q1Base_.reset();
        q1Collector_.reset();
        q1Emitter_.reset();
        q1_.reset();
        q2Base_.reset();
        q2Collector_.reset();
        q2Emitter_.reset();
        q2_.reset();
        fuzzSmoother_.snapTo(0.5f);
        volumeSmoother_.snapTo(0.5f);
        fuzzR_ = 500.0f;
        dcBlockState_ = 0.0f;
        dcBlockPrev_  = 0.0f;
    }

    WdfPnpBjt& q1() noexcept { return q1_; }
    WdfPnpBjt& q2() noexcept { return q2_; }

private:
    // Q1 stage
    WdfSeriesAdaptor2<WdfCapacitor, WdfResistiveVoltageSource> q1Base_;
    WdfResistiveVoltageSource q1Emitter_;
    WdfResistiveVoltageSource q1Collector_;
    WdfPnpBjt q1_;

    // Q2 stage
    WdfSeriesAdaptor2<WdfCapacitor, WdfResistiveVoltageSource> q2Base_;
    WdfResistiveVoltageSource q2Emitter_;
    WdfResistiveVoltageSource q2Collector_;
    WdfPnpBjt q2_;

    // Unused members (kept for reference from init signature)
    WdfResistor q1BaseR_;           // fuzz pot element (Rp updated dynamically)
    WdfResistiveVoltageSource q1InputSrc_; // direct input source

    ParameterSmoother fuzzSmoother_;
    ParameterSmoother volumeSmoother_;
    float fuzzR_ = 500.0f;
    float sampleRate_ = 48000.0f;

    float dcBlockCoeff_ = 0.0f;
    float dcBlockState_ = 0.0f;
    float dcBlockPrev_  = 0.0f;
};

/// Sola Sound Tone Bender MkI — PNP OC75 common-emitter fuzz.
///
/// Similar to Rangemaster but with OC75 transistor and an Attack control
/// that varies emitter resistance (0–100kΩ), setting the gain/fuzz amount.
///
/// Component values:
///   C_input = 10nF, R_bias_eff ≈ 4.7kΩ
///   R_emitter_base = 4.7kΩ (to +9V), C_emitter = 50μF bypass
///   R_attack = 0–100kΩ additional emitter resistance
///   R_collector = 33kΩ to GND
class ToneBenderMk1Circuit {
public:
    void init(float sampleRate) {
        sampleRate_ = sampleRate;

        // Base input: 10nF + 4.7kΩ bias effective R
        constexpr float kCinput = 10e-9f;
        constexpr float kRbiasEff = 4700.0f;
        baseInput_.childA.init(kCinput, sampleRate);
        baseInput_.childB.init(kRbiasEff);
        baseInput_.updatePortResistance();

        // Emitter: 4.7kΩ to +9V, bypassed by 50μF
        constexpr float kRemit = 4700.0f;
        constexpr float kCemit = 50e-6f;
        attackR_ = 0.0f;
        emitter_.childA.init(kRemit + attackR_);
        emitter_.childA.setVoltage(9.0f);
        emitter_.childB.init(kCemit, sampleRate);
        emitter_.updatePortResistance();

        // Collector: 33kΩ to GND
        constexpr float kRcoll = 33000.0f;
        collector_.init(kRcoll);
        collector_.setVoltage(0.0f);

        bjt_ = WdfPnpBjt::makeOC75(baseInput_.port.Rp,
                                    collector_.port.Rp,
                                    emitter_.port.Rp);

        attackSmoother_.init(sampleRate, 20.0f);
        attackSmoother_.snapTo(0.0f);

        dcBlockCoeff_ = expf(-6.283185307f * 10.0f / sampleRate);
        dcBlockState_ = 0.0f;
        dcBlockPrev_  = 0.0f;
    }

    /// Attack control: 0.0 = minimum attack (no extra R), 1.0 = maximum (100kΩ).
    void setAttack(float attack) noexcept {
        attackSmoother_.setTarget(std::clamp(attack, 0.0f, 1.0f));
    }

    [[nodiscard]] float process(float input) noexcept {
        float newAttackR = attackSmoother_.process() * 100000.0f;
        if (fabsf(newAttackR - attackR_) > 100.0f) {
            attackR_ = newAttackR;
            constexpr float kRemit = 4700.0f;
            emitter_.childA.init(kRemit + attackR_);
            emitter_.childA.setVoltage(9.0f);
            emitter_.updatePortResistance();
            bjt_.portE.Rp = emitter_.port.Rp;
        }

        baseInput_.childB.setVoltage(input * 0.01f);
        baseInput_.childA.reflect();
        baseInput_.childB.reflect();
        baseInput_.reflect();

        collector_.reflect();

        emitter_.childA.reflect();
        emitter_.childB.reflect();
        emitter_.reflect();

        bjt_.portB.a = baseInput_.port.b;
        bjt_.portC.a = collector_.port.b;
        bjt_.portE.a = emitter_.port.b;
        bjt_.reflect();

        baseInput_.port.a = bjt_.portB.b;
        baseInput_.scatter();
        collector_.port.a = bjt_.portC.b;
        emitter_.port.a = bjt_.portE.b;
        emitter_.scatter();

        float Vc = bjt_.portC.voltage();
        float out = (1.0f - dcBlockCoeff_) * (Vc - dcBlockPrev_) + dcBlockCoeff_ * dcBlockState_;
        dcBlockState_ = out;
        dcBlockPrev_  = Vc;

        return std::clamp(out * 5.0f, -1.0f, 1.0f);
    }

    void reset() noexcept {
        baseInput_.reset();
        emitter_.reset();
        collector_.reset();
        bjt_.reset();
        attackSmoother_.snapTo(0.0f);
        attackR_ = 0.0f;
        dcBlockState_ = 0.0f;
        dcBlockPrev_  = 0.0f;
    }

    WdfPnpBjt& bjt() noexcept { return bjt_; }

private:
    WdfSeriesAdaptor2<WdfCapacitor, WdfResistiveVoltageSource> baseInput_;
    WdfParallelAdaptor2<WdfResistiveVoltageSource, WdfCapacitor> emitter_;
    WdfResistiveVoltageSource collector_;
    WdfPnpBjt bjt_;

    ParameterSmoother attackSmoother_;
    float attackR_ = 0.0f;
    float sampleRate_ = 48000.0f;

    float dcBlockCoeff_ = 0.0f;
    float dcBlockState_ = 0.0f;
    float dcBlockPrev_  = 0.0f;
};

/// GeBoostCircuit — parameterized PNP germanium boost.
///
/// Generic single-transistor PNP boost circuit driven by user-supplied
/// component values and transistor parameters. Topology mirrors Rangemaster.
class GeBoostCircuit {
public:
    struct ComponentValues {
        float Cinput   = 5e-9f;    ///< Input coupling cap [F]
        float Rbase    = 8700.0f;  ///< Base bias Thevenin resistance [Ω]
        float Remitter = 3900.0f;  ///< Emitter resistor [Ω]
        float Cemitter = 47e-6f;   ///< Emitter bypass cap [F]
        float Rcollect = 3300.0f;  ///< Collector load resistor [Ω]
        float Vsupply  = 9.0f;     ///< Supply voltage [V]
        float outputGain = 5.0f;   ///< Makeup gain applied to output
    };

    void init(const ComponentValues& cv,
              const WdfPnpBjt::Params& transistorParams,
              float sampleRate) {
        sampleRate_ = sampleRate;
        outputGain_ = cv.outputGain;

        baseInput_.childA.init(cv.Cinput, sampleRate);
        baseInput_.childB.init(cv.Rbase);
        baseInput_.updatePortResistance();

        emitter_.childA.init(cv.Remitter);
        emitter_.childA.setVoltage(cv.Vsupply);
        emitter_.childB.init(cv.Cemitter, sampleRate);
        emitter_.updatePortResistance();

        collector_.init(cv.Rcollect);
        collector_.setVoltage(0.0f);

        bjt_.init(transistorParams,
                  baseInput_.port.Rp,
                  collector_.port.Rp,
                  emitter_.port.Rp);

        dcBlockCoeff_ = expf(-6.283185307f * 10.0f / sampleRate);
        dcBlockState_ = 0.0f;
        dcBlockPrev_  = 0.0f;
    }

    [[nodiscard]] float process(float input) noexcept {
        baseInput_.childB.setVoltage(input * 0.01f);
        baseInput_.childA.reflect();
        baseInput_.childB.reflect();
        baseInput_.reflect();

        collector_.reflect();

        emitter_.childA.reflect();
        emitter_.childB.reflect();
        emitter_.reflect();

        bjt_.portB.a = baseInput_.port.b;
        bjt_.portC.a = collector_.port.b;
        bjt_.portE.a = emitter_.port.b;
        bjt_.reflect();

        baseInput_.port.a = bjt_.portB.b;
        baseInput_.scatter();
        collector_.port.a = bjt_.portC.b;
        emitter_.port.a = bjt_.portE.b;
        emitter_.scatter();

        float Vc = bjt_.portC.voltage();
        float out = (1.0f - dcBlockCoeff_) * (Vc - dcBlockPrev_) + dcBlockCoeff_ * dcBlockState_;
        dcBlockState_ = out;
        dcBlockPrev_  = Vc;

        return std::clamp(out * outputGain_, -1.0f, 1.0f);
    }

    void reset() noexcept {
        baseInput_.reset();
        emitter_.reset();
        collector_.reset();
        bjt_.reset();
        dcBlockState_ = 0.0f;
        dcBlockPrev_  = 0.0f;
    }

    WdfPnpBjt& bjt() noexcept { return bjt_; }

private:
    WdfSeriesAdaptor2<WdfCapacitor, WdfResistiveVoltageSource> baseInput_;
    WdfParallelAdaptor2<WdfResistiveVoltageSource, WdfCapacitor> emitter_;
    WdfResistiveVoltageSource collector_;
    WdfPnpBjt bjt_;

    float sampleRate_   = 48000.0f;
    float outputGain_   = 5.0f;
    float dcBlockCoeff_ = 0.0f;
    float dcBlockState_ = 0.0f;
    float dcBlockPrev_  = 0.0f;
};
