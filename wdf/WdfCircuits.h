#pragma once

#include "WdfOnePort.h"
#include "WdfAdaptors.h"
#include "WdfNonlinear.h"
#include "dsp/ParameterSmoother.h"
#include <algorithm>
#include <cmath>

/// RC Lowpass Filter — passive first-order lowpass via WDF bilinear transform.
///
/// Topology:
///   Vin (resistive source, Rs=1kΩ) --[R_series]-- Cap to ground
///   Output = voltage across capacitor
///
/// The WDF approach for this linear circuit reduces to an exact bilinear-transform
/// IIR filter. The transfer function H(s) = 1/(1 + s*Rt*C) where Rt = Rs + R
/// is discretized via the bilinear (Tustin) transform s = 2*fs*(z-1)/(z+1):
///
///   H(z) = g * (1 + z^-1) / (1 - c * z^-1)
///   where alpha = 2*fs*Rt*C, c = (alpha-1)/(alpha+1), g = 1/(alpha+1)
///
/// Cutoff: fc = 1 / (2π * Rt * C)
/// Parameters: R (100Ω–100kΩ), C (100pF–10μF)
class RCLowpassCircuit {
public:
    void init(float R, float C, float sampleRate) {
        sampleRate_ = sampleRate;
        C_ = C;
        computeCoeffs(R);
        prevX_ = 0.0f;
        prevY_ = 0.0f;
    }

    void setR(float R) {
        computeCoeffs(R);
    }

    [[nodiscard]] float process(float input) noexcept {
        // Bilinear-transform IIR: y[n] = g*(x[n] + x[n-1]) + c*y[n-1]
        float y = g_ * (input + prevX_) + c_ * prevY_;
        prevX_ = input;
        prevY_ = y;
        return y;
    }

    void reset() noexcept {
        prevX_ = 0.0f;
        prevY_ = 0.0f;
    }

private:
    void computeCoeffs(float R) {
        float Rt = 1000.0f + R; // source impedance (1kΩ) + series R
        float alpha = 2.0f * sampleRate_ * Rt * C_;
        g_ = 1.0f / (alpha + 1.0f);
        c_ = (alpha - 1.0f) / (alpha + 1.0f);
    }

    float g_ = 0.0f;
    float c_ = 0.0f;
    float C_ = 0.0f;
    float prevX_ = 0.0f;
    float prevY_ = 0.0f;
    float sampleRate_ = 48000.0f;
};

/// Diode Clipper Circuit — input resistor + antiparallel diodes.
///
/// Topology:
///   Vin (resistive source) --[series]-- R_input --[node]-- antiparallel diodes to ground
///   Output = voltage at the clipping node
///
/// This is the basis of: Tube Screamer clipping, MXR Distortion+, many OD pedals.
///
/// WDF tree (nonlinear element at root):
///   The diode pair is the "root" element — it sees the combined impedance of the
///   rest of the tree and solves analytically via Lambert W.
///
///   Tree:
///     Series adaptor (source, R_input)
///       ChildA = WdfResistiveVoltageSource
///       ChildB = WdfResistor (R_input)
///     The series adaptor's port connects to the antiparallel diodes.
///     The diodes' port resistance = series adaptor's Rp.
///
/// For a 1.0f input, output is approximately ±0.5V for silicon (≈ ±0.3V for germanium).
/// Apply makeup gain of ~3–5x for normalized output.
class DiodeClipperCircuit {
public:
    enum class DiodeType { Silicon, Germanium };

    void init(float R_input, DiodeType type, float sampleRate) {
        sampleRate_ = sampleRate;

        diodeIs_ = (type == DiodeType::Silicon) ? 1e-7f : 1e-6f;
        float Vt = 0.02585f;

        // Init components directly through the adaptor tree
        series_.childA.init(1000.0f); // 1kΩ source impedance
        series_.childB.init(R_input);
        series_.updatePortResistance();

        // Diodes see the series adaptor's port resistance
        diodes_.init(diodeIs_, Vt, series_.port.Rp);

        // Makeup gain: silicon clips at ~±0.6V, germanium at ~±0.3V
        makeupGain_ = (type == DiodeType::Silicon) ? 3.0f : 5.0f;
    }

    void setInputResistance(float R) {
        series_.childB.init(R);
        series_.updatePortResistance();
        // Re-init diodes with new port resistance
        diodes_.init(diodeIs_, 0.02585f, series_.port.Rp);
    }

    [[nodiscard]] float process(float input) noexcept {
        // Set input
        series_.childA.setVoltage(input);

        // Bottom-up reflect
        series_.childA.reflect();
        series_.childB.reflect();
        series_.reflect();

        // The series adaptor's reflected wave is the diode's incident wave
        diodes_.port.a = series_.port.b;

        // Diode solves analytically (Lambert W)
        diodes_.reflect();

        // The diode's reflected wave becomes the series adaptor's incident wave
        series_.port.a = diodes_.port.b;

        // Top-down scatter
        series_.scatter();

        // Output voltage at the diode node, with makeup gain
        float out = diodes_.port.voltage() * makeupGain_;
        return std::clamp(out, -1.0f, 1.0f);
    }

    void reset() noexcept {
        series_.reset();
        diodes_.reset();
    }

private:
    WdfSeriesAdaptor2<WdfResistiveVoltageSource, WdfResistor> series_;
    WdfAntiparallelDiodes diodes_;
    float sampleRate_ = 48000.0f;
    float diodeIs_ = 1e-7f;
    float makeupGain_ = 3.0f;
};

/// Tone Stack Circuit — passive Baxandall-style RC tone control.
///
/// Simplified topology: two cascaded RC sections.
///   Section 1 (treble): R_treble + C_treble (highpass)
///   Section 2 (bass):   R_bass + C_bass (lowpass)
///
/// The tone knob (0–1) crossfades between the two RC sections by varying
/// the effective resistances:
///   R_treble = tone * R_pot (less resistance = more treble passes)
///   R_bass = (1 - tone) * R_pot (less resistance = more bass passes)
///
/// Component values:
///   R_pot = 100kΩ (typical tone pot)
///   C_treble = 220pF (fc_treble ≈ 7.2kHz at center)
///   C_bass = 22nF (fc_bass ≈ 72Hz at center)
///
/// Uses two cascaded RC lowpass WDF stages.
class ToneStackCircuit {
public:
    void init(float sampleRate) {
        sampleRate_ = sampleRate;
        toneSmoother_.init(sampleRate, 20.0f);
        toneSmoother_.snapTo(0.5f);
        currentTone_ = 0.5f;
        updateComponents(0.5f);
    }

    void setTone(float tone) {
        toneSmoother_.setTarget(std::clamp(tone, 0.0f, 1.0f));
    }

    [[nodiscard]] float process(float input) noexcept {
        float tone = toneSmoother_.process();

        // Only rebuild components if tone changed significantly
        if (fabsf(tone - currentTone_) > 0.001f) {
            currentTone_ = tone;
            updateComponents(tone);
        }

        // Run through treble section (highpass behavior via series R)
        float mid = trebleSection_.process(input);

        // Run through bass section (lowpass behavior)
        return bassSection_.process(mid);
    }

    void reset() noexcept {
        trebleSection_.reset();
        bassSection_.reset();
        toneSmoother_.snapTo(0.5f);
        currentTone_ = 0.5f;
    }

private:
    void updateComponents(float tone) {
        // R_pot = 100kΩ
        constexpr float kRpot = 100000.0f;
        constexpr float kCtreble = 220e-12f;  // 220pF
        constexpr float kCbass = 22e-9f;       // 22nF

        // Both lowpass sections: lower R = higher cutoff = more highs pass.
        // tone=1 (bright): low R on both → high cutoffs → treble passes
        // tone=0 (dark):   high R on both → low cutoffs → treble cut
        // Different cap values give different frequency ranges.
        float rTreble = std::max((1.0f - tone) * kRpot, 100.0f);
        float rBass = std::max((1.0f - tone) * kRpot, 100.0f);

        trebleSection_.init(rTreble, kCtreble, sampleRate_);
        bassSection_.init(rBass, kCbass, sampleRate_);
    }

    RCLowpassCircuit trebleSection_;
    RCLowpassCircuit bassSection_;
    ParameterSmoother toneSmoother_;
    float sampleRate_ = 48000.0f;
    float currentTone_ = 0.5f;
};

/// BJT Gain Stage Circuit — NPN common-emitter amplifier.
///
/// Topology:
///   Vin → R_base (100kΩ) → BJT base
///   Collector: Vcc (9V) through R_load (10kΩ)
///   Emitter: R_emitter (1kΩ) to ground, bypassed by C_emitter
///
/// Models: Rangemaster gain stage, Fuzz Face first stage, any single-transistor
/// gain stage in guitar pedals.
///
/// Component values (germanium defaults):
///   R_base  = 100kΩ — sets input impedance
///   R_load  = 10kΩ  — collector load (sets voltage gain)
///   R_emit  = 1kΩ   — emitter degeneration (thermal stability)
///   C_emit  = 10μF  — emitter bypass (boosts AC gain)
///   hFE     = 70    — germanium typical (silicon: 200)
///   Is      = 1e-6  — germanium (silicon: 1e-12)
///   Vcc     = 9V    — standard pedal supply
///
/// WARNING: This is the most expensive circuit (~3000-5000 cycles/sample).
/// Limit to one per effect on the Cortex-M7 target.
class BJTGainStageCircuit {
public:
    enum class TransistorType { Germanium, Silicon };

    void init(TransistorType type, float sampleRate) {
        sampleRate_ = sampleRate;
        type_ = type;

        float Is, hFE;
        if (type == TransistorType::Germanium) {
            Is = 1e-6f;    // germanium saturation current
            hFE = 70.0f;   // typical germanium gain
        } else {
            Is = 1e-12f;   // silicon saturation current
            hFE = 200.0f;  // typical silicon gain
        }
        float Vt = 0.02585f;

        // Component values
        constexpr float kRbase = 100000.0f;   // 100kΩ base resistor
        constexpr float kRload = 10000.0f;    // 10kΩ collector load
        constexpr float kRemit = 1000.0f;     // 1kΩ emitter resistor
        constexpr float kVcc = 9.0f;          // 9V supply

        // Init components
        inputSrc_.init(kRbase);
        collectorSupply_.init(kRload);
        collectorSupply_.setVoltage(kVcc);
        emitterR_.init(kRemit);

        // BJT: port resistances from surrounding network
        bjt_.init(Is, Vt, hFE, kRbase, kRload, kRemit);

        // Bias point: set initial conditions for warm-start
        biasGain_ = 1.0f;
        outputDC_ = 0.0f;

        // DC blocking: simple one-pole highpass state
        dcBlockState_ = 0.0f;
        dcBlockPrev_ = 0.0f;
        // ~10 Hz highpass coefficient
        dcBlockCoeff_ = expf(-6.283185307f * 10.0f / sampleRate);
    }

    void setBias(float bias) {
        // Adjust base bias by scaling the input source resistance
        // bias 0.0 = high R (less drive), 1.0 = low R (more drive)
        float rBase = 200000.0f * (1.0f - bias * 0.8f); // 200k down to 40k
        inputSrc_.init(std::max(rBase, 1000.0f));
    }

    void setGain(float gain) {
        biasGain_ = 1.0f + gain * 4.0f; // 1x to 5x output scaling
    }

    [[nodiscard]] float process(float input) noexcept {
        // Set input voltage (scaled to realistic guitar signal level ~100mV)
        inputSrc_.setVoltage(input * 0.1f);

        // Reflect all sources
        inputSrc_.reflect();
        collectorSupply_.reflect();
        emitterR_.reflect();

        // Set BJT port incident waves from the surrounding network
        bjt_.portB.a = inputSrc_.port.b;
        bjt_.portC.a = collectorSupply_.port.b;
        bjt_.portE.a = emitterR_.port.b;

        // BJT solves (Newton-Raphson)
        bjt_.reflect();

        // Scatter back to sources
        inputSrc_.port.a = bjt_.portB.b;
        collectorSupply_.port.a = bjt_.portC.b;
        emitterR_.port.a = bjt_.portE.b;

        // Output = collector voltage, inverted and scaled
        float Vc = bjt_.portC.voltage();

        // DC blocking highpass
        float dcBlocked = (1.0f - dcBlockCoeff_) * (Vc - dcBlockPrev_) + dcBlockCoeff_ * dcBlockState_;
        dcBlockState_ = dcBlocked;
        dcBlockPrev_ = Vc;

        // Scale and clamp output
        float out = dcBlocked * biasGain_;
        return std::clamp(out, -1.0f, 1.0f);
    }

    void reset() noexcept {
        bjt_.reset();
        inputSrc_.reset();
        collectorSupply_.reset();
        emitterR_.reset();
        dcBlockState_ = 0.0f;
        dcBlockPrev_ = 0.0f;
    }

private:
    WdfNpnBjt bjt_;
    WdfResistiveVoltageSource inputSrc_;
    WdfResistiveVoltageSource collectorSupply_;
    WdfResistor emitterR_;

    TransistorType type_ = TransistorType::Germanium;
    float sampleRate_ = 48000.0f;
    float biasGain_ = 1.0f;
    float outputDC_ = 0.0f;

    // DC blocking
    float dcBlockState_ = 0.0f;
    float dcBlockPrev_ = 0.0f;
    float dcBlockCoeff_ = 0.0f;
};
