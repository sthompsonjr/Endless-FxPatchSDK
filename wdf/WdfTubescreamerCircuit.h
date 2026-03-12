#pragma once

#include "WdfNonlinear.h"
#include "WdfOpAmpJRC4558.h"
#include "../dsp/OnePoleFilter.h"
#include "../dsp/ParameterSmoother.h"
#include "../dsp/Saturation.h"
#include <algorithm>
#include <cmath>

/// TS808Circuit — Ibanez TS808 Tubescreamer overdrive circuit model.
///
/// Signal flow:
///   input
///     → [drive/tone/level parameter smoothers]
///     → × kInputScale (0.1 — normalized to ~100mV circuit level)
///     → [OnePoleFilter highpass, fc=720 Hz, fixed]   ← Cin+Rin input coupling
///     → [bilinear IIR inverting gain, drive-dependent] ← H(s)=-(Rf/Rin)/(1+s·Cf·Rf)
///     → [WdfAntiparallelDiodes, 1N914 silicon]        ← Lambert W soft clipping
///     → [WdfOpAmpJRC4558 slew + rail clamping]
///     → × kOutputScale (1.5)
///     → [OnePoleFilter DCBlock, 10 Hz]
///     → [OnePoleFilter lowpass tone filter, 500–5000 Hz]
///     → × level
///     → sat::softClip()
///     → output
///
/// The input highpass at 720 Hz (Rin=4.7kΩ, Cin=0.047µF) is responsible
/// for the Tubescreamer's structural "mid-hump": bass below 720 Hz is
/// attenuated before clipping, so the clipper acts mainly on midrange
/// energy. This is NOT an EQ choice — it is a circuit topology effect.
///
/// The feedback capacitor Cf=0.047µF creates a lowpass in the closed-loop
/// gain: higher frequencies see less gain and therefore less clipping.
/// Combined with the input highpass, this creates a bandpass clipping
/// region centred around 700 Hz–3 kHz.
class TS808Circuit {
public:
    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;

        opAmp_.init(sampleRate);

        // Input highpass: Rin=4.7kΩ, Cin=0.047µF → fc ≈ 720 Hz
        inputHP_.init(sampleRate);
        inputHP_.setType(OnePoleFilter::Type::Highpass);
        inputHP_.setFrequency(1.0f / (6.283185307f * kRin * kCin));

        dcBlocker_.init(sampleRate);
        dcBlocker_.setType(OnePoleFilter::Type::DCBlock);

        toneFilter_.init(sampleRate);
        toneFilter_.setType(OnePoleFilter::Type::Lowpass);
        toneFilter_.setFrequency(1500.0f); // default mid position

        driveSmoother_.init(sampleRate, 20.0f);
        driveSmoother_.snapTo(0.5f);
        toneSmoother_.init(sampleRate, 20.0f);
        toneSmoother_.snapTo(0.5f);
        levelSmoother_.init(sampleRate, 20.0f);
        levelSmoother_.snapTo(0.5f);

        currentRf_ = driveToRf(0.5f);
        rebuildFeedbackNetwork(currentRf_);

        iirState_ = 0.0f;
        xPrev_    = 0.0f;
    }

    void setDrive(float drive) noexcept {
        driveSmoother_.setTarget(std::clamp(drive, 0.0f, 1.0f));
    }

    void setTone(float tone) noexcept {
        toneSmoother_.setTarget(std::clamp(tone, 0.0f, 1.0f));
    }

    void setLevel(float level) noexcept {
        levelSmoother_.setTarget(std::clamp(level, 0.0f, 1.0f));
    }

    /// Expose op-amp for creative controls (age, character, slew).
    WdfOpAmpJRC4558& opAmp() noexcept { return opAmp_; }

    /// Override input highpass frequency (clarity control: 200–1500 Hz).
    void setClarityFreq(float hz) noexcept {
        inputHP_.setFrequency(std::clamp(hz, 20.0f, sampleRate_ * 0.45f));
    }

    [[nodiscard]] float process(float input) noexcept {
        // Advance smoothers
        float drive = driveSmoother_.process();
        float tone  = toneSmoother_.process();
        float level = levelSmoother_.process();

        // Rebuild feedback network when drive changes significantly (>0.1%)
        float newRf = driveToRf(drive);
        if (fabsf(newRf - currentRf_) > currentRf_ * 0.001f) {
            currentRf_ = newRf;
            rebuildFeedbackNetwork(currentRf_);
        }

        // Update tone filter frequency: 500 Hz (tone=0) to 5 kHz (tone=1), log
        float toneFc = 500.0f * powf(10.0f, tone);
        toneFilter_.setFrequency(std::min(toneFc, sampleRate_ * 0.45f));

        // Scale to circuit level (~100mV)
        float x = input * kInputScale;

        // Input highpass: removes bass below ~720 Hz (structural mid-hump)
        x = inputHP_.process(x);

        // Closed-loop IIR: bilinear transform of H(s) = -(Rf/Rin)/(1+s·Cf·Rf)
        // H(z) = iirA0_ * (x + xPrev_) + iirB1_ * iirState_
        float vIdeal = iirA0_ * (x + xPrev_) + iirB1_ * iirState_;
        xPrev_    = x;
        iirState_ = vIdeal;

        // Antiparallel diode soft clipping (1N914 silicon, symmetric)
        diodes_.port.a = 2.0f * vIdeal;
        diodes_.reflect();
        float vClipped = diodes_.port.voltage();

        // JRC4558 op-amp: slew rate + rail clamping
        float vOut = opAmp_.process(-vClipped);

        // Scale to normalised range
        vOut *= kOutputScale;

        // DC blocking
        vOut = dcBlocker_.process(vOut);

        // Tone filter (post-clipping lowpass shelving)
        vOut = toneFilter_.process(vOut);

        // Level
        vOut *= level;

        // Final soft clip
        vOut = sat::softClip(vOut);

        return vOut;
    }

    void reset() noexcept {
        opAmp_.reset();
        diodes_.reset();
        inputHP_.reset();
        dcBlocker_.reset();
        toneFilter_.reset();
        iirState_ = 0.0f;
        xPrev_    = 0.0f;
    }

private:
    void rebuildFeedbackNetwork(float Rf) noexcept {
        float gain  = Rf / kRin;
        float tau   = Rf * kCf;
        float beta  = 2.0f * sampleRate_ * tau;
        float inv   = 1.0f / (1.0f + beta);
        iirA0_      = -gain * inv;
        iirB1_      = (beta - 1.0f) * inv;
        diodes_.init(kDiodeIs, kDiodeVt, Rf);
    }

    [[nodiscard]] float driveToRf(float drive) const noexcept {
        // Log taper: drive=0 → kRfMin (1kΩ), drive=1 → kRfMax (500kΩ)
        return kRfMin * expf(logf(kRfMax / kRfMin) * drive);
    }

    // Circuit constants (TS808 schematic values)
    static constexpr float kRin    = 4700.0f;    // input resistor
    static constexpr float kCin    = 0.047e-6f;  // input coupling cap → fc=720 Hz
    static constexpr float kCf     = 0.047e-6f;  // feedback cap
    static constexpr float kRfMin  = 1000.0f;    // min feedback R (drive=0)
    static constexpr float kRfMax  = 500000.0f;  // max feedback R (drive=1)
    static constexpr float kDiodeIs = 1e-7f;     // 1N914 silicon saturation current
    static constexpr float kDiodeVt = 0.02585f;  // thermal voltage at room temp
    static constexpr float kInputScale  = 0.1f;  // normalised → ~100mV circuit level
    static constexpr float kOutputScale = 1.5f;  // ±0.6V clip → ~±0.9V norm

    WdfOpAmpJRC4558      opAmp_;
    WdfAntiparallelDiodes diodes_;
    OnePoleFilter        inputHP_;
    OnePoleFilter        toneFilter_;
    OnePoleFilter        dcBlocker_;
    ParameterSmoother    driveSmoother_;
    ParameterSmoother    toneSmoother_;
    ParameterSmoother    levelSmoother_;

    float sampleRate_  = 48000.0f;
    float iirA0_       = 0.0f;
    float iirB1_       = 0.0f;
    float iirState_    = 0.0f;
    float xPrev_       = 0.0f;
    float currentRf_   = 10000.0f;
};

// ============================================================

/// TS9Circuit — Ibanez TS9 Tubescreamer variant.
///
/// The TS9 differs from the TS808 primarily in its output buffer:
///   - TS808: unity-gain op-amp buffer (smooth, slightly darker)
///   - TS9:   transistor emitter-follower buffer (slightly brighter,
///             more forward-sounding due to HF shelf from lower Rout)
///
/// This class wraps TS808Circuit and adds a post-processing stage
/// that models the output buffer difference. Rather than a full
/// WdfNpnBjt simulation (expensive), we use a simple HF shelf
/// approximation that captures the tonal character within budget.
class TS9Circuit {
public:
    enum class OutputBuffer {
        TS808_OpAmp,      ///< Original TS808 op-amp unity-gain buffer
        TS9_Transistor,   ///< TS9 transistor emitter-follower buffer
    };

    void init(float sampleRate, OutputBuffer bufType = OutputBuffer::TS9_Transistor) noexcept {
        sampleRate_  = sampleRate;
        bufferType_  = bufType;
        core_.init(sampleRate);

        // TS9 transistor shelf: highpass at 8 kHz, 6% blend
        ts9Shelf_.init(sampleRate);
        ts9Shelf_.setType(OnePoleFilter::Type::Highpass);
        ts9Shelf_.setFrequency(8000.0f);
    }

    void setDrive(float drive) noexcept  { core_.setDrive(drive); }
    void setTone(float tone) noexcept    { core_.setTone(tone); }
    void setLevel(float level) noexcept  { core_.setLevel(level); }
    void setOutputBuffer(OutputBuffer t) noexcept { bufferType_ = t; }
    WdfOpAmpJRC4558& opAmp() noexcept   { return core_.opAmp(); }
    void setClarityFreq(float hz) noexcept { core_.setClarityFreq(hz); }

    [[nodiscard]] float process(float input) noexcept {
        float out = core_.process(input);

        if (bufferType_ == OutputBuffer::TS9_Transistor) {
            // Transistor emitter-follower: slight HF emphasis
            // Mix 6% of high-passed signal for a forward-sounding character
            out = out + 0.06f * ts9Shelf_.process(out);
            out = sat::softClip(out);
        }
        // TS808_OpAmp: pass through unchanged (op-amp buffer is flat)

        return out;
    }

    void reset() noexcept {
        core_.reset();
        ts9Shelf_.reset();
    }

private:
    float         sampleRate_ = 48000.0f;
    OutputBuffer  bufferType_ = OutputBuffer::TS9_Transistor;
    TS808Circuit  core_;
    OnePoleFilter ts9Shelf_;
};

// ============================================================

/// KlonClipStage — Klon Centaur asymmetric clipping stage.
///
/// The Klon uses a pair of mismatched germanium-parameter diodes in
/// anti-series (one in each direction but with different characteristics),
/// producing asymmetric soft clipping. This generates warm even-harmonic
/// content in addition to odd harmonics, contributing to the Klon's
/// "transparent" character at lower gain settings.
///
/// Signal flow:
///   input
///     → × kInputScale
///     → [bilinear IIR inverting gain, Rf=gainRf_]
///     → [asymmetric clipping: diodeHigh + diodeLow (mismatched Is)]
///     → [WdfOpAmpJRC4558 slew + rail clamping]
///     → × kOutputScale
///     → [OnePoleFilter DCBlock]
///     → sat::softClip()
///     → output
///
/// Asymmetry: diodeHigh has Is=1e-7 (silicon-ish threshold),
///            diodeLow  has Is=0.7e-7 (slightly lower → different threshold).
/// The mismatch creates different positive/negative clip points, producing
/// even-harmonic distortion that sits differently in a mix.
class KlonClipStage {
public:
    void init(float sampleRate) noexcept {
        sampleRate_ = sampleRate;
        opAmp_.init(sampleRate);

        dcBlocker_.init(sampleRate);
        dcBlocker_.setType(OnePoleFilter::Type::DCBlock);

        gainSmoother_.init(sampleRate, 20.0f);
        gainSmoother_.snapTo(0.5f);

        currentRf_ = gainToRf(0.5f);
        rebuildGainNetwork(currentRf_);

        iirState_ = 0.0f;
        xPrev_    = 0.0f;
    }

    void setGain(float gain) noexcept {
        gainSmoother_.setTarget(std::clamp(gain, 0.0f, 1.0f));
    }

    WdfOpAmpJRC4558& opAmp() noexcept { return opAmp_; }

    [[nodiscard]] float process(float input) noexcept {
        float gain = gainSmoother_.process();

        float newRf = gainToRf(gain);
        if (fabsf(newRf - currentRf_) > currentRf_ * 0.001f) {
            currentRf_ = newRf;
            rebuildGainNetwork(currentRf_);
        }

        float x = input * kInputScale;

        // Closed-loop IIR inverting gain
        float vIdeal = iirA0_ * (x + xPrev_) + iirB1_ * iirState_;
        xPrev_    = x;
        iirState_ = vIdeal;

        // Asymmetric clipping: two WdfIdealDiode instances with different Is
        // Positive half: diodeHigh clips when vIdeal > 0
        diodeHigh_.port.a = 2.0f * vIdeal;
        diodeHigh_.reflect();
        float vPos = diodeHigh_.port.voltage();

        // Negative half: diodeLow clips when vIdeal < 0 (polarity flipped)
        diodeLow_.port.a = -2.0f * vIdeal;
        diodeLow_.reflect();
        float vNeg = -diodeLow_.port.voltage();

        // Composite: take whichever clamp is tighter for each polarity
        float vClipped;
        if (vIdeal >= 0.0f) {
            vClipped = vPos;
        } else {
            vClipped = vNeg;
        }

        // JRC4558 op-amp
        float vOut = opAmp_.process(-vClipped);

        vOut *= kOutputScale;
        vOut = dcBlocker_.process(vOut);
        vOut = sat::softClip(vOut);

        return vOut;
    }

    void reset() noexcept {
        opAmp_.reset();
        diodeHigh_.reset();
        diodeLow_.reset();
        dcBlocker_.reset();
        iirState_ = 0.0f;
        xPrev_    = 0.0f;
    }

private:
    void rebuildGainNetwork(float Rf) noexcept {
        float gain  = Rf / kRin;
        float tau   = Rf * kCf;
        float beta  = 2.0f * sampleRate_ * tau;
        float inv   = 1.0f / (1.0f + beta);
        iirA0_      = -gain * inv;
        iirB1_      = (beta - 1.0f) * inv;

        // Mismatched diodes: slightly different Is → asymmetric clip thresholds
        diodeHigh_.init(1e-7f,   kDiodeVt, Rf);  // higher Is → clips sooner
        diodeLow_.init(0.7e-7f,  kDiodeVt, Rf);  // lower  Is → clips later
    }

    [[nodiscard]] float gainToRf(float gain) const noexcept {
        return kRfMin * expf(logf(kRfMax / kRfMin) * gain);
    }

    static constexpr float kRin         = 10000.0f;   // input resistor
    static constexpr float kCf          = 0.047e-6f;  // feedback cap
    static constexpr float kRfMin       = 1000.0f;    // gain=0
    static constexpr float kRfMax       = 100000.0f;  // gain=1
    static constexpr float kDiodeVt     = 0.026f;     // thermal voltage
    static constexpr float kInputScale  = 0.1f;
    static constexpr float kOutputScale = 2.0f;

    WdfOpAmpJRC4558 opAmp_;
    WdfIdealDiode   diodeHigh_;
    WdfIdealDiode   diodeLow_;
    OnePoleFilter   dcBlocker_;
    ParameterSmoother gainSmoother_;

    float sampleRate_ = 48000.0f;
    float iirA0_      = 0.0f;
    float iirB1_      = 0.0f;
    float iirState_   = 0.0f;
    float xPrev_      = 0.0f;
    float currentRf_  = 10000.0f;
};
