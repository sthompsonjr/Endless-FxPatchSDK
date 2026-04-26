#pragma once

// EH-7850 Deluxe Memory Man — Complete Signal Chain Assembly
// Component source: EH-7850 SPICE/component reference (project upload)
// Madbean Total Recall v1.2 BOM — primary component source
//
// DmmFeedbackEq NOTE: Original WDF tree (session 1) produced unity gain
// at all frequencies due to WdfResistor::b[n]=0 (Fettweis 1986, Table 1).
// Fixed in session 2d Part A with direct IIR biquad from bilinear
// transform of H(s) = 0.24s / (0.0528s² + 1.2928s + 1).
//
// References:
//   Holters & Parker, DAFx-18 (2018) — BBD VA modeling
//   Raffel & Smith, DAFx-10 (2010) — BBD DSP modeling
//   Werner, Smith, Abel, DAFx-15 (2015) — WDF Sallen-Key
//   Werner et al., EUSIPCO 2016 — op-amp WDF modeling
//   Fettweis (1986), Proc. IEEE 74(2) — foundational WDF theory
//   Yeh & Smith, DAFx-08 — WDF element table (resistor b[n]=0)
//   Välimäki et al., DAFX book (2011) §5.3 — one-sample loop delay
//
// SIGNAL CHAIN (per sample, in order):
//   x_in = x + feedbackLoop.getFeedbackContribution()  // one-sample loop delay
//   y = inputBuf.process(x_in)    // IC1a RC4558 non-inverting, Av≈3.14×, HPF 16 Hz
//   y = compressor.process(y)     // NE570 ch.1, 2:1, τ=100 ms
//   y = aaFilter.process(y)       // IC3a Sallen-Key LPF, fc≈2.1 kHz
//   y = bbdCore.process(y)        // MN3005×2 BBDLine<32768> + AnalogLfo
//   y = reconFilter.process(y)    // IC5a Sallen-Key LPF fc≈3.8 kHz + ×4.545 gain recovery
//   y = expander.process(y)       // NE570 ch.2, 1:2, τ=47 ms
//   feedbackLoop.update(y)        // darkens+inverts+clips; result used next sample
//   y = outputBuf.process(y)      // IC9b RC4558 unity-gain follower
//   return y                      // wet signal; PatchImpl mixes with dry
//
// ONE-SAMPLE LOOP DELAY: feedbackLoop.getFeedbackContribution() returns
// the state computed by the PREVIOUS sample's update() call. This is a
// one-sample loop delay, inaudible at 48 kHz. Standard digital delay
// modeling practice (Välimäki et al., DAFX 2011, §5.3).
//
// FULL EFFECT CYCLE BUDGET (528 MHz / 48 kHz = 11,000 cycles/sample):
//   DmmInputBuffer:          ~200–350 cycles
//   DmmCompressor:           ~150–300 cycles
//   DmmAntiAliasFilter:      ~200–350 cycles
//   DmmBbdCore (BBD + LFO):  ~215–375 cycles
//   DmmReconFilter:          ~200–350 cycles
//   DmmExpander:             ~150–300 cycles
//   DmmFeedbackLoop:
//     DmmFeedbackEq (IIR):   ~30–60  cycles  (biquad; replaces ~300-cycle WDF tree)
//     Gain + softClip:       ~20–40  cycles
//     ParameterSmoother:     ~15–25  cycles
//   DmmOutputBuffer:         ~80–120 cycles
//   ParameterSmoother (mode):~15–25  cycles
//   ─────────────────────────────────────────
//   TOTAL ESTIMATED:         ~1,275–2,295 cycles/sample
//   % of 11,000 budget:      11.6–20.9%
//   Hard fail threshold:     6,600 cycles (60%) — NOT REACHED
//   Headroom for PatchImpl:  >8,700 cycles/sample

#include "wdf/DmmCircuits.h"       // DmmInputBuffer, DmmAntiAliasFilter,
                                   // DmmReconFilter, DmmOutputBuffer,
                                   // DmmFeedbackEq (IIR biquad, fixed Part A)
#include "dsp/DmmCompander.h"      // DmmCompressor, DmmExpander
#include "wdf/DmmBbdCore.h"        // BBDLine<32768> + AnalogLfo
#include "wdf/DmmFeedbackLoop.h"   // feedback state + gain + EQ + polarity
#include "dsp/ParameterSmoother.h"

#include <cstddef>

struct DmmDelayCircuit {
    // Signal chain members — value-type, no heap
    DmmInputBuffer     inputBuf;
    DmmAntiAliasFilter aaFilter;
    DmmReconFilter     reconFilter;
    DmmOutputBuffer    outputBuf;
    DmmCompressor      compressor;
    DmmExpander        expander;
    DmmBbdCore         bbdCore;
    DmmFeedbackLoop    feedbackLoop;
    ParameterSmoother  modeSmooth;   // 10 ms smooth for chorus/vibrato transitions

    // init() in signal-chain order.
    // workingBuffer/bufferSize are passed to DmmBbdCore for API symmetry;
    // BBDLine<32768> owns its CircularBuffer internally (BSS allocation).
    void init(float sampleRate,
              float* workingBuffer,
              size_t bufferSize) noexcept {
        inputBuf.init(sampleRate);
        compressor.init(sampleRate);
        aaFilter.init(sampleRate);
        bbdCore.init(sampleRate, workingBuffer, bufferSize);
        reconFilter.init(sampleRate);
        expander.init(sampleRate);
        feedbackLoop.init(sampleRate);
        outputBuf.init(sampleRate);
        modeSmooth.init(sampleRate, 10.0f);
        modeSmooth.snapTo(0.0f);  // default: chorus mode
    }

    // process() — per-sample wet signal.
    // Caller is responsible for wet/dry mixing (session 3 PatchImpl).
    [[nodiscard]] float process(float x) noexcept {
        // One-sample loop delay: feedbackState from previous sample's update()
        const float x_in = x + feedbackLoop.getFeedbackContribution();

        float y = inputBuf.process(x_in);   // +~10 dB gain, HPF 16 Hz
        y = compressor.process(y);           // 2:1 NE570 ch.1, τ=100 ms
        y = aaFilter.process(y);             // Sallen-Key LPF, fc≈2.1 kHz
        y = bbdCore.process(y);              // BBDLine<32768> + modulated delay
        y = reconFilter.process(y);          // Sallen-Key LPF fc≈3.8 kHz, ×4.545 gain
        y = expander.process(y);             // 1:2 NE570 ch.2, τ=47 ms

        // Update feedback state for next sample (polarity inversion + EQ + clip inside)
        feedbackLoop.update(y);

        y = outputBuf.process(y);            // RC4558 unity-gain follower
        return y;
    }

    void reset() noexcept {
        inputBuf.reset();
        compressor.reset();
        aaFilter.reset();
        bbdCore.reset();
        reconFilter.reset();
        expander.reset();
        feedbackLoop.reset();
        outputBuf.reset();
        modeSmooth.snapTo(0.0f);
    }

    // --- Parameter setters ---

    // Delay time knob: v in [0,1] → 82–550 ms (linear in time)
    void setDelayKnob(float v) noexcept {
        bbdCore.setDelayKnob(v);
    }

    // Feedback knob: v in [0,1], log taper via squaring; cap at kRunawayGain
    void setFeedbackKnob(float v) noexcept {
        feedbackLoop.setFeedbackKnob(v);
    }

    // Mode knob: v < 0.5 → chorus (3 Hz LFO), v ≥ 0.5 → vibrato (6 Hz LFO).
    // A 10 ms smoother prevents click artefacts from abrupt LFO rate changes.
    // Vibrato dry mute is handled by PatchImpl (session 3), not here.
    void setModeKnob(float v) noexcept {
        modeSmooth.setTarget(v);
        const bool chorus = (modeSmooth.process() < 0.5f);
        bbdCore.setChorusMode(chorus);
    }

    // Runaway mode: clamps feedback gain to kRunawayGain (0.92) for self-oscillation
    void setRunawayMode(bool active) noexcept {
        feedbackLoop.setRunawayMode(active);
    }
};
