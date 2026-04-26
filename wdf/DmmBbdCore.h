#pragma once
// EH-7850 BBD Core — BBDLine + AnalogLfo wrapper
// Component source: EH-7850 SPICE/component reference (project upload)
//   Section 4: MN3005 × 2 cascaded, 8192 stages, clock 4–50 kHz
//   Section B: CD4047 VCO, AnalogLfo CEM3340 emulation
// Holters & Parker, DAFx-18 (2018) — canonical BBD VA modeling reference
// Raffel & Smith, DAFx-10 (2010) — practical BBD modeling
//
// CYCLE BUDGET (528 MHz / 48 kHz = 11,000 cycles/sample available):
//   BBDLine<32768>: ~150–250 cycles/sample
//   AnalogLfo:      ~50–100 cycles/sample
//   ParameterSmoother (delay knob): ~15–25 cycles/sample
//   Total DmmBbdCore: ~215–375 cycles/sample (2.0–3.4% of budget)
//
// DELAY DERIVATION (EH-7850, Section 4):
//   N_stages = 8192 (two MN3005 × 4096 stages cascaded)
//   T_delay  = N_stages / (2 × f_clock) = 4096 / f_clock
//   Clock range: 4 kHz – 50 kHz
//     T_min = 4096 / 50000 = 81.92 ms → 82.0 ms (rounded up)
//     T_max = 550.0 ms (bias headroom limit per EH-7850 reference)
//   Knob [0,1] → T_delay_ms = 82.0 + knob × 468.0  (linear in time)
//   T_delay_samples = T_delay_ms × (48000 / 1000)
//     knob = 0 → 3936 samples, knob = 1 → 26400 samples
//   Max modulated delay (vibrato, knob = 1): 26400 + 720 = 27120 < 32766 — in range
//
// BBD COMPANDER DECISION:
//   BBDLine::setCompanderAmount(0.0f): disabled here because DmmCompander
//   (session 2a) handles companding externally. Enabling both would
//   double-compress and double-expand the signal, altering the intended
//   dynamic character of the EH-7850.
//
// CLOCK NOISE:
//   setClockNoiseLevel(0.15f): emulates CD4047 clock bleed into the EH-7850
//   audio path. The EH-7850 is known for audible clock noise at higher delay
//   settings; 0.15 matches the bleed characteristic from the SPICE reference.
//
// BUFFER NOTE:
//   BBDLine<32768>::init() does not accept an external buffer pointer — it
//   manages CircularBuffer<float,32768> internally (128 KB). When DmmBbdCore
//   is instantiated as a file-scope static (e.g. static PatchImpl patch;),
//   this buffer resides in BSS, satisfying the no-heap, no-stack constraint.
//   The workingBuffer/bufferSize parameters in init() are accepted for API
//   symmetry with other session components but are unused by BBDLine itself.
//
// LFO NOTE:
//   AnalogLfo::process() returns bipolar [-1, 1] — used directly.
//   No centering needed: delaySamp = base + lfoOut * depth.
//   processUnipolar() ([0,1]) is a separate method and is not used here.

#include "dsp/BBDLine.h"
#include "dsp/AnalogLfo.h"
#include "dsp/ParameterSmoother.h"
#include <algorithm>
#include <cstddef>

struct DmmBbdCore {
    BBDLine<32768>    bbd;
    AnalogLfo         lfo;
    ParameterSmoother delaySmooth;

    float baseDelaySamples = 3936.0f;  // 82 ms × 48 = default minimum
    bool  isChorusMode     = true;

    static constexpr float kDelayMinMs       = 82.0f;
    static constexpr float kDelayMaxMs       = 550.0f;
    static constexpr float kDelayRangeMs     = kDelayMaxMs - kDelayMinMs;
    static constexpr float kSampleRate       = 48000.0f;
    static constexpr float kChorusDepthSamp  = 240.0f;
    static constexpr float kVibratoDepthSamp = 720.0f;
    static constexpr float kChorusLfoHz      = 3.0f;
    static constexpr float kVibratoLfoHz     = 6.0f;
    static constexpr float kClockNoise       = 0.15f;
    static constexpr float kCompanderAmt     = 0.0f;  // disabled; DmmCompander handles it

    // workingBuffer/bufferSize unused — BBDLine owns CircularBuffer<float,32768> internally
    void init(float sampleRate, float* /*workingBuffer*/, size_t /*bufferSize*/) noexcept {
        bbd.init(sampleRate);
        bbd.setCompanderAmount(kCompanderAmt);
        bbd.setClockNoiseLevel(kClockNoise);

        lfo.init(sampleRate);
        lfo.setFrequency(kChorusLfoHz);
        lfo.setDriftAmount(0.003f);
        lfo.setDriftRate(0.1f);
        lfo.setJitterAmount(0.0005f);  // setJitterAmount() confirmed present in AnalogLfo.h

        delaySmooth.init(sampleRate, 50.0f);
        delaySmooth.snapTo(kDelayMinMs * kSampleRate / 1000.0f);
    }

    void setDelayKnob(float v) noexcept {
        float t_ms   = kDelayMinMs + v * kDelayRangeMs;
        float samples = t_ms * (kSampleRate / 1000.0f);
        delaySmooth.setTarget(samples);
    }

    void setChorusMode(bool chorus) noexcept {
        isChorusMode = chorus;
        lfo.setFrequency(chorus ? kChorusLfoHz : kVibratoLfoHz);
        // LFO phase not reset on mode change — avoids discontinuity click
    }

    [[nodiscard]] float process(float x) noexcept {
        baseDelaySamples = delaySmooth.process();

        // AnalogLfo::process() returns bipolar [-1, 1] — no centering needed
        float lfoOut = lfo.process();

        float depth    = isChorusMode ? kChorusDepthSamp : kVibratoDepthSamp;
        float delaySamp = baseDelaySamples + lfoOut * depth;
        delaySamp = std::clamp(delaySamp, 1.0f, 32767.0f);

        // setDelaySamples() takes float — no cast needed
        bbd.setDelaySamples(delaySamp);
        return bbd.process(x);
    }

    void reset() noexcept {
        bbd.reset();
        lfo.reset();  // AnalogLfo::reset() confirmed present — clears phase and drift phases
    }
};
