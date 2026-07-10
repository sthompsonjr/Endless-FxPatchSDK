#pragma once

// Boss DC-2 Dimension C — Linear filter and EQ stages (Session 1 of 6)
//
// Component source: reference/DC2_component_reference.md (value authority;
// transcribed from the Aion FX Blueshift redraw of the Boss DC-2 schematic).
// Every numeric constant below cites a designator from that document.
//
// CLASSIFICATION (see "Block classification" in the reference doc):
// All DC-2 audio-path filters are active RC networks around ideal buffers
// (2N5088 emitter followers, hFE >= 300) or ideal op-amps (nullor convention,
// Werner et al., EUSIPCO 2016). Purely resistive/RC divider trees are
// algebraically transparent in this WDF library (WdfResistor b[n] = 0,
// Fettweis 1986, Proc. IEEE 74(2); see the DmmFeedbackEq precedent in
// wdf/DmmCircuits.h) and MUST NOT be built as WDF. Consequently every block
// in this file is a bilinear-transform section (Yeh & Smith, DAFx-06
// discretization convention: s = 2*fs*(1 - z^-1)/(1 + z^-1), prewarped at
// each stage's analytically derived corner).
//
// SIGNAL CHAIN CONTEXT (mono in -> stereo out; BBD/compander in session 3):
//   in -> Dc2InputBuffer -> Dc2PreEmphasis -> [NE570 comp] -> Dc2AntiAliasLpf
//      -> split: BBD-C / BBD-B -> Dc2ReconLpf (x2) -> [NE570 expanders]
//      -> Dc2OutputMixer (x2, with cross-feedback between channels)
//   dry taps: pre-emphasis output -> mixers (R71/R92) and
//             Dc2DryLowpass -> mixers (R87/R88)
//
// POLARITY: Dc2PreEmphasis and Dc2OutputMixer invert (physical op-amp signs).
// Net dry and wet paths are both non-inverting overall; OUT_L/OUT_R in-phase.
//
// CYCLE BUDGET (528 MHz / 48 kHz = 11,000 cycles/sample; whole-patch
// WARN > 4,400, FAIL > 6,600):
//   Dc2InputBuffer      ~15-25   (1st-order DF2T)
//   Dc2PreEmphasis      ~15-25
//   Dc2AntiAliasLpf     ~30-60   (biquad DF2T)
//   Dc2ReconLpf (x2)    ~45-85 each (1st-order + biquad)
//   Dc2DryLowpass       ~15-25
//   Dc2OutputMixer (x2) ~60-100 each (4 MACs + biquad + 1st-order)
//   Session-1 total (stereo): ~290-505 cycles/sample -- ~3-5% of budget.
// Hot paths are branch-free; __builtin_expect not applicable (no branches).
// No per-sample transcendentals; WDF_USE_FAST_MATH not needed in this file.
//
// Desktop compile gate (single command line, wrapped here for readability):
//   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra
//       -DENDLESS_DESKTOP_BUILD -I. -I./sdk -I./wdf -I./dsp
//       tests/dc2_filters_test.cpp -lm -o /tmp/dc2_test

#include <cmath>

// ---------------------------------------------------------------------------
// Dc2Constants — every value traces to reference/DC2_component_reference.md
// by designator (sections cited per group).
// ---------------------------------------------------------------------------
struct Dc2Constants {
    // §1 Input buffer: R6 10k, C4 47n, R5 1M
    static constexpr float kR6  = 10e3f;
    static constexpr float kC4  = 47e-9f;
    static constexpr float kR5  = 1e6f;

    // §2 Pre-emphasis: R7 47k, R8 10k, C5 4n7, R9 100k, R10 33k
    //    (C7 1uF short / C8 100pF open in-band; see reference doc §"decisions")
    static constexpr float kR7  = 47e3f;
    static constexpr float kR8  = 10e3f;
    static constexpr float kC5  = 4.7e-9f;
    static constexpr float kR9  = 100e3f;
    static constexpr float kR10 = 33e3f;

    // §4/§9 Anti-alias & reconstruction LPF ladders (identical):
    //    R25/R26/R27 = R66/R65/R64 = R19/R18/R17 = 10k
    //    C22/C43/C25 = 470pF, C20/C44/C24 = 1n8, C21/C45/C23 = 220pF
    static constexpr float kRlad = 10e3f;
    static constexpr float kC22  = 470e-12f;
    static constexpr float kC20  = 1.8e-9f;
    static constexpr float kC21  = 220e-12f;

    // §9 Recon input coupling: C42/C26 22n into R67||R69 = R20||R21 = 680k||680k
    static constexpr float kC42   = 22e-9f;
    static constexpr float kRbias = 340e3f;   // 680k || 680k

    // §11 Dry LF buffer: R12 68k, C11 22n (R11 2k2 + Q2 base Z negligible)
    static constexpr float kR12 = 68e3f;
    static constexpr float kC11 = 22e-9f;

    // §12 Output mixers (identical values both sides; A-side designators):
    //    R71/R92 33k (dry), R87/R88 220k (dry LF), R82/R85 47k (wet)
    //    Feedback: R75/R76 47k, C51/C52 4n7, R74/R77 10k (C50/C53 47pF out-of-band)
    //    Cross-feedback bridged-T: R80/R81 180k, R84/R83 33k, C56/C57 1n2,
    //    C55/C58 3n3 to the OPPOSITE mixer output.
    static constexpr float kR71 = 33e3f;
    static constexpr float kR87 = 220e3f;
    static constexpr float kR82 = 47e3f;
    static constexpr float kR75 = 47e3f;
    static constexpr float kC51 = 4.7e-9f;
    static constexpr float kR74 = 10e3f;
    static constexpr float kR80 = 180e3f;
    static constexpr float kR84 = 33e3f;
    static constexpr float kC56 = 1.2e-9f;
    static constexpr float kC55 = 3.3e-9f;
};

// ---------------------------------------------------------------------------
// Shared bilinear-transform sections.
//
// First order:  H(s) = (n1*s + n0) / (d1*s + d0)
//   s = K*(1-z^-1)/(1+z^-1),  K = w_pre / tan(w_pre / (2*fs))  (prewarp)
//   b0 = (n1*K + n0)/A0, b1 = (n0 - n1*K)/A0, A0 = d1*K + d0,
//   a1 = (d0 - d1*K)/A0.
//
// Biquad:       H(s) = (n2*s^2 + n1*s + n0) / (d2*s^2 + d1*s + d0)
//   b0 = (n2*K^2 + n1*K + n0)/A0,  b1 = 2*(n0 - n2*K^2)/A0,
//   b2 = (n2*K^2 - n1*K + n0)/A0,  A0 = d2*K^2 + d1*K + d0,
//   a1 = 2*(d0 - d2*K^2)/A0,       a2 = (d2*K^2 - d1*K + d0)/A0.
//
// Both use Direct Form II Transposed (same structure as dsp/BiquadFilter.h;
// coefficients here come from arbitrary analog polynomials, which the
// parametric BiquadFilter types cannot express — hence local sections,
// following the DmmCircuits.h precedent).
// ---------------------------------------------------------------------------
struct Dc2FirstOrderSection {
    // [DTCM] per-sample state
    float s1 = 0.0f;
    // [CONTROL-RATE] coefficients, set by initAnalog()
    float b0 = 1.0f, b1 = 0.0f, a1 = 0.0f;

    void initAnalog(float n1, float n0, float d1, float d0,
                    float prewarpHz, float sampleRate) noexcept {
        const float w = 6.2831853f * prewarpHz;
        const float K = w / std::tan(w / (2.0f * sampleRate));
        const float A0 = d1 * K + d0;
        b0 = (n1 * K + n0) / A0;
        b1 = (n0 - n1 * K) / A0;
        a1 = (d0 - d1 * K) / A0;
        s1 = 0.0f;
    }

    [[nodiscard]] float process(float x) noexcept {
        const float y = b0 * x + s1;
        s1 = b1 * x - a1 * y;
        return y;
    }

    void reset() noexcept { s1 = 0.0f; }
};

struct Dc2BiquadSection {
    // [DTCM] per-sample state
    float s1 = 0.0f, s2 = 0.0f;
    // [CONTROL-RATE] coefficients, set by initAnalog()
    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;

    void initAnalog(float n2, float n1, float n0,
                    float d2, float d1, float d0,
                    float prewarpHz, float sampleRate) noexcept {
        const float w  = 6.2831853f * prewarpHz;
        const float K  = w / std::tan(w / (2.0f * sampleRate));
        const float K2 = K * K;
        const float A0 = d2 * K2 + d1 * K + d0;
        b0 = (n2 * K2 + n1 * K + n0) / A0;
        b1 = 2.0f * (n0 - n2 * K2) / A0;
        b2 = (n2 * K2 - n1 * K + n0) / A0;
        a1 = 2.0f * (d0 - d2 * K2) / A0;
        a2 = (d2 * K2 - d1 * K + d0) / A0;
        s1 = 0.0f;  s2 = 0.0f;
    }

    [[nodiscard]] float process(float x) noexcept {
        const float y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
        return y;
    }

    void reset() noexcept { s1 = 0.0f;  s2 = 0.0f; }
};

// ---------------------------------------------------------------------------
// Block 1: Dc2InputBuffer — IC1A OPA2134 unity follower with input HPF.
//
// Netlist (ref doc §1): IN -> R6 10k -> C4 47n -> node{R5 1M -> VR} -> IC1A(+),
// IC1A out -> IC1A(-) (unity).
//
// H(s) derivation (ideal op-amp, voltage divider):
//   H(s) = R5 / (R6 + 1/(s*C4) + R5)
//        = k * s*tau / (1 + s*tau),   tau = C4*(R5+R6),  k = R5/(R5+R6)
//   tau = 47e-9 * 1.01e6 = 47.47 ms  ->  fc = 1/(2*pi*tau) = 3.353 Hz
//   k   = 1e6/1.01e6 = 0.990099
// Analog polys: N = (k*tau)s + 0,  D = tau*s + 1.  Prewarp at fc (negligible
// warp at 3.35 Hz).
// Bilinear at fs=48k: K ~= 2*fs; tau*K = 4557.1:
//   b0 = 0.989882, b1 = -0.989882, a1 = -0.999561  (poles/zeros: z=1 zero, ~DC pole)
// ---------------------------------------------------------------------------
struct Dc2InputBuffer {
    // [DTCM] hot path: one 1st-order section
    Dc2FirstOrderSection hpf;

    void init(float sampleRate) noexcept {
        constexpr float tau = Dc2Constants::kC4 * (Dc2Constants::kR5 + Dc2Constants::kR6);
        constexpr float k   = Dc2Constants::kR5 / (Dc2Constants::kR5 + Dc2Constants::kR6);
        constexpr float fc  = 1.0f / (6.2831853f * tau);   // 3.353 Hz
        hpf.initAnalog(k * tau, 0.0f, tau, 1.0f, fc, sampleRate);
    }

    [[nodiscard]] float process(float x) noexcept { return hpf.process(x); }
    void reset() noexcept { hpf.reset(); }
};

// ---------------------------------------------------------------------------
// Block 2: Dc2PreEmphasis — IC1B OPA2134 inverting shelf (+9.6 dB HF boost).
//
// Netlist (ref doc §2): Zi = R7 || (R8 + 1/sC5) from IC1A out to virtual
// ground; Zf = R10 || (1/sC8) || (R9 + 1/sC7) from virtual ground to output.
//
// Audio-band model (C7 1uF short above 1.6 Hz, C8 100pF pole at 64 kHz
// dropped — both documented in ref doc "Session-1 modeling decisions"):
//   Zi(s) = R7 * (1 + s*C5*R8) / (1 + s*C5*(R7+R8))
//   Zf    = R10 || R9 = 24.812k                    (resistive in-band)
//   H(s)  = -Zf/Zi = -k0 * (1 + s*tz) / (1 + s*tp)
//     k0 = (R10||R9)/R7 = 24812.03/47000 = 0.527916
//     tz = C5*(R7+R8) = 4.7e-9*57e3 = 267.9 us -> fz = 594.06 Hz
//     tp = C5*R8      = 4.7e-9*10e3 =  47.0 us -> fp = 3386.28 Hz
//     HF gain = k0*tz/tp = 3.00912 (+9.57 dB)
// Prewarp at fp = 3386.28 Hz (same prewarp as the mixer's inverse shelf so
// the exact analog cancellation carries to the digital domain).
// Bilinear at fs=48k (K = 21276.6/tan(0.221595) = 94439.8):
//   A0 = tp*K + 1 = 5.4387;  b0 = -(k0*(tz*K+1))/A0 ... computed in init().
// Sign: inverting stage; output = -H(s)*x with H as above (negative gain).
// ---------------------------------------------------------------------------
struct Dc2PreEmphasis {
    // [DTCM] hot path
    Dc2FirstOrderSection shelf;

    void init(float sampleRate) noexcept {
        constexpr float rf = (Dc2Constants::kR10 * Dc2Constants::kR9)
                           / (Dc2Constants::kR10 + Dc2Constants::kR9);   // 24812.03
        constexpr float k0 = rf / Dc2Constants::kR7;                     // 0.527916
        constexpr float tz = Dc2Constants::kC5 * (Dc2Constants::kR7 + Dc2Constants::kR8);
        constexpr float tp = Dc2Constants::kC5 * Dc2Constants::kR8;
        constexpr float fp = 1.0f / (6.2831853f * tp);                   // 3386.28 Hz
        // Negative numerator: physical inversion of IC1B.
        shelf.initAnalog(-k0 * tz, -k0, tp, 1.0f, fp, sampleRate);
    }

    [[nodiscard]] float process(float x) noexcept { return shelf.process(x); }
    void reset() noexcept { shelf.reset(); }
};

// ---------------------------------------------------------------------------
// Block 3: Dc2AntiAliasLpf — Q5 2N5088 bootstrapped emitter-follower LPF.
//
// Netlist (ref doc §4): in -> R25 10k -> n1{C22 470p->GND} -> R26 10k
//   -> n2{C20 1n8 -> OUTPUT (bootstrap)} -> R27 10k -> n3{C21 220p->GND}
//   -> Q5 base; Q5 emitter = output (ideal unity buffer, R28 10k load).
//
// Nodal analysis with ideal buffer (Vout = V3), G = 1/10k:
//   n1: (Vin-V1)G = V1*s*C22 + (V1-V2)G
//   n2: (V1-V2)G  = (V2-Vout)s*C20 + (V2-V3)G
//   n3: (V2-V3)G  = V3*s*C21
// Solving (symbolic, sympy; see ref doc):
//   H(s) = 1 / (a3*s^3 + a2*s^2 + a1*s + 1)
//   a1 = 1.1300e-5, a2 = 9.9880e-11, a3 = 1.8612e-16
// Factorization:
//   complex pair: f0 = 17 933.4 Hz, Q = 0.99305
//   real pole:    f1 = 67 350.3 Hz  -- ABOVE NYQUIST at fs=48k: DROPPED.
//   (error from dropping: <=0.06 dB @8 kHz, <=0.4 dB @20 kHz; documented.)
// Implemented section: H(s) = w0^2 / (s^2 + (w0/Q)*s + w0^2),
//   w0 = 2*pi*17933.4 = 112 679.6 rad/s, w0/Q = 113 468.4.
// Prewarp at f0 = 17 933.4 Hz.
// ---------------------------------------------------------------------------
struct Dc2AntiAliasLpf {
    // [DTCM] hot path
    Dc2BiquadSection lpf;

    // Analytically derived from the R=10k / 470p / 1n8 / 220p ladder above.
    static constexpr float kF0 = 17933.4f;
    static constexpr float kQ  = 0.99305f;

    void init(float sampleRate) noexcept {
        const float w0 = 6.2831853f * kF0;
        lpf.initAnalog(0.0f, 0.0f, w0 * w0,
                       1.0f, w0 / kQ, w0 * w0,
                       kF0, sampleRate);
    }

    [[nodiscard]] float process(float x) noexcept { return lpf.process(x); }
    void reset() noexcept { lpf.reset(); }
};

// ---------------------------------------------------------------------------
// Block 4: Dc2ReconLpf — Q10 (ch. C) / Q4 (ch. B) reconstruction filter.
//
// Netlist (ref doc §9): BBD out (pins 7+8, 56k load) -> C42 22n ->
//   bias node{R67 680k -> +9V, R69 680k -> GND} -> identical 10k/470p/1n8/220p
//   bootstrapped EF ladder as Dc2AntiAliasLpf (Q10/Q4, R70/R24 10k loads).
//
// Input coupling H(s) = s*th / (1 + s*th), th = C42*(R67||R69) = 22n*340k
//   = 7.480 ms -> fc = 21.277 Hz. Prewarp at 21.277 Hz.
// LPF section: same cubic as anti-alias (f0 = 17 933.4 Hz, Q = 0.99305,
//   67.35 kHz real pole dropped). Full derivation in Dc2AntiAliasLpf comment.
// ---------------------------------------------------------------------------
struct Dc2ReconLpf {
    // [DTCM] hot path
    Dc2FirstOrderSection hpf;
    Dc2BiquadSection     lpf;

    void init(float sampleRate) noexcept {
        constexpr float th = Dc2Constants::kC42 * Dc2Constants::kRbias;  // 7.480 ms
        constexpr float fh = 1.0f / (6.2831853f * th);                   // 21.277 Hz
        hpf.initAnalog(th, 0.0f, th, 1.0f, fh, sampleRate);
        const float w0 = 6.2831853f * Dc2AntiAliasLpf::kF0;
        lpf.initAnalog(0.0f, 0.0f, w0 * w0,
                       1.0f, w0 / Dc2AntiAliasLpf::kQ, w0 * w0,
                       Dc2AntiAliasLpf::kF0, sampleRate);
    }

    [[nodiscard]] float process(float x) noexcept {
        return lpf.process(hpf.process(x));
    }
    void reset() noexcept { hpf.reset(); lpf.reset(); }
};

// ---------------------------------------------------------------------------
// Block 5: Dc2DryLowpass — Q2 2N5088 dry low-frequency buffer.
//
// Netlist (ref doc §11): dry rail -> R12 68k -> node{C11 22n -> GND}
//   -> R11 2k2 -> Q2 base (EF, R13 10k load) -> C6 1uF -> R89 1M -> VR.
//
// With ideal buffer (Q2 base Z ~= hFE*(re + 10k||load) >= 1 MOhm for
// hFE >= 300, so R11+Zbase loading shifts fc by <3% and gain by <0.2 dB —
// below the 1.5 dB threshold; C6/R89 corner 0.16 Hz treated as unity):
//   H(s) = 1 / (1 + s*R12*C11),  tau = 68e3*22e-9 = 1.496 ms
//   fc = 106.39 Hz.
//
// DISCRETIZATION NOTE — matched-Nyquist zero instead of the default BLT zero:
// A plain bilinear one-pole at fc = 106 Hz forces a transmission zero at
// z = -1, but the analog magnitude at fs/2 is fc-limited, not zero
// (|H_a(j*pi*fs)| = 0.004433 at fs = 48 kHz). Deep in the -20 dB/dec region
// (8 kHz = fs/6) the tan() frequency warp then costs -0.85 dB — outside the
// 0.5 dB gate — and no prewarp choice fixes both 1 kHz and 8 kHz at once.
// Fix: keep the BLT-prewarped pole p (prewarp at fc), and replace the zero
// with one that matches the analog magnitude EXACTLY at DC and at fs/2:
//   p = (tau*K - 1)/(tau*K + 1),      K = wc/tan(wc/(2*fs))
//   m = 1/sqrt(1 + (pi*fs*tau)^2)     (analog |H| at fs/2)
//   g*(1 + c)/(1 - p) = 1  (DC),  g*(1 - c)/(1 + p) = m  (fs/2)
//   => g = ((1-p) + m*(1+p))/2,  c = (1-p)/g - 1
// At fs = 48 kHz: p = 0.986170, g = 1.13171e-2, c = 0.222043.
// Residual error: 0.00 dB @100 Hz, -0.005 dB @1 kHz, -0.30 dB @8 kHz.
// (Same convention as Orfanidis-style magnitude-matched digital filters.)
// Feeds R87/R88 220k into both mixers: low-end dry reinforcement.
// ---------------------------------------------------------------------------
struct Dc2DryLowpass {
    // [DTCM] hot path
    Dc2FirstOrderSection lpf;

    void init(float sampleRate) noexcept {
        constexpr float tau = Dc2Constants::kR12 * Dc2Constants::kC11;   // 1.496 ms
        constexpr float wc  = 1.0f / tau;                                // 668.4 rad/s
        const float K = wc / std::tan(wc / (2.0f * sampleRate));
        const float p = (tau * K - 1.0f) / (tau * K + 1.0f);
        const float pifsTau = 3.14159265f * sampleRate * tau;
        const float m = 1.0f / std::sqrt(1.0f + pifsTau * pifsTau);
        const float g = 0.5f * ((1.0f - p) + m * (1.0f + p));
        const float c = (1.0f - p) / g - 1.0f;
        // Direct coefficient install: H(z) = g*(1 + c*z^-1)/(1 - p*z^-1)
        lpf.b0 = g;
        lpf.b1 = g * c;
        lpf.a1 = -p;
        lpf.s1 = 0.0f;
    }

    [[nodiscard]] float process(float x) noexcept { return lpf.process(x); }
    void reset() noexcept { lpf.reset(); }
};

// ---------------------------------------------------------------------------
// Block 6: Dc2OutputMixer — IC12A (OUT_L) / IC12B (OUT_R) inverting summer
// with de-emphasis feedback shelf and "Dimension" cross-feedback bridged-T.
// Values identical for both channels (ref doc §12; A-side designators).
//
// Virtual-ground superposition: each input contributes current V/R into the
// summing node; the shared feedback impedance converts the current sum to
// the output voltage:  Vout = -Zf(s) * I_sum.
//
// Feedback (C50 47pF dropped, pole at 72 kHz):
//   Zf(s) = R75 || (R74 + 1/(s*C51))
//         = R75 * (1 + s*C51*R74) / (1 + s*C51*(R74+R75))
//   Normalized shelf: Zf(s)/R75 = (1 + s*47.0us) / (1 + s*267.9us)
//     -> corners 3386.28 Hz (zero of denominator... pole) / 594.06 Hz (zero),
//        DC gain 1, HF gain R74||R75/R75 = 0.17544.
//   This is the EXACT INVERSE of Dc2PreEmphasis's shelf shape
//   (C51*(R74+R75) = C5*(R7+R8) and C51*R74 = C5*R8), so dry stays flat.
//   Prewarp at 3386.28 Hz — same constant as Dc2PreEmphasis, which makes the
//   digital cascade exactly flat too (bilinear substitution distributes over
//   the product).
//
// Scaled input gains (dimensionless, x R75):
//   dry (rail, R71 33k):    R75/R71 = 47/33  = 1.424242
//   dry LF (Q2, R87 220k):  R75/R87 = 47/220 = 0.213636
//   wet (R82/R85 47k):      R75/R82 = 1.0
//   (R86 39k / C62 18n mono-sum branch inactive in stereo; see ref doc §13.)
//
// Cross-feedback admittance (from the OPPOSITE mixer's output; ref doc §12):
//   I/V_other = Y(s) = 1 / ( 1/(s*C55) + R80 || (R84 + 1/(s*C56)) )
//   With t1 = C56*R84 = 39.6us, t2 = C56*(R84+R80) = 255.6us,
//        cr = C55*R80 = 594.0us:
//   Y(s) = s*C55*(1 + s*t2) / (1 + s*(t2 + cr) + s^2*cr*t1)
//   Scaled by R75 (dimensionless transfer to output before the shelf):
//   R75*Y(s) = (3.96436e-8*s^2 + 1.551e-4*s) /
//              (2.35224e-8*s^2 + 8.496e-4*s + 1)
//   Denominator roots: 193.9 Hz and 5554.6 Hz (both real);
//   HF limit = R75/(R80||R84) = 47k/27.89k = 1.6854.
//   Prewarp at the geometric mean f = sqrt(1/(2*pi)^2 / (cr*t1))... =
//   1/(2*pi*sqrt(cr*t1)) = 1037.7 Hz.
//   Combined with the shelf and the inverting sum this yields the measured
//   -21 dB @100 Hz ... -11 dB @16 kHz inverted HF cross-blend.
//
// The cross input must be the OPPOSITE channel's previous output sample
// (one-sample loop delay, Valimaki et al., DAFX 2011 §5.3 — same convention
// as DmmDelayCircuit). Session 3 wires the two mixers together.
// ---------------------------------------------------------------------------
struct Dc2OutputMixer {
    // [DTCM] hot path
    Dc2BiquadSection     crossY;   // R75*Y(s): cross-feedback current path
    Dc2FirstOrderSection zfShelf;  // Zf(s)/R75: common de-emphasis shelf

    static constexpr float kGainDry   = Dc2Constants::kR75 / Dc2Constants::kR71; // 1.424242
    static constexpr float kGainDryLf = Dc2Constants::kR75 / Dc2Constants::kR87; // 0.213636
    static constexpr float kGainWet   = Dc2Constants::kR75 / Dc2Constants::kR82; // 1.0

    void init(float sampleRate) noexcept {
        // Zf/R75 shelf — prewarp matched to Dc2PreEmphasis (3386.28 Hz)
        constexpr float tn = Dc2Constants::kC51 * Dc2Constants::kR74;    // 47.0 us
        constexpr float td = Dc2Constants::kC51 * (Dc2Constants::kR74 + Dc2Constants::kR75);
        constexpr float fp = 1.0f / (6.2831853f * tn);                   // 3386.28 Hz
        zfShelf.initAnalog(tn, 1.0f, td, 1.0f, fp, sampleRate);

        // R75*Y(s) cross-feedback biquad
        constexpr float t1 = Dc2Constants::kC56 * Dc2Constants::kR84;    // 39.6 us
        constexpr float t2 = Dc2Constants::kC56 * (Dc2Constants::kR84 + Dc2Constants::kR80);
        constexpr float cr = Dc2Constants::kC55 * Dc2Constants::kR80;    // 594.0 us
        constexpr float n2 = Dc2Constants::kC55 * Dc2Constants::kR75 * t2; // 3.96436e-8
        constexpr float n1 = Dc2Constants::kC55 * Dc2Constants::kR75;      // 1.551e-4
        constexpr float d2 = cr * t1;                                      // 2.35224e-8
        constexpr float d1 = t2 + cr;                                      // 8.496e-4
        const float fgeo = 1.0f / (6.2831853f * std::sqrt(d2));            // 1037.7 Hz
        crossY.initAnalog(n2, n1, 0.0f, d2, d1, 1.0f, fgeo, sampleRate);
    }

    // dryRail: pre-emphasized dry (IC1B out); dryLf: Dc2DryLowpass out;
    // wet: this channel's expander output; crossIn: OPPOSITE mixer's output
    // (previous sample). Returns this mixer's output (inverting).
    [[nodiscard]] float process(float dryRail, float dryLf,
                                float wet, float crossIn) noexcept {
        const float iSum = kGainDry   * dryRail
                         + kGainDryLf * dryLf
                         + kGainWet   * wet
                         + crossY.process(crossIn);
        return -zfShelf.process(iSum);
    }

    void reset() noexcept { crossY.reset(); zfShelf.reset(); }
};

// ---------------------------------------------------------------------------
// Self-test: per-block sine probes at 100 Hz / 1 kHz / 8 kHz, measured gain
// compared against the FULL analytic |H(s)| (including the elements the
// digital model drops: C7, C8, C50, 67.35 kHz pole) — every block must land
// within 0.5 dB. Build per the compile gate at the top of this file.
// ---------------------------------------------------------------------------
#ifdef DC2_FILTERS_SELFTEST
#include <cstdio>
#include <complex>

namespace dc2_selftest {

using cf = std::complex<float>;

inline cf par(cf a, cf b) { return a * b / (a + b); }

// Full analytic transfer functions (no simplifications) ---------------------
inline float refInputBuffer(float f) {
    const cf s(0.0f, 6.2831853f * f);
    const cf zc4 = 1.0f / (s * Dc2Constants::kC4);
    return std::abs(cf(Dc2Constants::kR5) /
                    (Dc2Constants::kR6 + zc4 + Dc2Constants::kR5));
}
inline float refPreEmphasis(float f) {
    const cf s(0.0f, 6.2831853f * f);
    const cf zi = par(cf(Dc2Constants::kR7),
                      Dc2Constants::kR8 + 1.0f / (s * Dc2Constants::kC5));
    const cf zf = 1.0f / (1.0f / cf(Dc2Constants::kR10)
                        + s * 100e-12f                              // C8
                        + 1.0f / (Dc2Constants::kR9 + 1.0f / (s * 1e-6f))); // R9+C7
    return std::abs(zf / zi);
}
inline float refLadderCubic(float f) {
    const cf s(0.0f, 6.2831853f * f);
    // Full cubic incl. the 67.35 kHz pole the digital model drops
    const cf d = 1.8612e-16f * s * s * s + 9.988e-11f * s * s + 1.13e-5f * s + 1.0f;
    return std::abs(1.0f / d);
}
inline float refRecon(float f) {
    const cf s(0.0f, 6.2831853f * f);
    const cf th = s * (Dc2Constants::kC42 * Dc2Constants::kRbias);
    return std::abs(th / (1.0f + th)) * refLadderCubic(f);
}
inline float refDryLowpass(float f) {
    const cf s(0.0f, 6.2831853f * f);
    return std::abs(1.0f / (1.0f + s * (Dc2Constants::kR12 * Dc2Constants::kC11)));
}
inline cf refZf(float f) {   // full feedback incl. C50 47pF
    const cf s(0.0f, 6.2831853f * f);
    return 1.0f / (1.0f / cf(Dc2Constants::kR75)
                 + s * 47e-12f
                 + 1.0f / (Dc2Constants::kR74 + 1.0f / (s * Dc2Constants::kC51)));
}
inline float refMixerR(float f, float rin) { return std::abs(refZf(f)) / rin; }
inline float refMixerCross(float f) {
    const cf s(0.0f, 6.2831853f * f);
    const cf z1 = par(cf(Dc2Constants::kR80),
                      Dc2Constants::kR84 + 1.0f / (s * Dc2Constants::kC56));
    return std::abs(refZf(f) / (1.0f / (s * Dc2Constants::kC55) + z1));
}

// Measurement helper ---------------------------------------------------------
template <typename Fn>
float measureGain(Fn&& proc, float freq, float fs) {
    constexpr int kWarm = 9600, kMeas = 9600;
    const float w = 6.2831853f * freq / fs;
    for (int i = 0; i < kWarm; ++i)
        (void)proc(0.05f * std::sin(w * static_cast<float>(i)));
    float si = 0.0f, so = 0.0f;
    for (int i = 0; i < kMeas; ++i) {
        const float x = 0.05f * std::sin(w * static_cast<float>(kWarm + i));
        const float y = proc(x);
        si += x * x;  so += y * y;
    }
    return std::sqrt(so / si);
}

inline int g_passed = 0, g_failed = 0;

inline void check(const char* name, float f, float got, float expected) {
    const float errDb = 20.0f * std::log10(got / expected);
    const bool ok = std::fabs(errDb) <= 0.5f;
    std::printf("[%s] %-28s %7.0f Hz: got %8.4f  expect %8.4f  err %+6.3f dB\n",
                ok ? "PASS" : "FAIL", name, f, got, expected, errDb);
    (ok ? g_passed : g_failed)++;
}

} // namespace dc2_selftest

int main() {
    using namespace dc2_selftest;
    constexpr float kFs = 48000.0f;
    const float freqs[3] = {100.0f, 1000.0f, 8000.0f};

    for (float f : freqs) {
        Dc2InputBuffer b;  b.init(kFs);
        check("Dc2InputBuffer", f,
              measureGain([&](float x) { return b.process(x); }, f, kFs),
              refInputBuffer(f));
    }
    for (float f : freqs) {
        Dc2PreEmphasis b;  b.init(kFs);
        check("Dc2PreEmphasis", f,
              measureGain([&](float x) { return b.process(x); }, f, kFs),
              refPreEmphasis(f));
    }
    for (float f : freqs) {
        Dc2AntiAliasLpf b;  b.init(kFs);
        check("Dc2AntiAliasLpf", f,
              measureGain([&](float x) { return b.process(x); }, f, kFs),
              refLadderCubic(f));
    }
    for (float f : freqs) {
        Dc2ReconLpf b;  b.init(kFs);
        check("Dc2ReconLpf", f,
              measureGain([&](float x) { return b.process(x); }, f, kFs),
              refRecon(f));
    }
    for (float f : freqs) {
        Dc2DryLowpass b;  b.init(kFs);
        check("Dc2DryLowpass", f,
              measureGain([&](float x) { return b.process(x); }, f, kFs),
              refDryLowpass(f));
    }
    // Mixer: each input path in isolation against the full analytic model.
    for (float f : freqs) {
        Dc2OutputMixer m;  m.init(kFs);
        check("Mixer dry (R71)", f,
              measureGain([&](float x) { return m.process(x, 0, 0, 0); }, f, kFs),
              refMixerR(f, Dc2Constants::kR71));
    }
    for (float f : freqs) {
        Dc2OutputMixer m;  m.init(kFs);
        check("Mixer dryLF (R87)", f,
              measureGain([&](float x) { return m.process(0, x, 0, 0); }, f, kFs),
              refMixerR(f, Dc2Constants::kR87));
    }
    for (float f : freqs) {
        Dc2OutputMixer m;  m.init(kFs);
        check("Mixer wet (R82)", f,
              measureGain([&](float x) { return m.process(0, 0, x, 0); }, f, kFs),
              refMixerR(f, Dc2Constants::kR82));
    }
    for (float f : freqs) {
        Dc2OutputMixer m;  m.init(kFs);
        check("Mixer cross (C55 T)", f,
              measureGain([&](float x) { return m.process(0, 0, 0, x); }, f, kFs),
              refMixerCross(f));
    }
    // Integration: pre-emphasis -> mixer dry input must be net-flat (~-2.5 dB)
    for (float f : freqs) {
        Dc2PreEmphasis pe;   pe.init(kFs);
        Dc2OutputMixer m;    m.init(kFs);
        check("Dry path flatness", f,
              measureGain([&](float x) { return m.process(pe.process(x), 0, 0, 0); }, f, kFs),
              refPreEmphasis(f) * refMixerR(f, Dc2Constants::kR71));
    }
    // Stability / NaN guard: 1 s of pseudo-noise through the full set.
    {
        Dc2InputBuffer ib;   ib.init(kFs);
        Dc2PreEmphasis pe;   pe.init(kFs);
        Dc2AntiAliasLpf aa;  aa.init(kFs);
        Dc2ReconLpf rc;      rc.init(kFs);
        Dc2DryLowpass dl;    dl.init(kFs);
        Dc2OutputMixer mL;   mL.init(kFs);
        Dc2OutputMixer mR;   mR.init(kFs);
        float x = 0.31623f, outL = 0.0f, outR = 0.0f;
        bool finite = true;
        for (int i = 0; i < 48000; ++i) {
            x = std::sin(x * 1.618033f + 0.5f) * 0.31623f;
            const float d  = pe.process(ib.process(x));
            const float w  = rc.process(aa.process(d));
            const float lf = dl.process(d);
            const float pL = outL, pR = outR;   // one-sample cross delay
            outL = mL.process(d, lf, w, pR);
            outR = mR.process(d, lf, w, pL);
            if (!std::isfinite(outL) || !std::isfinite(outR)) { finite = false; break; }
        }
        std::printf("[%s] cross-coupled stereo chain NaN/Inf check (48000 samples)\n",
                    finite ? "PASS" : "FAIL");
        (finite ? g_passed : g_failed)++;
    }

    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
#endif // DC2_FILTERS_SELFTEST
