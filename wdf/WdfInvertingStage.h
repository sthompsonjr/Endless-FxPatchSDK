#pragma once

#include "WdfNonlinear.h"
#include "WdfOpAmpLM741.h"
#include <algorithm>
#include <cmath>

/// Inverting op-amp stage with diode clipping in feedback loop.
///
/// Models an LM741 in inverting configuration:
///   Vin --[R1]--+-- (V_minus, virtual ground)
///               |
///         [Rf || C2]  (feedback network, parallel)
///               |
///         [antiparallel diodes]  (clipping in feedback loop)
///               |
///             Vout
///
/// Architecture:
///   1. Compute ideal closed-loop output via IIR lowpass:
///      Vout_ideal(s) = -(Rf/R1) * Vin / (1 + s*Rf*C2)
///   2. Apply antiparallel diode soft clipping (WDF Lambert W)
///   3. Apply LM741 slew rate limiting and rail clamping
///
/// The IIR directly models the closed-loop transfer function under the
/// virtual ground approximation (valid when A0 >> Rf/R1). The diode
/// clipping uses the WDF Lambert W solver for accurate soft-knee behavior.
class WdfInvertingStage {
public:
    void init(float R1, float Rf, float C2,
              float diodeIs, float diodeVt, float sampleRate) {
        sampleRate_ = sampleRate;
        R1_ = R1;
        C2_ = C2;
        diodeIs_ = diodeIs;
        diodeVt_ = diodeVt;

        updateFeedbackCoeffs(Rf);

        // Diode port resistance: the output impedance seen by the diodes
        // In the feedback loop, diodes see approximately Rf || Z_C2 in parallel.
        // At audio frequencies, this is dominated by Z_C2.
        // Use Rf as the port resistance (reasonable approximation).
        diodes_.init(diodeIs, diodeVt, Rf);

        // Init LM741 model
        opamp_.init(sampleRate);

        // Clear state
        iirState_ = 0.0f;
        xPrev_ = 0.0f;
    }

    /// Update feedback resistance (called when gain knob changes).
    void setFeedbackResistance(float Rf) {
        updateFeedbackCoeffs(Rf);
        diodes_.init(diodeIs_, diodeVt_, Rf);
    }

    [[nodiscard]] float process(float vIn) noexcept {
        // Step 1: Compute ideal closed-loop output via IIR
        // H(z) = -(Rf/R1) * (1+z^-1) / ((1+beta) + (1-beta)*z^-1)
        // Bilinear transform of H(s) = -(Rf/R1) / (1 + s*Rf*C2)
        float vIdeal = iirA0_ * (vIn + xPrev_) + iirB1_ * iirState_;
        xPrev_ = vIn;
        iirState_ = vIdeal;

        // Step 2: Apply antiparallel diode soft clipping
        // The diodes limit the output voltage to approximately ±0.6V (silicon).
        // Using WDF Lambert W solver for accurate soft-knee characteristic.
        diodes_.port.a = 2.0f * vIdeal; // incident wave from Thevenin source
        diodes_.reflect();
        float vClipped = diodes_.port.voltage();

        // Step 3: Apply LM741 model (slew rate + rail clamping)
        // Feed the clipped signal as if it were the op-amp's output
        // The LM741 model adds bandwidth limitation and slew rate effects.
        float vOut = opamp_.process(-vClipped);

        return vOut;
    }

    void reset() noexcept {
        diodes_.reset();
        opamp_.reset();
        iirState_ = 0.0f;
        xPrev_ = 0.0f;
    }

    /// Access LM741 for parameter tweaking (e.g. dying battery effect).
    WdfOpAmpLM741& opamp() { return opamp_; }

private:
    void updateFeedbackCoeffs(float Rf) {
        Rf_ = Rf;

        // Closed-loop gain and time constant
        float gain = Rf / R1_;          // inverting gain magnitude
        float tau = Rf * C2_;           // feedback time constant

        // Bilinear transform: H(s) = -gain / (1 + s*tau)
        // H(z) = -gain * (1+z^-1) / ((1+beta) + (1-beta)*z^-1)
        float beta = 2.0f * sampleRate_ * tau;
        float invOnePlusBeta = 1.0f / (1.0f + beta);
        iirA0_ = -gain * invOnePlusBeta;   // negative for inverting
        iirB1_ = (beta - 1.0f) * invOnePlusBeta;
    }

    WdfAntiparallelDiodes diodes_;
    WdfOpAmpLM741 opamp_;

    // Circuit parameters
    float R1_ = 68000.0f;
    float Rf_ = 100000.0f;
    float C2_ = 47e-9f;
    float sampleRate_ = 48000.0f;
    float diodeIs_ = 1e-7f;
    float diodeVt_ = 0.02585f;

    // IIR filter state (closed-loop transfer function)
    float iirA0_ = 0.0f;
    float iirB1_ = 0.0f;
    float iirState_ = 0.0f;
    float xPrev_ = 0.0f;
};
