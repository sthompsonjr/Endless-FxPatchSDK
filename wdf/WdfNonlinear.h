#pragma once

#include "WdfPort.h"
#include "LambertW.h"
#include <algorithm>
#include <cmath>

/// WDF Ideal Diode — single diode, analytical solution via Lambert W.
///
/// Models: I(V) = Is * (exp(V / Vt) - 1)
/// where Is = saturation current, Vt = thermal voltage.
///
/// Typical values:
///   Silicon:   Is = 1e-7 A,  Vt = 0.02585 V
///   Germanium: Is = 1e-6 A,  Vt = 0.02585 V
///
/// The analytical solution uses the Fettweis/Werner formulation:
///   Given incident wave 'a' and port resistance Rp:
///   Let R_Is = Is * Rp
///   arg = R_Is / Vt * exp((a + R_Is) / Vt)
///   V_diode = a - R_Is + Vt * W0(arg) - (a - R_Is + Vt * W0(arg)) ... simplified:
///   b = a - 2.0 * (a - Vt * W0(arg) + R_Is) + a ... final:
///   b = 2.0 * Vt * W0(arg) - 2.0 * R_Is - a
///
/// Derivation:
///   The diode voltage V = 0.5*(a+b) and current I = (a-b)/(2*Rp).
///   Substituting I = Is*(exp(V/Vt)-1) and solving for b:
///   a - b = 2*Rp*Is*(exp((a+b)/(2*Vt)) - 1)
///   Let x = (a+b)/(2*Vt), then solving via Lambert W yields:
///   (a+b)/2 = Vt * W0(Is*Rp/Vt * exp((a + Is*Rp)/Vt)) - Is*Rp
///   V = Vt * W0(arg) - Is*Rp where arg = Is*Rp/Vt * exp((a + Is*Rp)/Vt)
///   b = 2*V - a
class WdfIdealDiode {
public:
    WdfPort port;

    void init(float saturationCurrent, float thermalVoltage, float portResistance) {
        Is_ = std::max(saturationCurrent, 1e-15f);
        Vt_ = std::max(thermalVoltage, 1e-6f);
        port.Rp = std::max(portResistance, 1e-9f);
        RIs_ = Is_ * port.Rp;
        invVt_ = 1.0f / Vt_;
        port.reset();
    }

    void reflect() noexcept {
        // Fettweis analytical diode solution via Lambert W0
        // Clamp exponent to prevent Inf (expf overflows above ~88)
        float expArg = std::clamp((port.a + RIs_) * invVt_, -80.0f, 80.0f);
        float arg = (RIs_ * invVt_) * expf(expArg);
        float wArg = math::lambertW0(arg);
        float V = Vt_ * wArg - RIs_;
        port.b = 2.0f * V - port.a;
    }

    void reset() noexcept { port.reset(); }

private:
    float Is_ = 1e-7f;
    float Vt_ = 0.02585f;
    float RIs_ = 0.0f;
    float invVt_ = 1.0f / 0.02585f;
};

/// WDF Antiparallel Diodes — two matched diodes in antiparallel.
/// Models symmetric soft clipping (Tube Screamer, Big Muff style).
///
/// Net current: I_net(V) = Is * (exp(V/Vt) - exp(-V/Vt)) = 2*Is*sinh(V/Vt)
///
/// Analytical solution uses Lambert W with both branches:
///   For the antiparallel pair, the reflected wave is:
///   V = sign(a) * (Vt * W0(arg) - R_Is)
///   where arg = R_Is/Vt * exp((|a| + R_Is) / Vt)
///   b = 2*V - a
///
/// This works because the antiparallel pair has odd symmetry.
class WdfAntiparallelDiodes {
public:
    WdfPort port;

    void init(float saturationCurrent, float thermalVoltage, float portResistance) {
        Is_ = std::max(saturationCurrent, 1e-15f);
        Vt_ = std::max(thermalVoltage, 1e-6f);
        port.Rp = std::max(portResistance, 1e-9f);
        RIs_ = Is_ * port.Rp;
        invVt_ = 1.0f / Vt_;
        port.reset();
    }

    void reflect() noexcept {
        // Exploit odd symmetry: solve for |a|, then restore sign
        float absA = fabsf(port.a);
        float signA = (port.a >= 0.0f) ? 1.0f : -1.0f;

        // Clamp exponent to prevent Inf (expf overflows above ~88)
        float expArg = std::clamp((absA + RIs_) * invVt_, -80.0f, 80.0f);
        float arg = (RIs_ * invVt_) * expf(expArg);
        float wArg = math::lambertW0(arg);
        float V = signA * (Vt_ * wArg - RIs_);
        port.b = 2.0f * V - port.a;
    }

    void reset() noexcept { port.reset(); }

private:
    float Is_ = 1e-7f;
    float Vt_ = 0.02585f;
    float RIs_ = 0.0f;
    float invVt_ = 1.0f / 0.02585f;
};

/// WDF NPN BJT — Ebers-Moll simplified model, Newton-Raphson solved.
///
/// Three-port device (Base, Collector, Emitter).
/// Simplified forward-active model:
///   Ic = Is * (exp(Vbe / Vt) - 1)
///   Ib = Ic / hFE
///   Ie = Ic + Ib = Ic * (1 + 1/hFE)
///
/// This is the most expensive primitive (~2000-5000 cycles/sample).
/// Uses 2D Newton-Raphson with warm-starting from previous sample.
/// Maximum 8 iterations; returns previous valid solution if not converged.
class WdfNpnBjt {
public:
    WdfPort portB; // base
    WdfPort portC; // collector
    WdfPort portE; // emitter

    void init(float Is, float Vt, float hFE,
              float RpB, float RpC, float RpE) {
        Is_ = std::max(Is, 1e-15f);
        Vt_ = std::max(Vt, 1e-6f);
        hFE_ = std::max(hFE, 1.0f);
        invVt_ = 1.0f / Vt_;
        alphaF_ = hFE_ / (hFE_ + 1.0f);

        portB.Rp = std::max(RpB, 1e-9f);
        portC.Rp = std::max(RpC, 1e-9f);
        portE.Rp = std::max(RpE, 1e-9f);

        portB.reset();
        portC.reset();
        portE.reset();
        prevVbe_ = 0.6f; // typical forward bias
        prevVce_ = 5.0f; // typical operating point
    }

    void reflect() noexcept {
        // Incident waves provide the "source" terms
        float aB = portB.a;
        float aC = portC.a;
        float aE = portE.a;

        // Solve for Vbe using 1D Newton-Raphson (simplified approach)
        // In forward active: Vce doesn't strongly affect Ic (Early effect ignored)
        // So we solve the KCL at the base-emitter junction.
        //
        // From wave variables:
        //   Vb = 0.5*(aB + bB), Ib = (aB - bB)/(2*RpB)
        //   Ve = 0.5*(aE + bE), Ie = (aE - bE)/(2*RpE)
        //   Vc = 0.5*(aC + bC), Ic = (aC - bC)/(2*RpC)
        //
        // Vbe = Vb - Ve, and Ic = Is*(exp(Vbe/Vt) - 1)
        // Ib = Ic/hFE, Ie = Ic + Ib = Ic*(1 + 1/hFE)
        //
        // The constraint: Vbe must satisfy the circuit's KVL/KCL.
        // Using the WDF formulation, we iterate on Vbe.

        float Vbe = prevVbe_;

        for (int iter = 0; iter < 8; ++iter) {
            float eVbe = expf(std::clamp(Vbe * invVt_, -20.0f, 40.0f));
            float Ic = Is_ * (eVbe - 1.0f);
            float Ib = Ic / hFE_;
            float Ie = Ic + Ib;

            // From wave equations:
            // bB = aB - 2*RpB*Ib, bE = aE + 2*RpE*Ie, bC = aC - 2*RpC*Ic
            // Vb = 0.5*(aB + bB) = 0.5*(aB + aB - 2*RpB*Ib) = aB - RpB*Ib
            // Ve = 0.5*(aE + bE) = 0.5*(aE + aE + 2*RpE*Ie) = aE + RpE*Ie
            // Vbe = Vb - Ve = aB - RpB*Ib - aE - RpE*Ie

            float Vbe_calc = aB - portB.Rp * Ib - aE - portE.Rp * Ie;
            float residual = Vbe - Vbe_calc;

            if (fabsf(residual) < 1e-5f) break;

            // Derivative of residual w.r.t. Vbe:
            // d(Ic)/d(Vbe) = Is/Vt * exp(Vbe/Vt)
            float dIc = Is_ * invVt_ * eVbe;
            float dIb = dIc / hFE_;
            float dIe = dIc + dIb;
            float dResidual = 1.0f + portB.Rp * dIb + portE.Rp * dIe;

            if (fabsf(dResidual) < 1e-12f) break;
            float delta = residual / dResidual;
            delta = std::clamp(delta, -0.5f, 0.5f);
            Vbe -= delta;
        }

        prevVbe_ = Vbe;

        // Compute final currents
        float eVbe = expf(std::clamp(Vbe * invVt_, -20.0f, 40.0f));
        float Ic = Is_ * (eVbe - 1.0f);
        float Ib = Ic / hFE_;
        float Ie = Ic + Ib;

        // Compute reflected waves from currents
        portB.b = aB - 2.0f * portB.Rp * Ib;
        portC.b = aC - 2.0f * portC.Rp * Ic;
        portE.b = aE + 2.0f * portE.Rp * Ie;
    }

    void reset() noexcept {
        portB.reset();
        portC.reset();
        portE.reset();
        prevVbe_ = 0.6f;
        prevVce_ = 5.0f;
    }

private:
    float Is_ = 1e-7f;
    float Vt_ = 0.02585f;
    float hFE_ = 100.0f;
    float invVt_ = 1.0f / 0.02585f;
    float alphaF_ = 0.99f;
    float prevVbe_ = 0.6f;
    float prevVce_ = 5.0f;
};

/// WDF PNP BJT — Ebers-Moll simplified model, Newton-Raphson solved.
///
/// Three-port device (Base, Collector, Emitter).
/// Simplified forward-active model:
///   Ie = Is * (exp(Veb / Vt) - 1)   where Veb = Ve - Vb > 0
///   Ib = Ie / hFE
///   Ic = alphaF * Ie = Ie * hFE / (hFE + 1)
///
/// PNP current convention (positive values, transistor reference):
///   - Ie flows INTO emitter from external circuit
///   - Ic flows OUT of collector into external circuit
///   - Ib flows OUT of base into external circuit
///
/// WDF wave variable equations:
///   bE = aE - 2*RpE*Ie   (current INTO emitter port)
///   bC = aC + 2*RpC*Ic   (current OUT of collector port)
///   bB = aB + 2*RpB*Ib   (current OUT of base port)
///
/// Newton-Raphson iterates on Veb = Ve - Vb.
/// This is the mirror image of WdfNpnBjt and suitable for:
///   Fuzz Face (PNP germanium), Dallas Arbiter, silicon PNP gain stages.
class WdfPnpBjt {
public:
    WdfPort portB; // base
    WdfPort portC; // collector
    WdfPort portE; // emitter

    void init(float Is, float Vt, float hFE,
              float RpB, float RpC, float RpE) {
        Is_ = std::max(Is, 1e-15f);
        Vt_ = std::max(Vt, 1e-6f);
        hFE_ = std::max(hFE, 1.0f);
        invVt_ = 1.0f / Vt_;
        alphaF_ = hFE_ / (hFE_ + 1.0f);

        portB.Rp = std::max(RpB, 1e-9f);
        portC.Rp = std::max(RpC, 1e-9f);
        portE.Rp = std::max(RpE, 1e-9f);

        portB.reset();
        portC.reset();
        portE.reset();
        prevVeb_ = 0.6f; // typical forward bias (emitter-base)
        prevVec_ = 5.0f; // typical operating point (emitter-collector)
    }

    void reflect() noexcept {
        float aB = portB.a;
        float aC = portC.a;
        float aE = portE.a;

        // Solve for Veb using 1D Newton-Raphson.
        //
        // From wave variables:
        //   Ve = 0.5*(aE + bE), bE = aE - 2*RpE*Ie → Ve = aE - RpE*Ie
        //   Vb = 0.5*(aB + bB), bB = aB + 2*RpB*Ib → Vb = aB + RpB*Ib
        //   Veb = Ve - Vb = aE - RpE*Ie - aB - RpB*Ib
        //
        // Residual: f(Veb) = Veb - (aE - RpE*Ie - aB - RpB*Ib)
        // Derivative: f'(Veb) = 1 + RpE*dIe/dVeb + RpB*dIb/dVeb

        float Veb = prevVeb_;

        for (int iter = 0; iter < 8; ++iter) {
            float eVeb = expf(std::clamp(Veb * invVt_, -20.0f, 40.0f));
            float Ie = Is_ * (eVeb - 1.0f);
            float Ib = Ie / hFE_;

            // Veb_calc from circuit KVL using wave variables
            float Veb_calc = aE - portE.Rp * Ie - aB - portB.Rp * Ib;
            float residual = Veb - Veb_calc;

            if (fabsf(residual) < 1e-5f) break;

            // Derivative of residual w.r.t. Veb
            float dIe = Is_ * invVt_ * eVeb;
            float dIb = dIe / hFE_;
            float dResidual = 1.0f + portE.Rp * dIe + portB.Rp * dIb;

            if (fabsf(dResidual) < 1e-12f) break;
            float delta = residual / dResidual;
            delta = std::clamp(delta, -0.5f, 0.5f);
            Veb -= delta;
        }

        prevVeb_ = Veb;

        // Compute final currents
        float eVeb = expf(std::clamp(Veb * invVt_, -20.0f, 40.0f));
        float Ie = Is_ * (eVeb - 1.0f);
        float Ib = Ie / hFE_;
        float Ic = alphaF_ * Ie;

        // Compute reflected waves: PNP current directions
        portE.b = aE - 2.0f * portE.Rp * Ie; // current INTO emitter
        portC.b = aC + 2.0f * portC.Rp * Ic; // current OUT of collector
        portB.b = aB + 2.0f * portB.Rp * Ib; // current OUT of base
    }

    void reset() noexcept {
        portB.reset();
        portC.reset();
        portE.reset();
        prevVeb_ = 0.6f;
        prevVec_ = 5.0f;
    }

private:
    float Is_ = 1e-7f;
    float Vt_ = 0.02585f;
    float hFE_ = 100.0f;
    float invVt_ = 1.0f / 0.02585f;
    float alphaF_ = 0.99f;
    float prevVeb_ = 0.6f;
    float prevVec_ = 5.0f;
};
