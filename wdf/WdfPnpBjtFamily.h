#pragma once

#include "WdfPort.h"
#include "WdfNonlinear.h"   // for diode_physics::Vt
#include <cmath>
#include <algorithm>

/// WdfPnpBjtFamily — parametric PNP BJT, full Ebers-Moll with Early effect.
/// 3-port element: one port each for Base, Collector, Emitter.
/// 2D Newton-Raphson solves for (Veb, Vcb) per sample.
///
/// PNP current convention (all positive in forward active):
///   IE flows INTO emitter, IC flows OUT of collector, IB flows OUT of base.
///   KCL: IE = IC + IB
///
/// WDF wave reflections (matching WdfPnpBjt sign convention):
///   port_B.b = aB + 2*RpB*IB   (IB flows OUT of base → negative port current)
///   port_C.b = aC + 2*RpC*IC   (IC flows OUT of collector → negative port current)
///   port_E.b = aE - 2*RpE*IE   (IE flows INTO emitter → positive port current)
///
/// CALLER SETS port_B.Rp, port_C.Rp, port_E.Rp before calling reflect().
///
/// Interface note: port names use underscores (port_B, port_C, port_E) to
/// distinguish from the existing WdfPnpBjt which uses camelCase.
class WdfPnpBjtFamily {
public:
    WdfPort port_B;   // Base port
    WdfPort port_C;   // Collector port
    WdfPort port_E;   // Emitter port

    WdfPnpBjtFamily() = default;

    /// Initialize transistor parameters. Must be called before reflect().
    /// Does NOT set port_B/C/E.Rp — caller sets those from the adaptor tree.
    void init(float Is, float Bf, float Br, float n, float Va) noexcept {
        Is_ = std::max(Is, 1e-30f);
        Bf_ = std::max(Bf, 1.0f);
        Br_ = std::max(Br, 1e-6f);
        n_  = std::max(n,  0.1f);
        Va_ = std::max(Va, 0.1f);
        prev_Veb_   = 0.2f;    // warm-start: typical germanium PNP Veb
        prev_Vcb_   = -1.0f;
        last_Ic_    = 0.0f;
        last_Ib_    = 0.0f;
        iter_count_ = 0;
        converged_  = false;
    }

    /// Audio-path. Reads port_B.a, port_C.a, port_E.a and all three Rp values.
    /// Solves Ebers-Moll + port equations via 2D NR for (Veb, Vcb).
    /// Writes port_B.b, port_C.b, port_E.b.
    void reflect() noexcept {
        const float nVt = n_ * diode_physics::Vt;
        const float RpB = port_B.Rp;
        const float RpC = port_C.Rp;
        const float RpE = port_E.Rp;
        const float aB  = port_B.a;
        const float aC  = port_C.a;
        const float aE  = port_E.a;

        // Precompute driving terms for residuals.
        // PNP port equations:
        //   Ve = aE - RpE*IE,  Vb = aB + RpB*IB,  Vc = aC + RpC*IC
        //   Veb = Ve - Vb = (aE - aB) - RpE*IE - RpB*IB
        //                 = (aE - aB) - (RpE+RpB)*IB - RpE*IC   [since IE=IB+IC]
        //   Vcb = Vc - Vb = (aC - aB) + RpC*IC - RpB*IB
        const float dEA = aE - aB;   // driving term for F1
        const float dCA = aC - aB;   // driving term for F2

        float Veb = prev_Veb_;
        float Vcb = prev_Vcb_;

        constexpr int   kMaxIter = 20;
        constexpr float kTol     = 1.0e-6f;
        constexpr float kStep    = 0.5f;
        constexpr float kDetMin  = 1.0e-20f;

        bool conv = false;

        for (int i = 0; i < kMaxIter; ++i) {
            iter_count_ = i + 1;

            // Clamp exponent arguments before expf
            float expEB = expf(std::clamp(Veb / nVt, -80.0f, 80.0f));
            float expCB = expf(std::clamp(Vcb / nVt, -80.0f, 80.0f));

            // Early factor (clamped for stability)
            // PNP: Ea = 1 + (Veb - Vcb) / Va
            // In forward active: Veb>0, Vcb<0 → (Veb - Vcb) > 0 → Ea > 1 ✓
            float Ea = std::clamp(1.0f + (Veb - Vcb) / Va_, 0.01f, 100.0f);

            // Collector and base currents
            float Ic = Is_ * (expEB - expCB) * Ea;
            float Ib = (Is_ / Bf_) * (expEB - 1.0f) + (Is_ / Br_) * (expCB - 1.0f);

            // Residuals
            //   F1: Veb - (aE-aB) + (RpE+RpB)*Ib + RpE*Ic = 0
            //   F2: Vcb - (aC-aB) - RpC*Ic + RpB*Ib = 0
            float F1 = Veb - dEA + (RpE + RpB) * Ib + RpE * Ic;
            float F2 = Vcb - dCA - RpC * Ic + RpB * Ib;

            if (fabsf(F1) < kTol && fabsf(F2) < kTol) {
                conv = true;
                break;
            }

            // Jacobian partial derivatives
            float inv_nVt  = 1.0f / nVt;
            float inv_Va   = 1.0f / Va_;
            float diffEB   = expEB - expCB;   // cached

            float dIb_dVeb = (Is_ / Bf_) * inv_nVt * expEB;
            float dIb_dVcb = (Is_ / Br_) * inv_nVt * expCB;
            float dIc_dVeb = Is_ * inv_nVt * expEB * Ea  + Is_ * diffEB * inv_Va;
            float dIc_dVcb = -Is_ * inv_nVt * expCB * Ea - Is_ * diffEB * inv_Va;

            // Jacobian 2x2
            float J00 = 1.0f + (RpE + RpB) * dIb_dVeb + RpE * dIc_dVeb;
            float J01 =        (RpE + RpB) * dIb_dVcb + RpE * dIc_dVcb;
            float J10 =  RpB * dIb_dVeb               - RpC * dIc_dVeb;
            float J11 = 1.0f + RpB * dIb_dVcb         - RpC * dIc_dVcb;

            // Solve 2×2 via Cramer's rule
            float det = J00 * J11 - J01 * J10;
            if (fabsf(det) < kDetMin) continue;  // near-singular: skip step

            float invDet = 1.0f / det;
            float dVeb = (F1 * J11 - F2 * J01) * invDet;
            float dVcb = (F2 * J00 - F1 * J10) * invDet;

            Veb -= std::clamp(dVeb, -kStep, kStep);
            Vcb -= std::clamp(dVcb, -kStep, kStep);
        }

        converged_ = conv;
        prev_Veb_  = Veb;
        prev_Vcb_  = Vcb;

        // Compute final currents at solution
        float expEB = expf(std::clamp(Veb / nVt, -80.0f, 80.0f));
        float expCB = expf(std::clamp(Vcb / nVt, -80.0f, 80.0f));
        float Ea    = std::clamp(1.0f + (Veb - Vcb) / Va_, 0.01f, 100.0f);

        float Ic = Is_ * (expEB - expCB) * Ea;
        float Ib = (Is_ / Bf_) * (expEB - 1.0f) + (Is_ / Br_) * (expCB - 1.0f);
        float Ie = Ib + Ic;

        last_Ic_ = Ic;
        last_Ib_ = Ib;

        // Reflected waves (PNP sign convention, matching WdfPnpBjt)
        port_B.b = aB + 2.0f * RpB * Ib;   // IB flows OUT of base
        port_C.b = aC + 2.0f * RpC * Ic;   // IC flows OUT of collector
        port_E.b = aE - 2.0f * RpE * Ie;   // IE flows INTO emitter
    }

    /// Diagnostics — read after reflect().
    float lastVeb()   const noexcept { return prev_Veb_; }
    float lastVcb()   const noexcept { return prev_Vcb_; }
    float lastIc()    const noexcept { return last_Ic_; }
    float lastIb()    const noexcept { return last_Ib_; }
    int   lastIters() const noexcept { return iter_count_; }
    bool  lastConv()  const noexcept { return converged_; }

    void reset() noexcept {
        port_B.reset();
        port_C.reset();
        port_E.reset();
        prev_Veb_   = 0.2f;
        prev_Vcb_   = -1.0f;
        last_Ic_    = 0.0f;
        last_Ib_    = 0.0f;
        iter_count_ = 0;
        converged_  = false;
    }

private:
    float Is_  = 4.87e-6f;
    float Bf_  = 60.0f;
    float Br_  = 2.0f;
    float n_   = 1.3f;
    float Va_  = 20.0f;

    float prev_Veb_   = 0.2f;
    float prev_Vcb_   = -1.0f;
    float last_Ic_    = 0.0f;
    float last_Ib_    = 0.0f;
    int   iter_count_ = 0;
    bool  converged_  = false;
};

// ──────────────────────────────────────────────────────────────────────────────
// pnp_params namespace — transistor parameter structs for WdfPnpBjtFamily
// ──────────────────────────────────────────────────────────────────────────────
namespace pnp_params {

    // ── Silicon PNP ───────────────────────────────────────────────────────────
    // Source: SPICE reference (mirror of 2N3904).
    struct N2N3906 {
        static constexpr float Is = 6.734e-15f;
        static constexpr float Bf = 180.0f;
        static constexpr float Br = 4.0f;
        static constexpr float n  = 1.259f;
        static constexpr float Va = 100.0f;
    };

    // ── Germanium PNP ─────────────────────────────────────────────────────────
    // Source: GE/Mullard datasheet; Fuzz Face original.
    // NOTE: These values differ from existing WdfPnpBjt factory presets.
    //   The existing presets use estimated/simplified values (AC128: Is=3e-6, hFE=80, Vaf=50).
    //   The values here are from the SPICE datasheet fit (Is=4.87e-6, Bf=60, Va=20).
    //   The warm-start is 0.2V (germmanium Veb) vs 0.6V (silicon Vbe).
    struct Ac128 {
        static constexpr float Is = 4.87e-6f;   // high Is → low Vbe threshold ~0.2V
        static constexpr float Bf = 60.0f;
        static constexpr float Br = 2.0f;
        static constexpr float n  = 1.3f;
        static constexpr float Va = 20.0f;      // low Va → strong Early effect
    };
    // Source: British germanium; OC81 vintage.
    struct Oc81 {
        static constexpr float Is = 3.0e-6f;
        static constexpr float Bf = 120.0f;
        static constexpr float Br = 3.0f;
        static constexpr float n  = 1.3f;
        static constexpr float Va = 30.0f;
    };
    // Source: Mullard rare; boutique NKT275.
    struct Nkt275 {
        static constexpr float Is = 2.5e-6f;
        static constexpr float Bf = 150.0f;
        static constexpr float Br = 3.5f;
        static constexpr float n  = 1.3f;
        static constexpr float Va = 25.0f;
    };

} // namespace pnp_params
