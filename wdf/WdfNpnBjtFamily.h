#pragma once

#include "WdfPort.h"
#include "WdfNonlinear.h"   // for diode_physics::Vt
#include <cmath>
#include <algorithm>

/// WdfNpnBjtFamily — parametric NPN BJT, full Ebers-Moll with Early effect.
/// 3-port element: one port each for Base, Collector, Emitter.
/// 2D Newton-Raphson solves for (Vbe, Vbc) per sample.
///
/// CALLER SETS port_B.Rp, port_C.Rp, port_E.Rp before calling reflect().
/// These are set once at circuit init time (resistive passive tree).
/// For circuits where adaptor tree resistances change (VC elements adjacent
/// to this BJT), update the port Rp values before each reflect() call.
///
/// Interface note: port names use underscores (port_B, port_C, port_E) to
/// distinguish from the existing WdfNpnBjt / WdfPnpBjt which use camelCase
/// (portB, portC, portE). The reflect() signature is identical: void, noexcept.
class WdfNpnBjtFamily {
public:
    WdfPort port_B;   // Base port:     caller sets Rp and a; reflect() writes b
    WdfPort port_C;   // Collector port
    WdfPort port_E;   // Emitter port

    WdfNpnBjtFamily() = default;

    /// Initialize transistor parameters. Must be called before reflect().
    /// Does NOT set port_B/C/E.Rp — caller sets those from the adaptor tree.
    void init(float Is, float Bf, float Br, float n, float Va) noexcept {
        Is_ = std::max(Is, 1e-30f);
        Bf_ = std::max(Bf, 1.0f);
        Br_ = std::max(Br, 1e-6f);
        n_  = std::max(n,  0.1f);
        Va_ = std::max(Va, 0.1f);
        prev_Vbe_  = 0.6f;
        prev_Vbc_  = -1.0f;
        last_Ic_   = 0.0f;
        last_Ib_   = 0.0f;
        iter_count_ = 0;
        converged_  = false;
    }

    /// Audio-path. Reads port_B.a, port_C.a, port_E.a and all three Rp values.
    /// Solves Ebers-Moll + port equations via 2D NR for (Vbe, Vbc).
    /// Writes port_B.b, port_C.b, port_E.b.
    void reflect() noexcept {
        const float nVt = n_ * diode_physics::Vt;
        const float RpB = port_B.Rp;
        const float RpC = port_C.Rp;
        const float RpE = port_E.Rp;
        const float aB  = port_B.a;
        const float aC  = port_C.a;
        const float aE  = port_E.a;

        // Precompute driving terms for residuals (Section 5.3)
        const float dBE = aB - aE;   // appears in F1
        const float dBC = aB - aC;   // appears in F2

        float Vbe = prev_Vbe_;
        float Vbc = prev_Vbc_;

        constexpr int   kMaxIter = 20;
        constexpr float kTol     = 1.0e-6f;
        constexpr float kStep    = 0.5f;
        constexpr float kDetMin  = 1.0e-20f;

        bool conv = false;

        for (int i = 0; i < kMaxIter; ++i) {
            iter_count_ = i + 1;

            // Clamp exponent arguments before expf (guards against ±Inf)
            float expBE = expf(std::clamp(Vbe / nVt, -80.0f, 80.0f));
            float expBC = expf(std::clamp(Vbc / nVt, -80.0f, 80.0f));

            // Early factor (clamped for stability)
            float Ea = std::clamp(1.0f + (Vbe - Vbc) / Va_, 0.01f, 100.0f);

            // Collector and base currents
            float Ic = Is_ * (expBE - expBC) * Ea;
            float Ib = (Is_ / Bf_) * (expBE - 1.0f) + (Is_ / Br_) * (expBC - 1.0f);

            // Residuals (Section 5.3)
            //   F1: Vbe - (aB-aE) + (RpB+RpE)*Ib + RpE*Ic = 0
            //   F2: Vbc - (aB-aC) + RpB*Ib - RpC*Ic = 0
            float F1 = Vbe - dBE + (RpB + RpE) * Ib + RpE * Ic;
            float F2 = Vbc - dBC + RpB * Ib - RpC * Ic;

            if (fabsf(F1) < kTol && fabsf(F2) < kTol) {
                conv = true;
                break;
            }

            // Jacobian partial derivatives (Section 5.4)
            float inv_nVt  = 1.0f / nVt;
            float inv_Va   = 1.0f / Va_;
            float diffEB   = expBE - expBC;

            float dIb_dVbe = (Is_ / Bf_) * inv_nVt * expBE;
            float dIb_dVbc = (Is_ / Br_) * inv_nVt * expBC;
            float dIc_dVbe = Is_ * inv_nVt * expBE * Ea  + Is_ * diffEB * inv_Va;
            float dIc_dVbc = -Is_ * inv_nVt * expBC * Ea - Is_ * diffEB * inv_Va;

            // Jacobian 2x2
            float J00 = 1.0f + (RpB + RpE) * dIb_dVbe + RpE * dIc_dVbe;
            float J01 =        (RpB + RpE) * dIb_dVbc + RpE * dIc_dVbc;
            float J10 =  RpB * dIb_dVbe               - RpC * dIc_dVbe;
            float J11 = 1.0f + RpB * dIb_dVbc         - RpC * dIc_dVbc;

            // Solve 2×2 via Cramer's rule
            float det = J00 * J11 - J01 * J10;
            if (fabsf(det) < kDetMin) continue;  // near-singular: skip step

            float invDet = 1.0f / det;
            float dVbe = (F1 * J11 - F2 * J01) * invDet;
            float dVbc = (F2 * J00 - F1 * J10) * invDet;

            Vbe -= std::clamp(dVbe, -kStep, kStep);
            Vbc -= std::clamp(dVbc, -kStep, kStep);
        }

        converged_ = conv;
        prev_Vbe_  = Vbe;
        prev_Vbc_  = Vbc;

        // Compute final currents at solution
        float expBE = expf(std::clamp(Vbe / nVt, -80.0f, 80.0f));
        float expBC = expf(std::clamp(Vbc / nVt, -80.0f, 80.0f));
        float Ea    = std::clamp(1.0f + (Vbe - Vbc) / Va_, 0.01f, 100.0f);

        float Ic = Is_ * (expBE - expBC) * Ea;
        float Ib = (Is_ / Bf_) * (expBE - 1.0f) + (Is_ / Br_) * (expBC - 1.0f);
        float Ie = -(Ib + Ic);

        last_Ic_ = Ic;
        last_Ib_ = Ib;

        // Reflected waves (NPN: Ib flows INTO base, Ic flows OUT of collector,
        //   Ie = -(Ib+Ic) flows INTO emitter from external circuit perspective)
        port_B.b = aB - 2.0f * RpB * Ib;
        port_C.b = aC - 2.0f * RpC * Ic;
        port_E.b = aE - 2.0f * RpE * Ie;   // Ie negative → aE + 2*RpE*(Ib+Ic)
    }

    /// Diagnostics — read after reflect().
    float lastVbe()   const noexcept { return prev_Vbe_; }
    float lastVbc()   const noexcept { return prev_Vbc_; }
    float lastIc()    const noexcept { return last_Ic_; }
    float lastIb()    const noexcept { return last_Ib_; }
    int   lastIters() const noexcept { return iter_count_; }
    bool  lastConv()  const noexcept { return converged_; }

    void reset() noexcept {
        port_B.reset();
        port_C.reset();
        port_E.reset();
        prev_Vbe_   = 0.6f;
        prev_Vbc_   = -1.0f;
        last_Ic_    = 0.0f;
        last_Ib_    = 0.0f;
        iter_count_ = 0;
        converged_  = false;
    }

private:
    float Is_  = 6.734e-15f;
    float Bf_  = 416.0f;
    float Br_  = 0.739f;
    float n_   = 1.259f;
    float Va_  = 74.03f;

    float prev_Vbe_   = 0.6f;
    float prev_Vbc_   = -1.0f;
    float last_Ic_    = 0.0f;
    float last_Ib_    = 0.0f;
    int   iter_count_ = 0;
    bool  converged_  = false;
};

// ──────────────────────────────────────────────────────────────────────────────
// npn_params namespace — transistor parameter structs for WdfNpnBjtFamily
// ──────────────────────────────────────────────────────────────────────────────
namespace npn_params {

    // ── Silicon NPN ────────────────────────────────────────────────────────────
    // Source: Fairchild/ON Semi SPICE model.
    struct N2N3904 {
        static constexpr float Is = 6.734e-15f;  // transport saturation current, A
        static constexpr float Bf = 416.0f;      // forward current gain
        static constexpr float Br = 0.739f;      // reverse current gain
        static constexpr float n  = 1.259f;      // ideality factor
        static constexpr float Va = 74.03f;      // Early voltage, V
    };
    // Source: Mullard datasheet fit. Premium matched NPN.
    struct Bc549c {
        static constexpr float Is = 5.0e-15f;
        static constexpr float Bf = 600.0f;
        static constexpr float Br = 0.5f;
        static constexpr float n  = 1.25f;
        static constexpr float Va = 100.0f;
    };
    // Source: Mullard datasheet. Classic BC109C — Fuzz Face NPN variant, Range Boost.
    struct Bc109c {
        static constexpr float Is = 8.0e-15f;
        static constexpr float Bf = 450.0f;
        static constexpr float Br = 0.65f;
        static constexpr float n  = 1.26f;
        static constexpr float Va = 120.0f;
    };
    // Source: Mullard datasheet. Predecessor to BC109.
    struct Bc108 {
        static constexpr float Is = 1.0e-14f;
        static constexpr float Bf = 420.0f;
        static constexpr float Br = 0.6f;
        static constexpr float n  = 1.26f;
        static constexpr float Va = 100.0f;
    };
    // Source: Central Semiconductor LTspice model, rev. A, 2010-08-18
    //   IS=38.116E-15  BF=599.06  BR=10.364  NF=1.0(default)  VAF=100
    // NE=1.9979/ISE are Gummel-Poon non-ideal base terms — not used by this solver.
    struct N2N5088 {
        static constexpr float Is = 38.116e-15f;  // transport saturation current (IS)
        static constexpr float Bf = 599.06f;      // forward beta (BF)
        static constexpr float Br = 10.364f;      // reverse beta (BR)
        static constexpr float n  = 1.0f;         // forward ideality factor (NF default)
        static constexpr float Va = 100.0f;       // Early voltage (VAF)
    };
    // Source: SPICE model. Early small-signal NPN.
    struct N2N2222 {
        static constexpr float Is = 14.34e-15f;
        static constexpr float Bf = 255.0f;
        static constexpr float Br = 0.6f;
        static constexpr float n  = 1.307f;
        static constexpr float Va = 74.03f;
    };
    // Source: Philips datasheet.
    struct Bc183 {
        static constexpr float Is = 3.0e-15f;
        static constexpr float Bf = 500.0f;
        static constexpr float Br = 0.5f;
        static constexpr float n  = 1.25f;
        static constexpr float Va = 100.0f;
    };
    struct Bc184 {
        static constexpr float Is = 3.0e-15f;
        static constexpr float Bf = 500.0f;
        static constexpr float Br = 0.5f;
        static constexpr float n  = 1.25f;
        static constexpr float Va = 100.0f;
    };
    // Source: Philips datasheet. Medium-power NPN.
    struct Bc337 {
        static constexpr float Is = 4.0e-15f;
        static constexpr float Bf = 300.0f;
        static constexpr float Br = 0.3f;
        static constexpr float n  = 1.25f;
        static constexpr float Va = 80.0f;
    };
    // Source: Philips datasheet. Budget silicon NPN.
    struct Bc547 {
        static constexpr float Is = 5.0e-15f;
        static constexpr float Bf = 400.0f;
        static constexpr float Br = 0.5f;
        static constexpr float n  = 1.25f;
        static constexpr float Va = 80.0f;
    };
    // Darlington / high-gain NPN. Source: approximate; verify against MPSA13 datasheet.
    struct Mpsa13 {
        static constexpr float Is = 0.5e-15f;
        static constexpr float Bf = 5000.0f;
        static constexpr float Br = 1.0f;
        static constexpr float n  = 1.3f;
        static constexpr float Va = 150.0f;
    };
    struct Mpsa18 {
        static constexpr float Is = 0.5e-15f;
        static constexpr float Bf = 5000.0f;
        static constexpr float Br = 1.0f;
        static constexpr float n  = 1.3f;
        static constexpr float Va = 150.0f;
    };
    struct Mpsa42 {
        static constexpr float Is = 0.5e-15f;
        static constexpr float Bf = 5000.0f;
        static constexpr float Br = 1.0f;
        static constexpr float n  = 1.3f;
        static constexpr float Va = 150.0f;
    };

    // ── Germanium NPN ─────────────────────────────────────────────────────────
    // Source: Mullard GE datasheet. Rare; high hFE, soft knee.
    struct Ac127 {
        static constexpr float Is = 1.0e-11f;
        static constexpr float Bf = 100.0f;
        static constexpr float Br = 2.0f;
        static constexpr float n  = 1.3f;
        static constexpr float Va = 80.0f;
    };

} // namespace npn_params
