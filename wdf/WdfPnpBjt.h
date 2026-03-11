#pragma once

#include "WdfPort.h"
#include <algorithm>
#include <cmath>

/// WDF PNP BJT — Ebers-Moll model with Early effect, 2D Newton-Raphson solved.
///
/// Three-port device (Base, Collector, Emitter).
/// Forward-active model with Early effect:
///   IC_no_early = Is * (exp(Veb / Vt) - 1)    Veb = Ve - Vb > 0 for forward bias
///   earlyFactor = 1 + |Vce| / Vaf              Vce = Vc - Ve < 0 in active region
///   IC = IC_no_early * earlyFactor
///   IB = IC / hFE
///   IE = IC + IB
///
/// PNP current convention (all positive in forward active):
///   IE flows INTO emitter from external circuit
///   IC flows OUT of collector into external circuit
///   IB flows OUT of base into external circuit
///   KCL: IE = IC + IB
///
/// WDF wave variable equations (signs opposite to NPN):
///   portE.b = aE - 2*RpE*IE   (IE INTO emitter port  → positive port current)
///   portC.b = aC + 2*RpC*IC   (IC OUT of collector port → negative port current)
///   portB.b = aB + 2*RpB*IB   (IB OUT of base port   → negative port current)
///
/// Port voltages derived from wave variables:
///   Ve = aE - RpE*IE
///   Vb = aB + RpB*IB
///   Vc = aC + RpC*IC
///
/// 2D Newton-Raphson iterates on (Veb, Vce) simultaneously using an analytical
/// Jacobian whose cross-terms cancel: det = 1 + A*dIC_dVeb - B*dIC_dVce.
/// Maximum 8 iterations; warm-starts from previous sample.
///
/// Suitable for: Dallas Rangemaster (OC44), Tone Bender MkI (OC75),
///               Fuzz Face (AC128), 2N3906 silicon PNP gain stages.
///
/// WARNING: Expensive (~2000-6000 cycles/sample). Limit to one per effect.
class WdfPnpBjt {
public:
    /// Transistor parameters. Use pnp_presets::* for named device values.
    struct Params {
        float Is  = 3e-6f;    ///< Saturation current [A]
        float Vt  = 0.02585f; ///< Thermal voltage [V] (kT/q ≈ 26mV at 25°C)
        float hFE = 80.0f;    ///< Forward current gain (beta)
        float Vaf = 50.0f;    ///< Early voltage [V] (output impedance effect)
    };

    WdfPort portB; ///< Base port
    WdfPort portC; ///< Collector port
    WdfPort portE; ///< Emitter port

    void init(const Params& params, float RpB, float RpC, float RpE) noexcept {
        p.Is  = std::max(params.Is,  1e-15f);
        p.Vt  = std::max(params.Vt,  1e-6f);
        p.hFE = std::max(params.hFE, 1.0f);
        p.Vaf = std::max(params.Vaf, 0.1f);
        invVt_ = 1.0f / p.Vt;

        portB.Rp = std::max(RpB, 1e-9f);
        portC.Rp = std::max(RpC, 1e-9f);
        portE.Rp = std::max(RpE, 1e-9f);

        portB.reset();
        portC.reset();
        portE.reset();

        // Warm-start: silicon has higher Veb forward bias than germanium
        prevVeb_ = (p.Is < 1e-9f) ? 0.6f : 0.15f;
        prevVce_ = -4.0f;
        lastIter_ = 0;
        didConverge_ = true;
    }

    void reflect() noexcept {
        float Veb = prevVeb_;
        float Vce = prevVce_;
        solveNR(Veb, Vce);
        prevVeb_ = Veb;
        prevVce_ = Vce;

        // Compute currents at solution
        float IC = collectorCurrent(Veb, Vce);
        float IB = IC / p.hFE;
        float IE = IC + IB;

        // PNP wave reflections
        portB.b = portB.a + 2.0f * portB.Rp * IB; // IB flows OUT of base
        portC.b = portC.a + 2.0f * portC.Rp * IC; // IC flows OUT of collector
        portE.b = portE.a - 2.0f * portE.Rp * IE; // IE flows INTO emitter
    }

    void reset() noexcept {
        portB.reset();
        portC.reset();
        portE.reset();
        prevVeb_ = (p.Is < 1e-9f) ? 0.6f : 0.15f;
        prevVce_ = -4.0f;
        lastIter_ = 0;
        didConverge_ = true;
    }

    [[nodiscard]] int  lastIterationCount() const noexcept { return lastIter_; }
    [[nodiscard]] bool lastConverged()      const noexcept { return didConverge_; }

    // --- Factory presets ------------------------------------------------

    static WdfPnpBjt makeOC44(float RpB, float RpC, float RpE) noexcept {
        WdfPnpBjt bjt;
        bjt.init(Params{ 1e-5f, 0.02585f, 70.0f, 35.0f }, RpB, RpC, RpE);
        return bjt;
    }

    static WdfPnpBjt makeOC75(float RpB, float RpC, float RpE) noexcept {
        WdfPnpBjt bjt;
        bjt.init(Params{ 3e-6f, 0.02585f, 45.0f, 40.0f }, RpB, RpC, RpE);
        return bjt;
    }

    static WdfPnpBjt makeAC128(float RpB, float RpC, float RpE) noexcept {
        WdfPnpBjt bjt;
        bjt.init(Params{ 3e-6f, 0.02585f, 80.0f, 50.0f }, RpB, RpC, RpE);
        return bjt;
    }

    static WdfPnpBjt make2N3906(float RpB, float RpC, float RpE) noexcept {
        WdfPnpBjt bjt;
        bjt.init(Params{ 1e-12f, 0.02585f, 200.0f, 100.0f }, RpB, RpC, RpE);
        return bjt;
    }

private:
    Params p;
    float invVt_ = 1.0f / 0.02585f;
    float prevVeb_ = 0.15f;
    float prevVce_ = -4.0f;
    int   lastIter_ = 0;
    bool  didConverge_ = true;

    [[nodiscard]] float collectorCurrent(float Veb, float Vce) const noexcept {
        float expArg = std::min(Veb * invVt_, 80.0f);
        float IC_no_early = p.Is * (expf(expArg) - 1.0f);
        float earlyFactor = 1.0f + fabsf(Vce) / p.Vaf;
        return IC_no_early * earlyFactor;
    }

    void computeResiduals(float Veb, float Vce,
                          float& f1, float& f2) const noexcept {
        float IC = collectorCurrent(Veb, Vce);
        float IB = IC / p.hFE;
        float IE = IC + IB;

        // Ve = aE - RpE*IE,  Vb = aB + RpB*IB  →  Veb_circuit = Ve - Vb
        float Veb_circ = portE.a - portE.Rp * IE - portB.a - portB.Rp * IB;
        f1 = Veb - Veb_circ;

        // Vc = aC + RpC*IC,  Vce_circuit = Vc - Ve = (aC + RpC*IC) - (aE - RpE*IE)
        float Vce_circ = portC.a + portC.Rp * IC - (portE.a - portE.Rp * IE);
        f2 = Vce - Vce_circ;
    }

    void computeJacobian(float Veb, float Vce,
                         float& J11, float& J12,
                         float& J21, float& J22) const noexcept {
        float expArg = std::min(Veb * invVt_, 80.0f);
        float IC_no_early = p.Is * (expf(expArg) - 1.0f);
        float earlyFactor = 1.0f + fabsf(Vce) / p.Vaf;

        float dIC_dVeb = (IC_no_early + p.Is) * earlyFactor * invVt_;
        float dIC_dVce = IC_no_early * ((Vce < 0.0f) ? -1.0f : 1.0f) / p.Vaf;

        // Precomputed groupings (from plan):
        //   A = RpE*(1+1/hFE) + RpB/hFE    (base-emitter combination)
        //   B = RpC + RpE*(1+1/hFE)         (collector-emitter combination)
        float inv_hFE = 1.0f / p.hFE;
        float RpE_tot = portE.Rp * (1.0f + inv_hFE); // RpE*(1+1/hFE)
        float A = RpE_tot + portB.Rp * inv_hFE;
        float B = portC.Rp + RpE_tot;

        J11 = 1.0f + A * dIC_dVeb;
        J12 = A * dIC_dVce;
        J21 = -B * dIC_dVeb;
        J22 = 1.0f - B * dIC_dVce;
    }

    void solveNR(float& Veb, float& Vce) noexcept {
        didConverge_ = false;
        for (int i = 0; i < 8; ++i) {
            lastIter_ = i + 1;
            float f1, f2;
            computeResiduals(Veb, Vce, f1, f2);

            if (fabsf(f1) < 1e-5f && fabsf(f2) < 1e-4f) {
                didConverge_ = true;
                break;
            }

            float J11, J12, J21, J22;
            computeJacobian(Veb, Vce, J11, J12, J21, J22);

            // det = J11*J22 - J12*J21 (simplifies to 1 + A*dIC_dVeb - B*dIC_dVce)
            float det = J11 * J22 - J12 * J21;
            if (fabsf(det) < 1e-12f) break;
            float invDet = 1.0f / det;

            // Standard Cramer's rule: J * delta = f → delta = J^{-1} * f
            // Newton step: x_new = x - delta
            float dVeb = (f1 * J22 - f2 * J12) * invDet;
            float dVce = (J11 * f2 - J21 * f1) * invDet;

            // Clamp step size for stability
            dVeb = std::clamp(dVeb, -0.5f, 0.5f);
            dVce = std::clamp(dVce, -5.0f, 5.0f);

            Veb -= dVeb;
            Vce -= dVce;
        }
    }
};

/// Named transistor presets for direct use with WdfPnpBjt::Params.
namespace pnp_presets {
    /// OC44 — germanium PNP (Dallas Rangemaster treble boost, ~1966)
    static constexpr WdfPnpBjt::Params OC44  = { 1e-5f, 0.02585f, 70.0f,  35.0f };
    /// OC75 — germanium PNP (Tone Bender MkI fuzz, Marshall Supa Fuzz)
    static constexpr WdfPnpBjt::Params OC75  = { 3e-6f, 0.02585f, 45.0f,  40.0f };
    /// AC128 — germanium PNP (Dallas Fuzz Face, Arbiter Fuzz Face)
    static constexpr WdfPnpBjt::Params AC128 = { 3e-6f, 0.02585f, 80.0f,  50.0f };
    /// 2N3906 — silicon PNP (modern high-gain PNP pedals)
    static constexpr WdfPnpBjt::Params N3906 = { 1e-12f, 0.02585f, 200.0f, 100.0f };
} // namespace pnp_presets
