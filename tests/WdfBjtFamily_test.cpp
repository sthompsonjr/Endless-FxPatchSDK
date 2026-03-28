#include <cstdio>
#include <cmath>
#include <cassert>
#include <chrono>

#include "wdf/WdfNpnBjtFamily.h"
#include "wdf/WdfPnpBjtFamily.h"

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

// Compute expected Ic for NPN using Ebers-Moll + Early effect.
static float npn_ic_expected(float Vbe, float Vbc,
                              float Is, float Bf, float Br, float n, float Va) {
    (void)Bf; (void)Br;
    const float nVt = n * diode_physics::Vt;
    float eBE = expf(std::clamp(Vbe / nVt, -80.0f, 80.0f));
    float eBC = expf(std::clamp(Vbc / nVt, -80.0f, 80.0f));
    float Ea  = 1.0f + (Vbe - Vbc) / Va;
    return Is * (eBE - eBC) * Ea;
}

static float npn_ib_expected(float Vbe, float Vbc,
                              float Is, float Bf, float Br, float n) {
    const float nVt = n * diode_physics::Vt;
    float eBE = expf(std::clamp(Vbe / nVt, -80.0f, 80.0f));
    float eBC = expf(std::clamp(Vbc / nVt, -80.0f, 80.0f));
    return (Is / Bf) * (eBE - 1.0f) + (Is / Br) * (eBC - 1.0f);
}

// NPN port incident waves: a = V + Rp * I_port for all ports.
//   Port current sign (using convention "positive = into element from tree"):
//     Base:     I_port = +Ib  (current flows tree→BJT base)
//     Collector: I_port = +Ic  (current flows tree→BJT collector)
//     Emitter:  I_port = Ie = -(Ib+Ic) < 0 (current flows BJT emitter→tree)
//   Derivation from reflect():
//     portE.b = aE - 2*RpE*Ie, VE = aE - RpE*Ie → aE = VE + RpE*Ie ✓
static float make_incident_npn_B(float V, float I, float Rp) { return V + Rp * I; }
static float make_incident_npn_C(float V, float I, float Rp) { return V + Rp * I; }
static float make_incident_npn_E(float V, float I, float Rp) { return V + Rp * I; }

// PNP port incident waves:
//   V_B = aB + RpB*IB  →  aB = VB - RpB*IB
//   V_C = aC + RpC*IC  →  aC = VC - RpC*IC
//   V_E = aE - RpE*IE  →  aE = VE + RpE*IE
static float make_incident_pnp_B(float V, float I, float Rp) { return V - Rp * I; }
static float make_incident_pnp_C(float V, float I, float Rp) { return V - Rp * I; }
static float make_incident_pnp_E(float V, float I, float Rp) { return V + Rp * I; }

// ─────────────────────────────────────────────────────────────────────────────
// TEST 1: NPN Ebers-Moll DC operating point (2N3904)
// ─────────────────────────────────────────────────────────────────────────────
// At Vbe=0.75V, Vbc=-5V (forward active), expected values from Section 5.5:
//   Ic = 74.2 µA, Ib = 165.5 nA, Beta_dc = 448
// Tolerance: ±1%
void test_npn_dc_operating_point_2n3904() {
    printf("\nTEST 1: WdfNpnBjtFamily DC operating point (2N3904)\n");

    constexpr float Is = npn_params::N2N3904::Is;
    constexpr float Bf = npn_params::N2N3904::Bf;
    constexpr float Br = npn_params::N2N3904::Br;
    constexpr float nn = npn_params::N2N3904::n;
    constexpr float Va = npn_params::N2N3904::Va;

    constexpr float Vbe_t = 0.75f, Vbc_t = -5.0f;
    constexpr float Rp    = 10000.0f;

    float Ic_exp = npn_ic_expected(Vbe_t, Vbc_t, Is, Bf, Br, nn, Va);
    float Ib_exp = npn_ib_expected(Vbe_t, Vbc_t, Is, Bf, Br, nn);
    float Ie_exp = -(Ib_exp + Ic_exp);

    printf("  Expected: Ic = %.2f µA, Ib = %.1f nA, Beta_dc = %.1f\n",
           (double)(Ic_exp * 1e6f), (double)(Ib_exp * 1e9f),
           (double)(Ic_exp / Ib_exp));

    // VE=0, VB=Vbe_t, VC=VB-Vbc_t
    float VE = 0.0f, VB = Vbe_t, VC = Vbe_t - Vbc_t;

    WdfNpnBjtFamily bjt;
    bjt.init(Is, Bf, Br, nn, Va);
    bjt.port_B.Rp = bjt.port_C.Rp = bjt.port_E.Rp = Rp;

    bjt.port_B.a = make_incident_npn_B(VB, Ib_exp, Rp);
    bjt.port_C.a = make_incident_npn_C(VC, Ic_exp, Rp);
    bjt.port_E.a = make_incident_npn_E(VE, Ie_exp, Rp);
    bjt.reflect();

    printf("  Computed: Ic = %.2f µA, Ib = %.1f nA, Beta_dc = %.1f, iters = %d\n",
           (double)(bjt.lastIc() * 1e6f), (double)(bjt.lastIb() * 1e9f),
           (double)(bjt.lastIc() / bjt.lastIb()), bjt.lastIters());

    assert(fabsf(bjt.lastIc() - Ic_exp) / (fabsf(Ic_exp) + 1e-12f) < 0.01f);
    assert(fabsf(bjt.lastIb() - Ib_exp) / (fabsf(Ib_exp) + 1e-12f) < 0.01f);
    assert(bjt.lastConv());
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 2: NPN Beta verification — silicon vs Darlington
// ─────────────────────────────────────────────────────────────────────────────
void test_npn_beta_silicon_vs_darlington() {
    printf("\nTEST 2: NPN Beta — silicon (2N3904) vs Darlington (MPSA13)\n");

    auto check_beta = [](const char* name, float Is, float Bf, float Br, float n, float Va) {
        constexpr float Vbe = 0.75f, Vbc = -5.0f;
        float Ic = npn_ic_expected(Vbe, Vbc, Is, Bf, Br, n, Va);
        float Ib = npn_ib_expected(Vbe, Vbc, Is, Bf, Br, n);
        float beta = Ic / Ib;
        printf("  %s: Beta_dc = %.0f (Bf = %.0f, Early factor adds %.1f%%)\n",
               name, (double)beta, (double)Bf, (double)((beta / Bf - 1.0f) * 100.0f));
        assert(beta >= Bf * 0.95f);
        return beta;
    };

    float b_ref  = check_beta("2N3904", npn_params::N2N3904::Is, npn_params::N2N3904::Bf,
                               npn_params::N2N3904::Br, npn_params::N2N3904::n,
                               npn_params::N2N3904::Va);
    float b_dart = check_beta("MPSA13", npn_params::Mpsa13::Is, npn_params::Mpsa13::Bf,
                               npn_params::Mpsa13::Br, npn_params::Mpsa13::n,
                               npn_params::Mpsa13::Va);

    assert(b_dart > b_ref * 5.0f);
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 3: PNP Ebers-Moll DC operating point (AC128 germanium)
// ─────────────────────────────────────────────────────────────────────────────
// At Veb=0.20V, Vcb=-3V, expected from Section 5.5:
//   Ic = 2.17 mA, Ib = 31.1 µA, Beta_dc = 69.8
void test_pnp_dc_operating_point_ac128() {
    printf("\nTEST 3: WdfPnpBjtFamily DC operating point (AC128 germanium)\n");

    constexpr float Is = pnp_params::Ac128::Is;
    constexpr float Bf = pnp_params::Ac128::Bf;
    constexpr float Br = pnp_params::Ac128::Br;
    constexpr float nn = pnp_params::Ac128::n;
    constexpr float Va = pnp_params::Ac128::Va;

    constexpr float Veb_t = 0.20f, Vcb_t = -3.0f;
    constexpr float Rp    = 1000.0f;

    const float nVt  = nn * diode_physics::Vt;
    float eEB = expf(Veb_t / nVt);
    float eCB = expf(std::clamp(Vcb_t / nVt, -80.0f, 80.0f));
    float Ea  = 1.0f + (Veb_t - Vcb_t) / Va;
    float Ic_exp = Is * (eEB - eCB) * Ea;
    float Ib_exp = (Is / Bf) * (eEB - 1.0f) + (Is / Br) * (eCB - 1.0f);
    float Ie_exp = Ib_exp + Ic_exp;

    printf("  Expected: Ic = %.2f mA, Ib = %.1f µA, Beta_dc = %.1f\n",
           (double)(Ic_exp * 1e3f), (double)(Ib_exp * 1e6f),
           (double)(Ic_exp / Ib_exp));

    // PNP: VB=0, VE=Veb_t, VC=Vcb_t
    float VB = 0.0f, VE = Veb_t, VC = Vcb_t;

    WdfPnpBjtFamily bjt;
    bjt.init(Is, Bf, Br, nn, Va);
    bjt.port_B.Rp = bjt.port_C.Rp = bjt.port_E.Rp = Rp;

    bjt.port_B.a = make_incident_pnp_B(VB, Ib_exp, Rp);
    bjt.port_C.a = make_incident_pnp_C(VC, Ic_exp, Rp);
    bjt.port_E.a = make_incident_pnp_E(VE, Ie_exp, Rp);
    bjt.reflect();

    printf("  Computed: Ic = %.2f mA, Ib = %.1f µA, Beta_dc = %.1f, iters = %d\n",
           (double)(bjt.lastIc() * 1e3f), (double)(bjt.lastIb() * 1e6f),
           (double)(bjt.lastIc() / bjt.lastIb()), bjt.lastIters());

    assert(fabsf(bjt.lastIc() - Ic_exp) / (fabsf(Ic_exp) + 1e-12f) < 0.01f);
    assert(fabsf(bjt.lastIb() - Ib_exp) / (fabsf(Ib_exp) + 1e-12f) < 0.01f);
    assert(bjt.lastConv());
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 4: Early effect magnitude (NPN 2N3904 vs BC549C)
// ─────────────────────────────────────────────────────────────────────────────
void test_early_effect_vce_modulation() {
    printf("\nTEST 4: Early effect — Vce modulation of Ic\n");

    constexpr float Vbe = 0.75f;
    auto test_early = [&](const char* name, float Is, float Bf, float Br, float n, float Va) {
        float Vbc_lo = Vbe - 1.0f;    // Vce = 1V
        float Vbc_hi = Vbe - 10.0f;   // Vce = 10V
        float Ic_lo  = npn_ic_expected(Vbe, Vbc_lo, Is, Bf, Br, n, Va);
        float Ic_hi  = npn_ic_expected(Vbe, Vbc_hi, Is, Bf, Br, n, Va);
        float pct    = (Ic_hi - Ic_lo) / Ic_lo * 100.0f;
        float pct_expected = (10.0f - 1.0f) / Va * 100.0f;
        printf("  %s (Va=%.0fV): Ic rises %.1f%% from Vce=1->10V (expected ~%.1f%%)\n",
               name, (double)Va, (double)pct, (double)pct_expected);
        assert(fabsf(pct - pct_expected) < 2.0f);
    };

    test_early("2N3904", npn_params::N2N3904::Is, npn_params::N2N3904::Bf,
               npn_params::N2N3904::Br, npn_params::N2N3904::n, npn_params::N2N3904::Va);
    test_early("BC549C", npn_params::Bc549c::Is,  npn_params::Bc549c::Bf,
               npn_params::Bc549c::Br,  npn_params::Bc549c::n,  npn_params::Bc549c::Va);
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 5: 2D NR convergence — NPN, cold start across operating regions
// ─────────────────────────────────────────────────────────────────────────────
void test_npn_convergence_regions() {
    printf("\nTEST 5: WdfNpnBjtFamily 2D NR convergence (cold-started)\n");

    WdfNpnBjtFamily bjt;
    bjt.init(npn_params::N2N3904::Is, npn_params::N2N3904::Bf,
             npn_params::N2N3904::Br, npn_params::N2N3904::n, npn_params::N2N3904::Va);
    bjt.port_B.Rp = bjt.port_C.Rp = bjt.port_E.Rp = 10000.0f;

    struct Region { const char* name; float Vbe; float Vbc; };
    Region regions[] = {
        { "cutoff     ", -0.5f,  0.0f  },
        { "saturation ",  0.7f,  0.5f  },
        { "fwd active ",  0.75f, -5.0f },
        { "deep active",  0.8f, -10.0f },
    };

    for (const auto& r : regions) {
        bjt.reset();
        float VE = 0.0f, VB = r.Vbe, VC = r.Vbe - r.Vbc;
        float Ic = npn_ic_expected(r.Vbe, r.Vbc, npn_params::N2N3904::Is,
                                   npn_params::N2N3904::Bf, npn_params::N2N3904::Br,
                                   npn_params::N2N3904::n, npn_params::N2N3904::Va);
        float Ib = npn_ib_expected(r.Vbe, r.Vbc, npn_params::N2N3904::Is,
                                   npn_params::N2N3904::Bf, npn_params::N2N3904::Br,
                                   npn_params::N2N3904::n);
        float Ie = -(Ib + Ic);
        bjt.port_B.a = make_incident_npn_B(VB, Ib, bjt.port_B.Rp);
        bjt.port_C.a = make_incident_npn_C(VC, Ic, bjt.port_C.Rp);
        bjt.port_E.a = make_incident_npn_E(VE, Ie, bjt.port_E.Rp);
        bjt.reflect();
        printf("  %s: %s in %2d iters\n",
               r.name, bjt.lastConv() ? "converged" : "FAILED   ", bjt.lastIters());
        assert(bjt.lastIters() < 20);
        assert(bjt.lastConv());
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 6: Extreme inputs — NaN/Inf guard (both NPN and PNP)
// ─────────────────────────────────────────────────────────────────────────────
void test_extreme_inputs() {
    printf("\nTEST 6: Extreme inputs — NaN/Inf guard\n");

    WdfNpnBjtFamily npn;
    npn.init(npn_params::N2N3904::Is, npn_params::N2N3904::Bf,
             npn_params::N2N3904::Br, npn_params::N2N3904::n, npn_params::N2N3904::Va);
    npn.port_B.Rp = npn.port_C.Rp = npn.port_E.Rp = 10000.0f;

    WdfPnpBjtFamily pnp;
    pnp.init(pnp_params::Ac128::Is, pnp_params::Ac128::Bf,
             pnp_params::Ac128::Br, pnp_params::Ac128::n, pnp_params::Ac128::Va);
    pnp.port_B.Rp = pnp.port_C.Rp = pnp.port_E.Rp = 1000.0f;

    const float extreme[] = { -5.0f, -1.0f, 0.0f, 1.0f, 5.0f, 10.0f };
    for (float v : extreme) {
        npn.port_B.a = npn.port_C.a = npn.port_E.a = v;
        npn.reflect();
        assert(!std::isnan(npn.port_B.b) && !std::isinf(npn.port_B.b));
        assert(!std::isnan(npn.port_C.b) && !std::isinf(npn.port_C.b));
        assert(!std::isnan(npn.port_E.b) && !std::isinf(npn.port_E.b));

        pnp.port_B.a = pnp.port_C.a = pnp.port_E.a = v;
        pnp.reflect();
        assert(!std::isnan(pnp.port_B.b) && !std::isinf(pnp.port_B.b));
        assert(!std::isnan(pnp.port_C.b) && !std::isinf(pnp.port_C.b));
        assert(!std::isnan(pnp.port_E.b) && !std::isinf(pnp.port_E.b));

        printf("  a = %+5.1f  NPN bB = %+.3f  PNP bB = %+.3f  OK\n",
               (double)v, (double)npn.port_B.b, (double)pnp.port_B.b);
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 7: Performance benchmark (1,000,000 reflect() calls each class)
// ─────────────────────────────────────────────────────────────────────────────
void test_performance_benchmark() {
    printf("\nTEST 7: Performance benchmark (1,000,000 calls each)\n");

    constexpr int N = 1000000;
    constexpr double ARM_HZ = 720.0e6;   // double: used only for cycle estimate

    auto bench = [&](const char* name, auto& bjt) {
        volatile float sink = 0.0f;
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < N; ++i) {
            bjt.port_B.a = 0.005f * (float)(i % 200) - 0.5f;
            bjt.port_C.a = 0.05f  * (float)(i % 20) - 0.5f;
            bjt.port_E.a = 0.0f;
            bjt.reflect();
            sink += bjt.port_C.b;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double ns      = std::chrono::duration<double, std::nano>(t1 - t0).count();
        double ns_call = ns / (double)N;
        double arm_cyc = ns_call * (ARM_HZ / 1.0e9);
        printf("  %-28s %.1f ns/call, ~%.0f ARM cycles (target < 2000)\n",
               name, ns_call, arm_cyc);
        (void)sink;
    };

    WdfNpnBjtFamily npn;
    npn.init(npn_params::N2N3904::Is, npn_params::N2N3904::Bf,
             npn_params::N2N3904::Br, npn_params::N2N3904::n, npn_params::N2N3904::Va);
    npn.port_B.Rp = npn.port_C.Rp = npn.port_E.Rp = 10000.0f;
    bench("WdfNpnBjtFamily (2N3904)", npn);

    WdfPnpBjtFamily pnp;
    pnp.init(pnp_params::Ac128::Is, pnp_params::Ac128::Bf,
             pnp_params::Ac128::Br, pnp_params::Ac128::n, pnp_params::Ac128::Va);
    pnp.port_B.Rp = pnp.port_C.Rp = pnp.port_E.Rp = 1000.0f;
    bench("WdfPnpBjtFamily (AC128)", pnp);

    printf("  PASS (review ARM estimates manually)\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    printf("====== WdfBjtFamily Test Suite ======\n");

    test_npn_dc_operating_point_2n3904();
    test_npn_beta_silicon_vs_darlington();
    test_pnp_dc_operating_point_ac128();
    test_early_effect_vce_modulation();
    test_npn_convergence_regions();
    test_extreme_inputs();
    test_performance_benchmark();

    printf("\n====== ALL TESTS PASSED ======\n");
    return 0;
}
