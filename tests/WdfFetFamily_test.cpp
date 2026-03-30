#include <cstdio>
#include <cmath>
#include <cassert>
#include <chrono>

#include "wdf/WdfJfetFamily.h"
#include "wdf/WdfMosfetFamily.h"

// ─────────────────────────────────────────────────────────────────────────────
// TEST 1: WdfJfetFamily Rds values — J201 (Vp=-0.8V, Idss=0.6mA)
// Expected values from Section 5.3:
//   Vgs=-0.4V: Rds = 0.64 / (2 × 6e-4 × 0.4) = 1333 Ω
//   Vgs=-0.2V: Rds = 0.64 / (2 × 6e-4 × 0.6) = 889 Ω
//   Vgs=-0.7V: Rds = 0.64 / (2 × 6e-4 × 0.1) = 5333 Ω
// Tolerance: ±1% (arithmetic precision only, model is exact)
// ─────────────────────────────────────────────────────────────────────────────
void test_jfet_j201_rds() {
    printf("\nTEST 1: WdfJfetFamily Rds — J201 (Vp=-0.8V, Idss=0.6mA)\n");

    WdfJfetFamily jfet;
    jfet.init(jfet_params::J201::Vp, jfet_params::J201::Idss);

    struct TestPoint { float Vgs; float Rds_expected; };
    TestPoint pts[] = {
        { -0.4f, 1333.3f },
        { -0.2f,  888.9f },
        { -0.7f, 5333.3f },
    };
    for (const auto& p : pts) {
        float Rds = jfet.computeRds(p.Vgs);
        float err = fabsf(Rds - p.Rds_expected) / p.Rds_expected;
        printf("  Vgs = %+.2f V: Rds = %.1f Ohm (expected %.1f Ohm, err = %.2f%%)\n",
               (double)p.Vgs, (double)Rds, (double)p.Rds_expected, (double)(err * 100.0f));
        assert(err < 0.01f);  // ±1%
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 2: WdfJfetFamily Rds — 2N3819 (Vp=-3.0V, Idss=5.0mA)
// Expected values from Section 5.3:
//   Vgs=-1.5V: Rds = 9.0 / (2 × 5e-3 × 1.5) = 600 Ω
//   Vgs=-0.5V: Rds = 9.0 / (2 × 5e-3 × 2.5) = 360 Ω
// ─────────────────────────────────────────────────────────────────────────────
void test_jfet_2n3819_rds() {
    printf("\nTEST 2: WdfJfetFamily Rds — 2N3819 (Vp=-3.0V, Idss=5.0mA)\n");

    WdfJfetFamily jfet;
    jfet.init(jfet_params::N2N3819::Vp, jfet_params::N2N3819::Idss);

    struct TestPoint { float Vgs; float Rds_expected; };
    TestPoint pts[] = {
        { -1.5f, 600.0f },
        { -0.5f, 360.0f },
    };
    for (const auto& p : pts) {
        float Rds = jfet.computeRds(p.Vgs);
        float err = fabsf(Rds - p.Rds_expected) / p.Rds_expected;
        printf("  Vgs = %+.2f V: Rds = %.1f Ohm (expected %.1f Ohm, err = %.2f%%)\n",
               (double)p.Vgs, (double)Rds, (double)p.Rds_expected, (double)(err * 100.0f));
        assert(err < 0.01f);
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 3: J201 vs 2N3819 — Rds sweep range comparison
// J201 sweeps Vgs from -0.8V to 0V (range = 0.8V).
// 2N3819 sweeps Vgs from -3.0V to 0V (range = 3.0V).
// The Rds ratio at full-on vs full-off should reflect Idss and Vp differences.
// Rds_min_J201 = Vp²/(2×Idss×|Vp|) = 0.64/(2×6e-4×0.8) = 667 Ω
// Rds_min_2N3819 = 9.0/(2×5e-3×3.0) = 300 Ω
// J201 should have higher minimum Rds → lower maximum allpass pole frequency.
// ─────────────────────────────────────────────────────────────────────────────
void test_jfet_device_comparison() {
    printf("\nTEST 3: J201 vs 2N3819 Rds range comparison\n");

    WdfJfetFamily j201, n2n3819;
    j201.init(jfet_params::J201::Vp, jfet_params::J201::Idss);
    n2n3819.init(jfet_params::N2N3819::Vp, jfet_params::N2N3819::Idss);

    // Rds near Vgs=0 (fully on)
    float Rds_j201_min    = j201.computeRds(-0.01f);
    float Rds_n2n3819_min = n2n3819.computeRds(-0.01f);

    printf("  J201   Rds(Vgs~0) = %.1f Ohm (expected ~667 Ohm)\n",
           (double)Rds_j201_min);
    printf("  2N3819 Rds(Vgs~0) = %.1f Ohm (expected ~300 Ohm)\n",
           (double)Rds_n2n3819_min);

    // J201 should have significantly higher minimum Rds
    assert(Rds_j201_min > Rds_n2n3819_min * 1.5f);

    // Both should reach maximum Rds at pinch-off
    float Rds_j201_max    = j201.computeRds(jfet_params::J201::Vp * 1.05f);
    float Rds_n2n3819_max = n2n3819.computeRds(jfet_params::N2N3819::Vp * 1.05f);
    printf("  J201   Rds(near Vp) = %.1f Ohm (clamped to max)\n",
           (double)Rds_j201_max);
    printf("  2N3819 Rds(near Vp) = %.1f Ohm (clamped to max)\n",
           (double)Rds_n2n3819_max);

    assert(Rds_j201_max >= 1.0e5f);    // near kRdsMax
    assert(Rds_n2n3819_max >= 1.0e5f);
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 4: JFET pinch-off guard — no division by zero
// At Vgs = Vp exactly (headroom = 0), computeRds must return kRdsMax.
// At Vgs beyond Vp (Vgs < Vp), must also return kRdsMax (clamped).
// ─────────────────────────────────────────────────────────────────────────────
void test_jfet_pinchoff_guard() {
    printf("\nTEST 4: JFET pinch-off guard (divide-by-zero prevention)\n");

    WdfJfetFamily jfet;
    jfet.init(jfet_params::J201::Vp, jfet_params::J201::Idss);

    float at_vp     = jfet.computeRds(jfet_params::J201::Vp);
    float beyond_vp = jfet.computeRds(jfet_params::J201::Vp - 1.0f);
    float extreme   = jfet.computeRds(-100.0f);
    float pos_vgs   = jfet.computeRds(+1.0f);   // wrong polarity — must clamp

    printf("  Rds at Vgs=Vp:       %.1f Ohm\n", (double)at_vp);
    printf("  Rds at Vgs < Vp:     %.1f Ohm\n", (double)beyond_vp);
    printf("  Rds at Vgs=-100:     %.1f Ohm\n", (double)extreme);
    printf("  Rds at Vgs=+1 (bad): %.1f Ohm\n", (double)pos_vgs);

    assert(at_vp >= 1.0e5f);
    assert(beyond_vp >= 1.0e5f);
    assert(extreme >= 1.0e5f);
    assert(pos_vgs >= 1.0e5f);
    assert(!std::isnan(at_vp) && !std::isinf(at_vp));
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 5: WdfJfetFamily port.Rp update — setGateVoltage propagates to port
// ─────────────────────────────────────────────────────────────────────────────
void test_jfet_port_rp_update() {
    printf("\nTEST 5: WdfJfetFamily setGateVoltage updates port.Rp\n");

    WdfJfetFamily jfet;
    jfet.init(jfet_params::J310::Vp, jfet_params::J310::Idss);

    float Vgs_vals[] = { -1.8f, -1.0f, -0.5f, -0.1f };
    for (float Vgs : Vgs_vals) {
        jfet.setGateVoltage(Vgs);
        float expected = jfet.computeRds(Vgs);
        float actual   = jfet.port.Rp;
        assert(fabsf(actual - expected) < 0.01f);
        printf("  Vgs = %+.2f V: port.Rp = %.1f Ohm == computeRds() = %.1f Ohm OK\n",
               (double)Vgs, (double)actual, (double)expected);
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 6: WdfMosfetFamily Rds — 2N7000 (Vt=1.5V, Rds_on=50Ω)
// Expected values from Section 5.3:
//   Vgs=3.0V: Rds = 50 × 1.5 / (3.0-1.5) = 50 Ω  (matches Rds_on by design)
//   Vgs=2.0V: Rds = 50 × 1.5 / (2.0-1.5) = 150 Ω
//   Vgs=1.5V: Rds → kRdsOff (threshold, device off)
//   Vgs=1.0V: Rds = kRdsOff (below threshold)
// ─────────────────────────────────────────────────────────────────────────────
void test_mosfet_2n7000_rds() {
    printf("\nTEST 6: WdfMosfetFamily Rds — 2N7000 (Vt=1.5V, Rds_on=50Ohm)\n");

    WdfMosfetFamily mos;
    mos.init(mosfet_params::N2N7000::Vt, mosfet_params::N2N7000::Rds_on);

    struct TestPoint { float Vgs; float Rds_expected; bool should_be_off; };
    TestPoint pts[] = {
        { 3.0f,  50.0f,  false },  // fully on: Rds = Rds_on by construction
        { 2.0f, 150.0f,  false },  // partially on
        { 1.5f, 1.0e6f,  true  },  // at threshold: off
        { 1.0f, 1.0e6f,  true  },  // below threshold: off
    };

    for (const auto& p : pts) {
        float Rds = mos.computeRds(p.Vgs);
        printf("  Vgs = %.1f V: Rds = %.1f Ohm (expected %.1f Ohm)\n",
               (double)p.Vgs, (double)Rds, (double)p.Rds_expected);
        if (p.should_be_off) {
            assert(Rds >= 1.0e5f);   // off state: very high resistance
        } else {
            float err = fabsf(Rds - p.Rds_expected) / p.Rds_expected;
            assert(err < 0.01f);
        }
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 7: MOSFET threshold continuity — no discontinuity crossing Vt
// As Vgs sweeps from below Vt to above, Rds should decrease monotonically.
// No sudden jump other than the threshold clamping (which is intentional).
// ─────────────────────────────────────────────────────────────────────────────
void test_mosfet_threshold_continuity() {
    printf("\nTEST 7: WdfMosfetFamily threshold continuity\n");

    WdfMosfetFamily mos;
    mos.init(mosfet_params::Bs170::Vt, mosfet_params::Bs170::Rds_on);

    float prev_Rds = 1.0e6f;
    bool monotone = true;
    for (int i = 0; i <= 40; ++i) {
        float Vgs = 1.0f + (float)i * 0.1f;   // sweep 1.0V to 5.0V
        float Rds = mos.computeRds(Vgs);
        if (Vgs > mosfet_params::Bs170::Vt + 0.2f) {
            // Past threshold: must be monotonically decreasing
            if (Rds > prev_Rds * 1.01f) { monotone = false; }
        }
        prev_Rds = Rds;
    }
    printf("  Rds decreases monotonically above threshold: %s\n",
           monotone ? "OK" : "FAILED");
    assert(monotone);
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 8: Extreme inputs — no NaN/Inf for both classes
// ─────────────────────────────────────────────────────────────────────────────
void test_extreme_inputs() {
    printf("\nTEST 8: Extreme inputs — NaN/Inf guard\n");

    WdfJfetFamily jfet;
    jfet.init(jfet_params::J201::Vp, jfet_params::J201::Idss);

    WdfMosfetFamily mos;
    mos.init(mosfet_params::N2N7000::Vt, mosfet_params::N2N7000::Rds_on);

    const float vals[] = { -100.0f, -10.0f, -0.001f, 0.0f, 0.001f, 1.0f, 10.0f, 100.0f };
    for (float v : vals) {
        float jfet_rds = jfet.computeRds(v);
        float mos_rds  = mos.computeRds(v);
        assert(!std::isnan(jfet_rds) && !std::isinf(jfet_rds));
        assert(!std::isnan(mos_rds)  && !std::isinf(mos_rds));

        jfet.setGateVoltage(v);
        jfet.reflect();
        assert(!std::isnan(jfet.port.b) && !std::isinf(jfet.port.b));

        mos.setGateVoltage(v);
        mos.reflect();
        assert(!std::isnan(mos.port.b) && !std::isinf(mos.port.b));
    }
    printf("  All extreme inputs clean OK\n");
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 9: Performance benchmark
// ─────────────────────────────────────────────────────────────────────────────
void test_performance() {
    printf("\nTEST 9: Performance benchmark (2,000,000 setGateVoltage+reflect calls)\n");

    constexpr int    N      = 2000000;
    constexpr double ARM_HZ = 720.0e6;

    auto bench = [&](const char* name, auto& dev, float vmin, float vmax) {
        volatile float sink = 0.0f;
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < N; ++i) {
            float v = vmin + (vmax - vmin) * (float)(i % 1000) / 1000.0f;
            dev.setGateVoltage(v);
            dev.reflect();
            sink += dev.port.b + dev.port.Rp;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double ns      = std::chrono::duration<double, std::nano>(t1 - t0).count();
        double ns_call = ns / (double)N;
        double arm_cyc = ns_call * (ARM_HZ / 1.0e9);
        printf("  %-32s %.1f ns/call, ~%.0f ARM cycles (target < 200)\n",
               name, ns_call, arm_cyc);
        (void)sink;
    };

    WdfJfetFamily j201;
    j201.init(jfet_params::J201::Vp, jfet_params::J201::Idss);
    bench("WdfJfetFamily (J201)", j201, -0.7f, 0.0f);

    WdfJfetFamily n2n3819;
    n2n3819.init(jfet_params::N2N3819::Vp, jfet_params::N2N3819::Idss);
    bench("WdfJfetFamily (2N3819)", n2n3819, -2.5f, 0.0f);

    WdfMosfetFamily mos;
    mos.init(mosfet_params::N2N7000::Vt, mosfet_params::N2N7000::Rds_on);
    bench("WdfMosfetFamily (2N7000)", mos, 0.0f, 5.0f);

    printf("  PASS (review ARM estimates manually)\n");
}

int main() {
    printf("====== WdfFetFamily Test Suite ======\n");

    test_jfet_j201_rds();
    test_jfet_2n3819_rds();
    test_jfet_device_comparison();
    test_jfet_pinchoff_guard();
    test_jfet_port_rp_update();
    test_mosfet_2n7000_rds();
    test_mosfet_threshold_continuity();
    test_extreme_inputs();
    test_performance();

    printf("\n====== ALL TESTS PASSED ======\n");
    return 0;
}
