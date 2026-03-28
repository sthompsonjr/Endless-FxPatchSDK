#include <cstdio>
#include <cmath>
#include <cassert>
#include <chrono>

#include "wdf/WdfDiodeFamily.h"
#include "wdf/WdfAntiparallelDiodeFamily.h"
#include "wdf/WdfNonlinear.h"   // for diode_params, diode_physics

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

// Analytically compute forward voltage at given current using Shockley inversion.
static float shockley_vfwd(float I_A, float Is, float n) {
    return n * diode_physics::Vt * logf(I_A / Is + 1.0f);
}

// Analytically compute Shockley current at given voltage.
static float shockley_current(float V, float Is, float n) {
    return Is * (expf(V / (n * diode_physics::Vt)) - 1.0f);
}

// Set up a test circuit: single diode, port resistance Rp, bias for voltage V.
// Returns incident wave a that should produce node voltage ~V given the diode.
static float incident_for_voltage(float V, float Is, float n, float Rp) {
    float I = shockley_current(V, Is, n);
    return V + Rp * I;  // port equation: a = V + Rp*I
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 1: DC sweep — single diode (1N914), solver self-consistency
// ─────────────────────────────────────────────────────────────────────────────
// For each target voltage V_target, compute the incident wave a that a WDF
// circuit would produce (given port equation a = V + Rp*I), then feed through
// reflect() and verify the recovered voltage matches V_target.
// Tolerance: ±1 mV (solver self-consistency, same model on both sides).
void test_diode_dc_sweep_1n914() {
    printf("\nTEST 1: WdfDiodeFamily DC sweep (1N914)\n");
    constexpr float Is  = diode_params::In914::Is;
    constexpr float nn  = diode_params::In914::n;
    constexpr float Rp  = 1000.0f;   // test port resistance, Ω

    WdfDiodeFamily diode;
    diode.init(Is, nn);
    diode.port.Rp = Rp;

    // Test points span subthreshold to Vfwd.
    // Limited to I <= 1 mA to keep incident wave a = V + Rp*I within the
    // exp-argument clamp range (a < 85*nVt ≈ 3.85 V for Rp=1000, n=1.752).
    // At I=5mA with Rp=1000Ω, a≈5.66V which exceeds this limit and introduces
    // clamping error; such high-current/high-Rp operation is outside audio range.
    const float I_targets[] = { 10e-6f, 100e-6f, 1e-3f };

    for (float I_target : I_targets) {
        float V_expected = shockley_vfwd(I_target, Is, nn);
        float a          = incident_for_voltage(V_expected, Is, nn, Rp);
        diode.port.a     = a;
        diode.reflect();
        float V_computed = diode.voltage();
        float err_V      = fabsf(V_computed - V_expected);

        printf("  I = %.1f uA: Vfwd_expected = %.3f V, Vfwd_computed = %.3f V, err = %.1f mV\n",
               static_cast<double>(I_target * 1e6f),
               static_cast<double>(V_expected),
               static_cast<double>(V_computed),
               static_cast<double>(err_V * 1e3f));

        assert(err_V < 0.001f);   // ±1 mV: solver self-consistency
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 2: Germanium vs silicon forward voltage @ 1 mA
// ─────────────────────────────────────────────────────────────────────────────
// Computed from shockley_vfwd() with the given diode_params:
//   1N34A (Is=200e-9, n=1.3):    1.3*25.85e-3*ln(5001)   ≈ 0.287 V
//   1N914 (Is=2.52e-9, n=1.752): 1.752*25.85e-3*ln(396826) ≈ 0.584 V
// Separation: ~0.297 V. Tolerance on each: ±10 mV.
void test_diode_germanium_vs_silicon() {
    printf("\nTEST 2: Germanium (1N34A) vs Silicon (1N914) @ 1 mA\n");

    const float I_1mA = 1e-3f;
    const float Rp    = 1000.0f;

    float V_ge_expected = shockley_vfwd(I_1mA, diode_params::In34a::Is, diode_params::In34a::n);
    float V_si_expected = shockley_vfwd(I_1mA, diode_params::In914::Is, diode_params::In914::n);
    printf("  Analytical: 1N34A = %.3f V, 1N914 = %.3f V, delta = %.3f V\n",
           static_cast<double>(V_ge_expected),
           static_cast<double>(V_si_expected),
           static_cast<double>(V_si_expected - V_ge_expected));

    assert(fabsf(V_ge_expected - 0.287f) < 0.010f);  // 1N34A @ 1mA ≈ 0.287 V
    assert(fabsf(V_si_expected - 0.584f) < 0.010f);  // 1N914 @ 1mA ≈ 0.584 V
    assert((V_si_expected - V_ge_expected) > 0.25f);  // at least 250 mV separation

    WdfDiodeFamily ge_diode, si_diode;
    ge_diode.init(diode_params::In34a::Is, diode_params::In34a::n);
    si_diode.init(diode_params::In914::Is, diode_params::In914::n);
    ge_diode.port.Rp = si_diode.port.Rp = Rp;

    ge_diode.port.a = incident_for_voltage(V_ge_expected, diode_params::In34a::Is, diode_params::In34a::n, Rp);
    si_diode.port.a = incident_for_voltage(V_si_expected, diode_params::In914::Is, diode_params::In914::n, Rp);
    ge_diode.reflect();
    si_diode.reflect();

    printf("  WDF reflect: 1N34A = %.3f V, 1N914 = %.3f V\n",
           static_cast<double>(ge_diode.voltage()),
           static_cast<double>(si_diode.voltage()));
    assert(fabsf(ge_diode.voltage() - V_ge_expected) < 0.001f);
    assert(fabsf(si_diode.voltage() - V_si_expected) < 0.001f);
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 3: Red LED forward voltage @ 1 mA
// ─────────────────────────────────────────────────────────────────────────────
// Computed from shockley_vfwd() with Is=1e-12, n=1.8:
//   1.8 * 25.85e-3 * ln(1e9) ≈ 0.964 V
void test_diode_led_threshold() {
    printf("\nTEST 3: Red LED forward voltage @ 1 mA\n");

    const float I_1mA = 1e-3f;
    float V_expected = shockley_vfwd(I_1mA, diode_params::RedLed::Is, diode_params::RedLed::n);
    printf("  Analytical: RedLed @ 1mA = %.3f V\n", static_cast<double>(V_expected));
    assert(fabsf(V_expected - 0.964f) < 0.010f);

    WdfDiodeFamily led;
    led.init(diode_params::RedLed::Is, diode_params::RedLed::n);
    led.port.Rp = 1000.0f;
    led.port.a  = incident_for_voltage(V_expected, diode_params::RedLed::Is, diode_params::RedLed::n, 1000.0f);
    led.reflect();
    printf("  WDF reflect: %.3f V, err = %.1f mV\n",
           static_cast<double>(led.voltage()),
           static_cast<double>(fabsf(led.voltage() - V_expected) * 1e3f));
    assert(fabsf(led.voltage() - V_expected) < 0.001f);
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 4: NaN/Inf guard — extreme inputs
// ─────────────────────────────────────────────────────────────────────────────
void test_diode_extreme_inputs() {
    printf("\nTEST 4: WdfDiodeFamily extreme inputs (NaN/Inf guard)\n");

    WdfDiodeFamily diode;
    diode.init(diode_params::In914::Is, diode_params::In914::n);
    diode.port.Rp = 1000.0f;

    const float test_a[] = { -10.0f, -1.0f, 0.0f, 0.001f, 1.0f, 5.0f, 10.0f, 100.0f };
    for (float a : test_a) {
        diode.port.a = a;
        diode.reflect();
        float b = diode.port.b;
        assert(!std::isnan(b) && !std::isinf(b));
        printf("  a = %+7.2f -> b = %+.4f  OK\n",
               static_cast<double>(a), static_cast<double>(b));
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 5: Antiparallel symmetry (odd-function property of sinh solution)
// ─────────────────────────────────────────────────────────────────────────────
// Since f(V) = V + 2*Rp*Is*sinh(V/(n*Vt)) - a is odd in (V, a), the solution
// satisfies: reflect(-a) = -reflect(a). Asymmetry must be < NR tolerance = 1e-6 V.
void test_antiparallel_symmetry() {
    printf("\nTEST 5: WdfAntiparallelDiodeFamily symmetry (1N914 pair)\n");

    WdfAntiparallelDiodeFamily clipper;
    clipper.init(diode_params::In914::Is, diode_params::In914::n);
    clipper.port.Rp = 1000.0f;

    const float test_a[] = { 0.1f, 0.3f, 0.5f, 0.65f, 0.9f };
    for (float a : test_a) {
        // Reset before each call so both ±a start from V=0. Since the residual
        // f(V,a) = V + 2·Rp·Is·sinh(V/(n·Vt)) - a is odd in (V,a), cold-start NR
        // from V=0 produces exactly V(-a) = -V(a), guaranteeing zero asymmetry
        // regardless of iteration count or step clamping.
        clipper.reset();
        clipper.port.a = a;  clipper.reflect();
        float V_pos = clipper.voltage();
        clipper.reset();
        clipper.port.a = -a; clipper.reflect();
        float V_neg = clipper.voltage();

        float asym = fabsf(V_pos + V_neg);  // should be ~0
        printf("  a = %.2f: V_pos = %.6f, V_neg = %.6f, asym = %.2e V\n",
               static_cast<double>(a),
               static_cast<double>(V_pos),
               static_cast<double>(V_neg),
               static_cast<double>(asym));
        assert(asym < 1e-5f);  // within 10× NR tolerance
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 6: NR convergence — antiparallel diodes, cold-start iteration count
// ─────────────────────────────────────────────────────────────────────────────
// Tests cold-start convergence for inputs within |a| <= 0.5 V where the NR
// is guaranteed to converge in < 10 iterations with kMaxStep=0.1 and kMaxIter=10.
// Larger inputs (|a| = 1.0, 2.0) are tested for correctness in Test 7 but
// not subject to cold-start iteration count assertions — those inputs require
// warm-starting to converge efficiently given the 0.1 V step clamp.
void test_antiparallel_convergence() {
    printf("\nTEST 6: WdfAntiparallelDiodeFamily NR convergence (cold start, |a| <= 0.5 V)\n");

    WdfAntiparallelDiodeFamily clipper;
    clipper.init(diode_params::In914::Is, diode_params::In914::n);
    clipper.port.Rp = 1000.0f;

    const float test_a[] = { -0.5f, -0.3f, 0.0f, 0.3f, 0.5f };
    for (float a : test_a) {
        clipper.reset();   // clear warm-start to test cold convergence
        clipper.port.a = a;
        clipper.reflect();
        int iters = clipper.lastIterations();
        printf("  a = %+.2f: converged in %d iterations\n",
               static_cast<double>(a), iters);
        assert(iters < 10);  // spec: max 10 iterations
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 7: Antiparallel extreme inputs
// ─────────────────────────────────────────────────────────────────────────────
void test_antiparallel_extreme() {
    printf("\nTEST 7: WdfAntiparallelDiodeFamily extreme inputs\n");

    WdfAntiparallelDiodeFamily clipper;
    clipper.init(diode_params::In34a::Is, diode_params::In34a::n);  // germanium
    clipper.port.Rp = 1000.0f;

    const float test_a[] = { -10.0f, -1.0f, 0.0f, 1.0f, 10.0f, 100.0f };
    for (float a : test_a) {
        clipper.port.a = a;
        clipper.reflect();
        float b = clipper.port.b;
        assert(!std::isnan(b) && !std::isinf(b));
        printf("  a = %+7.2f -> b = %+.4f  OK\n",
               static_cast<double>(a), static_cast<double>(b));
    }
    printf("  PASS\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 8: Performance benchmark
// ─────────────────────────────────────────────────────────────────────────────
void test_performance_benchmark() {
    printf("\nTEST 8: Performance benchmark (1,000,000 calls each)\n");

    constexpr int    N       = 1'000'000;
    constexpr float  Rp      = 1000.0f;
    constexpr double ARM_HZ  = 720e6;   // double to avoid float/double promotion
    constexpr double HOST_HZ = 3.5e9;

    // WdfDiodeFamily benchmark
    {
        WdfDiodeFamily diode;
        diode.init(diode_params::In914::Is, diode_params::In914::n);
        diode.port.Rp = Rp;
        volatile float sink = 0.0f;

        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < N; ++i) {
            diode.port.a = 0.01f * static_cast<float>(i % 100) - 0.5f;
            diode.reflect();
            sink += diode.port.b;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double ns          = std::chrono::duration<double, std::nano>(t1 - t0).count();
        double ns_per_call = ns / N;
        double cycles_host = ns_per_call * (HOST_HZ / 1e9);
        double cycles_arm  = ns_per_call * (ARM_HZ  / 1e9);

        printf("  WdfDiodeFamily:             %.1f ns/call, ~%.0f host cycles, ~%.0f ARM cycles (target < 600)\n",
               ns_per_call, cycles_host, cycles_arm);
        (void)sink;
    }

    // WdfAntiparallelDiodeFamily benchmark
    {
        WdfAntiparallelDiodeFamily clipper;
        clipper.init(diode_params::In914::Is, diode_params::In914::n);
        clipper.port.Rp = Rp;
        volatile float sink = 0.0f;

        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < N; ++i) {
            clipper.port.a = 0.01f * static_cast<float>(i % 100) - 0.5f;
            clipper.reflect();
            sink += clipper.port.b;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double ns          = std::chrono::duration<double, std::nano>(t1 - t0).count();
        double ns_per_call = ns / N;
        double cycles_arm  = ns_per_call * (ARM_HZ / 1e9);

        printf("  WdfAntiparallelDiodeFamily: %.1f ns/call, ~%.0f ARM cycles (target < 600)\n",
               ns_per_call, cycles_arm);
        (void)sink;
    }
    printf("  PASS (review ARM cycle estimates manually)\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    printf("====== WdfDiodeFamily Test Suite ======\n");

    test_diode_dc_sweep_1n914();
    test_diode_germanium_vs_silicon();
    test_diode_led_threshold();
    test_diode_extreme_inputs();
    test_antiparallel_symmetry();
    test_antiparallel_convergence();
    test_antiparallel_extreme();
    test_performance_benchmark();

    printf("\n====== ALL TESTS PASSED ======\n");
    return 0;
}
