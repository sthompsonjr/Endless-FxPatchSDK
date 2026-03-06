#include "../wdf/wdf.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>

static int totalTests = 0;
static int passedTests = 0;

static void check(bool condition, const char* name) {
    totalTests++;
    if (condition) {
        passedTests++;
        std::printf("  PASS: %s\n", name);
    } else {
        std::printf("  FAIL: %s\n", name);
    }
}

static bool approx(float a, float b, float eps = 0.001f) {
    return fabsf(a - b) < eps;
}

// ============================================================
// Lambert W validation
// ============================================================
static void testLambertW() {
    std::printf("\n--- Lambert W ---\n");

    check(approx(math::lambertW0(0.0f), 0.0f, 1e-4f), "W0(0) == 0");
    check(approx(math::lambertW0(1.0f), 0.5671f, 1e-3f), "W0(1) ≈ 0.5671");
    check(approx(math::lambertW0(expf(1.0f)), 1.0f, 1e-3f), "W0(e) ≈ 1.0");
    check(approx(math::lambertW0(100.0f), 3.3856f, 1e-2f), "W0(100) ≈ 3.3856");

    check(approx(math::lambertWn1(-0.1f), -3.5772f, 1e-2f), "W-1(-0.1) ≈ -3.5772");
    check(approx(math::lambertWn1(-1.0f / expf(1.0f)), -1.0f, 1e-2f), "W-1(-1/e) ≈ -1.0");

    // Verify identity: W(x) * exp(W(x)) == x
    float testVals[] = { 0.1f, 1.0f, 10.0f, 100.0f };
    bool identityOk = true;
    for (float x : testVals) {
        float w = math::lambertW0(x);
        float check_val = w * expf(w);
        if (fabsf(check_val - x) / (fabsf(x) + 1e-6f) > 0.01f) {
            identityOk = false;
        }
    }
    check(identityOk, "W0(x)*exp(W0(x)) ≈ x for test values");
}

// ============================================================
// WDF component reflection laws
// ============================================================
static void testWdfComponents() {
    std::printf("\n--- WDF Components ---\n");

    // Resistor: b = 0 for any a
    WdfResistor r;
    r.init(1000.0f);
    r.port.a = 5.0f;
    r.reflect();
    check(approx(r.port.b, 0.0f), "Resistor: b == 0 for any a");

    // Capacitor: b[n] = a[n-1] (one-sample delay)
    WdfCapacitor c;
    c.init(1e-6f, 48000.0f);
    c.port.a = 3.0f;
    c.reflect(); // b should be 0 (initial state)
    check(approx(c.port.b, 0.0f), "Capacitor: first b == 0 (initial)");
    c.port.a = 7.0f;
    c.reflect(); // b should be 3.0 (previous a)
    check(approx(c.port.b, 3.0f), "Capacitor: b[n] == a[n-1]");

    // Inductor: b[n] = -a[n-1]
    WdfInductor l;
    l.init(0.001f, 48000.0f);
    l.port.a = 4.0f;
    l.reflect(); // b should be 0 (initial -0)
    check(approx(l.port.b, 0.0f), "Inductor: first b == 0 (initial)");
    l.port.a = 2.0f;
    l.reflect(); // b should be -4.0
    check(approx(l.port.b, -4.0f), "Inductor: b[n] == -a[n-1]");

    // Ideal voltage source: b = 2*Vs - a
    WdfIdealVoltageSource vs;
    vs.init();
    vs.setVoltage(5.0f);
    vs.port.a = 3.0f;
    vs.reflect();
    check(approx(vs.port.b, 7.0f), "Voltage source: b == 2*Vs - a");

    // Resistive voltage source
    WdfResistiveVoltageSource rvs;
    rvs.init(1000.0f);
    rvs.setVoltage(5.0f);
    rvs.port.a = 3.0f;
    rvs.reflect();
    check(approx(rvs.port.b, 7.0f), "Resistive V source: b == 2*Vs - a");

    // Resistive current source: b = a + 2*Rp*Is
    WdfResistiveCurrentSource rcs;
    rcs.init(1000.0f);
    rcs.setCurrent(0.001f); // 1mA
    rcs.port.a = 1.0f;
    rcs.reflect();
    check(approx(rcs.port.b, 3.0f), "Current source: b == a + 2*Rp*Is");

    // WdfPort voltage/current
    WdfPort p;
    p.Rp = 100.0f;
    p.a = 6.0f;
    p.b = 4.0f;
    check(approx(p.voltage(), 5.0f), "Port voltage = 0.5*(a+b)");
    check(approx(p.current(), 0.01f), "Port current = (a-b)/(2*Rp)");
}

// ============================================================
// Newton-Raphson solver
// ============================================================
static void testNewtonRaphson() {
    std::printf("\n--- Newton-Raphson ---\n");

    // x^2 - 2 = 0 → x = sqrt(2) ≈ 1.4142
    auto r1 = math::newtonRaphson(
        1.0f,
        [](float x) { return x * x - 2.0f; },
        [](float x) { return 2.0f * x; }
    );
    check(approx(r1.value, 1.4142f, 1e-3f), "NR: sqrt(2) ≈ 1.4142");
    check(r1.converged, "NR: sqrt(2) converged");

    // exp(x) - 3 = 0 → x = ln(3) ≈ 1.0986
    auto r2 = math::newtonRaphson(
        1.0f,
        [](float x) { return expf(x) - 3.0f; },
        [](float x) { return expf(x); }
    );
    check(approx(r2.value, 1.0986f, 1e-3f), "NR: ln(3) ≈ 1.0986");
    check(r2.converged, "NR: ln(3) converged");

    // Near-zero derivative: x^3 = 0 starting near 0
    auto r3 = math::newtonRaphson(
        0.001f,
        [](float x) { return x * x * x; },
        [](float x) { return 3.0f * x * x; }
    );
    // Should not produce NaN
    check(std::isfinite(r3.value), "NR: no NaN on near-zero derivative");

    // Halley's method
    auto r4 = math::halley(
        1.0f,
        [](float x) { return x * x - 2.0f; },
        [](float x) { return 2.0f * x; },
        [](float) { return 2.0f; }
    );
    check(approx(r4.value, 1.4142f, 1e-3f), "Halley: sqrt(2) ≈ 1.4142");
}

// ============================================================
// Diode model physical correctness
// ============================================================
static void testDiodeModels() {
    std::printf("\n--- Diode Models ---\n");

    // WdfIdealDiode: verify monotonically increasing I-V
    WdfIdealDiode diode;
    diode.init(1e-7f, 0.02585f, 1000.0f);

    bool monotonic = true;
    float prevDiodeV = -1.0f;
    for (int i = 0; i < 100; ++i) {
        float a = -0.5f + static_cast<float>(i) * 0.012f; // sweep -0.5 to 0.7
        diode.port.a = a;
        diode.reflect();
        float V = diode.port.voltage();
        if (V < prevDiodeV - 1e-4f) monotonic = false;
        prevDiodeV = V;
    }
    check(monotonic, "Ideal diode: V(a) is monotonically increasing");

    // WdfAntiparallelDiodes: verify odd symmetry f(-x) ≈ -f(x)
    WdfAntiparallelDiodes apd;
    apd.init(1e-7f, 0.02585f, 1000.0f);

    bool symmetric = true;
    for (int i = 1; i < 20; ++i) {
        float a = static_cast<float>(i) * 0.05f;

        apd.port.a = a;
        apd.reflect();
        float bPos = apd.port.b;

        apd.port.a = -a;
        apd.reflect();
        float bNeg = apd.port.b;

        if (fabsf(bPos + bNeg) > 1e-3f) {
            symmetric = false;
            break;
        }
    }
    check(symmetric, "Antiparallel diodes: odd symmetry f(-x) == -f(x)");
}

// ============================================================
// Circuit tests
// ============================================================
static void testRCLowpass() {
    std::printf("\n--- RC Lowpass Circuit ---\n");

    RCLowpassCircuit rc;
    // fc = 1/(2*pi*10k*10nF) ≈ 1.6kHz
    rc.init(10000.0f, 10e-9f, 48000.0f);

    // DC test: feed 0.5V, verify steady-state output approaches 0.5V
    float out = 0.0f;
    for (int i = 0; i < 48000; ++i) { // 1 second
        out = rc.process(0.5f);
    }
    check(approx(out, 0.5f, 0.05f), "RC LP: DC input → steady-state matches");

    // Step response: no overshoot (passive circuit)
    rc.reset();
    float maxOut = 0.0f;
    for (int i = 0; i < 48000; ++i) {
        out = rc.process(1.0f);
        if (out > maxOut) maxOut = out;
    }
    check(maxOut <= 1.05f, "RC LP: step response has no overshoot");
}

static void testDiodeClipper() {
    std::printf("\n--- Diode Clipper Circuit ---\n");

    DiodeClipperCircuit dc;
    // R_input = 4.7kΩ — matches Tube Screamer input resistor
    dc.init(4700.0f, DiodeClipperCircuit::DiodeType::Silicon, 48000.0f);

    // Sine wave at full amplitude — verify clipping occurs
    bool noNaN = true;
    float maxOut = 0.0f;
    for (int i = 0; i < 10000; ++i) {
        float input = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
        float out = dc.process(input);
        if (!std::isfinite(out)) { noNaN = false; break; }
        if (fabsf(out) > maxOut) maxOut = fabsf(out);
    }
    check(noNaN, "Diode clipper: no NaN/Inf in 10000 samples");
    check(maxOut <= 1.0f, "Diode clipper: output clamped to [-1, 1]");
    check(maxOut > 0.1f, "Diode clipper: produces audible output");

    // Germanium should clip softer (lower threshold)
    DiodeClipperCircuit dcGe;
    dcGe.init(4700.0f, DiodeClipperCircuit::DiodeType::Germanium, 48000.0f);
    float maxGe = 0.0f;
    for (int i = 0; i < 10000; ++i) {
        float input = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f);
        float out = dcGe.process(input);
        if (fabsf(out) > maxGe) maxGe = fabsf(out);
    }
    check(maxGe > 0.05f, "Germanium clipper: produces output");
}

static void testToneStack() {
    std::printf("\n--- Tone Stack Circuit ---\n");

    // Measure energy at 10kHz with tone=0 vs tone=1
    auto measureHighFreqEnergy = [](float toneVal) {
        ToneStackCircuit ts;
        ts.init(48000.0f);
        ts.setTone(toneVal);
        float energy = 0.0f;
        for (int i = 0; i < 48000; ++i) {
            // 10kHz sine
            float input = sinf(static_cast<float>(i) * 6.283185307f * 10000.0f / 48000.0f) * 0.5f;
            float out = ts.process(input);
            if (i > 4800) energy += out * out; // skip settling
        }
        return energy;
    };

    float energyDark = measureHighFreqEnergy(0.0f);
    float energyBright = measureHighFreqEnergy(1.0f);
    check(energyBright > energyDark, "Tone stack: more treble energy with tone=1 vs tone=0");

    // Basic functionality: produces output
    ToneStackCircuit ts;
    ts.init(48000.0f);
    ts.setTone(0.5f);
    float maxOut = 0.0f;
    for (int i = 0; i < 4800; ++i) {
        float out = ts.process(sinf(static_cast<float>(i) * 0.1f) * 0.5f);
        if (fabsf(out) > maxOut) maxOut = fabsf(out);
    }
    check(maxOut > 0.01f, "Tone stack: produces output at center position");
}

static void testBJTGainStage() {
    std::printf("\n--- BJT Gain Stage Circuit ---\n");

    BJTGainStageCircuit bjt;
    bjt.init(BJTGainStageCircuit::TransistorType::Germanium, 48000.0f);

    // DC bias test: 0 input for 1000 samples, verify output settles
    bool settled = true;
    float lastOut = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        float out = bjt.process(0.0f);
        if (!std::isfinite(out)) { settled = false; break; }
        lastOut = out;
    }
    check(settled && std::isfinite(lastOut), "BJT: DC bias settles to finite value");

    // Signal test: produces output with sine input
    bjt.reset();
    float maxOut = 0.0f;
    bool noNaN = true;
    for (int i = 0; i < 10000; ++i) {
        float input = sinf(static_cast<float>(i) * 6.283185307f * 440.0f / 48000.0f) * 0.3f;
        float out = bjt.process(input);
        if (!std::isfinite(out)) { noNaN = false; break; }
        if (fabsf(out) > maxOut) maxOut = fabsf(out);
    }
    check(noNaN, "BJT: no NaN/Inf with sine input");
    check(maxOut > 0.01f, "BJT: produces audible output");
    check(maxOut <= 1.0f, "BJT: output clamped to [-1, 1]");

    // Silicon BJT should also work
    BJTGainStageCircuit bjtSi;
    bjtSi.init(BJTGainStageCircuit::TransistorType::Silicon, 48000.0f);
    bool siOk = true;
    for (int i = 0; i < 1000; ++i) {
        float out = bjtSi.process(0.0f);
        if (!std::isfinite(out)) { siOk = false; break; }
    }
    check(siOk, "BJT silicon: DC bias settles");
}

// ============================================================
// Performance estimate
// ============================================================
static void testPerformance() {
    std::printf("\n--- Performance Estimate ---\n");

    constexpr int kSamples = 100000;
    float input[256]; // reuse buffer
    for (int i = 0; i < 256; ++i) {
        input[i] = sinf(static_cast<float>(i) * 0.1f) * 0.5f;
    }

    // RC Lowpass
    {
        RCLowpassCircuit rc;
        rc.init(10000.0f, 10e-9f, 48000.0f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = rc.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: RCLowpass: %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }

    // Diode Clipper
    {
        DiodeClipperCircuit dc;
        dc.init(4700.0f, DiodeClipperCircuit::DiodeType::Silicon, 48000.0f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = dc.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: DiodeClipper: %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }

    // Tone Stack
    {
        ToneStackCircuit ts;
        ts.init(48000.0f);
        ts.setTone(0.5f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = ts.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: ToneStack: %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }

    // BJT Gain Stage
    {
        BJTGainStageCircuit bjt;
        bjt.init(BJTGainStageCircuit::TransistorType::Germanium, 48000.0f);
        auto start = std::chrono::high_resolution_clock::now();
        float dummy = 0.0f;
        for (int i = 0; i < kSamples; ++i) dummy = bjt.process(input[i & 255]);
        auto end = std::chrono::high_resolution_clock::now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        (void)dummy;
        std::printf("  INFO: BJTGainStage: %lld ns/sample\n", static_cast<long long>(ns / kSamples));
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    std::printf("=== WDF Primitives Test Suite ===\n");

    testLambertW();
    testWdfComponents();
    testNewtonRaphson();
    testDiodeModels();
    testRCLowpass();
    testDiodeClipper();
    testToneStack();
    testBJTGainStage();
    testPerformance();

    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);

    return (passedTests == totalTests) ? 0 : 1;
}
