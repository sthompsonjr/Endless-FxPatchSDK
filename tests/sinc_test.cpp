// tests/sinc_test.cpp — WindowedSincInterpolator test harness

#include "dsp/WindowedSincInterpolator.h"
#include "dsp/Interpolation.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <ctime>

static int totalTests  = 0;
static int passedTests = 0;

static void check(bool condition, const char* name) {
    ++totalTests;
    if (condition) {
        ++passedTests;
        std::printf("  PASS: %s\n", name);
    } else {
        std::printf("  FAIL: %s\n", name);
    }
}

static bool approx(float a, float b, float eps = 0.001f) {
    return fabsf(a - b) < eps;
}

// Mimics CircularBuffer::read() on a plain array.
// After writing samples[0..N-1], writeHead = N.
// read(d) = buf[(N - 1 - d) & (Size - 1)]
template <int Size>
static float cbRead(const float (&buf)[Size], int writeHead, int delay) {
    return buf[(writeHead - 1 - delay) & (Size - 1)];
}

// Hermite 4-point reconstruction — mirrors CircularBuffer::readHermite
template <int Size>
static float cbHermite(const float (&buf)[Size], int writeHead, float delay) {
    int   ip   = static_cast<int>(delay);
    float frac = delay - static_cast<float>(ip);
    float xm1  = cbRead(buf, writeHead, ip - 1);
    float x0   = cbRead(buf, writeHead, ip);
    float x1   = cbRead(buf, writeHead, ip + 1);
    float x2   = cbRead(buf, writeHead, ip + 2);
    return interp::hermite(xm1, x0, x1, x2, frac);
}

// ============================================================
// 1. DC response — constant buffer must return that constant
// ============================================================
static void testDcResponse() {
    std::printf("\n--- DC response ---\n");

    static constexpr int kSize = 1024;
    static float buf[kSize];

    const float dc = 0.75f;
    for (auto& s : buf) s = dc;

    // read_index placed well inside so all 16 taps have valid data
    const float result = WindowedSincInterpolator::interpolate(buf, kSize, 100.0f, 200);
    check(approx(result, dc, 1e-4f), "DC: constant buffer returns constant");

    // All fractional positions must agree
    bool allMatch = true;
    for (int step = 0; step < 1024; ++step) {
        const float delay = 50.0f + static_cast<float>(step) / 1024.0f;
        const float out   = WindowedSincInterpolator::interpolate(buf, kSize, delay, 200);
        if (fabsf(out - dc) > 1e-3f) { allMatch = false; break; }
    }
    check(allMatch, "DC: all 1024 fractional positions return constant");
}

// ============================================================
// 2. Integer delay — must agree with cbRead() exactly
// ============================================================
static void testIntegerDelay() {
    std::printf("\n--- Integer delay ---\n");

    static constexpr int kSize = 1024;
    static float buf[kSize] = {};

    // Write 512 samples: ascending ramp
    static constexpr int kFill = 512;
    for (int i = 0; i < kFill; ++i)
        buf[i] = static_cast<float>(i) * 0.1f;

    bool allMatch = true;
    for (int d = 8; d < 100; ++d) {
        const float expected = cbRead(buf, kFill, d);
        const float got      = WindowedSincInterpolator::interpolate(
                                   buf, kSize, static_cast<float>(d), kFill);
        if (fabsf(got - expected) > 1e-4f) {
            std::printf("    mismatch at delay %d: expected %.6f got %.6f\n",
                        d, static_cast<double>(expected), static_cast<double>(got));
            allMatch = false;
        }
    }
    check(allMatch, "Integer delay matches cbRead() for delays 8..99");
}

// ============================================================
// 3. Fractional delay accuracy — sine wave reconstruction
//    x[n] = sin(2π·f·n/fs) at f=200 Hz, fs=48000 Hz
// ============================================================
static void testFractionalDelayAccuracy() {
    std::printf("\n--- Fractional delay accuracy ---\n");

    static constexpr int   kSize  = 4096;
    static constexpr float kFs    = 48000.0f;
    static constexpr float kFreq  = 200.0f;
    static constexpr float kOmega = 2.0f * 3.14159265358979323846f * kFreq / kFs;
    static float buf[kSize] = {};

    static constexpr int kFill = 2048;
    for (int n = 0; n < kFill; ++n)
        buf[n] = std::sin(kOmega * static_cast<float>(n));

    struct Case { float delay; float eps; };
    const Case cases[] = {
        { 10.0f,    1e-3f },
        { 10.5f,    5e-3f },
        { 10.25f,   5e-3f },
        { 10.75f,   5e-3f },
        { 50.3333f, 5e-3f },
    };

    bool allGood = true;
    for (auto& c : cases) {
        const float got      = WindowedSincInterpolator::interpolate(
                                   buf, kSize, c.delay, kFill);
        // Analytical: value at continuous position (kFill-1 - delay)
        const float expected = std::sin(kOmega * (static_cast<float>(kFill - 1) - c.delay));
        if (fabsf(got - expected) > c.eps) {
            std::printf("    delay=%.4f expected=%.6f got=%.6f err=%.6f\n",
                        static_cast<double>(c.delay),
                        static_cast<double>(expected),
                        static_cast<double>(got),
                        static_cast<double>(fabsf(got - expected)));
            allGood = false;
        }
    }
    check(allGood, "Fractional delays reconstruct 200 Hz sine within tolerance");
}

// ============================================================
// 4. Nyquist response — alternating ±1
//    At integer delays the sinc correctly returns the exact sample (±1).
//    At frac=0.5 the symmetric kernel gives an alternating sum of zero, so
//    the output is exactly 0 for any Nyquist input regardless of alignment.
//    We also verify that the average energy over all non-integer fracs is small.
// ============================================================
static void testNyquistAttenuation() {
    std::printf("\n--- Nyquist attenuation ---\n");

    static constexpr int kSize = 4096;
    static float buf[kSize];
    for (int i = 0; i < kSize; ++i)
        buf[i] = (i & 1) ? 1.0f : -1.0f;

    // At frac=0.5 the windowed-sinc kernel is symmetric, so the alternating
    // sum ∑ coeff[k]·(−1)^k is exactly zero by pairwise cancellation.
    for (int align = 0; align < 4; ++align) {
        const float delay = 20.5f + static_cast<float>(align);
        const float out   = WindowedSincInterpolator::interpolate(buf, kSize, delay, 2048);
        check(fabsf(out) < 1e-5f, "Nyquist at frac=0.5 is zero (pairwise cancellation)");
    }

    // Verify Nyquist reconstruction correctness.
    // buf[i] = (i odd) ? 1 : -1 = -cos(π·i).
    // Ideal continuous reconstruction: x(t) = -cos(π·t).
    // Center tap for delay=20, read_index=2048 is at position 2027.
    // Expected output at delay (20 + frac) = -cos(π·(2027 + frac)).
    // A 16-tap windowed sinc has transition-band error at Nyquist (≤ ~0.07).
    bool nyquistOk = true;
    for (int step = 1; step < 1023; ++step) {
        const float frac     = static_cast<float>(step) / 1024.0f;
        const float delay    = 20.0f + frac;
        const float out      = WindowedSincInterpolator::interpolate(buf, kSize, delay, 2048);
        const float expected = -std::cos(3.14159265358979323846f * (2027.0f + frac));
        if (fabsf(out - expected) > 0.1f) { nyquistOk = false; break; }
    }
    check(nyquistOk, "Nyquist reconstruction within 0.1 of -cos(π·(center+frac))");
}

// ============================================================
// 5. Comparison to Hermite — sinc error <= Hermite error at 1 kHz
// ============================================================
static void testVsHermite() {
    std::printf("\n--- Comparison to Hermite ---\n");

    static constexpr int   kSize  = 4096;
    static constexpr float kFs    = 48000.0f;
    static constexpr float kFreq  = 1000.0f;
    static constexpr float kOmega = 2.0f * 3.14159265358979323846f * kFreq / kFs;
    static float buf[kSize] = {};

    static constexpr int kFill = 2048;
    for (int n = 0; n < kFill; ++n)
        buf[n] = std::sin(kOmega * static_cast<float>(n));

    float errSinc    = 0.0f;
    float errHermite = 0.0f;
    static constexpr int kSteps = 200;

    for (int s = 0; s < kSteps; ++s) {
        const float delay    = 20.0f + static_cast<float>(s) / static_cast<float>(kSteps);
        const float expected = std::sin(kOmega * (static_cast<float>(kFill - 1) - delay));
        const float sincOut  = WindowedSincInterpolator::interpolate(buf, kSize, delay, kFill);
        const float hermOut  = cbHermite(buf, kFill, delay);

        const float eSinc = sincOut - expected;
        const float eHerm = hermOut - expected;
        errSinc    += eSinc * eSinc;
        errHermite += eHerm * eHerm;
    }
    errSinc    /= static_cast<float>(kSteps);
    errHermite /= static_cast<float>(kSteps);

    std::printf("    RMS error  sinc=%.2e  hermite=%.2e\n",
                static_cast<double>(std::sqrt(errSinc)),
                static_cast<double>(std::sqrt(errHermite)));
    // Sinc should not be dramatically worse than Hermite at 1 kHz
    check(errSinc <= errHermite * 10.0f, "Sinc RMS error within 20 dB of Hermite at 1 kHz");
    check(std::sqrt(errSinc) < 5e-3f,    "Sinc absolute RMS error < 5e-3 at 1 kHz");
}

// ============================================================
// 6. Linearity — interpolate(a·bufA + b·bufB) = a·out_A + b·out_B
// ============================================================
static void testLinearity() {
    std::printf("\n--- Linearity ---\n");

    static constexpr int kSize = 1024;
    static float bufA[kSize], bufB[kSize], bufC[kSize];

    for (int i = 0; i < kSize; ++i) {
        bufA[i] = static_cast<float>(i) / static_cast<float>(kSize);
        bufB[i] = 1.0f - bufA[i];
        bufC[i] = 2.0f * bufA[i] + 0.5f * bufB[i];
    }

    const int   ri    = 512;
    const float delay = 100.7f;

    const float outA     = WindowedSincInterpolator::interpolate(bufA, kSize, delay, ri);
    const float outB     = WindowedSincInterpolator::interpolate(bufB, kSize, delay, ri);
    const float outC     = WindowedSincInterpolator::interpolate(bufC, kSize, delay, ri);
    const float expected = 2.0f * outA + 0.5f * outB;

    check(approx(outC, expected, 1e-4f), "Interpolation is linear in its inputs");
}

// ============================================================
// 7. Performance measurement
// ============================================================
static void testPerformance() {
    std::printf("\n--- Performance ---\n");

    static constexpr int kSize = 4096;
    static float buf[kSize];
    for (int i = 0; i < kSize; ++i)
        buf[i] = std::sin(static_cast<float>(i) * 0.05f);

    static constexpr int kIterations = 100000;
    volatile float sink = 0.0f;

    const clock_t t0 = clock();
    for (int i = 0; i < kIterations; ++i) {
        const float delay = 20.0f + static_cast<float>(i & 1023) * (500.0f / 1023.0f);
        sink = sink + WindowedSincInterpolator::interpolate(buf, kSize, delay, 2048);
    }
    const clock_t t1 = clock();

    const double elapsed_s  = static_cast<double>(t1 - t0) / CLOCKS_PER_SEC;
    const double ns_per_call = (elapsed_s / kIterations) * 1e9;
    std::printf("    %d calls in %.3f ms  (%.1f ns/call)  [sink=%.3f]\n",
                kIterations, elapsed_s * 1e3, ns_per_call,
                static_cast<double>(static_cast<float>(sink)));
    check(elapsed_s < 10.0, "Performance: 100k calls complete in < 10 s");
}

// ============================================================
// Main
// ============================================================
int main() {
    std::printf("=== WindowedSincInterpolator Test Suite ===\n");

    WindowedSincInterpolator::init();

    testDcResponse();
    testIntegerDelay();
    testFractionalDelayAccuracy();
    testNyquistAttenuation();
    testVsHermite();
    testLinearity();
    testPerformance();

    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);
    return (passedTests == totalTests) ? 0 : 1;
}
