#include "dsp/ReverbPrimitives.h"

#include <cassert>
#include <cmath>
#include <cstdio>
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

static float l2norm(const float* x, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; ++i) sum += x[i] * x[i];
    return sqrtf(sum);
}

// ============================================================
// hadamard8 tests
// ============================================================
static void testHadamard8() {
    std::printf("\n--- hadamard8 ---\n");

    // Test 1: Losslessness — unit impulse e0
    {
        float x[8] = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        float normIn = l2norm(x, 8);
        fdn::hadamard8(x);
        float normOut = l2norm(x, 8);
        check(approx(normOut, normIn, 1e-5f),
              "hadamard8 lossless unit impulse e0");
    }

    // Test 2: Losslessness — unit impulse e3
    {
        float x[8] = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        float normIn = l2norm(x, 8);
        fdn::hadamard8(x);
        float normOut = l2norm(x, 8);
        check(approx(normOut, normIn, 1e-5f),
              "hadamard8 lossless unit impulse e3");
    }

    // Test 3: Losslessness — non-uniform input
    {
        float x[8] = { 0.1f, 0.3f, 0.5f, 0.7f, 0.2f, 0.4f, 0.6f, 0.8f };
        float normIn = l2norm(x, 8);
        fdn::hadamard8(x);
        float normOut = l2norm(x, 8);
        check(approx(normOut, normIn, 1e-5f),
              "hadamard8 lossless non-uniform input");
    }

    // Test 4: Involutory — H8 * H8 = I
    {
        float x[8]        = { 0.1f, -0.3f, 0.5f, -0.7f, 0.2f, -0.4f, 0.6f, -0.8f };
        float original[8] = { 0.1f, -0.3f, 0.5f, -0.7f, 0.2f, -0.4f, 0.6f, -0.8f };
        fdn::hadamard8(x);
        fdn::hadamard8(x);
        bool ok = true;
        for (int i = 0; i < 8; ++i) {
            if (!approx(x[i], original[i], 1e-4f)) ok = false;
        }
        check(ok, "hadamard8 involutory (H8*H8 = I)");
    }

    // Test 5: DC input — all ones -> [sqrt(8), 0, 0, ..., 0]
    {
        float x[8] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        fdn::hadamard8(x);
        float expected0 = sqrtf(8.0f);
        bool ok = approx(x[0], expected0, 1e-5f);
        for (int i = 1; i < 8; ++i) {
            if (!approx(x[i], 0.0f, 1e-5f)) ok = false;
        }
        check(ok, "hadamard8 DC input -> sqrt(8) in output[0]");
    }

    // Test 6: Alternating input — energy concentrates in one bin
    {
        float x[8] = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f };
        fdn::hadamard8(x);
        float expected = sqrtf(8.0f);
        float maxVal = 0.0f;
        int maxIdx = -1;
        for (int i = 0; i < 8; ++i) {
            if (fabsf(x[i]) > maxVal) { maxVal = fabsf(x[i]); maxIdx = i; }
        }
        bool ok = approx(maxVal, expected, 1e-5f);
        for (int i = 0; i < 8; ++i) {
            if (i != maxIdx && !approx(x[i], 0.0f, 1e-5f)) ok = false;
        }
        check(ok, "hadamard8 alternating input concentrates energy");
    }

    // Test 7: Max magnitude bounded by L2 norm + all outputs finite
    {
        float x[8] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        fdn::hadamard8(x);
        bool ok = true;
        for (int i = 0; i < 8; ++i) {
            if (fabsf(x[i]) > sqrtf(8.0f) + 1e-4f) ok = false;
            if (!std::isfinite(x[i])) ok = false;
        }
        check(ok, "hadamard8 max magnitude bounded and finite");
    }

    // Test 8: Structural consistency with H4 decomposition
    // When x[4..7] = 0, output[0..3] == output[4..7]
    {
        float x[8] = { 0.5f, -0.3f, 0.8f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f };
        fdn::hadamard8(x);
        bool ok = true;
        for (int i = 0; i < 4; ++i) {
            if (!approx(x[i], x[i + 4], 1e-5f)) ok = false;
        }
        check(ok, "hadamard8 matches H4 structural decomposition");
    }

    // Test 9: Iterated stability — 10,000 applications, energy preserved
    {
        float x[8] = { 0.5f, 0.5f, 0.5f, 0.5f, -0.5f, -0.5f, -0.5f, -0.5f };
        float normInit = l2norm(x, 8);
        for (int i = 0; i < 8; ++i) x[i] /= normInit;

        for (int iter = 0; iter < 10000; ++iter) {
            fdn::hadamard8(x);
        }
        float normFinal = l2norm(x, 8);
        check(fabsf(normFinal - 1.0f) < 1e-3f,
              "hadamard8 iterated stability (10k iterations)");
    }

    // Test 10: Performance benchmark (informational, no assertion)
    {
        float x[8] = { 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f };
        constexpr int ITERATIONS = 1000000;
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < ITERATIONS; ++i) {
            fdn::hadamard8(x);
            x[0] += 1e-20f;  // prevent dead-code elimination
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
        double nsPerCall = ns / ITERATIONS;
        std::printf("  INFO: hadamard8 performance: %.2f ns/call\n", nsPerCall);
        check(true, "hadamard8 performance benchmark (informational)");
    }
}

int main() {
    std::printf("=== ReverbPrimitives Test Suite ===\n");
    testHadamard8();
    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);
    return (passedTests == totalTests) ? 0 : 1;
}
