#include "PitchDetector.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

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

static bool approx(float a, float b, float eps = 1.0f) {
    return fabsf(a - b) < eps;
}

// Generate enough samples of a sine wave and feed to detector.
// Returns the last detected pitch (Hz) and sets confidenceOut.
static float detectPitch(float freqHz, float sampleRate, float& confidenceOut) {
    PitchDetector det;
    det.init(sampleRate);

    // Feed several windows worth of samples to allow convergence
    int numSamples = pitch_constants::window_size * 8;
    float lastPitch = 0.0f;
    confidenceOut = 0.0f;

    for (int i = 0; i < numSamples; ++i) {
        float t = static_cast<float>(i) / sampleRate;
        float sample = sinf(6.283185307f * freqHz * t);
        if (det.process(sample)) {
            lastPitch = det.getPitch();
            confidenceOut = det.getConfidence();
        }
    }
    return lastPitch;
}

// ============================================================
// Test: 440 Hz sine wave
// ============================================================
static void testSine440Hz() {
    std::printf("\n--- Sine wave 440 Hz ---\n");

    float conf = 0.0f;
    float pitch = detectPitch(440.0f, 48000.0f, conf);

    // Allow ±10% tolerance — coarse-to-fine on a clean sine should be close
    float tolerance = 440.0f * 0.10f;
    check(approx(pitch, 440.0f, tolerance), "440 Hz detected within 10%");
    check(conf > pitch_constants::min_confidence, "confidence above threshold for pure tone");
}

// ============================================================
// Test: Low frequency E2 (~82 Hz)
// ============================================================
static void testLowFrequency82Hz() {
    std::printf("\n--- Low frequency 82 Hz (E2) ---\n");

    float conf = 0.0f;
    float pitch = detectPitch(82.0f, 48000.0f, conf);

    float tolerance = 82.0f * 0.15f;
    check(approx(pitch, 82.0f, tolerance), "82 Hz detected within 15%");
}

// ============================================================
// Test: High frequency 1318 Hz (E6)
// ============================================================
static void testHighFrequency1318Hz() {
    std::printf("\n--- High frequency 1318 Hz (E6) ---\n");

    float conf = 0.0f;
    float pitch = detectPitch(1318.0f, 48000.0f, conf);

    float tolerance = 1318.0f * 0.15f;
    check(approx(pitch, 1318.0f, tolerance), "1318 Hz detected within 15%");
}

// ============================================================
// Test: Confidence on pure tone vs. noise
// ============================================================
static void testConfidenceToneVsNoise() {
    std::printf("\n--- Confidence: pure tone vs. noise ---\n");

    // Pure tone confidence
    float toneConf = 0.0f;
    detectPitch(440.0f, 48000.0f, toneConf);

    // White noise — use a simple LCG
    PitchDetector det;
    det.init(48000.0f);
    float noiseConf = 0.0f;
    unsigned int seed = 12345;
    int numSamples = pitch_constants::window_size * 8;
    for (int i = 0; i < numSamples; ++i) {
        seed = seed * 1664525u + 1013904223u;
        float sample = static_cast<float>(static_cast<int>(seed)) / 2147483648.0f;
        if (det.process(sample)) {
            noiseConf = det.getConfidence();
        }
    }

    check(toneConf > noiseConf, "pure tone has higher confidence than noise");
}

// ============================================================
// Test: Octave jump response
// ============================================================
static void testOctaveJumpResponse() {
    std::printf("\n--- Octave jump response ---\n");

    const float sampleRate = 48000.0f;
    PitchDetector det;
    det.init(sampleRate);

    // Feed 220 Hz for several windows, then switch to 440 Hz
    int halfSamples = pitch_constants::window_size * 6;
    float lastPitchHigh = 0.0f;

    for (int i = 0; i < halfSamples; ++i) {
        float t = static_cast<float>(i) / sampleRate;
        float sample = sinf(6.283185307f * 220.0f * t);
        det.process(sample);
    }

    for (int i = 0; i < halfSamples; ++i) {
        float t = static_cast<float>(halfSamples + i) / sampleRate;
        float sample = sinf(6.283185307f * 440.0f * t);
        if (det.process(sample)) {
            lastPitchHigh = det.getPitch();
        }
    }

    // After the switch, detected pitch should be closer to 440 than to 220
    check(fabsf(lastPitchHigh - 440.0f) < fabsf(lastPitchHigh - 220.0f),
          "detector tracks octave jump from 220 Hz to 440 Hz");
}

// ============================================================
// Test: process() fires on hop boundary
// ============================================================
static void testHopBoundary() {
    std::printf("\n--- Hop boundary (process() return value) ---\n");

    PitchDetector det;
    det.init(48000.0f);

    int fireCount = 0;
    for (int i = 0; i < pitch_constants::hop_size * 4; ++i) {
        if (det.process(0.0f)) {
            ++fireCount;
        }
    }

    check(fireCount == 4, "process() returns true exactly once per hop_size samples");
}

// ============================================================
// Test: reset() clears state
// ============================================================
static void testReset() {
    std::printf("\n--- reset() ---\n");

    PitchDetector det;
    det.init(48000.0f);

    // Feed some signal so there is state
    for (int i = 0; i < 1024; ++i) {
        float sample = sinf(6.283185307f * 440.0f * static_cast<float>(i) / 48000.0f);
        det.process(sample);
    }

    det.reset();

    check(det.getPitch() == 0.0f, "getPitch() returns 0 after reset");
    check(det.getConfidence() == 0.0f, "getConfidence() returns 0 after reset");
    check(!det.isPitchValid(), "isPitchValid() returns false after reset");
}

// ============================================================
// Test: isPitchValid() threshold
// ============================================================
static void testIsPitchValid() {
    std::printf("\n--- isPitchValid() ---\n");

    float conf = 0.0f;
    detectPitch(440.0f, 48000.0f, conf);

    // Re-run a fresh detector and check isPitchValid at end
    PitchDetector det;
    det.init(48000.0f);
    bool wasValid = false;
    for (int i = 0; i < pitch_constants::window_size * 8; ++i) {
        float t = static_cast<float>(i) / 48000.0f;
        float sample = sinf(6.283185307f * 440.0f * t);
        if (det.process(sample) && det.isPitchValid()) {
            wasValid = true;
        }
    }
    check(wasValid, "isPitchValid() returns true for a clean 440 Hz tone");
}

// ============================================================
// Main
// ============================================================
int main() {
    std::printf("=== PitchDetector Test Suite ===\n");

    testSine440Hz();
    testLowFrequency82Hz();
    testHighFrequency1318Hz();
    testConfidenceToneVsNoise();
    testOctaveJumpResponse();
    testHopBoundary();
    testReset();
    testIsPitchValid();

    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);
    return (passedTests == totalTests) ? 0 : 1;
}
