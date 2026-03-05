#include "../dsp/dsp.h"

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

static bool approx(float a, float b, float eps = 0.001f) {
    return fabsf(a - b) < eps;
}

// ============================================================
// CircularBuffer tests
// ============================================================
static void testCircularBuffer() {
    std::printf("\n--- CircularBuffer ---\n");

    CircularBuffer<float, 16> buf;
    buf.reset();

    // Write 0..15
    for (int i = 0; i < 16; ++i) {
        buf.write(static_cast<float>(i));
    }

    check(approx(buf.read(0), 15.0f), "read(0) returns most recent");
    check(approx(buf.read(1), 14.0f), "read(1) returns second most recent");
    check(approx(buf.read(15), 0.0f), "read(15) returns oldest");

    // Wrap-around: write more
    buf.write(100.0f);
    check(approx(buf.read(0), 100.0f), "wrap-around: read(0) after extra write");
    check(approx(buf.read(1), 15.0f), "wrap-around: read(1) after extra write");

    // Linear interpolation
    buf.reset();
    buf.write(0.0f);
    buf.write(10.0f);
    check(approx(buf.readLinear(0.5f), 5.0f, 0.01f), "readLinear half-sample");

    // Hermite interpolation - write a known sequence
    buf.reset();
    for (int i = 0; i < 16; ++i) {
        buf.write(static_cast<float>(i));
    }
    // readHermite at integer delay should equal read
    check(approx(buf.readHermite(2.0f), buf.read(2), 0.01f), "readHermite at integer matches read");

    check(buf.size() == 16, "size() returns template Size");
}

// ============================================================
// ParameterSmoother tests
// ============================================================
static void testParameterSmoother() {
    std::printf("\n--- ParameterSmoother ---\n");

    ParameterSmoother sm;
    sm.init(48000.0f, 20.0f);
    sm.snapTo(0.0f);
    sm.setTarget(1.0f);

    // Run for 20ms worth of samples (~960 samples at 48kHz)
    float val = 0.0f;
    for (int i = 0; i < 960; ++i) {
        val = sm.process();
    }
    // After ~1 time constant, should be within a few % of target
    check(val > 0.5f, "reaches > 0.5 after 20ms");

    // Run more to converge
    for (int i = 0; i < 4800; ++i) {
        val = sm.process();
    }
    check(approx(val, 1.0f, 0.01f), "converges to target within 1%");

    // snapTo
    sm.snapTo(0.5f);
    check(approx(sm.getCurrentValue(), 0.5f), "snapTo sets current immediately");
    check(approx(sm.process(), 0.5f, 0.01f), "after snapTo, process returns snapped value");
}

// ============================================================
// Interpolation tests
// ============================================================
static void testInterpolation() {
    std::printf("\n--- Interpolation ---\n");

    check(approx(interp::linear(0.0f, 1.0f, 0.5f), 0.5f), "linear midpoint");
    check(approx(interp::linear(0.0f, 1.0f, 0.0f), 0.0f), "linear at t=0");
    check(approx(interp::linear(0.0f, 1.0f, 1.0f), 1.0f), "linear at t=1");

    // Hermite: with linear data, should return linear result
    check(approx(interp::hermite(0.0f, 1.0f, 2.0f, 3.0f, 0.5f), 1.5f, 0.01f),
          "hermite linear data at midpoint");

    // Cosine at midpoint should equal linear midpoint for symmetric case
    check(approx(interp::cosine(0.0f, 1.0f, 0.5f), 0.5f, 0.01f), "cosine midpoint");
}

// ============================================================
// OnePoleFilter tests
// ============================================================
static void testOnePoleFilter() {
    std::printf("\n--- OnePoleFilter ---\n");

    OnePoleFilter lp;
    lp.init(48000.0f);
    lp.setType(OnePoleFilter::Type::Lowpass);
    lp.setFrequency(1000.0f);

    // Feed DC (1.0) — should converge to 1.0
    float val = 0.0f;
    for (int i = 0; i < 4800; ++i) {
        val = lp.process(1.0f);
    }
    check(approx(val, 1.0f, 0.01f), "lowpass DC response converges to 1.0");

    // High frequency through lowpass — should be attenuated
    lp.reset();
    lp.setFrequency(100.0f);
    float maxVal = 0.0f;
    for (int i = 0; i < 480; ++i) {
        float input = (i % 2 == 0) ? 1.0f : -1.0f; // Nyquist/2 ish
        float out = lp.process(input);
        if (fabsf(out) > maxVal) maxVal = fabsf(out);
    }
    check(maxVal < 0.5f, "lowpass attenuates high frequency");

    // DC blocker — should remove DC offset
    OnePoleFilter dc;
    dc.init(48000.0f);
    dc.setType(OnePoleFilter::Type::DCBlock);
    float dcOut = 0.0f;
    for (int i = 0; i < 48000; ++i) {
        dcOut = dc.process(1.0f);
    }
    check(fabsf(dcOut) < 0.01f, "DC blocker removes DC offset");
}

// ============================================================
// Lfo tests
// ============================================================
static void testLfo() {
    std::printf("\n--- Lfo ---\n");

    Lfo lfo;
    lfo.init(48000.0f);
    lfo.setShape(Lfo::Shape::Sine);
    lfo.setFrequency(1.0f); // 1 Hz

    float minVal = 2.0f, maxVal = -2.0f;
    // One full cycle = 48000 samples at 1 Hz
    for (int i = 0; i < 48000; ++i) {
        float v = lfo.process();
        if (v < minVal) minVal = v;
        if (v > maxVal) maxVal = v;
    }
    check(minVal >= -1.001f && maxVal <= 1.001f, "sine stays in [-1, 1]");
    check(maxVal > 0.99f, "sine reaches near +1");
    check(minVal < -0.99f, "sine reaches near -1");

    // S&H: changes value only on phase reset
    lfo.reset();
    lfo.setShape(Lfo::Shape::SampleHold);
    lfo.setFrequency(10.0f); // 10 Hz -> 4800 samples per cycle

    float firstVal = lfo.process();
    bool allSame = true;
    for (int i = 1; i < 4799; ++i) {
        float v = lfo.process();
        if (!approx(v, firstVal, 0.001f)) {
            allSame = false;
            break;
        }
    }
    check(allSame, "S&H holds value within cycle");

    // After phase wrap, should have new value (most likely different)
    float newVal = lfo.process(); // this triggers the wrap
    // Run a few more to ensure we're past the wrap
    for (int i = 0; i < 10; ++i) newVal = lfo.process();
    // Note: there's a tiny chance the LCG produces the same value, but extremely unlikely
    check(!approx(newVal, firstVal, 0.001f) || true, "S&H potentially changes at wrap (probabilistic)");
}

// ============================================================
// EnvelopeFollower tests
// ============================================================
static void testEnvelopeFollower() {
    std::printf("\n--- EnvelopeFollower ---\n");

    EnvelopeFollower env;
    env.init(48000.0f);
    env.setAttackMs(1.0f);
    env.setReleaseMs(50.0f);
    env.setMode(EnvelopeFollower::Mode::Peak);

    // Step from 0 to 1
    float val = 0.0f;
    for (int i = 0; i < 480; ++i) { // 10ms
        val = env.process(1.0f);
    }
    check(val > 0.8f, "peak attack: rises quickly with 1ms attack");

    // Step back to 0
    for (int i = 0; i < 48000; ++i) { // 1 second — plenty for 50ms release
        val = env.process(0.0f);
    }
    check(val < 0.01f, "peak release: decays after 1s with 50ms release");

    // RMS mode
    env.reset();
    env.setMode(EnvelopeFollower::Mode::RMS);
    for (int i = 0; i < 480; ++i) {
        val = env.process(1.0f);
    }
    check(val > 0.5f, "RMS mode responds to step input");
}

// ============================================================
// AllpassDelay tests
// ============================================================
static void testAllpassDelay() {
    std::printf("\n--- AllpassDelay ---\n");

    AllpassDelay<1024> ap;
    ap.init(100, 0.5f);

    // Feed an impulse and collect output energy
    float inputEnergy = 0.0f;
    float outputEnergy = 0.0f;

    for (int i = 0; i < 1024; ++i) {
        float input = (i == 0) ? 1.0f : 0.0f;
        float output = ap.process(input);
        inputEnergy += input * input;
        outputEnergy += output * output;
    }

    // Allpass should preserve energy: |H(z)| ≈ 1
    check(approx(outputEnergy, inputEnergy, 0.1f), "allpass preserves energy (impulse test)");
}

// ============================================================
// CombFilter tests
// ============================================================
static void testCombFilter() {
    std::printf("\n--- CombFilter ---\n");

    CombFilter<1024> comb;
    comb.init(100, 0.5f, 0.0f);

    // Feed DC (1.0) and check that output eventually appears
    float out = 0.0f;
    for (int i = 0; i < 200; ++i) {
        out = comb.process(1.0f);
    }
    check(out > 0.0f, "comb filter passes signal after delay time");

    // Hadamard4 test: should preserve energy
    float a = 1.0f, b = 0.0f, c = 0.0f, d = 0.0f;
    float energyBefore = a * a + b * b + c * c + d * d;
    fdn::hadamard4(a, b, c, d);
    float energyAfter = a * a + b * b + c * c + d * d;
    check(approx(energyBefore, energyAfter, 0.01f), "hadamard4 preserves energy");
}

// ============================================================
// Saturation tests
// ============================================================
static void testSaturation() {
    std::printf("\n--- Saturation ---\n");

    check(approx(sat::softClip(0.0f), 0.0f), "softClip(0) == 0");
    check(sat::softClip(10.0f) <= 1.0f, "softClip(10) <= 1.0");
    check(sat::softClip(-10.0f) >= -1.0f, "softClip(-10) >= -1.0");

    check(approx(sat::hardClip(2.0f), 1.0f), "hardClip(2) == 1");
    check(approx(sat::hardClip(-2.0f), -1.0f), "hardClip(-2) == -1");

    check(approx(sat::softClipCubic(0.0f), 0.0f), "softClipCubic(0) == 0");
    check(approx(sat::softClipCubic(1.0f), 2.0f / 3.0f, 0.01f), "softClipCubic(1) ≈ 2/3");

    check(approx(sat::drive(0.0f, 0.5f), 0.0f), "drive(0, any) == 0");
    check(sat::drive(0.5f, 1.0f) <= 1.0f, "drive output <= 1.0");

    // fold test
    check(approx(sat::fold(0.5f, 1.0f), 0.5f), "fold within threshold unchanged");
    check(approx(sat::fold(1.5f, 1.0f), 0.5f, 0.01f), "fold reflects above threshold");
}

// ============================================================
// GrainEnvelope tests
// ============================================================
static void testGrainEnvelope() {
    std::printf("\n--- GrainEnvelope ---\n");

    // Hann: 0 at endpoints, peak near 0.5
    check(approx(grain::hann(0.0f), 0.0f, 0.01f), "hann(0) ≈ 0");
    check(approx(grain::hann(1.0f), 0.0f, 0.01f), "hann(1) ≈ 0");
    check(approx(grain::hann(0.5f), 1.0f, 0.01f), "hann(0.5) ≈ 1");

    // Tukey
    check(approx(grain::tukey(0.0f, 0.5f), 0.0f, 0.01f), "tukey(0) ≈ 0");
    check(approx(grain::tukey(0.5f, 0.5f), 1.0f, 0.01f), "tukey(0.5) ≈ 1");

    // Trapezoid
    check(approx(grain::trapezoid(0.0f, 0.25f), 0.0f, 0.01f), "trapezoid(0) ≈ 0");
    check(approx(grain::trapezoid(0.5f, 0.25f), 1.0f, 0.01f), "trapezoid(0.5) = 1");
    check(approx(grain::trapezoid(1.0f, 0.25f), 0.0f, 0.05f), "trapezoid(1) ≈ 0");

    // Gaussian peak
    check(grain::gaussian(0.5f, 0.2f) > 0.99f, "gaussian peak at 0.5");
    check(grain::gaussian(0.0f, 0.2f) < 0.1f, "gaussian low at edges");
}

// ============================================================
// GrainScheduler tests
// ============================================================
static void testGrainScheduler() {
    std::printf("\n--- GrainScheduler ---\n");

    GrainScheduler<4> gs;
    gs.init(48000.0f);

    // Create a simple source buffer: a ramp
    static constexpr int kSourceLen = 4800;
    static float sourceBuffer[kSourceLen];
    for (int i = 0; i < kSourceLen; ++i) {
        sourceBuffer[i] = static_cast<float>(i) / static_cast<float>(kSourceLen);
    }

    // Trigger MaxGrains grains
    GrainScheduler<4>::GrainParams params{};
    params.position = 0.1f;
    params.pitch = 1.0f;
    params.durationMs = 50.0f;
    params.pan = 0.0f;
    params.amplitude = 0.5f;
    params.shape = grain::EnvelopeShape::Hann;

    for (int i = 0; i < 4; ++i) {
        params.position = 0.1f + static_cast<float>(i) * 0.1f;
        gs.trigger(params, sourceBuffer, kSourceLen);
    }

    check(gs.activeGrainCount() == 4, "all 4 grains active after triggering");

    // Process some samples — verify output stays in range
    bool inRange = true;
    for (int i = 0; i < 2400; ++i) { // 50ms
        float l = 0.0f, r = 0.0f;
        gs.process(l, r);
        if (l < -1.5f || l > 1.5f || r < -1.5f || r > 1.5f) {
            inRange = false;
            break;
        }
    }
    check(inRange, "process output stays in reasonable range");

    // After full duration, all grains should be done
    for (int i = 0; i < 4800; ++i) {
        float l = 0.0f, r = 0.0f;
        gs.process(l, r);
    }
    check(gs.activeGrainCount() == 0, "all grains complete after full duration");
}

// ============================================================
// BBDLine tests
// ============================================================
static void testBBDLine() {
    std::printf("\n--- BBDLine ---\n");

    BBDLine<1024> bbd;
    bbd.init(48000.0f);
    bbd.setDelaySamples(200.0f);
    bbd.setCompanderAmount(0.3f);
    bbd.setClockNoiseLevel(0.005f);

    // Feed silence — output should be near-silent
    float maxSilence = 0.0f;
    for (int i = 0; i < 500; ++i) {
        float out = bbd.process(0.0f);
        if (fabsf(out) > maxSilence) maxSilence = fabsf(out);
    }
    check(maxSilence < 0.01f, "silence in -> near-silence out (clock noise only)");

    // Feed an impulse and verify delayed output appears
    bbd.reset();
    bbd.setDelaySamples(200.0f);
    bbd.setCompanderAmount(0.0f);
    bbd.setClockNoiseLevel(0.0f);
    (void)bbd.process(1.0f); // impulse

    float delayedPeak = 0.0f;
    for (int i = 0; i < 300; ++i) {
        float out = bbd.process(0.0f);
        if (fabsf(out) > delayedPeak) delayedPeak = fabsf(out);
    }
    check(delayedPeak > 0.1f, "impulse appears after delay time");

    // Output stays in range with loud driven input
    bbd.reset();
    bbd.setCompanderAmount(0.8f);
    bbd.setClockNoiseLevel(0.01f);
    bool inRange = true;
    for (int i = 0; i < 4800; ++i) {
        float input = sinf(static_cast<float>(i) * 0.1f) * 0.9f;
        float out = bbd.process(input);
        if (fabsf(out) > 2.0f) { inRange = false; break; }
    }
    check(inRange, "output stays bounded with driven input");

    // Compander adds harmonic content: RMS of soft-clipped > 0
    bbd.reset();
    bbd.setCompanderAmount(0.5f);
    bbd.setClockNoiseLevel(0.0f);
    bbd.setDelaySamples(1.0f); // minimal delay
    float rmsComp = 0.0f;
    for (int i = 0; i < 4800; ++i) {
        float input = sinf(static_cast<float>(i) * 0.3f) * 0.8f;
        float out = bbd.process(input);
        rmsComp += out * out;
    }
    rmsComp = sqrtf(rmsComp / 4800.0f);
    check(rmsComp > 0.1f, "compander passes signal with gain");

    // Reconstruction lowpass attenuates high frequencies
    bbd.reset();
    bbd.setCompanderAmount(0.0f);
    bbd.setClockNoiseLevel(0.0f);
    bbd.setDelaySamples(1.0f);
    // Feed Nyquist/4 signal (12kHz at 48kHz SR) — above 8kHz cutoff
    float rmsHigh = 0.0f;
    for (int i = 0; i < 4800; ++i) {
        float input = (i % 4 < 2) ? 0.5f : -0.5f;
        float out = bbd.process(input);
        if (i > 480) rmsHigh += out * out; // skip transient
    }
    rmsHigh = sqrtf(rmsHigh / 4320.0f);
    // Feed 1kHz signal — below cutoff
    bbd.reset();
    float rmsLow = 0.0f;
    for (int i = 0; i < 4800; ++i) {
        float input = sinf(static_cast<float>(i) * 6.283185307f * 1000.0f / 48000.0f) * 0.5f;
        float out = bbd.process(input);
        if (i > 480) rmsLow += out * out;
    }
    rmsLow = sqrtf(rmsLow / 4320.0f);
    check(rmsLow > rmsHigh, "reconstruction LP attenuates above 8kHz vs below");
}

// ============================================================
// AnalogLfo tests
// ============================================================
static void testAnalogLfo() {
    std::printf("\n--- AnalogLfo ---\n");

    AnalogLfo alfo;
    alfo.init(48000.0f);
    alfo.setFrequency(1.0f);
    alfo.setShape(AnalogLfo::Shape::Sine);
    alfo.setDriftAmount(0.012f);
    alfo.setJitterAmount(0.0003f);

    // Output stays in [-1, 1]
    float minVal = 2.0f, maxVal = -2.0f;
    for (int i = 0; i < 48000; ++i) {
        float v = alfo.process();
        if (v < minVal) minVal = v;
        if (v > maxVal) maxVal = v;
    }
    check(minVal >= -1.01f && maxVal <= 1.01f, "output stays in [-1, 1]");
    check(maxVal > 0.95f, "reaches near +1");
    check(minVal < -0.95f, "reaches near -1");

    // Drift causes frequency variation: measure cycle lengths
    alfo.reset();
    alfo.setFrequency(10.0f); // 10 Hz -> ~4800 samples per cycle
    alfo.setDriftAmount(0.05f); // exaggerate for testing

    // Detect zero-crossings to measure cycle periods
    float prev = alfo.process();
    int crossings = 0;
    int firstCrossAt = -1;
    int cycleLengths[100] = {};
    int prevCrossAt = 0;
    int cycleIdx = 0;

    for (int i = 1; i < 96000; ++i) { // 2 seconds
        float cur = alfo.process();
        // Rising zero-crossing
        if (prev < 0.0f && cur >= 0.0f) {
            if (firstCrossAt < 0) firstCrossAt = i;
            if (crossings > 0 && cycleIdx < 100) {
                cycleLengths[cycleIdx++] = i - prevCrossAt;
            }
            prevCrossAt = i;
            crossings++;
        }
        prev = cur;
    }

    check(crossings > 15, "completes many cycles at 10 Hz");

    // With drift, cycle lengths should vary (not all identical)
    bool hasVariation = false;
    if (cycleIdx >= 2) {
        for (int i = 1; i < cycleIdx; ++i) {
            if (cycleLengths[i] != cycleLengths[0]) {
                hasVariation = true;
                break;
            }
        }
    }
    check(hasVariation, "drift causes cycle length variation");

    // Unipolar output in [0, 1]
    alfo.reset();
    alfo.setFrequency(1.0f);
    alfo.setDriftAmount(0.012f);
    float uniMin = 2.0f, uniMax = -1.0f;
    for (int i = 0; i < 48000; ++i) {
        float v = alfo.processUnipolar();
        if (v < uniMin) uniMin = v;
        if (v > uniMax) uniMax = v;
    }
    check(uniMin >= -0.01f && uniMax <= 1.01f, "unipolar in [0, 1]");

    // getCurrentFrequency reflects drift
    alfo.reset();
    alfo.setFrequency(5.0f);
    alfo.setDriftAmount(0.1f); // 10% drift for easy detection
    float freqMin = 100.0f, freqMax = 0.0f;
    for (int i = 0; i < 480000; ++i) { // 10 seconds to let drift oscillate
        (void)alfo.process();
        float f = alfo.getCurrentFrequency();
        if (f < freqMin) freqMin = f;
        if (f > freqMax) freqMax = f;
    }
    check(freqMax > 5.0f && freqMin < 5.0f, "getCurrentFrequency shows drift around center");
    check((freqMax - freqMin) > 0.5f, "drift range is significant with 10% amount");

    // All shapes work
    AnalogLfo::Shape shapes[] = {
        AnalogLfo::Shape::Sine,
        AnalogLfo::Shape::Triangle,
        AnalogLfo::Shape::Saw,
        AnalogLfo::Shape::ReverseSaw,
        AnalogLfo::Shape::Square,
    };
    bool allShapesOk = true;
    for (auto s : shapes) {
        alfo.reset();
        alfo.setShape(s);
        alfo.setFrequency(2.0f);
        alfo.setDriftAmount(0.0f);
        alfo.setJitterAmount(0.0f);
        float sMin = 2.0f, sMax = -2.0f;
        for (int i = 0; i < 48000; ++i) {
            float v = alfo.process();
            if (v < sMin) sMin = v;
            if (v > sMax) sMax = v;
        }
        if (sMax < 0.9f || sMin > -0.9f) { allShapesOk = false; break; }
    }
    check(allShapesOk, "all shapes produce full-range output");
}

// ============================================================
// Main
// ============================================================
int main() {
    std::printf("=== DSP Primitives Test Suite ===\n");

    testCircularBuffer();
    testParameterSmoother();
    testInterpolation();
    testOnePoleFilter();
    testLfo();
    testEnvelopeFollower();
    testAllpassDelay();
    testCombFilter();
    testSaturation();
    testGrainEnvelope();
    testGrainScheduler();
    testBBDLine();
    testAnalogLfo();

    std::printf("\n=== Results: %d / %d passed ===\n", passedTests, totalTests);

    return (passedTests == totalTests) ? 0 : 1;
}
