#pragma once
// EH-7850 NE570 Compander — Behavioral DSP model
// Component source: EH-7850 SPICE/component reference (project upload)
//   Section 2 (NE570 ch.1): C7=10µF → τ_comp=100ms, Av=√(Vref/Venv)
//   Section 6 (NE570 ch.2): C18=4.7µF → τ_exp=47ms, Av=Venv/Vref
// Behavioral model: Philips AN174 §3.1 (compressor) and §3.2 (expander)
// Giannoulis, Massberg & Reiss, JAES 2012 — compressor design reference
// Thomas Henry, "Making Music with the NE570 Compander", Lulu 2011
//
// CYCLE BUDGET (528 MHz / 48 kHz = 11,000 cycles/sample available):
//   DmmCompressor: ~150–300 cycles/sample
//   DmmExpander:   ~150–300 cycles/sample
//   Total:         ~300–600 cycles/sample (2.7–5.5% of budget)

#include "EnvelopeFollower.h"
#include "Saturation.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

#ifdef DMM_USE_FAST_MATH
// 2-iteration Newton-Raphson sqrt approximation
// Error < 0.15% for argument range [1e-4, 10.0] at 48 kHz signal levels
// Saves ~20–40 cycles vs std::sqrt on Cortex-M7
namespace dmm_detail {
[[nodiscard]] inline float fastSqrt(float x) noexcept {
    union { float f; uint32_t i; } u = { x };
    u.i = 0x1fbb4000 + (u.i >> 1);
    u.f = 0.5f * (u.f + x / u.f);   // Newton iteration 1
    u.f = 0.5f * (u.f + x / u.f);   // Newton iteration 2
    return u.f;
}
} // namespace dmm_detail
#endif

struct DmmCompressor {
    EnvelopeFollower follower;

    static constexpr float kVrefInt   = 0.100f;
    static constexpr float kGainMin   = 0.1f;
    static constexpr float kGainMax   = 10.0f;
    static constexpr float kAttackMs  = 100.0f;   // C7=10µF × 10kΩ
    static constexpr float kReleaseMs = 100.0f;

    void init(float sampleRate) noexcept {
        follower.init(sampleRate);
        follower.setMode(EnvelopeFollower::Mode::Peak);
        follower.setAttackMs(kAttackMs);
        follower.setReleaseMs(kReleaseMs);
    }

    [[nodiscard]] float process(float x) noexcept {
        float env = follower.process(x);
        float denom = env > 1e-6f ? env : 1e-6f;
#ifdef DMM_USE_FAST_MATH
        float gain = dmm_detail::fastSqrt(kVrefInt / denom);
#else
        float gain = std::sqrt(kVrefInt / denom);
#endif
        gain = std::clamp(gain, kGainMin, kGainMax);
        return x * gain;
    }

    void reset() noexcept { follower.reset(); }
};

struct DmmExpander {
    EnvelopeFollower follower;

    static constexpr float kVrefInt   = 0.100f;   // matched to compressor
    static constexpr float kGainMin   = 0.1f;
    static constexpr float kGainMax   = 10.0f;
    static constexpr float kAttackMs  = 47.0f;    // C18=4.7µF × 10kΩ
    static constexpr float kReleaseMs = 47.0f;

    void init(float sampleRate) noexcept {
        follower.init(sampleRate);
        follower.setMode(EnvelopeFollower::Mode::Peak);
        follower.setAttackMs(kAttackMs);
        follower.setReleaseMs(kReleaseMs);
    }

    [[nodiscard]] float process(float x) noexcept {
        float env = follower.process(x);
        float num = env > 1e-6f ? env : 1e-6f;
        float gain = num / kVrefInt;
        gain = std::clamp(gain, kGainMin, kGainMax);
        return x * gain;
    }

    void reset() noexcept { follower.reset(); }
};

#ifdef DMM_COMPANDER_SELF_TEST

#include <cstdio>
#include <cmath>

static inline void dmmCompanderSelfTest() {
    constexpr int   kSR      = 48000;
    constexpr int   kMeas    = 1000;
    // 5 × 100ms = 500ms prime for compressor (100ms time constant)
    constexpr int   kPrimeC  = 24000;
    // 5 × 47ms ≈ 235ms prime for expander (47ms time constant), via loop over kMeas
    constexpr int   kPrimeE  = 12000;
    constexpr float kAmp     = 0.316f;   // -10 dBFS
    constexpr float kFreq    = 1000.0f;
    constexpr float kTwoPi   = 6.283185307f;

    auto sig = [&](int i) noexcept -> float {
        return kAmp * std::sin(kTwoPi * kFreq * static_cast<float>(i) / static_cast<float>(kSR));
    };

    // --- Phase 1: prime compressor for 500ms then record kMeas samples ---
    DmmCompressor comp;
    comp.init(static_cast<float>(kSR));
    for (int i = 0; i < kPrimeC; ++i) { (void)comp.process(sig(i)); }

    float inputBuf[kMeas];
    float compBuf[kMeas];
    float inputRmsAcc = 0.0f;
    float compRmsAcc  = 0.0f;
    for (int i = 0; i < kMeas; ++i) {
        inputBuf[i]   = sig(kPrimeC + i);
        compBuf[i]    = comp.process(inputBuf[i]);
        inputRmsAcc  += inputBuf[i] * inputBuf[i];
        compRmsAcc   += compBuf[i]  * compBuf[i];
    }

    // --- Phase 2: prime expander on compBuf looped for 235ms, then measure ---
    DmmExpander expd;
    expd.init(static_cast<float>(kSR));
    for (int i = 0; i < kPrimeE; ++i) { (void)expd.process(compBuf[i % kMeas]); }

    float expandRmsAcc = 0.0f;
    for (int i = 0; i < kMeas; ++i) {
        float e = expd.process(compBuf[i]);
        expandRmsAcc += e * e;
    }

    float inputRms  = std::sqrt(inputRmsAcc  / static_cast<float>(kMeas));
    float compRms   = std::sqrt(compRmsAcc   / static_cast<float>(kMeas));
    float expandRms = std::sqrt(expandRmsAcc / static_cast<float>(kMeas));

    float distComp   = std::fabs(compRms   - inputRms);
    float distExpand = std::fabs(expandRms - inputRms);

    bool compressTest = compRms < inputRms;
    bool expandTest   = distExpand < distComp;

    std::printf("DmmCompander self-test:\n");
    std::printf("  input RMS=%.4f  comp RMS=%.4f  expand RMS=%.4f\n",
                inputRms, compRms, expandRms);
    std::printf("  Compressor attenuates:     %s  (%.4f < %.4f)\n",
                compressTest ? "PASS" : "FAIL", compRms, inputRms);
    std::printf("  Expander restores closer:  %s  (dist_exp=%.4f < dist_comp=%.4f)\n",
                expandTest ? "PASS" : "FAIL", distExpand, distComp);
    std::printf("  Overall: %s\n", (compressTest && expandTest) ? "PASS" : "FAIL");
}

int main() {
    dmmCompanderSelfTest();
    return 0;
}

#endif // DMM_COMPANDER_SELF_TEST
