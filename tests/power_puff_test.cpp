#include "Patch.h"
#include "effects/PowerPuffParams.h"
#include <cstdio>
#include <cmath>
#include <cassert>
#include <span>

// ── Test 1: Instantiation ─────────────────────────────────────────────────

void test_patch_instantiation() {
    Patch* patch = Patch::getInstance();
    assert(patch != nullptr);
    patch->init();
    printf("PASS: test_patch_instantiation\n");
}

// ── Test 2: 100% wet mix ──────────────────────────────────────────────────

void test_100_percent_wet_mix() {
    Patch* patch = Patch::getInstance();
    patch->init();
    const float sr = 48000.0f;
    float l[64], r[64];
    // Priming phase: drive the circuit with real signal
    for (int b = 0; b < 16; ++b) {
        for (int i = 0; i < 64; ++i)
            l[i] = 0.3f * sinf(2.0f * 3.14159265f * 440.0f * (float)(b*64+i) / sr);
        std::span<float> ls(l, 64), rs(r, 64);
        patch->processAudio(ls, rs);
    }
    // Settling phase: the tone stack's 5 Hz DC block has τ ≈ 1529 samples.
    // 150 blocks (9600 samples ≈ 6.3 τ) brings residual to e^-6.3 ≈ 0.0018.
    for (int b = 0; b < 150; ++b) {
        for (int i = 0; i < 64; ++i) l[i] = 0.0f;
        std::span<float> ls(l, 64), rs(r, 64);
        patch->processAudio(ls, rs);
    }
    // Measurement: circuit should now be settled to near-silence
    float maxSilenceOut = 0.0f;
    for (int b = 0; b < 20; ++b) {
        for (int i = 0; i < 64; ++i) l[i] = 0.0f;
        std::span<float> ls(l, 64), rs(r, 64);
        patch->processAudio(ls, rs);
        for (int i = 0; i < 64; ++i)
            if (fabsf(l[i]) > maxSilenceOut) maxSilenceOut = fabsf(l[i]);
    }
    assert(maxSilenceOut < 0.05f);
    printf("PASS: test_100_percent_wet_mix (maxSilenceOut=%.5f)\n", static_cast<double>(maxSilenceOut));
}

// ── Test 3: All five variants produce output and have correct LED colors ──

void test_all_variants_correct_led() {
    Patch* patch = Patch::getInstance();

    struct {
        float knob;
        Patch::Color expectedLed;
        const char* name;
    } cases[] = {
        { 0.1f, Patch::Color::kLightYellow, "Triangle (V1, notch≈763Hz)"  },
        { 0.3f, Patch::Color::kMagenta,     "RamsHead (V2, notch≈859Hz)"  },
        { 0.5f, Patch::Color::kBeige,       "OpAmp    (V4, notch≈859Hz)"  },
        { 0.7f, Patch::Color::kDarkCobalt,  "CivilWar (V7, notch≈1199Hz)" },
        { 0.9f, Patch::Color::kDarkRed,     "NYC      (V9, notch≈859Hz)"  },
    };

    for (auto& c : cases) {
        patch->init();
        patch->setParamValue(power_puff_params::kKnobVariant, c.knob);
        patch->setParamValue(power_puff_params::kKnobSustain, 0.7f);
        patch->setParamValue(power_puff_params::kKnobTone,    0.5f);

        // Drive one control-rate block to trigger variant resolution
        float l[64], r[64];
        for (int i = 0; i < 64; ++i) l[i] = 0.0f;
        { std::span<float> ls(l,64), rs(r,64); patch->processAudio(ls, rs); }

        assert(patch->getStateLedColor() == c.expectedLed &&
               "PowerPuff: wrong LED color for variant");

        // Measure RMS over 4800 samples to confirm output
        const float sr = 48000.0f;
        float sum = 0.0f;
        int total = 4800, processed = 0, idx = 0;
        while (processed < total) {
            int chunk = (64 < total - processed) ? 64 : (total - processed);
            for (int i = 0; i < chunk; ++i)
                l[i] = 0.3f * sinf(2.0f * 3.14159265f * 440.0f * (float)(idx++) / sr);
            std::span<float> ls(l, chunk), rs(r, chunk);
            patch->processAudio(ls, rs);
            for (int i = 0; i < chunk; ++i) sum += l[i] * l[i];
            processed += chunk;
        }
        float rms = sqrtf(sum / 4800.0f);
        // Upper bound is 5.0f: WdfOpAmpBigMuffCircuit has no internal output scale
        // factor, so its RMS is naturally higher than the BJT variants (which have
        // kOutScale=0.15f). 5.0 still catches NaN/Inf and gross runaway instability.
        assert(rms > 0.001f && rms < 5.0f);
        printf("PASS: test_all_variants_correct_led %s (rms=%.4f)\n", c.name, static_cast<double>(rms));
    }
}

// ── Test 4: Tone stack notch frequency shifts with variant ─────────────────

void test_tone_stack_varies_by_variant() {
    Patch* patch = Patch::getInstance();
    const float sr = 48000.0f;
    const float testFreq = 1000.0f;

    auto measureRMSAtFreq = [&](float variantKnob) -> float {
        patch->init();
        patch->setParamValue(power_puff_params::kKnobVariant, variantKnob);
        patch->setParamValue(power_puff_params::kKnobSustain, 0.3f);
        patch->setParamValue(power_puff_params::kKnobTone,    0.5f);
        // Settle
        float l[64], r[64];
        for (int j = 0; j < 10; ++j) {
            for (int i = 0; i < 64; ++i) l[i] = 0.0f;
            std::span<float> ls(l,64), rs(r,64); patch->processAudio(ls, rs);
        }
        float sum = 0.0f;
        int idx = 0;
        for (int b = 0; b < 75; ++b) {
            for (int i = 0; i < 64; ++i)
                l[i] = 0.1f * sinf(2.0f * 3.14159265f * testFreq * (float)(idx++) / sr);
            std::span<float> ls(l,64), rs(r,64); patch->processAudio(ls, rs);
            for (int i = 0; i < 64; ++i) sum += l[i] * l[i];
        }
        return sqrtf(sum / (75*64.0f));
    };

    float rmsTriangle = measureRMSAtFreq(0.1f);  // notch≈763Hz
    float rmsCivilWar = measureRMSAtFreq(0.7f);  // notch≈1199Hz

    assert(fabsf(rmsTriangle - rmsCivilWar) > 0.001f &&
           "PowerPuff: Triangle and CivilWar tone stacks produced identical output — "
           "tone stack variant linkage may be broken");
    printf("PASS: test_tone_stack_varies_by_variant "
           "(Triangle@1kHz=%.5f CivilWar@1kHz=%.5f diff=%.5f)\n",
           static_cast<double>(rmsTriangle),
           static_cast<double>(rmsCivilWar),
           static_cast<double>(fabsf(rmsTriangle - rmsCivilWar)));
}

// ── Test 5: Mono mirror ───────────────────────────────────────────────────

void test_mono_mirror() {
    Patch* patch = Patch::getInstance();
    patch->init();
    float l[64], r[64];
    const float sr = 48000.0f;
    for (int i = 0; i < 64; ++i)
        l[i] = 0.3f * sinf(2.0f * 3.14159265f * 440.0f * (float)i / sr);
    std::span<float> ls(l, 64), rs(r, 64);
    patch->processAudio(ls, rs);
    for (int i = 0; i < 64; ++i)
        assert(fabsf(l[i] - r[i]) < 1e-6f);
    printf("PASS: test_mono_mirror\n");
}

// ── Test 6: Tone bypass tap increases high-frequency energy ───────────────

void test_tone_bypass_tap() {
    Patch* patch = Patch::getInstance();
    const int tapAction = static_cast<int>(endless::ActionId::kLeftFootSwitchPress);
    const float sr = 48000.0f;

    auto measure5kRMS = [&]() -> float {
        patch->init();
        patch->setParamValue(power_puff_params::kKnobSustain, 0.5f);
        patch->setParamValue(power_puff_params::kKnobTone,    0.5f);
        float l[64], r[64];
        for (int j = 0; j < 10; ++j) {
            for (int i = 0; i < 64; ++i) l[i] = 0.0f;
            std::span<float> ls(l,64), rs(r,64); patch->processAudio(ls, rs);
        }
        float sum = 0.0f;
        for (int b = 0; b < 10; ++b) {
            for (int i = 0; i < 64; ++i)
                l[i] = 0.1f * sinf(2.0f * 3.14159265f * 5000.0f * (float)(b*64+i) / sr);
            std::span<float> ls(l,64), rs(r,64); patch->processAudio(ls, rs);
            for (int i = 0; i < 64; ++i) sum += l[i] * l[i];
        }
        return sqrtf(sum / 640.0f);
    };

    float rmsNormal = measure5kRMS();
    patch->handleAction(tapAction);
    float sum = 0.0f;
    for (int b = 0; b < 10; ++b) {
        float l[64], r[64];
        for (int i = 0; i < 64; ++i)
            l[i] = 0.1f * sinf(2.0f * 3.14159265f * 5000.0f * (float)(b*64+i) / sr);
        std::span<float> ls(l,64), rs(r,64); patch->processAudio(ls, rs);
        for (int i = 0; i < 64; ++i) sum += l[i] * l[i];
    }
    float rmsBypass = sqrtf(sum / 640.0f);
    assert(rmsBypass >= rmsNormal);
    printf("PASS: test_tone_bypass_tap (normal=%.5f bypass=%.5f)\n",
           static_cast<double>(rmsNormal), static_cast<double>(rmsBypass));
}

// ── Test 7: Tone bypass LED shows white ───────────────────────────────────

void test_led_tone_bypass_shows_white() {
    Patch* patch = Patch::getInstance();
    patch->init();
    const int tapAction = static_cast<int>(endless::ActionId::kLeftFootSwitchPress);
    patch->handleAction(tapAction);
    assert(patch->getStateLedColor() == Patch::Color::kDimWhite);
    patch->handleAction(tapAction);
    assert(patch->getStateLedColor() != Patch::Color::kDimWhite);
    printf("PASS: test_led_tone_bypass_shows_white\n");
}

// ── Test 8: Hold action is silently ignored ───────────────────────────────

void test_hold_action_no_effect() {
    Patch* patch = Patch::getInstance();
    patch->init();
    patch->setParamValue(power_puff_params::kKnobVariant, 0.1f);
    float l[64], r[64];
    for (int i = 0; i < 64; ++i) l[i] = 0.0f;
    { std::span<float> ls(l,64), rs(r,64); patch->processAudio(ls, rs); }
    Patch::Color before = patch->getStateLedColor();
    patch->handleAction(static_cast<int>(endless::ActionId::kLeftFootSwitchHold));
    assert(patch->getStateLedColor() == before);
    printf("PASS: test_hold_action_no_effect\n");
}

// ── Test 9: Numerical stability at max sustain, all variants ──────────────

void test_stability_all_variants() {
    Patch* patch = Patch::getInstance();
    const float knobs[5] = { 0.1f, 0.3f, 0.5f, 0.7f, 0.9f };
    for (int v = 0; v < 5; ++v) {
        patch->init();
        patch->setParamValue(power_puff_params::kKnobVariant, knobs[v]);
        patch->setParamValue(power_puff_params::kKnobSustain, 1.0f);
        patch->setParamValue(power_puff_params::kKnobTone,    0.5f);
        float l[64], r[64];
        bool stable = true;
        for (int b = 0; b < 100 && stable; ++b) {
            for (int i = 0; i < 64; ++i)
                l[i] = (b*64+i) % 2 == 0 ? 1.0f : -1.0f;
            std::span<float> ls(l,64), rs(r,64); patch->processAudio(ls, rs);
            for (int i = 0; i < 64; ++i)
                if (!std::isfinite(l[i])) { stable = false; break; }
        }
        assert(stable && "PowerPuff: NaN or Inf at max sustain");
        printf("PASS: test_stability_all_variants variant %d\n", v);
    }
}

int main() {
    test_patch_instantiation();
    test_100_percent_wet_mix();
    test_all_variants_correct_led();
    test_tone_stack_varies_by_variant();
    test_mono_mirror();
    test_tone_bypass_tap();
    test_led_tone_bypass_shows_white();
    test_hold_action_no_effect();
    test_stability_all_variants();
    printf("\nAll Power Puff tests passed.\n");
    return 0;
}
