#include <cstdio>
#include <cmath>
#include <cassert>
#include "WdfBigMuffToneStack.h"

static float measureRMS(WdfBigMuffToneStack& s, float freqHz, float sr, int N) {
    for (int i = 0; i < 500; ++i)
        (void)s.process(0.5f * sinf(2.0f * 3.14159265f * freqHz * (float)i / sr));
    float sum = 0.0f;
    for (int i = 0; i < N; ++i) {
        float y = s.process(0.5f * sinf(2.0f * 3.14159265f * freqHz * (float)(i+500) / sr));
        sum += y * y;
    }
    return sqrtf(sum / (float)N);
}

void test_dc_blocking() {
    WdfBigMuffToneStack s;
    s.init(48000.0f, WdfBigMuffToneStack::Variant::Triangle);
    s.setTone(0.5f);
    float lastOut = 0.0f;
    for (int i = 0; i < 9600; ++i) lastOut = s.process(0.5f);
    assert(fabsf(lastOut) < 0.005f && "DC not blocked");
    printf("PASS: test_dc_blocking\n");
}

void test_triangle_midnotch() {
    WdfBigMuffToneStack s;
    s.init(48000.0f, WdfBigMuffToneStack::Variant::Triangle);
    s.setTone(0.5f);
    float sr = 48000.0f;
    float rms_notch = measureRMS(s, 763.0f, sr, 4800);
    s.reset(); s.setTone(0.5f);
    float rms_lo = measureRMS(s, 150.0f, sr, 4800);
    s.reset(); s.setTone(0.5f);
    float rms_hi = measureRMS(s, 2500.0f, sr, 4800);
    assert((rms_lo > 1.41f * rms_notch || rms_hi > 1.41f * rms_notch) && "No notch");
    printf("PASS: test_triangle_midnotch\n");
}

void test_civilwar_notch_higher() {
    float sr = 48000.0f;
    WdfBigMuffToneStack tri, cw;
    tri.init(sr, WdfBigMuffToneStack::Variant::Triangle); tri.setTone(0.5f);
    cw.init(sr, WdfBigMuffToneStack::Variant::CivilWar); cw.setTone(0.5f);
    float rms_tri = measureRMS(tri, 1000.0f, sr, 4800);
    float rms_cw = measureRMS(cw, 1000.0f, sr, 4800);
    assert(rms_cw < rms_tri && "CivilWar should attenuate 1kHz more");
    printf("PASS: test_civilwar_notch_higher\n");
}

void test_matched_variants() {
    float sr = 48000.0f;
    WdfBigMuffToneStack rh, oa, nyc;
    rh.init(sr, WdfBigMuffToneStack::Variant::RamsHead); rh.setTone(0.5f);
    oa.init(sr, WdfBigMuffToneStack::Variant::OpAmp); oa.setTone(0.5f);
    nyc.init(sr, WdfBigMuffToneStack::Variant::NYC); nyc.setTone(0.5f);
    for (int i = 0; i < 500; ++i) { (void)rh.process(0.0f); (void)oa.process(0.0f); (void)nyc.process(0.0f); }
    bool ok = true;
    for (int i = 0; i < 2400; ++i) {
        float x = 0.5f * sinf(2.0f * 3.14159265f * 500.0f * (float)i / sr);
        float y_rh = rh.process(x);
        float y_oa = oa.process(x);
        float y_nyc = nyc.process(x);
        if (fabsf(y_rh - y_oa) > 1e-5f) ok = false;
        if (fabsf(y_rh - y_nyc) > 1e-5f) ok = false;
    }
    assert(ok && "Matched variants must be identical");
    printf("PASS: test_matched_variants\n");
}

void test_bass_at_ccw() {
    WdfBigMuffToneStack s;
    s.init(48000.0f, WdfBigMuffToneStack::Variant::RamsHead);
    s.setTone(0.0f);
    float rms_100 = measureRMS(s, 100.0f, 48000.0f, 4800);
    s.reset(); s.setTone(0.0f);
    float rms_3k = measureRMS(s, 3000.0f, 48000.0f, 4800);
    assert(rms_100 > 3.0f * rms_3k && "Bass not dominant at k=0");
    printf("PASS: test_bass_at_ccw\n");
}

void test_treble_at_cw() {
    WdfBigMuffToneStack s;
    s.init(48000.0f, WdfBigMuffToneStack::Variant::RamsHead);
    s.setTone(1.0f);
    float rms_3k = measureRMS(s, 3000.0f, 48000.0f, 4800);
    s.reset(); s.setTone(1.0f);
    float rms_100 = measureRMS(s, 100.0f, 48000.0f, 4800);
    assert(rms_3k > 3.0f * rms_100 && "Treble not dominant at k=1");
    printf("PASS: test_treble_at_cw\n");
}

void test_stability() {
    const WdfBigMuffToneStack::Variant variants[] = {
        WdfBigMuffToneStack::Variant::Triangle, WdfBigMuffToneStack::Variant::RamsHead,
        WdfBigMuffToneStack::Variant::OpAmp, WdfBigMuffToneStack::Variant::CivilWar,
        WdfBigMuffToneStack::Variant::NYC };
    for (int v = 0; v < 5; ++v) {
        WdfBigMuffToneStack s;
        s.init(48000.0f, variants[v]); s.setTone(0.5f);
        for (int i = 0; i < 4800; ++i) {
            float y = s.process((i % 2 == 0) ? 0.9f : -0.9f);
            assert(std::isfinite(y) && "NaN/Inf detected");
        }
    }
    printf("PASS: test_stability\n");
}

int main() {
    test_dc_blocking();
    test_triangle_midnotch();
    test_civilwar_notch_higher();
    test_matched_variants();
    test_bass_at_ccw();
    test_treble_at_cw();
    test_stability();
    printf("\nAll tests passed.\n");
    return 0;
}
