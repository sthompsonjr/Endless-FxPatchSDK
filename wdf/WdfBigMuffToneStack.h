#pragma once
#include <algorithm>
#include <cmath>

namespace bmp_tone_params {
    static constexpr float R_pot = 100000.0f;
    static constexpr float C8 = 10e-9f;
    static constexpr float R_source = 15000.0f;

    struct ToneParams { float R8, R5, C9, f_lp, f_hp, f_notch; };

    static constexpr ToneParams kTriangle = {33000.0f,33000.0f,4e-9f,481.9f,1206.8f,762.8f};
    static constexpr ToneParams kRamsHead = {39000.0f,22000.0f,4e-9f,408.0f,1808.5f,858.9f};
    static constexpr ToneParams kOpAmp = {39000.0f,22000.0f,4e-9f,408.0f,1808.5f,858.9f};
    static constexpr ToneParams kCivilWar = {20000.0f,22000.0f,4e-9f,795.8f,1808.5f,1199.3f};
    static constexpr ToneParams kNYC = {39000.0f,22000.0f,4e-9f,408.0f,1808.5f,858.9f};
}

class WdfBigMuffToneStack {
public:
    enum class Variant { Triangle, RamsHead, OpAmp, CivilWar, NYC };

    void init(float sampleRate, Variant variant) noexcept {
        sampleRate_ = sampleRate;
        loadVariantParams(variant);
        toneK_ = 0.5f;
        s1_ = 0.0f;
        s2_ = 0.0f;
        dcX1_ = 0.0f;
        dcY1_ = 0.0f;
        recomputeCoeffs();
    }

    void reset() noexcept {
        s1_ = 0.0f;
        s2_ = 0.0f;
        dcX1_ = 0.0f;
        dcY1_ = 0.0f;
    }

    [[nodiscard]] float process(float x) noexcept {
        const float v = b0_ * x + s1_;
        s1_ = b1_ * x - a1_ * v + s2_;
        s2_ = b0_ * x - a2_ * v;
        const float y = v - dcX1_ + dcR_ * dcY1_;
        dcX1_ = v;
        dcY1_ = y;
        return y;
    }

    void setTone(float k) noexcept {
        toneK_ = std::clamp(k, 0.0f, 1.0f);
        recomputeCoeffs();
    }

    void setVariant(Variant variant) noexcept {
        loadVariantParams(variant);
        s1_ = 0.0f;
        s2_ = 0.0f;
        dcX1_ = 0.0f;
        dcY1_ = 0.0f;
        recomputeCoeffs();
    }

private:
    float sampleRate_ = 48000.0f;
    float toneK_ = 0.5f;
    float omega0_ = 4794.0f;
    float Q_ = 1.0f;

    float b0_ = 0.0f, b1_ = 0.0f;
    float a1_ = 0.0f, a2_ = 0.0f;
    float s1_ = 0.0f, s2_ = 0.0f;

    float dcR_ = 0.999f;
    float dcX1_ = 0.0f;
    float dcY1_ = 0.0f;

    void recomputeCoeffs() noexcept {
        const float k = toneK_;
        const float c = 2.0f * sampleRate_;
        const float c2 = c * c;
        const float w2 = omega0_ * omega0_;
        const float cw = c * omega0_ / Q_;

        const float norm = 1.0f / (c2 + cw + w2);
        b0_ = (k * c2 + (1.0f - k) * w2) * norm;
        b1_ = 2.0f * ((1.0f - k) * w2 - k * c2) * norm;
        a1_ = 2.0f * (w2 - c2) * norm;
        a2_ = (c2 - cw + w2) * norm;

        constexpr float kTwoPi = 6.2831853f;
        dcR_ = 1.0f - kTwoPi * 5.0f / sampleRate_;
    }

    void loadVariantParams(Variant v) noexcept {
        constexpr float kTwoPi = 6.2831853f;
        const bmp_tone_params::ToneParams* p = &bmp_tone_params::kRamsHead;
        switch (v) {
            case Variant::Triangle:  p = &bmp_tone_params::kTriangle;  break;
            case Variant::RamsHead:  p = &bmp_tone_params::kRamsHead;  break;
            case Variant::OpAmp:     p = &bmp_tone_params::kOpAmp;     break;
            case Variant::CivilWar:  p = &bmp_tone_params::kCivilWar;  break;
            case Variant::NYC:       p = &bmp_tone_params::kNYC;       break;
        }
        omega0_ = kTwoPi * p->f_notch;
        const float rho = p->f_hp / p->f_lp;
        Q_ = std::sqrt(rho) / (rho - 1.0f);
    }
};
