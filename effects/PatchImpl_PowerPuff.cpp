#include "sdk/Patch.h"
#include "wdf/WdfBigMuffCircuit.h"
#include "wdf/WdfOpAmpBigMuffCircuit.h"
#include "dsp/ParameterSmoother.h"
#include "effects/PowerPuffParams.h"
#include <cmath>
#include <algorithm>

// Power Puff — Five-Variant EHX Big Muff Pi
// ============================================
// Variants (Knob 0 selects):
//   0: Triangle  (V1, BC549C)  tone stack: R8=33kΩ R5=33kΩ C9=4nF notch≈763Hz  LED: kLightYellow
//   1: Rams Head (V2, BC549C)  tone stack: R8=39kΩ R5=22kΩ C9=4nF notch≈859Hz  LED: kMagenta
//   2: Op-Amp    (V4, LM741)   tone stack: R8=39kΩ R5=22kΩ C9=4nF notch≈859Hz  LED: kBeige
//   3: Civil War (V7, 2N3904)  tone stack: R8=20kΩ R5=22kΩ C9=4nF notch≈1199Hz LED: kDarkCobalt
//   4: NYC       (V9, 2N5088)  tone stack: R8=39kΩ R5=22kΩ C9=4nF notch≈859Hz  LED: kDarkRed
//
// Tone stack component values are baked into each circuit object at init() via
// bigmuff::Variant → WdfBigMuffToneStack::Variant mapping in WdfBigMuffCircuit.
// Knob 0 switches the active circuit; the correct tone stack activates automatically.
// Knob 1 controls tone pot position (0.0–1.0) within the active variant's stack.
// Knob 2 controls sustain/gain.
//
// Left footswitch tap:  toggle tone bypass across all variants (V5 feature)
// Left footswitch hold: no action
//
// Mono processing: Big Muff Pi is a mono circuit.
// Right output mirrors left. No sonic information lost.
// 100% wet mix — no dry signal.
//
// Volume is fixed at 0.7f; no knob available for it.
// kMagenta used for Rams Head: no purple/violet member in Color enum.
// kBeige used for Op-Amp: no orange/amber member in Color enum.

class PatchImpl : public Patch {
public:
    void init() noexcept override {
        // Pass variant at init — this sets BJT params AND tone stack component values
        // in one call (WdfBigMuffCircuit::init calls applyVariant + toneStack_.init).
        triangle_.init(static_cast<float>(kSampleRate), bigmuff::Variant::Triangle);
        ramsHead_.init(static_cast<float>(kSampleRate), bigmuff::Variant::RamsHead);
        civilWar_.init(static_cast<float>(kSampleRate), bigmuff::Variant::CivilWar);
        nyc_.init(static_cast<float>(kSampleRate), bigmuff::Variant::NYC);
        opAmp_.init(static_cast<float>(kSampleRate));
        // OpAmp circuit initialises its tone stack with Variant::OpAmp internally.

        toneSmoother_.init(static_cast<float>(kSampleRate), 20.0f);
        toneSmoother_.snapTo(0.5f);
        sustainSmoother_.init(static_cast<float>(kSampleRate), 20.0f);
        sustainSmoother_.snapTo(0.5f);

        setToneOnAll(0.5f);
        setSustainOnAll(0.5f);
        setVolumeOnAll(0.7f);
        setToneBypassOnAll(false);

        activeVariant_  = power_puff_params::kVariantTriangle;
        toneBypass_     = false;
        controlCounter_ = 0;
        targetVariant_  = 0.0f;
    }

    void setWorkingBuffer(std::span<float, kWorkingBufferSize> buffer) noexcept override {
        (void)buffer;  // No delay lines — working buffer not used.
    }

    void processAudio(std::span<float> left,
                      std::span<float> right) noexcept override {
        const int n = static_cast<int>(left.size());

        for (int i = 0; i < n; ++i) {
            const float smoothedTone    = toneSmoother_.process();
            const float smoothedSustain = sustainSmoother_.process();

            // Control-rate block: variant resolution + parameter push to active circuit.
            // Variant knob not smoothed — smoothing would produce undefined mid-topology state.
            if (++controlCounter_ >= power_puff_params::kControlRateDivider) {
                controlCounter_ = 0;

                int newVariant = static_cast<int>(
                    targetVariant_ * power_puff_params::kVariantScale);
                newVariant = std::clamp(newVariant, 0,
                    power_puff_params::kNumVariants - 1);

                if (newVariant != activeVariant_) {
                    activeVariant_ = newVariant;
                    // Reset clears stale WDF capacitor state and prevents click.
                    // Tone stack component values are already correct for the new
                    // variant (set at init time via bigmuff::Variant mapping).
                    resetActiveVariant();
                }
                applyParamsToActive(smoothedSustain, smoothedTone);
            }

            const float y = processActiveVariant(left[i]);
            left[i]  = y;
            right[i] = y;
        }
    }

    ParameterMetadata getParameterMetadata(int paramIdx) noexcept override {
        switch (paramIdx) {
            case power_puff_params::kKnobVariant:  return { 0.0f, 1.0f, 0.0f };
            case power_puff_params::kKnobTone:     return { 0.0f, 1.0f, 0.5f };
            case power_puff_params::kKnobSustain:  return { 0.0f, 1.0f, 0.5f };
            default:                               return { 0.0f, 1.0f, 0.0f };
        }
    }

    void setParamValue(int paramIdx, float value) noexcept override {
        value = std::clamp(value, 0.0f, 1.0f);
        switch (paramIdx) {
            case power_puff_params::kKnobVariant:  targetVariant_ = value;                break;
            case power_puff_params::kKnobTone:     toneSmoother_.setTarget(value);        break;
            case power_puff_params::kKnobSustain:  sustainSmoother_.setTarget(value);     break;
            default: break;
        }
    }

    bool isParamEnabled(int paramIdx, ParamSource source) noexcept override {
        return (paramIdx >= 0 && paramIdx <= 2 &&
                source == ParamSource::kParamSourceKnob);
    }

    void handleAction(int actionIdx) noexcept override {
        if (actionIdx == static_cast<int>(endless::ActionId::kLeftFootSwitchPress)) {
            toneBypass_ = !toneBypass_;
            setToneBypassOnAll(toneBypass_);
        }
        // kLeftFootSwitchHold: intentionally no action
    }

    Color getStateLedColor() noexcept override {
        if (toneBypass_) return Color::kDimWhite;

        switch (activeVariant_) {
            case power_puff_params::kVariantTriangle:  return Color::kLightYellow;
            case power_puff_params::kVariantRamsHead:  return Color::kMagenta;    // closest to violet/purple
            case power_puff_params::kVariantOpAmp:     return Color::kBeige;      // closest to orange/amber
            case power_puff_params::kVariantCivilWar:  return Color::kDarkCobalt;
            case power_puff_params::kVariantNYC:       return Color::kDarkRed;
            default:                                   return Color::kDimWhite;
        }
    }

private:
    BigMuffCircuit           triangle_;  // V1: BC549C, tone R8=33k R5=33k C9=4nF
    BigMuffCircuit           ramsHead_;  // V2: BC549C, tone R8=39k R5=22k C9=4nF
    BigMuffCircuit           civilWar_;  // V7: 2N3904, tone R8=20k R5=22k C9=4nF
    BigMuffCircuit           nyc_;       // V9: 2N5088, tone R8=39k R5=22k C9=4nF
    WdfOpAmpBigMuffCircuit   opAmp_;     // V4: LM741,  tone R8=39k R5=22k C9=4nF

    ParameterSmoother toneSmoother_;
    ParameterSmoother sustainSmoother_;

    int   activeVariant_  = power_puff_params::kVariantTriangle;
    bool  toneBypass_     = false;
    int   controlCounter_ = 0;
    float targetVariant_  = 0.0f;

    [[nodiscard]] float processActiveVariant(float x) noexcept {
        switch (activeVariant_) {
            case power_puff_params::kVariantTriangle:  return triangle_.process(x);
            case power_puff_params::kVariantRamsHead:  return ramsHead_.process(x);
            case power_puff_params::kVariantOpAmp:     return opAmp_.process(x);
            case power_puff_params::kVariantCivilWar:  return civilWar_.process(x);
            case power_puff_params::kVariantNYC:       return nyc_.process(x);
            default:                                   return 0.0f;
        }
    }

    void applyParamsToActive(float sustain, float tone) noexcept {
        switch (activeVariant_) {
            case power_puff_params::kVariantTriangle:
                triangle_.setSustain(sustain);  triangle_.setTone(tone);  break;
            case power_puff_params::kVariantRamsHead:
                ramsHead_.setSustain(sustain);  ramsHead_.setTone(tone);  break;
            case power_puff_params::kVariantOpAmp:
                opAmp_.setSustain(sustain);     opAmp_.setTone(tone);     break;
            case power_puff_params::kVariantCivilWar:
                civilWar_.setSustain(sustain);  civilWar_.setTone(tone);  break;
            case power_puff_params::kVariantNYC:
                nyc_.setSustain(sustain);       nyc_.setTone(tone);       break;
            default: break;
        }
    }

    void setToneOnAll(float t) noexcept {
        triangle_.setTone(t);  ramsHead_.setTone(t);
        civilWar_.setTone(t);  nyc_.setTone(t);  opAmp_.setTone(t);
    }

    void setSustainOnAll(float s) noexcept {
        triangle_.setSustain(s);  ramsHead_.setSustain(s);
        civilWar_.setSustain(s);  nyc_.setSustain(s);  opAmp_.setSustain(s);
    }

    void setVolumeOnAll(float v) noexcept {
        triangle_.setVolume(v);  ramsHead_.setVolume(v);
        civilWar_.setVolume(v);  nyc_.setVolume(v);  opAmp_.setVolume(v);
    }

    void setToneBypassOnAll(bool bypass) noexcept {
        triangle_.setToneBypass(bypass);  ramsHead_.setToneBypass(bypass);
        civilWar_.setToneBypass(bypass);  nyc_.setToneBypass(bypass);
        opAmp_.setToneBypass(bypass);
    }

    void resetActiveVariant() noexcept {
        switch (activeVariant_) {
            case power_puff_params::kVariantTriangle:  triangle_.reset();  break;
            case power_puff_params::kVariantRamsHead:  ramsHead_.reset();  break;
            case power_puff_params::kVariantOpAmp:     opAmp_.reset();     break;
            case power_puff_params::kVariantCivilWar:  civilWar_.reset();  break;
            case power_puff_params::kVariantNYC:       nyc_.reset();       break;
            default: break;
        }
    }
};

static PatchImpl patch;

Patch* Patch::getInstance() {
    return &patch;
}
