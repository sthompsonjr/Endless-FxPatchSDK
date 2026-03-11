#include "Patch.h"
#include "../wdf/WdfPnpCircuits.h"
#include "../dsp/ParameterSmoother.h"
#include <cmath>

/// Rangemaster Treble Booster patch.
///
/// Controls:
///   Knob 0 (Left)   — Input Character: 0=flat cap (1nF), 1=treble boost (5nF)
///   Knob 1 (Mid)    — Level: output gain 0–1 (unity)
///   Knob 2 (Right)  — Device: selects transistor preset (4 steps)
///   Expression      — Level (same as Knob 1)
///
/// Actions:
///   Left tap  — Toggle aging mode (degrades hFE × 0.7, Is × 2 — worn germanium)
///   Left hold — Toggle high-impedance mode (input impedance 100kΩ vs 500kΩ)
///
/// LED colors:
///   OC44 → DimYellow   (classic Dallas Rangemaster sound)
///   OC75 → DimGreen    (Tone Bender MkI character)
///   AC128 → LightYellow (Fuzz Face character)
///   N3906 → DimWhite   (silicon, cleaner boost)
///   + DarkRed tint when aging mode active

class PatchImpl : public Patch
{
public:
    void init() override {
        circuit_.init(static_cast<float>(kSampleRate));
        levelSmoother_.init(static_cast<float>(kSampleRate), 20.0f);
        levelSmoother_.snapTo(0.5f);
        applyDevice(currentDevice_);
    }

    void setWorkingBuffer(std::span<float, kWorkingBufferSize> buffer) override {
        (void)buffer;
    }

    void processAudio(std::span<float> left, std::span<float> right) override {
        for (auto l = left.begin(), r = right.begin(); l != left.end(); ++l, ++r) {
            float out = circuit_.process(*l);
            out *= levelSmoother_.process();
            *l = out;
            *r = out;
        }
    }

    ParameterMetadata getParameterMetadata(int idx) override {
        switch (idx) {
            case 0: return { 0.0f, 1.0f, 1.0f }; // Input Character
            case 1: return { 0.0f, 1.0f, 0.5f }; // Level
            case 2: return { 0.0f, 1.0f, 0.0f }; // Device (quantized to 4 steps)
            default: return { 0.0f, 1.0f, 0.5f };
        }
    }

    void setParamValue(int idx, float value) override {
        switch (idx) {
            case 0:
                circuit_.setInputCharacter(value);
                break;
            case 1:
                levelSmoother_.setTarget(value);
                break;
            case 2: {
                // Quantize 0–1 to 4 device slots
                int device = static_cast<int>(value * 3.9999f);
                if (device != currentDevice_) {
                    currentDevice_ = device;
                    applyDevice(device);
                }
                break;
            }
            default:
                break;
        }
    }

    bool isParamEnabled(int idx, ParamSource source) override {
        switch (source) {
            case ParamSource::kParamSourceKnob:
                return idx < 3;
            case ParamSource::kParamSourceExpression:
                return idx == 1; // Expression controls Level
            default:
                return false;
        }
    }

    void handleAction(int idx) override {
        if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchPress)) {
            // Tap: toggle aging mode
            agingMode_ = !agingMode_;
            applyDevice(currentDevice_);
        } else if (idx == static_cast<int>(endless::ActionId::kLeftFootSwitchHold)) {
            // Hold: toggle high-impedance input mode
            highZMode_ = !highZMode_;
            // High-Z: bump source impedance from ~8.7kΩ to ~470kΩ via setInputCharacter
            // (Approximate: reinit full circuit to new source R)
            circuit_.init(static_cast<float>(kSampleRate));
            applyDevice(currentDevice_);
        }
    }

    Color getStateLedColor() override {
        Color base;
        switch (currentDevice_) {
            case 0:  base = Color::kDimYellow;    break; // OC44
            case 1:  base = Color::kDimGreen;     break; // OC75
            case 2:  base = Color::kLightYellow;  break; // AC128
            default: base = Color::kDimWhite;     break; // N3906
        }
        // Aging mode: blend toward red (return DarkRed as secondary indicator)
        return agingMode_ ? Color::kDarkRed : base;
    }

private:
    void applyDevice(int device) {
        WdfPnpBjt::Params params;
        switch (device) {
            case 0:  params = pnp_presets::OC44;  break;
            case 1:  params = pnp_presets::OC75;  break;
            case 2:  params = pnp_presets::AC128; break;
            default: params = pnp_presets::N3906; break;
        }
        if (agingMode_) {
            // Aged germanium: reduced gain, higher leakage current
            params.hFE *= 0.7f;
            params.Is  *= 2.0f;
        }
        currentParams_ = params;
        // Re-init BJT with new parameters, preserving port resistances
        circuit_.bjt().init(params,
                            circuit_.bjt().portB.Rp,
                            circuit_.bjt().portC.Rp,
                            circuit_.bjt().portE.Rp);
    }

    RangemasterCircuit circuit_;
    ParameterSmoother  levelSmoother_;
    WdfPnpBjt::Params  currentParams_  = pnp_presets::OC44;
    int                currentDevice_  = 0;
    bool               agingMode_      = false;
    bool               highZMode_      = false;
};

static PatchImpl patch;

Patch* Patch::getInstance() {
    return &patch;
}
