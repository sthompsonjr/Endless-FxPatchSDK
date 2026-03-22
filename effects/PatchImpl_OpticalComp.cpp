#include "sdk/Patch.h"
#include "wdf/WdfOpticalCircuits.h"
#include <algorithm>
#include <cmath>

/// Optical Compressor patch — three circuits selectable via right knob.
///
/// Control mapping:
///   Left knob  (idx 0): Peak Reduction / Sensitivity
///   Mid knob   (idx 1): Gain (makeup gain — manual)
///   Right knob (idx 2): Circuit select:
///                         0.00–0.33: PC-2A (full optical, LA-2A style)
///                         0.33–0.66: Optical Leveler (single-knob, simpler)
///                         0.66–1.00: Hybrid Opt/OTA (optical detector + OTA VCA)
///   Expression (idx 0): HF Emphasis (0.0=flat sidechain, 1.0=treble-focused)
///
///   Left tap:   Toggle Attack mode (fast: tau_fast≈5ms / slow: tau_fast≈50ms)
///   Left hold:  Toggle Release mode (short: tau_release≈200ms / long: tau_release≈2000ms)
///
/// LED:
///   PC-2A:   kLightYellow (vintage gold)
///   Leveler: kDimYellow   (simpler, dimmer)
///   Hybrid:  kDimCyan     (modern hybrid)
///   GR meter: LDR inertia dims the LED — high inertia (heavy compression) = dimmer LED
///   Fast attack mode: LED shifts toward kDimWhite (brighter/crisper look)
///   Long release: LED holds brightness longer (mirrors LDR behavior — organic)

class PatchImpl : public Patch
{
public:
    void init() override
    {
        float sr = static_cast<float>(kSampleRate);

        pc2a_.init(sr);
        pc2a_.setPeakReduction(peakReduction_);
        pc2a_.setGain(gain_);
        pc2a_.setHFEmphasis(hfEmphasis_);

        leveler_.init(sr);
        leveler_.setThreshold(peakReduction_);

        hybrid_.init(sr);
        hybrid_.setSensitivity(peakReduction_);
        hybrid_.setMakeupGain(gain_);
    }

    void setWorkingBuffer(std::span<float, kWorkingBufferSize> buffer) override
    {
        (void)buffer;
    }

    void processAudio(std::span<float> audioBufferLeft,
                      std::span<float> audioBufferRight) override
    {
        for (auto leftIt = audioBufferLeft.begin(), rightIt = audioBufferRight.begin();
             leftIt != audioBufferLeft.end();
             ++leftIt, ++rightIt)
        {
            float inL = *leftIt;
            float inR = *rightIt;
            float outL = 0.0f;
            float outR = 0.0f;

            if (circuit_ == 0) {
                // PC-2A: linked stereo compression
                pc2a_.processStereo(inL, inR, outL, outR);
            } else if (circuit_ == 1) {
                // Optical Leveler: process each channel independently with same threshold
                outL = leveler_.process(inL);
                outR = leveler_.process(inR);
            } else {
                // Hybrid Opt/OTA: process each channel independently
                outL = hybrid_.process(inL);
                outR = hybrid_.process(inR);
            }

            *leftIt  = outL;
            *rightIt = outR;
        }
    }

    ParameterMetadata getParameterMetadata(int paramIdx) override
    {
        switch (paramIdx)
        {
            case static_cast<int>(endless::ParamId::kParamLeft):   // Peak Reduction / Sensitivity
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
            case static_cast<int>(endless::ParamId::kParamMid):    // Gain (makeup)
                return ParameterMetadata{ 0.0f, 1.0f, 0.5f };
            case static_cast<int>(endless::ParamId::kParamRight):  // Circuit select
                return ParameterMetadata{ 0.0f, 1.0f, 0.0f };
            default:
                return ParameterMetadata{ 0.0f, 1.0f, 0.0f };
        }
    }

    bool isParamEnabled(int paramIdx, ParamSource source) override
    {
        if (source == ParamSource::kParamSourceExpression && paramIdx == 0) {
            return true;  // Expression pedal controls HF Emphasis (idx 0)
        }
        // All knob parameters enabled
        if (source == ParamSource::kParamSourceKnob) {
            return paramIdx >= 0 && paramIdx <= 2;
        }
        return false;
    }

    void setParamValue(int paramIdx, float value) override
    {
        switch (paramIdx)
        {
            case static_cast<int>(endless::ParamId::kParamLeft):
                // Peak Reduction / Sensitivity — maps to all three circuits
                peakReduction_ = value;
                pc2a_.setPeakReduction(value);
                leveler_.setThreshold(value);
                hybrid_.setSensitivity(value);
                break;

            case static_cast<int>(endless::ParamId::kParamMid):
                // Gain (makeup) — for PC-2A and Hybrid
                gain_ = value;
                pc2a_.setGain(value);
                hybrid_.setMakeupGain(value);
                break;

            case static_cast<int>(endless::ParamId::kParamRight):
                // Circuit select: 0–0.33 = PC-2A, 0.33–0.66 = Leveler, 0.66–1 = Hybrid
                if (value < 0.33f) {
                    circuit_ = 0;
                } else if (value < 0.66f) {
                    circuit_ = 1;
                } else {
                    circuit_ = 2;
                }
                break;

            default:
                break;
        }
    }

    void handleAction(int actionIdx) override
    {
        switch (actionIdx)
        {
            case static_cast<int>(endless::ActionId::kLeftFootSwitchPress):
                // Toggle attack mode: fast (5ms) ↔ slow (50ms)
                fastAttack_ = !fastAttack_;
                applyTimeConstants();
                break;

            case static_cast<int>(endless::ActionId::kLeftFootSwitchHold):
                // Toggle release mode: short (200ms) ↔ long (2000ms)
                longRelease_ = !longRelease_;
                applyTimeConstants();
                break;

            default:
                break;
        }
    }

    Color getStateLedColor() override
    {
        // Base color by circuit type
        Color base;
        float inertia = 0.0f;

        if (circuit_ == 0) {
            base    = Color::kLightYellow;
            inertia = pc2a_.getLdrInertia();
        } else if (circuit_ == 1) {
            base    = Color::kDimYellow;
            inertia = 0.5f;  // Leveler doesn't expose inertia directly; use static dim
        } else {
            base    = Color::kDimCyan;
            inertia = 0.5f;
        }

        // Fast attack: blend toward kDimWhite (brighter, crisper look)
        if (fastAttack_) {
            return Color::kDimWhite;
        }

        // GR meter: high inertia (heavy compression) → use dimmer color variant
        // Use a simple threshold: inertia > 0.5 → dimmer
        if (inertia > 0.5f) {
            // Return dim version of the base color
            if (circuit_ == 0) return Color::kDimYellow;   // dim yellow (was light yellow)
            if (circuit_ == 2) return Color::kDimBlue;     // dim blue (was dim cyan)
        }

        return base;
    }

private:
    PC2ACircuit           pc2a_;
    OpticalLevelerCircuit leveler_;
    HybridOptOtaCircuit   hybrid_;

    int   circuit_      = 0;      // 0=PC-2A, 1=Leveler, 2=Hybrid
    float peakReduction_ = 0.5f;
    float gain_          = 0.5f;
    float hfEmphasis_    = 0.0f;
    bool  fastAttack_    = false;  // false=default (50ms), true=fast (5ms)
    bool  longRelease_   = false;  // false=short (200ms), true=long (2000ms)

    /// Apply current attack/release time constant settings to all circuits.
    void applyTimeConstants() noexcept
    {
        float attackVal  = fastAttack_  ? 0.0f : 1.0f;  // 0=fast(5ms), 1=slow(50ms)
        float releaseVal = longRelease_ ? 1.0f : 0.0f;  // 0=short(200ms), 1=long(2000ms)
        hybrid_.setAttack(attackVal);
        hybrid_.setRelease(releaseVal);
        // PC-2A and Leveler don't expose time constant controls in this implementation
    }
};

Patch* Patch::getInstance()
{
    static PatchImpl instance;
    return &instance;
}
