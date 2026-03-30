#pragma once

#include "WdfPort.h"
#include <cmath>
#include <algorithm>

// WdfJfetFamily — parametric N-channel JFET, analytical triode/saturation model.
// Functionally identical to WdfJfet but accepts Vp and Idss at init()
// rather than through a Params struct. Enables jfet_params namespace selection.
//
// Use: init(Vp, Idss), then setGateVoltage(Vgs) each sample, then reflect().
// The caller (adaptor tree) reads port.Rp after setGateVoltage() to propagate
// updated port resistance to the adaptor chain before scatter.
//
// Physics: JFET triode square-law model.
//   Rds = Vp² / (2 × Idss × (Vgs - Vp))   [exact within square-law model]
// Boundary guards prevent divide-by-zero near pinch-off.
// Reflected wave b = 0 (resistor model — all energy absorbed).
class WdfJfetFamily {
public:
    WdfPort port;   // Drain-Source port. port.Rp = Rds(Vgs).

    WdfJfetFamily() = default;

    // Initialize device parameters. Does NOT set port.Rp — call setGateVoltage() first.
    // vp:   pinch-off voltage (V). Must be negative for N-channel.
    // idss: drain saturation current (A) at Vgs=0.
    void init(float vp, float idss) noexcept {
        Vp   = std::min(vp,   -1.0e-4f);   // must be negative
        Idss = std::max(idss,  1.0e-9f);   // must be positive
    }

    // Set gate-to-source voltage. Updates port.Rp = Rds(Vgs).
    // Call once per sample before scatter phase of adaptor tree.
    // Vgs clamped to [Vp*1.1, 0]; no error produced for out-of-range values.
    void setGateVoltage(float Vgs) noexcept {
        float Vgs_c = std::clamp(Vgs, Vp * 1.1f, 0.0f);
        currentVgs  = Vgs_c;
        currentRds  = computeRds(Vgs_c);
        port.Rp     = currentRds;
    }

    // Compute Rds for given Vgs without updating port.Rp. Used for testing.
    // Positive Vgs (forward-biased gate, outside model validity) returns kRdsMax.
    // Vgs below Vp (below pinch-off) is clamped to prevent divide-by-zero.
    [[nodiscard]] float computeRds(float Vgs) const noexcept {
        if (Vgs > 0.0f) return kRdsMax;   // positive Vgs: gate forward-biased, outside model
        float Vgs_c   = std::clamp(Vgs, Vp * 1.1f, 0.0f);
        float headroom = Vgs_c - Vp;
        if (headroom < 1.0e-4f) return kRdsMax;
        float Rds = (Vp * Vp) / (2.0f * Idss * headroom);
        return std::clamp(Rds, kRdsMin, kRdsMax);
    }

    // Audio-path. Triode region: b = 0 (resistor model — all energy absorbed).
    // MUST be noexcept.
    void reflect() noexcept {
        port.b = 0.0f;
    }

    void reset() noexcept {
        currentVgs = Vp * 0.5f;
        setGateVoltage(currentVgs);
        port.a = 0.0f;
        port.b = 0.0f;
    }

    [[nodiscard]] float getCurrentRds() const noexcept { return currentRds; }
    [[nodiscard]] float getCurrentVgs() const noexcept { return currentVgs; }

private:
    float Vp         = -3.0f;
    float Idss       = 5.0e-3f;
    float currentRds = 1000.0f;
    float currentVgs = -1.5f;

    // Guard values — not device parameters
    static constexpr float kRdsMax = 1.0e6f;
    static constexpr float kRdsMin = 10.0f;
};

// ──────────────────────────────────────────────────────────────────────────────
// jfet_params namespace — named device parameter structs for WdfJfetFamily.
//
// NOTE on J201: jfet_params::J201 uses Vp=-0.8f, Idss=0.6e-3f (Linear Systems
// datasheet). The existing jfet_presets::J201 (in WdfJfet.h, if present) uses
// Vp=-0.4f, Idss=0.5e-3f — an alternate characterization for a different unit
// spread. Both are physically valid. Do NOT merge or reconcile these namespaces;
// they model different operating-point assumptions of the same device.
// ──────────────────────────────────────────────────────────────────────────────
namespace jfet_params {

    // Source: Linear Systems J201 datasheet. Classic low-Vp phaser JFET.
    // NOTE: these values differ from existing jfet_presets::J201 (Vp=-0.4, Idss=0.5mA).
    // The existing preset is an alternate characterization. Both are valid for
    // different unit spreads. See pre-flight note in prompt Section 2.
    struct J201 {
        static constexpr float Vp   = -0.8f;    // pinch-off voltage, V
        static constexpr float Idss = 0.6e-3f;  // drain saturation current, A
    };

    // Source: Vishay/Siliconix J310 datasheet. Higher Idss variant.
    struct J310 {
        static constexpr float Vp   = -2.0f;
        static constexpr float Idss = 1.0e-3f;
    };

    // Source: RCA 2N3819 datasheet. Classic vintage JFET, wide Vgs range.
    struct N2N3819 {
        static constexpr float Vp   = -3.0f;
        static constexpr float Idss = 5.0e-3f;
    };

} // namespace jfet_params
