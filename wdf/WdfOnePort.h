#pragma once

#include "WdfPort.h"
#include <algorithm>
#include <cmath>

/// WDF Resistor — pure absorption (all energy dissipated).
/// Reflection law: b = 0
class WdfResistor {
public:
    WdfPort port;

    void init(float R) {
        port.Rp = std::max(R, 1e-9f);
        port.reset();
    }

    void reflect() noexcept {
        port.b = 0.0f;
    }

    void reset() noexcept { port.reset(); }
};

/// WDF Capacitor — one-sample delay of incident wave.
/// Rp = 1 / (2 * C * fs), reflection: b[n] = a[n-1]
class WdfCapacitor {
public:
    WdfPort port;

    void init(float C, float sampleRate) {
        float c = std::max(C, 1e-15f);
        port.Rp = 1.0f / (2.0f * c * sampleRate);
        state_ = 0.0f;
        port.reset();
    }

    void reflect() noexcept {
        port.b = state_;
        state_ = port.a;
    }

    void reset() noexcept {
        state_ = 0.0f;
        port.reset();
    }

private:
    float state_ = 0.0f;
};

/// WDF Inductor — one-sample delay with sign flip.
/// Rp = 2 * L * fs, reflection: b[n] = -a[n-1]
class WdfInductor {
public:
    WdfPort port;

    void init(float L, float sampleRate) {
        float l = std::max(L, 1e-15f);
        port.Rp = 2.0f * l * sampleRate;
        state_ = 0.0f;
        port.reset();
    }

    void reflect() noexcept {
        port.b = -state_;
        state_ = port.a;
    }

    void reset() noexcept {
        state_ = 0.0f;
        port.reset();
    }

private:
    float state_ = 0.0f;
};

/// WDF Ideal Voltage Source — near-zero port resistance.
/// Reflection: b = 2*Vs - a
class WdfIdealVoltageSource {
public:
    WdfPort port;

    void init() {
        port.Rp = 1e-6f; // near-zero, not exactly zero
        Vs_ = 0.0f;
        port.reset();
    }

    void setVoltage(float Vs) noexcept { Vs_ = Vs; }

    void reflect() noexcept {
        port.b = 2.0f * Vs_ - port.a;
    }

    void reset() noexcept {
        Vs_ = 0.0f;
        port.reset();
    }

private:
    float Vs_ = 0.0f;
};

/// WDF Resistive Voltage Source — voltage source with series output resistance.
/// This is the correct way to inject audio into a WDF circuit.
/// Rp = R_source (output impedance of driving stage, e.g. 1kΩ)
/// Reflection: b = 2*Vs - a
class WdfResistiveVoltageSource {
public:
    WdfPort port;

    void init(float R_source) {
        port.Rp = std::max(R_source, 1e-9f);
        Vs_ = 0.0f;
        port.reset();
    }

    void setVoltage(float Vs) noexcept { Vs_ = Vs; }

    void reflect() noexcept {
        port.b = 2.0f * Vs_ - port.a;
    }

    void reset() noexcept {
        Vs_ = 0.0f;
        port.reset();
    }

private:
    float Vs_ = 0.0f;
};

/// WDF Resistive Current Source — current source with parallel resistance.
/// Rp = R_parallel
/// Reflection: b = a + 2*Rp*Is
class WdfResistiveCurrentSource {
public:
    WdfPort port;

    void init(float R_parallel) {
        port.Rp = std::max(R_parallel, 1e-9f);
        Is_ = 0.0f;
        port.reset();
    }

    void setCurrent(float Is) noexcept { Is_ = Is; }

    void reflect() noexcept {
        port.b = port.a + 2.0f * port.Rp * Is_;
    }

    void reset() noexcept {
        Is_ = 0.0f;
        port.reset();
    }

private:
    float Is_ = 0.0f;
};
