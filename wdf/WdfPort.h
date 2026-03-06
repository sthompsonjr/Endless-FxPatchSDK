#pragma once

/// Wave variable port — the atomic unit of every WDF circuit.
///
/// Each port carries an incident wave 'a' (input) and reflected wave 'b' (output),
/// along with a port resistance Rp that characterizes the port's impedance.
///
/// Wave variables relate to conventional voltage/current by:
///   a = V + Rp * I   (incident)
///   b = V - Rp * I   (reflected)
///   V = 0.5 * (a + b)
///   I = (a - b) / (2 * Rp)
///
/// IMPORTANT: Rp must never be zero — this causes division by zero in current().
/// Components that set Rp must ensure it is always positive and finite.
struct WdfPort {
    float Rp = 1.0f;   // port resistance (Ohms)
    float a  = 0.0f;   // incident wave
    float b  = 0.0f;   // reflected wave

    [[nodiscard]] float voltage() const noexcept { return 0.5f * (a + b); }
    [[nodiscard]] float current() const noexcept { return (a - b) / (2.0f * Rp); }
    void reset() noexcept { a = 0.0f; b = 0.0f; }
};
