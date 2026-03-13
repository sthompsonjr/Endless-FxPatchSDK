#pragma once

#include "WdfPort.h"

/// Two-port series adaptor.
/// Connects ChildA and ChildB in series. ChildB is the "adapted" port
/// (its Rp is determined by the adaptor, not independently).
///
/// Junction law: Rp = Rp_A + Rp_B
/// Scattering: uses standard series junction equations.
template<typename ChildA, typename ChildB>
class WdfSeriesAdaptor2 {
public:
    WdfPort port;
    ChildA childA;
    ChildB childB;

    void init(float sampleRate) {
        childA.init(sampleRate);
        childB.init(sampleRate);
        updatePortResistance();
    }

    /// Overload for components that need different init signatures.
    /// Call childA.init(...) and childB.init(...) manually before calling this.
    void updatePortResistance() {
        port.Rp = childA.port.Rp + childB.port.Rp;
    }

    /// Bottom-up: reflect children, then compute adaptor reflection.
    void reflect() noexcept {
        childA.reflect();
        childB.reflect();
        port.b = -(childA.port.b + childB.port.b);
    }

    /// Top-down: scatter incident wave to children.
    void scatter() noexcept {
        childA.port.a = port.a - childB.port.b;
        childB.port.a = port.a - childA.port.b;
    }

    void reset() noexcept {
        childA.reset();
        childB.reset();
        port.reset();
    }
};

/// Two-port parallel adaptor.
/// Connects ChildA and ChildB in parallel. ChildB is adapted.
///
/// Junction law: 1/Rp = 1/Rp_A + 1/Rp_B
/// Uses reflection coefficient gamma for efficient scattering.
template<typename ChildA, typename ChildB>
class WdfParallelAdaptor2 {
public:
    WdfPort port;
    ChildA childA;
    ChildB childB;

    void init(float sampleRate) {
        childA.init(sampleRate);
        childB.init(sampleRate);
        updatePortResistance();
    }

    void updatePortResistance() {
        port.Rp = (childA.port.Rp * childB.port.Rp) /
                  (childA.port.Rp + childB.port.Rp);
        gamma_ = childA.port.Rp / (childA.port.Rp + childB.port.Rp);
    }

    void reflect() noexcept {
        childA.reflect();
        childB.reflect();
        port.b = childA.port.b + childB.port.b - port.a;
        // Corrected: standard parallel junction reflection
        // b_root = gamma * b_A + (1 - gamma) * b_B
        port.b = gamma_ * childA.port.b + (1.0f - gamma_) * childB.port.b;
    }

    void scatter() noexcept {
        float diff = childB.port.b - childA.port.b;
        childA.port.a = port.a + gamma_ * diff;
        childB.port.a = port.a - (1.0f - gamma_) * diff;
    }

    void reset() noexcept {
        childA.reset();
        childB.reset();
        port.reset();
    }

private:
    float gamma_ = 0.5f;
};

/// Three-port series adaptor.
/// P1 and P2 are children; P3 is the root (unadapted) port.
/// Junction law: Rp3 = Rp1 + Rp2
template<typename P1, typename P2>
class WdfSeriesAdaptor3 {
public:
    WdfPort port; // root port (P3)
    P1 child1;
    P2 child2;

    void init(float sampleRate) {
        child1.init(sampleRate);
        child2.init(sampleRate);
        updatePortResistance();
    }

    void updatePortResistance() {
        port.Rp = child1.port.Rp + child2.port.Rp;
        gamma1_ = child1.port.Rp / port.Rp;
        gamma2_ = child2.port.Rp / port.Rp;
    }

    /// Reflect: bottom-up gather from children.
    void reflect() noexcept {
        child1.reflect();
        child2.reflect();
        port.b = -(child1.port.b + child2.port.b);
    }

    /// Scatter: top-down distribute to children from root.
    void scatter() noexcept {
        // Standard 3-port series scatter:
        child1.port.a = port.a - child2.port.b;
        child2.port.a = port.a - child1.port.b;
    }

    void reset() noexcept {
        child1.reset();
        child2.reset();
        port.reset();
    }

private:
    float gamma1_ = 0.5f;
    float gamma2_ = 0.5f;
};

/// Three-port parallel adaptor.
/// P1 and P2 are children; P3 is the root (unadapted) port.
/// Junction law: 1/Rp3 = 1/Rp1 + 1/Rp2
template<typename P1, typename P2>
class WdfParallelAdaptor3 {
public:
    WdfPort port; // root port (P3)
    P1 child1;
    P2 child2;

    void init(float sampleRate) {
        child1.init(sampleRate);
        child2.init(sampleRate);
        updatePortResistance();
    }

    void updatePortResistance() {
        port.Rp = (child1.port.Rp * child2.port.Rp) /
                  (child1.port.Rp + child2.port.Rp);
        gamma1_ = child1.port.Rp / (child1.port.Rp + child2.port.Rp);
    }

    void reflect() noexcept {
        child1.reflect();
        child2.reflect();
        port.b = gamma1_ * child1.port.b + (1.0f - gamma1_) * child2.port.b;
    }

    void scatter() noexcept {
        float diff = child2.port.b - child1.port.b;
        child1.port.a = port.a + gamma1_ * diff;
        child2.port.a = port.a - (1.0f - gamma1_) * diff;
    }

    void reset() noexcept {
        child1.reset();
        child2.reset();
        port.reset();
    }

private:
    float gamma1_ = 0.5f;
};
