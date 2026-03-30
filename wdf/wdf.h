#pragma once

// Wave Digital Filter library for Polyend Endless
// Provides analog circuit modeling via the WDF formalism.
// All components are no-heap, single-precision, C++20.
//
// Consolidated from all feature branches into dependency order.

// ── Foundation (no WDF dependencies) ──────────────────────────────────────
#include "WdfPort.h"

// ── Math utilities ─────────────────────────────────────────────────────────
#include "LambertW.h"
#include "NewtonRaphson.h"

// ── One-port elements (depend only on WdfPort) ─────────────────────────────
#include "WdfOnePort.h"

// ── Adaptors (depend on WdfPort and one-port elements) ─────────────────────
#include "WdfAdaptors.h"

// ── Nonlinear elements (depend on WdfPort, LambertW, NewtonRaphson) ────────
#include "WdfNonlinear.h"     // WdfIdealDiode, WdfAntiparallelDiodes, WdfNpnBjt,
                               // WdfDiodeToGround, diode_physics, diode_params
#include "WdfDiodeFamily.h"              // WdfDiodeFamily (parametric single diode, Lambert W)
#include "WdfAntiparallelDiodeFamily.h"  // WdfAntiparallelDiodeFamily (parametric pair, 1D NR)

// ── Op-amps (depend on WdfPort; independent of each other) ─────────────────
#include "WdfOpAmpLM741.h"
#include "WdfOpAmpLM308.h"
#include "WdfOpAmpJRC4558.h"

// ── Transistors (depend on WdfPort, NewtonRaphson) ─────────────────────────
#include "WdfPnpBjt.h"
#include "WdfNpnBjtFamily.h"   // WdfNpnBjtFamily (parametric NPN, Ebers-Moll + Early, 2D NR)
#include "WdfPnpBjtFamily.h"   // WdfPnpBjtFamily (parametric PNP, Ebers-Moll + Early, 2D NR)
#include "WdfJfetFamily.h"     // WdfJfetFamily (parametric N-channel JFET, analytical triode)
#include "WdfMosfetFamily.h"   // WdfMosfetFamily (parametric N-channel MOSFET, soft-clipper)

// ── Voltage-controlled elements (depend on WdfPort, NewtonRaphson) ─────────
#include "WdfOta.h"
#include "WdfPhotoresistor.h"

// ── Inverting stages (depend on WdfAdaptors, op-amps) ──────────────────────
#include "WdfInvertingStage.h"

// ── Circuit assemblies (depend on elements above) ──────────────────────────
#include "WdfCircuits.h"              // RCLowpass, DiodeClipper, ToneStack, BJTGainStage
#include "DOD250Circuit.h"            // DOD 250 overdrive
#include "WdfRatCircuit.h"            // ProCo RAT distortion
#include "WdfTubescreamerCircuit.h"   // TS808, TS9, Klon
#include "WdfPnpCircuits.h"           // Rangemaster, FuzzFace, ToneBender
#include "WdfWahCircuit.h"            // CryBaby wah, AutoWah
#include "WdfCompressorCircuits.h"    // Dynacomp, OrangeSqz
#include "WdfOpticalCircuits.h"       // PC-2A, OpticalLeveler, HybridOptOta
#include "WdfBigMuffCircuit.h"        // EHX Big Muff Pi fuzz
