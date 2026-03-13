// wdf/wdf.h — WDF analog modeling primitives umbrella header
// Include order strictly follows dependency graph.
#pragma once

// TODO: Implement WDF primitives. Planned files:
//
// Foundation (no WDF dependencies)
//   #include "WdfPort.h"
//
// Math utilities
//   #include "LambertW.h"
//   #include "NewtonRaphson.h"
//
// One-port elements (depend only on WdfPort)
//   #include "WdfOnePort.h"
//
// Adaptors (depend on WdfPort and one-port elements)
//   #include "WdfAdaptors.h"
//
// Nonlinear elements (depend on WdfPort, LambertW, NewtonRaphson)
//   #include "WdfNonlinear.h"
//
// Transistors (depend on WdfPort, NewtonRaphson)
//   #include "WdfPnpBjt.h"
//   #include "WdfJfet.h"
//
// Op-amps (depend on WdfPort; independent of each other)
//   #include "WdfOpAmpLM741.h"
//   #include "WdfOpAmpJRC4558.h"
//   #include "WdfOpAmpLM308.h"
//
// Voltage-controlled elements (depend on WdfPort, NewtonRaphson)
//   #include "WdfOta.h"
//   #include "WdfPhotoresistor.h"
//
// Phase shifter stages (depend on WdfJfet, WdfOpAmpLM741, dsp/OnePoleFilter)
//   #include "WdfPhaserStage.h"
//
// Circuit assemblies (depend on all elements above)
//   #include "WdfCircuits.h"
//   #include "WdfPnpCircuits.h"
//   #include "WdfWahCircuit.h"
//   #include "WdfTubescreamerCircuit.h"
//   #include "WdfCompressorCircuits.h"
//   #include "WdfOpticalCircuits.h"
//   #include "WdfPhaserCircuits.h"
//   #include "WdfRatCircuit.h"
