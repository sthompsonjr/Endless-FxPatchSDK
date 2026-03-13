// dsp/dsp.h — Foundational DSP primitives umbrella header
// Include order: primitives before composites, no circular dependencies.
#pragma once

// TODO: Implement DSP primitives. Planned files:
//
// Core utilities (no dependencies)
//   #include "Interpolation.h"
//   #include "Saturation.h"
//
// Single-state filters (depend only on math)
//   #include "OnePoleFilter.h"
//   #include "ParameterSmoother.h"
//
// Delay and buffer (depend on Interpolation)
//   #include "CircularBuffer.h"
//   #include "AllpassDelay.h"
//
// Modulators (depend on OnePoleFilter)
//   #include "Lfo.h"
//   #include "EnvelopeFollower.h"
//
// Reverb building blocks (depend on CircularBuffer, AllpassDelay, OnePoleFilter)
//   #include "ReverbPrimitives.h"
//
// Grain synthesis (depends on CircularBuffer, Interpolation, Lfo, EnvelopeFollower)
//   #include "GrainEnvelope.h"
//   #include "GrainScheduler.h"
//
// Spring reverb (depends on all of the above)
//   #include "SpringReverb.h"
