// Dytronics Tri-Stereo Chorus — Session 4 voice-assembly self-test harness.
//
// Thin translation unit around the self-test main embedded in
// wdf/TscCircuit.h (per session spec: test lives under
// #ifdef TSC_CIRCUIT_SELFTEST in the header).
//
// Build & run (single command line, wrapped here for readability):
//   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra
//       -DENDLESS_DESKTOP_BUILD -I. -I./sdk -I./wdf -I./dsp
//       tests/tsc_circuit_test.cpp -lm -o tsc_circuit_test && ./tsc_circuit_test
//
// Expected: WDF input coupling within 0.5 dB of the analytic HPF, per-line
// CV spacing 120 deg within 2 deg and stable over 60 s (both generators),
// per-line delay swing strictly increasing with intensity, finite nonzero
// audio with center-split L/R symmetry within 0.1 dB, and rate clamping to
// the 0.03..7.45 Hz manual range.

#define TSC_CIRCUIT_SELFTEST
#include "wdf/TscCircuit.h"
