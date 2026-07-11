// Boss DC-2 Dimension C — Session 3 circuit-assembly self-test harness.
//
// Thin translation unit around the self-test main embedded in
// wdf/Dc2Circuit.h (per session spec: test lives under
// #ifdef DC2_CIRCUIT_SELFTEST in the header).
//
// Build & run (single command line, wrapped here for readability):
//   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra
//       -DENDLESS_DESKTOP_BUILD -I. -I./sdk -I./wdf -I./dsp
//       tests/dc2_circuit_test.cpp -lm -o dc2_circuit_test && ./dc2_circuit_test
//
// Expected: compander identity within 0.5 dB (-40..0 dBFS), motionless
// antiphase CV sum, mode table with modes 1,2 slower than 3,4, and
// finite in-phase audio in all four modes.

#define DC2_CIRCUIT_SELFTEST
#include "wdf/Dc2Circuit.h"
