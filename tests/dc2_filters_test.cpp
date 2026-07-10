// Boss DC-2 Dimension C — Session 1 filter/EQ self-test harness.
//
// Thin translation unit around the self-test main embedded in
// wdf/Dc2Filters.h (per session spec: test lives under
// #ifdef DC2_FILTERS_SELFTEST in the header).
//
// Build & run (single command line, wrapped here for readability):
//   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra
//       -DENDLESS_DESKTOP_BUILD -I. -I./sdk -I./wdf -I./dsp
//       tests/dc2_filters_test.cpp -lm -o dc2_test && ./dc2_test
//
// Expected: every block within 0.5 dB of the full analytic |H(s)| at
// 100 Hz / 1 kHz / 8 kHz, plus a cross-coupled stereo NaN/Inf soak.

#define DC2_FILTERS_SELFTEST
#include "wdf/Dc2Filters.h"
