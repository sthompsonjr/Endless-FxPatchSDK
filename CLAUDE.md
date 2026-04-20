# Endless-FxPatchSDK — Claude Code Configuration

## Project Overview

This is a high-performance DSP library targeting the Polyend Endless pedal (ARM Cortex-M7 @ 720 MHz). The library implements analog circuit modeling via Wave Digital Filters (WDF) and DSP primitives for real-time audio effect processing.

**Hard constraints (enforced in all code):**
- No heap allocation, exceptions, or RTTI
- Single-precision float only (1.0f, not 1.0)
- C++20 standard, noexcept audio paths
- 15,000 cycles/sample budget
- Position-independent binary at 0x80000000

---

## Documentation-First Strategy

### When Starting a New Session

1. **Read the three documentation files first:**
   - `build_pipeline.txt` — Prioritized implementation candidates and status
   - `library_inventory.txt` — Complete catalog of all WDF, DSP, and effect classes
   - `INVENTORY.md` — Annotated directory tree with file roles

   These files are **regenerated from filesystem scan** after each implementation session. They are the authoritative source of truth for:
   - What exists in the codebase
   - Test status and cycle costs
   - Implementation status and blocking dependencies
   - File organization and dependencies

2. **If the documentation files are missing or out-of-date:**
   - Run the `docs_sync` prompt to regenerate from filesystem scan
   - This scans all WDF, DSP, and effect headers and produces updated versions
   - Output goes to `build/docs_sync/` and must be copied to project root

3. **Only analyze source code when:**
   - Documentation exists but you need implementation details beyond the inventory
   - The task requires reading specific circuit equations, solver algorithms, or test harnesses
   - You need to modify or debug code

### Why This Strategy

The repository contains:
- 31 WDF circuit/primitive headers
- 23 DSP primitive headers
- 10 effect implementations
- 22 test harnesses

Scanning all of this on every session wastes tokens and time. The documentation files provide:
- Complete class and method inventory with cycle costs
- Solver types and precision requirements
- Test status and dependencies
- Circuit topology summaries

---

## Key Files and Their Roles

### Documentation (Read First)
- `build_pipeline.txt` — Status tracker for next 3 tiers of effects to build
- `library_inventory.txt` — Catalog: classes, methods, dependencies, test status
- `INVENTORY.md` — Directory tree: file roles and architecture summary

### SDK (Read-Only)
- `sdk/Patch.h` — Abstract base class (all effects inherit from this)
- `sdk/PatchABI.h` — C ABI for firmware loader
- `sdk/PatchCppWrapper.cpp` — C++ ↔ C ABI bridge

### DSP Primitives (Header-Only, No State)
- `dsp/OnePoleFilter.h`, `BiquadFilter.h`, `StateVariableFilter.h` — Filters
- `dsp/Lfo.h`, `AnalogLfo.h`, `UniVibeLfo.h` — Oscillators
- `dsp/CircularBuffer.h`, `AllpassDelay.h`, `MultiTapDelay.h` — Delays
- `dsp/GrainScheduler.h`, `ReverbPrimitives.h` — Complex processors
- `dsp/ParameterSmoother.h`, `EnvelopeFollower.h` — Envelope control
- `dsp/Saturation.h`, `Interpolation.h` — Utility functions

### WDF Circuits (Analog Modeling)
- **Primitives** (`WdfDiodeFamily`, `WdfNpnBjtFamily`, etc.) — Single elements
- **Families** — Parametric versions with device presets
- **Named circuits** (`WdfBigMuffCircuit`, `WdfRatCircuit`, etc.) — Complete effects
- **Op-amps** (`WdfOpAmpLM741`, `WdfOpAmpLM308`, `WdfOpAmpJRC4558`) — Op-amp models
- **Support** (`LambertW.h`, `NewtonRaphson.h`, `WdfAdaptors.h`) — Math and infrastructure

### Effect Patches
- `effects/PatchImpl_*.cpp` — Implementations (one active at a time in `source/PatchImpl.cpp`)
- `tests/*_test.cpp` — Native build test harnesses (compile with `g++ -std=c++20`)

---

## Task Guidance

### Implementing a New Effect

1. Check `build_pipeline.txt` for prerequisites and blocking dependencies
2. Read the circuit header (e.g., `WdfBigMuffCircuit.h`) to understand topology
3. Reference similar effect (`PatchImpl_BigMuff.cpp`) for structure and knob mapping
4. Implement `PatchImpl_NewEffect.cpp` inheriting from `Patch`
5. Run test harness: `g++ -std=c++20 -I. -I./dsp -I./wdf tests/new_test.cpp -lm -o test && ./test`
6. After completing: run `docs_sync` prompt to update documentation

### Debugging a Test Failure

1. Read test file to understand what's being tested
2. Check `library_inventory.txt` for the class's solver type and cycle cost estimate
3. Read the implementation in `wdf/` or `dsp/` to verify algorithm
4. Check for boundary conditions (exp overflow, division by zero, etc.)
5. Run test harness locally and capture error output

### Cycle Budget Analysis

When optimizing code:
1. Reference the cycle cost table in Section 3 of the implementation prompt
2. Estimate the cost of your changes using the tier system
3. Sum costs: solver (most expensive) + adaptors + I/O filtering
4. If total exceeds ~12,000 cycles, flag for optimization review

---

## Implementation Workflow

### Before Writing Code

1. Read `build_pipeline.txt` — confirm prerequisites are present
2. Read relevant circuit/DSP headers — understand dependencies
3. Read a similar effect implementation — follow the pattern
4. Check test harness directory — see if tests exist for prerequisites

### After Writing Code

1. Compile native test: `g++ -std=c++20 -I. -I./dsp -I./wdf tests/your_test.cpp -lm -o test`
2. Cross-compile check: `arm-none-eabi-g++ -std=c++20 -march=armv7e-m+fp -mfpu=fpv5-sp-d16 ...` (link only)
3. Run test harness and validate output
4. Commit with message referencing the implementation prompt session link
5. Run `docs_sync` prompt to regenerate documentation

### Before Pushing to Master

1. Ensure all tests pass on native build
2. Ensure cross-compile succeeds (link check)
3. Run `docs_sync` to update documentation
4. Create PR with clear description of what was added/fixed
5. Reference the implementation prompt session link in PR body

---

## Known Issues & Regressions

As of 2026-04-20:

- **pnp_bjt_test.cpp** — Suspected Jacobian cross-partial error in WdfPnpBjt solver
- **wah_test.cpp** — Suspected pot resistance update not propagating through adaptor tree
- **power_puff_test.cpp** — Compile error: missing Patch.h header path

These are flagged for repair. See `build_pipeline.txt` Tier 3 for WDF Test Audit status.

---

## Cycle Budget Reference

| Solver Type | Cost Range | Example |
|---|---|---|
| Analytical (Lambert W, tanh) | 200–600 | WdfDiodeFamily |
| 1D Newton-Raphson | 400–800 | WdfAntiparallelDiodes |
| 2D Newton-Raphson | 800–2000 | WdfNpnBjtFamily |
| WDF adaptor (per node) | 50–100 | WdfSeriesAdaptor2 |
| OnePoleFilter | 30–50 | OnePoleFilter |
| Hermite interpolation | 80–120 | CircularBuffer |
| ParameterSmoother | 15–25 | ParameterSmoother |

---

## File Organization Rules

### What Goes Where

- **dsp/**: Header-only DSP primitives. No state. Reusable. NoExcept.
- **wdf/**: WDF elements and circuits. Can have state (port variables). Analytical or iterative solvers.
- **effects/**: `PatchImpl_*.cpp` files only. One active at a time via `source/PatchImpl.cpp`.
- **source/**: Active effect (`PatchImpl.cpp`, `dsp/DelayLine.h`, `dsp/Looper.h`).
- **tests/**: Native-build test harnesses (`g++ -std=c++20`). ARM execution not available.
- **sdk/**: Patch interface and ABI. Read-only. Do not modify.

### Naming Conventions

- Classes: PascalCase (`WdfBigMuffCircuit`, `BiquadFilter`, `ParameterSmoother`)
- Methods: camelCase (`reflect()`, `setFrequency()`, `process()`)
- Namespaces: snake_case (`diode_params`, `bjt_params`, `sat`, `interp`, `fdn`)
- Files: PascalCase.h or PascalCase.cpp (`WdfDiodeFamily.h`, `PatchImpl_BigMuff.cpp`)
- Test files: `snake_case_test.cpp` (`big_muff_test.cpp`, `dsp_test.cpp`)

---

## Quick Reference: Session Startup

```
1. Read build_pipeline.txt (what to build next)
2. Read library_inventory.txt (what exists + dependencies + test status)
3. Read INVENTORY.md (directory structure + file roles)
4. If docs are missing/stale → run docs_sync prompt
5. For implementation details → read specific source files
6. For code changes → follow hard constraints (no heap, no exceptions, 1.0f suffix)
```

---

## Contact & Documentation

- **Implementation prompts**: Generated per-effect with complete WDF circuit analysis, cycle budget, and test harness requirements
- **This configuration**: `CLAUDE.md` (project root)
- **Settings**: `.claude/settings.json` (context window configuration)
- **Documentation generation**: `docs_sync` reusable prompt (reads filesystem, outputs to `build/docs_sync/`)
