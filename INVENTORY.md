# Endless-FxPatchSDK — Consolidated Branch Inventory

> **Branch:** `claude/consolidate-sdk-structure-1uNAY`
> **Date:** 2026-03-13
> **Commits:**
> ```
> fc7be0c feat: collect all DSP/WDF code from feature branches into canonical structure
> 3cb0687 chore: consolidate SDK into canonical directory structure
> 02f22ca Initial Beta version of the SDK
> ```

---

## Summary

| Category   | Files | Lines  | Classes/Structs | Namespaces |
|------------|-------|--------|-----------------|------------|
| DSP        | 15    | ~1,060 | 14              | 4          |
| WDF        | 22    | ~5,300 | 48+             | 2          |
| Effects    | 7     | ~1,100 | 7               | —          |
| SDK        | 5     | ~410   | 2               | —          |
| Source     | 3     | ~340   | 2               | 1          |
| Tests      | 9     | ~3,870 | —               | —          |
| Config     | 5     | ~200   | —               | —          |
| **Total**  | **66**| **~12,280** | **73+**    | **7**      |

---

## Directory Tree

```
Endless-FxPatchSDK/
├── .clang-format
├── .gitignore
├── .vscode/
│   ├── settings.json
│   └── tasks.json
├── Makefile
├── README.md
├── linker.ld
│
├── sdk/                              ← SDK infrastructure (read-only)
│   ├── Patch.h                       ← Abstract base class for effects
│   ├── PatchABI.h                    ← C ABI for loader/binary interface
│   ├── PatchCppWrapper.h             ← C++ wrapper declarations
│   ├── PatchCppWrapper.cpp           ← C++ wrapper implementation
│   └── patch_main.c                  ← C entry point
│
├── dsp/                              ← Foundational DSP primitives
│   ├── dsp.h                         ← Umbrella header
│   ├── AllpassDelay.h
│   ├── AnalogLfo.h
│   ├── BBDLine.h
│   ├── CircularBuffer.h
│   ├── EnvelopeFollower.h
│   ├── GrainEnvelope.h
│   ├── GrainScheduler.h
│   ├── Interpolation.h
│   ├── Lfo.h
│   ├── OnePoleFilter.h
│   ├── ParameterSmoother.h
│   ├── ReverbPrimitives.h
│   ├── Saturation.h
│   └── UniVibeLfo.h
│
├── wdf/                              ← WDF analog circuit modeling
│   ├── wdf.h                         ← Umbrella header (dependency order)
│   │
│   │  ── Foundation ──
│   ├── WdfPort.h                     ← Wave variable port
│   ├── LambertW.h                    ← Lambert W function
│   ├── NewtonRaphson.h               ← Newton-Raphson / Halley solvers
│   │
│   │  ── Elements ──
│   ├── WdfOnePort.h                  ← R, C, L, voltage/current sources
│   ├── WdfAdaptors.h                 ← Series/parallel 2-port and 3-port
│   ├── WdfNonlinear.h                ← Diodes + NPN BJT
│   ├── WdfPnpBjt.h                   ← PNP BJT transistor
│   ├── WdfOta.h                      ← Operational transconductance amp
│   ├── WdfPhotoresistor.h            ← Light-dependent resistor
│   │
│   │  ── Op-Amps ──
│   ├── WdfOpAmpLM741.h               ← LM741 (general purpose)
│   ├── WdfOpAmpLM308.h               ← LM308 (RAT)
│   ├── WdfOpAmpJRC4558.h             ← JRC4558 (Tubescreamer)
│   ├── WdfInvertingStage.h            ← Inverting amp w/ feedback diodes
│   │
│   │  ── Circuits ──
│   ├── WdfCircuits.h                  ← RCLowpass, DiodeClipper, ToneStack, BJTGainStage
│   ├── DOD250Circuit.h                ← DOD 250 Overdrive
│   ├── WdfRatCircuit.h                ← ProCo RAT (5 variants)
│   ├── WdfTubescreamerCircuit.h       ← TS808, TS9, Klon
│   ├── WdfPnpCircuits.h               ← Rangemaster, FuzzFace, ToneBender, GeBoost
│   ├── WdfWahCircuit.h                ← CryBaby, AutoWah
│   ├── WdfCompressorCircuits.h        ← DynacompCircuit
│   └── WdfOpticalCircuits.h           ← PC-2A, OpticalLeveler, HybridOptOta
│
├── effects/                           ← One PatchImpl per effect
│   ├── PatchImpl_Bitcrush.cpp
│   ├── PatchImpl_DOD250.cpp
│   ├── PatchImpl_OpticalComp.cpp
│   ├── PatchImpl_Rangemaster.cpp
│   ├── PatchImpl_Rat.cpp
│   ├── PatchImpl_Tubescreamer.cpp
│   └── PatchImpl_Wah.cpp
│
├── source/                            ← Active effect (SDK-required)
│   ├── PatchImpl.cpp                  ← Currently: DOD250
│   └── dsp/
│       ├── DelayLine.h                ← Circular buffer delay line
│       └── Looper.h                   ← Stereo record/playback looper
│
└── tests/                             ← Native-build test harnesses
    ├── run_all_tests.sh               ← Build + run all, report pass/fail
    ├── dsp_test.cpp
    ├── wdf_test.cpp
    ├── lm741_test.cpp
    ├── lm308_rat_test.cpp
    ├── jrc4558_test.cpp
    ├── pnp_bjt_test.cpp
    ├── wah_test.cpp
    └── photoresistor_test.cpp
```

---

## Detailed File Inventory

### `sdk/` — SDK Infrastructure

#### `sdk/Patch.h` (~120 lines)
Abstract base class for all effects.

- **Class:** `Patch`
- **Enums:** `ParamId`, `ActionId`, `ParamSource`, `Color`
- **Struct:** `ParameterMetadata { minValue, maxValue, defaultValue }`
- **Virtual methods:**
  - `init()` — one-time initialization
  - `setWorkingBuffer(uint8_t*, size_t)` — attach 2.4 MB external memory
  - `processAudio(float* left, float* right, int samples)` — stereo processing (called every block)
  - `getParameterMetadata(int idx, ParameterMetadata&)` — parameter info
  - `setParamValue(int idx, float value)` — set knob/expression values (0.0–1.0)
  - `isParamEnabled(int idx, int sourceId)` — query parameter availability
  - `handleAction(int actionId)` — footswitch press/hold events
  - `getStateLedColor()` — state indicator LED color
- **Constants:** `kWorkingBufferSize = 2400000`, `kSampleRate = 48000`
- **Static:** `getInstance()` — singleton factory

#### `sdk/PatchABI.h` (~80 lines)
C-compatible ABI for loader/patch binary interface.

- **Defines:** `PATCH_MAGIC = 0x48435450` ("PTCH"), `PATCH_ABI_VERSION = 0x000B`
- **Struct:** `PatchHeader` — binary header at start of `.bin` image
  - `magic`, `abi_version`, `flags`, `init` (function pointer)
  - Agent hooks: `agent_update_buffers`, `agent_set_buffer`, `agent_get_buffer_size`, etc.
  - Memory layout: `image_size`, `bss_begin`, `bss_size`
- **Function pointer types:** `PatchInitFn`, `PatchAgentUpdateBuffersFn`, etc.
- **Struct:** `PatchEnv { abi_version, user_ctx }`

#### `sdk/PatchCppWrapper.h` + `sdk/PatchCppWrapper.cpp` (~130 lines combined)
C++ wrapper bridging `Patch` virtual methods to C function pointers.

#### `sdk/patch_main.c` (~80 lines)
C entry point. Validates ABI version, calls `__libc_init_array()`, invokes `patch_agent_init()`.

---

### `dsp/` — Foundational DSP Primitives

#### `dsp/dsp.h` (umbrella)
Includes all DSP headers.

#### `dsp/Interpolation.h` (~24 lines)
- **Namespace:** `interp`
- **Functions:**
  - `linear(a, b, t)` — linear interpolation
  - `hermite(xm1, x0, x1, x2, t)` — 4-point cubic Hermite
  - `cosine(a, b, t)` — cosine interpolation

#### `dsp/Saturation.h` (~40 lines)
- **Namespace:** `sat`
- **Functions:**
  - `softClip(x)` — tanh-based soft clip
  - `softClipCubic(x)` — polynomial approximation
  - `hardClip(x, thresh)` — hard clip to threshold
  - `fold(x)` — wavefolding
  - `drive(x, amount)` — parameterized drive

#### `dsp/OnePoleFilter.h` (~65 lines)
- **Class:** `OnePoleFilter`
- **Enum:** `Type { Lowpass, Highpass, DCBlock }`
- **Methods:** `init(sampleRate)`, `setFrequency(hz)`, `setType(type)`, `process(x)`, `reset()`

#### `dsp/ParameterSmoother.h` (~35 lines)
- **Class:** `ParameterSmoother`
- **Methods:** `init(sampleRate, smoothMs)`, `setTarget(value)`, `process()` → smoothed value, `getCurrentValue()`, `snapTo(value)`

#### `dsp/CircularBuffer.h` (~60 lines)
- **Class:** `CircularBuffer<T, Size>`
- **Methods:** `reset()`, `write(x)`, `read(delaySamples)`, `readLinear(delay)`, `readHermite(delay)`, `size()`

#### `dsp/AllpassDelay.h` (~30 lines)
- **Class:** `AllpassDelay<Size>`
- **Methods:** `init(coeff)`, `process(x)`, `reset()`
- **Purpose:** Schroeder allpass for reverb diffusion

#### `dsp/Lfo.h` (~86 lines)
- **Class:** `Lfo`
- **Enum:** `Shape { Sine, Triangle, Saw, ReverseSaw, Square, SampleHold }`
- **Methods:** `init(sampleRate)`, `setFrequency(hz)`, `setShape(s)`, `setPhaseOffset(p)`, `reset()`, `process()` → bipolar, `processUnipolar()` → 0–1

#### `dsp/AnalogLfo.h` (~168 lines)
- **Class:** `AnalogLfo`
- **Enum:** `Shape { Sine, Triangle, Saw, ReverseSaw, Square }`
- **Methods:** `init(sampleRate)`, `setFrequency(hz)`, `setShape(s)`, `setDriftAmount(d)`, `setDriftRate(r)`, `setJitterAmount(j)`, `reset()`, `process()`, `processUnipolar()`, `getPhase()`, `getCurrentFrequency()`
- **Purpose:** CEM3340-style LFO with frequency drift and per-sample jitter

#### `dsp/UniVibeLfo.h` (~168 lines)
- **Class:** `UniVibeLfo`
- **Struct:** `LdrState { filterState, sensitivityOffset, output }`
- **Methods:** `init(sampleRate)`, `setRate(hz)`, `setIntensity(i)`, `reset()`, `process()`, `getLdrOutput(stageIdx)`, `getBulbBrightness()`
- **Purpose:** Uni-Vibe incandescent bulb + LDR photoresistor modeling (4 LDR stages)

#### `dsp/EnvelopeFollower.h` (~66 lines)
- **Class:** `EnvelopeFollower`
- **Enum:** `Mode { Peak, RMS }`
- **Methods:** `init(sampleRate)`, `setAttackMs(ms)`, `setReleaseMs(ms)`, `setMode(m)`, `process(x)`, `getLevel()`, `reset()`

#### `dsp/ReverbPrimitives.h` (~59 lines)
- **Class:** `CombFilter<Size>`
- **Namespace:** `fdn`
- **Function:** `hadamard4(a, b, c, d)` — 4×4 Hadamard matrix mix
- **Methods:** `init(feedback, damp)`, `process(x)`, `reset()`
- **Purpose:** Building blocks for FDN reverb

#### `dsp/GrainEnvelope.h` (~42 lines)
- **Namespace:** `grain`
- **Enum:** `EnvelopeShape { Hann, Tukey, Trapezoid }`
- **Functions:** `hann(phase)`, `tukey(phase, ratio)`, `trapezoid(phase, attack, release)`, `gaussian(phase, sigma)`

#### `dsp/GrainScheduler.h` (~149 lines)
- **Class:** `GrainScheduler<MaxGrains>`
- **Struct:** `GrainParams { position, pitch, durationMs, pan, amplitude, shape }`
- **Methods:** `init(sampleRate, buffer, bufferSize)`, `trigger(params)`, `process()` → stereo pair, `activeGrainCount()`, `reset()`
- **Purpose:** Polyphonic grain synthesis scheduler

#### `dsp/BBDLine.h` (~160 lines)
- **Class:** `BBDLine<Size>`
- **Methods:** `init(sampleRate)`, `setDelaySamples(n)`, `setDelayMs(ms)`, `setCompanderAmount(a)`, `setClockNoiseLevel(n)`, `process(x)`, `reset()`
- **Purpose:** Bucket Brigade Device delay line emulation with compander distortion, clock noise, reconstruction lowpass

---

### `wdf/` — Wave Digital Filter Library

#### `wdf/wdf.h` (umbrella)
Merged from all 5 feature branches. Includes all WDF headers in strict dependency order.

#### `wdf/WdfPort.h` (~24 lines)
- **Struct:** `WdfPort { Rp, a, b }`
- **Methods:** `voltage()` → `(a + b) / 2`, `current()` → `(a - b) / (2 * Rp)`, `reset()`
- **Purpose:** Atomic wave variable port — all WDF elements have at least one port

#### `wdf/LambertW.h` (~88 lines)
- **Namespace:** `math`
- **Functions:**
  - `lambertW0(x)` — principal branch W₀(x) via Halley iteration
  - `lambertWn1(x)` — W₋₁(x) branch for negative arguments
- **Purpose:** Analytical solution for diode equations: `I = Is * (e^(V/Vt) - 1)`

#### `wdf/NewtonRaphson.h` (~95 lines)
- **Namespace:** `math`
- **Struct:** `NRResult { value, converged, iterations }`
- **Functions:**
  - `newtonRaphson(f, df, x0, tol, maxIter)` — 1D Newton-Raphson
  - `halley(f, df, ddf, x0, tol, maxIter)` — 3rd-order Halley's method
- **Purpose:** Nonlinear equation solving for transistors, diodes

#### `wdf/WdfOnePort.h` (~161 lines)
- **Classes:**
  - `WdfResistor` — `b = 0` (pure absorption, port resistance = R)
  - `WdfCapacitor` — `b = a[n-1]` (unit delay of incident wave)
  - `WdfInductor` — `b = -a[n-1]` (inverted delay)
  - `WdfIdealVoltageSource` — `b = 2*Vs - a` (wave-domain ideal source)
  - `WdfResistiveVoltageSource` — voltage source with series resistance
  - `WdfResistiveCurrentSource` — current source with parallel resistance

#### `wdf/WdfAdaptors.h` (~188 lines)
- **Classes (all template):**
  - `WdfSeriesAdaptor2<ChildA, ChildB>` — 2-port series junction
  - `WdfParallelAdaptor2<ChildA, ChildB>` — 2-port parallel junction
  - `WdfSeriesAdaptor3<P1, P2>` — 3-port series (one port is the root)
  - `WdfParallelAdaptor3<P1, P2>` — 3-port parallel
- **Key concept:** Reflection coefficient `α = (Rp2 - Rp1) / (Rp1 + Rp2)` scatters waves at the junction

#### `wdf/WdfNonlinear.h` (~314 lines)
- **Classes:**
  - `WdfIdealDiode` — single diode, Lambert W closed-form solution
  - `WdfAntiparallelDiodes` — symmetric soft clipping (two antiparallel diodes)
  - `WdfNpnBjt` — Ebers-Moll 3-port NPN BJT, 2D Newton-Raphson solver
  - `WdfDiodeToGround` — asymmetric single-diode clipping (RAT circuit)
- **Factory functions:** `make1N914()`, `make1N4148()`, `makeGermanium()`, `makeLED()`
- **Note:** `WdfDiodeToGround` only existed on `add-dsp-primitives` branch — merged from there

#### `wdf/WdfOpAmpLM741.h` (~109 lines)
- **Class:** `WdfOpAmpLM741`
- **Specs:** A₀=100 (40dB), fₚ=100Hz, SR=0.5V/µs, V_rail=±13V, V_os=1mV
- **Methods:** `init(sampleRate)`, `setRailVoltage(v)`, `process(vPlus, vMinus)`, `reset()`
- **Purpose:** General-purpose op-amp for DOD250 and inverting stages

#### `wdf/WdfOpAmpLM308.h` (~172 lines)
- **Class:** `WdfOpAmpLM308`
- **Specs:** A₀=100, fₚ=100Hz, SR=0.3V/µs (characteristically slow), V_rail=±7.5V, V_os=2mV
- **Methods:** `init(sampleRate)`, `setRailVoltage(v)`, `setCompCapPF(pF)`, `setAge(0–1)`, `setCharacter(0–1)`, `process(vPlus, vMinus)`, `reset()`
- **Purpose:** Heart of the ProCo RAT distortion — the slow slew rate creates the RAT's characteristic grit

#### `wdf/WdfOpAmpJRC4558.h` (~162 lines)
- **Class:** `WdfOpAmpJRC4558`
- **Specs:** A₀=50, fₚ=300Hz, SR≈0.32V/µs, V_rail=±13V, V_os=2mV
- **Methods:** `init(sampleRate)`, `setRailVoltage(v)`, `setAge(0–1)`, `setCharacter(0–1)`, `setSlew(0–1)`, `process(vPlus, vMinus)`, `reset()`
- **Purpose:** Ibanez TS808/TS9 Tubescreamer op-amp

#### `wdf/WdfPnpBjt.h` (~233 lines)
- **Class:** `WdfPnpBjt`
- **Struct:** `Params { Is, Vt, hFE, Vaf }`
- **Methods:** `init(sampleRate, params)`, `reflect(vB, vC, vE)`, `reset()`, `lastIterationCount()`, `lastConverged()`
- **Factory presets:**
  - `makeOC44()` — Germanium, Rangemaster (Is=200µA, Vt=28mV, hFE=70, Vaf=40V)
  - `makeOC75()` — Germanium, Tone Bender (Is=100µA, Vt=28mV, hFE=100, Vaf=40V)
  - `makeAC128()` — Germanium, Fuzz Face (Is=150µA, Vt=28mV, hFE=120, Vaf=50V)
  - `make2N3906()` — Silicon reference (Is=6.7pA, Vt=26mV, hFE=200, Vaf=100V)
- **Solver:** 2D Newton-Raphson for Ebers-Moll equations with Early effect

#### `wdf/WdfOta.h` (~60 lines)
- **Class:** `WdfOta`
- **Methods:** `init(sampleRate)`, `setIabc(current)`, `process(vIn)`, `getGain()`, `getIabc()`, `reset()`
- **Namespace:** `ca3080_params` — CA3080 OTA constants
- **Purpose:** Operational Transconductance Amplifier VCA for compressor circuits

#### `wdf/WdfPhotoresistor.h` (~146 lines)
- **Class:** `WdfPhotoresistor`
- **Struct:** `Params { R_dark, R_bright, tau_fast, tau_slow, tau_release, w_fast_attack, sampleRate }`
- **Namespace:** `ldr_params` — NSL-32SR3 and Silonex SL5542 presets
- **Methods:** `init(params)`, `setLightLevel(0–1)`, `reflect()`, `reset()`, `getCurrentResistance()`, `getCurrentLightLevel()`, `isAttacking()`, `getInertia()`
- **Purpose:** Two-time-constant LDR model for optical compressors (LA-2A style)

#### `wdf/WdfInvertingStage.h` (~125 lines)
- **Class:** `WdfInvertingStage`
- **Methods:** `init(sampleRate, Rin, Rf)`, `setFeedbackResistance(Rf)`, `process(vIn)`, `reset()`, `opamp()`
- **Circuit:** LM741 inverting amplifier with variable gain, feedback capacitor, clipping diodes

#### `wdf/WdfCircuits.h` (~371 lines)
- **Classes:**
  - `RCLowpassCircuit` — 1st-order passive RC lowpass, bilinear transform IIR
  - `DiodeClipperCircuit` — Input R + antiparallel diodes to ground (TS-style clipping)
  - `ToneStackCircuit` — Passive 2-section Baxandall RC tone control
  - `BJTGainStageCircuit` — NPN common-emitter with collector/emitter resistor loads

#### `wdf/DOD250Circuit.h` (~146 lines)
- **Class:** `DOD250Circuit`
- **Circuit:** LM741 inverting amp, Rg=4.7kΩ, Rf=4.7k–500kΩ (gain pot), feedback C for HF rolloff, 1N914 antiparallel diodes
- **Controls:** `setGain(0–1)`, `setLevel(0–1)`, `setTone(0–1)` (2kHz–20kHz post-clipper lowpass)
- **Methods:** `init(sampleRate)`, `process(x)`, `reset()`, `stage()` → access inverting stage

#### `wdf/WdfRatCircuit.h` (~353 lines)
- **Class:** `WdfRatCircuit`
- **Enum:** `RatVariant { Original, TurboRat, YouDirtyRat, WhiteRat, GermaniumMod, kCount }`
- **Circuit:** LM308 inverting, R_dist=100Ω–1MΩ, C_feedback=47pF, diode variants per model
  - **Original:** 1N914 antiparallel silicon
  - **Turbo RAT:** LED clipping (higher headroom)
  - **You Dirty RAT:** Asymmetric silicon (1N914 + germanium)
  - **White RAT:** No diodes (op-amp hard rail clipping)
  - **Germanium Mod:** Antiparallel germanium (softer, lower threshold)
- **Controls:** `setDistortion(0–1)`, `setFilter(0–1)`, `setVolume(0–1)`, `setVariant(v)`, `setSlewCharacter(0–1)`, `setAge(0–1)`

#### `wdf/WdfTubescreamerCircuit.h` (~405 lines)
- **Classes:**
  - `TS808Circuit` — Ibanez TS808: JRC4558 inverting amp, antiparallel 1N914 in feedback, input HPF at 720Hz, post-clipping tone filter
  - `TS9Circuit` — TS808 variant with transistor output buffer and HF shelf
  - `KlonClipStage` — Klon Centaur: mismatched germanium-parameter diodes for asymmetric clipping
- **Controls (TS808):** `setDrive(0–1)`, `setTone(0–1)`, `setLevel(0–1)`, `setAge(0–1)`, `setAsymmetry(0–1)`

#### `wdf/WdfPnpCircuits.h` (~548 lines)
- **Classes:**
  - `RangemasterCircuit` — Dallas Rangemaster: OC44 PNP germanium, input coupling 5nF, common-emitter treble boost
  - `FuzzFaceCircuit` — Dallas Arbiter Fuzz Face: two-stage AC128 PNP, inter-stage coupling via C2, silicon/germanium switchable
  - `ToneBenderMk1Circuit` — Sola Sound Tone Bender: OC75 PNP with variable attack emitter R
  - `GeBoostCircuit` — Generic parameterized PNP boost (any transistor preset)
- **Controls (Rangemaster):** `setBoost(0–1)`, `setOutputGain(0–1)`, `setTransistor(params)`
- **Controls (FuzzFace):** `setFuzz(0–1)`, `setVolume(0–1)`, `setBias(0–1)`

#### `wdf/WdfWahCircuit.h` (~367 lines)
- **Classes:**
  - `CryBabyCircuit` — Thomas Organ/Dunlop: LC resonant tank (L1 inductor, C1+C2 caps) with NPN BJT buffer, R_wah sweep controls resonant peak
  - `AutoWahCircuit` — Auto-controlled wah with 4 modes
- **WDF tree:** `Series2(Src, Series2(C1, Parallel2(Series2(L1, RWah), C2)))`
- **Enum:** `WahMode { Expression, Envelope, Lfo, EnvLfo }`
- **Controls:** `setSweep(0–1)`, `setResonance(0–1)`, `setSensitivity(0–1)`, `setMode(m)`, `setLfoRate(hz)`

#### `wdf/WdfCompressorCircuits.h` (~105 lines)
- **Class:** `DynacompCircuit`
- **Circuit:** OTA (CA3080) VCA with fixed-time-constant envelope, MXR Dynacomp-style
- **Controls:** `setSensitivity(0–1)`, `setOutputLevel(0–1)`

#### `wdf/WdfOpticalCircuits.h` (~359 lines)
- **Classes:**
  - `PC2ACircuit` — Teletronix LA-2A / Effectrode PC-2A: LDR shunt divider VCA, envelope detection, optional HF sidechain emphasis
  - `OpticalLevelerCircuit` — Single-knob optical leveler (simplified LA-2A wrapper)
  - `HybridOptOtaCircuit` — OTA VCA controlled by LDR resistance mapping, log-linear transfer

---

### `effects/` — Effect Implementations

Each file implements `PatchImpl : public Patch` for one effect.

#### `effects/PatchImpl_Bitcrush.cpp` (~68 lines)
- **Effect:** Bitcrusher (basic DSP, no WDF)
- **Controls:** Knob 0–2 + expression pedal
- **LED:** Blue (active), Dim Blue (bypass)

#### `effects/PatchImpl_DOD250.cpp` (~122 lines)
- **Effect:** DOD 250 Overdrive
- **Circuit:** `DOD250Circuit`
- **Controls:** Gain, Level, Tone
- **Includes:** `wdf/DOD250Circuit.h`, `dsp/ParameterSmoother.h`, `dsp/Saturation.h`

#### `effects/PatchImpl_Rat.cpp` (~126 lines)
- **Effect:** ProCo RAT Distortion
- **Circuit:** `WdfRatCircuit`
- **Controls:** Distortion, Filter (reversed sweep), Volume
- **Feature:** 5 RAT variant switching with crossfade
- **Includes:** `wdf/WdfRatCircuit.h`, `dsp/Saturation.h`

#### `effects/PatchImpl_Tubescreamer.cpp` (~226 lines)
- **Effect:** Tubescreamer Family
- **Circuits:** `TS808Circuit`, `TS9Circuit`, `KlonClipStage` (3-position selector)
- **Controls:** Drive, Tone, Circuit selector, Level (expression)
- **Feature:** Footswitch press = age toggle, hold = asymmetry/clarity cycle
- **LEDs:** Circuit color (TS808=DimGreen, TS9=LightGreen, Klon=DimYellow)
- **Includes:** `wdf/WdfTubescreamerCircuit.h`, `dsp/ParameterSmoother.h`, `dsp/Saturation.h`

#### `effects/PatchImpl_Rangemaster.cpp` (~151 lines)
- **Effect:** Dallas Rangemaster Treble Booster
- **Circuit:** `RangemasterCircuit`
- **Controls:** Treble/Boost amount, output gain
- **Includes:** `wdf/WdfPnpCircuits.h`, `dsp/ParameterSmoother.h`

#### `effects/PatchImpl_Wah.cpp` (~190 lines)
- **Effect:** CryBaby Wah Pedal
- **Circuit:** `AutoWahCircuit`
- **Modes:** Expression, Envelope, LFO, EnvLfo
- **Controls:** Sweep, Resonance, Sensitivity, Output Level
- **Feature:** Mode selection via footswitch
- **Includes:** `wdf/WdfWahCircuit.h`, `dsp/ParameterSmoother.h`

#### `effects/PatchImpl_OpticalComp.cpp` (~226 lines)
- **Effect:** Optical Compressor (LA-2A style)
- **Circuit:** `PC2ACircuit`
- **Controls:** Peak Reduction, Gain, HF Emphasis
- **Feature:** Stereo linked compression via `processStereo()`
- **Includes:** `wdf/WdfOpticalCircuits.h`

---

### `source/` — Active Effect + App-Level DSP

#### `source/PatchImpl.cpp` (~122 lines)
Copy of `effects/PatchImpl_DOD250.cpp`. Changed via `make effect E=<Name>`.

#### `source/dsp/DelayLine.h` (~89 lines)
- **Namespace:** `endless::dsp`
- **Class:** `DelayLine`
- **Methods:** `init(std::span<float>)`, `write(x)`, `read(delaySamples)`, `readInterpolated(delay)`, `process(x, delay)`, `capacity()`
- **Purpose:** External-memory circular buffer delay line

#### `source/dsp/Looper.h` (~115 lines)
- **Namespace:** `endless::dsp`
- **Class:** `Looper`
- **Enum:** `State { kIdle, kRecording, kPlaying }`
- **Methods:** `init(buffer, size)`, `toggle()`, `process(inL, inR, outL, outR, wet)`, `getState()`, `getLoopLength()`, `maxLength()`
- **Purpose:** Stereo record/playback looper (splits buffer into L/R halves)

---

### `tests/` — Test Harnesses

| Test File                    | Lines | Subject                                        | Status |
|------------------------------|-------|------------------------------------------------|--------|
| `dsp_test.cpp`               | ~753  | All DSP primitives                             | PASS   |
| `wdf_test.cpp`               | ~426  | WDF port, adaptors, nonlinear basics           | PASS   |
| `lm741_test.cpp`             | ~504  | LM741 op-amp model verification                | PASS*  |
| `lm308_rat_test.cpp`         | ~562  | LM308 + RAT circuit (all 5 variants)           | PASS   |
| `jrc4558_test.cpp`           | ~584  | JRC4558 op-amp + TS808/TS9/Klon                | PASS   |
| `pnp_bjt_test.cpp`           | ~483  | PNP BJT (OC44, OC75, AC128, 2N3906)           | FAIL** |
| `wah_test.cpp`               | ~436  | CryBaby wah + AutoWah modes                   | FAIL** |
| `photoresistor_test.cpp`     | ~594  | LDR model + PC-2A/OpticalLeveler/HybridOptOta | PASS   |

\* lm741 test not in run_all_tests.sh rotation but compiles and runs
\** Pre-existing runtime failures (10/36 pnp subtests fail on NR convergence/gain range; 6/15 wah subtests fail on resonance/freq range)

#### `tests/run_all_tests.sh`
Builds and runs all test harnesses. Reports pass/fail/skip summary. Exit code 0 if all pass, 1 if any fail.

---

### Configuration Files

| File              | Purpose                                        |
|-------------------|------------------------------------------------|
| `.clang-format`   | Code formatting rules                          |
| `.gitignore`      | Ignores: `build/`, `_conflicts/`, `*.bin/elf/o/su`, `Makefile.orig` |
| `.vscode/settings.json` | Editor settings                           |
| `.vscode/tasks.json`    | Build tasks                               |
| `README.md`       | SDK documentation                              |

---

### `Makefile` — Build System

**Targets:**

| Target          | Description                                         |
|-----------------|-----------------------------------------------------|
| `all`           | ARM Cortex-M7 cross-compile (default)               |
| `native`        | Build active effect for host testing                 |
| `tests`         | Build and run all test harnesses                     |
| `effect E=Name` | Copy effect to `source/PatchImpl.cpp` and build      |
| `list-effects`  | List available effects                               |
| `active-effect` | Show current active effect                           |
| `headers-check` | Verify all headers compile clean                     |
| `clean`         | Remove build artifacts                               |

**Compiler flags:**
- ARM: `-mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -O3 -std=c++20 -fno-exceptions -fno-rtti -Wdouble-promotion -Werror`
- Native: `-std=c++20 -O2 -Wall -Wextra -Wdouble-promotion`

**Include paths:** `-I. -I./sdk -I./dsp -I./wdf`

---

### `linker.ld`
ARM linker script. Places `.patch_header` section at `PATCH_LOAD_ADDR`, followed by `.text`, `.rodata`, `.data`, `.bss`. Exports symbols for `PatchHeader` fields.

---

## Analog Circuits Modeled

### Distortion / Overdrive
| Circuit            | Real Pedal          | Op-Amp   | Clipping             |
|--------------------|---------------------|----------|----------------------|
| DOD250Circuit      | DOD 250 Overdrive   | LM741    | 1N914 antiparallel   |
| WdfRatCircuit      | ProCo RAT           | LM308    | 5 variants (see above)|
| TS808Circuit       | Ibanez TS808        | JRC4558  | 1N914 in feedback    |
| TS9Circuit         | Ibanez TS9          | JRC4558  | 1N914 + output buffer|
| KlonClipStage      | Klon Centaur         | —        | Mismatched germanium |

### Fuzz / Boost
| Circuit                | Real Pedal           | Transistor     |
|------------------------|----------------------|----------------|
| RangemasterCircuit     | Dallas Rangemaster   | OC44 PNP (Ge)  |
| FuzzFaceCircuit        | Dallas Arbiter       | AC128 PNP (Ge) |
| ToneBenderMk1Circuit   | Sola Sound Mk1       | OC75 PNP (Ge)  |
| GeBoostCircuit         | Generic boost        | Any PNP preset |

### Filter / Modulation
| Circuit           | Real Pedal     | Topology                  |
|-------------------|----------------|---------------------------|
| CryBabyCircuit    | Dunlop Cry Baby| LC tank + NPN buffer      |
| AutoWahCircuit    | Auto-wah       | CryBaby + envelope/LFO    |

### Dynamics
| Circuit                | Real Pedal         | Gain Element        |
|------------------------|--------------------|---------------------|
| DynacompCircuit        | MXR Dynacomp       | CA3080 OTA          |
| PC2ACircuit            | LA-2A / PC-2A      | LDR shunt divider   |
| OpticalLevelerCircuit  | Optical leveler     | LDR (simplified)    |
| HybridOptOtaCircuit    | Hybrid compressor   | OTA + LDR mapping   |

### Building Blocks
| Circuit              | Purpose                              |
|----------------------|--------------------------------------|
| RCLowpassCircuit     | Passive RC lowpass                   |
| DiodeClipperCircuit  | Input R + antiparallel diodes        |
| ToneStackCircuit     | Passive Baxandall tone control       |
| BJTGainStageCircuit  | NPN common-emitter gain stage        |
| WdfInvertingStage    | Op-amp inverting amp w/ feedback     |

---

## Source Provenance

| File(s) | Source Branch |
|---------|--------------|
| 15 DSP headers, WDF core (10), DOD250+RAT circuits, LM308, 3 tests, DOD250 effect | `origin/claude/add-dsp-primitives-FayiX` |
| WdfOpAmpJRC4558, WdfTubescreamerCircuit, jrc4558_test, PatchImpl_Tubescreamer | `origin/claude/add-jrc4558-tubescreamer-hcgUP` |
| WdfOta, WdfPhotoresistor, WdfCompressor/OpticalCircuits, photoresistor_test, PatchImpl_OpticalComp | `origin/claude/add-photoresistor-optical-compressor-5ltzK` |
| WdfPnpBjt, WdfPnpCircuits, pnp_bjt_test, PatchImpl_Rangemaster | `origin/claude/add-pnp-bjt-wdf-Oislj` |
| WdfWahCircuit, wah_test, PatchImpl_Wah | `origin/claude/wah-pedal-circuit-unSpR` |
| PatchImpl_Bitcrush (original SDK example) | `master` (initial commit) |

**Conflict resolution:** `WdfNonlinear.h` taken from `add-dsp-primitives` — only version with all 4 classes including `WdfDiodeToGround`.
