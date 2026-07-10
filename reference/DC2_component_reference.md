# Boss DC-2 Dimension C — Component Reference (Session 1)

> **Value authority for all DC-2 code.** Once created, this document supersedes direct
> PDF reads for DC-2 component values (see `reference/MANIFEST.md`).
>
> **Primary source:** `reference/blueshift_documentation.pdf` (Aion FX Blueshift, faithful
> DC-2 clone) — BOM pages 4–6, schematic page 9. The schematic page is vector artwork;
> in addition to visual transcription, every net cited below was verified by extracting
> the page to SVG and programmatically tracing wire/junction geometry (endpoint merge +
> junction-dot confirmation). Designators are Blueshift's, not Boss factory numbers.
>
> **Factory scan cross-check: NOT PERFORMED — SOURCE FILE INVALID.** See `## Discrepancies`.

---

## 0. Conventions and supply rails

| Rail | Derivation | Nominal | Purpose |
|---|---|---|---|
| +9V  | DC jack via D1 1N4002 (reverse-protection shunt), C1 100µF reservoir | 9.0 V | Op-amps IC1/IC12, NE570s, Q2/Q4/Q5/Q6/Q9/Q10 collectors, LED via R95 |
| VR   | +9V → R4 10k → VR; R3 10k → GND; C3 47µF | 4.5 V | Audio bias reference (op-amp + inputs, coupling bias) |
| +7V  | RG1 78L06, GND pin lifted by D8 1N914 → ≈6.6–6.8 V; C41 100n in, C40 100n out | ≈6.8 V | BBD/clock section supply ("VA") |
| VA   | Directly on +7V output node | ≈6.8 V | LFO/servo section, R59/R60 bias, R52/R53 filtered sub-rails |
| VB   | VA → R31 100R → VB; C28 10µF | ≈6.8 V | IC3 MN3207 VDD (pin 5) + IC4 MN3102 VDD (pin 1) |
| VC   | VA → R30 100R → VC; C38 10µF | ≈6.8 V | IC9 MN3207 VDD (pin 5) + IC8 MN3102 VDD (pin 1) |
| VA/2 | VA → R59 10k → node → R60 10k → GND; C36 47µF | ≈3.4 V | Bias for IC5B(+), IC6A(+), IC7A/IC7B references, R34 |

Boxed letter pads (A, B, C, D, E, F, G, H, I, J, K, L, M, N, O) are the pin-header
connections between the two PCBs. Electrically each adjacent pair drawn in the signal
path (J–I, L–K, G/F–H, D/C–E) is a through-connection; they are treated as shorts.
Pad A = circuit input (calibration injection point, = IC1A output net). Pad B = ground
reference pad at C14(−). TP1/TP2 = calibration test points at the reconstruction-filter
outputs (channel C / channel B respectively).

Channel naming used throughout: **channel C** = IC9 MN3207 + IC8 MN3102, VC rail,
TR1 trim, servo IC6B/Q8, recon Q10 → expander IC11 half-2 → **OUT_L** mixer IC12A.
**channel B** = IC3 MN3207 + IC4 MN3102, VB rail, TR2 trim, servo IC5A/Q7, recon Q4 →
expander IC11 half-1 → **OUT_R** mixer IC12B.

---

## 1. Input buffer (IC1A, OPA2134 ½)

| Ref | Value | From | To |
|---|---|---|---|
| R6 | 10k | IN jack | C4 |
| C4 | 47n | R6 | node N_IN+ |
| R5 | 1M | node N_IN+ | VR |
| — | — | N_IN+ | IC1A pin 3 (+) |
| — | — | IC1A pin 1 (out) | IC1A pin 2 (−) [unity follower]; net "A" |

H(s) = (R5/(R5+R6)) · sτ/(1+sτ), τ = C4·(R5+R6) = 47.47 ms → fc = 3.35 Hz,
passband gain 0.9901.

## 2. Pre-emphasis (IC1B, OPA2134 ½, inverting)

| Ref | Value | From | To |
|---|---|---|---|
| C5 | 4n7 | IC1A out | R8 |
| R8 | 10k | C5 | IC1B pin 6 (−) |
| R7 | 47k | IC1A out | IC1B pin 6 (−) |
| R10 | 33k | IC1B pin 6 | IC1B pin 7 (out) |
| C8 | 100pF | IC1B pin 6 | IC1B pin 7 |
| R9 | 100k | IC1B pin 6 | C7 (via header pins) |
| C7 | 1µF film | R9 | IC1B pin 7 |
| — | — | IC1B pin 5 (+) | VR |

Zi = R7 ∥ (R8 + 1/sC5);  Zf = R10 ∥ (1/sC8) ∥ (R9 + 1/sC7);  H = −Zf/Zi.
Audio-band (C7 short above ≈1.6 Hz, C8 pole at 64 kHz):
H(s) ≈ −0.52792·(1 + s·267.9 µs)/(1 + s·47.0 µs); zero 594.1 Hz, pole 3386.3 Hz,
HF gain 3.009 (+9.57 dB). IC1B output node = "dry rail" feeding C9 (compressor),
R12 (dry LF buffer), R71 (OUT_L mixer), R92 (OUT_R mixer).

## 3. Compressor (IC2 NE570, half 1) — single, shared by both channels

NE570 pin map (Philips DS): 1 RECT CAP1, 2 RECT IN1, 3 ΔG IN1, 4 GND, 5 INV.IN1,
6 R3 1, 7 OUT1, 8 THD TRIM1; 9–16 mirror for half 2; 13 = VCC.

| Ref | Value | From | To |
|---|---|---|---|
| C9 | 10µF | IC1B pin 7 (dry rail) | IC2 pin 6 (input through internal R3 20k) |
| C15 | 10µF | IC2 pin 7 (OUT1) | IC2 pins 2+3 (tied: rect + ΔG feedback ⇒ 2:1 compressor) |
| C10 | 0.47µF tant | IC2 pin 1 (CRECT) | GND |
| R15 | 10k | IC2 pin 7 | node K1 (DC feedback, AN174 topology) |
| R14 | 10k | node K1 | IC2 pin 5 (INV.IN) |
| C14 | 10µF | node K1 | GND (node also = pad B) |
| D3 | 1N5225 (3.0V zener) | IC2 pin 5 (cathode) | D4 anode |
| D4 | 1N5225 (3.0V zener) | D3 anode | C16 |
| C16 | 1n | D4 cathode | IC2 pin 7 (output rail) |
| C18 | 100pF | IC2 pin 8 (THD trim) | GND |
| C17 | 1µF film | IC2 pin 7 | anti-alias filter input node |
| R16 | 100k | AA filter input node | VR |
| — | — | IC2 pins 10+11+12 tied; pins 9, 14, 15, 16 n.c. | (half 2 unused, unity-tied) |
| — | — | IC2 pin 13 | +9V; pin 4 → GND |

D3/D4 + C16: back-to-back zener limiter between the op-amp summing node and the output —
clamps compressor output swing before the BBDs (overload protection).

## 4. Anti-alias low-pass (Q5 2N5088) — single, shared

Bootstrapped emitter-follower 3rd-order LPF; C17/R16 couple and bias its input at VR.

| Ref | Value | From | To |
|---|---|---|---|
| R25 | 10k | filter input node (C17/R16) | node A1 |
| C22 | 470pF | node A1 | GND |
| R26 | 10k | node A1 | node A2 |
| C20 | 1n8 | node A2 | Q5 emitter (bootstrap) |
| R27 | 10k | node A2 | Q5 base |
| C21 | 220pF | Q5 base | GND |
| Q5 | 2N5088 | C: +9V, B: R27/C21, E: output bus | emitter follower |
| R28 | 10k | Q5 emitter | GND |

With ideal unity buffer: H(s) = 1 / (1.8612e-16·s³ + 9.988e-11·s² + 1.130e-5·s + 1)
= complex pair f0 = 17 933 Hz, Q = 0.9930, plus real pole 67 350 Hz.
Q5 emitter bus feeds R62 (→Q9, channel C) and R29 (→Q6, channel B).

## 5. BBD input buffers and BBD/clock cells

Channel C (mirror values for channel B in parentheses):

| Ref | Value | From | To |
|---|---|---|---|
| R62 (R29) | 10k | AA output bus | Q9 (Q6) base |
| Q9 (Q6) | 2N5088 | C: +9V; E: below | emitter follower |
| R63 (R23) | 10k | Q9 (Q6) emitter | GND |
| C39 (C27) | 1µF | Q9 (Q6) emitter | IC9 (IC3) pin 3 (IN), via header |
| R61 (R32) | 100k | IC9 (IC3) pin 3 | TR1 (TR2) wiper |
| TR1 (TR2) | 100k B trimmer | pin 1 → VC (VB), pin 3 → GND | bias ≈3.4 V (calibration) |
| IC9 (IC3) | MN3207 | pin 5 → VC (VB); pin 1 → GND | 1024-stage BBD |
| — | — | IC9 (IC3) pins 7+8 tied (OUT1+OUT2) | recon line |
| R68 (R22) | 56k | pins 7+8 | GND (output load) |
| C65 (C31) | 1µF | IC8 (IC4) pin 8 VGG_out → IC9 (IC3) pin 4 VGG | GND (VGG decoupling) |
| IC8 (IC4) | MN3102 | pin 1 → VC (VB), pin 3 → GND | clock driver, CP1/CP2 (pins 2/4) → BBD pins 6/2 |

MN3207 pins: 1 GND, 2 CP1, 3 IN, 4 VGG, 5 VDD, 6 CP2, 7 OUT1, 8 OUT2.
MN3102 pins: 1 VDD, 2 CP1, 3 GND, 4 CP2, 5 OX3, 6 OX2, 7 OX1, 8 VGG_out.

## 6. Clock VCO / servo (per channel; C-channel refs, B in parentheses)

| Ref | Value | From | To |
|---|---|---|---|
| R53 (R52) | 100R | VA | filtered sub-rail F node |
| C34 (C33) | 10µF | F node | GND (also powers TL072 pin 8: IC6 (IC5)) |
| R56 (R41) | 1k5 | F node | Q8 (Q7) base |
| R58 (R40) | 10k | Q8 (Q7) base | GND |
| R55 (R42) | 8k2 | F node | Q8 (Q7) emitter |
| Q8 (Q7) | 2N5087 PNP | collector | IC6B (IC5A) pin 6/2 (−) node [current source] |
| R57 (R43) | 6k8 | comparator (−) node | D7 (D6) cathode / C35 (C32) node |
| C35 (C32) | 100pF | D7 (D6) cathode | GND (VCO timing cap) |
| D7 (D6) | 1N914 | anode ← IC8 (IC4) pin 5 (OX3) | cathode → C35 (C32) |
| IC6B (IC5A) | TL072 ½ | output | IC8 (IC4) pin 7 (OX1, external excitation) |
| R50 (R46) | 100k | comparator output | comparator (+) input (hysteresis) |
| R54 (R47) | 5k6 | CV source: IC6A out (IC5B out) | comparator (+) node |
| R49 (R44) | 4k7 | comparator (+) node | GND |

Relaxation VCO: Q8 sources ≈20 µA into C35 through R57 (ramp); comparator flips when the
ramp crosses the CV-shifted threshold; MN3102 inverts OX1→OX3 and D7 discharges/clamps
C35; MN3102 divides the OX oscillation by 2 into two-phase CP1/CP2. LFO CV shifts the
comparator threshold → clock-frequency modulation → delay-time modulation.

## 7. LFO (IC7 TL022) and mode switches

| Ref | Value | From | To |
|---|---|---|---|
| IC7A | TL022 ½ | comparator | pin 2 (−) → VA/2; pin 3 (+) node T |
| R33 | 33k | IC7B out (triangle) | node T |
| R35 | 100k | IC7A out (square) | node T |
| C37 | 10n | IC7A out | node T (speed-up) |
| R34 | 100k | node T | VA/2 |
| R36 | 330k | IC7A out | node N (rate node) |
| R37 | 180k | node N | IC7B pin 6 (−) |
| C29 | 4µ7 bipolar | IC7B pin 6 (−) | IC7B out (integrator) |
| — | — | IC7B pin 5 (+) | VA/2 |
| R38 | 22k | IC7B out | node M (depth node) |
| R39 | 33k | node M | IC5B pin 6 (−) |

Mode switches (MODE1 = all off). DPDT switches act on both nodes at once:

| Switch | Pole "depth" side (node M → IC7B out) | Pole "rate" side (node N → IC7A out) |
|---|---|---|
| MODE2 (SPDT) | short (R38 bypassed) | — |
| MODE3 | via R97 470k | via R98 82k |
| MODE4 | via R99 220k | direct short (R36 bypassed) |

Unused throws (positions "3"/"A3"/"B3") are n.c.

## 8. LFO CV scaling / antiphase drive

| Ref | Value | From | To |
|---|---|---|---|
| IC5B | TL072 ½ | inverting amp | pin 5 (+) → VA/2 |
| R39 | 33k | node M (triangle, attenuated) | IC5B pin 6 (−) |
| R45 | 47k | IC5B pin 6 | IC5B pin 7 (out) [feedback] |
| — | — | IC5B out = CV_B | → R47 5k6/R44 4k7 divider → IC5A (+) |
| R48 | 47k | IC5B out | IC6A pin 2 (−) |
| R51 | 47k | IC6A pin 2 | IC6A pin 1 (out) [feedback] |
| — | — | IC6A pin 3 (+) | VA/2 |
| — | — | IC6A out = CV_C = −CV_B (about VA/2) | → R54 5k6/R49 4k7 divider → IC6B (+) |

IC6A inverts the CV ⇒ the two BBD clocks modulate in **antiphase** (audio polarity of
both outputs remains in-phase).

## 9. Reconstruction low-pass filters (Q10 = channel C, Q4 = channel B)

| Ref (C) | Ref (B) | Value | From | To |
|---|---|---|---|---|
| C42 | C26 | 22n | BBD pins 7+8 / 56k load | bias node |
| R67 | R20 | 680k | +9V | bias node |
| R69 | R21 | 680k | bias node | GND |
| R66 | R19 | 10k | bias node | node F1 |
| C43 | C25 | 470pF | node F1 | GND |
| R65 | R18 | 10k | node F1 | node F2 |
| C44 | C24 | 1n8 | node F2 | emitter (bootstrap) |
| R64 | R17 | 10k | node F2 | Q10 (Q4) base |
| C45 | C23 | 220pF | Q10 (Q4) base | GND |
| Q10 | Q4 | 2N5088 | C: +9V | emitter follower |
| R70 | R24 | 10k | emitter | GND |
| — | — | — | emitter = TP1 (TP2) | → C46 (C19) 10µF → expander input |

Input coupling HPF: 22n into 680k∥680k = 340k → fc = 21.3 Hz. LPF section identical to
the anti-alias cubic (f0 = 17 933 Hz, Q = 0.9930, real pole 67 350 Hz).

## 10. Expanders (IC11 NE570, both halves)

| Ref | Value | From | To |
|---|---|---|---|
| C46 | 10µF | Q10 emitter (recon C out) | IC11 pins 15+14 tied (RECT IN2 + ΔG IN2) ⇒ expander |
| C48 | 0.47µF tant | IC11 pin 16 (CRECT2) | GND |
| — | — | IC11 pins 10+11 tied (OUT2 + R3 2 ⇒ R3 = feedback) | wet-C bus (direct) |
| C61 | 100pF | IC11 pin 9 (THD trim 2) | GND |
| C19 | 10µF | Q4 emitter (recon B out) | IC11 pins 2+3 tied (RECT IN1 + ΔG IN1) |
| C47 | 0.47µF tant | IC11 pin 1 (CRECT1) | GND |
| — | — | IC11 pins 6+7 tied (R3 1 + OUT1) | C13 |
| C13 | 10µF | IC11 pins 6+7 | wet-B bus |
| C12 | 100pF | IC11 pin 8 (THD trim 1) | GND |
| — | — | IC11 pin 13 → +9V; pin 4 → GND; pins 5, 12 n.c. (INV.IN internal) | |

## 11. Dry low-frequency buffer (Q2)

| Ref | Value | From | To |
|---|---|---|---|
| R12 | 68k | dry rail (IC1B out) | node D1 |
| C11 | 22n | node D1 | GND |
| R11 | 2k2 | node D1 | Q2 base |
| Q2 | 2N5088 | C: +9V | emitter follower |
| R13 | 10k | Q2 emitter | GND |
| C6 | 1µF | Q2 emitter | dry-LF bus |
| R89 | 1M | dry-LF bus | VR |

1st-order LPF, fc ≈ 106 Hz (R12·C11); feeds R87 (OUT_L) and R88 (OUT_R): low-frequency
dry reinforcement of both outputs.

## 12. Output mixers and spatial EQ (IC12A = OUT_L, IC12B = OUT_R)

Both mixers are identical inverting summers around OPA2134 halves, + input at VR.
Summing node of IC12A = pads J–I; of IC12B = pads L–K.

Inputs into IC12A (OUT_L) summing node:

| Ref | Value | From |
|---|---|---|
| R71 | 33k | dry rail (pre-emphasized) |
| R87 | 220k | dry-LF bus (Q2/C6) |
| R82 | 47k | ← C59 18n ← wet-C bus (IC11 pins 10+11) |
| R86 | 39k | ← C62 18n ← J113 mono-sum node (see §13) |

Inputs into IC12B (OUT_R) summing node:

| Ref | Value | From |
|---|---|---|
| R92 | 33k | dry rail (pre-emphasized) |
| R88 | 220k | dry-LF bus (Q2/C6) |
| R85 | 47k | ← C60 18n ← wet-B bus (IC11 pins 6+7 via C13) |

Feedback network (identical both sides; IC12A refs / IC12B refs):

| Ref | Value | From | To |
|---|---|---|---|
| R75 / R76 | 47k | summing node | output |
| C50 / C53 | 47pF | summing node | output |
| C51 / C52 | 4n7 | summing node | R74 / R77 |
| R74 / R77 | 10k | C51 / C52 | output |

Zf(s) = R75 ∥ (R74 + 1/sC51): shelf with corners 594.1 Hz / 3386.3 Hz — the exact
inverse of the pre-emphasis (C51·(R74+R75) = C5·(R7+R8) and C51·R74 = C5·R8), so the
dry path is net-flat.

Cross-feedback bridged-T (the "Dimension" spatial network):

| Ref | Value | From | To |
|---|---|---|---|
| R80 / R81 | 180k | IC12A / IC12B summing node | node Y_A / Y_B |
| R84 / R83 | 33k | same summing node | C56 / C57 |
| C56 / C57 | 1n2 | R84 / R83 | node Y_A / Y_B |
| C55 / C58 | 3n3 | node Y_A / Y_B | **opposite** mixer output (IC12B out / IC12A out) |

Injected current: I = V_other/(1/sC55 + R80∥(R84 + 1/sC56)) — high-passed, inverted
cross-blend of the opposite output (−21 dB @100 Hz → −11 dB @16 kHz including Zf).

Output couplings:

| Ref | Value | From | To |
|---|---|---|---|
| C49 / C54 | 1µF | IC12A / IC12B out | pads G/F–H, D/C–E |
| R73 / R78 | 100k | output node | GND |
| R72 / R79 | 1k | output node | OUT_L (pad P) / OUT_R (pad O) |
| C63 / C64 | 1µF | net "A" (IC1A out) | output nodes (bypass dry path; bypass wiring only) |

## 13. Mono/stereo switching (Q3 J113) and ring sense

| Ref | Value | From | To |
|---|---|---|---|
| Q3 | J113 JFET | drain: C62 left node | source: wet-B bus |
| R90 | 1M | VR | Q3 drain node (C62 left) |
| R91 | 1M | VR | Q3 source node (wet-B bus) |
| D5 | 1N914 | anode: ring-sense line | cathode: Q3 gate |
| R1 | 1M | ring-sense line | OUT_R_RING (pad M) |
| R2 | 100k | +9V | ring-sense line |
| C2 | 47n | ring-sense line | GND |

No plug in OUT B ⇒ ring open ⇒ R2 pulls sense high ⇒ J113 ON ⇒ wet-B is also injected
into the OUT_L mixer through C62 18n → R86 39k (mono sum at output A). Plug inserted ⇒
ring grounded ⇒ J113 OFF ⇒ stereo. (Stereo operation is the modeling target; the R86/C62
branch is inactive in stereo.)

## 14. Indicator / bypass

| Ref | Value | Role |
|---|---|---|
| R95 | 3k9 | +9V → LED (5mm) → header |
| OPTO | H11F1 | opto-FET bypass element; pin 1/2 LED side, 6/4 = OPT_IN/OPT_OUT |
| C30 | 220n | pad N line → GND (switch debounce/pop filter) |

Not part of the audio model.

## 15. BOM ↔ schematic audit

Every schematic designator above matches the BOM values on pages 4–5 of the Blueshift
document. BOM designator gaps (intentional in Blueshift numbering, no schematic
counterpart found): R93, R94, R96; D2; Q1; IC10. All capacitors C1–C65 accounted for.
2N5088 (NPN, hFE 300–900) and 2N5087 (PNP) per BOM "Transistors" table; NE570/571
interchangeable per part note 2.

---

## Discrepancies

| # | Item | Blueshift | Factory scan | Resolution |
|---|---|---|---|---|
| 1 | **Factory scan file `BBSS2501411.pdf`** | — | File contained a *Behavioral and Brain Sciences* manuscript ("A speculative argument against consciousness in AI", N. Block), 8 pages, **no schematic content on any page**; removed from `reference/` 2026-07-10 at user request | **[FACTORY SCAN UNAVAILABLE — WRONG FILE UPLOADED, SINCE REMOVED].** Cross-check not performed. Blueshift redraw stands as sole authority (it is the designated working authority). User must supply the correct Boss DC-2 factory scan; re-audit then. |
| 2 | BOM lists MODE2 as SPDT, MODE3/MODE4 as DPDT | Schematic shows MODE2 with one pole (depth side only); MODE3A/B and MODE4A/B poles on depth/rate sides | consistent | No action |
| 3 | R80/R81 printed "180K" in schematic, "180k" in BOM | same value | — | No action |

No `[UNREADABLE - ASK USER]` items: every component value and connection in the audio
path was resolved from the vector schematic and confirmed by geometric net tracing.

---

## Block classification

| Block | Realization | Justification |
|---|---|---|
| Input buffer (IC1A + R6/C4/R5) | **Biquad path (1st-order digital HPF)** | Passive RC into ideal op-amp follower (nullor); series-C/shunt-R divider is algebraically transparent as WDF (resistor b[n]=0, Fettweis 1986) — project rule forbids WDF for RC dividers |
| Pre-emphasis (IC1B network) | **Biquad (1st-order shelf)** | Op-amp gain stage (nullor convention, Werner et al. 2016); impedances only define H(s) |
| Compressor IC2 + zener limiter | **Deferred to session 3** (NE570 behavioral model, DmmCompander precedent) | Nonlinear gain cell; not a filter |
| Anti-alias LPF (Q5 + RC ladder) | **Biquad (2nd order; 67 kHz real pole dropped)** | Bootstrapped emitter follower ≡ unity buffer (2N5088 hFE ≥ 300 ⇒ loading error ≪ 1.5 dB); active RC filter — not series/parallel decomposable WDF without R-type adaptor (forbidden this build) |
| BBD input buffers Q9/Q6 + bias | **Unity (ideal buffer)** | Emitter followers; couplings are sub-audio (1µF/100k → 1.6 Hz) |
| BBD + clock + servo + LFO | **Deferred to sessions 3+** | Sampled analog delay + VCO; not a filter |
| Reconstruction LPFs (Q10/Q4) | **1st-order HPF (21.3 Hz) + same biquad** | As anti-alias; plus 22n/680k∥680k input coupling |
| Expander IC11 | **Deferred to session 3** (NE570 behavioral) | Nonlinear gain cell |
| Dry LF buffer (Q2 + R12/C11) | **Biquad path (1st-order LPF 106 Hz)** | RC into EF buffer; RC divider rule |
| Output mixers IC12A/B + EQ + cross-feedback | **Biquad set: common feedback shelf (1st-order) + cross-feedback biquad (2nd-order)** | Inverting op-amp summers (nullor); virtual-ground superposition makes each input a scalar; C55/C56 bridged-T handled as admittance biquad |
| Output couplings (C49/R73/R72 etc.) | **Unity (ignored)** | 1.6 Hz HPF, out of band |
| Mono J113 branch (C62/R86) | **Not modeled (stereo target)**; hook documented | Mode switch, not a filter |

No WDF one-port chains survive classification: every passive network in the DC-2 audio
path is either an RC divider (forbidden as WDF by the b[n]=0 rule) or embedded in an
active stage. No R-type adaptor is required anywhere. **Session-1 output is therefore
100% bilinear-transform sections**, consistent with the expected classification.

---

## Session-1 modeling decisions (documented deviations)

1. **AA/recon real pole at 67 350 Hz dropped** — above Nyquist at fs = 48 kHz; error
   ≤ 0.06 dB at 8 kHz, ≤ 0.4 dB at 20 kHz.
2. **C8 100pF (pre-emphasis) and C50/C53 47pF (mixer feedback) ignored** — poles at
   64 kHz / 72 kHz, out of band.
3. **C7 1µF (pre-emphasis) treated as short** — affects response only below ≈1.6 Hz
   (model error ≤ 0.012 dB at 20 Hz).
4. **Sub-audio couplings treated as unity**: C17/R16 (1.6 Hz), C39/C27+R61/R32 bias
   (≈1.6 Hz), C6/R89 (0.16 Hz), C46/C19 expander couplings, C49/C54+R73/R78 (1.6 Hz).
   The two in-band HPFs are kept: input C4/(R5+R6) = 3.35 Hz and recon 22n/340k = 21.3 Hz.
5. **Emitter followers (Q2/Q4/Q5/Q6/Q9/Q10, 2N5088)** modeled as ideal unity buffers:
   output impedance ≈ re + R_src/hFE ≈ 55–100 Ω against ≥10k loads, gain error
   < 0.1 dB — far below the 1.5 dB test threshold. Op-amps ideal (nullor).
6. **Polarity**: IC1B and IC12A/B invert; NE570 stages invert. Net dry and wet paths
   are both non-inverting overall, and OUT_L/OUT_R are in-phase (mono-compatible).
   Session-1 blocks carry their physical signs.
7. **Dc2DryLowpass discretization** — bilinear-prewarped pole with a magnitude-matched
   zero (exact at DC and fs/2) instead of the default BLT zero at z = −1: a plain BLT
   one-pole at fc = 106 Hz misses the 0.5 dB gate at the 8 kHz probe (−0.85 dB tan-warp
   error deep in the −20 dB/dec region, irreducible by prewarp choice). Residual error:
   −0.30 dB @8 kHz, ≤0.005 dB @100 Hz/1 kHz. Derivation in `wdf/Dc2Filters.h`.
