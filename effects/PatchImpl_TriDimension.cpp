/*
 * PatchImpl_TriDimension.cpp — "TriDimension": Dytronics Tri-Stereo Chorus
 * (wdf/TscCircuit.h) + Boss DC-2 Dimension C (wdf/Dc2Circuit.h) behind a
 * three-zone routing knob. Tridimension Session 5 of 6.
 *
 * VIRTUAL CONTROLS (six-slot heel/toe morph array — NOT hardware indices):
 *   v0 dc2Mode  — discrete 4-position: mode = min(3, int(x * 4.0f)); no smoothing
 *   v1 tscRate  — log-feel 0.03..7.45 Hz: hz = 0.03f * powf(7.45f/0.03f, x);
 *                 knob value smoothed 20 ms at control rate
 *   v2 routing  — three zones with 0.02 hysteresis at the 0.33 / 0.66 bounds:
 *                 x < 0.33 TSC→DC2 series | 0.33..0.66 DC2→TSC series |
 *                 x > 0.66 parallel. Zone changes trigger the 512-sample
 *                 equal-power crossfade.
 *   v3/v4/v5    — TSC intensity L/C/R, 0..1 (smoothed per sample inside
 *                 TscCircuit, 20 ms)
 *
 * HARDWARE MAP: base layer knobs 0/1/2 → v0/v1/v2; hold layer → v3/v4/v5.
 *   kLeftFootSwitchHold  — toggles the knob layer (LATCHED: the API has no
 *                          hold-release event — verified in sdk/Patch.h)
 *   kLeftFootSwitchPress — toggles DC-2 enable, through the crossfade
 *   Expression (param 3) — heel/toe snapshot morph across all six virtual
 *   controls: effective[i] = heel[i] + (toe[i] - heel[i]) * expr, with expr
 *   smoothed over ~20 ms.
 *   SNAPSHOT WRITE RULE: a knob edit writes heel[] when raw expr <= 0.5,
 *   else toe[]. SOFT TAKEOVER on every layer toggle and every expr
 *   0.5-crossing: a disengaged knob re-engages only when it comes within
 *   0.02 of the stored value or sweeps across it; engaged writes then follow
 *   the DMM discipline (store only — application happens at control rate).
 *
 * AUDIO: source = 0.5f * (inL + inR) — BOTH hardware units are mono-input.
 *   Full unit output (each unit's documented composition):
 *     TSC: full = in * kDryGain(1.0)     + wet * kWetGain(0.7)
 *     DC2: full = in * kDryGain(0.75188) + wet   (wet summing weight 1.0)
 *   Series A→B: B's input is A's full MONO-SUMMED output 0.5f*(fullL+fullR).
 *   Parallel: out = source + 0.707f * (wetTsc + wetDc2) per channel (LOCKED
 *   default; wetTsc carries the TSC's calibrated kWetGain, wetDc2's weight
 *   is the DC-2's own 1.0).
 *   DC-2 DISABLED: its hop degrades to a wire — series zones pass the
 *   upstream signal through (stereo where the upstream is the TSC), the
 *   parallel zone drops the DC-2 wet term. Both circuits still RUN every
 *   sample (constant cycle cost; re-enable resumes warm — no thump).
 *
 * MIX OWNERSHIP: the firmware right-footswitch layer owns input level,
 * output level and global wet/dry. This patch exposes NO mix control and
 * keeps the units' authentic internal dry/wet summing.
 *
 * CROSSFADE ENGINE (one ramp shared by zone changes and DC-2 toggles; if
 * both trigger they fold into one combined target — a change landing
 * mid-ramp RESTARTS the ramp toward the new combined config):
 *   equal-power g_out = cos(x*pi/2), g_in = sin(x*pi/2), x linear over 512
 *   samples, realized as an incremental 2D rotation (no per-sample trig).
 *   During the ramp both series feeds use the PREVIOUS sample's full mono
 *   outputs (one-sample loop delay — Dc2Circuit cross-feedback precedent)
 *   so both configurations' feeds exist regardless of evaluation order.
 *   The two circuit INPUTS blend linearly (they are strongly correlated
 *   versions of the same program); the two composed OUTPUTS blend
 *   equal-power. Degenerate pairs whose output formulas are IDENTICAL
 *   (every config that composes to the TSC full output: zone0/DC2-off,
 *   zone1/on, zone1/off) skip the output crossfade — the input ramp alone
 *   morphs the sound; cos+sin applied to two identical signals would pump
 *   +3 dB mid-ramp.
 *
 * LED: base layer Color::kLightBlueColor, hold layer Color::kLightYellow
 *   (spellings verified against sdk/Patch.h). DC-2 disabled: a brief 50 ms
 *   blink once per second to the DIM same-hue member — kDimBlue on the base
 *   layer, kDimYellow on the hold layer. Choice documented: the enum has no
 *   "off" member, and a same-hue dim blink signals DC-2-off while keeping
 *   the active layer readable at a glance. The pulse counter increments
 *   unconditionally in processAudio (DMM pattern).
 *
 * WORKING BUFFER: NOT used. All delay lines are small fixed-size members
 *   (5 x BBDLine<1024> ≈ 5 x 4 KB, DTCM class state) — do NOT wire
 *   setWorkingBuffer to anything.
 *
 * CYCLE BUDGET (528 MHz / 48 kHz = 11,000 cycles/sample; whole-patch
 * WARN > 4,400, FAIL > 6,600):
 *   TscCircuit processSample     ~1,310–2,230   (Session 4 estimate)
 *   Dc2Circuit processSample     ~  980–1,765   (Session 3 estimate)
 *   patch glue + composition     ~   60–120
 *   control tick amortized /32   ~   10–20      (powf lives here, not per sample)
 *   crossfade extra (ramps only) ~   30–50
 *   ────────────────────────────────────────
 *   TOTAL ESTIMATED              ~2,400–4,200 cycles/sample — under the
 *   4,400 WARN threshold but close at the top of the range. Session 6 runs
 *   the formal benchmark before any DONE marker is written.
 *
 * Desktop compile gate (single command line, wrapped for readability):
 *   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra
 *       -DENDLESS_DESKTOP_BUILD -I. -I./sdk -I./wdf -I./dsp
 *       -c effects/PatchImpl_TriDimension.cpp
 * Smoke test: add -DTRIDIMENSION_SMOKE and link with -lm (main() below).
 */

#include "sdk/Patch.h"
#include "wdf/Dc2Circuit.h"
#include "wdf/TscCircuit.h"
#include "dsp/ParameterSmoother.h"

#include <array>
#include <cmath>

// SESSION 5 VERIFIED API FACTS (sdk/Patch.h read in full):
// setParamValue    : indices 0/1/2 = knobs, index 3 = expression pedal
//                    (PatchImpl_Wah.cpp precedent); called from the audio
//                    thread — no synchronization needed
// handleAction     : ActionId::kLeftFootSwitchPress = 0, kLeftFootSwitchHold = 1;
//                    there is NO release event — hold is a latched toggle
// getStateLedColor : returns Color enum; kLightBlueColor / kLightYellow /
//                    kDimBlue / kDimYellow all verified spellings
// processAudio     : two dynamic std::span<float>, equal sizes guaranteed
// setWorkingBuffer : std::span<float, 2400000> — intentionally unused here

class PatchImpl : public Patch
{
public:
    // -------------------------------------------------------------------------
    // Platform + patch constants (528 MHz operational clock, not marketed 720)
    // -------------------------------------------------------------------------
    static constexpr float kClockHz       = 528'000'000.0f;
    static constexpr float kSampleRateF   = 48'000.0f;
    static constexpr float kCyclesPerSamp = kClockHz / kSampleRateF;   // 11,000

    static constexpr int   kControlInterval  = 32;    // [CONTROL-RATE] divider
    static constexpr int   kXfadeSamples     = 512;   // spec: 512-sample ramp
    static constexpr float kInvXfadeSamples  = 1.0f / static_cast<float>(kXfadeSamples);
    static constexpr float kTakeoverWindow   = 0.02f; // soft-takeover pickup band
    static constexpr float kZoneLow          = 0.33f; // routing boundaries
    static constexpr float kZoneHigh         = 0.66f;
    static constexpr float kZoneHyst         = 0.02f; // hysteresis band each side
    static constexpr float kParallelWetGain  = 0.707f; // LOCKED parallel default
    static constexpr float kExprSmoothMs     = 20.0f; // expression morph smoothing
    static constexpr float kRateSmoothMs     = 20.0f; // v1 knob smoothing
    static constexpr int   kLedPeriodSamples = 48'000; // 1 s blink period
    static constexpr int   kLedBlinkSamples  = 2'400;  // 50 ms blink window

    // Virtual-control defaults (heel == toe at init → morph inert until the
    // snapshots diverge). v1 = 0.5f maps to the TSC's documented default rate
    // (0.03 * (7.45/0.03)^0.5 = 0.473 Hz, its [DATASHEET-derived] default);
    // v2 = 0.165f sits mid-zone-0 (TSC→DC2 series); v3..v5 match TscCircuit's
    // kDefaultIntensity.
    static constexpr std::array<float, 6> kDefaultV{
        0.0f, 0.5f, 0.165f, 0.5f, 0.5f, 0.5f};

    // -------------------------------------------------------------------------
    // Patch interface
    // -------------------------------------------------------------------------

    void init() override
    {
        // WORKING BUFFER NOT USED: every delay line in both circuits is a
        // small fixed member (5 x BBDLine<1024> ≈ 5 x 4 KB total, DTCM).
        // Nobody should wire setWorkingBuffer for this patch.
        tsc_.init(kSampleRateF);
        dc2_.init(kSampleRateF);

        // Control-rate smoothers run once per kControlInterval samples, so
        // they are initialized at the CONTROL rate for a true ~20 ms tau.
        const float controlRate = kSampleRateF / static_cast<float>(kControlInterval);
        exprSm_.init(controlRate, kExprSmoothMs);
        exprSm_.snapTo(0.0f);
        rateSm_.init(controlRate, kRateSmoothMs);
        rateSm_.snapTo(kDefaultV[1]);

        heel_ = kDefaultV;
        toe_  = kDefaultV;
        eff_  = kDefaultV;

        // Equal-power ramp as an incremental rotation: per-sample step
        // delta = (pi/2)/512; (cos, sin) advance by one rotation per sample.
        const float delta = 1.5707963f * kInvXfadeSamples;
        xfCd_ = cosf(delta);
        xfSd_ = sinf(delta);

        targetCfg_ = Config{0, true};
        oldCfg_    = targetCfg_;
        xfadeCount_ = kXfadeSamples;   // idle — no ramp in flight
    }

    void setWorkingBuffer(std::span<float, kWorkingBufferSize> /*buffer*/) override
    {
        // Intentionally empty — see the note in init(). All state is in
        // fixed-size DTCM members; external RAM is not touched.
    }

    void processAudio(std::span<float> audioBufferLeft,
                      std::span<float> audioBufferRight) noexcept override
    {
        const size_t n = audioBufferLeft.size();   // right is the same size
        for (size_t i = 0; i < n; ++i)
        {
            // [CONTROL-RATE] every 32 samples: morph, mode, rate, intensity,
            // routing hysteresis + crossfade trigger.
            if (__builtin_expect(ctrlPhase_ == 0, 0))
                controlTick();
            if (__builtin_expect(++ctrlPhase_ >= kControlInterval, 0))
                ctrlPhase_ = 0;

            // Both hardware units are mono-input: mono-sum the patch input.
            const float source = 0.5f * (audioBufferLeft[i] + audioBufferRight[i]);

            float outL, outR;
            if (__builtin_expect(xfadeCount_ >= kXfadeSamples, 1))
                processSteady(source, outL, outR);
            else
                processXfade(source, outL, outR);

            // [DTCM] previous-sample full mono outputs feed the crossfade
            // path's series hops (one-sample loop delay).
            prevTscFullMono_ = 0.5f * (tscFullL_ + tscFullR_);
            prevDc2FullMono_ = 0.5f * (dc2FullL_ + dc2FullR_);

            audioBufferLeft[i]  = outL;
            audioBufferRight[i] = outR;

            // LED pulse counter runs unconditionally (DMM pattern).
            if (__builtin_expect(++ledCounter_ >= kLedPeriodSamples, 0))
                ledCounter_ = 0;
        }
    }

    ParameterMetadata getParameterMetadata(int paramIdx) override
    {
        switch (paramIdx)
        {
            // Base-layer meanings shown; the hold layer reuses the same
            // hardware range for v3/v4/v5 (all 0..1).
            case 0:  return ParameterMetadata{0.0f, 1.0f, kDefaultV[0]}; // DC-2 mode
            case 1:  return ParameterMetadata{0.0f, 1.0f, kDefaultV[1]}; // TSC rate
            case 2:  return ParameterMetadata{0.0f, 1.0f, kDefaultV[2]}; // routing
            case 3:  return ParameterMetadata{0.0f, 1.0f, 0.0f};         // expression
            default: return ParameterMetadata{0.0f, 1.0f, 0.5f};
        }
    }

    void setParamValue(int paramIdx, float value) override
    {
        // Audio-thread call (per sdk/Patch.h) — plain member writes are safe.
        if (value < 0.0f) value = 0.0f;
        if (value > 1.0f) value = 1.0f;

        if (paramIdx >= 0 && paramIdx <= 2)
        {
            handleKnobEdit(paramIdx, value);
        }
        else if (paramIdx == 3)
        {
            // Expression 0.5-crossing → soft takeover on all three knobs.
            if ((exprRaw_ <= 0.5f) != (value <= 0.5f))
                disengageKnobs();
            exprRaw_ = value;
            exprSm_.setTarget(value);
        }
    }

    bool isParamEnabled(int paramIdx, ParamSource source) override
    {
        switch (source)
        {
            case ParamSource::kParamSourceKnob:
                return paramIdx >= 0 && paramIdx <= 2;
            case ParamSource::kParamSourceExpression:
                return paramIdx == 3;   // expression pedal → snapshot morph
            default:
                return false;
        }
    }

    void handleAction(int actionIdx) override
    {
        if (actionIdx == static_cast<int>(endless::ActionId::kLeftFootSwitchPress))
        {
            // DC-2 enable toggle; the config change is picked up at the next
            // control tick (≤ 32 samples) and runs through the crossfade.
            dc2Enabled_ = !dc2Enabled_;
            if (!dc2Enabled_)
                ledCounter_ = 0;   // start the off-blink immediately
        }
        else if (actionIdx == static_cast<int>(endless::ActionId::kLeftFootSwitchHold))
        {
            // Latched layer toggle (no release event in the API) — the three
            // hardware knobs re-map to the other virtual-control triple, so
            // they must re-engage via soft takeover.
            layerHold_ = !layerHold_;
            disengageKnobs();
        }
    }

    Color getStateLedColor() noexcept override
    {
        // DC-2-off indication: brief dim-blink once per second, same hue as
        // the active layer (choice documented in the file header).
        const bool blink = !dc2Enabled_ && (ledCounter_ < kLedBlinkSamples);
        if (layerHold_)
            return blink ? Color::kDimYellow : Color::kLightYellow;
        return blink ? Color::kDimBlue : Color::kLightBlueColor;
    }

#ifdef TRIDIMENSION_SMOKE
    // Test-only observability (desktop smoke build; never compiled on target)
    [[nodiscard]] float smokeEffective(int i) const noexcept
    {
        return eff_[static_cast<size_t>(i)];
    }
    [[nodiscard]] int  smokeZone() const noexcept { return targetCfg_.zone; }
    [[nodiscard]] bool smokeDc2Enabled() const noexcept { return dc2Enabled_; }
    [[nodiscard]] bool smokeHoldLayer() const noexcept { return layerHold_; }
#endif

private:
    // Routing/enable configuration — the crossfade engine's endpoints.
    struct Config
    {
        int  zone  = 0;      // 0 = TSC→DC2, 1 = DC2→TSC, 2 = parallel
        bool dc2On = true;
        bool operator==(const Config&) const = default;
    };

    // -------------------------------------------------------------------------
    // Per-sample unit runners. BOTH units run EVERY sample regardless of
    // routing/enable state: cycle cost stays constant and a re-enabled DC-2
    // resumes with warm state.
    // -------------------------------------------------------------------------
    void runTsc(float in) noexcept
    {
        float wetL, wetR;
        tsc_.processSample(in, wetL, wetR);
        tscWetGL_ = wetL * TscCircuit::kWetGain;   // calibrated wet contribution
        tscWetGR_ = wetR * TscCircuit::kWetGain;
        tscFullL_ = in * TscCircuit::kDryGain + tscWetGL_;
        tscFullR_ = in * TscCircuit::kDryGain + tscWetGR_;
    }

    void runDc2(float in) noexcept
    {
        float wetL, wetR;
        dc2_.processSample(in, wetL, wetR);
        dc2WetL_  = wetL;   // DC-2 wet summing weight is 1.0 (kWetGainL/R)
        dc2WetR_  = wetR;
        dc2FullL_ = in * Dc2Circuit::kDryGain + wetL;
        dc2FullR_ = in * Dc2Circuit::kDryGain + wetR;
    }

    // Steady state: series feeds use the CURRENT sample's full mono output
    // (spec: B's input is A's full mono-summed output).
    void processSteady(float source, float& outL, float& outR) noexcept
    {
        const Config c = targetCfg_;
        switch (c.zone)
        {
            case 0:   // TSC → DC2 series
            {
                runTsc(source);
                runDc2(0.5f * (tscFullL_ + tscFullR_));
                if (__builtin_expect(c.dc2On, 1))
                {
                    outL = dc2FullL_;
                    outR = dc2FullR_;
                }
                else
                {
                    // DC-2 hop bypassed to a wire: upstream stereo passes.
                    outL = tscFullL_;
                    outR = tscFullR_;
                }
                break;
            }
            case 1:   // DC2 → TSC series
            {
                runDc2(source);
                runTsc(c.dc2On ? 0.5f * (dc2FullL_ + dc2FullR_) : source);
                outL = tscFullL_;
                outR = tscFullR_;
                break;
            }
            default:  // 2 — parallel (locked 0.707 wet summing)
            {
                runTsc(source);
                runDc2(source);
                if (__builtin_expect(c.dc2On, 1))
                {
                    outL = source + kParallelWetGain * (tscWetGL_ + dc2WetL_);
                    outR = source + kParallelWetGain * (tscWetGR_ + dc2WetR_);
                }
                else
                {
                    outL = source + kParallelWetGain * tscWetGL_;
                    outR = source + kParallelWetGain * tscWetGR_;
                }
                break;
            }
        }
    }

    // Crossfade helpers: what each unit's input would be under a config.
    // Series feeds use the PREVIOUS sample's full mono outputs during the
    // ramp (one-sample loop delay) so both configs' feeds exist regardless
    // of evaluation order — see the file-header crossfade note.
    [[nodiscard]] float tscInFor(const Config& c, float source) const noexcept
    {
        return (c.zone == 1 && c.dc2On) ? prevDc2FullMono_ : source;
    }
    [[nodiscard]] float dc2InFor(const Config& c, float source) const noexcept
    {
        return (c.zone == 0) ? prevTscFullMono_ : source;
    }

    // Output-composition identity classes (degenerate-pair detection):
    //   0 = DC-2 full stereo, 1 = TSC full stereo, 2 = parallel with DC-2,
    //   3 = parallel without DC-2.
    [[nodiscard]] static int formulaId(const Config& c) noexcept
    {
        if (c.zone == 0) return c.dc2On ? 0 : 1;
        if (c.zone == 1) return 1;
        return c.dc2On ? 2 : 3;
    }

    void composeFor(int id, float source, float& outL, float& outR) const noexcept
    {
        switch (id)
        {
            case 0:
                outL = dc2FullL_;
                outR = dc2FullR_;
                break;
            case 1:
                outL = tscFullL_;
                outR = tscFullR_;
                break;
            case 2:
                outL = source + kParallelWetGain * (tscWetGL_ + dc2WetL_);
                outR = source + kParallelWetGain * (tscWetGR_ + dc2WetR_);
                break;
            default:
                outL = source + kParallelWetGain * tscWetGL_;
                outR = source + kParallelWetGain * tscWetGR_;
                break;
        }
    }

    void processXfade(float source, float& outL, float& outR) noexcept
    {
        // Inputs blend LINEARLY (strongly correlated feeds); outputs blend
        // equal-power. t is the linear ramp position in [0, 1).
        const float t = static_cast<float>(xfadeCount_) * kInvXfadeSamples;
        const float tscIn = (1.0f - t) * tscInFor(oldCfg_, source)
                          + t * tscInFor(targetCfg_, source);
        const float dc2In = (1.0f - t) * dc2InFor(oldCfg_, source)
                          + t * dc2InFor(targetCfg_, source);

        // Feeds depend only on source and previous-sample outputs, so the
        // evaluation order is free here.
        runTsc(tscIn);
        runDc2(dc2In);

        const int idOld = formulaId(oldCfg_);
        const int idNew = formulaId(targetCfg_);
        float newL, newR;
        composeFor(idNew, source, newL, newR);
        if (__builtin_expect(idOld == idNew, 0))
        {
            // Degenerate pair: identical output formulas — the input ramp
            // alone morphs the sound (equal-power on identical signals
            // would pump +3 dB mid-ramp).
            outL = newL;
            outR = newR;
        }
        else
        {
            float oldL, oldR;
            composeFor(idOld, source, oldL, oldR);
            // g_out = cos(x*pi/2) = xfC_, g_in = sin(x*pi/2) = xfS_.
            outL = xfC_ * oldL + xfS_ * newL;
            outR = xfC_ * oldR + xfS_ * newR;
        }

        // Advance the equal-power rotation and the ramp counter.
        const float c = xfC_, s = xfS_;
        xfC_ = c * xfCd_ - s * xfSd_;
        xfS_ = s * xfCd_ + c * xfSd_;
        ++xfadeCount_;   // reaching kXfadeSamples parks the engine (idle)
    }

    // -------------------------------------------------------------------------
    // [CONTROL-RATE] tick — every kControlInterval samples
    // -------------------------------------------------------------------------
    void controlTick() noexcept
    {
        // Expression morph across all six virtual controls (smoothed ~20 ms).
        const float expr = exprSm_.process();
        for (size_t i = 0; i < 6; ++i)
            eff_[i] = heel_[i] + (toe_[i] - heel_[i]) * expr;

        // v0 — DC-2 mode: discrete 4-position, NO smoothing (spec).
        int mode = static_cast<int>(eff_[0] * 4.0f);
        if (mode > 3) mode = 3;
        if (__builtin_expect(mode != dc2_.getMode(), 0))
            dc2_.setMode(mode);

        // v1 — TSC rate: 20 ms smoothed knob, log-feel mapping. The powf
        // lives here at control rate (1.5 kHz), never in the sample loop.
        rateSm_.setTarget(eff_[1]);
        const float rx = rateSm_.process();
        tsc_.setRate(TscCircuit::kRateMinHz
                     * powf(TscCircuit::kRateMaxHz / TscCircuit::kRateMinHz, rx));

        // v3..v5 — per-line intensities (TscCircuit smooths per sample).
        tsc_.setIntensity(0, eff_[3]);
        tsc_.setIntensity(1, eff_[4]);
        tsc_.setIntensity(2, eff_[5]);

        // v2 — routing zone with hysteresis, plus the DC-2 enable state:
        // any difference from the active target starts (or restarts) the
        // shared crossfade ramp.
        const Config desired{zoneFromKnob(eff_[2], targetCfg_.zone), dc2Enabled_};
        if (__builtin_expect(!(desired == targetCfg_), 0))
        {
            oldCfg_     = targetCfg_;
            targetCfg_  = desired;
            xfadeCount_ = 0;      // restart the ramp (spec)
            xfC_        = 1.0f;   // g_out = cos(0)
            xfS_        = 0.0f;   // g_in  = sin(0)
        }
    }

    // Three-zone lookup with a 0.02 hysteresis band at each boundary. The
    // current zone widens its own territory by the band, so jitter around a
    // boundary cannot retrigger crossfades. Two-zone jumps (expression-driven
    // morphs) resolve in one call.
    [[nodiscard]] static int zoneFromKnob(float x, int currentZone) noexcept
    {
        switch (currentZone)
        {
            case 0:
                if (x > kZoneHigh + kZoneHyst) return 2;
                if (x > kZoneLow + kZoneHyst)  return 1;
                return 0;
            case 1:
                if (x < kZoneLow - kZoneHyst)  return 0;
                if (x > kZoneHigh + kZoneHyst) return 2;
                return 1;
            default:   // 2
                if (x < kZoneLow - kZoneHyst)  return 0;
                if (x < kZoneHigh - kZoneHyst) return 1;
                return 2;
        }
    }

    // -------------------------------------------------------------------------
    // Knob dispatch: layer mapping, snapshot write rule, soft takeover
    // -------------------------------------------------------------------------
    void handleKnobEdit(int knob, float value) noexcept
    {
        const size_t vIdx = static_cast<size_t>(layerHold_ ? knob + 3 : knob);
        // Write rule: heel when raw expr <= 0.5, else toe.
        std::array<float, 6>& snap = (exprRaw_ <= 0.5f) ? heel_ : toe_;
        const float stored = snap[vIdx];

        const size_t k = static_cast<size_t>(knob);
        if (__builtin_expect(!knobEngaged_[k], 0))
        {
            // Pickup: engage within the 0.02 window, or when the physical
            // knob sweeps across the stored value between two edits.
            const bool near = fabsf(value - stored) <= kTakeoverWindow;
            const bool crossed =
                lastKnobValid_[k]
                && (lastKnobPos_[k] - stored) * (value - stored) <= 0.0f;
            if (near || crossed)
                knobEngaged_[k] = true;
        }
        lastKnobPos_[k]   = value;
        lastKnobValid_[k] = true;

        // Engaged path mirrors the DMM discipline: store only — the value is
        // applied (with smoothing where specified) at the control tick.
        if (knobEngaged_[k])
            snap[vIdx] = value;
    }

    void disengageKnobs() noexcept
    {
        knobEngaged_[0] = false;
        knobEngaged_[1] = false;
        knobEngaged_[2] = false;
        // lastKnobPos_ stays valid: the physical positions did not move.
    }

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    // [DTCM] hot path: both units run every sample (~5 x 4 KB delay lines
    // live inside as fixed members — the external working buffer is unused).
    TscCircuit tsc_;
    Dc2Circuit dc2_;

    // [DTCM] per-sample scratch: current-sample full/wet outputs.
    float tscFullL_ = 0.0f, tscFullR_ = 0.0f;
    float tscWetGL_ = 0.0f, tscWetGR_ = 0.0f;
    float dc2FullL_ = 0.0f, dc2FullR_ = 0.0f;
    float dc2WetL_  = 0.0f, dc2WetR_  = 0.0f;

    // [DTCM] previous-sample full mono outputs (crossfade series feeds).
    float prevTscFullMono_ = 0.0f;
    float prevDc2FullMono_ = 0.0f;

    // [DTCM] shared crossfade engine (zone changes + DC-2 toggles).
    Config targetCfg_{};
    Config oldCfg_{};
    int    xfadeCount_ = kXfadeSamples;   // >= kXfadeSamples → idle
    float  xfC_  = 1.0f, xfS_  = 0.0f;    // (g_out, g_in) rotation state
    float  xfCd_ = 1.0f, xfSd_ = 0.0f;    // per-sample rotation, set in init()

    // [CONTROL-RATE] morph + control state.
    int ctrlPhase_ = 0;
    std::array<float, 6> heel_ = kDefaultV;   // snapshot at expression heel
    std::array<float, 6> toe_  = kDefaultV;   // snapshot at expression toe
    std::array<float, 6> eff_  = kDefaultV;   // effective values (last tick)
    float exprRaw_ = 0.0f;                    // raw pedal (write rule / crossing)
    ParameterSmoother exprSm_;                // ~20 ms, runs at control rate
    ParameterSmoother rateSm_;                // v1 smoothing, control rate

    bool layerHold_  = false;   // false = base layer (v0..v2), true = hold (v3..v5)
    bool dc2Enabled_ = true;

    // Soft takeover: engaged at load (first writes adopt hardware positions,
    // DMM-style); disengaged on layer toggles and expression 0.5-crossings.
    bool  knobEngaged_[3]   = {true, true, true};
    bool  lastKnobValid_[3] = {false, false, false};
    float lastKnobPos_[3]   = {0.0f, 0.0f, 0.0f};

    // LED pulse state (counter runs unconditionally in processAudio).
    int ledCounter_ = 0;
};

static PatchImpl patch;

Patch* Patch::getInstance()
{
    return &patch;
}

// ---------------------------------------------------------------------------
// Smoke test (Session 5 verification steps). Desktop-only:
//   g++ -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra
//       -DENDLESS_DESKTOP_BUILD -DTRIDIMENSION_SMOKE
//       -I. -I./sdk -I./wdf -I./dsp
//       effects/PatchImpl_TriDimension.cpp -lm -o tridimension_smoke
// ---------------------------------------------------------------------------
#ifdef TRIDIMENSION_SMOKE
#include <cstdio>

namespace tridimension_smoke {

constexpr int   kFsI     = 48000;
constexpr float kSineAmp = 0.2512f;                          // -12 dBFS
constexpr float kW1k     = 6.2831853f * 1000.0f / 48000.0f;  // 48 samples/cycle

inline int g_passed = 0, g_failed = 0;

inline void verdict(bool ok, const char* what)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    (ok ? g_passed : g_failed)++;
}

// Feeds audio in 48-sample blocks. The 1 kHz tone spans exactly 48 samples
// per cycle, and every feed length below is a multiple of 48, so each call
// starts AND ends at phase zero — the input itself is always continuous
// across config changes; any step > 0.5 is the patch's fault.
struct Runner
{
    PatchImpl& p;
    float prevL = 0.0f, prevR = 0.0f;
    bool  prevValid = false;
    bool  finite  = true;
    float maxStep = 0.0f;

    // Returns the average stereo RMS over the fed span.
    float run(int numSamples, bool sine)
    {
        float acc = 0.0f;
        int   fed = 0;
        while (fed < numSamples)
        {
            float bufL[48], bufR[48];
            const int m = (numSamples - fed) < 48 ? (numSamples - fed) : 48;
            for (int i = 0; i < m; ++i)
            {
                const float x =
                    sine ? kSineAmp * std::sin(kW1k * static_cast<float>(fed + i))
                         : 0.0f;
                bufL[i] = x;
                bufR[i] = x;
            }
            p.processAudio(std::span<float>(bufL, static_cast<size_t>(m)),
                           std::span<float>(bufR, static_cast<size_t>(m)));
            for (int i = 0; i < m; ++i)
            {
                if (__builtin_expect(!std::isfinite(bufL[i]) || !std::isfinite(bufR[i]), 0))
                    finite = false;
                if (prevValid)
                {
                    const float dl = std::fabs(bufL[i] - prevL);
                    const float dr = std::fabs(bufR[i] - prevR);
                    const float d  = dl > dr ? dl : dr;
                    if (d > maxStep) maxStep = d;
                }
                prevL = bufL[i];
                prevR = bufR[i];
                prevValid = true;
                acc += 0.5f * (bufL[i] * bufL[i] + bufR[i] * bufR[i]);
            }
            fed += m;
        }
        return std::sqrt(acc / static_cast<float>(numSamples));
    }
};

constexpr int kPress = static_cast<int>(endless::ActionId::kLeftFootSwitchPress);
constexpr int kHold  = static_cast<int>(endless::ActionId::kLeftFootSwitchHold);

// 1) Six configurations: every zone with DC-2 on and off. 4800 samples of
//    silence then 4800 of 1 kHz at -12 dBFS each; finite + nonzero asserted
//    here, the global click gate at the end covers all transitions.
inline void testSixConfigs(PatchImpl& p, Runner& r)
{
    struct ZoneProbe { float knob; int zone; };
    constexpr ZoneProbe probes[3] = {{0.16f, 0}, {0.50f, 1}, {0.84f, 2}};
    for (const auto& z : probes)
    {
        p.setParamValue(2, z.knob);

        if (!p.smokeDc2Enabled()) p.handleAction(kPress);
        (void)r.run(4800, false);
        float rms = r.run(4800, true);
        char label[96];
        std::snprintf(label, sizeof label,
                      "zone %d reached (knob %.2f), DC-2 on: output nonzero (RMS %.4f)",
                      z.zone, static_cast<double>(z.knob), static_cast<double>(rms));
        verdict(p.smokeZone() == z.zone && rms > 1e-3f, label);

        p.handleAction(kPress);   // DC-2 off — crossfade during the silence
        (void)r.run(4800, false);
        rms = r.run(4800, true);
        std::snprintf(label, sizeof label,
                      "zone %d, DC-2 off: output nonzero (RMS %.4f)",
                      z.zone, static_cast<double>(rms));
        verdict(!p.smokeDc2Enabled() && rms > 1e-3f, label);

        p.handleAction(kPress);   // back on for the next zone
    }
}

// 2) Transition torture: continuous tone while walking every zone boundary
//    and toggling DC-2 mid-signal. Feeds are multiples of 48 → the input is
//    phase-continuous; the click monitor sees every crossfade.
inline void testTransitionsUnderSignal(PatchImpl& p, Runner& r)
{
    p.setParamValue(2, 0.16f); (void)r.run(2400, true);
    p.setParamValue(2, 0.50f); (void)r.run(2400, true);   // zone 0 → 1
    p.setParamValue(2, 0.84f); (void)r.run(2400, true);   // zone 1 → 2
    p.handleAction(kPress);    (void)r.run(2400, true);   // DC-2 off, mid-signal
    p.setParamValue(2, 0.16f); (void)r.run(2400, true);   // zone 2 → 0 jump, off
    p.handleAction(kPress);    (void)r.run(2400, true);   // DC-2 back on
    verdict(p.smokeZone() == 0 && p.smokeDc2Enabled(),
            "transition walk ends in zone 0 with DC-2 on");
}

// 3) Layer toggle, knob writes at expr = 0 and expr = 1, soft takeover.
inline void testLayersAndTakeover(PatchImpl& p, Runner& r)
{
    // expr to heel end (already 0.0 — no crossing) and settle.
    p.setParamValue(3, 0.0f);
    (void)r.run(4800, false);

    // Base-layer knob 1 at expr = 0 → writes heel[1]; knobs are engaged
    // (no layer toggle or 0.5-crossing has happened yet).
    p.setParamValue(1, 0.7f);
    (void)r.run(480, false);
    verdict(std::fabs(p.smokeEffective(1) - 0.7f) < 1e-3f,
            "knob write at expr=0 lands in heel (effective follows)");

    // expr to toe end: crossing 0.5 disengages all knobs.
    p.setParamValue(3, 1.0f);
    (void)r.run(4800, false);
    verdict(std::fabs(p.smokeEffective(1) - 0.5f) < 1e-2f,
            "expr=1 reads the toe snapshot (default 0.5)");

    // Disengaged, far from stored toe value 0.5 and NOT crossing it
    // (0.7 → 0.6 stays above 0.5): the edit must be ignored.
    p.setParamValue(1, 0.6f);
    (void)r.run(480, false);
    verdict(std::fabs(p.smokeEffective(1) - 0.5f) < 1e-2f,
            "disengaged knob edit ignored (soft takeover)");

    // Pickup within the 0.02 window, then drag to 0.9 → toe[1] = 0.9.
    p.setParamValue(1, 0.51f);
    p.setParamValue(1, 0.9f);
    (void)r.run(480, false);
    verdict(std::fabs(p.smokeEffective(1) - 0.9f) < 1e-2f,
            "knob re-engages in pickup window; toe write lands");

    // Hold action → hold layer; knob 0 now maps to v3 and is disengaged.
    p.handleAction(kHold);
    verdict(p.smokeHoldLayer(), "hold action latches the hold layer");
    p.setParamValue(0, 0.3f);   // |0.3 - 0.5| > 0.02, no cross → ignored
    (void)r.run(480, false);
    verdict(std::fabs(p.smokeEffective(3) - 0.5f) < 1e-2f,
            "hold-layer knob disengaged after layer toggle");
    p.setParamValue(0, 0.49f);  // within 0.02 of stored 0.5 → engages
    p.setParamValue(0, 0.8f);   // drag → toe[3] = 0.8
    (void)r.run(480, false);
    verdict(std::fabs(p.smokeEffective(3) - 0.8f) < 1e-2f,
            "hold-layer knob writes v3 after pickup");
    p.handleAction(kHold);      // back to base layer
    verdict(!p.smokeHoldLayer(), "second hold returns to base layer");
}

// 4) Full expression sweep toe → heel under signal: effective values must
//    interpolate monotonically between the snapshots and stay in range.
//    Snapshots at this point: v1 heel 0.7 / toe 0.9, v3 heel 0.5 / toe 0.8.
inline void testExpressionSweep(PatchImpl& p, Runner& r)
{
    p.setParamValue(3, 1.0f);
    (void)r.run(4800, true);
    float prev1 = p.smokeEffective(1);
    float prev3 = p.smokeEffective(3);
    bool mono = true, inRange = true;
    for (int k = 20; k >= 0; --k)
    {
        p.setParamValue(3, static_cast<float>(k) / 20.0f);
        (void)r.run(2400, true);   // 50 ms per step (20 ms smoothing tau)
        const float e1 = p.smokeEffective(1);
        const float e3 = p.smokeEffective(3);
        if (e1 > prev1 + 1e-4f || e3 > prev3 + 1e-4f) mono = false;
        if (e1 < 0.7f - 1e-3f || e1 > 0.9f + 1e-3f)   inRange = false;
        if (e3 < 0.5f - 1e-3f || e3 > 0.8f + 1e-3f)   inRange = false;
        prev1 = e1;
        prev3 = e3;
    }
    verdict(mono, "effective values interpolate monotonically over the sweep");
    verdict(inRange, "effective values stay inside [heel, toe]");
    verdict(std::fabs(p.smokeEffective(1) - 0.7f) < 1e-2f
                && std::fabs(p.smokeEffective(3) - 0.5f) < 1e-2f,
            "heel values reached at expr=0");
}

}   // namespace tridimension_smoke

int main()
{
    using namespace tridimension_smoke;
    std::printf("=== TriDimension smoke test (Session 5) ===\n");

    PatchImpl& p = *static_cast<PatchImpl*>(Patch::getInstance());
    p.init();
    Runner r{p};

    testSixConfigs(p, r);
    testTransitionsUnderSignal(p, r);
    testLayersAndTakeover(p, r);
    testExpressionSweep(p, r);

    // Global gates: cover EVERY sample fed above, including all zone/enable
    // crossfades and the expression-driven morphs.
    verdict(r.finite, "no NaN/Inf anywhere in the run");
    std::printf("max per-sample output step = %.4f (gate 0.5)\n",
                static_cast<double>(r.maxStep));
    verdict(r.maxStep < 0.5f, "no per-sample step > 0.5 (click check)");

    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
#endif   // TRIDIMENSION_SMOKE
