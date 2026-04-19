#pragma once

namespace power_puff_params {

    static constexpr int kNumVariants = 5;

    // Variant indices — each maps to a distinct circuit topology and tone stack
    static constexpr int kVariantTriangle  = 0;  // V1  BC549C   R8=33k R5=33k C9=4nF notch≈763Hz
    static constexpr int kVariantRamsHead  = 1;  // V2  BC549C   R8=39k R5=22k C9=4nF notch≈859Hz
    static constexpr int kVariantOpAmp     = 2;  // V4  LM741    R8=39k R5=22k C9=4nF notch≈859Hz
    static constexpr int kVariantCivilWar  = 3;  // V7  2N3904   R8=20k R5=22k C9=4nF notch≈1199Hz
    static constexpr int kVariantNYC       = 4;  // V9  2N5088   R8=39k R5=22k C9=4nF notch≈859Hz

    static constexpr int kKnobVariant = 0;
    static constexpr int kKnobTone    = 1;
    static constexpr int kKnobSustain = 2;

    static constexpr int   kControlRateDivider = 32;
    static constexpr float kVariantScale        = 4.9999f;

} // namespace power_puff_params
