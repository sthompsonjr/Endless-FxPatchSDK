# Reference Library Manifest

> Session 1 creates `DC2_component_reference.md` in this directory. Once that file exists, it supersedes direct PDF reads for all DC-2 component values.

**Status: COMPLETE.** All 8 REQUIRED files are installed and verified. Session 1's pre-flight check against this directory should pass.

## Installed Files

| Canonical Name | Size | Pages | SHA-256 | Flag | Role |
|---|---|---|---|---|---|
| blueshift_documentation.pdf | 1,098,442 bytes | 17 | `ee8ec1a12a2c5257c13cd07cc001d0f11f266144456d1507a36d81720ea11f69` | REQUIRED | Aion FX Blueshift build doc: redrawn DC-2 schematic plus full BOM; working authority for DC-2 component values |
| BBSS2501411.pdf | 751,740 bytes | 8 | `6c13f60227c30824025973dcebbf9afebda3d7ad40ffb6b37e416609351c4854` | REQUIRED | Boss factory DC-2 schematic scan; cross-check |
| MN3207.pdf | 181,324 bytes | 5 | `3f2188db499992086eda09fa6b15d057adc05d1c79e1bbd24f9dd0765367afb9` | REQUIRED | BBD datasheet |
| panasonicmn3102.pdf | 158,562 bytes | 4 | `422bf10c82e62162a9aa0bc2a361266558f2c747e3cd1a5aaf1debbe8f673fde` | REQUIRED | Clock driver datasheet |
| philipsne570ne571sa571.pdf | 135,633 bytes | 12 | `8d7acdfd413c92bb2f351140026e75431163ab0359cf12efd32a7d733005982a` | REQUIRED | Compander product spec |
| philipsne570ne571sa571applicationnotes.pdf | 154,865 bytes | 10 | `b686710ac1ee1ca7716196943d991d31a8b1754f1c9e1d4b05aa9a0c87189b1b` | REQUIRED | AN174: compander gain and timing equations |
| Dytronics_TriStereo_Chorus_Manual.pdf | 2,629,126 bytes | 13 | `570b83004200ed3c7997ec924f710ef59a7d776c811377c91a7cf961c215cc67` | REQUIRED | UAD/Softube manual: TSC LFO modes and rate range |
| JH__Triple_Chorus.pdf | 2,329,572 bytes | 8 | `bce64b81a5538b7ff757476488044672a4d11cb583074d2bf60d52c901543aa5` | REQUIRED | Jurgen Haible Triple Chorus schematics; TSC structural analog |

All eight files verified: size > 10 KB, `file`/`pypdf` reports valid PDF, page count > 0.

## Missing (Optional) Files

| Canonical Name | Flag | Role | Status |
|---|---|---|---|
| DC2_OM.pdf | OPTIONAL | DC-2 owner's manual | NOT ATTACHED |
| DynoMyPiano.pdf | OPTIONAL | Italo De Angelis TSC architecture notes | NOT ATTACHED |
| Tri_Stereo_Chorus_Block_Diagram__freestompboxes_org.pdf | OPTIONAL | TSC block diagram (freestompboxes.org) | NOT ATTACHED |
