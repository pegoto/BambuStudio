// Port of web/fins/pad.js -- the bed pad: a thin breakaway footprint under a part whose
// bed contact is too small to hold it. `PAD` holds the style knobs, `buildPad` builds
// the conforming oval ('sure') or hands off to brimPad (the one-layer brim-style pad
// cut around the part's first-layer outline: 'light' and 'custom').
#pragma once
#include "../FinConfig.hpp"
#include "../Math.hpp"
#include "../Prop/Surface.hpp"
#include <optional>
#include <string>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// JS PAD. Mutable module state (fins.js applyTunables writes grab / style / custom);
/// thread_local like FIN and PROP.
struct PadConfig {
    double cell = 1.2;        // mm; radial vertex spacing across the conforming oval disc
    // mm the pad rises PAST the part underside to bite in near the contact (a light
    // tack). May be NEGATIVE: PETG sets a gap (-0.1); `conform` floors every column at
    // 0.05 so a gap pad stays a valid watertight solid and still kisses the resting edge.
    double grab = 0.05;
    // 'auto'   -- 'light', except on a small foot (first-layer outline under
    //             minGripOutline) where it builds 'sure' instead.
    // 'light'  -- the brim-style pad: one layer, brimGap off the part.
    // 'sure'   -- the conforming pad: padH thick, tacked `grab` into the part.
    // 'custom' -- the brim-style mesh with every number the user's (custom).
    std::string style = "auto";
    struct Custom { double h = 0.5, gap = 0.0, grip = 0.05, margin = 4.0; } custom;
    // mm off the part's first-layer outline. Slicers close any slice gap under
    // 2 x slice_closing_radius (0.049) = 0.098mm, so it needs the margin over 0.1.
    double brimGap = 0.12;
    // mm of first-layer outline below which Light can't grip: Light becomes Sure hold.
    double minGripOutline = 20.0;
    double brimCell = 0.1;    // mm mesh spacing of the brim-style pad
};

extern thread_local PadConfig PAD;

/// buildPad's / brimPad's returned record (JS { r1, r2, cells, height, points, oval,
/// style, autoSure?, smallFoot, outline }).
struct PadResult {
    double      r1 = 0, r2 = 0;
    int         cells = 0;
    double      height = 0;
    int         points = 0;           // contact.length
    bool        oval = true;
    std::string style;                // sure | light | custom
    std::optional<bool> autoSure;     // set on the 'sure' pad only (undefined on the brim-style ones)
    bool        smallFoot = false;
    double      outline = 0;          // first-layer outline length, mm
};

/// buildPad(contact, partTris, out, layerH = FIN.tineH): the pad under the bed contact
/// points `contact` (seated x, y), conformed to the seated part `partTris`. Appends its
/// triangles (vertex triples) to `out` in emission order. nullopt (JS null) when there
/// are fewer than 3 contact points. A non-finite or <= 0.05 layerH falls back to
/// FIN.tineH, as in the JS.
std::optional<PadResult> buildPad(const std::vector<Vec2d> &contact, const SurfTris &partTris, Tris &out,
                                  double layerH = FIN.tineH);

}} // namespace
