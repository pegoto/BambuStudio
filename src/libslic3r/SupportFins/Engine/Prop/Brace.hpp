// Port of web/prop/brace.js -- BRACES: buttresses for a short wall that stands tall on
// the plate (local issue 036). A wall over BRACE.aspect x its length gets up to four
// ribs at the plate: one off each end along its line, one off each face at its middle.
// A rib is a triangle in its own vertical plane, the wall's thickness, rising
// BRACE.rise of the wall's height at the stem and running out to the plate
// PROP.footRatio x height away, on a flange strip the foot's height and footMin either
// side. Built only from a station whose floor is the plate, only where the part stays
// sideClear off it, and only if `clear(tris)` passes.
#pragma once
#include "../Math.hpp"
#include "../PropConfig.hpp"
#include "Contact.hpp"
#include "Surface.hpp"
#include <functional>

namespace Slic3r { namespace SupportFins {

struct BraceConfig {
    double aspect   = PropConfig{}.maxShortAspect; // braced when height > this x length (JS: PROP.maxShortAspect at load)
    double rise     = 0.5;   // rib height at the stem, as a share of the wall's height there
    double onPlate  = 0.5;   // mm: a station whose floor is this close to the bed stands on the plate
    double minReach = 1.5;   // mm: a rib shorter than this is not tried (blocked ribs retry at half reach)
    double step     = 0.25;  // mm: part clearance sampled this finely under a rib
};
extern thread_local BraceConfig BRACE;

/// Brace the wall whose contact line is `top` (stations [x, y, z], z the part above)
/// and whose bottom at station i is `floorZ(i)`. Appends the ribs' triangles to `out`
/// and returns how many were built (0 when the wall is stocky enough or nothing fits).
/// `clear(ribTris)` is the caller's check against supports already built (empty: true).
int braceWall(const ContactLine &top, const std::function<double(int)> &floorZ, const SurfTris &tris,
              double zBed, Tris &out, const std::function<bool(const Tris &)> &clear = nullptr);

}} // namespace
