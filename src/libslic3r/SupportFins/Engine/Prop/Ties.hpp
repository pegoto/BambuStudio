// Port of web/prop/ties.js -- TIES: a zigzag of struts between two neighbouring walls
// (local issue 036): a Warren truss in the vertical plane through both walls, every
// strut rising TIE.slope (55deg), each starting on the plate or on the strut before
// it. Struts stop under zMax; each end runs TIE.into past its wall's centreline. Built
// only where the part stays sideClear off every strut.
#pragma once
#include "../Math.hpp"
#include "../PropConfig.hpp"
#include "Surface.hpp"

namespace Slic3r { namespace SupportFins {

struct TieConfig {
    double slope;            // rise per run: tan(55deg), steeper than the 45deg a printer holds
    double thick = 1.2;      // mm: a strut's depth, measured straight up
    double minSpan = 2.0;    // mm: closer than this the two walls' feet already meet
    double maxSpan = 12.0;   // mm: farther than this a strut is a wall of its own
    double minHeight = 8;    // mm: a wall shorter than this stands without help
    double rise = 0.7;       // ties stop at this share of the lower wall's height
    int    perWall = 2;      // ties a wall takes part in, nearest neighbours first
    double step = 0.25;      // mm: part clearance sampled this finely along a strut
    double into = 0.35;      // mm: a strut's end runs this far past each wall's centreline
    TieConfig();
};
extern thread_local TieConfig TIE;

/// The tie between plan points `a` (on wall A's line) and `b` (on wall B's), both
/// standing on the plate at zBed, under `zMax` (the lower wall's limit). Appends the
/// struts to `out` and returns how many, or 0 (out untouched) when none fits.
int tieWalls(const Vec2d &a, const Vec2d &b, double zMax, const SurfTris &tris, double zBed, Tris &out);

}} // namespace
