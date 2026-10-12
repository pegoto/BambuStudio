// Port of web/cutout.js -- WALL CUTOUTS (issue #34): holes through a breakaway wall
// to save filament.
//
// prop/sweep.js (sweep / sweepBetween) and fins/wedges.js (emitBlade) sweep each wall
// as per-station cross-sections and hand them here; cutWall either emits the wall
// with holes or returns false, and the caller emits it solid as before. Everything
// is in the wall's own (s, z) plane: s runs along the wall, z up. The pattern choice
// lives here (CUT.pattern), set by fins.js applyTunables from tunables.cutout.
#pragma once
#include "Math.hpp"
#include "PropConfig.hpp"
#include "Solids.hpp"
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// CUT -- the cutout tunables. Mutable module state in the JS (applyTunables writes
/// `pattern`); thread_local here. The reasoning behind each number is in cutout.js.
struct CutConfig {
    std::string pattern = "none"; // which holes to cut: 'none' or one of CUTOUT_PATTERNS
    double pitch = 8.0;      // target cell width along the wall, mm (one hole per cell)
    double web = 1.6;        // solid left between neighbouring holes, mm -- the strut width
    double rail = 1.2;       // solid kept above the bottom taper and below the top tip, mm
    double post = 2.0;       // solid end post at each end of the wall, mm
    double minHalf = 1.2;    // narrowest hole worth cutting (half-width), mm
    double minHole = 3.0;    // shortest hole worth cutting, mm
    double slope = 1.4;      // a hole's roof rises >= slope per unit across (~55deg): no bridging
    double maxSlope = 3.0;   // a hole taller than this (x half-width) is stacked into several
    double eps = 0.05;       // overlap into the neighbouring solid so the union is clean
    // LATTICE: small diamonds in staggered rows filling the whole wall between the bands.
    double latPitch = 6.0;   // centre-to-centre along a row, mm
    double latStrut = 1.2;   // strut width, measured square to the strut, mm
    double latSlope = 1.5;   // strut rise per unit across (~56deg), steeper than `slope`
    double roofMin = 1.0;    // a clipped lattice hole's roof may be as flat as 45deg, no flatter
    double minArea = 3.0;    // smallest lattice hole worth cutting, mm2
    double minSaved = 0.10;  // a wall whose holes open less of its face than this stays solid
};

extern thread_local CutConfig CUT;

/// ['none', ...Object.keys(HOLES), 'lattice'].
inline constexpr std::array<const char *, 5> CUTOUT_PATTERNS = {"none", "diamond", "triangle", "arch", "lattice"};
/// CUTOUT_PATTERNS.includes(name)
bool isCutoutPattern(const std::string &name);

/// One hole of holesFor: half-width, bottom, top (z relative to the cell's floor).
struct CutHole { double a, z0, z1; };

/// holesFor(kind, w, H): the holes that fit a cell `w` wide and `H` tall, or {} when
/// none does. Tall cells stack holes with a `web` between them.
std::vector<CutHole> holesFor(const std::string &kind, double w, double H);

/// One element of cutWall's `st`: the per-station wall data sweep / sweepBetween /
/// emitBlade build. Optional fields are the ones only some callers set.
struct CutStation {
    Vec3d  p = Vec3d::Zero();     // the station's line point; only p[0], p[1] are read
    double sx = 0, sy = 0;        // horizontal unit vector across the wall
    double top = 0;               // the section's top (zTop)
    double ztip = 0;              // where the top taper begins (zt)
    std::optional<double> holeTop; // holes stop here instead of ztip (sweep, sweepBetween)
    std::optional<double> tipHalf; // the top's half-width; default wall.tip / 2
    double bot = 0;               // the section's bottom
    double botTip = 0;            // top of the bottom flange / taper
    bool   taperBot = false;      // a welded tip-wide bottom (sweepBetween, footGap 0)
    std::optional<double> botN, botP; // a tilted lifted bottom (sweepBetween)
};

/// cutWall's `wall` argument: { th, tip, minStations } (PROP for prop walls,
/// { th: PERP.th, tip: PERP.th, minStations: 3 } for wedge blades).
struct CutWallDims {
    double th = 1.0;
    double tip = 0.6;
    int    minStations = 3;
};

/// cutWall(st, full, out, wall): emit `st` as a wall with CUT.pattern holes and return
/// true, or return false (nothing emitted) to have the caller emit its solid ribbon.
/// `full` is the caller's solid sections (one per station), reused for the end posts.
bool cutWall(const std::vector<CutStation> &st, const std::vector<Section> &full, Tris &out, const CutWallDims &wall);
/// cutWall(st, wall, out, PROP), as prop/sweep.js calls it.
inline bool cutWall(const std::vector<CutStation> &st, const std::vector<Section> &full, Tris &out, const PropConfig &prop) {
    return cutWall(st, full, out, CutWallDims{prop.th, prop.tip, prop.minStations});
}

}} // namespace Slic3r::SupportFins
