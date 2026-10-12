// Port of web/draw.js -- DRAW MODE: the user hands the tool one contact line and it
// sweeps ONE breakaway wall along it (`drawnLine`, `drawnWall`) -- plus the plugins'
// Draw entry, plugins/shared/engine/draw_entry.js `drawWall` (the part as a posed
// soup, seated and analyzed exactly as computeFins does, then one drawnWall).
//
// Geometry only. Reuses Prop/'s sweep and its three line-settling passes verbatim.
//
// This header deliberately includes neither Fins.hpp nor Prop/Raster.hpp (they both
// define `TriRange` differently), so Entry.cpp and Prop users can both include it.
#pragma once
#include "Math.hpp"
#include "Overhangs.hpp"
#include "PropConfig.hpp"
#include "Prop/Contact.hpp"
#include "Prop/Crest.hpp"
#include "Prop/Surface.hpp"
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// A hand-drawn wall may be much shorter than an auto wall (PROP.minSpan): 2 mm still
/// gives the three stations a sweep needs (local issue 011).
inline constexpr double DRAW_MIN_LEN = 2;

/// The contact polyline for a wall drawn from `a` to `b` (surface points, PRINT space).
/// Straight in XY; each station's height is the part surface nearest the drawn line's
/// own height there (`under`: only down-facing surfaces), then contourTop, lowerSag and
/// settleTop(0.25) pull the top to a clean `gap` below the part, each bounded to `band`.
/// nullopt (JS null) when a and b coincide in XY. `step` nullopt = PROP.stationStep.
std::optional<ContactLine> drawnLine(const Vec3d &a, const Vec3d &b, const SurfTris &tris,
                                     std::optional<double> step = std::nullopt, double band = INF, bool under = false);

/// drawnWall's `opts` (JS object; every field optional upstream).
struct DrawnWallOpts {
    bool under = false;                  // follow only down-facing surfaces (the fill pass's walls)
    bool plateOnly = false;              // never stand on the part (#218)
    bool brace = false;                  // ribs at the plate for a tall short wall (prop/brace.js)
    bool tines = false;                  // the grip comb -- only with `topo` set (JS `opts.tines && opts.topo`)
    std::optional<double> tineDensity;   // tineStepFor(opts.tineDensity)
    std::optional<double> layerHeight;   // crest layer (`|| PROP.tineH`) and tine height (`?? PROP.tineH`)
    const Topology *topo = nullptr;      // the part in topology form, for emitTines' bite direction
    Rot    rot = IDENTITY3;
    Offset offset;
};

/// drawnWall's result. JS returns one of:
///   { ok: false, reason }
///   { ok: true, tris, length, height, partAttached: true, tines, braces, top, floors }   part-attached
///   { ok: true, tris, length, height, partAttached, squat: true, tines, top }            squat fallback
///   { ok: true, tris, length, height, tines, braces, top, floors }                       plate wall
/// Fields a shape leaves undefined are false / nullopt here.
struct DrawnWall {
    bool        ok = false;
    std::string reason;                  // UTF-8, the site's words
    Tris        tris;                    // vertex triples, emission order
    IfaceTags   tags;                    // interface tags parallel to `tris` (Crest.hpp), padded to its size
    double      length = 0;              // XY length of the drawn line
    double      height = 0;              // tallest stand, (top - gap) - floor
    bool        partAttached = false;
    bool        squat = false;
    int         tines = 0;
    std::optional<int> braces;           // ribs built (not on a squat wall)
    ContactLine top;                     // the wall's contact line (surface-z stations)
    std::optional<std::vector<double>> floors;   // each station's bottom z (not on a squat wall)
};

/// Build one drawn breakaway wall from `a` to `b` (PRINT space) under the part `tris`
/// (whole part, seated, flat 9 per face) standing on `zBed`. When the interface crest is
/// on, `tris` becomes the crest's part (crestPart), as upstream does.
DrawnWall drawnWall(const Vec3d &a, const Vec3d &b, const std::shared_ptr<const SurfTris> &tris, double zBed = 0,
                    const DrawnWallOpts &opts = {});

// ---- plugins/shared/engine/draw_entry.js ------------------------------------------

/// draw_entry.js drawWall's options: fins_entry.js ENGINE_DEFAULTS merged with the
/// host's (only these are read; the rest of a computeFins options object is ignored).
struct DrawWallOptions {
    std::string material = "pla";
    bool        tines = true;
    double      tineDensity = 0;
    double      layerHeight = 0.2;
    bool        plateOnly = false;
    double      threshold = DEFAULT_THRESHOLD;
    std::string padStyle = "auto";       // validated (clearances), else unused
    std::string cutout = "none";         // CUT.pattern for the wall
};

/// draw_entry.js drawWall's result: { ok: true, triangles, offset, stats } or
/// { ok: false, reason }.
struct DrawWallResult {
    bool               ok = false;
    std::string        reason;
    std::vector<float> triangles;        // 9 per face, SEATED frame, float32 (seat.js flatten)
    Offset             offset;           // seated = input + offset
    struct {
        double length = 0, height = 0;
        int    tines = 0;
        bool   partAttached = false;
    } stats;
};

/// draw_entry.js drawWall(positions, a, b, options): the part as computeFins takes it
/// (posed soup, 9 per face) and the wall's ends in that same frame. Throws
/// std::invalid_argument on a non-finite endpoint, a bad soup or a bad option value.
DrawWallResult drawWall(const Soup &positions, const Vec3d &a, const Vec3d &b, const DrawWallOptions &options = {});

}} // namespace
