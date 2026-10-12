// Port of web/prop.js -- PROP mode, the breakaway wall that stands UNDER an overhang:
// `buildProps` (the auto-placer's whole pass over a part), `buildShortWalls` (fins/
// shortwalls.js's last-resort pass), `noProps`, `coverRowSpan`.
//
// The JS file also re-exports everything under web/prop/ so fins.js, draw.js and ui/
// import from one place. C++ callers include the leaf headers directly; this header
// pulls in the ones whose types appear in buildProps' contract (Prop/Raster.hpp holds
// the prop record, the result and the raster veto; Prop/Rival.hpp the skipped counters
// and the region set; Prop/Tracks.hpp the patches).
#pragma once
#include "Math.hpp"
#include "Overhangs.hpp"
#include "PropConfig.hpp"
#include "Prop/Raster.hpp"
#include "Prop/Rival.hpp"
#include "Prop/Tracks.hpp"
#include <optional>

namespace Slic3r { namespace SupportFins {

/// buildProps' prop record. prop.js builds three shapes into one array -- the plate
/// wall, the part-attached wall (buildPartAttached's prop spread in) and the squat wall
/// (buildSquatBed's descriptor spread in) -- and every field any of them carries is a
/// member of PropRecord (Prop/Raster.hpp, where raster.js first reads it back). The
/// fields fins/fill.js, fins/shortwalls.js and fins.js add on their own copies (fill,
/// joined, short) are theirs, not prop.js's.
using Prop = PropRecord;

/// mm row pitch at the sparsest coverage (0). Deliberately wider than
/// maxUnsupportedSpan: the mid-slider (0.5) is the structural anti-sag cap.
inline constexpr double COVER_SPARSE_SPAN = 30.0;

/// Wide-face row pitch for a coverage setting: 0.5 -> maxUnsupportedSpan, 0 ->
/// COVER_SPARSE_SPAN, 1 -> half the cap. Monotonic. NaN propagates (JS Math.min/max).
double coverRowSpan(double coverage);

/// buildProps' `opts`. JS reads (and only these):
///   opts.step          station step (?? PROP.stationStep)        -> step
///   opts.coverage      0 sparse .. 1 dense (?? 0.5)              -> coverage
///   opts.tines         tines only when `=== true`                -> tines
///   opts.tineDensity   tineStepFor(opts.tineDensity)             -> tineDensity
///   opts.layerHeight   tine height (?? PROP.tineH)               -> layerHeight
///   opts.plateOnly     `=== true`                                -> plateOnly
///   opts.raster        `=== false` runs the normal pass alone    -> raster
///   opts.rasterVeto    fins/wedges.js wedgeVeto in auto mode     -> rasterVeto
///   opts.onlyRegions   buildShortWalls: a Set of region indices  -> onlyRegions
///   opts.shortWalls    buildShortWalls: minSpanFor's `short`     -> shortWalls
/// nullopt is JS undefined (the default applies at call time, from the current PROP).
struct PropOpts {
    std::optional<double> step;
    std::optional<double> coverage;
    bool                  tines = false;
    std::optional<double> tineDensity;
    std::optional<double> layerHeight;
    bool                  plateOnly = false;
    bool                  raster = true;
    RasterVeto            rasterVeto;     // empty: no veto (JS undefined)
    std::optional<RegionSet> onlyRegions; // nullopt: every region (JS `?? null`)
    bool                  shortWalls = false;
};

/// What buildProps returns when it builds nothing (a part seated on a point).
/// JS: { triangles: [], props: [], served: 0, volume: 0, skipped: all zero } -- the
/// fields it leaves undefined (servedRegions, tines, sagRisk) read as their defaults.
PropsResult noProps();

/// Build a breakaway prop under every overhang region that can take one: the normal
/// pass (tube line or split patches per region), then -- unless opts.raster is false --
/// a raster pass over the regions it left partly bare, each region keeping the pass
/// that holds more of it (Prop/Raster.hpp withRaster). Triangles are vertex triples in
/// emission order; `tags` (parallel, Crest.hpp) are the interface crest/tine tags.
PropsResult buildProps(const Topology &topo, const AnalyzeResult &result, const Rot &rot, const PropOpts &opts = {});

/// The last-resort pass (fins/shortwalls.js): the normal placement for `regions` only,
/// with the short-wall floor (minSpanFor `short`). No raster race.
/// JS buildShortWalls(topo, result, rot, opts, regions) =
///   buildPass(topo, result, rot, { ...opts, onlyRegions: regions, shortWalls: true }, null).
PropsResult buildShortWalls(const Topology &topo, const AnalyzeResult &result, const Rot &rot, const PropOpts &opts,
                            const RegionSet &regions);

}} // namespace
