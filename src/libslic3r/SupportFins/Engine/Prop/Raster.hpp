// Port of web/prop/raster.js -- RASTER placement: walls across a region's WHOLE
// footprint, the way a slicer's grid support stands, raced against the patch/tube
// placement region by region. `rasterTracks` lays the tracks, `rasterRest` hands a
// track's leftover pieces back to the loop, and `withRaster` (buildProps' second half)
// runs the raster pass over the regions the normal pass left partly bare and keeps,
// per region, whichever pass holds more overhang area (by a margin, and never raising
// the region's lowest grip past gripRise). The measurements behind every number are in
// the upstream file.
//
// buildProps' prop record (PropRecord) and result (PropsResult) live here: raster.js
// is the first ported module that reads them back (prop.js will build them).
#pragma once
#include "../Math.hpp"
#include "../Overhangs.hpp"
#include "../PropConfig.hpp"
#include "Clearance.hpp"
#include "Contact.hpp"
#include "Crest.hpp"
#include "Rival.hpp"
#include "Surface.hpp"
#include "Tines.hpp"
#include "Tracks.hpp"
#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// A raster region replaces the normal one only if it holds RACE_REL more of the normal
/// pass's held area, and at least RACE_ABS mm2.
inline constexpr double RACE_REL = 0.10;
inline constexpr double RACE_ABS = 5.0;
/// A region the normal pass already holds this much of (own faces) is not rebuilt.
inline constexpr double RASTER_SKIP_HELD = 0.95;
/// A swap may not raise a region's lowest grip by more than GRIP_RISE mm, or
/// GRIP_RISE_REL of the grip's height, whichever is more.
inline constexpr double GRIP_RISE = 0.1;
inline constexpr double GRIP_RISE_REL = 0.05;
double gripRise(double z);

/// rasterTracks(regionPts, regionTris, partTris, step, rowSpan, support): parallel
/// straight tracks at `rowSpan` across the region's whole footprint (patchTracks on the
/// region), each cut where floorLine's plate/part class changes; pieces under
/// minStations dropped. `.spacing` is patchTracks' (nullopt = JS undefined).
Tracks rasterTracks(const std::vector<Vec3d> &regionPts, const SurfTris &regionTris, const SurfTris &partTris,
                    double step, double rowSpan, const Support *support);

/// One piece rasterRest returns: the stations (JS line.slice(a, b)) plus `from`, the
/// index its first station had in `line`.
struct RasterPiece : ContactLine {
    int from = 0;
};

/// rasterRest(line, run): the pieces of `line` either side of `run` ([a, b), longestRun's),
/// each at least minStations long, before first.
std::vector<RasterPiece> rasterRest(const ContactLine &line, const Run &run);

/// [first, end) into a triangle list (JS [s, e]).
using TriRange = std::array<size_t, 2>;

/// buildProps' prop record: every field prop.js writes (plate, part-attached and squat
/// walls alike) and raster.js / rival.js / fins.js read. JS `undefined` fields are
/// nullopt / false / empty. The plate wall writes region, tines, caps, grip, span,
/// height, area, stations, trimmed, volume, line, id, kind, triRanges; a part-attached
/// wall spreads PartAttachedProp (+ partAttached, inBore); a squat wall spreads
/// SquatWall (+ squat, triRange) and has no `trimmed`. raceRegions adds `raster`.
struct PropRecord {
    int    region = -1;
    int    tines = 0;
    std::optional<std::array<size_t, 2>> caps;   // [c0, c1) into TINECAP; nullopt with the capture off
    double grip = INF;                           // lowest tine bottom (emitTines grip.z); INF: no tines (JS `?? Infinity`)
    double span = 0;
    double height = 0;
    double area = 0;
    int    stations = 0;
    std::optional<int> trimmed;                  // line.length - stations (not on a squat wall)
    double volume = 0;
    ContactLine line;                            // the wall top: surface z - gap
    int    id = 0;
    std::string kind = "prop";
    std::vector<TriRange> triRanges;
    bool   partAttached = false;
    bool   inBore = false;
    bool   squat = false;
    std::optional<TriRange> triRange;            // a squat wall's own (SquatWall::triRange)
    bool   raster = false;                       // raceRegions: built by the raster pass
};

/// buildProps' / buildPass' result. `tags` rides with `triangles` (interface tags,
/// Crest.hpp). `sagRegions` is the raster pass's only (JS spreads it in when `raster`);
/// `rasterRegions` raceRegions' only.
struct PropsResult {
    Tris      triangles;
    IfaceTags tags;
    std::vector<PropRecord> props;
    Skipped   skipped;
    int       served = 0;
    std::vector<int> servedRegions;
    int       tines = 0;
    bool      sagRisk = false;
    std::optional<RegionSet> sagRegions;
    double    volume = 0;
    std::optional<int> rasterRegions;
};

/// opts.rasterVeto(q, normalProps): true drops raster wall `q` before the race (auto
/// mode: fins/wedges.js wedgeVeto). An empty function = no veto.
using RasterVeto = std::function<bool(const PropRecord &q, const std::vector<PropRecord> &normalProps)>;
/// buildProps' own pass builder for a set of regions (JS `(regions) => buildPass(..., regions)`).
using RasterPass = std::function<PropsResult(const RegionSet &regions)>;

/// withRaster(topo, result, rot, opts, normal, capFrom, rasterPass): buildProps' second
/// half. Runs `rasterPass` over the regions `normal` left partly bare and races them;
/// returns `normal` itself (a copy) when nothing swaps. `rasterVeto` is opts.rasterVeto.
/// The tine capture (TINECAP, Tines.hpp) ends up holding the kept walls' tines only,
/// after whatever it held before buildProps began (`capFrom`); the raster pass and the
/// veto run with a fresh capture swapped in.
PropsResult withRaster(const Topology &topo, const AnalyzeResult &result, const Rot &rot,
                       const RasterVeto &rasterVeto, const PropsResult &normal, size_t capFrom,
                       const RasterPass &rasterPass);

// ---- module-internal in the JS; exposed for tests ---------------------------------

/// overhangFaces' per-face record (JS [cx, cy, cz, area, region]).
struct OverFace {
    double cx = 0, cy = 0, cz = 0, area = 0;
    int    region = 0;
};
/// JS arrays of prop records hold references; these hold pointers into the pass results.
using PropRefs = std::vector<const PropRecord *>;

/// Overhang faces seated once (result.offset), regions in order, faces in order.
std::vector<OverFace> overhangFaces(const Topology &topo, const AnalyzeResult &result, const Rot &rot);
/// Overhang area of `faces` held by wall tops `tops` (within maxUnsupportedSpan / 2 in
/// plan, 0-1.5 mm below the face, -0.05 slack).
double heldBy(const std::vector<OverFace> &faces, const std::vector<Vec3d> &tops);
/// Every wall's top line, concatenated.
std::vector<Vec3d> wallTops(const PropRefs &qs);
/// Half-width of a wall's footprint: squat brim, part-attached th / 2, else footFor(height).
double halfFoot(const PropRecord &q);
/// The lowest grip of a set of walls; INF for none.
double gripZ(const PropRefs &qs);
/// Closest approach of two walls' centrelines in plan; INF if either is empty.
double planGap(const ContactLine &l1, const ContactLine &l2);
/// The regions worth a raster pass (insertion order = region order).
RegionSet rasterWanted(const std::vector<OverFace> &faces, const AnalyzeResult &result, const PropsResult &a);
/// Every overhang face within 2 x maxUnsupportedSpan of region ri's face-centroid bbox.
std::vector<OverFace> nearRegion(const std::vector<OverFace> &faces, int ri);
/// The normal walls `qa` a swapped region keeps, or nullopt (JS null) when the swap
/// must not happen.
std::optional<PropRefs> keepers(const PropRefs &qa, const PropRefs &qb, const std::vector<OverFace> &near);
/// Per region keep `a` or `b` (+ keepers); buildProps' shape with rasterRegions set.
PropsResult raceRegions(const std::vector<OverFace> &faces, const AnalyzeResult &result,
                        const PropsResult &a, const PropsResult &b);

}} // namespace
