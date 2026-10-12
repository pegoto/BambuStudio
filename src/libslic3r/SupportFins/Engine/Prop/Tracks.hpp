// Port of web/prop/tracks.js -- where the auto-placer ("Suggest") puts walls under a
// region: `splitRegion` cuts it into locally-straight sub-patches, `patchTracks` lays
// straight parallel tracks across each, and `tubeLine` gives a curved region one wall
// along its lowest line. The reasoning (and the measurements behind every number) is
// in the upstream file; only what a reader of the C++ needs is repeated here.
//
// The buildProps `patch` record (PropPatch) lives here because splitRegion makes the
// first ones; prop.js, lattice.js and rival.js add fields to the same objects.
#pragma once
#include "../Math.hpp"
#include "../Overhangs.hpp"
#include "../PropConfig.hpp"
#include "Contact.hpp"
#include "Surface.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace Slic3r { namespace SupportFins {

struct Rival;   // Rival.hpp: a small tube's race record

/// One buildProps `patch`: splitRegion's `{ faces, area }`, decorated by its callers.
/// Fields JS leaves `undefined` are empty / -1 / false here.
///   splitRegion     faces, area
///   buildProps      region, tris (p.region = ri; p.tris = regionTris), and for a tube
///                   region's patch: lines, smallTube
///   strutPatches    strut, axis, reach, lines, smallTube (lattice.js)
///   tubeRivalry     rival, region, tris on a small tube's queued split patches
/// JS patches are objects shared by reference (a smallTube's patches are the very
/// objects later queued on `patches`, and rival.js compares them by identity), so
/// the containers hold PatchPtr.
struct PropPatch {
    std::vector<int> faces;                          // topology face indices
    double area = 0;                                 // sum of topo.area over faces
    int region = -1;                                 // index into result.regions
    std::shared_ptr<const SurfTris> tris;            // the WHOLE region's seated triangles (regionTris)
    std::shared_ptr<const SurfTris> reach;           // a lattice strut's own triangles (JS `reach`)
    std::optional<std::vector<ContactLine>> lines;   // a tube's (or raster region's) tracks, built already
    std::optional<std::vector<std::shared_ptr<PropPatch>>> smallTube;   // a small tube's split patches
    bool strut = false;                              // a lattice strut (lattice.js)
    std::optional<Vec2d> axis;                       // patchTracks run direction override (a strut's long axis)
    std::shared_ptr<Rival> rival;                    // set by tubeRivalry.tubeDone
};
using PatchPtr = std::shared_ptr<PropPatch>;

/// The `support` argument of patchTracks / rasterTracks: JS `{ topo, rot, offset }`,
/// the seated part insidePart probes.
struct Support {
    const Topology *topo = nullptr;
    Rot    rot = IDENTITY3;
    Offset offset;
};

/// patchTracks' result: the kept tracks (a JS array of [x, y, z] station lists) plus
/// the array's `.spacing` property, the widest gap between adjacent rows (0 for one
/// row). nullopt = JS undefined, which the two early `return []` paths leave it.
struct Tracks : std::vector<ContactLine> {
    std::optional<double> spacing;
};

/// splitRegion(topo, faces, rot): the region's faces grown into locally-straight
/// sub-patches (each candidate judged against its SEED's normal, cos(splitAgreeDeg)),
/// seeds largest face first (stable). Each result carries faces + area only.
std::vector<PropPatch> splitRegion(const Topology &topo, const std::vector<int> &faces, const Rot &rot);

/// tubeLine(topo, faces, rot, pts, regionTris, step): the lowest-line track(s) of a
/// CURVED region, or nullopt (JS null) when the region is not a tube. A present but
/// EMPTY vector is JS `[]` (a tube whose every piece is under minStations), which
/// prop.js treats as "no tube" through `tube?.length`. `pts` are the region's
/// vertices + face centroids, `regionTris` its seated triangles (faces order).
std::optional<std::vector<ContactLine>> tubeLine(const Topology &topo, const std::vector<int> &faces, const Rot &rot,
                                                 const std::vector<Vec3d> &pts, const SurfTris &regionTris,
                                                 double step = PROP.stationStep);

/// patchTracks(pts, patchTris, step, support, span, axis): straight parallel tracks
/// across one patch, sampled on its surface, rows `span` apart, split where the patch
/// does not cover them. support nullptr = JS null; axis nullopt = JS null.
Tracks patchTracks(const std::vector<Vec3d> &pts, const SurfTris &patchTris, double step = PROP.stationStep,
                   const Support *support = nullptr, double span = PROP.maxUnsupportedSpan,
                   const std::optional<Vec2d> &axis = std::nullopt);

// ---- module-internal in the JS; exposed for tests ---------------------------------

/// tubeLine's per-face [normal, area] in print space, faces order (JS `rn`).
using AreaNormals = std::vector<std::pair<Vec3d, double>>;

/// edgePoints(tris, pts, gap): `pts` plus points every <= gap mm along every edge.
std::vector<Vec3d> edgePoints(const Soup &tris, const std::vector<Vec3d> &pts, double gap);

/// convexAbout(rn, regionTris, cx, cy, ux, uy): is this curved band the underside of a
/// small convex, two-sided tube about the line through (cx, cy) along (ux, uy)?
bool convexAbout(const AreaNormals &rn, const Soup &regionTris, double cx, double cy, double ux, double uy);

/// rowOffsets' / lowEdges' / freeEdge's `probe`: JS
/// `{ support, patchTris, ux, uy, vx, vy, uLo, uHi }`.
struct RowProbe {
    const Support  *support = nullptr;
    const SurfTris *patchTris = nullptr;
    double ux = 0, uy = 0, vx = 0, vy = 0, uLo = 0, uHi = 0;
};

/// rowOffsets' result: the row positions across the patch (v) plus `.spacing`.
struct RowOffsets : std::vector<double> {
    double spacing = 0;
};

/// lowEdges' row classes (JS strings 'none' | 'ok' | 'low' | 'part').
enum class RowClass : uint8_t { None, Ok, Low, Part };

/// One low edge: the row position nearest a low band, `dir` pointing away from it.
struct LowEdge {
    double v = 0;
    int    dir = 0;
};

/// lowEdges' result: the edges (JS array) plus its `.low(v)` closure, which reads the
/// sampled classes `cls` (one per `step` from `vLo`).
struct LowEdges : std::vector<LowEdge> {
    std::vector<RowClass> cls;
    std::vector<double>   vs;      // the sampled row positions (JS `vs`)
    double vLo = 0, step = 0.25;
    bool low(double v) const;
};

/// rowOffsets(vLo, vHi, rowSpan, probe): probe nullptr = JS null/false.
RowOffsets rowOffsets(double vLo, double vHi, double rowSpan, const RowProbe *probe);

/// lowEdges(probe, vLo, vHi)
LowEdges lowEdges(const RowProbe &probe, double vLo, double vHi);

/// lowEdgeRows(rows, edges, near)
std::vector<double> lowEdgeRows(const std::vector<double> &rows, const LowEdges &edges, double near);

/// freeEdge(probe, vEdge, dir): does the patch end in air along v = vEdge?
bool freeEdge(const RowProbe &probe, double vEdge, int dir);

}} // namespace
