// Port of web/inside.js -- is a point inside the part? Ray parity along +x, with a
// YZ bucket grid so it is cheap enough to call per probe; plus nearest-surface and
// exact solid-to-part clearance measurements over the same grid.
//
// FRAMES. Every function takes (topo, rot, offset) the way the JS does:
//   mesh frame   = topo.pos, the soup the Topology was built from (unrotated, unseated;
//                  what analyze(topo, threshold, rot) rotates).
//   print frame  = seated frame = rot * mesh + offset (offset = analyze(...).offset).
// Query points, solids and results are in the PRINT frame; the grid is built once in
// the MESH frame (orientation-independent) and queries are rotated back into it
// (inverse of an orthonormal rot = its transpose). With computeFins' rot = IDENTITY3
// the two frames differ by the offset only.
//
// CACHING. The grid (JS topo._insideGrid) and solidClearance's per-face generation
// stamp (topo._scSeen / topo._scGen) live on the Topology as mutable members (see
// Overhangs.hpp), so these take const Topology&. Not thread-safe on one Topology,
// exactly as the JS is single-threaded.
#pragma once
#include "Overhangs.hpp"
#include <cstdint>
#include <optional>
#include <vector>

namespace Slic3r { namespace SupportFins {

inline constexpr int    INSIDE_GRID = 64;     // JS GRID: buckets per axis
inline constexpr double INSIDE_EPS = 1e-9;    // JS EPS

/// buildGrid(topo)'s result (JS { start, items, cell, minY, minZ, sy, sz }): every
/// triangle bucketed by its (y, z) extent in the MESH frame, as CSR.
struct InsideGrid {
    std::vector<int32_t> start;   // GRID*GRID + 1 (JS `counts`)
    std::vector<int32_t> items;   // face indices
    double minY = 0, minZ = 0, sy = 0, sz = 0;
    /// JS grid.cell(y, z): the bucket of a mesh-frame (y, z). NaN in -> NaN out (JS
    /// propagates it through Math.min/max), so it is returned as a double.
    double cell(double y, double z) const;
};

/// buildGrid(topo) -- internal in the JS, exposed for tests. Cached on topo._insideGrid.
const InsideGrid &buildGrid(const Topology &topo);

/// nearestPart's { d, cosUp }.
struct NearestHit {
    double d = 0;       // distance to the closest part surface point
    double cosUp = 0;   // PRINT-space z share of the direction query -> surface (0 when d <= EPS)
};

/// nearestPart(topo, rot, offset, qx, qy, qz, rMax = 0.45): the part's closest surface
/// point within rMax of the PRINT-frame point (qx, qy, qz), or nullopt (JS null) if
/// nothing is closer than rMax (strictly). Grid broad phase; no dedup.
std::optional<NearestHit> nearestPart(const Topology &topo, const Rot &rot, const Offset &offset,
                                      double qx, double qy, double qz, double rMax = 0.45);

/// solidClearance's { d, cosUp, x, y, z }.
struct SolidHit {
    double d = 0;         // minimum triangle-to-triangle distance (0 when they cross)
    double cosUp = 0;     // PRINT-space z share of the approach solid -> part (0 when d <= EPS)
    double x = 0, y = 0, z = 0;   // the closest point on the SOLID, PRINT frame
};

/// solidClearance(topo, rot, offset, verts, rMax = 0.45): exact minimum clearance between
/// a solid (PRINT-frame triangle soup, three vertices per triangle; size must be a
/// multiple of 3, else std::invalid_argument -- the JS throws a TypeError) and the part, or nullopt if it is at
/// least rMax everywhere. Advances topo._scGen.
std::optional<SolidHit> solidClearance(const Topology &topo, const Rot &rot, const Offset &offset,
                                       const Tris &verts, double rMax = 0.45);

/// insidePart(topo, rot, offset, qx, qy, qz): containment of a PRINT-frame point, by ray
/// parity along the mesh frame's +x.
bool insidePart(const Topology &topo, const Rot &rot, const Offset &offset,
                double qx, double qy, double qz);

}} // namespace
