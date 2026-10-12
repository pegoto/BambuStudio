// Port of web/planes.js -- wall sites: the flat, near-upright patches of the part that
// a fin can stand against.
//
// The fin stands BESIDE the part, off an upright face, and leans with the part (by up
// to MAX_LEAN_DEG, a self-supporting overhang for a thin wall). Everything here is
// therefore expressed in the patch's own frame, not the world's. Because a patch is
// certified flat to FLAT_TOL, the distance from the fin wall to the part is the
// standoff everywhere on it, so a tine is a fixed-size nub needing no per-tine ray.
#pragma once
#include "Math.hpp"
#include "Overhangs.hpp"
#include <optional>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// How far from vertical a face may lean and still take a fin. Past 45 degrees the
/// fin wall itself becomes an overhang that needs support.
inline constexpr double MAX_LEAN_DEG = 45;

/// How far a face's normal may swing from its seed's and still join the patch. Wide on
/// purpose (a 32-sided cone steps 11.25 degrees per facet); the distance band against
/// the SEED's plane is the real guard.
inline constexpr double NORMAL_AGREE_DEG = 35;

/// How far the real surface may stray from the patch's fitted plane, in mm. The budget
/// is asymmetric: AWAY from the fin only costs tine length (FLAT_TOL_IN); TOWARD the
/// fin would put the wall inside the part (GROW_SLACK, while growing, before the
/// supporting plane is known -- see fitPatch).
inline constexpr double FLAT_TOL_IN = 1.20;    // away from the fin; paid for in tine length
inline constexpr double GROW_SLACK = 0.35;     // outward slack while growing

inline constexpr double MIN_PATCH_H = 4.0;     // mm, measured up the face
inline constexpr double MIN_PATCH_W = 4.0;     // mm, measured across it
inline constexpr double MIN_PATCH_AREA = 25.0; // mm^2

/// JS `{x, y}` / `{x, y, z}` fields of a patch, kept as named members so ports read
/// `p.n.z`, `p.u.x` exactly like upstream.
struct PatchXY  { double x = 0, y = 0; };
struct PatchXYZ { double x = 0, y = 0, z = 0; };

/// One fin site, the object fitPatch() returns. The plane is `n . q = d`, n the
/// outward 3D normal; the frame (n, u, t) is orthonormal and right-handed:
///   n  out of the part
///   u  across the face, always HORIZONTAL (u.z == 0, so it has no z member)
///   t  up the face, n x u, always with a positive z component
struct WallPatch {
    std::vector<int> faces;       // topology face indices, in growth order (seed first)
    double   area = 0;            // sum of the faces' areas, mm^2
    PatchXY  mid;                 // world xy of the (u, t) bbox centre on the plane
    PatchXYZ n;                   // unit outward normal (area-weighted mean)
    PatchXY  u;                   // unit horizontal across-face direction
    PatchXYZ t;                   // unit up-face direction
    double   h = 0;               // hypot(n.x, n.y), the normal's horizontal share
    double   d = 0;               // supporting-plane offset: max over vertices of n . v
    double   u0 = 0, u1 = 0;      // extent across the face
    double   t0 = 0, t1 = 0;      // extent up the face
    double   z0 = 0, z1 = 0;      // world z extent
    double   lean = 0;            // |asin(n.z)| in degrees
    double   flatness = 0;        // max(devOut, -devIn)
    /// (u, t, dn) per vertex, 9 per face in `faces` order: the in-plane coordinates
    /// plus the SIGNED distance from the plane (<= 0; negative = receded from the fin).
    std::vector<double> tris;
};

/// The optional `stats` out-object of findWallPatches: counters keyed as upstream.
/// Upstream only creates a key on its first increment; 0 here means "absent".
struct PatchStats {
    int grown = 0;        // patches grown from a seed
    int tooSmall = 0;     // total area < MIN_PATCH_AREA
    int notUpright = 0;   // fitted normal has no horizontal share
    int notFlat = 0;      // surface recedes past FLAT_TOL_IN
    int tooShort = 0;     // t extent < MIN_PATCH_H
    int tooNarrow = 0;    // u extent < MIN_PATCH_W
};

/// faceAdjacency(topo): neighbour lists in CSR form. Neighbours of face f are
/// nbr[start[f] .. start[f+1]).
struct FaceAdjacency {
    std::vector<int> start;   // nFaces + 1
    std::vector<int> nbr;
};

/// findWallPatches(topo, rot, offset, stats): every patch of the part a fin could stand
/// against, in orientation `rot` with the part seated by `offset` (from analyze()).
/// Largest area first (stable).
std::vector<WallPatch> findWallPatches(const Topology &topo, const Rot &rot, const Offset &offset,
                                       PatchStats *stats = nullptr);

/// faceAdjacency(topo). UPSTREAM caches the result on the topology (topo._adjStart /
/// _adjNbr); Topology has no slot for it here, so it is rebuilt per call. The result
/// is a pure function of the topology, so this changes cost, not behaviour.
FaceAdjacency faceAdjacency(const Topology &topo);

/// patchProbe(patch, u, t): signed distance of the real surface from the patch plane at
/// (u, t) -- negative meaning receded away from the fin -- or nullopt if the patch does
/// not cover that point.
std::optional<double> patchProbe(const WallPatch &patch, double u, double t);

/// patchCovers(patch, u, t): is (u, t) inside the patch?
bool patchCovers(const WallPatch &patch, double u, double t);

/// patchPoint(p, w, u, t): world point at patch-frame coordinates: `w` out of the face,
/// `u` across, `t` up.
Vec3d patchPoint(const WallPatch &p, double w, double u, double t);

/// tAtZ(p, w, z): the `t` at which the plane offset by `w` crosses world z = `z`.
inline double tAtZ(const WallPatch &p, double w, double z) { return (z - p.n.z * (p.d + w)) / p.t.z; }

/// zAt(p, w, t): world z of a patch-frame point. `u` never contributes: u is horizontal.
inline double zAt(const WallPatch &p, double w, double t) { return p.n.z * (p.d + w) + p.t.z * t; }

}} // namespace Slic3r::SupportFins
