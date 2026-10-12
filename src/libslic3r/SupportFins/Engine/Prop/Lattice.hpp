// Port of web/prop/lattice.js -- lattice nets: a big overhang region that is really
// many struts joined at nodes (issue #121). `latticeStruts` cuts such a region back
// into its struts (footprint raster, Zhang-Suen skeleton, cut at junctions, faces to
// the nearest branch through the footprint); `strutPatches` routes each strut the way
// buildProps routes a standalone small region. buildProps runs it in the raster
// pass's slot.
#pragma once
#include "../Math.hpp"
#include "../Overhangs.hpp"
#include "Tracks.hpp"
#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// Plan raster cell, mm.
inline constexpr double LATTICE_CELL = 0.5;          // JS CELL
/// A strut's underside is at most NET_HALF from its edge (99th percentile of the
/// footprint), and a net has at least MIN_STRUTS of them.
inline constexpr double LATTICE_NET_HALF = 3.5;      // JS NET_HALF
inline constexpr int    LATTICE_MIN_STRUTS = 5;      // JS MIN_STRUTS
inline constexpr double LATTICE_NODE_CUT = 1.5;      // JS NODE_CUT: cells removed around a junction

/// `seated(f)`: face f's three seated vertices (prop.js
/// `(f) => [0, 1, 2].map((i) => seat(pos, f * 9 + i * 3, rot, off, [0, 0, 0]))`).
using Seated = std::function<std::array<Vec3d, 3>(int)>;

/// latticeStruts' result: face groups, one per strut (JS array of face arrays), plus
/// the array's `.lost`: faces no live branch reached.
struct Struts : std::vector<std::vector<int>> {
    int lost = 0;
};

/// latticeStruts(topo, faces, seated): the struts of a lattice net, or nullopt (JS
/// null) when `faces` is not one.
std::optional<Struts> latticeStruts(const Topology &topo, const std::vector<int> &faces, const Seated &seated);

/// strutPatches(topo, rot, groups, seated, step, region, regionTris): buildProps
/// patches for a net's struts, each marked `strut`. A tube strut: { faces, area,
/// region, tris: regionTris, reach: own tris, lines: tube, smallTube: split | none,
/// strut } (its split patches carry strut + axis); otherwise { faces, area, region,
/// tris: regionTris, strut, axis }.
std::vector<PatchPtr> strutPatches(const Topology &topo, const Rot &rot, const std::vector<std::vector<int>> &groups,
                                   const Seated &seated, double step, int region,
                                   const std::shared_ptr<const SurfTris> &regionTris);

/// longAxis(pts): unit principal axis of points in plan (internal in the JS).
Vec2d longAxis(const std::vector<Vec3d> &pts);

}} // namespace
