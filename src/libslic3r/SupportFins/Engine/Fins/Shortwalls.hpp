// Port of web/fins/shortwalls.js -- short walls, the LAST RESORT (issue #121). A
// ring-and-strut lattice is all 4-6 mm strut undersides, every one under PROP.minSpan,
// so the walls dropped them as stubs and half the red printed into air. Once the walls
// and wedges are in, the regions still bare get one more placement pass with the
// short-wall floor (prop/clearance.js minSpanFor `short`): a wall down to minSpanShort,
// kept only while it is at most maxShortAspect times as tall as it is long.
//
// Why last and not in the normal pass: a short wall that "serves" a face stands a
// wedge or a raster row down, and those hold it better. Here it only lands where
// nothing else did, so a part the engine already served is untouched.
//
// A wall that comes within sideClear of a wall or wedge already built is dropped
// (bounding boxes, conservative), and so is one that comes within sideClear of a
// short wall kept before it.
#pragma once
#include "../Fins.hpp"
#include "../Prop.hpp"
#include <array>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// lastResortWalls' result (JS { triangles, props, served, tines }). `tags` rides with
/// `triangles`; each prop carries isShort and triRanges into `triangles`.
struct ShortWalls {
    Tris      triangles;
    IfaceTags tags;
    std::vector<BuiltProp> props;
    std::vector<int> served;   // regions, in the Set's insertion order
    int       tines = 0;
};

/// lastResortWalls(topo, result, rot, opts, servedRegions, wedgeTris, existing): short
/// walls for the regions `servedRegions` and `wedgeTris` left bare. `existing` is every
/// support triangle already built (vertex list). `opts` is buildShortWalls' (JS
/// `{ ...opts, tines: withTines, coverage }`).
ShortWalls lastResortWalls(const Topology &topo, const AnalyzeResult &result, const Rot &rot, const PropOpts &opts,
                           const std::vector<int> &servedRegions, const Tris &wedgeTris, const Tris &existing);

// ---- module-internal in the JS; exposed for tests ---------------------------------

/// [minX, minY, minZ, maxX, maxY, maxZ] over tris[from..to), one vertex per entry.
using Box6 = std::array<double, 6>;
Box6 boxOf(const Tris &tris, size_t from, size_t to);
/// Do boxes a and b overlap, a grown by `pad`?
bool overlaps(const Box6 &a, const Box6 &b, double pad);

}} // namespace
