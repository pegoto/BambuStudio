// Port of web/fins/wedges.js -- ANGLED WEDGES: the perpendicular rib auto stands under
// a broad downward face no prop wall reached. `buildPerpFins` tiles wedges along a
// patch (`perpColumns` picks the stations, emitFoot the plate foot), `gripPatches`
// lists the faces a wedge can stand under, `propServesPatch` skips faces a prop
// already holds, `wedgeVeto` keeps buildProps' raster walls off the faces left to a
// wedge, and `unservedAfterWedges` / `bareAfterWedges` recount what is still red.
//
// This header deliberately does NOT include Prop/Raster.hpp: that header and Fins.hpp
// define `TriRange` differently, and Fins.cpp (applyTunables writes PERP.gap) must
// include this one. PropRecord is forward-declared and RasterVeto redeclared with the
// identical type (a redeclaration of an alias to the same type is legal; Wedges.cpp
// includes both, so the compiler checks they agree). Include Prop/Raster.hpp to build
// or read PropRecords.
#pragma once
#include "../Math.hpp"
#include "../Overhangs.hpp"
#include "../Planes.hpp"
#include "../Prop/Crest.hpp"
#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

namespace Slic3r { namespace SupportFins {

struct PropRecord;   // Prop/Raster.hpp: buildProps' prop record
/// Prop/Raster.hpp's RasterVeto (opts.rasterVeto), the same type.
using RasterVeto = std::function<bool(const PropRecord &q, const std::vector<PropRecord> &normalProps)>;

/// JS PERP. Mutable module state (fins.js applyTunables writes `gap`); thread_local
/// like FIN and PROP. The reasoning behind each number is in wedges.js.
struct PerpConfig {
    double th = 1.2;        // wedge thickness (thin across the face)
    double gap = 0.2;       // breakaway clearance under the contact (matches PROP/FIN)
    double footHalf = 3.0;  // foot flange half-width past the wedge, each side
    double footH = 0.6;
    double pitch = 24.0;    // mm between wedges across the face (a ROW, not a wall of plastic)
    double tStep = 1.5;     // sampling step up the face
    double minH = 2.0;      // skip a wedge shorter than this
    double inset = 2.0;     // keep wedges off the very edges of the patch
    int    maxRow = 14;     // hard cap on wedges per patch, so a wide face never sprays
    // A wedge is for a BROAD leaning face props can't reach. Below this face area (or
    // u-extent) it is a small/curved patch better left to props or draw-mode.
    double minArea = 500;
    double minWidth = 22;
};

extern thread_local PerpConfig PERP;

/// gripPatches' result (JS { patches, faceMap }). faceMap is a JS Map from face index
/// to the FIRST patch holding it; here the value is an index into `patches`, and
/// `faceOrder` keeps the Map's insertion order (its keys, as JS iterates them).
struct GripPatches {
    std::vector<WallPatch>       patches;
    std::unordered_map<int, int> faceMap;
    std::vector<int>             faceOrder;
};

/// gripPatches(topo, result, rot): every downward (n.z < -0.05) wall patch in this pose
/// (findWallPatches at result.offset) and a face -> patch index (DRAW mode).
GripPatches gripPatches(const Topology &topo, const AnalyzeResult &result, const Rot &rot);

/// perpColumns(p, lo, hi, pitch): the u positions to stand wedges at across [lo, hi]:
/// one even row per band of standable u's (a bore splits the face into bands), at most
/// PERP.maxRow in all.
std::vector<double> perpColumns(const WallPatch &p, double lo, double hi, double pitch);

/// buildPerpFins' `opts` ({ tines, pitch, tineDensity }, all optional in the JS).
struct WedgeOpts {
    bool tines = true;                      // JS `opts.tines !== false`
    std::optional<double> pitch;            // JS `opts.pitch ?? PERP.pitch`
    std::optional<double> tineDensity;      // tineStepFor(opts.tineDensity); nullopt = undefined
};

/// One wedge's record (JS { triRange, line, height, span }).
struct PerpWedge {
    std::array<size_t, 2> triRange{{0, 0}};  // [before, out.length): elements of `triangles`
    std::vector<Vec3d>    line;              // the contact profile up the face (JS `contact`)
    double                height = 0;        // max top z (contact z - PERP.gap)
    double                span = 0;          // plan length of the contact profile
};

/// buildPerpFins' result (JS { triangles, tines, count, wedges }). `tags` rides with
/// `triangles` (interface tags, Prop/Crest.hpp), padded to triangles.size().
struct PerpFins {
    Tris      triangles;
    IfaceTags tags;
    int       tines = 0;
    int       count = 0;
    std::vector<PerpWedge> wedges;
};

/// buildPerpFins(p, topo, rot, offset, opts): tile perpendicular wedges across one
/// down-facing patch.
PerpFins buildPerpFins(const WallPatch &p, const Topology &topo, const Rot &rot, const Offset &offset,
                       const WedgeOpts &opts = WedgeOpts());

/// propServesPatch(p, props): does a prop wall's line already stand under this patch's
/// footprint (its four (u, t) corners' xy bbox, plus 8 mm)? A non-squat prop's points
/// under PROP.minHeight do not count.
bool propServesPatch(const WallPatch &p, const std::vector<PropRecord> &props);

/// wedgeVeto(patches, topo, rot, offset, wedgeOpts): buildProps' opts.rasterVeto in auto
/// mode. The closure holds copies of `patches`, `rot`, `offset`, `wedgeOpts` and a
/// pointer to `topo` (which must outlive it), and caches the patches left to a wedge per
/// `normalProps` OBJECT -- the JS compares by reference, here by address -- so pass the
/// same vector for one build. Copies of the returned function share that cache, as
/// references to one JS closure do. Reads PERP / PROP when called (thread_local).
RasterVeto wedgeVeto(const std::vector<WallPatch> &patches, const Topology &topo, const Rot &rot,
                     const Offset &offset, const WedgeOpts &wedgeOpts);

/// bareAfterWedges' result: indices into result.regions, ascending (JS number[]).
using BareRegions = std::vector<int>;

/// bareAfterWedges(topo, rot, result, servedRegions, wedgeTris): the regions neither in
/// `servedRegions` nor within maxUnsupportedSpan in plan (and 0..3 mm below, 0.5 above)
/// of any wedge vertex.
BareRegions bareAfterWedges(const Topology &topo, const Rot &rot, const AnalyzeResult &result,
                            const std::vector<int> &servedRegions, const Tris &wedgeTris);

/// unservedAfterWedges(...): bareAfterWedges(...).length.
int unservedAfterWedges(const Topology &topo, const Rot &rot, const AnalyzeResult &result,
                        const std::vector<int> &servedRegions, const Tris &wedgeTris);

}} // namespace Slic3r::SupportFins
