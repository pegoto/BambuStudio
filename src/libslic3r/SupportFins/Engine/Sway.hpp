// Port of web/sway.js -- SWAY BRACES: buttress ribs that keep a TALL part from
// drifting, sagging and wobbling as it grows.
//
// A sway brace is:
//   1. the RIB, a vertical plate standing PERPENDICULAR to an upright face (edge-on to
//      it, its stiff direction). Its inner edge follows the face at the breakaway gap,
//      and it TAPERS: deep at the bed, narrow at the top;
//   2. the FOOT, a thin flange on the plate so the rib stays down;
//   3. the TINES, one-layer horizontal bridges from the rib's inner edge into the part,
//      spaced EVENLY up the whole height (the sway is at the top).
//
// The gap arrives in the options rather than being read from FIN/PROP, so this module
// depends on neither: the caller passes the material's numbers.
//
// THE TAG. With Interface material on everywhere (opts.iface == All) a tine's vertices
// carry the JS 4th component v[3] = 1; here that is a parallel IfaceTags byte per
// element of the triangle list (as Prop/Crest.hpp), 1 for a tine vertex, else 0. Tags
// are always the same length as their Tris.
#pragma once
#include "Fins.hpp"
#include "Math.hpp"
#include "Overhangs.hpp"
#include "Planes.hpp"
#include "PropConfig.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// Per-element interface tags parallel to a Tris (JS v[3] === 1). Same alias as
/// Prop/Crest.hpp (a redeclaration to the same type is legal).
using IfaceTags = std::vector<uint8_t>;

/// SWAY: every number a sway brace is built from. Never mutated upstream, so const.
struct SwayConfig {
    // which faces take a brace
    double maxLeanDeg = 30;    // a face leaning further than this is an overhang or a roof, not a side
    double minFaceH = 30;      // mm of face height before auto-placement bothers bracing it
    double minTopFrac = 0.4;   // ...and it must reach this far up the part, or it braces the easy half
    double minPartH = 30;      // mm; a shorter part doesn't sway enough to need this

    // the rib
    double thMin = 1.2;        // wall thickness at the minimum height (the fins' own wall)
    double thPerMm = 0.004;    // grows with height: 1.2mm at 0, 2.2mm at 250mm
    double thMax = 2.4;
    double reach = 0.15;       // DEFAULT base depth as a fraction of rib height (UI: "Brace depth")
    double topDepth = 4;       // mm at the top: a flat edge, not a point (a point is a retraction)
    double minDepth = 8;
    double maxDepth = 60;
    double topClear = 1.0;     // the rib stops this far below the top of the face it braces
    double minRibH = 20;       // a shorter rib isn't bracing anything

    // the foot
    double footH = 0.6;
    double footHalf = 3.0;     // flange past the rib on each side
    double footPad = 3.0;      // ...and past its outer end

    // the tines (the standoff comes from the material profile, via opts)
    double gap = 0.2;
    // How far in from the face a tine looks for the part: placed only if the part is
    // there at half this, and its end stops on the surface (kissEnds, searching up to 2x).
    double tineReach = 0.3;
    double tineW = 0.5;        // one nozzle bead, as elsewhere
    double tineOverlap = 0.3;  // how far the tine sinks back into the rib, so they union
    double tineSpanMax = 1.5;  // past this much open air a tine is a bridge
    double tineSpacing = 6;    // DEFAULT mm between tines up the rib (UI: "Tine spacing")
    int    minTines = 3;
    double minGripShare = 0.3; // ...and at least this share of the rows up the rib must find the face
    // ...and it may not climb this far before its FIRST tine (a brace is a lone wall
    // below its lowest grip). The fraction keeps a short rib from being mostly stilt,
    // the absolute stops a very tall one from growing an unreasonable one.
    double stiltMax = 40;
    double stiltMaxFrac = 0.4;

    // auto-placement
    double pitch = 100;        // mm of face width per rib
    int    maxPerFace = 4;
    double endInset = 10;      // keep ribs near the ends of a face (FIN-SPEC: edges hide marks)
    int    maxFaces = 4;
    double minBearingSep = 60; // faces this far apart in bearing, so opposite + adjacent sides both count
    std::array<double, 7> nudges = {0, 3, -3, 6, -6, 10, -10};
    double clearance = 1.0;    // mm of air two braces must keep between them
    double levelStep = 10;     // mm between the heights a clash check compares
    // Half-width to assume for a prop wall or wedge the braces must stay clear of: the
    // widest foot either builds (PROP/PERP footHalf 3.0 + half a 1.2mm wall) rounded up.
    double wallHalf = 3.6;
};

inline const SwayConfig SWAY{};

/// One entry of a rib's `levels`: its footprint at height z (JS `{ z, a, b }`), a the
/// inner edge, b the outer, world points.
struct SwayLevel {
    double z = 0;
    Vec3d  a = Vec3d::Zero(), b = Vec3d::Zero();
};

/// buildSwayRib's result (JS `{ ok: true, tris, tines, height, depth, th, stilt, foot,
/// halfW, levels }` or `{ ok: false, reason }`), plus the `triRange` buildSwayBraces
/// writes onto each placed rib. Also stands in for the plain-data brace outlines
/// fins.js hands back (`{ foot, halfW, th, height, levels }`): swayClashes reads only those.
struct SwayRib {
    bool ok = false;
    std::optional<std::string> reason;   // set iff !ok
    Tris      tris;                      // outward-wound vertex triples, seated frame
    IfaceTags tags;                      // parallel to tris (1 = interface tine vertex)
    int    tines = 0;
    double height = 0;                   // H: the rib's height
    double depth = 0;                    // D0: its depth at the bed
    double th = 0;                       // wall thickness
    double stilt = 0;                    // how far it stands before its first tine (0 without tines)
    std::vector<Vec3d> foot;             // [inner, outer] on the plate; EMPTY = JS no `foot` (a failed rib)
    double halfW = 0;                    // the foot's half-width
    std::vector<SwayLevel> levels;
    std::optional<TriRange> triRange;    // buildSwayBraces: its vertex range into `triangles`
};

/// The `avoid` argument (JS: an array of braces, or `{ braces, walls }`): braces
/// standing (buildSwayRib results or outlines) and prop/wedge centrelines (polylines,
/// e.g. FinRecord::line). The JS plain-array form is `{ braces = arr }`.
struct SwayAvoid {
    std::vector<SwayRib>            braces;
    std::vector<std::vector<Vec3d>> walls;
};

/// The options object buildSwayRib / buildSwayBraces / swayAtFace take: fins.js's
/// `{ ...opts.sway, tines, layerHeight, iface: PROP.iface, avoid: { walls } }`.
/// SwayOpts (Fins.hpp) carries on / gripFrom / tineSpacing / reach / gap; a non-finite
/// number reads as missing (JS settings() num()), so the defaults below are the JS ones.
struct SwayBuildOpts : SwayOpts {
    bool   tines = true;                 // JS `opts.tines !== false`
    double layerHeight = 0.2;            // clamped to >= 0.04
    Iface  iface = Iface::Off;           // only All tags the tines
    std::optional<bool> allowStilt;      // JS `opts.allowStilt === true`; unset = undefined
    SwayAvoid avoid;                     // buildSwayBraces reads avoid.walls only
};

/// buildSwayBraces' result (JS `{ triangles, count, tines, skipped, ribs, reason }`).
struct SwayBraces {
    Tris      triangles;                 // every placed rib's tris, in placement order
    IfaceTags tags;                      // parallel to triangles
    int count = 0, tines = 0, skipped = 0;
    std::vector<SwayRib> ribs;           // each with triRange set
    std::optional<std::string> reason;   // set when nothing was placed (JS null otherwise)
};

/// Build one sway brace against patch `p`, centred across the face at `uc`. `partTris`
/// is the part in print space, 9 per face (printTriangles).
SwayRib buildSwayRib(const WallPatch &p, double uc, const Soup &partTris, const Topology &topo,
                     const Rot &rot, const Offset &offset, const SwayBuildOpts &opts = {});

/// Would `rib` run into any of `ribs`? Feet on the plate, then the ribs level by level.
bool swayClashes(const SwayRib &rib, const std::vector<SwayRib> &ribs);

/// Would `rib`'s foot run into a prop wall or wedge (centrelines, world points)?
bool swayClashesWall(const SwayRib &rib, const std::vector<std::vector<Vec3d>> &walls);

/// AUTO: brace the tallest upright sides of the part (buildSwayBraces(topo, result, rot, opts)).
SwayBraces buildSwayBraces(const Topology &topo, const AnalyzeResult &result, const Rot &rot,
                           const SwayBuildOpts &opts = {});

/// Is this face (by index) upright enough, in this pose, to take a sway brace?
bool faceIsUpright(const Topology &topo, const Rot &rot, int faceIndex);

/// The per-pose patch cache swayAtFace keeps upstream (a WeakMap keyed by topo, holding
/// the last pose's patches). Optional here: pass one to swayAtFace to skip re-running
/// findWallPatches when the pose is unchanged. It remembers the Topology by address,
/// so the caller must reset() it when that Topology is destroyed or modified. Without
/// a cache every call recomputes, which changes cost, not results.
struct SwayPatchCache {
    const Topology *topo = nullptr;
    Rot    rot{};
    Offset offset;
    std::vector<WallPatch> patches;
    std::unordered_map<int, int> byFace;  // face -> index into patches (first patch holding it)
    Soup   partTris;
    void reset() { topo = nullptr; patches.clear(); byFace.clear(); partTris.clear(); }
};

/// DRAW: a sway brace on the face the user clicked, at the spot they clicked (`point`,
/// print space). `avoid` is the braces / walls already standing; the new one may shift a
/// few mm to clear them, and is refused -- with that reason -- if it can't. opts'
/// allowStilt defaults to true here (JS `{ allowStilt: true, ...opts }`).
SwayRib swayAtFace(const Topology &topo, const AnalyzeResult &result, const Rot &rot, int faceIndex,
                   const Vec3d &point, const SwayBuildOpts &opts = {}, const SwayAvoid &avoid = {},
                   SwayPatchCache *cache = nullptr);

}} // namespace Slic3r::SupportFins
