// Port of web/prop/crest.js -- INTERFACE CREST: the band of a wall that touches the
// part, printed in a second material (GitHub #21). Modes (PROP.iface): Flat crests a
// station only under an underside within ifaceFlatDeg of level, All crests every
// station, Off nothing (and nothing here runs).
//
// THE SPLIT: a crest station's wall is cut crestH() below its top: the BODY below,
// the CREST above, both closed solids, the crest reaching CREST_OVERLAP into the body.
//
// THE TAG. In the JS a crest/tine vertex carries a 4th component, v[3] === 1, that
// rides with the vertex object through concatenation and slicing. Tris (Vec3d) has no
// room for it, so here it is a byte per element of `out` in a parallel IfaceTags the
// caller owns and passes (nullptr: untracked, as when nobody exports the interface).
// A tags vector shorter than `out` reads 0 past its end; markInterface/dropCollapsed
// first pad it to out.size(). A caller that truncates `out` (JS `out.length = n`)
// truncates its tags with it.
//
// Module state (the seated part the 'flat' test reads, and the layer height) is
// thread_local, set by crestPart at each build's entry.
#pragma once
#include "../Math.hpp"
#include "../PropConfig.hpp"
#include "../Solids.hpp"
#include "Surface.hpp"
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// How far the crest reaches down into the body, so the two overlap.
inline constexpr double CREST_OVERLAP = 0.01;

/// What a station gets: JS null | 'flat' | 'slope'.
enum class CrestKind { None, Flat, Slope };
using CrestKinds = std::vector<CrestKind>;

/// Per-element interface tags parallel to a Tris (JS v[3] === 1).
using IfaceTags = std::vector<uint8_t>;

/// P(o, z) -> world point: o across the wall, z up (the wall builders' section frame).
using Frame2 = std::function<Vec3d(double, double)>;

/// Is the crest on (either mode)?
bool crestOn();

/// Set the seated part the 'flat' test reads the underside from (nullptr: none) and the
/// print's layer height (0 / NaN: PROP.tineH, JS `layerHeight || PROP.tineH`).
void crestPart(std::shared_ptr<const SurfTris> tris, double layerHeight = 0);

/// The crest's height: PROP.ifaceLayers whole layers of the print.
double crestH();

/// What each station of a wall gets from its contact points (x, y, surface z): Flat,
/// Slope (only in All mode) or None; a lone crest station gets None. nullopt (JS null)
/// when the crest is off.
std::optional<CrestKinds> crestKinds(const std::vector<Vec3d> &pts);

/// Tag out[from..] as interface.
void markInterface(Tris &out, size_t from = 0, IfaceTags *tags = nullptr);

/// A wall's crest sections, one per station (nullopt where it has none), emitted as
/// tagged closed solids: one ribbon per run of >= 2 consecutive crest stations.
void emitCrest(const std::vector<std::optional<Section>> &sections, Tris &out, IfaceTags *tags = nullptr);

/// Drop out[from..]'s triangles with two corners on one point (value equality),
/// compacting in place (and the tags with it).
void dropCollapsed(Tris &out, size_t from, IfaceTags *tags = nullptr);

/// splitInterface's result: the body and interface triangles (JS `{ body, iface }`).
struct InterfaceSplit {
    Tris      body, iface;
    IfaceTags ifaceTags;   // all 1, parallel to iface (the vertices keep their tag in JS)
};
/// Split a triangle list into its body and interface triangles (a triangle's first
/// vertex decides, as in the JS).
InterfaceSplit splitInterface(const Tris &tris, const IfaceTags *tags);

/// crestCut's record (JS `{ cut, zt, wc, ring }`).
struct CrestCut {
    double cut = 0;   // the body's new top
    double zt = 0;    // the body's taper start
    double wc = 0;    // the body's half-width at `cut`
    // ring(P) = [P(+wb, cb), P(+wt, top), P(-wt, top), P(-wb, cb)]
    double wb = 0, wt = 0, cb = 0, top = 0;
    /// The crest section, 4 corners from CREST_OVERLAP under `cut` to `top`.
    Section ring(const Frame2 &P) const;
};

/// Where a tapered wall top (th wide up to `ztip`, necking to `tip` at `top`) splits
/// into body and crest for a station of `kind`; `floor` is the highest bottom vertex
/// of the section. nullopt (JS null) for no crest.
std::optional<CrestCut> crestCut(double top, double ztip, double floor, CrestKind kind,
                                 double th = PROP.th, double tip = PROP.tip);

/// A flat-topped wall's crest (a wedge blade's): where the body stops (the bed as floor).
double flatCut(double top);
/// The band from CREST_OVERLAP under flatCut(top) to `top`, `half` either side.
Section flatCrestRing(const Frame2 &P, double top, double half);

}} // namespace
