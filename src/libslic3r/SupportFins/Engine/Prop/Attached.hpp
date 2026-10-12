// Port of web/prop/attached.js -- PART-ATTACHED walls: an overhang above another part
// of the part stands its wall on that floor (Mold.hpp's floorLine) instead of stilting
// to the plate through it, and refuses a buried column (buriedColumn) or one a side
// wall cuts through (clearBetween). A wall inside a bore is built and flagged inBore.
// buildPartAttached is the verdict the auto-placer (prop.js buildPass) acts on.
//
// Interface tags: as Sweep.hpp. buildPartAttached passes the caller's IfaceTags to
// sweepBetween and truncates them with `out` when it rolls a wall back (JS
// `out.length = before`).
#pragma once
#include "../Inside.hpp"
#include "../Math.hpp"
#include "../Overhangs.hpp"
#include "../PropConfig.hpp"
#include "Contact.hpp"
#include "Crest.hpp"
#include "Mold.hpp"
#include "Surface.hpp"
#include <optional>
#include <string>

namespace Slic3r { namespace SupportFins {

/// How far below the clicked/probed overhang a settle pass may still pull the top down
/// when looking for a part-attached support. Shared with draw.js (and fins/fill.js) so
/// the paths measure a part-attached wall alike.
inline constexpr double PART_BAND = 3.0;

/// The bore probe (JS BORE): `dirs` compass rays from the column, `step` mm apart out
/// to `radius`; `walledMin` of them hitting part = enclosed.
struct BoreConfig {
    double radius = 10;
    int    dirs = 8;
    int    walledMin = 6;
    double step = 0.5;
};
inline constexpr BoreConfig BORE{};

/// The part-attached wall record (JS `prop`): prop.js spreads it into its prop record
/// and adds area/region/tines/caps/grip/trimmed/id/kind/triRanges (see Raster.hpp
/// PropRecord).
struct PartAttachedProp {
    double      span = 0;       // XY span, first to last station of the kept run
    double      height = 0;     // tallest (top - gap) - floor over the run
    int         stations = 0;   // subTop.length
    double      volume = 0;     // |signed volume| of out[before..] (the wall and its crest)
    bool        partAttached = true;
    bool        inBore = false;
    ContactLine line;           // the centreline: subTop [x, y, z - gap]
};

/// buildPartAttached's verdict. JS returns one of three objects:
///   { ok: true, prop }  -> ok, prop set: a part-attached wall was built into `out`.
///   { floored: why }    -> floored set ('stub' | 'buried' | 'blocked' | 'degenerate'):
///                          there is a floor but no safe wall fits; the caller counts
///                          it under skipped[why] and must not stilt to the plate.
///   { }                 -> neither: no floor, an ordinary bed overhang.
struct PartAttached {
    bool ok = false;
    std::optional<PartAttachedProp> prop;
    std::optional<std::string> floored;
};

/// Try to stand a PART-ATTACHED wall under `line` (the overhang contact polyline,
/// seated; not modified -- the JS works on a copy). Settles a PART_BAND-banded top,
/// reads the floor with floorLine, keeps the longest run of standable stations and
/// bridges it with moldLine + sweepBetween onto `out` (and `tags`).
PartAttached buildPartAttached(const ContactLine &line, const SurfTris &partTris, const Topology &topo,
                               const Rot &rot, const Offset &offset, Tris &out, IfaceTags *tags = nullptr);

// ---- module-internal in the JS; exposed for tests ---------------------------------

/// A column whose own centreline (mid-height between floor and top - gap) is inside
/// the part is buried.
bool buriedColumn(const ContactLine &top, const FloorLine &floor, size_t k, const Topology &topo,
                  const Rot &rot, const Offset &offset);

/// Is the column at station k walled nearly all round (a bore or slot)?
bool enclosedColumn(const ContactLine &top, const FloorLine &floor, size_t k, const Topology &topo,
                    const Rot &rot, const Offset &offset);

/// Can a part-attached wall at station k stand between its floor and the overhang
/// without piercing a side wall? Probes strictly between the ends.
bool clearBetween(const ContactLine &top, const FloorLine &floor, size_t k, const Topology &topo,
                  const Rot &rot, const Offset &offset);

}} // namespace
