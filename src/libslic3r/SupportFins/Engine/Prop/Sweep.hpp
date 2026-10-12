// Port of web/prop/sweep.js -- the wall solid: `sweep` emits the upside-down T (thin
// wall on a flat flange) along a contact line down to the plate, `sweepBetween` a
// part-attached wall between an overhang and the floor under it. `footFor` /
// `profileHalf` give the cross-section the clearance probes test against.
//
// Interface tags (see Crest.hpp): the sweeps take the caller's IfaceTags parallel to
// `out` (nullptr: untracked). They only touch it when the crest is on (dropCollapsed /
// emitCrest); the wall itself is untagged, so a tags vector left shorter than `out`
// reads 0 there, as Crest.hpp specifies.
#pragma once
#include "../Math.hpp"
#include "../PropConfig.hpp"
#include "Contact.hpp"
#include "Crest.hpp"
#include "Mold.hpp"

namespace Slic3r { namespace SupportFins {

/// Foot half-width for a wall of height `h`: footRatio * h, floored at footMin and
/// capped at footMax and at half the row spacing (maxUnsupportedSpan) less 0.5 mm.
double footFor(double h);

/// Half-width of the ⊥ cross-section at height `z` above the bed, for a wall whose
/// contact tips out at `top`: flange (footFor(top)) below baseH, th/2 up to the tip
/// taper, then necking to tip/2 at `top`. Shared by sweep's geometry and the
/// clearance probes.
double profileHalf(double z, double top);

/// Sweep the prop along `line` (stations [x, y, surface z]; the wall top is z - gap),
/// emitting an upside-down T onto `out`: the thin wall (with CUT.pattern holes when
/// cutWall takes it) and its flat base flange, two overlapping closed solids, plus the
/// interface crest when it is on. Returns false (nothing emitted) when a station
/// repeats its neighbours in XY or stands under `minH` above `zBed`.
/// An empty line throws std::invalid_argument (the JS throws a TypeError in ribbon).
bool sweep(const ContactLine &line, double zBed, Tris &out, double minH = PROP.minHeight,
           IfaceTags *tags = nullptr);

/// Sweep a PART-ATTACHED wall: top `gap` below `topLine`, bottom PROP.footGap above the
/// floor `botLine` (floorLine / moldLine stations, one per top station; a lifted
/// station's side floors tilt the bottom), on open plate (floor 0) standing on it.
/// Welded (footGap <= 0) the bottom tapers to the tip width. Returns false (nothing
/// emitted) when a station repeats in XY or has under `minH` of headroom.
/// An empty topLine throws std::invalid_argument; a botLine shorter than topLine
/// throws std::out_of_range (both TypeErrors in the JS).
bool sweepBetween(const ContactLine &topLine, const FloorLine &botLine, Tris &out,
                  double minH = PROP.minHeight, IfaceTags *tags = nullptr);

}} // namespace
