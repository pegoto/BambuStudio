// Port of web/prop/squat.js -- squat walls: the near-bed band of an overhang, too low
// for the full T-wall, gets a thin wall on a wide two-layer brim (`sweepSquat`), built
// over the low stations the tall run leaves behind (`buildSquatBed`).
//
// Interface tags: as Sweep.hpp. buildSquatBed truncates the tags with `out` whenever it
// rolls a wall back (JS `out.length = before`).
#pragma once
#include "../Inside.hpp"
#include "../Math.hpp"
#include "../Overhangs.hpp"
#include "../PropConfig.hpp"
#include "Clearance.hpp"
#include "Contact.hpp"
#include "Crest.hpp"
#include "Surface.hpp"
#include <array>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// Sweep a SQUAT breakaway wall onto `out`: a thin wall necking to the tip on a flat
/// squatBrimH-tall, squatBrimW-wide brim (two overlapping closed solids), plus the
/// interface crest when it is on. Returns false (nothing emitted) when a station
/// repeats in XY or stands under minHeightSquat. An empty line throws
/// std::invalid_argument (a TypeError in the JS ribbon).
bool sweepSquat(const ContactLine &line, double zBed, Tris &out, IfaceTags *tags = nullptr);

/// One placed squat wall (the JS descriptor buildSquatBed returns; prop.js spreads it
/// into its prop record and adds area/region/tines/caps/grip/id/kind/triRanges).
struct SquatWall {
    double      span = 0;      // XY span, first to last settled station
    double      height = 0;    // tallest settled top (surface z - gap) above the bed
    int         stations = 0;  // settled.length
    double      volume = 0;    // |signed volume| of the emitted solid
    bool        squat = true;
    ContactLine line;          // the centreline: settled [x, y, z - gap]
    std::array<size_t, 2> triRange{0, 0};   // [before, out.length): this wall in `out`
};

/// Build brimmed squat walls on the minHeightSquat <= h < minHeight stations of `line`
/// (stations `claimed[k]` skipped; nullptr: none), each settled against `regionTris`,
/// re-trimmed, swept, and dropped if it welds (solidClearance). Appends to `out`
/// (and `tags`) and returns the placed walls.
std::vector<SquatWall> buildSquatBed(const ContactLine &line, const SurfTris &regionTris, const Topology &topo,
                                     const Rot &rot, const Offset &offset, Tris &out,
                                     const Mask *claimed = nullptr, IfaceTags *tags = nullptr);

}} // namespace
