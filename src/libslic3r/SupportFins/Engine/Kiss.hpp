// Port of web/kiss.js -- tine ends that KISS the part's surface: shared by the wall
// tines (prop/tines.js) and the sway braces' tines (sway.js). Takes the reach and
// widths as arguments, so neither caller's settings leak into the other (sway.js reads
// no PROP/FIN).
#pragma once
#include "Inside.hpp"
#include <array>

namespace Slic3r { namespace SupportFins {

inline constexpr double TRIM_KISS = 0.01, TRIM_MIN = 0.05, TRIM_Z = 0.01;

/// kissEnds' destructured options { half, back, reach: R }.
struct KissOpts {
    double half = 0;    // half the tine's width (across = +-half)
    double back = 0;    // how far behind (x, y) the end may pull back
    double reach = 0;   // R: how far out the tine looks for the part
};

/// kissEnds' { bot: [left, right], top: [left, right] }: distances along (dx, dy) from
/// (x, y), left = across -half, right = across +half.
struct KissEnds {
    std::array<double, 2> bot{}, top{};
};

/// kissEnds(topo, rot, offset, x, y, dx, dy, zb, zt, { half, back, reach }): where a
/// tine meets the part. All of x, y, dx, dy (unit bite heading), zb, zt (the tine's
/// bottom and top) are in the PRINT frame; topo/rot/offset as insidePart takes them
/// (Inside.hpp). Per corner: no solid within reach -> reach; solid already back over
/// the wall end -> TRIM_MIN - back; else the bisected surface distance + TRIM_KISS.
KissEnds kissEnds(const Topology &topo, const Rot &rot, const Offset &offset,
                  double x, double y, double dx, double dy, double zb, double zt, const KissOpts &opts);

}} // namespace
