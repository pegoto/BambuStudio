// Port of web/kiss.js -- tine ends that KISS the part's surface.
#include "Kiss.hpp"
#include <cmath>
#include <limits>

namespace Slic3r { namespace SupportFins {

namespace {
// JS Math.min / Math.max on two doubles: NaN propagates, -0 < +0.
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    return (a < b || (a == b && std::signbit(a))) ? a : b;
}
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    return (a > b || (a == b && !std::signbit(a))) ? a : b;
}
} // namespace

/**
 * Where a tine meets the part, so it can KISS the surface instead of running on into
 * it: for each side edge of the tine (across = +-`half`) at its bottom and its top
 * (`zb`, `zt`, a hair inside the layer), the distance along the bite heading (dx, dy)
 * at which the part's solid begins, plus TRIM_KISS so the two still overlap (flush
 * faces weld into one non-manifold body in Fusion). The caller builds the tine's end
 * through those four corners, so on a sloped or curved underside the end leans with
 * the surface and no part of the tine sits inside the part.
 *
 * Per corner: no solid within reach on that line and height -> the full `reach`;
 * solid already back over the wall end -> TRIM_MIN past it. Distances are from (x, y):
 * a wall tine's seed on the wall top, a sway tine's point on the face. `back` bounds
 * how far behind that the end may pull back.
 */
KissEnds kissEnds(const Topology &topo, const Rot &rot, const Offset &offset,
                  double x, double y, double dx, double dy, double zb, double zt, const KissOpts &opts) {
    const double R = opts.reach, half = opts.half, back = opts.back;
    // The end may lean out past `reach` at the bottom: on a 40 deg underside the
    // surface there is ~0.24 mm farther than at the top, and capping it at the reach
    // cut the tine short of the surface at mid-layer, where the slicer reads it.
    const double lo0 = -back + TRIM_MIN, far = 2 * R;
    auto reach = [&](double s, double z) -> double {
        const double px = x - dy * s, py = y + dx * s;      // across = z x along, as emitTines
        auto inside = [&](double d) { return insidePart(topo, rot, offset, px + dx * d, py + dy * d, z); };
        // far probe first; a part thinner than that can put it in air past the part, so
        // fall back to `reach` itself before giving up
        double lo = lo0, hi;
        if (inside(far)) hi = far;
        else if (inside(R)) hi = R;
        else return R;                                   // no solid within reach: the old length
        if (inside(lo)) return lo0;                      // the part already reaches back over the wall
        for (int i = 0; i < 14; i++) {                   // ~0.0001 mm on a 1.25 span
            const double mid = (lo + hi) / 2;
            if (inside(mid)) hi = mid; else lo = mid;
        }
        return hi + TRIM_KISS;
    };
    // measured a hair inside the layer (a part face can sit exactly on a layer line),
    // then carried out to the tine's real bottom and top along the same lean
    const double k = TRIM_Z / (zt - zb - 2 * TRIM_Z);
    auto side = [&](double s) -> std::array<double, 2> {
        const double b = reach(s, zb + TRIM_Z), t = reach(s, zt - TRIM_Z);
        auto clamp = [&](double e) { return jsMin(far, jsMax(lo0, e)); };
        return {clamp(b + (b - t) * k), clamp(t + (t - b) * k)};
    };
    const std::array<double, 2> l = side(-half), r = side(half);
    KissEnds out;
    out.bot = {l[0], r[0]};
    out.top = {l[1], r[1]};
    return out;
}

}} // namespace
