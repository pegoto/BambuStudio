// Port of web/prop/rival.js: the non-template parts (heldArea, the Set and the
// skipped arithmetic). TubeRivalry itself is in Rival.hpp.
#include "Rival.hpp"
#include "../PropConfig.hpp"
#include <algorithm>
#include <cmath>

namespace Slic3r { namespace SupportFins {

namespace {
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    if (a == b) return std::signbit(a) ? a : b;
    return a < b ? a : b;
}
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    if (a == b) return std::signbit(a) ? b : a;
    return a > b ? a : b;
}
} // namespace

bool RegionSet::has(int r) const { return std::find(order.begin(), order.end(), r) != order.end(); }
void RegionSet::add(int r) { if (!has(r)) order.push_back(r); }

void addSkippedDelta(Skipped &a, const Skipped &b, const Skipped &c) {
    a.noLine += b.noLine - c.noLine;
    a.wanders += b.wanders - c.wanders;
    a.stub += b.stub - c.stub;
    a.blocked += b.blocked - c.blocked;
    a.degenerate += b.degenerate - c.degenerate;
    a.buried += b.buried - c.buried;
    a.weld += b.weld - c.weld;
    a.sliver += b.sliver - c.sliver;
    a.onPart += b.onPart - c.onPart;
}

// Overhang area held by support triangles tris[from..], judged over EVERY overhang
// face within reach of the region (a support vertex within maxUnsupportedSpan in plan
// and 0-3 mm below the face, the sweep's coverage rule).
double RivalHeld::operator()(const Soup &own, const Tris &tris, size_t from) {
    const double span = PROP.maxUnsupportedSpan;
    if (!overBuilt_) {
        overBuilt_ = true;
        for (const Region &g : result_.regions) {
            for (int f : g.faces) {
                const size_t t = size_t(f) * 9;
                overFaces_.push_back({(partTris_[t] + partTris_[t + 3] + partTris_[t + 6]) / 3,
                                      (partTris_[t + 1] + partTris_[t + 4] + partTris_[t + 7]) / 3,
                                      (partTris_[t + 2] + partTris_[t + 5] + partTris_[t + 8]) / 3,
                                      topo_.area[size_t(f)]});
            }
        }
    }
    double x0 = INF, y0 = INF, x1 = -INF, y1 = -INF;
    for (size_t k = 0; k < own.size(); k += 3) {
        // UPSTREAM reads own[k + 1] past the end of a length not a multiple of 3; a
        // triangle array always is one.
        x0 = jsMin(x0, own[k]); x1 = jsMax(x1, own[k]);
        y0 = jsMin(y0, own[k + 1]); y1 = jsMax(y1, own[k + 1]);
    }
    const double reach = 2 * span;             // a wall under the region, then its span
    double held = 0;
    for (const auto &of : overFaces_) {
        const double cx = of[0], cy = of[1], cz = of[2], a = of[3];
        if (cx < x0 - reach || cx > x1 + reach || cy < y0 - reach || cy > y1 + reach) continue;
        for (size_t i = from; i < tris.size(); i++) {
            const Vec3d &v = tris[i];
            if (v[2] > cz - 3 && v[2] < cz + 0.5 && hypot2(v[0] - cx, v[1] - cy) <= span) {
                held += a;
                break;
            }
        }
    }
    return held;
}

}} // namespace
