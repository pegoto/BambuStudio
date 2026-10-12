// Port of web/fins/shortwalls.js -- see Shortwalls.hpp.
#include "Shortwalls.hpp"
#include "Wedges.hpp"
#include <algorithm>

namespace Slic3r { namespace SupportFins {

Box6 boxOf(const Tris &tris, size_t from, size_t to) {
    Box6 b{{INF, INF, INF, -INF, -INF, -INF}};
    for (size_t i = from; i < to; i++) {
        const Vec3d &v = tris[i];
        for (int k = 0; k < 3; k++) { if (v[k] < b[k]) b[k] = v[k]; if (v[k] > b[k + 3]) b[k + 3] = v[k]; }
    }
    return b;
}

bool overlaps(const Box6 &a, const Box6 &b, double pad) {
    return a[0] - pad <= b[3] && b[0] <= a[3] + pad && a[1] - pad <= b[4] && b[1] <= a[4] + pad &&
           a[2] - pad <= b[5] && b[2] <= a[5] + pad;
}

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

ShortWalls lastResortWalls(const Topology &topo, const AnalyzeResult &result, const Rot &rot, const PropOpts &opts,
                           const std::vector<int> &servedRegions, const Tris &wedgeTris, const Tris &existing) {
    const BareRegions bare = bareAfterWedges(topo, rot, result, servedRegions, wedgeTris);
    ShortWalls none;
    if (bare.empty()) return none;
    RegionSet regions;
    for (int r : bare) regions.add(r);
    const PropsResult built = buildShortWalls(topo, result, rot, opts, regions);
    if (built.props.empty()) return none;

    std::vector<Box6> others;
    for (size_t i = 0; i < existing.size(); i += 3) others.push_back(boxOf(existing, i, i + 3));
    ShortWalls out;
    RegionSet served;
    for (const PropRecord &p : built.props) {
        const std::vector<TriRange> &ranges = p.triRanges;
        Box6 box{{INF, INF, INF, -INF, -INF, -INF}};
        for (const TriRange &r : ranges) {
            const Box6 rb = boxOf(built.triangles, r[0], r[1]);
            Box6 nb;
            for (int k = 0; k < 3; k++) nb[k] = jsMin(box[k], rb[k]);
            for (int k = 3; k < 6; k++) nb[k] = jsMax(box[k], rb[k]);
            box = nb;
        }
        bool hit = false;
        for (const Box6 &o : others) if (overlaps(box, o, PROP.sideClear)) { hit = true; break; }
        if (hit) continue;
        std::vector<TriRange> triRanges;
        for (const TriRange &r : ranges) {
            const size_t at = out.triangles.size();
            for (size_t i = r[0]; i < r[1]; i++) {
                out.triangles.push_back(built.triangles[i]);
                out.tags.push_back(i < built.tags.size() ? built.tags[i] : 0);
            }
            triRanges.push_back(TriRange{at, out.triangles.size()});
        }
        others.push_back(box);                               // and the next short wall keeps off this one
        BuiltProp q(p);
        q.triRanges = std::move(triRanges);
        q.isShort = true;
        out.props.push_back(std::move(q));
        served.add(p.region);
        out.tines += p.tines;
    }
    out.served = served.order;
    return out;
}

}} // namespace
