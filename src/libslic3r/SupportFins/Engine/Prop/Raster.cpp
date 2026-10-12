// Port of web/prop/raster.js. See Raster.hpp.
#include "Raster.hpp"
#include "Mold.hpp"
#include "Sweep.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace Slic3r { namespace SupportFins {

namespace {
// JS Math.min / Math.max: NaN if either is NaN; -0 < +0.
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
// JS truthiness of a number: !x for 0, -0, NaN.
inline bool truthy(double v) { return v != 0 && !std::isnan(v); }

// JS byRegion: a Map region -> its props, in first-seen order.
struct ByRegion {
    std::vector<int> order;
    std::map<int, PropRefs> m;
    const PropRefs *get(int r) const {
        auto it = m.find(r);
        return it == m.end() ? nullptr : &it->second;
    }
};
ByRegion byRegion(const std::vector<PropRecord> &props) {
    ByRegion b;
    for (const PropRecord &q : props) {
        auto it = b.m.find(q.region);
        if (it == b.m.end()) { b.order.push_back(q.region); it = b.m.emplace(q.region, PropRefs{}).first; }
        it->second.push_back(&q);
    }
    return b;
}
PropRefs refsOf(const std::vector<PropRecord> &props) {
    PropRefs r;
    for (const PropRecord &q : props) r.push_back(&q);
    return r;
}
} // namespace

double gripRise(double z) { return jsMax(GRIP_RISE, GRIP_RISE_REL * z); }

Tracks rasterTracks(const std::vector<Vec3d> &regionPts, const SurfTris &regionTris, const SurfTris &partTris,
                    double step, double rowSpan, const Support *support) {
    Tracks lines;
    const Tracks tracks = patchTracks(regionPts, regionTris, step, support, rowSpan);
    lines.spacing = tracks.spacing;       // row pitch, for the sparse-coverage sag warning
    for (const ContactLine &t : tracks) {
        // floorLine's plate/part split, the same test buildPartAttached counts with
        const FloorLine floor = floorLine(t, partTris);
        ContactLine cur;
        std::optional<bool> cls;
        for (size_t k = 0; k < t.size(); k++) {
            const bool onPart = floor[k][2] > PROP.gap + 0.5;
            if (cls.has_value() && onPart != *cls) {
                if (int(cur.size()) >= PROP.minStations) lines.push_back(cur);
                cur.clear();
            }
            cls = onPart;
            cur.push_back(t[k]);
        }
        if (int(cur.size()) >= PROP.minStations) lines.push_back(cur);
    }
    return lines;
}

std::vector<RasterPiece> rasterRest(const ContactLine &line, const Run &run) {
    std::vector<RasterPiece> rest;
    const std::array<std::array<int, 2>, 2> sides{{{0, run[0]}, {run[1], int(line.size())}}};
    for (const auto &[a, b] : sides) {
        if (b - a < PROP.minStations) continue;
        RasterPiece piece;
        piece.assign(line.begin() + a, line.begin() + b);
        piece.from = a;
        rest.push_back(std::move(piece));
    }
    return rest;
}

std::vector<OverFace> overhangFaces(const Topology &topo, const AnalyzeResult &result, const Rot &rot) {
    const Soup &pos = topo.pos;
    const Offset &off = result.offset;
    Vec3d v(0, 0, 0);
    std::vector<OverFace> faces;
    for (size_t ri = 0; ri < result.regions.size(); ri++) {
        for (const int f : result.regions[ri].faces) {
            double cx = 0, cy = 0, cz = 0;
            for (int i = 0; i < 3; i++) {
                seat(pos, size_t(f) * 9 + size_t(i) * 3, rot, off, v);
                cx += v[0] / 3; cy += v[1] / 3; cz += v[2] / 3;
            }
            faces.push_back({cx, cy, cz, topo.area[size_t(f)], int(ri)});
        }
    }
    return faces;
}

// Tops, not every support vertex as rival.js counts: a wall's base flange sits within
// 3 mm of any low face around it.
double heldBy(const std::vector<OverFace> &faces, const std::vector<Vec3d> &tops) {
    const double reach = PROP.maxUnsupportedSpan / 2;
    double held = 0;
    for (const OverFace &f : faces) {
        for (const Vec3d &p : tops) {
            const double dz = f.cz - p[2];
            if (dz >= -0.05 && dz <= 1.5 && hypot2(p[0] - f.cx, p[1] - f.cy) <= reach) {
                held += f.area;
                break;
            }
        }
    }
    return held;
}

std::vector<Vec3d> wallTops(const PropRefs &qs) {
    std::vector<Vec3d> tops;
    for (const PropRecord *q : qs) tops.insert(tops.end(), q->line.begin(), q->line.end());
    return tops;
}

double halfFoot(const PropRecord &q) {
    return q.squat ? PROP.squatBrimW : q.partAttached ? PROP.th / 2 : footFor(q.height);
}

double gripZ(const PropRefs &qs) {
    double z = INF;
    for (const PropRecord *q : qs) z = jsMin(z, q->grip);
    return z;
}

double planGap(const ContactLine &l1, const ContactLine &l2) {
    const auto seg = [](const Vec3d &p, const Vec3d &a, const Vec3d &b) {
        const double dx = b[0] - a[0], dy = b[1] - a[1], L = dx * dx + dy * dy;
        const double t = L > 1e-12 ? jsMax(0, jsMin(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L)) : 0;
        return hypot2(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy);
    };
    double d = INF;
    for (const auto &[P, Q] : {std::pair<const ContactLine *, const ContactLine *>{&l1, &l2}, {&l2, &l1}}) {
        for (const Vec3d &p : *P) {
            if (Q->size() == 1) d = jsMin(d, hypot2(p[0] - (*Q)[0][0], p[1] - (*Q)[0][1]));
            for (size_t i = 0; i + 1 < Q->size(); i++) d = jsMin(d, seg(p, (*Q)[i], (*Q)[i + 1]));
        }
    }
    return d;
}

PropsResult withRaster(const Topology &topo, const AnalyzeResult &result, const Rot &rot,
                       const RasterVeto &rasterVeto, const PropsResult &normal, size_t capFrom,
                       const RasterPass &rasterPass) {
    const std::vector<OverFace> faces = overhangFaces(topo, result, rot);
    const RegionSet want = rasterWanted(faces, result, normal);
    if (!want.size()) return normal;
    std::vector<TineCap> *cap = TINECAP;
    std::optional<std::vector<TineCap>> capNormal;
    if (cap) capNormal = *cap;
    std::vector<TineCap> fresh;
    if (cap) TINECAP = &fresh;
    PropsResult raster = rasterPass(want);
    if (rasterVeto) {
        std::vector<PropRecord> kept;
        for (const PropRecord &q : raster.props) if (!rasterVeto(q, normal.props)) kept.push_back(q);
        raster.props = std::move(kept);
    }
    std::vector<TineCap> *capRaster = TINECAP;
    TINECAP = cap;
    PropsResult raced = raceRegions(faces, result, normal, raster);
    if (!(raced.rasterRegions && *raced.rasterRegions)) return normal;
    if (cap) {
        cap->resize(capFrom);
        for (const PropRecord &q : raced.props) {
            if (!q.caps) continue;
            const std::vector<TineCap> &src = q.raster ? *capRaster : *capNormal;
            // JS Array.prototype.slice clamps to the array
            const size_t s = std::min((*q.caps)[0], src.size()), e = std::min((*q.caps)[1], src.size());
            if (s < e) cap->insert(cap->end(), src.begin() + std::ptrdiff_t(s), src.begin() + std::ptrdiff_t(e));
        }
    }
    return raced;
}

RegionSet rasterWanted(const std::vector<OverFace> &faces, const AnalyzeResult &result, const PropsResult &a) {
    const std::vector<Vec3d> tops = wallTops(refsOf(a.props));
    RegionSet want;
    for (int ri = 0; ri < int(result.regions.size()); ri++) {
        std::vector<OverFace> own;
        for (const OverFace &f : faces) if (f.region == ri) own.push_back(f);
        double area = 0;
        for (const OverFace &f : own) area += f.area;
        if (area > 0 && heldBy(own, tops) < RASTER_SKIP_HELD * area) want.add(ri);
    }
    return want;
}

std::vector<OverFace> nearRegion(const std::vector<OverFace> &faces, int ri) {
    double x0 = INF, y0 = INF, x1 = -INF, y1 = -INF;
    for (const OverFace &f : faces) {
        if (f.region != ri) continue;
        x0 = jsMin(x0, f.cx); x1 = jsMax(x1, f.cx);
        y0 = jsMin(y0, f.cy); y1 = jsMax(y1, f.cy);
    }
    const double reach = 2 * PROP.maxUnsupportedSpan;
    std::vector<OverFace> near;
    for (const OverFace &f : faces)
        if (f.cx >= x0 - reach && f.cx <= x1 + reach && f.cy >= y0 - reach && f.cy <= y1 + reach) near.push_back(f);
    return near;
}

// Kept: a normal wall that stands clear of every kept wall (feet + 1 mm apart in plan)
// and either holds area they miss or grips lower than they do. If the region's lowest
// grip still rises past gripRise, no swap.
std::optional<PropRefs> keepers(const PropRefs &qa, const PropRefs &qb, const std::vector<OverFace> &near) {
    PropRefs kept = qb, extra;
    std::vector<OverFace> bare;
    for (const OverFace &f : near) if (!truthy(heldBy({f}, wallTops(kept)))) bare.push_back(f);
    const auto clear = [&](const PropRecord &q) {
        for (const PropRecord *r : kept)
            if (!(planGap(q.line, r->line) >= halfFoot(q) + halfFoot(*r) + 1)) return false;
        return true;
    };
    // lowest-gripping first, so a low wall claims its spot before a neighbour
    PropRefs order = qa;
    std::stable_sort(order.begin(), order.end(), [](const PropRecord *x, const PropRecord *y) {
        return gripZ({x}) - gripZ({y}) < 0;     // JS comparator; NaN (INF - INF) reads as not-less
    });
    for (const PropRecord *q : order) {
        if (q->line.empty() || !clear(*q)) continue;
        const bool addsArea = heldBy(bare, q->line) >= RACE_ABS;
        const bool addsGrip = gripZ({q}) < gripZ(kept) - GRIP_RISE;
        if (!addsArea && !addsGrip) continue;
        extra.push_back(q);
        kept.push_back(q);
        for (int i = int(bare.size()) - 1; i >= 0; i--)
            if (truthy(heldBy({bare[size_t(i)]}, q->line))) bare.erase(bare.begin() + i);
    }
    const double za = gripZ(qa);
    if (gripZ(kept) > za + gripRise(za)) return std::nullopt;
    return extra;
}

// `skipped` stays the normal pass's: over-reporting a skip is the safe direction.
PropsResult raceRegions(const std::vector<OverFace> &faces, const AnalyzeResult &result,
                        const PropsResult &a, const PropsResult &b) {
    const ByRegion A = byRegion(a.props), B = byRegion(b.props);
    PropsResult res;
    Tris &out = res.triangles;
    IfaceTags &tags = res.tags;
    RegionSet served;
    int tines = 0, swapped = 0;
    double volume = 0;
    bool sagRisk = a.sagRisk;
    const auto take = [&](const PropsResult &build, const PropRecord &q) {
        std::vector<TriRange> triRanges;
        for (const TriRange &se : q.triRanges) {
            const size_t r0 = out.size();
            for (size_t i = se[0]; i < se[1]; i++) {
                out.push_back(build.triangles[i]);
                tags.push_back(i < build.tags.size() ? build.tags[i] : 0);
            }
            triRanges.push_back({r0, out.size()});
        }
        PropRecord q2 = q;
        q2.id = int(res.props.size());
        q2.triRanges = std::move(triRanges);
        q2.raster = &build == &b;
        res.props.push_back(std::move(q2));
        served.add(q.region);
        tines += q.tines;
        volume += q.volume;
    };
    const PropRefs none;
    for (int ri = 0; ri < int(result.regions.size()); ri++) {
        const PropRefs &qa = A.get(ri) ? *A.get(ri) : none, &qb = B.get(ri) ? *B.get(ri) : none;
        std::optional<PropRefs> extra;
        if (!qb.empty()) {
            const std::vector<OverFace> near = nearRegion(faces, ri);
            const double ha = !qa.empty() ? heldBy(near, wallTops(qa)) : 0;
            const double hb = heldBy(near, wallTops(qb));
            if (hb > ha + jsMax(RACE_ABS, RACE_REL * ha)) extra = keepers(qa, qb, near);
        }
        if (!extra) { for (const PropRecord *q : qa) take(a, *q); continue; }
        swapped++;
        sagRisk = sagRisk || (b.sagRegions ? b.sagRegions->has(ri) : false);
        for (const PropRecord *q : qb) take(b, *q);
        for (const PropRecord *q : *extra) take(a, *q);
    }
    res.skipped = a.skipped;
    res.served = int(served.size());
    res.servedRegions = served.order;
    res.tines = tines;
    res.sagRisk = sagRisk;
    res.volume = volume;
    res.rasterRegions = swapped;
    return res;
}

}} // namespace
