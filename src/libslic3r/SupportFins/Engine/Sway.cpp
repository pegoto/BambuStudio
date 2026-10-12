// Port of web/sway.js -- see Sway.hpp.
#include "Sway.hpp"
#include "Inside.hpp"
#include "Kiss.hpp"
#include "Solids.hpp"
#include "Ieee754.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace Slic3r { namespace SupportFins {

namespace {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

// JS Math.min / Math.max on two doubles: NaN propagates, -0 < +0.
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NaN;
    return (a < b || (a == b && std::signbit(a))) ? a : b;
}
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NaN;
    return (a > b || (a == b && !std::signbit(a))) ? a : b;
}

/// x.toFixed(0): the integer nearest the EXACT value of x, ties to the larger
/// magnitude (std::round), with "-" for any x < 0 (so (-0.4).toFixed(0) is "-0").
std::string toFixed0(double x) {
    if (std::isnan(x)) return "NaN";
    if (std::isinf(x)) return x > 0 ? "Infinity" : "-Infinity";
    char buf[64];
    if (std::fabs(x) >= 1e21) {   // toFixed falls back to ToString(x); unreachable for mm
        std::snprintf(buf, sizeof buf, "%.17g", x);
        return buf;
    }
    const bool neg = x < 0;
    std::snprintf(buf, sizeof buf, "%s%.0f", neg ? "-" : "", std::round(neg ? -x : x));
    return buf;
}

double leanCut() { return js_sin((SWAY.maxLeanDeg * M_PI) / 180); }

/// settings(opts): the caller's options resolved against the defaults, clamped to
/// what prints.
struct Settings {
    bool   tines;
    double layerH, gap, gripFrom, spacing, reach;
    bool   allowStilt;
};

Settings settings(const SwayBuildOpts &opts) {
    auto num = [](double v, double d) { return std::isfinite(v) ? v : d; };
    Settings s;
    s.tines = opts.tines;
    s.layerH = jsMax(0.04, num(opts.layerHeight, 0.2));
    s.gap = num(opts.gap, SWAY.gap);
    s.gripFrom = jsMax(0, num(opts.gripFrom, 0));
    s.spacing = jsMax(1, num(opts.tineSpacing, SWAY.tineSpacing));
    s.reach = jsMax(0.05, jsMin(0.5, num(opts.reach, SWAY.reach)));
    // Auto refuses a brace that would stand a long way up before its first tine; a
    // brace placed BY HAND is built anyway and reports the stilt instead.
    s.allowStilt = opts.allowStilt.has_value() && *opts.allowStilt;
    return s;
}

/// The part's triangles in print space, as a flat array.
Soup printTriangles(const Topology &topo, const Rot &rot, const Offset &offset) {
    const Soup &pos = topo.pos;
    Soup a(size_t(topo.nFaces) * 9);
    for (size_t i = 0; i < a.size(); i += 3) {
        const double x = pos[i], y = pos[i + 1], z = pos[i + 2];
        a[i] = rot[0] * x + rot[3] * y + rot[6] * z + offset.x;
        a[i + 1] = rot[1] * x + rot[4] * y + rot[7] * z + offset.y;
        a[i + 2] = rot[2] * x + rot[5] * y + rot[8] * z + offset.z;
    }
    return a;
}

using P3 = std::array<double, 3>;
using Poly3 = std::vector<P3>;

/// Sutherland-Hodgman: keep the part of `poly` where `f(v) >= 0`.
template <class F>
Poly3 clip(const Poly3 &poly, const F &f) {
    Poly3 out;
    const size_t n = poly.size();
    for (size_t i = 0; i < n; i++) {
        const P3 &a = poly[i], &b = poly[(i + 1) % n];
        const double fa = f(a), fb = f(b);
        if (fa >= 0) out.push_back(a);
        if ((fa >= 0) != (fb >= 0)) {
            const double t = fa / (fa - fb);
            out.push_back({a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t});
        }
    }
    return out;
}

/// Push a closed solid, flipping the winding outward if its signed volume is negative.
/// `tag` is the JS v[3] every vertex of `local` carries (all or none here).
void pushSolid(const Tris &local, bool tag, Tris &out, IfaceTags &tags) {
    double V = 0;
    for (size_t i = 0; i < local.size(); i += 3) {
        const Vec3d &a = local[i], &b = local[i + 1], &c = local[i + 2];
        V += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2])
           + a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    if (V < 0) {
        for (size_t i = 0; i < local.size(); i += 3) {
            out.push_back(local[i]); out.push_back(local[i + 2]); out.push_back(local[i + 1]);
        }
    } else {
        for (const Vec3d &v : local) out.push_back(v);
    }
    tags.resize(out.size(), tag ? 1 : 0);
}

/// A convex polygon `poly` ([[a, b], ...]) extruded from c = lo to c = hi, mapped by P(a, b, c).
template <class PF>
void prism(const std::vector<std::array<double, 2>> &poly, double lo, double hi, const PF &P, Tris &out, IfaceTags &tags) {
    const size_t n = poly.size();
    std::vector<Vec3d> L, H;
    L.reserve(n); H.reserve(n);
    for (const auto &q : poly) L.push_back(P(q[0], q[1], lo));
    for (const auto &q : poly) H.push_back(P(q[0], q[1], hi));
    Tris local;
    for (size_t i = 0; i < n; i++) {
        const size_t j = (i + 1) % n;
        local.push_back(L[i]); local.push_back(L[j]); local.push_back(H[j]);
        local.push_back(L[i]); local.push_back(H[j]); local.push_back(H[i]);
    }
    for (size_t i = 1; i + 1 < n; i++) {
        local.push_back(H[0]); local.push_back(H[i]); local.push_back(H[i + 1]);
        local.push_back(L[0]); local.push_back(L[i + 1]); local.push_back(L[i]);
    }
    pushSolid(local, false, out, tags);
}

/// The rib's frame on a patch: `s` is horizontal distance out of the face (along the
/// face normal's horizontal part), `uu` is across the face (the patch's own u, so
/// patchProbe reads it directly), z is z. A world point is s*nh + uu*u.
/// (JS `{ ...frameOf(p), uDir: { x: p.u.x, y: p.u.y } }`.)
struct Frame3 {
    struct { double x, y; } nh, u, uDir;
    Vec3d toWorld(double s, double uu, double z) const {
        return Vec3d(nh.x * s + u.x * uu, nh.y * s + u.y * uu, z);
    }
    double sOf(const Vec3d &q) const { return q[0] * nh.x + q[1] * nh.y; }
};

Frame3 frameOf(const WallPatch &p) {
    Frame3 fr;
    fr.nh = {p.n.x / p.h, p.n.y / p.h};
    fr.u = {p.u.x, p.u.y};
    fr.uDir = {p.u.x, p.u.y};
    return fr;
}

using Outline = std::vector<std::function<double(const P3 &)>>;

/// The lowest z at which any part triangle crosses the volume the rib would occupy,
/// or Infinity. Exact per triangle: clip to the rib's slab across the face, then
/// against the rib's own (s, z) outline.
double lowestHit(const Soup &tris, const Frame3 &fr, double uLo, double uHi, const Outline &outline) {
    double best = INF;
    P3 v[3];
    for (size_t i = 0; i < tris.size(); i += 9) {
        double umin = INF, umax = -INF;
        for (int k = 0; k < 3; k++) {
            const double x = tris[i + k * 3], y = tris[i + k * 3 + 1], z = tris[i + k * 3 + 2];
            v[k][0] = x * fr.nh.x + y * fr.nh.y;
            v[k][1] = x * fr.uDir.x + y * fr.uDir.y;
            v[k][2] = z;
            if (v[k][1] < umin) umin = v[k][1];
            if (v[k][1] > umax) umax = v[k][1];
        }
        if (umax < uLo || umin > uHi) continue;
        Poly3 poly = {v[0], v[1], v[2]};
        poly = clip(poly, [&](const P3 &q) { return q[1] - uLo; });
        poly = clip(poly, [&](const P3 &q) { return uHi - q[1]; });
        for (const auto &f : outline) {
            if (poly.size() < 2) break;
            poly = clip(poly, f);
        }
        if (poly.size() < 2) continue;
        for (const P3 &q : poly) if (q[2] < best) best = q[2];
    }
    return best;
}

SwayRib fail(std::string reason) {
    SwayRib r;
    r.ok = false;
    r.reason = std::move(reason);
    return r;
}

/// Closest distance between two 2D segments (only [0], [1] of each point are read).
double segDist(const Vec3d &a, const Vec3d &b, const Vec3d &c, const Vec3d &d) {
    auto pt = [](const Vec3d &p, const Vec3d &q, const Vec3d &r) {
        const double dx = r[0] - q[0], dy = r[1] - q[1];
        const double L = dx * dx + dy * dy;
        const double t = L > 0 ? jsMax(0, jsMin(1, ((p[0] - q[0]) * dx + (p[1] - q[1]) * dy) / L)) : 0;
        return hypot2(p[0] - q[0] - dx * t, p[1] - q[1] - dy * t);
    };
    auto cross = [](const Vec3d &o, const Vec3d &p, const Vec3d &q) {
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0]);
    };
    const double d1 = cross(a, b, c), d2 = cross(a, b, d), d3 = cross(c, d, a), d4 = cross(c, d, b);
    if (((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0))) return 0;
    return jsMin(jsMin(jsMin(pt(a, c, d), pt(b, c, d)), pt(c, a, b)), pt(d, a, b));
}

/// A rib's footprint at height z, interpolated between its sampled levels (2D: z = 0).
struct LevelAB { Vec3d a, b; };
LevelAB levelAt(const std::vector<SwayLevel> &levels, double z) {
    // UPSTREAM: an empty `levels` would read levels[0] (undefined) and throw; a rib
    // always has >= 2 (H >= minRibH > levelStep), so not guarded here either.
    int k = 0;
    const int n = int(levels.size());
    while (k < n - 2 && levels[k + 1].z < z) k++;
    const SwayLevel &A = levels[k], &B = levels[std::min(k + 1, n - 1)];
    const double f = B.z > A.z ? jsMax(0, jsMin(1, (z - A.z) / (B.z - A.z))) : 0;
    auto mix = [f](const Vec3d &p, const Vec3d &q) {
        return Vec3d(p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f, 0);
    };
    return {mix(A.a, B.a), mix(A.b, B.b)};
}

/// How high the face reaches at `u` -- the tallest rib a column there could carry.
double columnTop(const WallPatch &p, double u) {
    double top = -INF;
    for (double z = jsMax(0, p.z0); z <= p.z1 + 1e-6; z += 2) {
        if (patchProbe(p, u, tAtZ(p, 0, z)).has_value()) top = z;
    }
    return top;
}

/// Where along a face to stand its ribs: at the TALLEST columns, kept apart. Ties go
/// to the ends (FIN-SPEC: edges hide the tine marks).
std::vector<double> columnsFor(const WallPatch &p) {
    const double W = p.u1 - p.u0;
    const double n = jsMax(1, jsMin(SWAY.maxPerFace, js_round(W / SWAY.pitch)));
    const double inset = jsMin(SWAY.endInset, W / 4);
    const double a = p.u0 + inset, b = p.u1 - inset;
    const double steps = jsMax(1, jsMin(24, js_round((b - a) / 5)));
    const double mid = (a + b) / 2;
    struct Sample { double u, top, edge; };
    std::vector<Sample> samples;
    for (int k = 0; k <= steps; k++) {
        const double u = a + ((b - a) * k) / steps;
        samples.push_back({u, columnTop(p, u), std::abs(u - mid)});
    }
    // (y.top - x.top) || (y.edge - x.edge): NaN (-Inf - -Inf) and 0 fall through
    std::stable_sort(samples.begin(), samples.end(), [](const Sample &x, const Sample &y) {
        const double d = y.top - x.top;
        const double c = (d != 0 && !std::isnan(d)) ? d : (y.edge - x.edge);
        return c < 0;
    });
    const double apart = jsMax(SWAY.pitch / 2, W / (n + 1));
    std::vector<double> cols;
    for (const Sample &s : samples) {
        if (double(cols.size()) >= n) break;
        if (s.top == -INF) break;
        bool near = false;
        for (double c : cols) if (std::abs(c - s.u) < apart) { near = true; break; }
        if (near) continue;
        cols.push_back(s.u);
    }
    if (cols.empty()) cols.push_back(mid);
    return cols;
}

} // namespace

/**
 * Build one sway brace against patch `p`, centred across the face at `uc`.
 */
SwayRib buildSwayRib(const WallPatch &p, double uc, const Soup &partTris, const Topology &topo,
                     const Rot &rot, const Offset &offset, const SwayBuildOpts &opts) {
    const Settings S = settings(opts);
    if (std::abs(p.n.z) > leanCut()) {
        return fail("that face leans too far to stand a brace against — pick an upright side");
    }
    const Frame3 fr = frameOf(p);

    // How much of the face this column actually crosses, 1mm at a time.
    double fz0 = INF, fz1 = -INF;
    for (double z = jsMax(0, p.z0); z <= p.z1 + 1e-6; z += 1) {
        if (patchProbe(p, uc, tAtZ(p, 0, z)).has_value()) {
            if (z < fz0) fz0 = z;
            if (z > fz1) fz1 = z;
        }
    }
    if (fz1 == -INF) return fail("there is no face under that spot to brace");

    double H = fz1 - SWAY.topClear;
    if (H < SWAY.minRibH) {
        return fail("that face only reaches " + toFixed0(fz1) + "mm up — too short to need a brace");
    }
    auto thFor = [](double h) { return jsMin(SWAY.thMax, SWAY.thMin + SWAY.thPerMm * h); };
    double th = thFor(H);

    // Seat the rib's inner edge on the outermost point of the face INSIDE its own
    // slab: the patch plane touches the patch's global high point, which may be
    // elsewhere on the face, and the gap has to be a floor here.
    auto shiftIn = [&](double half) {
        double m = -INF;
        for (size_t i = 0; i < p.tris.size(); i += 9) {
            Poly3 poly = {
                P3{p.tris[i], p.tris[i + 1], p.tris[i + 2]},
                P3{p.tris[i + 3], p.tris[i + 4], p.tris[i + 5]},
                P3{p.tris[i + 6], p.tris[i + 7], p.tris[i + 8]},
            };
            poly = clip(poly, [&](const P3 &q) { return q[0] - (uc - half); });
            poly = clip(poly, [&](const P3 &q) { return (uc + half) - q[0]; });
            for (const P3 &q : poly) if (q[2] > m) m = q[2];
        }
        return m == -INF ? 0.0 : m;
    };

    // The outline in (s, z). The inner edge follows the (possibly leaning) face plane
    // at the gap; the outer edge tapers from `D0` at the bed to topDepth.
    double wIn = 0, sIn0 = 0, sInSlope = 0, D0 = 0;
    auto shape = [&]() {
        th = thFor(H);
        wIn = shiftIn(th / 2 + SWAY.tineW) + S.gap;
        const double a = fr.sOf(patchPoint(p, wIn, uc, tAtZ(p, wIn, 0)));
        const double b = fr.sOf(patchPoint(p, wIn, uc, tAtZ(p, wIn, 1)));
        sIn0 = a; sInSlope = b - a;
        D0 = jsMax(SWAY.minDepth, jsMin(SWAY.maxDepth, S.reach * H));
    };
    auto sIn = [&](double z) { return sIn0 + sInSlope * z; };
    auto depthAt = [&](double z) { return D0 + (SWAY.topDepth - D0) * jsMin(1, jsMax(0, z / H)); };

    // Shrink the rib until nothing on the part crosses it: a feature sticking out
    // above the face (a ledge, a shoulder) caps it below that feature.
    double hit = INF;
    for (int iter = 0; iter < 6; iter++) {
        shape();
        const Outline outline = {
            [&](const P3 &q) { return q[2] + 0.1; },
            [&](const P3 &q) { return (H + 0.3) - q[2]; },
            [&](const P3 &q) { return q[0] - (sIn(q[2]) - 0.05); },
            [&](const P3 &q) { return (sIn(q[2]) + depthAt(q[2]) + 0.1) - q[0]; },
        };
        hit = lowestHit(partTris, fr, uc - th / 2 - 0.3, uc + th / 2 + 0.3, outline);
        if (hit == INF) break;
        H = hit - 1.0;
        if (H < SWAY.minRibH) break;
    }
    if (hit != INF) {
        return fail("the part sticks out over that spot, so a brace standing on the plate can’t reach up the face");
    }

    // The foot has its own, wider footprint on the plate.
    const double sFootOut = sIn(0) + D0 + SWAY.footPad;
    const double footHalfW = th / 2 + SWAY.footHalf;
    const double footHit = lowestHit(partTris, fr, uc - footHalfW - 0.2, uc + footHalfW + 0.2, Outline{
        [&](const P3 &q) { return q[2] + 0.1; },
        [&](const P3 &q) { return (SWAY.footH + 0.2) - q[2]; },
        [&](const P3 &q) { return q[0] - (sIn(0) - 0.05); },
        [&](const P3 &q) { return (sFootOut + 0.1) - q[0]; },
    });
    if (footHit != INF) {
        return fail("the part’s base spreads out under this face, so there is no room on the plate for the brace’s foot");
    }

    SwayRib R;
    Tris &out = R.tris;
    IfaceTags &tags = R.tags;
    auto P = [&](double s, double z, double uu) { return fr.toWorld(s, uu, z); };
    prism({{sIn(0), 0}, {sIn(0) + D0, 0}, {sIn(H) + SWAY.topDepth, H}, {sIn(H), H}},
          uc - th / 2, uc + th / 2, P, out, tags);
    prism({{sIn(0), uc - footHalfW}, {sFootOut, uc - footHalfW}, {sFootOut, uc + footHalfW}, {sIn(0), uc + footHalfW}},
          0, SWAY.footH, [&](double s, double uu, double z) { return fr.toWorld(s, uu, z); }, out, tags);

    // Tines: evenly spaced up the face, each snapped into exactly one layer cell.
    int tines = 0;
    double firstGrip = INF, stilt = 0;
    if (S.tines) {
        const double zStart = jsMax(jsMax(fz0 + 0.5, S.gripFrom), SWAY.footH + 0.5);
        const double zEnd = jsMin(fz1, H) - 0.5;
        for (double z = zStart; z <= zEnd + 1e-6; z += S.spacing) {
            const double bot = js_round(z / S.layerH) * S.layerH;
            const double top = bot + S.layerH;
            if (top > H) break;
            const double zMid = bot + S.layerH / 2;
            std::optional<double> dev0 = patchProbe(p, uc, tAtZ(p, 0, zMid));
            if (!dev0) continue;
            double dev = *dev0;
            const std::optional<double> refined = patchProbe(p, uc, tAtZ(p, dev, zMid));
            if (refined) dev = *refined;
            const double sPart = fr.sOf(patchPoint(p, dev, uc, tAtZ(p, dev, zMid)));
            const double sWall = sIn(zMid);
            if (sWall - sPart > SWAY.tineSpanMax) continue;      // a bridge, not a tine
            const Vec3d bx = fr.toWorld(sPart - SWAY.tineReach / 2, uc, zMid);
            if (!insidePart(topo, rot, offset, bx[0], bx[1], bx[2])) continue;  // grips nothing
            // the tine runs from inside the rib to where it meets the part, its end
            // following the face between bottom and top (kissEnds). Frame: along = into
            // the part (-s) from the face point, across = (z x along).
            const Vec3d o = fr.toWorld(sPart, uc, 0);
            const double ax = -fr.nh.x, ay = -fr.nh.y;
            const double half = SWAY.tineW / 2;
            // measured from the face point, not the tine's back: `back` only bounds how
            // far out of the face the end may pull back, well short of the rib (ov)
            const KissEnds e = kissEnds(topo, rot, offset, o[0], o[1], ax, ay, bot, top,
                                        KissOpts{half, SWAY.tineOverlap, SWAY.tineReach});
            const double ov = -(sWall + SWAY.tineOverlap - sPart);
            const Frame PT = [&](double a, double b, double zz) {
                return Vec3d(o[0] + ax * a - ay * b, o[1] + ay * a + ax * b, zz);
            };
            auto outlineOf = [&](const std::array<double, 2> &eh) {
                return Poly2{Vec2d(ov, -half), Vec2d(eh[0], -half), Vec2d(eh[1], half), Vec2d(ov, half)};
            };
            Tris local;
            loftExtrude(outlineOf(e.bot), outlineOf(e.top), bot, top, PT, local);
            // with Interface material on everywhere, a tine prints in the interface
            // filament, tagged the way prop/crest.js tags the walls'; not under 'flat',
            // which leaves sideways grip alone
            pushSolid(local, opts.iface == Iface::All, out, tags);
            tines++;
            if (bot < firstGrip) firstGrip = bot;
        }
        // A tall rib tied on at a handful of points still lets the part wave about
        // between them, so the grip has to cover a real share of the height too.
        const double wanted = jsMax(SWAY.minTines, std::floor(SWAY.minGripShare * (zEnd - zStart) / S.spacing));
        if (tines < wanted) {
            return fail("too little of this face lines up with the brace for its tines to grip — try a flatter part of the side");
        }
        // Everything below the lowest tine is a lone wall holding nothing, and held by
        // nothing. Measured from the plate, or from "Brace grip from" when higher.
        stilt = jsMax(0, firstGrip - jsMax(SWAY.footH, S.gripFrom));
        const double maxStilt = jsMin(SWAY.stiltMax, SWAY.stiltMaxFrac * H);
        if (!S.allowStilt && stilt > maxStilt) {
            return fail("this side only starts " + toFixed0(firstGrip) + "mm up, so the brace "
                        + "would stand " + toFixed0(stilt) + "mm holding nothing before it grips (max "
                        + toFixed0(maxStilt) + "mm) " + "— rotate so this side reaches the plate");
        }
    }

    R.foot = {fr.toWorld(sIn(0), uc, 0), fr.toWorld(sFootOut, uc, 0)};
    // The rib's footprint at a ladder of heights, so a clash check can compare two
    // ribs at the SAME z.
    for (double z = 0;; z = jsMin(H, z + SWAY.levelStep)) {
        R.levels.push_back({z, fr.toWorld(sIn(z), uc, z), fr.toWorld(sIn(z) + depthAt(z), uc, z)});
        if (z >= H) break;
    }
    R.ok = true;
    R.tines = tines;
    R.height = H;
    R.depth = D0;
    R.th = th;
    R.stilt = stilt;
    R.halfW = footHalfW;
    return R;
}

/**
 * Would `rib` run into any of `ribs`? Two tests: the FEET on the plate (flange
 * included), and the RIBS themselves compared level by level up to the shorter one's
 * top. Needs SWAY.clearance of air between them.
 */
bool swayClashes(const SwayRib &rib, const std::vector<SwayRib> &ribs) {
    for (const SwayRib &r : ribs) {
        if (r.foot.empty()) continue;
        if (segDist(rib.foot[0], rib.foot[1], r.foot[0], r.foot[1]) < rib.halfW + r.halfW + SWAY.clearance) return true;
        const double need = (rib.th + r.th) / 2 + SWAY.clearance;
        const double top = jsMin(rib.height, r.height);
        for (const SwayLevel &L : rib.levels) {
            if (L.z > top) break;
            const LevelAB M = levelAt(r.levels, L.z);
            if (segDist(L.a, L.b, M.a, M.b) < need) return true;
        }
    }
    return false;
}

/**
 * Would `rib` run into a support that is already there -- a prop wall or a wedge?
 * Compares the rib's FOOT against each centreline in XY.
 */
bool swayClashesWall(const SwayRib &rib, const std::vector<std::vector<Vec3d>> &walls) {
    if (walls.empty()) return false;
    const double need = rib.halfW + SWAY.wallHalf + SWAY.clearance;
    for (const std::vector<Vec3d> &line : walls) {
        if (line.size() < 1) continue;
        if (line.size() == 1) {
            const Vec3d &p = line[0];
            if (segDist(rib.foot[0], rib.foot[1], p, p) < need) return true;
            continue;
        }
        for (size_t i = 1; i < line.size(); i++) {
            if (segDist(rib.foot[0], rib.foot[1], line[i - 1], line[i]) < need) return true;
        }
    }
    return false;
}

/**
 * AUTO: brace the tallest upright sides of the part, a few faces facing different
 * ways so it is held in both directions.
 */
SwayBraces buildSwayBraces(const Topology &topo, const AnalyzeResult &result, const Rot &rot,
                           const SwayBuildOpts &opts) {
    auto none = [](std::string reason) {
        SwayBraces b;
        b.reason = std::move(reason);
        return b;
    };
    const Soup partTris = printTriangles(topo, rot, result.offset);
    double partTop = 0;
    for (size_t i = 2; i < partTris.size(); i += 3) if (partTris[i] > partTop) partTop = partTris[i];
    if (partTop < SWAY.minPartH) return none("the part is only " + toFixed0(partTop) + "mm tall, too short to sway");

    const double cut = leanCut();
    struct Cand { const WallPatch *p; double score, bearing; };
    const std::vector<WallPatch> patches = findWallPatches(topo, rot, result.offset);
    std::vector<Cand> cands;
    for (const WallPatch &p : patches) {
        if (std::abs(p.n.z) <= cut
            && p.z1 - jsMax(0, p.z0) >= SWAY.minFaceH
            && p.z1 >= SWAY.minTopFrac * partTop)
            cands.push_back({&p, (p.z1 - jsMax(0, p.z0)) * (p.u1 - p.u0), js_atan2(p.n.y, p.n.x)});
    }
    std::stable_sort(cands.begin(), cands.end(), [](const Cand &a, const Cand &b) { return b.score - a.score < 0; });
    if (cands.empty()) return none("no upright side is tall and flat enough to brace in this pose");

    const double sep = (SWAY.minBearingSep * M_PI) / 180;
    auto angGap = [](double a, double b) {
        const double d = std::fmod(std::abs(a - b), 2 * M_PI);
        return jsMin(d, 2 * M_PI - d);
    };
    std::vector<Cand> faces;
    for (const Cand &c : cands) {
        if (int(faces.size()) >= SWAY.maxFaces) break;
        bool close = false;
        for (const Cand &f : faces) if (angGap(f.bearing, c.bearing) < sep) { close = true; break; }
        if (close) continue;
        faces.push_back(c);
    }

    // Auto has usually placed props and wedges before this runs, and a rib that
    // merges with one of them is a support that no longer breaks away in pieces.
    const std::vector<std::vector<Vec3d>> &walls = opts.avoid.walls;
    SwayBraces res;
    std::vector<SwayRib> &ribs = res.ribs;
    Tris &out = res.triangles;
    int tines = 0, skipped = 0;
    for (const Cand &fc : faces) {
        const WallPatch &p = *fc.p;
        for (double u : columnsFor(p)) {
            std::optional<SwayRib> placed;
            for (double du : SWAY.nudges) {
                const double uc = u + du;
                if (uc < p.u0 || uc > p.u1) continue;
                SwayRib r = buildSwayRib(p, uc, partTris, topo, rot, result.offset, opts);
                if (r.ok && !swayClashes(r, ribs) && !swayClashesWall(r, walls)) { placed = std::move(r); break; }
            }
            if (!placed) { skipped++; continue; }
            placed->triRange = TriRange(int(out.size()), int(out.size() + placed->tris.size()));
            for (const Vec3d &v : placed->tris) out.push_back(v);
            res.tags.insert(res.tags.end(), placed->tags.begin(), placed->tags.end());
            tines += placed->tines;
            ribs.push_back(std::move(*placed));
        }
    }
    res.count = int(ribs.size());
    res.tines = tines;
    res.skipped = skipped;
    if (ribs.empty()) res.reason = "the upright sides are blocked by other parts of the model in this pose";
    return res;
}

/** Is this face (by index) upright enough, in this pose, to take a sway brace? */
bool faceIsUpright(const Topology &topo, const Rot &rot, int faceIndex) {
    const size_t i = size_t(faceIndex) * 3;
    const double x = topo.nrm[i], y = topo.nrm[i + 1], z = topo.nrm[i + 2];
    const double nz = rot[2] * x + rot[5] * y + rot[8] * z;
    return std::abs(nz) <= leanCut();
}

namespace {
// Patches per pose, so a click in Draw doesn't re-run the face search.
const SwayPatchCache &patchesFor(const Topology &topo, const Rot &rot, const Offset &offset, SwayPatchCache &c) {
    if (c.topo == &topo && c.rot == rot && c.offset.x == offset.x && c.offset.y == offset.y && c.offset.z == offset.z)
        return c;
    c.reset();
    c.patches = findWallPatches(topo, rot, offset);
    for (size_t k = 0; k < c.patches.size(); k++)
        for (int f : c.patches[k].faces) c.byFace.emplace(f, int(k));   // first patch wins
    c.partTris = printTriangles(topo, rot, offset);
    c.topo = &topo;
    c.rot = rot;
    c.offset = offset;
    return c;
}
} // namespace

/**
 * DRAW: a sway brace on the face the user clicked, at the spot they clicked. Returns
 * buildSwayRib's result.
 */
SwayRib swayAtFace(const Topology &topo, const AnalyzeResult &result, const Rot &rot, int faceIndex,
                   const Vec3d &point, const SwayBuildOpts &optsIn, const SwayAvoid &avoid,
                   SwayPatchCache *cache) {
    // A brace you clicked is a brace you meant, so the stilt limit Auto obeys is
    // advisory here: it builds and reports `stilt` for the readout to mention.
    SwayBuildOpts opts = optsIn;
    if (!opts.allowStilt.has_value()) opts.allowStilt = true;
    SwayPatchCache local;
    const SwayPatchCache &pc = patchesFor(topo, rot, result.offset, cache ? *cache : local);
    const auto it = pc.byFace.find(faceIndex);
    if (it == pc.byFace.end()) return fail("that face is too small or curved to stand a brace against");
    const WallPatch &p = pc.patches[it->second];
    const double u = point[0] * p.u.x + point[1] * p.u.y;
    const std::vector<SwayRib> &braces = avoid.braces;
    const std::vector<std::vector<Vec3d>> &walls = avoid.walls;
    std::optional<SwayRib> last;
    bool hitBrace = false, hitWall = false;
    for (double du : {0.0, 2.0, -2.0, 4.0, -4.0}) {
        const double uc = jsMax(p.u0, jsMin(p.u1, u + du));
        SwayRib r = buildSwayRib(p, uc, pc.partTris, topo, rot, result.offset, opts);
        if (r.ok && swayClashes(r, braces)) { hitBrace = true; continue; }
        if (r.ok && swayClashesWall(r, walls)) { hitWall = true; continue; }
        if (r.ok) return r;
        last = std::move(r);
    }
    if (hitBrace) {
        return fail(std::string("it would run into another brace (on the facing wall, or right beside it) ")
                    + "— click a spot staggered from it");
    }
    if (hitWall) {
        return fail(std::string("a support already stands there, and the two would fuse into one piece ")
                    + "— click a spot clear of it");
    }
    return std::move(*last);
}

}} // namespace Slic3r::SupportFins
