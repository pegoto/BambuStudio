// Port of web/cutout.js -- see Cutout.hpp.
#include "Cutout.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace Slic3r { namespace SupportFins {

thread_local CutConfig CUT;

bool isCutoutPattern(const std::string &name) {
    for (const char *p : CUTOUT_PATTERNS) if (name == p) return true;
    return false;
}

namespace {

using V2 = Vec2d;

/// One entry of HOLES: the LEFT boundary from bottom to top as [half-width, z] with z
/// a fraction of the hole's height (nullopt: set by slope), and the roof share.
struct HoleShape {
    std::vector<std::pair<double, std::optional<double>>> prof;
    std::optional<double> roof;
};

/// HOLES[kind], or nullptr when kind is not one of its keys.
const HoleShape *holeShape(const std::string &kind) {
    static const HoleShape diamond{{{0, 0.0}, {1, 0.5}, {0, 1.0}}, 0.5};
    static const HoleShape triangle{{{1, 0.0}, {0, 1.0}}, 1.0};
    static const HoleShape arch{{{1, 0.0}, {1, std::nullopt}, {0, 1.0}}, std::nullopt}; // null: set by slope
    if (kind == "diamond") return &diamond;
    if (kind == "triangle") return &triangle;
    if (kind == "arch") return &arch;
    return nullptr;
}

/// JS Math.hypot(a, b) exactly as V8 computes it (builtins/math.tq MathHypot):
/// normalise by the larger magnitude, Kahan-sum the squares, sqrt, rescale. It is
/// NOT correctly rounded, so std::hypot (Math.hpp hypot2) differs from it in the
/// last bit -- e.g. hypot(SQRT1_2 step) arclengths of 1 vs 1.0000000000000002 --
/// and a lattice hole flips on that. Kept as separate statements so no compiler
/// fuses n * n - compensation into an FMA.
double jsHypot(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return (std::isinf(a) || std::isinf(b)) ? INF : NAN;
    const double xa = std::fabs(a), xb = std::fabs(b);
    double max = 0;
    if (xa > max) max = xa;
    if (xb > max) max = xb;
    if (max == INF) return INF;
    if (max == 0) return 0;
    double sum = 0, compensation = 0;
    for (const double x : {xa, xb}) {
        const double n = x / max;
        const double nn = n * n;
        const double summand = nn - compensation;
        const double preliminary = sum + summand;
        const double grown = preliminary - sum;
        compensation = grown - summand;
        sum = preliminary;
    }
    return std::sqrt(sum) * max;
}

/// JS Math.min(...v) / Math.max(...v): +/-Infinity for an empty list.
double jsMinOf(const std::vector<double> &v) { double m = INF; for (double x : v) m = std::min(m, x); return m; }
double jsMaxOf(const std::vector<double> &v) { double m = -INF; for (double x : v) m = std::max(m, x); return m; }

/// +x.toFixed(9), then through a Set (which stores -0 as +0). toFixed picks the n
/// nearest x * 1e9 on the EXACT decimal value of x, ties to the larger magnitude
/// ("pick the larger n" after taking |x|); glibc's %.9f breaks exact ties to even,
/// so round from the exact expansion here.
double toFixed9(double x) {
    if (!std::isfinite(x) || std::fabs(x) >= 1e21) return x; // toFixed falls back to ToString(x)
    static thread_local char buf[1200];
    std::snprintf(buf, sizeof(buf), "%.1100f", std::fabs(x)); // exact: a double has <= 1074 fraction digits
    char *dot = std::strchr(buf, '.');
    std::string keep(buf, size_t(dot - buf) + 10);           // integer part, '.', 9 digits
    if (dot[10] >= '5') {                                     // round the magnitude up
        int i = int(keep.size()) - 1;
        for (; i >= 0; i--) {
            if (keep[i] == '.') continue;
            if (keep[i] == '9') { keep[i] = '0'; continue; }
            keep[i]++;
            break;
        }
        if (i < 0) keep.insert(keep.begin(), '1');
    }
    double r = std::strtod(keep.c_str(), nullptr);
    if (x < 0) r = -r;
    return r == 0 ? 0.0 : r;
}

/**
 * The per-cell pattern's hole(s) for one cell, as convex CCW polygons in (s, z):
 * up the right side, back down the left (a pointed tip appears once).
 */
std::vector<Poly2> cellHoles(const std::string &kind, double s0, double s1, double lo, const std::vector<CutHole> &holes) {
    const double sm = (s0 + s1) / 2;
    std::vector<Poly2> res;
    for (const CutHole &h : holes) {
        const double a = h.a;
        const double b0 = lo + h.z0, b1 = lo + h.z1;
        Poly2 prof;
        for (const auto &ft : holeShape(kind)->prof) {
            const double f = ft.first;
            prof.push_back(V2(f * a, !ft.second ? b1 - CUT.slope * a : b0 + *ft.second * (b1 - b0)));
        }
        Poly2 poly;
        for (const V2 &v : prof) poly.push_back(V2(sm + v[0], v[1]));
        for (auto it = prof.rbegin(); it != prof.rend(); ++it) poly.push_back(V2(sm - (*it)[0], (*it)[1]));
        Poly2 kept;
        for (size_t i = 0; i < poly.size(); i++) {
            const V2 &v = poly[i], &u = poly[(i + 1) % poly.size()];
            if (std::fabs(u[0] - v[0]) > 1e-9 || std::fabs(u[1] - v[1]) > 1e-9) kept.push_back(v);
        }
        res.push_back(std::move(kept));
    }
    return res;
}

/** Clip a convex CCW polygon to the half-plane f(v) >= 0 (f linear). */
template <class F> Poly2 clipPoly(const Poly2 &poly, F f) {
    Poly2 out;
    for (size_t i = 0; i < poly.size(); i++) {
        const V2 A = poly[i], B = poly[(i + 1) % poly.size()];
        const double fa = f(A), fb = f(B);
        if (fa >= 0) out.push_back(A);
        if ((fa >= 0) != (fb >= 0)) {
            const double t = fa / (fa - fb);
            out.push_back(V2(A[0] + (B[0] - A[0]) * t, A[1] + (B[1] - A[1]) * t));
        }
    }
    return out;
}

double polyArea(const Poly2 &P) {
    double A = 0;
    for (size_t i = 0; i < P.size(); i++) {
        const double x0 = P[i][0], y0 = P[i][1];
        const double x1 = P[(i + 1) % P.size()][0], y1 = P[(i + 1) % P.size()][1];
        A += x0 * y1 - x1 * y0;
    }
    return A / 2;
}

/** Does every ROOF edge (solid above it) rise at least `k` per unit across? */
bool roofsOk(const Poly2 &P, double k) {
    for (size_t i = 0; i < P.size(); i++) {
        const double x0 = P[i][0], z0 = P[i][1];
        const double x1 = P[(i + 1) % P.size()][0], z1 = P[(i + 1) % P.size()][1];
        const double dx = x1 - x0, dz = z1 - z0;
        // CCW: the hole is on the left, so an edge running LEFTWARD has the hole below
        // it and solid above -- a roof.
        if (dx < -1e-9 && std::fabs(dz) < k * -dx - 1e-9) return false;
    }
    return true;
}

/**
 * LATTICE: diamonds in staggered rows over the whole hole zone (the wall's REAL
 * outline: under the top rail, over the foot rail, between the end posts). At the
 * edges each diamond is clipped to the zone and kept if every roof still rises at
 * CUT.roofMin; a flatter roof is re-peaked, and a diamond that still won't fit is
 * shrunk. Anything smaller than a useful hole is left solid.
 */
std::vector<Poly2> latticeHoles(const std::vector<double> &s, const std::vector<double> &zLo,
                                const std::vector<double> &zHi, double sA, double sB) {
    const double k = CUT.latSlope, p = CUT.latPitch;
    const double inset = CUT.latStrut / 2 / (k / jsHypot(1, k)); // strut half-width, measured flat
    const double a = p / 2 - inset, b = k * a;
    if (a < CUT.minHalf) return {};
    // the zone's edges as half-planes, one per station segment (convex clip is
    // conservative where the outline bends: the hole only ever gets smaller)
    std::vector<size_t> segs;
    for (size_t i = 0; i + 1 < s.size(); i++) {
        if (s[i + 1] <= sA || s[i] >= sB || s[i + 1] - s[i] < 1e-9) continue;
        segs.push_back(i);
    }
    auto line = [&](const std::vector<double> &zs, size_t i) {
        return [&zs, &s, i](const V2 &v) { return zs[i] + (zs[i + 1] - zs[i]) * (v[0] - s[i]) / (s[i + 1] - s[i]); };
    };
    auto fit = [&](const Poly2 &poly) -> std::optional<Poly2> {
        Poly2 P = clipPoly(poly, [&](const V2 &v) { return v[0] - sA; });
        P = clipPoly(P, [&](const V2 &v) { return sB - v[0]; });
        for (size_t i : segs) {
            const double lo = s[i], hi = s[i + 1];
            // a segment only bounds the hole over its own stretch of the wall
            const bool anyR = std::any_of(P.begin(), P.end(), [&](const V2 &v) { return v[0] > lo - 1e-9; });
            const bool anyL = std::any_of(P.begin(), P.end(), [&](const V2 &v) { return v[0] < hi + 1e-9; });
            if (!anyR || !anyL) continue;
            const auto top = line(zHi, i), bot = line(zLo, i);
            P = clipPoly(P, [&](const V2 &v) { return top(v) - v[1]; });
            P = clipPoly(P, [&](const V2 &v) { return v[1] - bot(v); });
            if (P.size() < 3) return std::nullopt;
        }
        // A roof clipped to a zone edge flatter than roofMin would be a bridge. Rather
        // than give the hole up, drop that roof to exactly roofMin from its HIGH
        // corner: the hole keeps its reach along the slope with a printable roof.
        for (int pass = 0; pass < 4 && P.size() >= 3 && !roofsOk(P, CUT.roofMin); pass++) {
            for (size_t n = 0; n < P.size(); n++) {
                const V2 A = P[n], B = P[(n + 1) % P.size()];
                const double dx = B[0] - A[0], dz = B[1] - A[1];
                if (!(dx < -1e-9 && std::fabs(dz) < CUT.roofMin * -dx - 1e-9)) continue;
                if (std::fabs(dz) < 0.25 * -dx) {
                    // a near-level roof: peak it in the middle, a gable rather than a lean-to
                    const V2 M((A[0] + B[0]) / 2, std::min(A[1], B[1]));
                    P = clipPoly(P, [&](const V2 &v) { return M[1] - CUT.roofMin * (v[0] - M[0]) - v[1]; });
                    P = clipPoly(P, [&](const V2 &v) { return M[1] + CUT.roofMin * (v[0] - M[0]) - v[1]; });
                } else {
                    const V2 H = A[1] >= B[1] ? A : B, Lo = A[1] >= B[1] ? B : A;
                    const double d = js_sign(Lo[0] - H[0]);
                    P = clipPoly(P, [&](const V2 &v) { return H[1] - CUT.roofMin * (v[0] - H[0]) * d - v[1]; });
                }
                break;
            }
        }
        if (P.size() < 3 || polyArea(P) < CUT.minArea || !roofsOk(P, CUT.roofMin)) return std::nullopt;
        // Too thin to be a hole: 2*area/perimeter is the inradius of a triangle and a
        // fair stand-in for any convex shape's half-thickness.
        double per = 0;
        for (size_t n = 0; n < P.size(); n++) {
            const V2 &u = P[n], &v = P[(n + 1) % P.size()];
            per += jsHypot(v[0] - u[0], v[1] - u[1]);
        }
        if (2 * polyArea(P) / per < CUT.minHalf / 1.5) return std::nullopt;
        return P;
    };
    std::vector<double> loIn, hiIn;
    for (size_t i = 0; i < zLo.size(); i++) if (s[i] >= sA && s[i] <= sB) loIn.push_back(zLo[i]);
    for (size_t i = 0; i < zHi.size(); i++) if (s[i] >= sA && s[i] <= sB) hiIn.push_back(zHi[i]);
    const double z0 = jsMinOf(loIn);
    const double zTop = jsMaxOf(hiIn);
    const double cols = std::ceil((sB - sA) / p) + 1;
    std::vector<Poly2> holes;
    for (double j = 0; z0 + j * (k * p / 2) - b <= zTop; j++) {
        const double zc = z0 + b * 0.5 + j * (k * p / 2);
        for (double i = -1; i <= cols; i++) {
            const double sm = sA + CUT.latStrut / 2 + a + i * p + (std::fmod(j, 2.0) != 0 ? p / 2 : 0);
            if (sm + a < sA || sm - a > sB) continue;
            for (double f : {1.0, 0.8, 0.62, 0.46, 0.34}) {
                const auto P = fit({V2(sm, zc - b * f), V2(sm + a * f, zc), V2(sm, zc + b * f), V2(sm - a * f, zc)});
                if (P) { holes.push_back(*P); break; }
            }
        }
    }
    return holes;
}

/** A convex polygon's extent [left, right] at height z, or nullopt if it misses. */
std::optional<std::array<double, 2>> spanAt(const Poly2 &P, double z) {
    double lo = INF, hi = -INF;
    for (size_t i = 0; i < P.size(); i++) {
        const double x0 = P[i][0], z0 = P[i][1];
        const double x1 = P[(i + 1) % P.size()][0], z1 = P[(i + 1) % P.size()][1];
        if ((z0 - z) * (z1 - z) > 0) continue;
        if (std::fabs(z1 - z0) < 1e-12) { lo = std::min(lo, std::min(x0, x1)); hi = std::max(hi, std::max(x0, x1)); continue; }
        const double x = x0 + (x1 - x0) * (z - z0) / (z1 - z0);
        lo = std::min(lo, x); hi = std::max(hi, x);
    }
    if (lo <= hi) return std::array<double, 2>{lo, hi};
    return std::nullopt;
}

/**
 * The solid of the strip [s0,s1] x [zb,zt] minus `obst` (convex polygons: the holes,
 * plus the regions above the zone's top edge and below its bottom edge), as convex
 * CCW polygons in (s, z). The strip is cut into horizontal slabs at every obstacle
 * corner and wherever an obstacle edge crosses the strip's side; inside a slab every
 * obstacle edge is one straight line, so the solid is a run of trapezoids between
 * neighbouring obstacles -- always convex.
 */
std::vector<Poly2> stripSolid(double s0, double s1, double zb, double zt, const std::vector<const Poly2 *> &obst) {
    std::vector<const Poly2 *> os;
    for (const Poly2 *Pp : obst) {
        const Poly2 &P = *Pp;
        auto some = [&](auto pred) { return std::any_of(P.begin(), P.end(), pred); };
        if (some([&](const V2 &v) { return v[0] > s0; }) && some([&](const V2 &v) { return v[0] < s1; })
            && some([&](const V2 &v) { return v[1] > zb; }) && some([&](const V2 &v) { return v[1] < zt; }))
            os.push_back(Pp);
    }
    std::vector<double> zs{zb, zt};
    for (const Poly2 *Pp : os) {
        const Poly2 &P = *Pp;
        for (size_t i = 0; i < P.size(); i++) {
            const double x0 = P[i][0], z0 = P[i][1];
            const double x1 = P[(i + 1) % P.size()][0], z1 = P[(i + 1) % P.size()][1];
            zs.push_back(z0);
            for (double side : {s0, s1}) {
                const double t = (side - x0) / (x1 - x0);
                if (x1 != x0 && t > 0 && t < 1) zs.push_back(z0 + t * (z1 - z0));
            }
        }
    }
    // [...new Set(filtered.map(+toFixed(9)))].sort((u, v) => u - v): dedupe then sort
    // ascending, the same unique sorted list as sorting then dropping repeats.
    std::vector<double> cuts;
    for (double z : zs) if (z >= zb && z <= zt) cuts.push_back(toFixed9(z));
    std::stable_sort(cuts.begin(), cuts.end(), [](double u, double v) { return u < v; });
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    auto clampS = [&](double x) { return std::max(s0, std::min(s1, x)); };
    std::vector<Poly2> polys;
    for (size_t n = 0; n + 1 < cuts.size(); n++) {
        const double za = cuts[n], zc = cuts[n + 1];
        if (zc - za < 1e-7) continue;
        const double zm = (za + zc) / 2;
        std::vector<const Poly2 *> act;
        for (const Poly2 *Pp : os) {
            const auto m = spanAt(*Pp, zm);
            if (m && (*m)[1] > s0 && (*m)[0] < s1) act.push_back(Pp);
        }
        std::stable_sort(act.begin(), act.end(), [&](const Poly2 *u, const Poly2 *v) {
            return (*spanAt(*u, zm))[0] - (*spanAt(*v, zm))[0] < 0;
        });
        // The solid's edges in this slab, as functions of z. Every obstacle edge is
        // straight across the slab, so each is read at two interior heights and
        // extended to the slab's ends -- reading AT an end can fall a rounding hair
        // outside an obstacle's corner and miss it.
        const double q1 = za + (zc - za) / 4, q2 = zc - (zc - za) / 4;
        struct Edge { bool constant; double c, m1, m2; };
        auto evalE = [&](const Edge &E, double z) {
            return E.constant ? E.c : clampS(E.m1 + (E.m2 - E.m1) * (z - q1) / (q2 - q1));
        };
        std::vector<Edge> L{{true, s0, 0, 0}}, R;
        for (const Poly2 *Pp : act) {
            auto o1 = spanAt(*Pp, q1); if (!o1) o1 = spanAt(*Pp, zm);
            auto o2 = spanAt(*Pp, q2); if (!o2) o2 = spanAt(*Pp, zm);
            const auto m1 = *o1, m2 = *o2;
            R.push_back({false, 0, m1[0], m2[0]});
            L.push_back({false, 0, m1[1], m2[1]});
        }
        R.push_back({true, s1, 0, 0});
        for (size_t g = 0; g < L.size(); g++) {
            const Poly2 q{V2(evalE(L[g], za), za), V2(evalE(R[g], za), za), V2(evalE(R[g], zc), zc), V2(evalE(L[g], zc), zc)};
            const double w0 = q[1][0] - q[0][0], w1 = q[2][0] - q[3][0];
            if (w0 < 1e-7 && w1 < 1e-7) continue;
            // a zero-width end (a hole's tip) makes the piece a triangle
            polys.push_back(w0 < 1e-7 ? Poly2{q[0], q[2], q[3]} : w1 < 1e-7 ? Poly2{q[0], q[1], q[2]} : q);
        }
    }
    return polys;
}

} // namespace

std::vector<CutHole> holesFor(const std::string &kind, double w, double H) {
    const HoleShape *shape = holeShape(kind);
    double a = (w - CUT.web) / 2;
    if (!shape || a < CUT.minHalf || H < CUT.minHole) return {};
    // The part of the hole height that is roof, per unit of it: the roof must rise
    // slope*a over that share, so the hole needs at least slope*a/roof of height.
    // The arch's straight sides take up whatever height is left over.
    const double roof = shape->roof.value_or(0);
    auto minH = [&](double h) { return roof != 0 ? CUT.slope * h / roof : CUT.slope * h + CUT.minHole / 2; };
    auto maxH = [&](double h) { return std::max(minH(h), 2 * CUT.maxSlope * h); };
    double n = std::max(1.0, std::ceil((H + CUT.web) / (maxH(a) + CUT.web)));
    double hh = (H - (n - 1) * CUT.web) / n;
    if (hh < minH(a) && n > 1) {         // stacking made them squat: one fewer, full width
        n--; hh = (H - (n - 1) * CUT.web) / n;   // (a bit over maxH -- only a steeper roof)
    } else if (hh < minH(a)) {           // a lone hole too squat for its width: narrow it
        a = roof != 0 ? hh * roof / CUT.slope : (hh - CUT.minHole / 2) / CUT.slope;
        if (a < CUT.minHalf) return {};
    }
    std::vector<CutHole> holes;
    for (double j = 0; j < n; j++) {
        const double z0 = j * (hh + CUT.web);
        holes.push_back({a, z0, z0 + hh});
    }
    return holes;
}

bool cutWall(const std::vector<CutStation> &st, const std::vector<Section> &full, Tris &out, const CutWallDims &wall) {
    const std::string &kind = CUT.pattern;
    if (!isCutoutPattern(kind) || kind == "none" || double(st.size()) < double(wall.minStations)) return false;
    // arclength of each station along the wall, in XY
    std::vector<double> s{0};
    for (size_t i = 1; i < st.size(); i++)
        s.push_back(s[i - 1] + jsHypot(st[i].p[0] - st[i - 1].p[0], st[i].p[1] - st[i - 1].p[1]));
    const double L = s.back();
    const double nCells = std::floor((L - 2 * CUT.post) / CUT.pitch);
    if (nCells < 1) return false;
    // Cells: strips between stations about CUT.pitch wide, snapped to stations so
    // each strip's edges sit on real cross-sections.
    auto snap = [&](double x) {
        size_t best = 0;
        for (size_t i = 1; i < s.size(); i++) if (std::fabs(s[i] - x) < std::fabs(s[best] - x)) best = i;
        return best;
    };
    // The cells share the length between the two end posts evenly, so the posts
    // stay CUT.post wide instead of soaking up the remainder.
    std::vector<size_t> edges;
    for (double c = 0; c <= nCells; c++) {
        const size_t i = snap(CUT.post + c * (L - 2 * CUT.post) / nCells);
        if (edges.empty() || i > edges.back()) edges.push_back(i);
    }
    if (edges.size() < 2) return false;
    // The hole zone at each station: under the top rail, over the foot rail. Where
    // the wall is too short for any zone the two meet in the middle.
    std::vector<double> zHi, zLo;
    for (const CutStation &q : st) {
        // under a flat interface crest ztip is the crest's cut; the holes still stop
        // where the wall's taper began (holeTop), so the crest takes no plastic from them
        const double hi = q.holeTop.value_or(q.ztip) - CUT.rail, lo = q.botTip + CUT.rail;
        const double m = (hi + lo) / 2;
        zHi.push_back(hi > lo ? hi : m);
        zLo.push_back(hi > lo ? lo : m);
    }
    const double sA = s[edges.front()], sB = s[edges.back()];
    std::vector<Poly2> holes;
    if (kind == "lattice") {
        holes = latticeHoles(s, zLo, zHi, sA, sB);
    } else {
        for (size_t c = 0; c + 1 < edges.size(); c++) {
            const size_t i0 = edges[c], i1 = edges[c + 1];
            double hi = INF, lo = -INF;
            for (size_t i = i0; i <= i1; i++) { hi = std::min(hi, zHi[i]); lo = std::max(lo, zLo[i]); }
            for (Poly2 &h : cellHoles(kind, s[i0], s[i1], lo, holesFor(kind, s[i1] - s[i0], hi - lo)))
                holes.push_back(std::move(h));
        }
    }
    if (holes.empty()) return false;
    // Not worth it for a hole or two: a wall whose holes would take out less than
    // CUT.minSaved of its face stays solid.
    double face = 0, open = 0;
    for (size_t i = 0; i + 1 < st.size(); i++)
        face += (s[i + 1] - s[i]) * ((st[i].top - st[i].bot) + (st[i + 1].top - st[i + 1].bot)) / 2;
    for (const Poly2 &H : holes) open += polyArea(H);
    if (open < CUT.minSaved * face) return false;

    // Bands: the top band reaches down to the zone's top edge, the bottom band up
    // to its bottom edge, station by station -- only between the end posts.
    const double th = wall.th / 2, tp = wall.tip / 2;
    std::vector<Section> top, bot;
    for (size_t i = edges.front(); i <= edges.back(); i++) {
        const CutStation &q = st[i];
        auto P = [&](double o, double z) { return Vec3d(q.p[0] + q.sx * o, q.p[1] + q.sy * o, z); };
        // keep each band inside the wall's own profile
        const double ct = std::max(q.botTip, std::min(zHi[i], q.ztip - 0.01));
        const double cb = std::min(q.ztip, std::max(zLo[i], q.botTip + 0.01));
        const double tq = q.tipHalf.value_or(tp); // narrower when an interface crest sits on it
        top.push_back({P(+th, ct), P(+th, q.ztip), P(+tq, q.top), P(-tq, q.top), P(-th, q.ztip), P(-th, ct)});
        bot.push_back(q.taperBot
            ? Section{P(+tp, q.bot), P(+th, q.botTip), P(+th, cb), P(-th, cb), P(-th, q.botTip), P(-tp, q.bot)}
            : Section{P(+th, q.botP.value_or(q.bot)), P(+th, cb), P(-th, cb), P(-th, q.botN.value_or(q.bot))}); // a tilted lifted bottom (sweepBetween)
    }
    ribbon(top, out);
    ribbon(bot, out);
    // End posts: the caller's full sections outside the cut span.
    if (edges.front() > 0) ribbon(std::vector<Section>(full.begin(), full.begin() + std::min(full.size(), edges.front() + 1)), out);
    if (edges.back() < st.size() - 1) ribbon(std::vector<Section>(full.begin() + std::min(full.size(), edges.back()), full.end()), out);

    // The strips: zone minus holes, mapped from (s, z) onto the wall. Across-wall
    // direction and XY position are interpolated between stations.
    struct At { double x, y, sx, sy; };
    auto at = [&](double x) {
        size_t i = 0;
        while (i < s.size() - 2 && s[i + 1] < x) i++;
        const double t = std::max(-1.0, std::min(2.0, (x - s[i]) / std::max(1e-9, s[i + 1] - s[i])));
        const CutStation &A = st[i], &B = st[i + 1];
        auto lerp = [&](double u, double v) { return u + (v - u) * t; };
        double sx = lerp(A.sx, B.sx), sy = lerp(A.sy, B.sy);
        const double n = jsHypot(sx, sy); sx /= n; sy /= n;
        return At{lerp(A.p[0], B.p[0]), lerp(A.p[1], B.p[1]), sx, sy};
    };
    const double e = CUT.eps;
    const double zMin = jsMinOf(zLo) - 1, zMax = jsMaxOf(zHi) + 1;
    // Beyond the zone's top and bottom edges, per station segment: obstacles like
    // the holes, pushed eps INTO the bands so the strips overlap them.
    std::vector<Poly2> beyond;
    for (size_t i = edges.front(); i < edges.back(); i++) {
        if (s[i + 1] - s[i] < 1e-9) continue;
        beyond.push_back({V2(s[i], zHi[i] + e), V2(s[i + 1], zHi[i + 1] + e), V2(s[i + 1], zMax + 1), V2(s[i], zMax + 1)});
        beyond.push_back({V2(s[i], zMin - 1), V2(s[i + 1], zMin - 1), V2(s[i + 1], zLo[i + 1] - e), V2(s[i], zLo[i] - e)});
    }
    std::vector<const Poly2 *> obst;
    for (const Poly2 &H : holes) obst.push_back(&H);
    for (const Poly2 &B : beyond) obst.push_back(&B);
    auto inHole = [&](double u, double v) {
        return std::any_of(holes.begin(), holes.end(), [&](const Poly2 &P) {
            const auto m = spanAt(P, v);
            return m && u > (*m)[0] && u < (*m)[1];
        });
    };
    const Frame P = [&](double u, double v, double o) {
        const At f = at(u);
        return Vec3d(f.x + f.sx * o, f.y + f.sy * o, v);
    };
    for (size_t c = 0; c + 1 < edges.size(); c++) {
        const double s0 = s[edges[c]], s1 = s[edges[c + 1]];
        double zb = INF, zt = -INF;
        for (size_t i = edges[c]; i <= edges[c + 1]; i++) { zb = std::min(zb, zLo[i] - e); zt = std::max(zt, zHi[i] + e); }
        for (const Poly2 &poly : stripSolid(s0, s1, zb, zt, obst)) {
            // grow the piece by eps across the strip's sides so it overlaps the posts /
            // next strip -- but never into a hole that straddles the side (the lattice's
            // do). A corner on the side is slid ALONG its inward edge (not straight
            // across), so a slanted hole edge stays exactly where it is.
            Poly2 grown;
            const size_t np = poly.size();
            for (size_t n = 0; n < np; n++) {
                const V2 &V = poly[n];
                const double u = V[0], v = V[1];
                if (u != s0 && u != s1) { grown.push_back(V); continue; }
                std::vector<V2> nb;
                for (const V2 &W : {poly[(n + np - 1) % np], poly[(n + 1) % np]}) if (W[0] != u) nb.push_back(W);
                if (nb.size() != 1) { grown.push_back(V); continue; } // a tip touching the side: leave it
                const V2 W = nb[0];
                const double k = e / std::fabs(W[0] - u);
                const V2 G(u + (u - W[0]) * k, v + (v - W[1]) * k);
                grown.push_back(inHole(G[0], G[1]) ? V : G);
            }
            boxExtrude(grown, -th, th, P, out);
        }
    }
    return true;
}

}} // namespace Slic3r::SupportFins
