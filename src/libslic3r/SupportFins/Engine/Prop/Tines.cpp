// Port of web/prop/tines.js -- grip tines. See Tines.hpp.
#include "Tines.hpp"
#include "../Inside.hpp"
#include "../Kiss.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace Slic3r { namespace SupportFins {

thread_local std::vector<TineCap> *TINECAP = nullptr;

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
// JS truthiness of a number
inline bool truthy(double v) { return v != 0 && !std::isnan(v); }

struct Dir { double x, y; };

/** Squared distance from point p to triangle (a,b,c). Ericson closest-point. */
double ptTriDist2(const Vec3d &p, const Vec3d &a, const Vec3d &b, const Vec3d &c) {
    auto sub = [](const Vec3d &u, const Vec3d &v) { return Vec3d(u[0] - v[0], u[1] - v[1], u[2] - v[2]); };
    auto dot = [](const Vec3d &u, const Vec3d &v) { return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; };
    const Vec3d ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
    const double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return dot(ap, ap);
    const Vec3d bp = sub(p, b);
    const double d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return dot(bp, bp);
    const Vec3d cp = sub(p, c);
    const double d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return dot(cp, cp);
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const double v = d1 / (d1 - d3);
        const Vec3d q(a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2]);
        const Vec3d w = sub(p, q); return dot(w, w);
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const double w = d2 / (d2 - d6);
        const Vec3d q(a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2]);
        const Vec3d u = sub(p, q); return dot(u, u);
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        const Vec3d q(b[0] + w * (c[0] - b[0]), b[1] + w * (c[1] - b[1]), b[2] + w * (c[2] - b[2]));
        const Vec3d u = sub(p, q); return dot(u, u);
    }
    const double denom = 1 / (va + vb + vc), v = vb * denom, w = vc * denom;
    const Vec3d q(a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w);
    const Vec3d u = sub(p, q); return dot(u, u);
}

/**
 * Horizontal INWARD-normal of the part face nearest a tine seed -- the direction a
 * tine must bite to grip "straight on". The heading comes from the PART, never from
 * the wall's run. Every face tied for nearest (within best*1e-9 + 1e-12: at an inside
 * corner two faces can be exactly equidistant, and float noise picked one) is
 * returned, by index; a face too flat to grip sideways (|horizontal normal| < 0.34)
 * is skipped. Unit horizontal {x, y} each.
 */
std::vector<Dir> biteDirsAt(const Topology &topo, const Rot &rot, const Offset &offset, double px, double py, double pz) {
    const Soup &pos = topo.pos;
    const std::vector<double> &nrm = topo.nrm;
    const int nF = topo.nFaces;
    const Vec3d P(px, py, pz);
    auto seatAt = [&](size_t o) { return seat(rot, offset, pos[o], pos[o + 1], pos[o + 2]); };
    std::vector<double> d2(nF > 0 ? nF : 0);
    double best = INF;
    for (int f = 0; f < nF; f++) {
        const size_t o = size_t(f) * 9;
        d2[f] = ptTriDist2(P, seatAt(o), seatAt(o + 3), seatAt(o + 6));
        if (d2[f] < best) best = d2[f];
    }
    if (!(best < INF)) return {};
    const double tol = best * 1e-9 + 1e-12;
    std::vector<Dir> dirs;
    for (int f = 0; f < nF; f++) {
        if (d2[f] > best + tol) continue;
        const double nx = nrm[f * 3], ny = nrm[f * 3 + 1], nz = nrm[f * 3 + 2];
        // seated normal, then INWARD (into the part) = negated, horizontal component only
        const double sx = rot[0] * nx + rot[3] * ny + rot[6] * nz;
        const double sy = rot[1] * nx + rot[4] * ny + rot[7] * nz;
        const double hx = -sx, hy = -sy, hm = hypot2(hx, hy);
        if (hm < 0.34) continue;                  // face too flat (near-horizontal ceiling) to grip sideways
        dirs.push_back({hx / hm, hy / hm});
    }
    return dirs;
}

/**
 * Square the part's bite headings to the wall: each becomes the nearer of the wall's
 * four axes (along the run either way, or straight across either way), nearest first,
 * then the next-nearer axis as a fallback; only axes within 90 degrees of the heading
 * (dot > 1e-6). The part's own headings come last (an angled tine still beats losing
 * the base grip).
 */
std::vector<Dir> squareToRun(const std::vector<Dir> &dirs, const std::vector<Vec3d> &line, long k) {
    // the run at station k: its own segment, or the nearest non-degenerate one
    // (a duplicated station has zero length)
    double rx = 0, ry = 0;
    const long nSeg = long(line.size()) - 1;
    for (long j = 0; j < nSeg && !(truthy(rx) || truthy(ry)); j++) {
        for (const long i : {k + j, k - j}) {
            if (i < 0 || i >= nSeg) continue;
            const double dx = line[i + 1][0] - line[i][0], dy = line[i + 1][1] - line[i][1];
            if (hypot2(dx, dy) > 1e-9) { rx = dx; ry = dy; break; }
        }
    }
    const double rn = hypot2(rx, ry);
    if (rn < 1e-9) return dirs;
    rx /= rn; ry /= rn;
    const double axes[4][2] = {{rx, ry}, {-rx, -ry}, {-ry, rx}, {ry, -rx}};
    struct Ranked { int i; double x, y, dot; };
    std::vector<Dir> out;
    std::unordered_set<int> seen;
    for (const Dir &c : dirs) {
        std::vector<Ranked> ranked;
        for (int i = 0; i < 4; i++) {
            const double ux = axes[i][0], uy = axes[i][1];
            const Ranked u{i, ux, uy, ux * c.x + uy * c.y};
            if (u.dot > 1e-6) ranked.push_back(u);
        }
        // JS .sort((p, q) => q.dot - p.dot): stable, descending (no NaN survives the filter)
        std::stable_sort(ranked.begin(), ranked.end(), [](const Ranked &p, const Ranked &q) { return q.dot - p.dot < 0; });
        for (const Ranked &u : ranked) {
            if (seen.count(u.i)) continue;       // tied faces often rank the same axis
            seen.insert(u.i);
            out.push_back({u.x, u.y});
        }
    }
    out.insert(out.end(), dirs.begin(), dirs.end());
    return out;
}

int comb(const std::vector<Vec3d> &line, const Topology &topo, const Rot &rot, const Offset &offset, Tris &out,
         double stepArg, double minTop, double tineH, const std::optional<TineBody> &body, TineGrip *grip) {
    if (line.size() < 2) return 0;

    // arc length along the run, to space nubs by a real distance not a station count
    std::vector<double> s{0};
    for (size_t i = 1; i < line.size(); i++)
        s.push_back(s[i - 1] + hypot2(line[i][0] - line[i - 1][0], line[i][1] - line[i - 1][1]));
    // JS s[i] past the end is undefined -> NaN in arithmetic
    auto sAt = [&](long i) { return i >= 0 && size_t(i) < s.size() ? s[size_t(i)] : NaN; };
    // `body` = [i0, i1], the station range of a tall wall's full-height BODY. It only
    // sizes the spacing below (the minGripTines floor counts the body); the comb runs
    // the whole wall.
    const double s0 = body ? sAt((*body)[0]) : 0;
    const double s1 = body ? sAt((*body)[1]) : s[s.size() - 1];
    const double total = s1 - s0;
    if (!(total > 0)) return 0;

    // The caller's step encodes tip-over risk; a wall still gets at least minGripTines
    // along its length (min() only ever TIGHTENS). PROP.tinesPerWall (calibration)
    // asks for exactly n, spaced over the WHOLE run.
    const double step = PROP.tinesPerWall > 0 ? s[s.size() - 1] / PROP.tinesPerWall
        : jsMin(stepArg, total / PROP.minGripTines);
    if (total < step && !(PROP.tinesPerWall > 0)) return 0;

    // half the tine's WIDTH across the run -- one nozzle bead (PROP.tineW)
    const double half = PROP.tineW / 2;

    int count = 0;
    // Place ONE tine at arc length d0 (relative to the body start s0); true if it landed.
    auto place = [&](double d0) -> bool {
        const double d = d0 + s0;                     // back onto the full run's arc length
        // interpolate the station at arc length d
        size_t k = 0;
        while (k < s.size() - 1 && s[k + 1] < d) k++;
        const double seg = jsMax(1e-9, sAt(long(k) + 1) - s[k]);
        const double f = (d - s[k]) / seg;
        // JS: line[k + 1][0] with k + 1 == line.length is a TypeError
        if (k + 1 >= line.size()) throw std::out_of_range("emitTines: station past the end of the line");
        const double x = line[k][0] + (line[k + 1][0] - line[k][0]) * f;
        const double y = line[k][1] + (line[k + 1][1] - line[k][1]) * f;
        const double z = line[k][2] + (line[k + 1][2] - line[k][2]) * f;   // surface z
        const double wallTop = z - PROP.gap;
        if (wallTop < minTop) return false;   // below the wall's base (flange or brim): no face to attach to

        // LAYER-SNAP so the tine prints as exactly ONE bead: its span snapped to the
        // NEAREST grid line of a plate-origin layer grid. The probe below still uses
        // the underside level (zMid), so PLACEMENT is unchanged.
        const double tineTop = js_round(z / tineH) * tineH;
        const double tineBot = tineTop - tineH;
        const double zMid = z - tineH / 2;

        // BITE DIRECTION from the PART, squared to the wall's nearest axis; the nub's
        // full reach must land inside the part, or no tine.
        const std::vector<Dir> cands = squareToRun(biteDirsAt(topo, rot, offset, x, y, zMid), line, long(k));
        const Dir *bd = nullptr;
        for (const Dir &c : cands)
            if (insidePart(topo, rot, offset, x + c.x * PROP.tineReach, y + c.y * PROP.tineReach, zMid)) { bd = &c; break; }
        if (!bd) return false;
        const double dirx = bd->x, diry = bd->y;

        // frame (along = bite dir, across = z x along, up = z) is right-handed
        const double ax = -diry, ay = dirx;                                // across = z x along
        const double base[3] = {x, y, 0};
        const Frame P = [&](double a, double b, double c) {
            return Vec3d(base[0] + dirx * a + ax * b, base[1] + diry * a + ay * b, c);
        };
        // quad in (along, across): from -overlap (into the wall) to where the tine meets
        // the part, its end leaning with the surface between bottom and top (kissEnds).
        const KissEnds e = kissEnds(topo, rot, offset, x, y, dirx, diry, tineBot, tineTop,
                                    KissOpts{half, PROP.tineOverlap, PROP.tineReach});
        const double ov = -PROP.tineOverlap;
        auto outline = [&](const std::array<double, 2> &eh) {
            return Poly2{Vec2d(ov, -half), Vec2d(eh[0], -half), Vec2d(eh[1], half), Vec2d(ov, half)};
        };
        loftExtrude(outline(e.bot), outline(e.top), tineBot, tineTop, P, out);
        if (grip && tineBot < grip->z) grip->z = tineBot;
        // WALL STEP. The tine is snapped to the part's layer, the wall top isn't, so the
        // tine can start above the wall; past half a layer the slice under it comes out
        // empty. Raise the wall under the tine's wall end only (-tineOverlap..0), from
        // half a layer inside the wall to half a layer inside the tine, inset STEP_INSET.
        if (tineBot - wallTop >= tineH / 2 - 1e-6) {
            const double ei = STEP_INSET;
            const Poly2 stepPoly{Vec2d(-PROP.tineOverlap + ei, -half + ei), Vec2d(-ei, -half + ei),
                                 Vec2d(-ei, half - ei), Vec2d(-PROP.tineOverlap + ei, half - ei)};
            boxExtrude(stepPoly, wallTop - tineH / 2, tineBot + tineH / 2, P, out);
        }
        // Test seam (JS globalThis.__TINECAP)
        if (TINECAP) TINECAP->push_back({x, y, zMid, dirx, diry});
        count++;
        return true;
    };
    // ONE COMB, LAID UP FROM THE BOTTOM: anchored at the lowest point a nub grips, then
    // stepping up the whole run; no nub within tineTopClear of the top end.
    // EDGE-BIASED spacing: dense (`step`) within tineEdgeBand of either end, thinned
    // (`step * tineMidFactor`, never past tineStepSparse) across the middle.
    const double S = s[s.size() - 1];
    // Ends level to within float noise keep the line's own order (local issue 023).
    const bool lowFirst = line[0][2] <= line[line.size() - 1][2] + 1e-6;
    auto at = [&](double u) { return place((lowFirst ? u : S - u) - s0); };   // u = arc length from the LOW end
    const double band = jsMin(PROP.tineEdgeBand, S / 2);
    const double midStep = PROP.tinesPerWall > 0 ? step     // an exact count: even, no edge bias
        : jsMin(step * PROP.tineMidFactor, PROP.tineStepSparse);
    const double uTop = S - jsMin(step / 2, PROP.tineTopClear);
    const double zLo = jsMin(line[0][2], line[line.size() - 1][2]);
    const double zHi = jsMax(line[0][2], line[line.size() - 1][2]);
    if (!body && zHi - zLo < PROP.tineSlopeMin) {
        // A LEVEL line with no tail (most wedges, squat walls): the plain comb.
        for (double d = step / 2; d < S;) {
            place(d);
            d += jsMin(d, S - d) <= band ? step : midStep;
        }
        return count;
    }
    // Start and scan scale down with the step on a very short run.
    const double scan = jsMin(PROP.tineW, step / 2);
    double u = jsMin(half, step / 2);
    while (u <= uTop && !at(u)) u += scan;
    // Dense while this nub OR the next sparse one sits in an end band.
    for (;;) {
        const bool dense = jsMin(u, S - u) <= band || S - (u + midStep) <= band;
        u += dense ? step : midStep;
        if (u > uTop) break;
        at(u);
    }
    return count;
}

} // namespace

double tineStepFor(std::optional<double> density) {
    const double d = jsMax(0, jsMin(1, density ? *density : 1));
    return PROP.tineStepSparse - d * (PROP.tineStepSparse - PROP.tineStep);
}

int emitTines(const std::vector<Vec3d> &line, const SurfTris * /*tris*/, const Topology &topo, const Rot &rot,
              const Offset &offset, Tris &out, std::optional<double> stepArg, std::optional<double> minTop,
              std::optional<double> tineH, const std::optional<TineBody> &body, TineGrip *grip, IfaceTags *tags) {
    // with the interface crest on everywhere ('all'), the whole comb (tines and wall
    // steps) prints in the interface material. Not under 'flat' (prop/crest.js).
    const size_t from = out.size();
    // JS defaulted parameters, evaluated at call time
    const int n = comb(line, topo, rot, offset, out,
                       stepArg ? *stepArg : PROP.tineStep,
                       minTop ? *minTop : PROP.baseH + 0.2,
                       tineH ? *tineH : PROP.tineH, body, grip);
    if (crestOn() && PROP.iface == Iface::All) markInterface(out, from, tags);
    return n;
}

}} // namespace
