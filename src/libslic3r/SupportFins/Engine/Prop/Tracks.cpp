// Port of web/prop/tracks.js. See Tracks.hpp; the reasoning behind each step (and the
// parts that measured it) is in the upstream file.
#include "Tracks.hpp"
#include "../Inside.hpp"
#include "../Planes.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_map>

#if __has_include("../Ieee754.hpp")
#include "../Ieee754.hpp"
#define SF_TRACKS_COS js_cos
#define SF_TRACKS_TAN js_tan
#else
#define SF_TRACKS_COS std::cos
#define SF_TRACKS_TAN std::tan
#endif

namespace Slic3r { namespace SupportFins {

namespace {
// JS Math.min / Math.max of two: NaN if either is NaN.
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    if (a == b) return std::signbit(a) ? a : b;   // min(0, -0) = -0
    return a < b ? a : b;
}
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    if (a == b) return std::signbit(a) ? b : a;   // max(0, -0) = 0
    return a > b ? a : b;
}
inline Vec3d rotN(const Rot &rot, double x, double y, double z) {
    return Vec3d(rot[0] * x + rot[3] * y + rot[6] * z,
                 rot[1] * x + rot[4] * y + rot[7] * z,
                 rot[2] * x + rot[5] * y + rot[8] * z);
}
} // namespace

// ---------------------------------------------------------------------------------
std::vector<PropPatch> splitRegion(const Topology &topo, const std::vector<int> &faces, const Rot &rot) {
    const auto &nrm = topo.nrm;
    const auto &area = topo.area;
    const double agreeCut = SF_TRACKS_COS((PROP.splitAgreeDeg * M_PI) / 180);
    const FaceAdjacency adj = faceAdjacency(topo);
    const auto &start = adj.start;
    const auto &nbr = adj.nbr;

    // rotated normals, for this region's faces only
    std::unordered_map<int, Vec3d> rn;
    rn.reserve(faces.size() * 2);
    for (int f : faces) {
        const double x = nrm[size_t(f) * 3], y = nrm[size_t(f) * 3 + 1], z = nrm[size_t(f) * 3 + 2];
        rn[f] = rotN(rot, x, y, z);
    }

    std::vector<int> order(faces);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return area[size_t(b)] - area[size_t(a)] < 0; });
    std::vector<uint8_t> assigned(size_t(topo.nFaces), 0);
    std::vector<PropPatch> patches;

    for (int seed : order) {
        if (assigned[size_t(seed)]) continue;
        const Vec3d s = rn.at(seed);
        const double sx = s[0], sy = s[1], sz = s[2];
        assigned[size_t(seed)] = 1;
        std::vector<int> members{seed};
        double total = area[size_t(seed)];
        std::vector<int> queue{seed};

        while (!queue.empty()) {
            const int f = queue.back();
            queue.pop_back();
            for (int e = start[size_t(f)]; e < start[size_t(f) + 1]; e++) {
                const int g = nbr[size_t(e)];
                if (assigned[size_t(g)]) continue;
                const auto it = rn.find(g);   // absent = not in this region
                if (it == rn.end()) continue;
                const Vec3d &gn = it->second;
                if (gn[0] * sx + gn[1] * sy + gn[2] * sz < agreeCut) continue;
                assigned[size_t(g)] = 1;
                members.push_back(g);
                total += area[size_t(g)];
                queue.push_back(g);
            }
        }
        PropPatch p;
        p.faces = std::move(members);
        p.area = total;
        patches.push_back(std::move(p));
    }
    return patches;
}

// ---------------------------------------------------------------------------------
std::optional<std::vector<ContactLine>> tubeLine(const Topology &topo, const std::vector<int> &faces, const Rot &rot,
                                                 const std::vector<Vec3d> &pts, const SurfTris &regionTris,
                                                 double step) {
    const auto &nrm = topo.nrm;
    const auto &area = topo.area;

    double regionArea = 0;
    for (int f : faces) regionArea += area[size_t(f)];
    // Below tubeMinArea only a CONVEX tube may take this route (checked once the
    // lowest line is known, below).
    const bool small = regionArea < PROP.tubeMinArea;
    if (small && regionArea < PROP.tubeSmallMinArea) return std::nullopt;

    // area-weighted mean normal, in print space
    double mx = 0, my = 0, mz = 0, A = 0;
    AreaNormals rn;
    rn.reserve(faces.size());
    for (int f : faces) {
        const double x = nrm[size_t(f) * 3], y = nrm[size_t(f) * 3 + 1], z = nrm[size_t(f) * 3 + 2];
        const Vec3d v = rotN(rot, x, y, z);
        const double a = area[size_t(f)];
        rn.emplace_back(v, a);
        A += a;
        mx += v[0] * a; my += v[1] * a; mz += v[2] * a;
    }
    const double mn = hypot3(mx, my, mz);
    if (mn < 1e-9 || A < 1e-9) return std::nullopt;
    mx /= mn; my /= mn; mz /= mn;

    // The FRACTION of area that deviates, never the worst face.
    const double cut = SF_TRACKS_COS((PROP.tubeSpreadDeg * M_PI) / 180);
    double deviant = 0;
    for (const auto &[v, a] : rn) {
        if (v[0] * mx + v[1] * my + v[2] * mz < cut) deviant += a;
    }
    if (deviant / A < PROP.tubeCurvedFrac) return std::nullopt;   // flat: rows handle it

    // The lowest line, from the mesh's own points.
    double dLo[2] = {INF, INF}, dHi[2] = {-INF, -INF};
    for (const Vec3d &p : pts) {
        for (int a = 0; a < 2; a++) {
            if (p[a] < dLo[a]) dLo[a] = p[a];
            if (p[a] > dHi[a]) dHi[a] = p[a];
        }
    }
    const double diag = hypot2(dHi[0] - dLo[0], dHi[1] - dLo[1]);
    const double nSamplesD = jsMax(8, jsMin(400, std::ceil(diag / step)));
    // JS passes NaN on through to contactLine for a NaN diag (NaN points); it has no
    // int representation, and no real region gets here with one.
    const int nSamples = std::isnan(nSamplesD) ? 0 : int(nSamplesD);
    // A small tube is often meshed as a few long strips: sample the edges too.
    const std::optional<ContactLine> rough =
        contactLine(small ? edgePoints(regionTris.pos, pts, step / 2) : pts, regionTris, nSamples);
    if (!rough || straightness(*rough) > PROP.maxWander) return std::nullopt;   // a ring, not a tube

    // RESAMPLE it the way patchTracks samples a track.
    double cx = 0, cy = 0;
    for (const Vec3d &p : *rough) { cx += p[0]; cy += p[1]; }
    cx /= double(rough->size()); cy /= double(rough->size());
    double sxx = 0, sxy = 0, syy = 0;
    for (const Vec3d &p : *rough) {
        const double dx = p[0] - cx, dy = p[1] - cy;
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    const double tr2 = sxx + syy, det = sxx * syy - sxy * sxy;
    const double lam = tr2 / 2 + std::sqrt(jsMax(0, (tr2 * tr2) / 4 - det));
    double ux = sxy, uy = lam - sxx;
    if (hypot2(ux, uy) < 1e-9) { ux = 1; uy = 0; }
    const double un = hypot2(ux, uy); ux /= un; uy /= un;
    double uLo = INF, uHi = -INF;
    for (const Vec3d &p : *rough) {
        const double u = (p[0] - cx) * ux + (p[1] - cy) * uy;
        if (u < uLo) uLo = u;
        if (u > uHi) uHi = u;
    }
    if (uHi - uLo < 1e-6) return std::nullopt;
    if (small && !convexAbout(rn, regionTris.pos, cx, cy, ux, uy)) return std::nullopt;   // a pocket
    const double nSt = jsMax(2, jsMin(400, std::ceil((uHi - uLo) / step)));

    std::vector<ContactLine> lines;
    ContactLine cur;
    for (int k = 0; k <= nSt; k++) {
        const double u = uLo + ((uHi - uLo) * k) / nSt;
        const double x = cx + ux * u, y = cy + uy * u;
        const std::optional<double> z = surfaceZAt(regionTris, x, y);
        if (!z) {
            if (!cur.empty()) { lines.push_back(std::move(cur)); cur.clear(); }
        } else {
            cur.push_back(Vec3d(x, y, *z));
        }
    }
    if (!cur.empty()) lines.push_back(std::move(cur));
    std::vector<ContactLine> kept;
    for (auto &t : lines)
        if (double(t.size()) >= PROP.minStations) kept.push_back(std::move(t));
    return kept;
}

// ---------------------------------------------------------------------------------
std::vector<Vec3d> edgePoints(const Soup &tris, const std::vector<Vec3d> &pts, double gap) {
    std::vector<Vec3d> out = pts;
    for (size_t t = 0; t < tris.size(); t += 9) {
        for (size_t e = 0; e < 3; e++) {
            const size_t a = t + e * 3, b = t + ((e + 1) % 3) * 3;
            // UPSTREAM reads past a partial last face as undefined -> NaN; tris here
            // are always whole faces.
            const double len = hypot3(tris[b] - tris[a], tris[b + 1] - tris[a + 1], tris[b + 2] - tris[a + 2]);
            const double n = std::ceil(len / gap);
            for (int k = 1; k < n; k++) {
                const double u = k / n;
                out.push_back(Vec3d(tris[a] + (tris[b] - tris[a]) * u, tris[a + 1] + (tris[b + 1] - tris[a + 1]) * u,
                                    tris[a + 2] + (tris[b + 2] - tris[a + 2]) * u));
            }
        }
    }
    return out;
}

// ---------------------------------------------------------------------------------
bool convexAbout(const AreaNormals &rn, const Soup &regionTris, double cx, double cy, double ux, double uy) {
    const double wx = -uy, wy = ux;                  // across the line, in plan
    double away = 0, toward = 0, left = 0, right = 0;
    for (size_t k = 0; k < rn.size(); k++) {
        const size_t t = k * 9;
        const double gx = (regionTris[t] + regionTris[t + 3] + regionTris[t + 6]) / 3;
        const double gy = (regionTris[t + 1] + regionTris[t + 4] + regionTris[t + 7]) / 3;
        const double d = (gx - cx) * wx + (gy - cy) * wy;   // which side, how far
        if (std::abs(d) < 0.05) continue;                    // on the line: no vote
        const Vec3d &n = rn[k].first;
        const double a = rn[k].second;
        // JS Math.sign(NaN) is NaN; js_sign gives 0. Either way s > 0 is false.
        const double s = (n[0] * wx + n[1] * wy) * js_sign(d);
        if (s > 0) away += a; else toward += a;
        if (d > 0) left += a; else right += a;
    }
    return away > PROP.tubeConvexFrac * (away + toward)
        && jsMin(left, right) > PROP.tubeTwoSidedFrac * (left + right);
}

// ---------------------------------------------------------------------------------
Tracks patchTracks(const std::vector<Vec3d> &pts, const SurfTris &patchTris, double step, const Support *support,
                   double span, const std::optional<Vec2d> &axis) {
    if (pts.empty()) return Tracks{};

    // The 2x2 XY covariance of the patch, plus its cross-terms with Z.
    double cx = 0, cy = 0, cz = 0;
    for (const Vec3d &p : pts) { cx += p[0]; cy += p[1]; cz += p[2]; }
    cx /= double(pts.size()); cy /= double(pts.size()); cz /= double(pts.size());
    double sxx = 0, sxy = 0, syy = 0, sxz = 0, syz = 0;
    for (const Vec3d &p : pts) {
        const double dx = p[0] - cx, dy = p[1] - cy, dz = p[2] - cz;
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
        sxz += dx * dz; syz += dy * dz;
    }
    const double det = sxx * syy - sxy * sxy;
    // underside plane z = a x + b y + c: gradient (a,b) = steepest descent
    const double gx = det > 1e-9 ? (syy * sxz - sxy * syz) / det : 0;
    const double gy = det > 1e-9 ? (sxx * syz - sxy * sxz) / det : 0;
    const double slope = hypot2(gx, gy);

    // Run direction: the axis override, else down the slope, else the longer
    // UNSUPPORTED world axis.
    double ux, uy;
    if (axis) {
        ux = (*axis)[0]; uy = (*axis)[1];
    } else if (slope >= PROP.contourSlopeMin) {
        ux = gx; uy = gy;
    } else {
        double xLo = INF, xHi = -INF, yLo = INF, yHi = -INF;
        for (const Vec3d &p : pts) {
            if (p[0] < xLo) xLo = p[0]; if (p[0] > xHi) xHi = p[0];
            if (p[1] < yLo) yLo = p[1]; if (p[1] > yHi) yHi = p[1];
        }
        double spanX = xHi - xLo, spanY = yHi - yLo;

        // Size each axis by its longest CONTIGUOUS free run over a grid: a cell with
        // solid part directly beneath is supported.
        if (support && spanX > 0 && spanY > 0) {
            const double gstep = jsMax(1, step);
            const int nx = int(jsMin(80, std::ceil(spanX / gstep)));
            const int ny = int(jsMin(80, std::ceil(spanY / gstep)));
            std::vector<std::vector<uint8_t>> grid;   // grid[i][j] = free?
            grid.reserve(size_t(nx) + 1);
            for (int i = 0; i <= nx; i++) {
                const double x = xLo + (spanX * i) / nx;
                std::vector<uint8_t> col;
                col.reserve(size_t(ny) + 1);
                for (int j = 0; j <= ny; j++) {
                    const double y = yLo + (spanY * j) / ny;
                    const std::optional<double> z = surfaceZAt(patchTris, x, y);
                    col.push_back(z && !insidePart(*support->topo, support->rot, support->offset, x, y, *z - 0.1));
                }
                grid.push_back(std::move(col));
            }
            const double dx = spanX / nx, dy = spanY / ny;
            int runX = 0, runY = 0;
            for (int j = 0; j <= ny; j++) {                     // longest free run along X
                int run = 0;
                for (int i = 0; i <= nx; i++) {
                    run = grid[size_t(i)][size_t(j)] ? run + 1 : 0;
                    if (run > runX) runX = run;
                }
            }
            for (int i = 0; i <= nx; i++) {                     // longest free run along Y
                int run = 0;
                for (int j = 0; j <= ny; j++) {
                    run = grid[size_t(i)][size_t(j)] ? run + 1 : 0;
                    if (run > runY) runY = run;
                }
            }
            if (runX > 0 && runY > 0) { spanX = runX * dx; spanY = runY * dy; }
        }
        if (spanX >= spanY) { ux = 1; uy = 0; } else { ux = 0; uy = 1; }
    }
    const double un = hypot2(ux, uy); ux /= un; uy /= un;
    const double vx = -uy, vy = ux;

    double uLo = INF, uHi = -INF, vLo = INF, vHi = -INF;
    for (const Vec3d &p : pts) {
        const double u = p[0] * ux + p[1] * uy;
        const double t = p[0] * vx + p[1] * vy;
        if (u < uLo) uLo = u; if (u > uHi) uHi = u;
        if (t < vLo) vLo = t; if (t > vHi) vHi = t;
    }
    if (uHi - uLo < 1e-6) return Tracks{};
    const double nSt = jsMax(2, jsMin(400, std::ceil((uHi - uLo) / step)));

    // `span` is the requested row spacing from the coverage slider (coverRowSpan).
    const double rowSpan = jsMax(1, span);
    // (free-edge rows only under a near-flat underside -- see PROP.edgeFlatDeg)
    const bool flat = slope <= SF_TRACKS_TAN((PROP.edgeFlatDeg * M_PI) / 180);
    RowProbe probe;
    if (support && flat) {
        probe.support = support; probe.patchTris = &patchTris;
        probe.ux = ux; probe.uy = uy; probe.vx = vx; probe.vy = vy; probe.uLo = uLo; probe.uHi = uHi;
    }
    const RowOffsets rows = rowOffsets(vLo, vHi, rowSpan, support && flat ? &probe : nullptr);

    std::vector<ContactLine> tracks;
    for (double v0 : rows) {
        ContactLine cur;
        for (int k = 0; k <= nSt; k++) {
            const double u = uLo + ((uHi - uLo) * k) / nSt;
            const double x = ux * u + vx * v0, y = uy * u + vy * v0;
            const std::optional<double> z = surfaceZAt(patchTris, x, y);
            if (!z) {
                if (!cur.empty()) { tracks.push_back(std::move(cur)); cur.clear(); }
            } else {
                cur.push_back(Vec3d(x, y, *z));
            }
        }
        if (!cur.empty()) tracks.push_back(std::move(cur));
    }
    Tracks kept;
    for (auto &t : tracks)
        if (double(t.size()) >= PROP.minStations) kept.push_back(std::move(t));
    // The lateral gap between adjacent rows this face ended up with (sagRisk).
    kept.spacing = rows.spacing;
    return kept;
}

// ---------------------------------------------------------------------------------
RowOffsets rowOffsets(double vLo, double vHi, double rowSpan, const RowProbe *probe) {
    const double vExt = vHi - vLo, e = PROP.edgeInset;
    const int n = int(jsMax(1, js_round(vExt / rowSpan)));
    std::vector<double> rows;
    rows.reserve(size_t(n));
    for (int w = 0; w < n; w++) rows.push_back(vLo + (vExt * (w + 0.5)) / n);
    const double near = 2 * e;                 // an old row this close already sits at the end
    if (probe && vExt > 2 * near) {
        const bool lo = rows[0] - vLo > near && freeEdge(*probe, vLo, -1);
        const bool hi = vHi - rows[size_t(n) - 1] > near && freeEdge(*probe, vHi, +1);
        if (lo || hi) {
            const double a = lo ? vLo + e : rows[0], b = hi ? vHi - e : rows[size_t(n) - 1];
            // Spread wider than both the old pitch and 2 x maxUnsupportedSpan, and the
            // gap gets another row instead.
            const double cap = jsMax(vExt / n, 2 * PROP.maxUnsupportedSpan);
            const int m = int(jsMax(n, std::ceil((b - a) / cap - 1e-9) + 1));
            std::vector<double> next;
            if (n == 1) {
                if (lo) next.push_back(a);
                next.push_back(rows[0]);
                if (hi) next.push_back(b);
            } else {
                for (int w = 0; w < m; w++) next.push_back(a + ((b - a) * w) / (m - 1));
            }
            rows = std::move(next);
        }
    }
    if (probe) rows = lowEdgeRows(rows, lowEdges(*probe, vLo, vHi), near);
    double widest = 0;
    for (size_t i = 1; i < rows.size(); i++) widest = jsMax(widest, rows[i] - rows[i - 1]);
    RowOffsets out;
    static_cast<std::vector<double> &>(out) = std::move(rows);
    out.spacing = out.size() > 1 ? widest : 0;
    return out;
}

// ---------------------------------------------------------------------------------
bool LowEdges::low(double v) const {
    // JS cls[Math.max(0, Math.min(cls.length - 1, Math.round((v - vLo) / step)))] === 'low'
    const double i = jsMax(0, jsMin(double(cls.size()) - 1, js_round((v - vLo) / step)));
    if (std::isnan(i) || i >= double(cls.size())) return false;   // cls[NaN] / cls[0] of [] are undefined
    return cls[size_t(i)] == RowClass::Low;
}

LowEdges lowEdges(const RowProbe &probe, double vLo, double vHi) {
    const SurfTris &patchTris = *probe.patchTris;
    const double ux = probe.ux, uy = probe.uy, vx = probe.vx, vy = probe.vy, uLo = probe.uLo, uHi = probe.uHi;
    const double wallZ = PROP.minHeightSquat + PROP.gap + 0.02;   // clearance.js's squat floor
    const double brimZ = PROP.squatBrimH + PROP.gap + 0.02;
    const double step = 0.25;
    const int nU = 9;
    auto at = [&](double u, double v) { return surfaceZAt(patchTris, ux * u + vx * v, uy * u + vy * v); };
    LowEdges edges;
    edges.vLo = vLo;
    edges.step = step;
    std::vector<double> &vs = edges.vs;
    std::vector<RowClass> &cls = edges.cls;
    for (double v = vLo; v <= vHi + 1e-9; v += step) {
        int hit = 0, ok = 0;
        bool sunk = false;
        for (int i = 0; i < nU; i++) {
            const double u = uLo + ((uHi - uLo) * (i + 0.5)) / nU;
            const std::optional<double> z = at(u, v);
            if (!z) continue;
            hit++;
            if (*z < wallZ) { sunk = true; continue; }
            const std::optional<double> b1 = at(u, v - PROP.squatBrimW), b2 = at(u, v + PROP.squatBrimW);
            if ((!b1 || *b1 >= brimZ) && (!b2 || *b2 >= brimZ)) ok++;
        }
        vs.push_back(v);
        cls.push_back(hit == 0 ? RowClass::None : ok * 2 >= hit ? RowClass::Ok
                      : ok == 0 && sunk ? RowClass::Low : RowClass::Part);
    }
    const long nCls = long(cls.size());
    auto reach = [&](long k, int dir) -> long {   // first OK row from k on, over PART rows
        for (long j = k; j >= 0 && j < nCls; j += dir) {
            if (cls[size_t(j)] == RowClass::Ok) return j;
            if (cls[size_t(j)] != RowClass::Part) return -1;
        }
        return -1;
    };
    for (long k = 0; k < nCls; k++) {
        if (cls[size_t(k)] != RowClass::Low) continue;
        long j = k;
        while (j + 1 < nCls && cls[size_t(j) + 1] == RowClass::Low) j++;
        if ((j - k + 1) * step < PROP.lowBandMin) { k = j; continue; }   // a dip, not a band
        const long below = reach(k - 1, -1), above = reach(j + 1, +1);
        if (below >= 0) edges.push_back(LowEdge{vs[size_t(below)], -1});
        if (above >= 0) edges.push_back(LowEdge{vs[size_t(above)], +1});
        k = j;
    }
    return edges;
}

// ---------------------------------------------------------------------------------
std::vector<double> lowEdgeRows(const std::vector<double> &rows, const LowEdges &edges, double near) {
    std::vector<double> out = rows;
    const double brim = 2 * PROP.squatBrimW;
    for (const LowEdge &edge : edges) {
        const double v = edge.v;
        const int dir = edge.dir;
        bool blocked = false;
        for (double r : out) {
            const double d = (r - v) * dir;
            if (d >= 0 ? d < brim : -d < near || (-d < brim && !edges.low(r))) { blocked = true; break; }
        }
        if (!blocked) out.push_back(v);
    }
    if (out.size() == rows.size()) return rows;
    std::stable_sort(out.begin(), out.end(), [](double a, double b) { return a - b < 0; });
    return out;
}

// ---------------------------------------------------------------------------------
bool freeEdge(const RowProbe &probe, double vEdge, int dir) {
    const Support &support = *probe.support;
    const SurfTris &patchTris = *probe.patchTris;
    const double ux = probe.ux, uy = probe.uy, vx = probe.vx, vy = probe.vy, uLo = probe.uLo, uHi = probe.uHi;
    int air = 0, part = 0;
    for (double t : {0.1, 0.3, 0.5, 0.7, 0.9}) {
        const double u = uLo + (uHi - uLo) * t;
        auto atX = [&](double v) { return ux * u + vx * v; };
        auto atY = [&](double v) { return uy * u + vy * v; };
        const double vi = vEdge - dir * 0.3;
        const double ix = atX(vi), iy = atY(vi);
        const std::optional<double> zo = surfaceZAt(patchTris, ix, iy);
        if (!zo) continue;
        const double z = *zo;
        // an edge down at the bed rests on it
        if (z < PROP.minHeightSquat) { part++; continue; }
        const double vo = vEdge + dir * 0.8;
        const double ox = atX(vo), oy = atY(vo);
        bool solid = false;
        for (double dz : {-2.0, -1.0, -0.3, 0.3, 1.0, 2.0}) {
            if (insidePart(*support.topo, support.rot, support.offset, ox, oy, z + dz)) { solid = true; break; }
        }
        if (solid) { part++; continue; }
        // the flush row's whole column must be clear too (both faces, down to the bed)
        for (int side : {+1, -1}) {
            const double vc = vEdge - dir * PROP.edgeInset + side * dir * (PROP.th / 2 + PROP.sideClear);
            const double cx = atX(vc), cy = atY(vc);
            for (double zz = z - 0.5; zz > 0.3; zz -= 1) {
                if (insidePart(*support.topo, support.rot, support.offset, cx, cy, zz)) return false;
            }
        }
        air++;
    }
    return air >= 2 && air > part;
}

}} // namespace
