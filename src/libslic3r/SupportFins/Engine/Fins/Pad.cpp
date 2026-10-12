// Port of web/fins/pad.js -- PAD and the bed pad (conforming oval, or the brim-style one).
#include "Pad.hpp"
#include "../Ieee754.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>

namespace Slic3r { namespace SupportFins {

thread_local PadConfig PAD;

namespace {

constexpr double PI = 3.141592653589793;   // Math.PI
constexpr double NaN_ = std::numeric_limits<double>::quiet_NaN();

/// JS Math.min(a, b) / Math.max(a, b): NaN if either is NaN; -0 < +0.
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NaN_;
    if (a == b) return std::signbit(a) ? a : b;
    return a < b ? a : b;
}
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NaN_;
    if (a == b) return std::signbit(a) ? b : a;
    return a > b ? a : b;
}

/// JS ToInt32 (what `^` does to its operands): truncate, wrap mod 2^32.
inline int32_t toInt32(double v) {
    if (!std::isfinite(v)) return 0;
    double t = std::trunc(v);
    t = std::fmod(t, 4294967296.0);
    if (t < 0) t += 4294967296.0;
    return int32_t(uint32_t(t));
}

/// +x.toFixed(9). toFixed picks the n nearest x * 1e9 on the EXACT decimal value of x,
/// ties to the larger magnitude; glibc's %.9f breaks exact ties to even, so round from
/// the exact expansion (as Cutout.cpp's toFixed9). A negative input that rounds to zero
/// keeps its sign (+"-0.000000000" is -0); -0 itself prints "0.000000000" (it is not < 0).
double toFixed9(double x) {
    if (!std::isfinite(x) || std::fabs(x) >= 1e21) return x;   // toFixed falls back to ToString(x)
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
    const double r = std::strtod(keep.c_str(), nullptr);
    return x < 0 ? -r : r;
}

struct Outline {
    std::vector<double> segs;   // [x0, y0, x1, y1, ...]
    double length = 0;
};

/**
 * The part's first-layer outline: its section at the layer's mid-height (where a
 * slicer cuts), as a flat [x0, y0, x1, y1, ...] segment list, plus its length.
 */
Outline firstLayerOutline(const SurfTris &partTris, double zc) {
    Outline o;
    for (size_t i = 0; i < partTris.size(); i += 9) {
        double pts[3][2];
        int n = 0;
        for (int a = 0; a < 3; a++) {
            const size_t p = i + a * 3, q = i + ((a + 1) % 3) * 3;
            const double za = partTris[p + 2] - zc, zb = partTris[q + 2] - zc;
            if ((za < 0) == (zb < 0)) continue;
            const double t = za / (za - zb);
            pts[n][0] = partTris[p] + t * (partTris[q] - partTris[p]);
            pts[n][1] = partTris[p + 1] + t * (partTris[q + 1] - partTris[p + 1]);
            n++;
        }
        if (n != 2) continue;
        o.segs.push_back(pts[0][0]); o.segs.push_back(pts[0][1]);
        o.segs.push_back(pts[1][0]); o.segs.push_back(pts[1][1]);
        o.length += hypot2(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]);
    }
    return o;
}

struct Frame { double cx, cy, ax, ay, bx, by, r1, r2; };

/**
 * The BRIM-STYLE pad (PAD.style 'light', and 'custom'): the same oval, `H` tall (one
 * layer for 'light'), standing `g` off the part's first-layer outline. `grab` raises
 * (or, negative, lowers) its top against the part's underside; 'light' uses 0.
 *
 * The gap has to come through the SLICER, which merges any slice gap under ~0.098mm,
 * so it is held exactly on the first layer: the pad's top there ramps LINEARLY with
 * the true distance from the first-layer outline, crossing mid-height exactly at `g`.
 * It is also held below the lowest underside within `g` (plus `grab`).
 *
 * The mesh is columns across the oval's long axis, each running rim to rim with the
 * same number of rows, at `brimCell` spacing. The two tips close with a fan; the
 * bottom is one flat fan (the ellipse is convex).
 */
PadResult brimPad(const SurfTris &partTris, const std::vector<Vec2d> &contact, const Frame &e, double h, double g,
                  double grab, double layerH, const std::vector<double> &outline, Tris &out) {
    const double cx = e.cx, cy = e.cy, ax = e.ax, ay = e.ay, bx = e.bx, by = e.by, r1 = e.r1, r2 = e.r2;
    const double H = std::isfinite(h) && h > 0.05 ? h : FIN.tineH;
    const double cell = PAD.brimCell;
    const double zc = layerH / 2;                 // first layer's mid-height: the slicer's cut
    std::vector<std::array<double, 2>> ring;
    if (g > 0) for (int k = 0; k < 16; k++) ring.push_back({g * js_cos(k * PI / 8), g * js_sin(k * PI / 8)});

    // The part's first-layer outline (firstLayerOutline, cut at zc), bucketed.
    const double B = 0.5, reach = jsMax(g, 0) + 2 * cell;
    std::unordered_map<int32_t, std::vector<double>> bucket;
    // JS `i * 73856093 ^ j * 19349663`: both products are exact doubles, then ToInt32.
    auto key = [](double i, double j) { return toInt32(i * 73856093) ^ toInt32(j * 19349663); };
    for (size_t i = 0; i < outline.size(); i += 4) {
        const double x0 = outline[i], y0 = outline[i + 1], x1 = outline[i + 2], y1 = outline[i + 3];
        for (double bi = std::floor((jsMin(x0, x1) - reach) / B); bi <= std::floor((jsMax(x0, x1) + reach) / B); bi++) {
            for (double bj = std::floor((jsMin(y0, y1) - reach) / B); bj <= std::floor((jsMax(y0, y1) + reach) / B); bj++) {
                std::vector<double> &arr = bucket[key(bi, bj)];
                arr.push_back(x0); arr.push_back(y0); arr.push_back(x1); arr.push_back(y1);
            }
        }
    }
    // Signed distance to the outline, capped at `reach` (negative = inside it).
    auto dist = [&](double x, double y) {
        const auto it = bucket.find(key(std::floor(x / B), std::floor(y / B)));
        double d2 = reach * reach;
        if (it != bucket.end()) {
            const std::vector<double> &arr = it->second;
            for (size_t k = 0; k < arr.size(); k += 4) {
                const double x0 = arr[k], y0 = arr[k + 1], dx = arr[k + 2] - x0, dy = arr[k + 3] - y0;
                const double l2 = dx * dx + dy * dy;
                const double t = l2 > 0 ? jsMax(0, jsMin(1, ((x - x0) * dx + (y - y0) * dy) / l2)) : 0;
                const double ex = x0 + t * dx - x, ey = y0 + t * dy - y;
                d2 = jsMin(d2, ex * ex + ey * ey);
            }
        }
        const double d = std::sqrt(d2);
        const auto low = surfaceZAt(partTris, x, y);
        return low && *low < zc ? -d : d;
    };
    // Ramp slope: one cell below the crossing is still above the 0.05 floor, so the
    // two vertices around the crossing are never clamped and interpolate exactly.
    const double k = jsMax(0.01, zc - 0.05) / cell;
    auto top = [&](double x, double y) {
        double low = surfaceZAt(partTris, x, y).value_or(INF);
        for (const auto &d : ring) low = jsMin(low, surfaceZAt(partTris, x + d[0], y + d[1]).value_or(INF));
        // With no gap asked for (Custom gap 0) grip alone decides -- a bite is a bite.
        const double ramp = g > 0 ? zc + k * (dist(x, y) - g) : INF;
        return jsMax(0.05, jsMin(jsMin(H, low + grab), ramp));
    };
    auto at = [&](double s, double t) { return Vec2d(cx + ax * s + bx * t, cy + ay * s + by * t); };
    auto vert = [&](double s, double t) { const Vec2d p = at(s, t); return Vec3d(p[0], p[1], top(p[0], p[1])); };

    // Rows are FRACTIONS of each column's half-width, shared by every column so the
    // grid stays structured. They are fine (brimCell) only across the band where the
    // part comes within the pad's height -- found by a coarse scan -- and 1.2mm apart
    // elsewhere.
    const int nS = int(jsMax(3, std::ceil((2 * r1) / cell) + 1));
    auto half = [&](double s) { const double q = s / r1; return r2 * std::sqrt(jsMax(0, 1 - q * q)); };
    double band = 0;
    for (double s = -r1 + 0.25; s < r1; s += 0.5) {
        const double te = half(s);
        if (te < cell) continue;
        for (double t = 0; t <= te; t += cell) {
            for (const double sg : {1.0, -1.0}) {
                const Vec2d p = at(s, sg * t);
                if (top(p[0], p[1]) < H - 1e-9) band = jsMax(band, t / te);
            }
        }
    }
    const double fine = cell / r2, coarse = jsMax(fine, 1.2 / r2);
    const double edge = jsMin(1, band + 2 * fine);
    // new Set([-1, 1]) ... [...fr].sort((a, b) => a - b): a Set stores -0 as +0 and
    // dedupes by value, so the rows are the distinct values, ascending.
    std::vector<double> rows{-1, 1};
    auto addRow = [&](double v) { rows.push_back(v == 0 ? 0.0 : v); };
    for (double f = 0; f <= edge + 1e-12; f += fine) { addRow(toFixed9(jsMin(f, 1))); addRow(-jsMin(toFixed9(f), 1)); }
    for (double f = edge + coarse; f < 1; f += coarse) { addRow(toFixed9(f)); addRow(-(toFixed9(f))); }
    std::stable_sort(rows.begin(), rows.end(), [](double a, double b) { return a - b < 0; });
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    const int nT = int(rows.size());
    std::vector<std::vector<Vec3d>> cols;   // interior columns i = 1 .. nS-2
    for (int i = 1; i < nS - 1; i++) {
        const double s = -r1 + (2 * r1 * i) / (nS - 1);
        const double te = half(s);
        std::vector<Vec3d> col;
        col.reserve(rows.size());
        for (const double f : rows) col.push_back(vert(s, te * f));
        cols.push_back(std::move(col));
    }
    const Vec3d tipL = vert(-r1, 0), tipR = vert(r1, 0);
    Tris tris;
    auto tri = [&](const Vec3d &a, const Vec3d &b, const Vec3d &d) { tris.push_back(a); tris.push_back(b); tris.push_back(d); };
    for (int j = 0; j < nT - 1; j++) {
        tri(tipL, cols[0][j + 1], cols[0][j]);
        const std::vector<Vec3d> &L = cols[cols.size() - 1];
        tri(tipR, L[j], L[j + 1]);
    }
    for (size_t i = 0; i + 1 < cols.size(); i++) {
        const std::vector<Vec3d> &A = cols[i], &Bc = cols[i + 1];
        // wound like the tip fans, side wall and bottom
        for (int j = 0; j < nT - 1; j++) { tri(A[j], Bc[j + 1], Bc[j]); tri(A[j], A[j + 1], Bc[j + 1]); }
    }
    // boundary ring (tipL, the +t rim left to right, tipR, the -t rim right to left),
    // then its side wall and the flat bottom fan
    std::vector<Vec3d> rim;
    rim.push_back(tipL);
    for (const auto &col : cols) rim.push_back(col[nT - 1]);
    rim.push_back(tipR);
    for (auto it = cols.rbegin(); it != cols.rend(); ++it) rim.push_back((*it)[0]);
    std::vector<Vec3d> flat;
    for (const Vec3d &v : rim) flat.emplace_back(v[0], v[1], 0);
    const Vec3d b0(cx, cy, 0);
    for (size_t k2 = 0; k2 < rim.size(); k2++) {
        const size_t m = (k2 + 1) % rim.size();
        tri(rim[k2], flat[k2], flat[m]); tri(rim[k2], flat[m], rim[m]);
        tri(b0, flat[m], flat[k2]);
    }
    // One winding was picked above; flip the lot if it came out inside-out.
    double vol = 0;
    for (size_t i = 0; i < tris.size(); i += 3) {
        const Vec3d &p = tris[i], &q = tris[i + 1], &r = tris[i + 2];
        vol += p[0] * (q[1] * r[2] - q[2] * r[1]) - p[1] * (q[0] * r[2] - q[2] * r[0]) + p[2] * (q[0] * r[1] - q[1] * r[0]);
    }
    out.reserve(out.size() + tris.size());
    for (size_t i = 0; i < tris.size(); i += 3) {
        if (vol < 0) { out.push_back(tris[i]); out.push_back(tris[i + 2]); out.push_back(tris[i + 1]); }
        else { out.push_back(tris[i]); out.push_back(tris[i + 1]); out.push_back(tris[i + 2]); }
    }
    PadResult res;
    res.r1 = r1; res.r2 = r2;
    res.cells = int(cols.size()) * nT;
    res.height = H;
    res.points = int(contact.size());
    res.oval = true;
    res.style = PAD.style == "custom" ? "custom" : "light";
    return res;
}

} // namespace

/**
 * A breakaway pad under the part's bed contact.
 *
 * A part tilted into a strong orientation rests on an EDGE, so its bed contact is
 * near zero and it peels off before the fins have anything to hold. The pad is a
 * wide, thin footprint -- thin enough to cut off, wide enough to stick.
 *
 * It is fed the part's lowest VERTICES for its outline AND the seated part triangles,
 * so its TOP can conform to the part instead of welding to it: a radial mesh of the
 * contact ellipse, each vertex given its own top height. OUTBOARD (no part overhead)
 * rises to the full padH; under the part it caps at part_low + grab.
 */
std::optional<PadResult> buildPad(const std::vector<Vec2d> &contact, const SurfTris &partTris, Tris &out, double layerH) {
    if (contact.size() < 3) return std::nullopt;

    double cx = 0, cy = 0;
    for (const Vec2d &p : contact) { cx += p[0]; cy += p[1]; }
    cx /= double(contact.size()); cy /= double(contact.size());

    // principal axis of the contact, so a long thin edge gets a long thin pad
    // instead of a circle sized to its length
    double sxx = 0, sxy = 0, syy = 0;
    for (const Vec2d &p : contact) {
        const double dx = p[0] - cx, dy = p[1] - cy;
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    const double tr = sxx + syy, det = sxx * syy - sxy * sxy;
    const double lam = tr / 2 + std::sqrt(jsMax(0, (tr * tr) / 4 - det));
    double ax = sxy, ay = lam - sxx;
    if (hypot2(ax, ay) < 1e-9) { ax = 1; ay = 0; }
    const double an = hypot2(ax, ay); ax /= an; ay /= an;
    const double bx = -ay, by = ax;

    double e1 = 0, e2 = 0;
    for (const Vec2d &p : contact) {
        const double dx = p[0] - cx, dy = p[1] - cy;
        e1 = jsMax(e1, std::abs(dx * ax + dy * ay));
        e2 = jsMax(e2, std::abs(dx * bx + dy * by));
    }
    const double margin = PAD.style == "custom" ? PAD.custom.margin : FIN.padMargin;
    const double r1 = e1 + margin, r2 = e2 + margin;

    // Height rule per vertex: OUTBOARD (no part overhead) rises to the full padH for
    // bed grip; UNDER the part it caps at part_low + grab.
    double sureGrab = PAD.grab;   // raised to >= 0 below when Light swaps to Sure hold
    auto conform = [&](double x, double y) {
        const auto low = surfaceZAt(partTris, x, y);
        if (!low) return FIN.padH;
        // grab may be negative (a PETG gap). Floor at 0.05 so every column stays
        // positive -- the disc watertight -- and a gap pad still kisses the resting edge.
        return jsMax(0.05, jsMin(FIN.padH, *low + sureGrab));
    };
    const Frame frame{cx, cy, ax, ay, bx, by, r1, r2};
    const double L1 = std::isfinite(layerH) && layerH > 0.05 ? layerH : FIN.tineH;
    // Light holds by first-layer squish along the part's first-layer OUTLINE. Below
    // PAD.minGripOutline of outline (a point, a cone tip, a small nub), Auto builds
    // Sure hold and says so (autoSure); otherwise Light. An explicit Light or Custom
    // is the user's call; `smallFoot` rides along so the UI can warn.
    const Outline outline = firstLayerOutline(partTris, L1 / 2);
    const bool smallFoot = outline.length < PAD.minGripOutline;
    auto tag = [&](PadResult pad) { pad.smallFoot = smallFoot; pad.outline = outline.length; return pad; };
    const bool light = PAD.style == "light" || (PAD.style == "auto" && !smallFoot);
    if (light) return tag(brimPad(partTris, contact, frame, L1, PAD.brimGap, 0, L1, outline.segs, out));
    if (PAD.style == "custom") {
        const PadConfig::Custom &c = PAD.custom;
        return tag(brimPad(partTris, contact, frame, c.h, c.gap, c.grip, L1, outline.segs, out));
    }
    const bool autoSure = PAD.style == "auto";
    // The swap exists to HOLD a part that barely touches the plate, so the pad must
    // meet it: flush or the material's tack, never a gap.
    if (autoSure) sureGrab = jsMax(PAD.grab, 0);
    const int nTheta = FIN.padSegs;
    const int nRing = int(jsMax(2, std::ceil(jsMax(r1, r2) / PAD.cell)));

    // Ring/segment vertex in world XY, at radial fraction `fr` and angle index `j`.
    auto vAt = [&](double fr, int j) {
        const double a = (2 * PI * j) / nTheta;
        const double s = r1 * fr * js_cos(a), t = r2 * fr * js_sin(a);
        return Vec2d(cx + ax * s + bx * t, cy + ay * s + by * t);
    };
    // Precompute the vertex ring positions + their conformed tops once (reused by the
    // top cap, the side wall, and the flat bottom), so every shared edge is keyed from
    // bit-identical coordinates and the soup stays edge-manifold.
    std::vector<std::vector<Vec3d>> V;   // V[i][j] = [x, y, topZ], i in 0..nRing, j in 0..nTheta-1
    for (int i = 0; i <= nRing; i++) {
        const double fr = double(i) / nRing;
        std::vector<Vec3d> row;
        row.reserve(size_t(nTheta));
        for (int j = 0; j < nTheta; j++) {
            const Vec2d p = vAt(fr, j);
            row.emplace_back(p[0], p[1], conform(p[0], p[1]));
        }
        V.push_back(std::move(row));
    }
    const Vec3d centreTop = V[0][0];   // ring 0 collapses to the centre (fr = 0)
    auto tri = [&](const Vec3d &a, const Vec3d &b, const Vec3d &c) { out.push_back(a); out.push_back(b); out.push_back(c); };
    double maxTop = 0;
    for (const auto &row : V) for (const Vec3d &v : row) if (v[2] > maxTop) maxTop = v[2];

    for (int j = 0; j < nTheta; j++) {
        const int jn = (j + 1) % nTheta;
        // TOP surface (normal up). Inner fan from the centre, then quad strips outward.
        tri(centreTop, V[1][j], V[1][jn]);
        for (int i = 1; i < nRing; i++) {
            tri(V[i][j], V[i + 1][j], V[i + 1][jn]);
            tri(V[i][j], V[i + 1][jn], V[i][jn]);
        }
        // BOTTOM surface at z=0 (normal down: reverse the top winding).
        const Vec3d b0(cx, cy, 0);
        auto bi = [&](int i, int k) { return Vec3d(V[i][k][0], V[i][k][1], 0); };
        tri(b0, bi(1, jn), bi(1, j));
        for (int i = 1; i < nRing; i++) {
            tri(bi(i, j), bi(i + 1, jn), bi(i + 1, j));
            tri(bi(i, j), bi(i, jn), bi(i + 1, jn));
        }
        // SIDE wall around the outer ring, top down to the plate (normal outward).
        const int oR = nRing;
        const Vec3d &tj = V[oR][j], &tjn = V[oR][jn];
        const Vec3d bj(tj[0], tj[1], 0), bjn(tjn[0], tjn[1], 0);
        tri(tj, bj, bjn);
        tri(tj, bjn, tjn);
    }

    PadResult res;
    res.r1 = r1; res.r2 = r2;
    res.cells = nTheta * nRing;
    res.height = maxTop;
    res.points = int(contact.size());
    res.oval = true;
    res.style = "sure";
    res.autoSure = autoSure;
    res.smallFoot = smallFoot;
    res.outline = outline.length;
    return res;
}

}} // namespace
