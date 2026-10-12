// Port of web/inside.js -- is a point inside the part? Ray-parity, with a grid so it
// is cheap enough to call while dragging.
//
// WHY THIS EXISTS. Fin placement searches with a proximity test (prop/clearance.js):
// does any part surface cross the slab the fin sweeps. That is cheap and lets the
// search consider thousands of windows, but it is not quite "is this fin inside the
// part" -- the gap showed up as a wall buried 2.6mm into a Voron housing that every
// cheaper check called clear. So the search stays cheap and the ANSWER gets verified:
// broad search, exact confirmation.
//
// THE GRID IS BUILT IN THE MESH'S ORIGINAL FRAME, never the rotated one: rotation
// cannot change which triangles exist, so a structure built once per file stays valid
// for every orientation. Query points are rotated back into that frame instead.
#include "Inside.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace Slic3r { namespace SupportFins {

namespace {

constexpr int    GRID = INSIDE_GRID;
constexpr double EPS = INSIDE_EPS;
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
inline double jsMin3(double a, double b, double c) { return jsMin(jsMin(a, b), c); }
inline double jsMax3(double a, double b, double c) { return jsMax(jsMax(a, b), c); }
inline double clampCell(double v) { return jsMin(GRID - 1, jsMax(0, v)); }

// V8's Math.hypot (builtins/math.tq): scale by the largest magnitude, Kahan-sum the
// squares, sqrt(sum) * max. std::hypot (Math.hpp hypot3) differs from it in the last
// bit on ~5% of inputs, so segTriHit's parallel test uses this one.
inline double v8Hypot3(double x, double y, double z) {
    const double a[3] = {std::fabs(x), std::fabs(y), std::fabs(z)};
    double max = 0;
    bool nan = false;
    for (double v : a) { if (std::isnan(v)) nan = true; else if (v > max) max = v; }
    if (max == INF) return INF;
    if (nan) return NaN;
    if (max == 0) return 0;
    double sum = 0, compensation = 0;
    for (double v : a) {
        const double n = v / max;
        const double summand = n * n - compensation;
        const double preliminary = sum + summand;
        compensation = (preliminary - sum) - summand;
        sum = preliminary;
    }
    return std::sqrt(sum) * max;
}

/// Closest point on triangle `f` to (px, py, pz), all in the mesh's own frame.
Vec3d triClosest(const Soup &pos, int f, double px, double py, double pz) {
    const size_t o = size_t(f) * 9;
    const double ax = pos[o], ay = pos[o + 1], az = pos[o + 2];
    const double abx = pos[o + 3] - ax, aby = pos[o + 4] - ay, abz = pos[o + 5] - az;
    const double acx = pos[o + 6] - ax, acy = pos[o + 7] - ay, acz = pos[o + 8] - az;
    const double apx = px - ax, apy = py - ay, apz = pz - az;

    const double d1 = abx * apx + aby * apy + abz * apz;
    const double d2 = acx * apx + acy * apy + acz * apz;
    if (d1 <= 0 && d2 <= 0) return Vec3d(ax, ay, az);

    const double bpx = px - pos[o + 3], bpy = py - pos[o + 4], bpz = pz - pos[o + 5];
    const double d3 = abx * bpx + aby * bpy + abz * bpz;
    const double d4 = acx * bpx + acy * bpy + acz * bpz;
    if (d3 >= 0 && d4 <= d3) return Vec3d(pos[o + 3], pos[o + 4], pos[o + 5]);

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const double t = d1 / (d1 - d3);
        return Vec3d(ax + abx * t, ay + aby * t, az + abz * t);
    }

    const double cpx = px - pos[o + 6], cpy = py - pos[o + 7], cpz = pz - pos[o + 8];
    const double d5 = abx * cpx + aby * cpy + abz * cpz;
    const double d6 = acx * cpx + acy * cpy + acz * cpz;
    if (d6 >= 0 && d5 <= d6) return Vec3d(pos[o + 6], pos[o + 7], pos[o + 8]);

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const double t = d2 / (d2 - d6);
        return Vec3d(ax + acx * t, ay + acy * t, az + acz * t);
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
        const double t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return Vec3d(pos[o + 3] + (pos[o + 6] - pos[o + 3]) * t,
                     pos[o + 4] + (pos[o + 7] - pos[o + 4]) * t,
                     pos[o + 5] + (pos[o + 8] - pos[o + 5]) * t);
    }

    const double denom = 1 / (va + vb + vc);
    const double v = vb * denom, w = vc * denom;
    return Vec3d(ax + abx * v + acx * w, ay + aby * v + acy * w, az + abz * v + acz * w);
}

/// Closest points between segments p0-p1 and q0-q1; returns [pt on P, pt on Q].
void segSegClosest(const Vec3d &p0, const Vec3d &p1, const Vec3d &q0, const Vec3d &q1, Vec3d &outP, Vec3d &outQ) {
    const double dx1 = p1[0] - p0[0], dy1 = p1[1] - p0[1], dz1 = p1[2] - p0[2];
    const double dx2 = q1[0] - q0[0], dy2 = q1[1] - q0[1], dz2 = q1[2] - q0[2];
    const double rx = p0[0] - q0[0], ry = p0[1] - q0[1], rz = p0[2] - q0[2];
    const double a = dx1 * dx1 + dy1 * dy1 + dz1 * dz1;
    const double e = dx2 * dx2 + dy2 * dy2 + dz2 * dz2;
    const double f = dx2 * rx + dy2 * ry + dz2 * rz;
    double s, t;
    if (a <= EPS && e <= EPS) { s = 0; t = 0; }
    else if (a <= EPS) { s = 0; t = jsMin(1, jsMax(0, f / e)); }
    else {
        const double c = dx1 * rx + dy1 * ry + dz1 * rz;
        if (e <= EPS) { t = 0; s = jsMin(1, jsMax(0, -c / a)); }
        else {
            const double b = dx1 * dx2 + dy1 * dy2 + dz1 * dz2;
            const double den = a * e - b * b;
            s = den > EPS ? jsMin(1, jsMax(0, (b * f - c * e) / den)) : 0;
            t = (b * s + f) / e;
            if (t < 0) { t = 0; s = jsMin(1, jsMax(0, -c / a)); }
            else if (t > 1) { t = 1; s = jsMin(1, jsMax(0, (b - c) / a)); }
        }
    }
    outP = Vec3d(p0[0] + dx1 * s, p0[1] + dy1 * s, p0[2] + dz1 * s);
    outQ = Vec3d(q0[0] + dx2 * t, q0[1] + dy2 * t, q0[2] + dz2 * t);
}

using Tri = std::array<Vec3d, 3>;

/// The three vertices of triangle `f`.
Tri triVerts(const Soup &pos, int f) {
    const size_t o = size_t(f) * 9;
    return {Vec3d(pos[o], pos[o + 1], pos[o + 2]),
            Vec3d(pos[o + 3], pos[o + 4], pos[o + 5]),
            Vec3d(pos[o + 6], pos[o + 7], pos[o + 8])};
}

inline double dist2(const Vec3d &p, const Vec3d &q) {
    const double dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2];
    return dx * dx + dy * dy + dz * dz;
}

/// Segment p-q against triangle A-B-C (Moller-Trumbore); the hit point or nullopt.
std::optional<Vec3d> segTriHit(const Vec3d &p, const Vec3d &q, const Vec3d &A, const Vec3d &B, const Vec3d &C) {
    const double dx = q[0] - p[0], dy = q[1] - p[1], dz = q[2] - p[2];
    const double e1x = B[0] - A[0], e1y = B[1] - A[1], e1z = B[2] - A[2];
    const double e2x = C[0] - A[0], e2y = C[1] - A[1], e2z = C[2] - A[2];
    const double hx = dy * e2z - dz * e2y;
    const double hy = dz * e2x - dx * e2z;
    const double hz = dx * e2y - dy * e2x;
    const double a = e1x * hx + e1y * hy + e1z * hz;
    // parallel; grazing is distance work. Relative to the lengths: `a` is a triple
    // product (mm^3), and an edge lying IN the triangle's plane measures ~1e-12 of
    // rounding on mm-sized parts -- past an absolute 1e-12 it read as a pierce at a
    // garbage point (cube X40: a wall's end cap in the plane of the part's top edge,
    // 0.2 mm clear, trimmed as a weld under a 1e-9 rad nudge; local issue 023).
    const double scale = v8Hypot3(dx, dy, dz) * v8Hypot3(e1x, e1y, e1z) * v8Hypot3(e2x, e2y, e2z);
    if (std::abs(a) <= 1e-9 * scale) return std::nullopt;
    const double f = 1 / a;
    const double sx = p[0] - A[0], sy = p[1] - A[1], sz = p[2] - A[2];
    const double u = f * (sx * hx + sy * hy + sz * hz);
    if (u < 0 || u > 1) return std::nullopt;
    const double qx = sy * e1z - sz * e1y;
    const double qy = sz * e1x - sx * e1z;
    const double qz = sx * e1y - sy * e1x;
    const double v = f * (dx * qx + dy * qy + dz * qz);
    if (v < 0 || u + v > 1) return std::nullopt;
    const double t = f * (e2x * qx + e2y * qy + e2z * qz);
    if (t < 0 || t > 1) return std::nullopt;
    return Vec3d(p[0] + dx * t, p[1] + dy * t, p[2] + dz * t);
}

/// Do triangles A and B cross? The piercing point if so, else nullopt.
std::optional<Vec3d> triTriPierce(const Tri &A, const Tri &B) {
    for (int e = 0; e < 3; e++) {
        std::optional<Vec3d> hit = segTriHit(A[e], A[(e + 1) % 3], B[0], B[1], B[2]);
        if (!hit) hit = segTriHit(B[e], B[(e + 1) % 3], A[0], A[1], A[2]);
        if (hit) return hit;
    }
    return std::nullopt;
}

/// triClosest, but for a triangle given as three vertex arrays.
Vec3d triClosestP(const Tri &T, const Vec3d &p) {
    const Vec3d &A = T[0], &B = T[1], &C = T[2];
    const double abx = B[0] - A[0], aby = B[1] - A[1], abz = B[2] - A[2];
    const double acx = C[0] - A[0], acy = C[1] - A[1], acz = C[2] - A[2];
    const double apx = p[0] - A[0], apy = p[1] - A[1], apz = p[2] - A[2];

    const double d1 = abx * apx + aby * apy + abz * apz;
    const double d2 = acx * apx + acy * apy + acz * apz;
    if (d1 <= 0 && d2 <= 0) return A;

    const double bpx = p[0] - B[0], bpy = p[1] - B[1], bpz = p[2] - B[2];
    const double d3 = abx * bpx + aby * bpy + abz * bpz;
    const double d4 = acx * bpx + acy * bpy + acz * bpz;
    if (d3 >= 0 && d4 <= d3) return B;

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const double t = d1 / (d1 - d3);
        return Vec3d(A[0] + abx * t, A[1] + aby * t, A[2] + abz * t);
    }

    const double cpx = p[0] - C[0], cpy = p[1] - C[1], cpz = p[2] - C[2];
    const double d5 = abx * cpx + aby * cpy + abz * cpz;
    const double d6 = acx * cpx + acy * cpy + acz * cpz;
    if (d6 >= 0 && d5 <= d6) return C;

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const double t = d2 / (d2 - d6);
        return Vec3d(A[0] + acx * t, A[1] + acy * t, A[2] + acz * t);
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
        const double t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return Vec3d(B[0] + (C[0] - B[0]) * t, B[1] + (C[1] - B[1]) * t,
                     B[2] + (C[2] - B[2]) * t);
    }

    const double denom = 1 / (va + vb + vc);
    const double v = vb * denom, w = vc * denom;
    return Vec3d(A[0] + abx * v + acx * w, A[1] + aby * v + acy * w,
                 A[2] + abz * v + acz * w);
}

// JS ToInt32 (what `x | 0` and an Int32Array store do to a number).
inline int32_t toInt32(int64_t v) { return int32_t(uint32_t(uint64_t(v))); }

} // namespace

double InsideGrid::cell(double y, double z) const {
    const double a = jsMin(GRID - 1, jsMax(0, std::floor((y - minY) * sy)));
    const double b = jsMin(GRID - 1, jsMax(0, std::floor((z - minZ) * sz)));
    return a * GRID + b;
}

/**
 * Bucket every triangle by its (y, z) extent in the mesh's own frame, so a ray
 * cast along +x only has to consider the triangles in one column.
 */
const InsideGrid &buildGrid(const Topology &topo) {
    if (topo._insideGrid) return *topo._insideGrid;
    const Soup &pos = topo.pos;
    const int nFaces = topo.nFaces;

    double minY = INF, maxY = -INF, minZ = INF, maxZ = -INF;
    for (int v = 0; v < nFaces * 3; v++) {
        const double y = pos[size_t(v) * 3 + 1], z = pos[size_t(v) * 3 + 2];
        if (y < minY) minY = y; if (y > maxY) maxY = y;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }
    const double sy = GRID / jsMax(1e-6, maxY - minY);
    const double sz = GRID / jsMax(1e-6, maxZ - minZ);

    // counting sort into CSR, so there is one flat array rather than 4096 little ones
    auto grid = std::make_shared<InsideGrid>();
    std::vector<int32_t> &counts = grid->start;
    counts.assign(GRID * GRID + 1, 0);
    auto span = [&](int f, int &a0, int &a1, int &b0, int &b1) {
        const size_t o = size_t(f) * 9;
        const double y0 = jsMin3(pos[o + 1], pos[o + 4], pos[o + 7]);
        const double y1 = jsMax3(pos[o + 1], pos[o + 4], pos[o + 7]);
        const double z0 = jsMin3(pos[o + 2], pos[o + 5], pos[o + 8]);
        const double z1 = jsMax3(pos[o + 2], pos[o + 5], pos[o + 8]);
        a0 = int(clampCell(std::floor((y0 - minY) * sy)));
        a1 = int(clampCell(std::floor((y1 - minY) * sy)));
        b0 = int(clampCell(std::floor((z0 - minZ) * sz)));
        b1 = int(clampCell(std::floor((z1 - minZ) * sz)));
    };

    for (int f = 0; f < nFaces; f++) {
        int a0, a1, b0, b1;
        span(f, a0, a1, b0, b1);
        for (int a = a0; a <= a1; a++)
            for (int b = b0; b <= b1; b++) counts[a * GRID + b + 1]++;
    }
    for (int i = 0; i < GRID * GRID; i++) counts[i + 1] += counts[i];
    grid->items.assign(size_t(counts[GRID * GRID]), 0);
    std::vector<int32_t> cur(counts.begin(), counts.begin() + GRID * GRID);
    for (int f = 0; f < nFaces; f++) {
        int a0, a1, b0, b1;
        span(f, a0, a1, b0, b1);
        for (int a = a0; a <= a1; a++)
            for (int b = b0; b <= b1; b++) grid->items[size_t(cur[a * GRID + b]++)] = f;
    }
    grid->minY = minY; grid->minZ = minZ; grid->sy = sy; grid->sz = sz;
    topo._insideGrid = grid;
    return *grid;
}

/**
 * The part's closest surface point within `rMax` of a PRINT-space query point, or
 * null if nothing is that close.
 *
 * This is the measurement `insidePart` cannot make: parity says in-or-out, and a
 * surface arriving TANGENT to a wall's face -- or a rib thinner than the gap between
 * two probes -- reads "out" at every probe while sitting 0.005mm away. You cannot
 * bracket a distance with containment tests; you have to measure it.
 *
 * Returns { d, cosUp }: the distance, and the PRINT-space vertical share of the
 * direction toward the surface, so the caller can tell a breakaway interface (above)
 * from a flank approach (sideways). Bounded by `rMax` so the grid's cell neighbourhood
 * suffices. Triangles spanning several cells are tested more than once rather than
 * deduplicated -- a repeat distance test is cheaper than the bookkeeping.
 */
std::optional<NearestHit> nearestPart(const Topology &topo, const Rot &rot, const Offset &offset,
                                      double qx, double qy, double qz, double rMax) {
    const Soup &pos = topo.pos;
    const InsideGrid &grid = buildGrid(topo);

    const double x = qx - offset.x, y = qy - offset.y, z = qz - offset.z;
    const double ox = rot[0] * x + rot[1] * y + rot[2] * z;
    const double oy = rot[3] * x + rot[4] * y + rot[5] * z;
    const double oz = rot[6] * x + rot[7] * y + rot[8] * z;

    // doubles, as in the JS: a NaN bound makes the loop run zero times
    const double a0 = clampCell(std::floor((oy - rMax - grid.minY) * grid.sy));
    const double a1 = clampCell(std::floor((oy + rMax - grid.minY) * grid.sy));
    const double b0 = clampCell(std::floor((oz - rMax - grid.minZ) * grid.sz));
    const double b1 = clampCell(std::floor((oz + rMax - grid.minZ) * grid.sz));

    double best = rMax * rMax, bx = 0, by = 0, bz = 0;
    bool found = false;
    for (double a = a0; a <= a1; a++) {
        for (double b = b0; b <= b1; b++) {
            const int c = int(a) * GRID + int(b);
            for (int32_t i = grid.start[c]; i < grid.start[c + 1]; i++) {
                const Vec3d q = triClosest(pos, grid.items[i], ox, oy, oz);
                const double dx = q[0] - ox, dy = q[1] - oy, dz = q[2] - oz;
                const double dd = dx * dx + dy * dy + dz * dz;
                if (dd < best) { best = dd; bx = dx; by = dy; bz = dz; found = true; }
            }
        }
    }
    if (!found) return std::nullopt;
    const double d = std::sqrt(best);
    // PRINT-space z of a mesh-frame direction: how far "up" the approach points
    const double upZ = rot[2] * bx + rot[5] * by + rot[8] * bz;
    return NearestHit{d, d > EPS ? upZ / d : 0};
}

/**
 * The exact minimum clearance between an added solid and the part, or null if it is
 * at least `rMax` everywhere.
 *
 * Probing the wall's surface -- with parity OR with distances -- can only promise
 * detection where a probe lands. So flip the measurement: the wall's triangles exist,
 * the part's triangles have a grid over them, so compute real triangle-to-triangle
 * distance -- vertex-vs-face both ways plus all nine edge pairs -- over the grid's
 * broad phase. This is the same measurement check_stl.py makes.
 *
 * `verts` is the solid's triangle soup in PRINT space, three vertices per triangle.
 * Returns { d, cosUp, x, y, z }: the distance, the print-space vertical share of the
 * approach, and the closest point on the SOLID (print space).
 */
std::optional<SolidHit> solidClearance(const Topology &topo, const Rot &rot, const Offset &offset,
                                       const Tris &verts, double rMax) {
    const Soup &pos = topo.pos;
    const InsideGrid &grid = buildGrid(topo);

    // JS reads mv[t + 2][0] of undefined and throws on a ragged soup
    if (verts.size() % 3 != 0) throw std::invalid_argument("solidClearance: verts must be whole triangles");

    // un-rotate the solid's vertices into the mesh frame once
    std::vector<Vec3d> mv(verts.size());
    for (size_t i = 0; i < verts.size(); i++) {
        const double x = verts[i][0] - offset.x, y = verts[i][1] - offset.y,
                     z = verts[i][2] - offset.z;
        mv[i] = Vec3d(rot[0] * x + rot[1] * y + rot[2] * z,
                      rot[3] * x + rot[4] * y + rot[5] * z,
                      rot[6] * x + rot[7] * y + rot[8] * z);
    }

    // A part triangle is registered in every YZ cell its extent touches, so the same
    // (wall triangle, part triangle) pair is otherwise measured once per shared cell.
    // A per-wall-triangle generation stamp collapses each pair to one test; skipping a
    // repeat of an identical pair cannot change the running minimum, so the output is
    // bit-identical. Stamp is cached on `topo` (like the grid) and the generation is
    // monotonic across calls.
    std::vector<int32_t> &seen = topo._scSeen;
    if (seen.size() != size_t(topo.nFaces)) seen.assign(size_t(topo.nFaces), 0);
    int64_t gen = toInt32(topo._scGen);
    if (gen > 2e9) { std::fill(seen.begin(), seen.end(), 0); gen = 0; }   // guard Int32 overflow

    double best = rMax * rMax;
    bool found = false;   // JS: bp !== null
    Vec3d bp = Vec3d::Zero(), bq = Vec3d::Zero();
    for (size_t t = 0; t < mv.size(); t += 3) {
        gen++;
        const Tri A = {mv[t], mv[t + 1], mv[t + 2]};
        double xLo = INF, xHi = -INF;
        double yLo = INF, yHi = -INF, zLo = INF, zHi = -INF;
        for (const Vec3d &q : A) {
            if (q[0] < xLo) xLo = q[0]; if (q[0] > xHi) xHi = q[0];
            if (q[1] < yLo) yLo = q[1]; if (q[1] > yHi) yHi = q[1];
            if (q[2] < zLo) zLo = q[2]; if (q[2] > zHi) zHi = q[2];
        }
        const double a0 = clampCell(std::floor((yLo - rMax - grid.minY) * grid.sy));
        const double a1 = clampCell(std::floor((yHi + rMax - grid.minY) * grid.sy));
        const double b0 = clampCell(std::floor((zLo - rMax - grid.minZ) * grid.sz));
        const double b1 = clampCell(std::floor((zHi + rMax - grid.minZ) * grid.sz));

        for (double a = a0; a <= a1; a++) {
            for (double b = b0; b <= b1; b++) {
                const int c = int(a) * GRID + int(b);
                for (int32_t i = grid.start[c]; i < grid.start[c + 1]; i++) {
                    const int f = grid.items[i];
                    if (int64_t(seen[f]) == gen) continue;   // already measured vs this wall tri
                    seen[f] = toInt32(gen);
                    const size_t o = size_t(f) * 9;
                    // reject on the axis-aligned boxes before any real geometry
                    const double bx0 = jsMin3(pos[o], pos[o + 3], pos[o + 6]);
                    if (bx0 > xHi + rMax) continue;
                    const double bx1 = jsMax3(pos[o], pos[o + 3], pos[o + 6]);
                    if (bx1 < xLo - rMax) continue;
                    const double by0 = jsMin3(pos[o + 1], pos[o + 4], pos[o + 7]);
                    if (by0 > yHi + rMax) continue;
                    const double by1 = jsMax3(pos[o + 1], pos[o + 4], pos[o + 7]);
                    if (by1 < yLo - rMax) continue;
                    const double bz0 = jsMin3(pos[o + 2], pos[o + 5], pos[o + 8]);
                    if (bz0 > zHi + rMax) continue;
                    const double bz1 = jsMax3(pos[o + 2], pos[o + 5], pos[o + 8]);
                    if (bz1 < zLo - rMax) continue;

                    const Tri B = triVerts(pos, f);

                    // INTERSECTION FIRST. Every distance primitive below is only valid
                    // for triangles that do not cross: an edge piercing the other
                    // triangle's interior is at distance zero while every vertex-face
                    // and edge-edge pair measures finite.
                    if (std::optional<Vec3d> pierce = triTriPierce(A, B)) {
                        best = 0; bp = *pierce; bq = *pierce; found = true;
                        goto done;   // nothing beats zero (JS: a = a1+1; b = b1+1; t = mv.length; break)
                    }

                    // verts of the part tri against the solid tri and vice versa
                    for (const Vec3d &q : B) {
                        const Vec3d cp = triClosestP(A, q);
                        const double dd = dist2(cp, q);
                        if (dd < best) { best = dd; bp = cp; bq = q; found = true; }
                    }
                    for (const Vec3d &q : A) {
                        const Vec3d cp = triClosestP(B, q);
                        const double dd = dist2(q, cp);
                        if (dd < best) { best = dd; bp = q; bq = cp; found = true; }
                    }
                    // all nine edge pairs
                    for (int ea = 0; ea < 3; ea++) {
                        for (int eb = 0; eb < 3; eb++) {
                            Vec3d pA, pB;
                            segSegClosest(A[ea], A[(ea + 1) % 3], B[eb], B[(eb + 1) % 3], pA, pB);
                            const double dd = dist2(pA, pB);
                            if (dd < best) { best = dd; bp = pA; bq = pB; found = true; }
                        }
                    }
                }
            }
        }
    }
done:
    topo._scGen = gen;   // keep the stamp generation monotonic across calls
    if (!found) return std::nullopt;
    const double d = std::sqrt(best);
    const double dx = bq[0] - bp[0], dy = bq[1] - bp[1], dz = bq[2] - bp[2];
    const double upZ = rot[2] * dx + rot[5] * dy + rot[8] * dz;
    // the solid-side point, back in print space
    const double px = rot[0] * bp[0] + rot[3] * bp[1] + rot[6] * bp[2] + offset.x;
    const double py = rot[1] * bp[0] + rot[4] * bp[1] + rot[7] * bp[2] + offset.y;
    const double pz = rot[2] * bp[0] + rot[5] * bp[1] + rot[8] * bp[2] + offset.z;
    return SolidHit{d, d > EPS ? upZ / d : 0, px, py, pz};
}

/**
 * Containment by ray parity along +x, for a query point given in PRINT space.
 *
 * `rot` is the column-major orientation and `offset` the seating translation, so the
 * point is un-seated and un-rotated back into the mesh's own frame before the test --
 * which is what lets the grid be orientation-independent.
 */
bool insidePart(const Topology &topo, const Rot &rot, const Offset &offset,
                double qx, double qy, double qz) {
    const Soup &pos = topo.pos;
    const InsideGrid &grid = buildGrid(topo);

    // inverse of an orthonormal rotation is its transpose
    const double x = qx - offset.x, y = qy - offset.y, z = qz - offset.z;
    const double ox = rot[0] * x + rot[1] * y + rot[2] * z;
    const double oy = rot[3] * x + rot[4] * y + rot[5] * z;
    const double oz = rot[6] * x + rot[7] * y + rot[8] * z;

    const double cd = grid.cell(oy, oz);
    if (std::isnan(cd)) return false;   // JS: start[NaN] is undefined, the loop never runs
    const int c = int(cd);
    int crossings = 0;
    for (int32_t i = grid.start[c]; i < grid.start[c + 1]; i++) {
        const size_t o = size_t(grid.items[i]) * 9;
        const double ay = pos[o + 1], az = pos[o + 2];
        const double by = pos[o + 4], bz = pos[o + 5];
        const double cy = pos[o + 7], cz = pos[o + 8];

        // barycentric of (oy, oz) within the triangle projected onto the y-z plane
        const double d = (bz - cz) * (ay - cy) + (cy - by) * (az - cz);
        if (std::abs(d) < EPS) continue;
        const double l1 = ((bz - cz) * (oy - cy) + (cy - by) * (oz - cz)) / d;
        const double l2 = ((cz - az) * (oy - cy) + (ay - cy) * (oz - cz)) / d;
        const double l3 = 1 - l1 - l2;
        if (l1 < 0 || l2 < 0 || l3 < 0) continue;

        // x where the ray meets the triangle's plane; only forward hits count
        const double hx = l1 * pos[o] + l2 * pos[o + 3] + l3 * pos[o + 6];
        if (hx > ox) crossings++;
    }
    return (crossings & 1) == 1;
}

}} // namespace
