// Port of web/prop/surface.js. See Surface.hpp.
#include "Surface.hpp"
#include <algorithm>
#include <cmath>

namespace Slic3r { namespace SupportFins {

Vec3d &seat(const Soup &pos, size_t i, const Rot &rot, const Offset &off, Vec3d &out) {
    const double x = pos[i], y = pos[i + 1], z = pos[i + 2];
    out[0] = rot[0] * x + rot[3] * y + rot[6] * z + off.x;
    out[1] = rot[1] * x + rot[4] * y + rot[7] * z + off.y;
    out[2] = rot[2] * x + rot[5] * y + rot[8] * z + off.z;
    return out;
}

/*
 * XY bucket grid over a `tris` array, so surfaceZAt scans only the triangles whose
 * footprint covers the query column instead of the whole mesh. The twin of
 * inside.js's YZ grid. Same barycentric test + lowest-z semantics as a linear scan.
 */
namespace {
// Math.min(63, Math.max(0, Math.floor(v))) as a cell index; NaN -> -1 (JS: a NaN
// index, which a typed array ignores and every loop bound rejects).
inline int cellIdx(double v) {
    const double f = std::floor(v);
    if (std::isnan(f)) return -1;
    return int(std::min(double(ZGRID - 1), std::max(0.0, f)));
}
inline double min3(double a, double b, double c) {
    if (std::isnan(a) || std::isnan(b) || std::isnan(c)) return NAN;
    return std::min(std::min(a, b), c);
}
inline double max3(double a, double b, double c) {
    if (std::isnan(a) || std::isnan(b) || std::isnan(c)) return NAN;
    return std::max(std::max(a, b), c);
}
} // namespace

ZGrid buildZGrid(const Soup &tris) {
    ZGrid g;
    double minX = INF, minY = INF, maxX = -INF, maxY = -INF;
    for (size_t i = 0; i < tris.size(); i += 3) {
        const double x = tris[i], y = i + 1 < tris.size() ? tris[i + 1] : NAN;
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
    }
    const double dX = maxX - minX, dY = maxY - minY;
    const double sx = ZGRID / (std::isnan(dX) ? dX : std::max(1e-6, dX));
    const double sy = ZGRID / (std::isnan(dY) ? dY : std::max(1e-6, dY));
    auto cx = [&](double x) { return cellIdx((x - minX) * sx); };
    auto cy = [&](double y) { return cellIdx((y - minY) * sy); };
    // [a0, a1, b0, b1]; false when any bound is NaN (the JS loops then run zero times)
    auto span = [&](size_t f, int s[4]) {
        const double ax = tris[f], ay = tris[f + 1], bx = tris[f + 3], by = tris[f + 4],
                     dx = tris[f + 6], dy = tris[f + 7];
        s[0] = cx(min3(ax, bx, dx)); s[1] = cx(max3(ax, bx, dx));
        s[2] = cy(min3(ay, by, dy)); s[3] = cy(max3(ay, by, dy));
        return s[0] >= 0 && s[1] >= 0 && s[2] >= 0 && s[3] >= 0;
    };
    const int N = ZGRID * ZGRID;
    std::vector<int32_t> counts(size_t(N) + 1, 0);
    int s[4];
    for (size_t f = 0; f + 8 < tris.size(); f += 9) {   // a partial last face reads NaN in JS: skipped
        if (!span(f, s)) continue;
        for (int a = s[0]; a <= s[1]; a++) for (int b = s[2]; b <= s[3]; b++) counts[size_t(a * ZGRID + b + 1)]++;
    }
    for (int i = 0; i < N; i++) counts[size_t(i) + 1] += counts[size_t(i)];
    std::vector<int32_t> items(static_cast<size_t>(counts[size_t(N)]));
    std::vector<int32_t> cur(counts.begin(), counts.begin() + N);
    for (size_t f = 0; f + 8 < tris.size(); f += 9) {
        if (!span(f, s)) continue;
        for (int a = s[0]; a <= s[1]; a++) for (int b = s[2]; b <= s[3]; b++) items[size_t(cur[size_t(a * ZGRID + b)]++)] = int32_t(f);
    }
    g.start = std::move(counts);
    g.items = std::move(items);
    g.minX = minX; g.minY = minY; g.maxX = maxX; g.maxY = maxY;
    g.sx = sx; g.sy = sy;
    return g;
}

const ZGrid &SurfTris::grid() const {
    if (!_grid) _grid = std::make_shared<const ZGrid>(buildZGrid(pos));
    return *_grid;
}

namespace {
// The grid cell's candidate triangles for column (x, y), or -1 outside the grid.
int cellOf(const ZGrid &g, double x, double y) {
    const double e = 1e-9;
    if (x < g.minX - e || x > g.maxX + e || y < g.minY - e || y > g.maxY + e) return -1;
    const int a = cellIdx((x - g.minX) * g.sx);
    const int b = cellIdx((y - g.minY) * g.sy);
    if (a < 0 || b < 0) return -1;   // NaN query: JS start[NaN] is undefined, no candidates
    return a * ZGRID + b;
}

// Height of triangle `i` of `tris` at column (x, y), or nullopt if the column misses it.
std::optional<double> zOn(const Soup &tris, size_t i, double x, double y) {
    const double ax = tris[i], ay = tris[i + 1], az = tris[i + 2];
    const double bx = tris[i + 3], by = tris[i + 4], bz = tris[i + 5];
    const double cx = tris[i + 6], cy = tris[i + 7], cz = tris[i + 8];
    const double den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
    if (std::abs(den) < 1e-12) return std::nullopt;
    const double l1 = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / den;
    const double l2 = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / den;
    const double l3 = 1 - l1 - l2;
    if (l1 < -1e-9 || l2 < -1e-9 || l3 < -1e-9) return std::nullopt;
    return l1 * az + l2 * bz + l3 * cz;
}
} // namespace

std::optional<double> surfaceZAt(const SurfTris &tris, double x, double y) {
    if (tris.size() == 0) return std::nullopt;
    const ZGrid &g = tris.grid();
    const int c = cellOf(g, x, y);
    if (c < 0) return std::nullopt;
    double best = INF;
    for (int32_t k = g.start[size_t(c)]; k < g.start[size_t(c) + 1]; k++) {
        const auto z = zOn(tris.pos, size_t(g.items[size_t(k)]), x, y);
        if (z && *z < best) best = *z;
    }
    if (best == INF) return std::nullopt;
    return best;
}

std::vector<SurfHit> surfaceHitsAt(const SurfTris &tris, double x, double y) {
    std::vector<SurfHit> hits;
    if (tris.size() == 0) return hits;
    const ZGrid &g = tris.grid();
    const int c = cellOf(g, x, y);
    if (c < 0) return hits;
    const Soup &t = tris.pos;
    for (int32_t k = g.start[size_t(c)]; k < g.start[size_t(c) + 1]; k++) {
        const size_t i = size_t(g.items[size_t(k)]);
        const auto z = zOn(t, i, x, y);
        if (!z) continue;
        const double ux = t[i + 3] - t[i], uy = t[i + 4] - t[i + 1], uz = t[i + 5] - t[i + 2];
        const double vx = t[i + 6] - t[i], vy = t[i + 7] - t[i + 1], vz = t[i + 8] - t[i + 2];
        const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        double hn = hypot3(nx, ny, nz);
        if (hn == 0 || std::isnan(hn)) hn = 1;   // JS `|| 1`
        hits.push_back({*z, nz / hn});
    }
    return hits;
}

std::vector<double> surfaceZsAt(const SurfTris &tris, double x, double y) {
    std::vector<double> zs;
    if (tris.size() == 0) return zs;
    const ZGrid &g = tris.grid();
    const int c = cellOf(g, x, y);
    if (c < 0) return zs;
    for (int32_t k = g.start[size_t(c)]; k < g.start[size_t(c) + 1]; k++) {
        const auto z = zOn(tris.pos, size_t(g.items[size_t(k)]), x, y);
        if (z) zs.push_back(*z);
    }
    return zs;
}

}} // namespace
