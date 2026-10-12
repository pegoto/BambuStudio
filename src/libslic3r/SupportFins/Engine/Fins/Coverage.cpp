// Port of web/fins/coverage.js -- PLATE_Z, reachOf, MUST / HOLE / LOW, classify.
#include "Coverage.hpp"
#include "../Inside.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace Slic3r { namespace SupportFins {

namespace {

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

/// A JS Map key: SameValueZero, so -0 is +0 (NaN never occurs for finite tops/queries).
inline double mapKey(double i, double j) { const double k = i * 100003 + j; return k == 0 ? 0.0 : k; }

// A TINY HOLE is a horizontal bore this wide or less (~6 mm; the DRO housing's
// top-plate holes are 3 mm). Its ceiling's overhang band (normal past 45deg) is a
// chord of 0.71 x the diameter, so a patch narrower than that, with part under it
// within the diameter and part either side just under its apex, is a hole's ceiling.
constexpr double HOLE_D = 6.0;
// The NEAR-PLATE strip: under the squat-wall floor (0.6 wall + 0.2 gap), where no wall
// fits. Below 0.6 the plate already holds it, so what LOW leaves unheld is 0.6-0.82.
constexpr double LOW_Z = 0.82;

} // namespace

void Reach::add(const Vec3d &p) {
    grid[mapKey(std::floor(p[0] / cell), std::floor(p[1] / cell))].push_back(p);
}

bool Reach::operator()(double cx, double cy, double cz) const {
    const double gi = std::floor(cx / cell), gj = std::floor(cy / cell);
    for (int di = -1; di <= 1; di++) for (int dj = -1; dj <= 1; dj++) {
        const auto it = grid.find(mapKey(gi + di, gj + dj));
        if (it == grid.end()) continue;
        for (const Vec3d &p : it->second) {
            const double dz = cz - p[2];
            if (dz >= -0.05 && dz <= 1.5 && hypot2(cx - p[0], cy - p[1]) <= R) return true;
        }
    }
    return false;
}

/**
 * reachOf(tops, R)(x, y, z): is there a wall-top point within R of (x, y) in plan and
 * 0..1.5 mm under z? The tops are gridded by plan cell, so a 1M-face mini isn't
 * faces x tops. `add(p)` grows the set (the fill pass adds its walls').
 */
Reach reachOf(const std::vector<Vec3d> &tops, double R) {
    Reach near;
    near.R = R;
    near.cell = jsMax(R, 1);
    for (const Vec3d &p : tops) near.add(p);
    return near;
}

std::vector<uint8_t> classify(const Topology &topo, const AnalyzeResult &res, const Rot &rot) {
    const Soup &pos = topo.pos;
    const int nFaces = topo.nFaces;
    const std::vector<int> &adjA = topo.adjA, &adjB = topo.adjB;
    const std::vector<double> &area = topo.area, &nrm = topo.nrm;
    const Offset off = res.offset;
    std::vector<uint8_t> cls(size_t(nFaces), 0);
    auto seatV = [&](size_t o) { return seat(rot, off, pos[o], pos[o + 1], pos[o + 2]); };
    // connected overhang patches, union-find over shared edges
    std::vector<int32_t> parent(static_cast<size_t>(nFaces));
    for (int i = 0; i < nFaces; i++) parent[size_t(i)] = i;
    auto find = [&](int32_t i) {
        while (parent[size_t(i)] != i) { const int32_t g = parent[size_t(parent[size_t(i)])]; parent[size_t(i)] = g; i = g; }
        return i;
    };
    for (size_t k = 0; k < adjA.size(); k++) {
        const int a = adjA[k], b = adjB[k];
        if (res.over[size_t(a)] && res.over[size_t(b)]) {
            const int32_t ra = find(a);   // JS evaluates parent[find(a)]'s key first
            const int32_t rb = find(b);
            parent[size_t(ra)] = rb;
        }
    }
    // JS Map root -> faces, iterated in insertion order
    std::unordered_map<int32_t, size_t> patchOf;
    std::vector<std::vector<int>> patches;
    for (int f = 0; f < nFaces; f++) if (res.over[size_t(f)]) {
        const int32_t r = find(f);
        auto it = patchOf.find(r);
        if (it == patchOf.end()) { it = patchOf.emplace(r, patches.size()).first; patches.emplace_back(); }
        patches[it->second].push_back(f);
    }
    auto inside = [&](double x, double y, double z) { return insidePart(topo, rot, off, x, y, z); };
    for (const std::vector<int> &faces : patches) {
        // plan extent across the patch's minor axis (2D PCA of its vertices)
        std::vector<Vec3d> pts;
        pts.reserve(faces.size() * 3);
        double top = -INF;
        for (const int f : faces) for (int i = 0; i < 3; i++) {
            const Vec3d v = seatV(size_t(f) * 9 + i * 3);
            pts.push_back(v);
            if (v[2] > top) top = v[2];
        }
        double mx = 0, my = 0;
        for (const Vec3d &v : pts) { mx += v[0]; my += v[1]; }
        mx /= double(pts.size()); my /= double(pts.size());
        double sxx = 0, sxy = 0, syy = 0;
        for (const Vec3d &v : pts) { const double dx = v[0] - mx, dy = v[1] - my; sxx += dx * dx; sxy += dx * dy; syy += dy * dy; }
        const double tr = sxx + syy, det = sxx * syy - sxy * sxy;
        const double lam = tr / 2 + std::sqrt(jsMax(0, (tr * tr) / 4 - det));
        double ux = sxy, uy = lam - sxx;
        if (hypot2(ux, uy) < 1e-9) { ux = 1; uy = 0; }
        const double un = hypot2(ux, uy); ux /= un; uy /= un;
        // Across the bore is whichever axis has part on BOTH sides: a short bore is about
        // as long as it is wide, so the principal axis can land either way.
        auto extent = [&](double ax, double ay) {
            double lo = INF, hi = -INF;
            for (const Vec3d &v : pts) { const double t = (v[0] - mx) * ax + (v[1] - my) * ay; lo = jsMin(lo, t); hi = jsMax(hi, t); }
            return hi - lo;
        };
        // probed from the patch's plan centre at the apex height: the highest vertex
        // itself can sit at the bore's mouth, where every probe grazes the boundary
        const double z = top - 0.5;                     // just under the apex
        const std::array<double, 3> apex{mx, my, top};
        // part either side, equally far: a round bore is symmetric about its apex
        auto wallAt = [&](double ax, double ay, double sg) -> std::optional<double> {
            for (double d = 0.25; d <= HOLE_D / 2 + 1; d += 0.25) if (inside(apex[0] + sg * ax * d, apex[1] + sg * ay * d, z)) return d;
            return std::nullopt;
        };
        auto walled = [&](double ax, double ay) {
            const auto a = wallAt(ax, ay, 1), b = wallAt(ax, ay, -1);
            return a && b && std::abs(*a - *b) <= 0.5;
        };
        // a CYLINDER: every ceiling face's normal is square to the bore axis, which runs
        // across the walled direction
        auto square = [&](double ax, double ay) {
            double offA = 0, tot = 0;
            for (const int f : faces) {
                double n[3];
                for (int k = 0; k < 3; k++) n[k] = rot[k] * nrm[size_t(f) * 3] + rot[3 + k] * nrm[size_t(f) * 3 + 1] + rot[6 + k] * nrm[size_t(f) * 3 + 2];
                offA += area[size_t(f)] * std::abs(n[0] * -ay + n[1] * ax);   // component along the bore
                tot += area[size_t(f)];
            }
            return tot > 0 && offA / tot < 0.15;
        };
        auto narrow = [&](double ax, double ay) { return extent(ax, ay) <= 0.71 * HOLE_D + 0.25 && walled(ax, ay) && square(ax, ay); };
        // and a CEILING: some face within ~14deg of straight down -- a bore's apex or a
        // slot's flat roof (a V-notch passes every test above and is no bridge)
        bool roof = false;
        for (const int f : faces) {
            if (rot[2] * nrm[size_t(f) * 3] + rot[5] * nrm[size_t(f) * 3 + 1] + rot[8] * nrm[size_t(f) * 3 + 2] < -0.97) { roof = true; break; }
        }
        // the probes start at the patch's plan centre, so that must be AIR: a flat collar
        // round a post has its centre over the stem
        bool hole = roof && !inside(apex[0], apex[1], z) && (narrow(-uy, ux) || narrow(ux, uy));
        if (hole) {
            // under the apex: the bore's floor within a diameter
            bool floor = false;
            for (double d = 0.5; d <= HOLE_D + 1e-9 && !floor; d += 0.5) floor = inside(apex[0], apex[1], apex[2] - d);
            hole = floor;
        }
        for (const int f : faces) {
            if (hole) { cls[size_t(f)] = HOLE; continue; }
            double cz = 0;
            for (int i = 0; i < 3; i++) cz += seatV(size_t(f) * 9 + i * 3)[2] / 3;
            cls[size_t(f)] = cz < LOW_Z ? LOW : MUST;
        }
    }
    return cls;
}

}} // namespace
