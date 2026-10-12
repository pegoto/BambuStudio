// Port of web/prop/lattice.js. See Lattice.hpp; the measurements behind NET_HALF /
// MIN_STRUTS and the issue history are in the upstream file.
#include "Lattice.hpp"
#include "../PropConfig.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace Slic3r { namespace SupportFins {

namespace {
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
inline double jsMin3(double a, double b, double c) { return jsMin(jsMin(a, b), c); }
inline double jsMax3(double a, double b, double c) { return jsMax(jsMax(a, b), c); }
} // namespace

std::optional<Struts> latticeStruts(const Topology &topo, const std::vector<int> &faces, const Seated &seated) {
    const double CELL = LATTICE_CELL;
    double area = 0;
    for (int f : faces) area += topo.area[size_t(f)];
    if (area < PROP.tubeMinArea) return std::nullopt;   // small regions already take the tube route

    // --- footprint raster ----------------------------------------------------
    std::vector<std::array<Vec3d, 3>> tri;
    tri.reserve(faces.size());
    for (int f : faces) tri.push_back(seated(f));
    double x0 = INF, y0 = INF, x1 = -INF, y1 = -INF;
    for (const auto &t : tri) for (const Vec3d &v : t) {
        if (v[0] < x0) x0 = v[0]; if (v[0] > x1) x1 = v[0];
        if (v[1] < y0) y0 = v[1]; if (v[1] > y1) y1 = v[1];
    }
    x0 -= 2 * CELL; y0 -= 2 * CELL;
    const double Wd = std::ceil((x1 - x0) / CELL) + 3, Hd = std::ceil((y1 - y0) / CELL) + 3;
    if (Wd * Hd > 4e6) return std::nullopt;             // a 1 m part: not a strut net
    if (std::isnan(Wd * Hd)) return std::nullopt;       // NaN vertices: JS rasters nothing (Uint8Array(NaN) is empty); unreachable for a real region
    const long W = long(Wd), H = long(Hd), WH = W * H;
    auto idx = [&](long i, long j) { return j * W + i; };
    auto cellI = [&](double x) { return long(std::floor((x - x0) / CELL)); };
    auto cellJ = [&](double y) { return long(std::floor((y - y0) / CELL)); };
    // JS typed arrays: an out-of-range read is undefined (falsy), a write is dropped.
    auto get = [&](const std::vector<uint8_t> &a, long k) -> uint8_t { return k >= 0 && k < WH ? a[size_t(k)] : 0; };
    std::vector<uint8_t> fp(static_cast<size_t>(WH), 0);
    for (const auto &t : tri) {
        const Vec3d &a = t[0], &b = t[1], &c = t[2];
        const long i0 = cellI(jsMin3(a[0], b[0], c[0])), j0 = cellJ(jsMin3(a[1], b[1], c[1]));
        const long i1 = cellI(jsMax3(a[0], b[0], c[0])), j1 = cellJ(jsMax3(a[1], b[1], c[1]));
        const double d = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
        for (long j = j0; j <= j1; j++) for (long i = i0; i <= i1; i++) {
            const double px = x0 + (i + 0.5) * CELL, py = y0 + (j + 0.5) * CELL;
            const long k = idx(i, j);
            if (std::abs(d) < 1e-12) { if (k >= 0 && k < WH) fp[size_t(k)] = 1; continue; }   // edge-on
            const double u = ((b[0] - px) * (c[1] - py) - (c[0] - px) * (b[1] - py)) / d;
            const double v = ((c[0] - px) * (a[1] - py) - (a[0] - px) * (c[1] - py)) / d;
            const double tol = CELL / std::sqrt(std::abs(d));   // a half-cell grace, so thin strips stay connected
            if (u >= -tol && v >= -tol && 1 - u - v >= -tol) { if (k >= 0 && k < WH) fp[size_t(k)] = 1; }
        }
    }

    // --- narrow everywhere? (distance to the footprint's edge, chamfer 3-4) ----
    std::vector<float> dist(static_cast<size_t>(WH));   // Float32Array
    for (long k = 0; k < WH; k++) dist[size_t(k)] = fp[size_t(k)] ? 1e9f : 0.0f;
    auto relax = [&](long k, long n, double w) {
        const double t = double(dist[size_t(n)]) + w;
        if (t < double(dist[size_t(k)])) dist[size_t(k)] = float(t);
    };
    for (long j = 1; j < H - 1; j++) for (long i = 1; i < W - 1; i++) {
        const long k = idx(i, j); if (!fp[size_t(k)]) continue;
        relax(k, k - 1, 3); relax(k, k - W, 3); relax(k, k - W - 1, 4); relax(k, k - W + 1, 4);
    }
    for (long j = H - 2; j > 0; j--) for (long i = W - 2; i > 0; i--) {
        const long k = idx(i, j); if (!fp[size_t(k)]) continue;
        relax(k, k + 1, 3); relax(k, k + W, 3); relax(k, k + W + 1, 4); relax(k, k + W - 1, 4);
    }
    std::vector<double> inside;
    for (long k = 0; k < WH; k++) if (fp[size_t(k)]) inside.push_back((double(dist[size_t(k)]) / 3) * CELL);
    std::stable_sort(inside.begin(), inside.end(), [](double a, double b) { return a - b < 0; });
    if (inside.empty() || inside[size_t(std::floor(0.99 * double(inside.size() - 1)))] > LATTICE_NET_HALF) return std::nullopt;

    // --- skeleton (Zhang-Suen thinning) --------------------------------------
    std::vector<uint8_t> sk = fp;
    auto nb = [&](long k, int p[8]) {   // p2..p9, clockwise from north
        p[0] = get(sk, k - W); p[1] = get(sk, k - W + 1); p[2] = get(sk, k + 1); p[3] = get(sk, k + W + 1);
        p[4] = get(sk, k + W); p[5] = get(sk, k + W - 1); p[6] = get(sk, k - 1); p[7] = get(sk, k - W - 1);
    };
    for (bool changed = true; changed;) {
        changed = false;
        for (int pass : {0, 1}) {
            std::vector<long> del;
            for (long j = 1; j < H - 1; j++) for (long i = 1; i < W - 1; i++) {
                const long k = idx(i, j); if (!sk[size_t(k)]) continue;
                int p[8];
                nb(k, p);
                int B = 0;
                for (int q = 0; q < 8; q++) B += p[q];
                if (B < 2 || B > 6) continue;
                int A = 0; for (int q = 0; q < 8; q++) if (!p[q] && p[(q + 1) % 8]) A++;
                if (A != 1) continue;
                if (pass == 0 ? (p[0] * p[2] * p[4] != 0 || p[2] * p[4] * p[6] != 0) : (p[0] * p[2] * p[6] != 0 || p[0] * p[4] * p[6] != 0)) continue;
                del.push_back(k);
            }
            for (long k : del) sk[size_t(k)] = 0;
            if (!del.empty()) changed = true;
        }
    }

    // --- cut at junctions into branches --------------------------------------
    const long N8[8] = {-W - 1, -W, -W + 1, -1, 1, W - 1, W, W + 1};
    std::vector<long> junction;
    for (long k = 0; k < WH; k++) {
        if (!sk[size_t(k)]) continue;
        int n = 0; for (long d : N8) if (get(sk, k + d)) n++;
        if (n >= 3) junction.push_back(k);
    }
    if (junction.empty()) return std::nullopt;          // one band or a ring: not a net
    std::vector<uint8_t> cut = sk;
    const long r = long(std::ceil(LATTICE_NODE_CUT));
    for (long k : junction) {
        const long i = k % W, j = (k - i) / W;
        for (long dj = -r; dj <= r; dj++) for (long di = -r; di <= r; di++) {
            const long ii = i + di, jj = j + dj;
            if (ii >= 0 && jj >= 0 && ii < W && jj < H) cut[size_t(idx(ii, jj))] = 0;
        }
    }
    std::vector<int32_t> label(size_t(WH), -1);
    auto labelAt = [&](long k) -> bool { return k >= 0 && k < WH && label[size_t(k)] < 0; };   // JS `label[k] < 0` (undefined < 0 is false)
    int branches = 0;
    for (long k = 0; k < WH; k++) {
        if (!cut[size_t(k)] || label[size_t(k)] >= 0) continue;
        std::vector<long> q{k}; label[size_t(k)] = branches;
        while (!q.empty()) {
            const long c = q.back(); q.pop_back();
            for (long d : N8) if (get(cut, c + d) && labelAt(c + d)) { label[size_t(c + d)] = branches; q.push_back(c + d); }
        }
        branches++;
    }
    if (branches < 2) return std::nullopt;

    // --- every footprint cell to its nearest live branch, through the footprint,
    // then faces by the branch under their centroid; spurs under tubeSmallMinArea
    // are dropped and the flood rerun.
    const std::vector<int32_t> seed = label;
    std::vector<long> centre;
    centre.reserve(faces.size());
    for (size_t n = 0; n < faces.size(); n++) {
        const auto &t = tri[n];
        const long i = cellI((t[0][0] + t[1][0] + t[2][0]) / 3), j = cellJ((t[0][1] + t[1][1] + t[2][1]) / 3);
        centre.push_back(idx(i, j));
    }
    std::vector<uint8_t> dead(size_t(branches), 0);
    std::vector<std::vector<int>> groups;
    for (;;) {
        std::vector<long> front;
        for (long k = 0; k < WH; k++) {
            label[size_t(k)] = seed[size_t(k)] >= 0 && !dead[size_t(seed[size_t(k)])] ? seed[size_t(k)] : -1;
            if (label[size_t(k)] >= 0) front.push_back(k);
        }
        while (!front.empty()) {
            std::vector<long> next;
            for (long c : front) for (long d : N8) {
                const long n = c + d;
                if (get(fp, n) && labelAt(n)) { label[size_t(n)] = label[size_t(c)]; next.push_back(n); }
            }
            front = std::move(next);
        }
        groups.assign(size_t(branches), std::vector<int>{});
        for (size_t n = 0; n < faces.size(); n++) {
            const long ck = centre[n];
            const int b = ck >= 0 && ck < WH ? label[size_t(ck)] : -1;
            if (b >= 0) groups[size_t(b)].push_back(faces[n]);
        }
        bool pruned = false;
        for (size_t b = 0; b < groups.size(); b++) {
            const auto &g = groups[b];
            if (g.empty()) continue;
            double s = 0;
            for (int f : g) s += topo.area[size_t(f)];
            if (s < PROP.tubeSmallMinArea) { dead[b] = 1; pruned = true; }
        }
        if (!pruned) break;
    }
    Struts sized;
    for (auto &g : groups) if (!g.empty()) sized.push_back(std::move(g));
    if (long(sized.size()) < LATTICE_MIN_STRUTS) return std::nullopt;
    // Faces no live branch reaches get no wall: counted as the caller's slivers.
    size_t got = 0;
    for (const auto &g : sized) got += g.size();
    sized.lost = int(faces.size()) - int(got);
    return sized;
}

std::vector<PatchPtr> strutPatches(const Topology &topo, const Rot &rot, const std::vector<std::vector<int>> &groups,
                                   const Seated &seated, double step, int region,
                                   const std::shared_ptr<const SurfTris> &regionTris) {
    std::vector<PatchPtr> patches;
    for (const std::vector<int> &faces : groups) {
        Soup flat(faces.size() * 9);
        std::vector<Vec3d> pts;
        double area = 0;
        for (size_t k = 0; k < faces.size(); k++) {
            const int f = faces[k];
            const std::array<Vec3d, 3> t = seated(f);
            area += topo.area[size_t(f)];
            for (size_t i = 0; i < 3; i++) {
                for (size_t c = 0; c < 3; c++) flat[k * 9 + i * 3 + c] = t[i][Eigen::Index(c)];
                pts.push_back(t[i]);
            }
            Vec3d g;
            for (int c = 0; c < 3; c++) g[c] = (t[0][c] + t[1][c] + t[2][c]) / 3;
            pts.push_back(g);
        }
        const auto tris = std::make_shared<SurfTris>(std::move(flat));
        const Vec2d axis = longAxis(pts);
        std::optional<std::vector<ContactLine>> tube = tubeLine(topo, faces, rot, pts, *tris, step);
        if (tube && !tube->empty()) {
            // raced against its split patches (rival.js) when small, as a standalone
            // small tube is; over tubeMinArea it is a plain tube
            const bool small = area < PROP.tubeMinArea;
            auto p = std::make_shared<PropPatch>();
            if (small) {
                std::vector<PatchPtr> split;
                for (PropPatch &s : splitRegion(topo, faces, rot)) {
                    s.strut = true;
                    s.axis = axis;
                    split.push_back(std::make_shared<PropPatch>(std::move(s)));
                }
                p->smallTube = std::move(split);
            }
            p->faces = faces; p->area = area; p->region = region; p->tris = regionTris; p->reach = tris;
            p->lines = std::move(*tube); p->strut = true;
            patches.push_back(std::move(p));
            continue;
        }
        // Not a tube: the WHOLE strut is one patch.
        auto p = std::make_shared<PropPatch>();
        p->faces = faces; p->area = area; p->region = region; p->tris = regionTris; p->strut = true; p->axis = axis;
        patches.push_back(std::move(p));
    }
    return patches;
}

Vec2d longAxis(const std::vector<Vec3d> &pts) {
    double cx = 0, cy = 0;
    for (const Vec3d &p : pts) { cx += p[0]; cy += p[1]; }
    cx /= double(pts.size()); cy /= double(pts.size());
    double sxx = 0, sxy = 0, syy = 0;
    for (const Vec3d &p : pts) { const double dx = p[0] - cx, dy = p[1] - cy; sxx += dx * dx; sxy += dx * dy; syy += dy * dy; }
    const double tr = sxx + syy, det = sxx * syy - sxy * sxy;
    const double lam = tr / 2 + std::sqrt(jsMax(0, (tr * tr) / 4 - det));
    double ux = sxy, uy = lam - sxx;
    if (hypot2(ux, uy) < 1e-9) { ux = sxx >= syy ? 1 : 0; uy = sxx >= syy ? 0 : 1; }
    const double n = hypot2(ux, uy);
    return Vec2d(ux / n, uy / n);
}

}} // namespace
