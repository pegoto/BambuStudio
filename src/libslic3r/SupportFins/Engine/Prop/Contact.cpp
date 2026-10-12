// Port of web/prop/contact.js. See Contact.hpp; the reasoning behind each pass (and
// the measurements that chose its numbers) is in the upstream file.
#include "Contact.hpp"
#include "../PropConfig.hpp"
#include <algorithm>
#include <cmath>

namespace Slic3r { namespace SupportFins {

double straightness(const ContactLine &line) {
    const size_t n = line.size();
    if (n < 3) return INF;

    double cx = 0, cy = 0;
    for (const auto &p : line) { cx += p[0]; cy += p[1]; }
    cx /= double(n); cy /= double(n);

    double sxx = 0, sxy = 0, syy = 0;
    for (const auto &p : line) {
        const double dx = p[0] - cx, dy = p[1] - cy;
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    const double tr = sxx + syy, det = sxx * syy - sxy * sxy;
    const double lam = tr / 2 + std::sqrt(std::max(0.0, (tr * tr) / 4 - det));
    double ax = sxy, ay = lam - sxx;
    if (hypot2(ax, ay) < 1e-9) { ax = 1; ay = 0; }
    const double an = hypot2(ax, ay); ax /= an; ay /= an;

    double ss = 0, lo = INF, hi = -INF;
    for (const auto &p : line) {
        const double dx = p[0] - cx, dy = p[1] - cy;
        const double t = dx * ax + dy * ay;
        const double perp = -dx * ay + dy * ax;
        ss += perp * perp;
        if (t < lo) lo = t;
        if (t > hi) hi = t;
    }
    const double chord = hi - lo;
    return chord < 1e-9 ? INF : std::sqrt(ss / double(n)) / chord;
}

std::optional<ContactLine> contactLine(const std::vector<Vec3d> &pts, const SurfTris &tris, int nSamples) {
    if (pts.size() < 3) return std::nullopt;

    double cx = 0, cy = 0;
    for (const auto &p : pts) { cx += p[0]; cy += p[1]; }
    cx /= double(pts.size()); cy /= double(pts.size());

    // principal horizontal direction, via the 2x2 covariance of the XY spread
    double sxx = 0, sxy = 0, syy = 0;
    for (const auto &p : pts) {
        const double dx = p[0] - cx, dy = p[1] - cy;
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    const double tr = sxx + syy, det = sxx * syy - sxy * sxy;
    const double lam = tr / 2 + std::sqrt(std::max(0.0, (tr * tr) / 4 - det));
    double ax = sxy, ay = lam - sxx;
    if (hypot2(ax, ay) < 1e-9) { ax = 1; ay = 0; }
    const double an = hypot2(ax, ay); ax /= an; ay /= an;

    double lo = INF, hi = -INF;
    for (const auto &p : pts) {
        const double t = (p[0] - cx) * ax + (p[1] - cy) * ay;
        if (t < lo) lo = t;
        if (t > hi) hi = t;
    }
    if (hi - lo < 1e-6) return std::nullopt;

    std::vector<int> buckets(size_t(std::max(0, nSamples)), -1);   // index into pts, -1 = null
    for (size_t i = 0; i < pts.size(); i++) {
        const Vec3d &p = pts[i];
        const double t = (p[0] - cx) * ax + (p[1] - cy) * ay;
        double k = std::floor(((t - lo) / (hi - lo)) * nSamples);
        if (k >= nSamples) k = nSamples - 1;
        const size_t kk = size_t(k);
        if (buckets[kk] < 0 || p[2] < pts[size_t(buckets[kk])][2]) buckets[kk] = int(i);
    }
    ContactLine line;
    for (int b : buckets) if (b >= 0) line.push_back(pts[size_t(b)]);
    if (line.size() < 3) return std::nullopt;

    // Re-fit each station's height to the surface directly above its OWN xy: the
    // bucket only chose where the wall should stand.
    for (size_t i = 0; i < line.size(); i++) {
        const auto zz = surfaceZAt(tris, line[i][0], line[i][1]);
        if (zz) line[i] = Vec3d(line[i][0], line[i][1], *zz);
    }

    contourTop(line, tris);
    lowerSag(line, tris);
    return line;
}

ContactLine &lowerSag(ContactLine &line, const SurfTris &tris, double band) {
    const int n = int(line.size());
    for (int pass = 0; pass < 3; pass++) {
        bool moved = false;
        for (int i = 0; i < n - 1; i++) {
            const double mx = (line[i][0] + line[i + 1][0]) / 2;
            const double my = (line[i][1] + line[i + 1][1]) / 2;
            const auto zz = surfaceZAt(tris, mx, my);
            if (!zz) continue;
            const double midWall = (line[i][2] + line[i + 1][2]) / 2;
            const double over = midWall - *zz;
            // `band`: a PART-ATTACHED top settles against the OVERHANG, not the floor
            if (over > band) continue;
            if (over > 1e-4) {
                line[i] = Vec3d(line[i][0], line[i][1], line[i][2] - over);
                line[i + 1] = Vec3d(line[i + 1][0], line[i + 1][1], line[i + 1][2] - over);
                moved = true;
            }
        }
        if (!moved) break;
    }
    return line;
}

ContactLine &contourTop(ContactLine &line, const SurfTris &tris, double band) {
    const double half = PROP.tip / 2;
    const int n = int(line.size());
    for (int i = 0; i < n; i++) {
        const Vec3d a = line[size_t(std::max(0, i - 1))];
        const Vec3d b = line[size_t(std::min(n - 1, i + 1))];
        const double rx = b[0] - a[0], ry = b[1] - a[1];
        const double rn = hypot2(rx, ry);
        if (rn < 1e-9) continue;
        const double sx = ry / rn, sy = -rx / rn;     // across the wall

        const double z0 = line[i][2];
        double z = z0;
        for (const double o : {-half, half}) {
            const auto zz = surfaceZAt(tris, line[i][0] + sx * o, line[i][1] + sy * o);
            // within `band` of the overhang only: past that it is the floor
            if (zz && *zz < z && z0 - *zz <= band) z = *zz;
        }
        line[i] = Vec3d(line[i][0], line[i][1], z);
    }
    return line;
}

ContactLine &settleTop(ContactLine &line, const SurfTris &tris, double step, double band) {
    const double half = PROP.tip / 2;
    const int n = int(line.size());

    // PER STATION, not one shift for the whole wall; iterated, because lowering a
    // station changes what its neighbours measure.
    for (int pass = 0; pass < 3; pass++) {
        std::vector<double> drop(size_t(n), 0.0);
        bool moved = false;

        for (int i = 0; i < n - 1; i++) {
            const Vec3d p = line[i], q = line[i + 1];
            const double dx = q[0] - p[0], dy = q[1] - p[1];
            const double seg = hypot2(dx, dy);
            if (seg < 1e-9) continue;
            const double sx = dy / seg, sy = -dx / seg;
            const double nn = std::max(1.0, std::ceil(seg / step));

            for (int k = 0; k <= nn; k++) {
                const double t = k / nn;
                const double x = p[0] + dx * t, y = p[1] + dy * t;
                const double topZ = (p[2] + (q[2] - p[2]) * t) - PROP.gap;
                for (const double o : {-half, 0.0, half}) {
                    const auto zz = surfaceZAt(tris, x + sx * o, y + sy * o);
                    if (!zz) continue;
                    // LOWER ONLY, never raise; a surface more than `band` below the
                    // top is the floor, not the overhang
                    if (topZ - *zz > band) continue;
                    const double need = PROP.gap - (*zz - topZ);
                    if (need <= 1e-4) continue;
                    // charge the deficit to whichever end of the span owns this sample
                    const int j = t < 0.5 ? i : i + 1;
                    if (need > drop[size_t(j)]) { drop[size_t(j)] = need; moved = true; }
                }
            }
        }

        if (!moved) break;
        for (int i = 0; i < n; i++) {
            if (drop[size_t(i)] > 0) line[i] = Vec3d(line[i][0], line[i][1], line[i][2] - drop[size_t(i)]);
        }
    }
    return line;
}

}} // namespace
