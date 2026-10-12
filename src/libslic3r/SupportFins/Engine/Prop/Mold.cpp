// Port of web/prop/mold.js. See Mold.hpp; the measurements behind each rule are in
// the upstream file.
#include "Mold.hpp"
#include "../PropConfig.hpp"
#include <algorithm>
#include <array>

namespace Slic3r { namespace SupportFins {

namespace {
// Math.max(a, b): NaN-propagating, +0 over -0.
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NAN;
    if (a == b) return std::signbit(a) ? b : a;
    return a > b ? a : b;
}
// Math.min(a, b): NaN-propagating, -0 under +0.
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NAN;
    if (a == b) return std::signbit(a) ? a : b;
    return a < b ? a : b;
}
inline double jsMaxOf(const std::vector<double> &v) {   // Math.max(...v)
    double m = -INF;
    for (double x : v) m = jsMax(m, x);
    return m;
}

/*
 * The floor under each side of a lifted bottom, so it can TILT with the part instead
 * of hanging level off the highest point. Returns [zNeg, zPos] for the -w and +w
 * sides. See the upstream comment for the MAX_DROP / MAX_HANG switch.
 */
std::array<double, 2> sideFloors(const std::vector<double> &f, const std::vector<double> &offs, double z, double w) {
    double zN = -INF, zP = -INF;
    for (size_t k = 0; k < offs.size(); k++) {
        const double o = offs[k];
        if (o <= -w) zN = jsMax(zN, f[k]);
        if (o >= w) zP = jsMax(zP, f[k]);
    }
    zN = zN < z - MAX_DROP - MAX_HANG ? z : jsMax(zN, z - MAX_DROP);
    zP = zP < z - MAX_DROP - MAX_HANG ? z : jsMax(zP, z - MAX_DROP);
    double lift = 0;
    for (size_t k = 0; k < offs.size(); k++) {
        const double o = offs[k];
        if (o <= -w || o >= w) continue;
        lift = jsMax(lift, f[k] - (zN + (zP - zN) * (o + w) / (2 * w)));
    }
    // Never ABOVE the level bottom (z, the highest floor)
    zN += lift; zP += lift;
    if (zN <= z && zP <= z) return {zN, zP};
    // So pin the high side at z and tilt only the low one, as far as the bulge lets it
    const bool pinP = zP > zN;
    double low = pinP ? zN - lift : zP - lift;
    for (size_t k = 0; k < offs.size(); k++) {
        const double o = offs[k];
        if (o <= -w || o >= w) continue;
        const double t = pinP ? (o + w) / (2 * w) : (w - o) / (2 * w);  // 1 at the pinned side
        low = jsMax(low, (f[k] - z * t) / (1 - t));
    }
    low = jsMin(low, z);
    return pinP ? std::array<double, 2>{low, z} : std::array<double, 2>{z, low};
}
} // namespace

FloorLine floorLine(const ContactLine &topLine, const SurfTris &tris, double margin, bool mold) {
    // Across the bottom's REAL width (see upstream): the welded tip, or for a bottom
    // lifted by footGap the full th plus footGap past each side.
    const double g = PROP.footGap, w = PROP.th / 2, t = PROP.tip / 2;
    const std::vector<double> offs = g > 0 ? std::vector<double>{-w - g, -w, -t, 0, t, w, w + g}
                                           : std::vector<double>{-t, 0, t};
    FloorLine bot;
    const int n = int(topLine.size());
    for (int i = 0; i < n; i++) {
        const Vec3d &a = topLine[size_t(std::max(0, i - 1))];
        const Vec3d &b = topLine[size_t(std::min(n - 1, i + 1))];
        const double rx = b[0] - a[0], ry = b[1] - a[1];
        double rn = hypot2(rx, ry);
        if (rn == 0 || std::isnan(rn)) rn = 1;    // JS `|| 1`
        const double sx = ry / rn, sy = -rx / rn;      // across the wall
        const double ceil = topLine[i][2] - margin;
        // molding: also footGap ahead and behind along the wall (the bottom's SHAPE only)
        const std::vector<double> along = mold && g > 0 ? std::vector<double>{-g, 0, g} : std::vector<double>{0};
        const double ax = rx / rn, ay = ry / rn;
        std::vector<double> f0, fm;                      // straight down; and molded (with along)
        for (const double o : offs) {
            double z0 = 0, zm = 0;                       // plate fallback
            for (const double d : along) {
                const double x = topLine[i][0] + sx * o + ax * d, y = topLine[i][1] + sy * o + ay * d;
                for (const double zz : surfaceZsAt(tris, x, y)) {
                    if (zz >= ceil) continue;            // highest surface below the overhang
                    if (zz > zm) zm = zz;
                    if (d == 0 && zz > z0) z0 = zz;
                }
            }
            f0.push_back(z0); fm.push_back(zm);
        }
        const double z = jsMaxOf(f0);
        if (!(g > 0)) { bot.emplace_back(topLine[i][0], topLine[i][1], z); continue; }
        const auto sides = sideFloors(fm, offs, jsMaxOf(fm), w);
        bot.emplace_back(topLine[i][0], topLine[i][1], z, sides[0], sides[1]);
    }
    return bot;
}

Mold moldLine(ContactLine top, const SurfTris &tris) {
    if (!(PROP.footGap > 0)) { FloorLine fl = floorLine(top, tris); return Mold{std::move(top), std::move(fl)}; }
    FloorLine floor = floorLine(top, tris, 1.0, true);
    auto jump = [](const FloorPt &a, const FloorPt &b) {
        double m = -INF;
        for (int k : {2, 3, 4}) m = jsMax(m, std::abs(a[k] - b[k]));
        return m;
    };
    for (int pass = 0; pass < 8; pass++) {
        ContactLine next;
        int added = 0;
        for (size_t i = 0; i < top.size(); i++) {
            next.push_back(top[i]);
            if (i + 1 == top.size()) break;
            const Vec3d &p = top[i], &q = top[i + 1];
            if (jump(floor[i], floor[i + 1]) <= MOLD_STEP) continue;
            if (hypot2(q[0] - p[0], q[1] - p[1]) < 2 * MOLD_MIN) continue;
            next.push_back(Vec3d((p[0] + q[0]) / 2, (p[1] + q[1]) / 2, (p[2] + q[2]) / 2));
            added++;
        }
        if (!added) break;
        top = std::move(next);
        floor = floorLine(top, tris, 1.0, true);
    }
    return Mold{std::move(top), std::move(floor)};
}

}} // namespace
