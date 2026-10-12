// Port of web/prop/ties.js. See Ties.hpp.
#include "Ties.hpp"
#include "../Solids.hpp"
#include <array>
#include <cmath>
#include <vector>

namespace Slic3r { namespace SupportFins {

TieConfig::TieConfig() : slope(std::tan((55 * M_PI) / 180)) {}
thread_local TieConfig TIE;

namespace {
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NAN;
    if (a == b) return std::signbit(a) ? b : a;
    return a > b ? a : b;
}

/// Does the part come within sideClear of the strut from (u0, z0) to (u1, z1) (bottom
/// edge; TIE.thick deep), or sit around it? Frame: u from `o` along `d`, v across.
bool strutHitsPart(const SurfTris &tris, const Vec2d &o, const Vec2d &d, double u0, double z0, double u1, double z1) {
    const double w = PROP.th / 2 + PROP.sideClear, c = PROP.sideClear;
    const double nu = jsMax(1, std::ceil(std::abs(u1 - u0) / TIE.step)), nv = std::ceil(2 * w / TIE.step);
    for (int i = 0; i <= nu; i++) {
        const double u = u0 + (u1 - u0) * i / nu, zb = z0 + (z1 - z0) * i / nu, zt = zb + TIE.thick;
        for (int k = 0; k <= nv; k++) {
            const double v = -w + 2 * w * k / nv;
            const double x = o[0] + d[0] * u + d[1] * v, y = o[1] + d[1] * u - d[0] * v;
            int above = 0;
            for (const double z : surfaceZsAt(tris, x, y)) {
                if (z >= zb - c && z <= zt + c) return true;
                if (z > zt) above++;
            }
            if (above % 2) return true;                 // inside the part
        }
    }
    return false;
}
} // namespace

int tieWalls(const Vec2d &a, const Vec2d &b, double zMax, const SurfTris &tris, double zBed, Tris &out) {
    const double dx = b[0] - a[0], dy = b[1] - a[1], span = hypot2(dx, dy);
    if (span < TIE.minSpan || span > TIE.maxSpan) return 0;
    const Vec2d d(dx / span, dy / span), o(a[0], a[1]);
    const double climb = span * TIE.slope;              // one strut's rise
    const double e = TIE.into;
    // the last strut's top, at its far end (TIE.into past the wall), stays under zMax
    const double n = std::floor((zMax - zBed - TIE.thick - e * TIE.slope) / climb);
    if (n < 1) return 0;
    std::vector<std::array<double, 4>> struts;
    for (int s = 0; s < n; s++) {
        // even struts A -> B, odd B -> A; each starts where the last one ended
        const double u0 = s % 2 ? span : 0, u1 = s % 2 ? 0 : span;
        const double z0 = zBed + s * climb, z1 = z0 + climb;
        // out to its ends: past each wall's centreline, where a slanted corner can show
        const double g = s % 2 ? -1 : 1, rise = e * TIE.slope;
        if (strutHitsPart(tris, o, d, u0 - g * e, jsMax(zBed, z0 - rise), u1 + g * e, z1 + rise)) return 0;
        struts.push_back({u0, z0, u1, z1});
    }
    const double h = PROP.th / 2;
    for (const auto &st : struts) {
        const double u0 = st[0], z0 = st[1], u1 = st[2], z1 = st[3];
        // TIE.into past each wall's centreline (overlapping solids), along the strut's slope
        const double s = js_sign(u1 - u0), e0 = u0 - s * e, e1 = u1 + s * e;
        const auto zAt = [&](double u) { return z0 + (z1 - z0) * (u - u0) / (u1 - u0); };
        const double lo = jsMax(zBed, zAt(e0));
        // (u, z) parallelogram, extruded across: frame (u, z, v) right-handed, v = u x z
        const Poly2 poly = s > 0
            ? Poly2{Vec2d(e0, lo), Vec2d(e1, zAt(e1)), Vec2d(e1, zAt(e1) + TIE.thick), Vec2d(e0, lo + TIE.thick)}
            : Poly2{Vec2d(e1, zAt(e1)), Vec2d(e0, lo), Vec2d(e0, lo + TIE.thick), Vec2d(e1, zAt(e1) + TIE.thick)};
        boxExtrude(poly, -h, h, [&](double p, double q, double r) { return Vec3d(o[0] + d[0] * p + d[1] * r, o[1] + d[1] * p - d[0] * r, q); }, out);
    }
    return int(struts.size());
}

}} // namespace
