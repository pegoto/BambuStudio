// Port of web/prop/brace.js. See Brace.hpp.
#include "Brace.hpp"
#include "../Solids.hpp"
#include <algorithm>
#include <cmath>
#include <optional>

namespace Slic3r { namespace SupportFins {

thread_local BraceConfig BRACE;

namespace {
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NAN;
    if (a == b) return std::signbit(a) ? b : a;
    return a > b ? a : b;
}
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NAN;
    if (a == b) return std::signbit(a) ? a : b;
    return a < b ? a : b;
}

/// Does part surface come within sideClear of the box u0..u1 x -w..w x z0..z1(u)
/// (rib frame: u out from the stem, v across it)?
bool ribHitsPart(const SurfTris &tris, const Vec2d &o, const Vec2d &d, double u0, double u1, double w, double z0,
                 const std::function<double(double)> &z1) {
    const double nu = std::ceil((u1 - u0) / BRACE.step), nv = std::ceil(2 * w / BRACE.step);
    for (int i = 0; i <= nu; i++) {
        const double u = u0 + (u1 - u0) * i / nu;
        for (int k = 0; k <= nv; k++) {
            const double v = -w + 2 * w * k / nv;
            const double x = o[0] + d[0] * u + d[1] * v, y = o[1] + d[1] * u - d[0] * v;
            for (const double z : surfaceZsAt(tris, x, y)) if (z >= z0 - 0.01 && z <= z1(u) + PROP.sideClear) return true;
        }
    }
    return false;
}

/// One rib from station point `o` (on the plate, at the stem's centre) in unit plan
/// direction `d`: `stem` is the rib's height there, `reach` how far out it lands.
/// Its triangles, or nullopt when the part is in the way.
std::optional<Tris> rib(const SurfTris &tris, const Vec2d &o, const Vec2d &d, double zBed, double stem, double reach) {
    const double u0 = -PROP.th / 2;                       // starts inside the stem: overlapping solids
    const double fw = PROP.footMin;                       // flange half-width
    const auto z1 = [&](double u) { return zBed + stem * jsMax(0, 1 - jsMax(0, u) / reach); };
    // out to sideClear past the tip, from the stem's centre (see upstream)
    const double lo = 0, hi = reach + PROP.sideClear;
    if (ribHitsPart(tris, o, d, lo, hi, PROP.th / 2 + PROP.sideClear, zBed, z1)) return std::nullopt;
    if (ribHitsPart(tris, o, d, lo, hi, fw + PROP.sideClear, zBed, [&](double) { return zBed + PROP.baseH; })) return std::nullopt;
    Tris out;
    // the rib: (u, z) triangle, extruded across. Frame (u, z, v) right-handed: v = u x z.
    boxExtrude({Vec2d(u0, zBed), Vec2d(reach, zBed), Vec2d(u0, zBed + stem)}, -PROP.th / 2, PROP.th / 2,
               [&](double a, double b, double c) { return Vec3d(o[0] + d[0] * a + d[1] * c, o[1] + d[1] * a - d[0] * c, b); }, out);
    // its flange: (u, v') rectangle, extruded up. Frame (u, v', z): v' = z x u.
    boxExtrude({Vec2d(u0, -fw), Vec2d(reach, -fw), Vec2d(reach, fw), Vec2d(u0, fw)}, zBed, zBed + PROP.baseH,
               [&](double a, double b, double c) { return Vec3d(o[0] + d[0] * a - d[1] * b, o[1] + d[1] * a + d[0] * b, c); }, out);
    return out;
}
} // namespace

int braceWall(const ContactLine &top, const std::function<double(int)> &floorZ, const SurfTris &tris,
              double zBed, Tris &out, const std::function<bool(const Tris &)> &clear) {
    const int n = int(top.size());
    if (n < 2) return 0;
    const Vec3d &a = top[0], &b = top[size_t(n - 1)];
    const double len = hypot2(b[0] - a[0], b[1] - a[1]);
    if (len < 1e-6) return 0;
    double height = 0;
    for (int i = 0; i < n; i++) height = jsMax(height, top[size_t(i)][2] - PROP.gap - floorZ(i));
    if (height <= BRACE.aspect * len) return 0;
    const double dx = (b[0] - a[0]) / len, dy = (b[1] - a[1]) / len;
    const double reach0 = PROP.footRatio * height;
    const int mid = n >> 1;
    const std::pair<int, Vec2d> ribs[4] = {{0, Vec2d(-dx, -dy)}, {n - 1, Vec2d(dx, dy)}, {mid, Vec2d(dy, -dx)}, {mid, Vec2d(-dy, dx)}};
    int built = 0;
    for (const auto &[i, d] : ribs) {
        if (floorZ(i) > zBed + BRACE.onPlate) continue;
        // under the tip taper: the contact and its breakaway stay exactly as they were
        const double stem = jsMin(BRACE.rise * (top[size_t(i)][2] - PROP.gap - zBed), top[size_t(i)][2] - PROP.gap - PROP.tipH - 1 - zBed);
        if (stem < PROP.baseH + 1) continue;
        for (double reach = reach0; reach >= BRACE.minReach; reach /= 2) {
            const auto r = rib(tris, Vec2d(top[size_t(i)][0], top[size_t(i)][1]), d, zBed, stem, reach);
            if (r && (!clear || clear(*r))) { out.insert(out.end(), r->begin(), r->end()); built++; break; }
        }
    }
    return built;
}

}} // namespace
