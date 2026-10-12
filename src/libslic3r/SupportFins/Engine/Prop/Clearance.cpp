// Port of web/prop/clearance.js. See Clearance.hpp.
#include "Clearance.hpp"
#include "Sweep.hpp"
#include <algorithm>
#include <cmath>
#include <utility>

namespace Slic3r { namespace SupportFins {

namespace {
// JS Math.max / Math.min: NaN if either is NaN; +0 beats -0 for max, -0 for min.
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
// mask[k], JS-style: past the end reads undefined (false)
inline bool at(const Mask &m, long k) { return k >= 0 && size_t(k) < m.size() && m[size_t(k)]; }
} // namespace

// Probe the FULL height, OUTSIDE the wall's own faces (half-width plus a margin), at
// least every 1.5 mm of height (clearProbes is a floor), stepping the offsets through
// the clearance corridor because insidePart is a parity test: a rib thinner than the
// corridor can sit wholly between the wall face and a single far-edge probe.
bool stationIsClear(const ContactLine &line, size_t k, const Topology &topo, const Rot &rot, const Offset &offset) {
    const size_t n = line.size();
    const Vec3d &p = line[k];
    const Vec3d &a = line[k == 0 ? 0 : k - 1];
    const Vec3d &b = line[std::min(n - 1, k + 1)];
    const double rx = b[0] - a[0], ry = b[1] - a[1];
    const double rn = hypot2(rx, ry);
    if (rn < 1e-9) return true;
    const double sx = ry / rn, sy = -rx / rn;      // across the wall

    const double top = p[2] - PROP.gap;

    const double nProbes = jsMax(double(PROP.clearProbes), std::ceil(top / 1.5));
    for (double i = 1; i <= nProbes; i++) {
        const double z = (top * i) / nProbes - 0.05;
        if (z <= 0) continue;
        const double half = profileHalf(z, top);   // flange, wall, or tip taper at this z
        const double ms[3] = {0.12, 0.24, PROP.sideClear};
        for (const double m : ms) {
            const double w = half + m;
            if (insidePart(topo, rot, offset, p[0] + sx * w, p[1] + sy * w, z)) return false;
            if (insidePart(topo, rot, offset, p[0] - sx * w, p[1] - sy * w, z)) return false;
        }
        if (insidePart(topo, rot, offset, p[0], p[1], z)) return false;
    }
    return true;
}

// The cross-section outline: the top flat, then both flanks every 2 mm down the full
// height at the profile's own half-width. 2 mm is enough BECAUSE this is a distance
// measure with a 0.45 mm reach.
bool stationCertified(const ContactLine &line, size_t k, const Topology &topo, const Rot &rot, const Offset &offset) {
    const size_t n = line.size();
    const Vec3d &p = line[k];
    const Vec3d &a = line[k == 0 ? 0 : k - 1];
    const Vec3d &b = line[std::min(n - 1, k + 1)];
    const double rx = b[0] - a[0], ry = b[1] - a[1];
    const double rn = hypot2(rx, ry);
    if (rn < 1e-9) return true;
    const double sx = ry / rn, sy = -rx / rn;

    const double top = p[2] - PROP.gap;

    std::vector<std::pair<double, double>> probes = {{0, top}, {PROP.tip / 2, top}, {-PROP.tip / 2, top}};
    for (double z = 0.4; z < top - 0.05; z += 2.0) {
        const double half = profileHalf(z, top);
        probes.emplace_back(half, z);
        probes.emplace_back(-half, z);
    }
    for (const auto &[o, z] : probes) {
        if (z <= 0.05) continue;
        const std::optional<NearestHit> hit = nearestPart(topo, rot, offset, p[0] + sx * o, p[1] + sy * o, z);
        if (!hit) continue;
        if (welds(*hit)) return false;
    }
    return true;
}

// 0.675 x gap from above: the same 0.135 at PLA's 0.2 and the same ~47.5 deg at any gap.
bool welds(double d, double cosUp) {
    return cosUp > 0.7 ? d < PROP.gap * 0.675 : d < 0.205;
}

bool pathToPlateIsClear(const ContactLine &line, const Topology &topo, const Rot &rot, const Offset &offset) {
    for (size_t k = 0; k < line.size(); k++) {
        if (!stationIsClear(line, k, topo, rot, offset)) return false;
    }
    return true;
}

std::optional<Run> longestRun(const Mask &usable) {
    std::optional<Run> best;
    int start = -1;
    const int n = int(usable.size());
    for (int i = 0; i <= n; i++) {
        if (i < n && usable[size_t(i)]) {
            if (start < 0) start = i;
        } else if (start >= 0) {
            if (!best || i - start > (*best)[1] - (*best)[0]) best = Run{start, i};
            start = -1;
        }
    }
    return best;
}

Mask withLowTails(const Mask &tall, const Mask &low) {
    const std::optional<Run> run = longestRun(tall);
    Mask mask(tall.size(), false);
    if (!run) return mask;
    long a = (*run)[0], b = (*run)[1];
    while (a > 0 && at(low, a - 1)) a--;
    while (size_t(b) < low.size() && low[size_t(b)]) b++;
    // UPSTREAM: with `low` longer than `tall` the JS writes past mask's end (growing it);
    // every caller passes equal lengths. Reproduced: the mask grows to b.
    if (size_t(b) > mask.size()) mask.resize(size_t(b), false);
    for (long k = a; k < b; k++) mask[size_t(k)] = true;
    return mask;
}

Mask bodyMask(const ContactLine &pts) {
    Mask m;
    m.reserve(pts.size());
    for (const Vec3d &p : pts) m.push_back(p[2] - PROP.gap >= PROP.minHeight);
    return m;
}

std::optional<Run> tallBody(const Mask &mask) {
    int a = -1, b = -1;
    for (size_t i = 0; i < mask.size(); i++) if (mask[i]) { a = int(i); break; }
    for (size_t i = mask.size(); i-- > 0;) if (mask[i]) { b = int(i); break; }
    if (a < 0 || b <= a) return std::nullopt;
    return Run{a, b};
}

double tallSpan(const ContactLine &pts, const Mask &mask) {
    const std::optional<Run> r = tallBody(mask);
    return r ? hypot2(pts[size_t((*r)[1])][0] - pts[size_t((*r)[0])][0], pts[size_t((*r)[1])][1] - pts[size_t((*r)[0])][1]) : 0;
}

double minSpanFor(const ContactLine &pts, const Mask &mask, bool smallTube, bool shortWall) {
    if (smallTube) return PROP.minSpanTube;
    const std::optional<Run> r = shortWall ? tallBody(mask) : std::nullopt;
    if (!r) return PROP.minSpan;
    double h = 0;
    for (int k = (*r)[0]; k <= (*r)[1]; k++) h = jsMax(h, pts[size_t(k)][2] - PROP.gap);
    return jsMin(PROP.minSpan, jsMax(PROP.minSpanShort, h / PROP.maxShortAspect));
}

ContactLine &insertFloorStations(ContactLine &line) {
    const double floorZ = PROP.minHeightSquat + PROP.gap + 0.02;   // a hair over, so float noise can't drop it
    auto cross = [&](size_t i, size_t j) -> std::optional<Vec3d> {   // i below the floor, j above
        const double zi = line[i][2], zj = line[j][2];
        if (!(zi < floorZ && zj > floorZ + 1e-3)) return std::nullopt;
        const double t = (floorZ - zi) / (zj - zi);
        Vec3d r;
        for (int c = 0; c < 3; c++) { const double v = line[i][c]; r[c] = v + (line[j][c] - v) * t; }
        return r;
    };
    // walk in from each end past every sub-floor station to the first crossing
    const long len = long(line.size());
    long j = len - 1;
    while (j > 0 && line[size_t(j)][2] < floorZ) j--;
    const std::optional<Vec3d> hiEnd = j < len - 1 ? cross(size_t(j + 1), size_t(j)) : std::nullopt;
    if (hiEnd) line.insert(line.begin() + (j + 1), *hiEnd);
    long i = 0;
    while (i < long(line.size()) - 1 && line[size_t(i)][2] < floorZ) i++;
    const std::optional<Vec3d> loEnd = i > 0 ? cross(size_t(i - 1), size_t(i)) : std::nullopt;
    if (loEnd) line.insert(line.begin() + i, *loEnd);
    return line;
}

}} // namespace
