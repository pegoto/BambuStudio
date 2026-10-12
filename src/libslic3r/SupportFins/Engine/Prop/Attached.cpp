// Port of web/prop/attached.js. See Attached.hpp.
#include "Attached.hpp"
#include "../Ieee754.hpp"
#include "Clearance.hpp"
#include "Sweep.hpp"
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace Slic3r { namespace SupportFins {

namespace {
// JS Math.max: NaN if either is NaN; +0 beats -0.
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NAN;
    if (a == b) return std::signbit(a) ? b : a;
    return a > b ? a : b;
}
// JS `out.length = n`, with the parallel interface tags.
inline void truncate(Tris &out, IfaceTags *tags, size_t n) {
    if (out.size() > n) out.resize(n);
    if (tags && tags->size() > n) tags->resize(n);
}

double zMidAt(const ContactLine &top, const FloorLine &floor, size_t k) {
    return (floor[k][2] + (top[k][2] - PROP.gap)) / 2;
}
} // namespace

bool buriedColumn(const ContactLine &top, const FloorLine &floor, size_t k, const Topology &topo,
                  const Rot &rot, const Offset &offset) {
    return insidePart(topo, rot, offset, top[k][0], top[k][1], zMidAt(top, floor, k));
}

// This USED to refuse the wall; since the 2026-09-27 print tests a bore is supported
// and only labelled `inBore` (Suggest orientation weighs it as a mild preference).
bool enclosedColumn(const ContactLine &top, const FloorLine &floor, size_t k, const Topology &topo,
                    const Rot &rot, const Offset &offset) {
    const Vec3d &p = top[k];
    const double zMid = zMidAt(top, floor, k);
    int walled = 0;
    for (int d = 0; d < BORE.dirs; d++) {
        const double ang = (double(d) / BORE.dirs) * 2 * M_PI;
        const double dx = js_cos(ang), dy = js_sin(ang);
        for (double r = BORE.step; r <= BORE.radius; r += BORE.step) {
            if (insidePart(topo, rot, offset, p[0] + dx * r, p[1] + dy * r, zMid)) {
                walled++;
                break;
            }
        }
    }
    return walled >= BORE.walledMin;
}

// Mirror of stationIsClear, bounded to the (floor, top) span the wall occupies, probing
// STRICTLY between the ends (the bottom welds to the floor, the top breaks away).
bool clearBetween(const ContactLine &top, const FloorLine &floor, size_t k, const Topology &topo,
                  const Rot &rot, const Offset &offset) {
    const Vec3d &p = top[k];
    const Vec3d &a = top[k >= 1 ? k - 1 : 0];
    const Vec3d &b = top[std::min(top.size() - 1, k + 1)];
    const double rx = b[0] - a[0], ry = b[1] - a[1];
    const double rn = hypot2(rx, ry);
    if (rn < 1e-9) return true;
    const double sx = ry / rn, sy = -rx / rn;
    const double zTop = p[2] - PROP.gap, zBot = floor[k][2];
    const double nP = jsMax(3, std::ceil((zTop - zBot) / 1.5));
    for (int i = 1; i < nP; i++) {                 // strictly interior heights
        const double z = zBot + ((zTop - zBot) * i) / nP;
        for (const double m : {0.12, 0.24, PROP.sideClear}) {
            const double w = PROP.th / 2 + m;
            if (insidePart(topo, rot, offset, p[0] + sx * w, p[1] + sy * w, z)) return false;
            if (insidePart(topo, rot, offset, p[0] - sx * w, p[1] - sy * w, z)) return false;
        }
    }
    return true;
}

PartAttached buildPartAttached(const ContactLine &line, const SurfTris &partTris, const Topology &topo,
                               const Rot &rot, const Offset &offset, Tris &out, IfaceTags *tags) {
    ContactLine top = line;
    contourTop(top, partTris, PART_BAND);
    lowerSag(top, partTris, PART_BAND);
    settleTop(top, partTris, 0.25, PART_BAND);
    const FloorLine floor = floorLine(top, partTris);

    // A real floor under a majority of stations, or this is a bed overhang -- let the
    // plate path have it (floorLine returns ~0 with clear air to the plate).
    int real = 0;
    for (const FloorPt &f : floor) if (f[2] > PROP.gap + 0.5) real++;
    if (real < jsMax(PROP.minStations, std::ceil(double(floor.size()) * 0.5))) return {};

    // Why each station failed, so a refusal is counted under its real reason.
    std::vector<std::string> why;
    Mask ok(top.size());
    for (size_t k = 0; k < top.size(); k++) {
        const Vec3d &p = top[k];
        ok[k] = [&] {
            if (floor[k][2] <= PROP.gap + 0.5) return false;               // no real floor
            if ((p[2] - PROP.gap) - floor[k][2] < PROP.minHeight) { why.push_back("stub"); return false; }
            if (buriedColumn(top, floor, k, topo, rot, offset)) { why.push_back("buried"); return false; }
            if (clearBetween(top, floor, k, topo, rot, offset)) return true;
            why.push_back("blocked");
            return false;
        }();
    }
    const std::optional<Run> run = longestRun(ok);
    if (!run || (*run)[1] - (*run)[0] < PROP.minStations) {
        // Mostly-failing stations name the refusal; a clear run that is merely short is a stub.
        // JS: counts in an object (keys in first-seen order), stable sort by count desc.
        std::vector<std::pair<std::string, int>> n;
        for (const std::string &w : why) {
            auto it = std::find_if(n.begin(), n.end(), [&](const auto &e) { return e.first == w; });
            if (it == n.end()) n.emplace_back(w, 1);
            else it->second++;
        }
        std::stable_sort(n.begin(), n.end(), [](const auto &a, const auto &b) { return b.second - a.second < 0; });
        PartAttached r;
        r.floored = why.size() * 2 >= floor.size() && !n.empty() ? n.front().first : std::string("stub");
        return r;
    }

    const ContactLine subTop(top.begin() + (*run)[0], top.begin() + (*run)[1]);
    const FloorLine subFloor(floor.begin() + (*run)[0], floor.begin() + (*run)[1]);
    const double span = hypot2(subTop[subTop.size() - 1][0] - subTop[0][0],
                               subTop[subTop.size() - 1][1] - subTop[0][1]);
    if (span < PROP.minSpanPart) { PartAttached r; r.floored = "stub"; return r; }

    const size_t before = out.size();
    const Mold mold = moldLine(subTop, partTris);
    if (!sweepBetween(mold.top, mold.floor, out, PROP.minHeight, tags)) {
        truncate(out, tags, before);
        PartAttached r; r.floored = "degenerate"; return r;
    }

    // In a bore when most of the kept run is walled all round.
    int walled = 0;
    for (int k = (*run)[0]; k < (*run)[1]; k++) if (enclosedColumn(top, floor, size_t(k), topo, rot, offset)) walled++;
    const bool inBore = walled * 2 > (*run)[1] - (*run)[0];

    double height = 0, vol = 0;
    for (size_t i = 0; i < subTop.size(); i++) {
        height = jsMax(height, (subTop[i][2] - PROP.gap) - subFloor[i][2]);
    }
    for (size_t i = before; i < out.size(); i += 3) {
        const Vec3d &a = out[i], &b = out[i + 1], &c = out[i + 2];
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2])
              + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
    }
    PartAttached r;
    r.ok = true;
    PartAttachedProp &prop = r.prop.emplace();
    prop.span = span;
    prop.height = height;
    prop.stations = int(subTop.size());
    prop.volume = std::abs(vol);
    prop.partAttached = true;
    prop.inBore = inBore;
    for (const Vec3d &p : subTop) prop.line.push_back(Vec3d(p[0], p[1], p[2] - PROP.gap));
    return r;
}

}} // namespace
