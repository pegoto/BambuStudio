// Port of web/prop/squat.js. See Squat.hpp.
#include "Squat.hpp"
#include "../Solids.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

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
} // namespace

bool sweepSquat(const ContactLine &line, double zBed, Tris &out, IfaceTags *tags) {
    std::vector<Section> wall, brim;
    std::vector<std::optional<Section>> crest;
    const std::optional<CrestKinds> kinds = crestKinds(line);   // the crest's plan, nullopt when off
    const double brimTop = zBed + PROP.squatBrimH;
    const size_t n = line.size();
    for (size_t i = 0; i < n; i++) {
        const Vec3d &p = line[i];
        const Vec3d &a = line[i == 0 ? 0 : i - 1];
        const Vec3d &b = line[std::min(n - 1, i + 1)];
        double rx = b[0] - a[0], ry = b[1] - a[1];
        const double rn = hypot2(rx, ry);
        if (rn < 1e-9) return false;
        rx /= rn; ry /= rn;
        const double sx = ry, sy = -rx;               // horizontal, across the wall

        const double top = p[2] - PROP.gap;
        const double h = top - zBed;
        if (h < PROP.minHeightSquat) return false;
        // neck to the tip over whatever height is left above the brim
        const double ztip = jsMax(top - PROP.tipH, brimTop + 0.05);
        const double px = p[0], py = p[1];
        auto P = [px, py, sx, sy](double o, double z) { return Vec3d(px + sx * o, py + sy * o, z); };
        // where the interface crest runs, the stem stops at the crest's cut
        const std::optional<CrestCut> c = crestCut(top, ztip, brimTop, kinds ? (*kinds)[i] : CrestKind::None);
        crest.push_back(c ? std::optional<Section>(c->ring(P)) : std::nullopt);
        const double zt = c ? c->zt : ztip, zTop = c ? c->cut : top, wTop = c ? c->wc : PROP.tip / 2;

        // the stem: bed to breakaway tip, th-wide then necking to the contact tip
        wall.push_back({
            P(+PROP.th / 2, zBed), P(+PROP.th / 2, zt), P(+wTop, zTop),
            P(-wTop, zTop), P(-PROP.th / 2, zt), P(-PROP.th / 2, zBed),
        });
        // the brim: a thin flat slab overlapping the wall's base, for plate grip
        brim.push_back({
            P(+PROP.squatBrimW, zBed), P(+PROP.squatBrimW, brimTop),
            P(-PROP.squatBrimW, brimTop), P(-PROP.squatBrimW, zBed),
        });
    }
    if (wall.empty()) throw std::invalid_argument("sweepSquat: empty line");   // JS: TypeError in ribbon
    const size_t from = out.size();
    ribbon(wall, out);
    ribbon(brim, out);
    if (kinds) { dropCollapsed(out, from, tags); emitCrest(crest, out, tags); }
    return true;
}

// The stations with minHeightSquat <= height < minHeight (DISJOINT from the tall run the
// caller builds) are walked into maximal runs, each settled on a private copy, re-trimmed
// and swept via sweepSquat. Same weld guard as the plate path.
std::vector<SquatWall> buildSquatBed(const ContactLine &line, const SurfTris &regionTris, const Topology &topo,
                                     const Rot &rot, const Offset &offset, Tris &out,
                                     const Mask *claimed, IfaceTags *tags) {
    const double zBed = 0;
    std::vector<SquatWall> placed;
    auto heightOf = [&](const Vec3d &p) { return (p[2] - PROP.gap) - zBed; };
    Mask usable(line.size(), false);
    for (size_t k = 0; k < line.size(); k++) {
        const double h = heightOf(line[k]);
        usable[k] = h >= PROP.minHeightSquat && h < PROP.minHeight
                 && !(claimed && k < claimed->size() && (*claimed)[k])
                 && stationIsClear(line, k, topo, rot, offset);
    }

    size_t k = 0;
    while (k < usable.size()) {
        if (!usable[k]) { k++; continue; }
        size_t j = k;
        while (j < usable.size() && usable[j]) j++;
        ContactLine raw(line.begin() + long(k), line.begin() + long(j));   // deep copy
        k = j;
        if (long(raw.size()) < PROP.minStations) continue;
        const double spanRaw = hypot2(raw.back()[0] - raw[0][0], raw.back()[1] - raw[0][1]);
        if (spanRaw < PROP.minSpanSquat) continue;

        // Put the closest approach on spec, then re-trim: settling can lift a station
        // into the tall band or drop one below the squat floor.
        settleTop(raw, regionTris);
        Mask avail(raw.size(), false);
        for (size_t i = 0; i < raw.size(); i++) {
            const double h = heightOf(raw[i]);
            avail[i] = h >= PROP.minHeightSquat && h < PROP.minHeight
                    && stationCertified(raw, i, topo, rot, offset);
        }
        const std::optional<Run> run = longestRun(avail);
        if (!run || (*run)[1] - (*run)[0] < PROP.minStations) continue;
        const ContactLine settled(raw.begin() + (*run)[0], raw.begin() + (*run)[1]);
        const double span = hypot2(settled.back()[0] - settled[0][0], settled.back()[1] - settled[0][1]);
        if (span < PROP.minSpanSquat) continue;

        const size_t before = out.size();
        if (!sweepSquat(settled, zBed, out, tags)) {
            truncate(out, tags, before);
            continue;
        }

        // an approach from above is the breakaway interface, anything else a flank weld
        const std::optional<SolidHit> hit = solidClearance(topo, rot, offset, Tris(out.begin() + long(before), out.end()), 0.25);
        if (hit && welds(*hit)) {
            truncate(out, tags, before);
            continue;
        }

        double zMax = -INF;
        for (const Vec3d &p : settled) zMax = jsMax(zMax, p[2]);
        const double top = zMax - PROP.gap;
        double vol = 0;
        for (size_t i = before; i < out.size(); i += 3) {
            const Vec3d &a = out[i], &b = out[i + 1], &c = out[i + 2];
            vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2])
                  + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
        }
        SquatWall w;
        w.span = span;
        w.height = top - zBed;
        w.stations = int(settled.size());
        w.volume = std::abs(vol);
        w.squat = true;
        w.line.reserve(settled.size());
        for (const Vec3d &p : settled) w.line.push_back(Vec3d(p[0], p[1], p[2] - PROP.gap));
        // Triangle range of THIS wall in the caller's `out` (the caller's tines for it
        // land later, after every squat wall: a second, non-contiguous segment).
        w.triRange = {before, out.size()};
        placed.push_back(std::move(w));
    }
    return placed;
}

}} // namespace
