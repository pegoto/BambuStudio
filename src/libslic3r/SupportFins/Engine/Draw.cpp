// Port of web/draw.js and plugins/shared/engine/draw_entry.js. See Draw.hpp.
#include "Draw.hpp"
#include "Cutout.hpp"
#include "Entry.hpp"
#include "Fins.hpp"
#include "Materials.hpp"
#include "Prop/Attached.hpp"
#include "Prop/Brace.hpp"
#include "Prop/Mold.hpp"
#include "Prop/Squat.hpp"
#include "Prop/Sweep.hpp"
#include "Prop/Tines.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>

namespace Slic3r { namespace SupportFins {

namespace {
// JS Math.max: NaN if either is NaN; +0 beats -0.
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    if (a == b) return std::signbit(a) ? b : a;
    return a > b ? a : b;
}

// UTF-8 for the two non-ASCII characters in upstream's messages.
#define EMDASH "\xE2\x80\x94"
#define RSQUO "\xE2\x80\x99"

// x.toFixed(1): the n nearest x * 10 on the EXACT decimal value of x, ties to the larger
// magnitude; glibc's %.1f breaks exact ties to even, so round from the exact expansion.
std::string toFixed1(double x) {
    if (std::isnan(x)) return "NaN";
    if (std::isinf(x)) return x > 0 ? "Infinity" : "-Infinity";
    if (std::fabs(x) >= 1e21) {   // toFixed falls back to ToString(x); never a wall length
        char b[64];
        std::snprintf(b, sizeof(b), "%.17g", x);
        return b;
    }
    static thread_local char buf[1200];
    std::snprintf(buf, sizeof(buf), "%.1100f", std::fabs(x));   // exact: a double has <= 1074 fraction digits
    char *dot = std::strchr(buf, '.');
    std::string keep(buf, size_t(dot - buf) + 2);               // integer part, '.', 1 digit
    if (dot[2] >= '5') {                                         // round the magnitude up
        int i = int(keep.size()) - 1;
        for (; i >= 0; i--) {
            if (keep[size_t(i)] == '.') continue;
            if (keep[size_t(i)] == '9') { keep[size_t(i)] = '0'; continue; }
            keep[size_t(i)]++;
            break;
        }
        if (i < 0) keep.insert(keep.begin(), '1');
    }
    // JS: a negative x whose rounded value is 0 still prints "-0.0" (x < 0 keeps the sign)
    if (x < 0) keep.insert(keep.begin(), '-');
    return keep;
}

/// Every hit above (x, y) with its normal's z: prop/surface.js's grid (draw.js `surfaceHits`).
inline std::vector<SurfHit> surfaceHits(const SurfTris &tris, double x, double y) { return surfaceHitsAt(tris, x, y); }

/// Every surface height directly above (x, y), as a list (the caller picks the one
/// nearest the line the user drew).
std::vector<double> surfaceZsAtLocal(const SurfTris &tris, double x, double y) {
    std::vector<double> zs;
    for (const SurfHit &h : surfaceHits(tris, x, y)) zs.push_back(h[0]);
    return zs;
}

/// Is (x, y, z) inside the part? Odd number of surfaces below it in that column.
bool solidAt(const SurfTris &tris, double x, double y, double z) {
    // a column through a shared edge hits both triangles: one surface, counted once
    std::vector<double> zs;
    for (double zz : surfaceZsAtLocal(tris, x, y)) if (zz < z) zs.push_back(zz);
    std::stable_sort(zs.begin(), zs.end(), [](double p, double q) { return p - q < 0; });
    int n = 0;
    for (size_t i = 0; i < zs.size(); i++) if (i == 0 || zs[i] - zs[i - 1] > 1e-5) n++;
    return n % 2 == 1;
}

/// Each station sits under a DOWN-facing surface: air just below it and part within half
/// a mm above.
bool underAnOverhang(const ContactLine &line, const SurfTris &tris) {
    for (const Vec3d &p : line) {
        if (solidAt(tris, p[0], p[1], p[2] - 0.02)) return false;
        bool any = false;
        for (double d : {0.02, 0.1, 0.25, 0.5}) if (solidAt(tris, p[0], p[1], p[2] + d)) { any = true; break; }
        if (!any) return false;
    }
    return true;
}

/// Does an interior station lie ON the part -- the surface it touches facing up?
bool onThePart(const ContactLine &line, const SurfTris &tris) {
    for (size_t i = 0; i < line.size(); i++) {
        if (i == 0 || i == line.size() - 1) continue;
        const Vec3d &p = line[i];
        std::optional<SurfHit> best;
        for (const SurfHit &h : surfaceHits(tris, p[0], p[1])) {
            if (std::fabs(h[0] - p[2]) < 0.5 && (!best || std::fabs(h[0] - p[2]) < std::fabs((*best)[0] - p[2]))) best = h;
        }
        if (best && (*best)[1] > 0) return true;
    }
    return false;
}

/// Does part surface cross z0..z1(i) anywhere under the wall, w(i) either side of its
/// centre line? Sampled every BAND_STEP along and across.
constexpr double BAND_STEP = 0.25;
bool bandHitsPart(const ContactLine &line, const SurfTris &tris, const std::function<double(int)> &w, double z0,
                  const std::function<double(int)> &z1) {
    for (size_t i = 0; i + 1 < line.size(); i++) {
        const Vec3d &p = line[i], &q = line[i + 1];
        const double len = hypot2(q[0] - p[0], q[1] - p[1]);
        if (len < 1e-9) continue;
        const double rx = (q[0] - p[0]) / len, ry = (q[1] - p[1]) / len;
        const double wi = jsMax(w(int(i)), w(int(i) + 1)), top = jsMax(z1(int(i)), z1(int(i) + 1));
        const double na = std::ceil(len / BAND_STEP), nc = std::ceil(2 * wi / BAND_STEP);
        for (double s = 0; s <= na; s++) for (double k = 0; k <= nc; k++) {
            const double t = len * s / na, o = -wi + 2 * wi * k / nc;
            for (double z : surfaceZsAt(tris, p[0] + rx * t + ry * o, p[1] + ry * t - rx * o)) {
                if (z >= z0 && z <= top) return true;
            }
        }
    }
    return false;
}

/// Points across a wall at each station, `w` either side of its centre line, at height z(i).
bool acrossClear(const ContactLine &line, const SurfTris &tris, double w, const std::function<double(int)> &z) {
    for (size_t i = 0; i < line.size(); i++) {
        const Vec3d &a = line[i == 0 ? 0 : i - 1], &b = line[std::min(line.size() - 1, i + 1)];
        double rx = b[0] - a[0], ry = b[1] - a[1];
        const double rn = hypot2(rx, ry);
        if (rn < 1e-9) return false;
        rx /= rn; ry /= rn;
        for (double o : {-w, -w / 2, 0.0, w / 2, w}) {
            if (solidAt(tris, line[i][0] + ry * o, line[i][1] - rx * o, z(int(i)))) return false;
        }
    }
    return true;
}

/// squatWall's result (JS `{ tris, top, height, partAttached }`).
struct SquatResult {
    Tris        tris;
    IfaceTags   tags;
    ContactLine top;
    double      height = 0;
    bool        partAttached = false;
};

/// The squat fallback for a drawn line no full-height wall fits, or nullopt. Part first
/// (sweepBetween down to minHeightSquat of headroom), then the plate: stations from
/// minHeightSquat to minHeight get a brimmed squat stem, taller runs the flanged wall.
std::optional<SquatResult> squatWall(const std::optional<ContactLine> &topPA, const ContactLine &line,
                                     const SurfTris &tris, double zBed, bool plateOnly = false) {
    if (!plateOnly && topPA && int(topPA->size()) >= PROP.minStations && underAnOverhang(*topPA, tris)) {
        const ContactLine &tp = *topPA;
        const FloorLine floor = floorLine(tp, tris);
        // the wall's middle, between its floor and its top, is air
        const bool midClear = acrossClear(tp, tris, PROP.th / 2, [&](int i) {
            return (tp[size_t(i)][2] - PROP.gap + floor[size_t(i)][2] + PROP.footGap) / 2;
        });
        bool anyFloor = false;
        for (const FloorPt &p : floor) if (p[2] > PROP.gap + 0.5) { anyFloor = true; break; }
        if (midClear && anyFloor) {
            const Mold mold = moldLine(tp, tris);
            SquatResult t;
            if (sweepBetween(mold.top, mold.floor, t.tris, PROP.minHeightSquat, &t.tags)) {
                double height = 0;
                for (size_t i = 0; i < tp.size(); i++) height = jsMax(height, tp[i][2] - PROP.gap - floor[i][2]);
                t.top = tp;
                t.height = height;
                t.partAttached = true;
                return t;
            }
        }
    }
    std::vector<double> h;
    for (const Vec3d &p : line) h.push_back(p[2] - PROP.gap - zBed);
    for (double v : h) if (v < PROP.minHeightSquat) return std::nullopt;
    // a plate wall stands only where the way down is clear
    for (const FloorPt &p : floorLine(line, tris)) if (p[2] > PROP.gap + 0.5) return std::nullopt;
    // ...and only under a real overhang, with room for the whole brim and the stem
    if (!underAnOverhang(line, tris)) return std::nullopt;
    if (!acrossClear(line, tris, PROP.squatBrimW, [&](int) { return zBed + PROP.squatBrimH + PROP.gap; })) return std::nullopt;
    if (!acrossClear(line, tris, PROP.th / 2, [&](int i) { return zBed + (line[size_t(i)][2] - PROP.gap - zBed) / 2; }))
        return std::nullopt;
    // runs of tall (full wall) and low (squat) stations; a tall run under two stations
    // can't sweep, so it goes squat with its neighbours
    const int n = int(line.size());
    std::vector<bool> tall(static_cast<size_t>(n));
    for (int i = 0; i < n; i++) tall[size_t(i)] = h[size_t(i)] >= PROP.minHeight;
    // JS tall[-1] / tall[n] are undefined: falsy
    const auto tallAt = [&](int i) { return i >= 0 && i < n && tall[size_t(i)]; };
    for (int i = 0; i < n; i++) {
        if (tall[size_t(i)] && !tallAt(i - 1) && !tallAt(i + 1)) tall[size_t(i)] = false;
    }
    SquatResult t;
    int i = 0;
    while (i < n) {
        int j = i;
        while (j + 1 < n && tall[size_t(j + 1)] == tall[size_t(i)]) j++;
        if (tall[size_t(i)]) {
            const ContactLine seg(line.begin() + i, line.begin() + j + 1);
            if (!sweep(seg, zBed, t.tris, PROP.minHeight, &t.tags)) return std::nullopt;
        } else {
            // a squat run reaches TWO stations into each tall neighbour: the pieces must
            // overlap, never sit flush. sweepSquat takes any h >= 0.6.
            const ContactLine seg(line.begin() + std::max(0, i - 2), line.begin() + std::min(n, j + 3));
            if (seg.size() < 2 || !sweepSquat(seg, zBed, t.tris, &t.tags)) return std::nullopt;
        }
        i = j + 1;
    }
    double hm = -INF;
    for (double v : h) hm = jsMax(hm, v);
    t.top = line;
    t.height = hm;
    t.partAttached = false;
    return t;
}

void padTags(DrawnWall &r) {
    if (r.tags.size() < r.tris.size()) r.tags.resize(r.tris.size(), 0);
}

void pick(const char *name, const std::string &v, std::initializer_list<const char *> allowed) {
    for (const char *a : allowed) if (v == a) return;
    std::string list;
    for (const char *a : allowed) { if (!list.empty()) list += ", "; list += a; }
    throw std::invalid_argument(std::string(name) + " must be one of " + list + ", got \"" + v + "\"");
}
} // namespace

std::optional<ContactLine> drawnLine(const Vec3d &a, const Vec3d &b, const SurfTris &tris, std::optional<double> step,
                                     double band, bool under) {
    const double st = step ? *step : PROP.stationStep;
    const double dx = b[0] - a[0], dy = b[1] - a[1];
    const double len = hypot2(dx, dy);
    if (len < 1e-6) return std::nullopt;
    const double n = jsMax(PROP.minStations - 1, std::ceil(len / st));
    ContactLine line;
    for (double k = 0; k <= n; k++) {
        const double t = k / n;
        const double x = a[0] + dx * t, y = a[1] + dy * t;
        const double hint = a[2] + (b[2] - a[2]) * t;   // the height the drawn line implies here
        double z = hint, best = INF;
        // `under`: only surfaces facing DOWN; a hand-drawn line keeps the nearest of all.
        std::vector<double> zs;
        if (under) {
            for (const SurfHit &h : surfaceHitsAt(tris, x, y)) if (h[1] < 0) zs.push_back(h[0]);
        } else {
            zs = surfaceZsAtLocal(tris, x, y);
        }
        for (double zz : zs) {
            const double d = std::fabs(zz - hint);
            if (d < best) { best = d; z = zz; }
        }
        line.push_back(Vec3d(x, y, z));
    }
    // `band` bounds every settle pass to surfaces within `band` of the clicked overhang.
    contourTop(line, tris, band);
    lowerSag(line, tris, band);
    settleTop(line, tris, 0.25, band);
    return line;
}

DrawnWall drawnWall(const Vec3d &a, const Vec3d &b, const std::shared_ptr<const SurfTris> &trisP, double zBed,
                    const DrawnWallOpts &opts) {
    const SurfTris &tris = *trisP;
    const double dx = b[0] - a[0], dy = b[1] - a[1];
    const double len = hypot2(dx, dy);
    DrawnWall r;
    if (len < DRAW_MIN_LEN) {
        // `${DRAW_MIN_LEN}` prints the number 2 as "2"
        r.reason = "wall too short " EMDASH " " + toFixed1(len) + "mm, needs 2mm";
        return r;
    }
    Tris &out = r.tris;
    IfaceTags &tags = r.tags;
    // the interface crest's 'flat' test reads the underside here
    if (crestOn()) crestPart(trisP, opts.layerHeight ? *opts.layerHeight : 0);
    // A drawn wall grips with the tine comb when Tines is on and the caller passes topo.
    const auto withTines = [&](const ContactLine &line, std::optional<double> minTop) -> int {
        return opts.tines && opts.topo
            ? emitTines(line, &tris, *opts.topo, opts.rot, opts.offset, out, tineStepFor(opts.tineDensity), minTop,
                        opts.layerHeight ? *opts.layerHeight : PROP.tineH, std::nullopt, nullptr, &tags)
            : 0;
    };
    const auto withBraces = [&](const ContactLine &line, const std::function<double(int)> &floorZ) -> int {
        return opts.brace ? braceWall(line, floorZ, tris, zBed, out) : 0;
    };
    // PART-ATTACHED first: if solid part sits below the overhang, the support stands on
    // THAT, not the plate. Probe with a BANDED top contour.
    const std::optional<ContactLine> topPA = drawnLine(a, b, tris, PROP.stationStep, PART_BAND, opts.under);
    // ...unless the line lies ON the part (drawn on an upward face).
    if (topPA && int(topPA->size()) >= PROP.minStations && !onThePart(*topPA, tris)) {
        const ContactLine &tp = *topPA;
        const FloorLine floor = floorLine(tp, tris);
        double floorMax = 0;
        for (const FloorPt &p : floor) if (p[2] > floorMax) floorMax = p[2];
        // Plate only (#218): part under the overhang refuses the wall outright.
        if (opts.plateOnly && floorMax > PROP.gap + 0.5) {
            r.reason = "part of the model sits under this overhang, and "
                       "Plate only is on " EMDASH " a support can" RSQUO "t stand on the part here";
            r.tris.clear();
            r.tags.clear();
            return r;
        }
        std::optional<Mold> mold;
        if (floorMax > PROP.gap + 0.5) mold = moldLine(tp, tris);
        if (mold && sweepBetween(mold->top, mold->floor, out, PROP.minHeight, &tags)) {
            double height = 0;
            for (size_t i = 0; i < tp.size(); i++) height = jsMax(height, (tp[i][2] - PROP.gap) - floor[i][2]);
            // a station whose floor is open air runs down to the plate: it most needs a brace
            const int braces = withBraces(tp, [&](int i) { return floor[size_t(i)][2]; });
            const int tines = withTines(tp, std::nullopt);
            r.ok = true;
            r.length = len;
            r.height = height;
            r.partAttached = true;
            r.tines = tines;
            r.braces = braces;
            r.top = tp;
            std::vector<double> fl;
            for (const FloorPt &f : floor) fl.push_back(f[2]);
            r.floors = std::move(fl);
            padTags(r);
            return r;
        }
    }

    const std::optional<ContactLine> lineO = drawnLine(a, b, tris, PROP.stationStep, INF, opts.under);
    if (!lineO || int(lineO->size()) < PROP.minStations) {
        r.tris.clear(); r.tags.clear();
        r.reason = "no surface found along that line";
        return r;
    }
    const ContactLine &line = *lineO;
    // The flange and the stem must clear the part beside the line.
    const auto top = [&](int i) { return line[size_t(i)][2] - PROP.gap - zBed; };
    bool allTall = true;
    for (size_t i = 0; i < line.size(); i++) if (!(top(int(i)) >= PROP.minHeight)) { allTall = false; break; }
    if (allTall
        && (bandHitsPart(line, tris, [&](int i) { return footFor(top(i)); }, zBed - 0.01,
                         [&](int) { return zBed + PROP.baseH + PROP.gap; })
            || bandHitsPart(line, tris, [&](int) { return PROP.th / 2; }, zBed - 0.01,
                            [&](int i) { return zBed + top(i) / 2; }))) {
        r.tris.clear(); r.tags.clear();
        r.reason = "too close to the part " EMDASH " the wall" RSQUO "s foot would cut "
                   "into it; draw the line a little further out";
        return r;
    }
    // Reaching here means no real floor was found below the overhang: a plate wall.
    if (!sweep(line, zBed, out, PROP.minHeight, &tags)) {
        // Neither full-height wall fits: the two SQUAT walls a user drawing by hand can
        // still want (local issue 011), tried only now.
        const std::optional<SquatResult> squat = squatWall(topPA, line, tris, zBed, opts.plateOnly);
        if (squat) {
            if (tags.size() < out.size()) tags.resize(out.size(), 0);
            out.insert(out.end(), squat->tris.begin(), squat->tris.end());
            for (size_t i = 0; i < squat->tris.size(); i++) tags.push_back(i < squat->tags.size() ? squat->tags[i] : 0);
            const int tines = withTines(squat->top, squat->partAttached ? std::nullopt : std::optional<double>(PROP.squatBrimH));
            r.ok = true;
            r.length = len;
            r.height = squat->height;
            r.partAttached = squat->partAttached;
            r.squat = true;
            r.tines = tines;
            r.top = squat->top;
            padTags(r);
            return r;
        }
        r.tris.clear(); r.tags.clear();
        const double clickTop = std::min(a[2], b[2]) - zBed;
        if (clickTop >= PROP.minHeight + PROP.gap) {
            r.reason = "this overhang sits above another part of the "
                       "model, so a wall standing on the plate can" RSQUO "t reach it " EMDASH " rotate so it "
                       "faces the plate";
            return r;
        }
        r.reason = "nothing to hold up there " EMDASH " the line sits at the plate";
        return r;
    }
    double height = 0;
    for (const Vec3d &p : line) height = jsMax(height, p[2] - PROP.gap - zBed);
    const int braces = withBraces(line, [&](int) { return zBed; });
    const int tines = withTines(line, std::nullopt);
    r.ok = true;
    r.length = len;
    r.height = height;
    r.tines = tines;
    r.braces = braces;
    r.top = line;
    r.floors = std::vector<double>(line.size(), zBed);
    padTags(r);
    return r;
}

DrawWallResult drawWall(const Soup &positions, const Vec3d &a, const Vec3d &b, const DrawWallOptions &opts) {
    for (const auto &[name, p] : {std::pair<const char *, const Vec3d *>{"a", &a}, {"b", &b}}) {
        if (!std::isfinite((*p)[0]) || !std::isfinite((*p)[1]) || !std::isfinite((*p)[2]))
            throw std::invalid_argument(std::string(name) + " must be [x, y, z] of finite numbers");
    }
    // seat.js clearances(opts)
    const Material *mat = material_of(opts.material);
    if (!mat) throw std::invalid_argument("material must be one of pla, petg, got \"" + opts.material + "\"");
    pick("padStyle", opts.padStyle, {"off", "auto", "light", "sure"});
    pick("cutout", opts.cutout, {"none", "diamond", "triangle", "arch", "lattice"});
    Tunables tunables;
    tunables.padH = mat->padH; tunables.padGrab = mat->padGrab; tunables.propGap = mat->propGap;
    tunables.padStyle = opts.padStyle == "off" ? "auto" : opts.padStyle;
    tunables.cutout = opts.cutout;

    Soup pos;
    const Offset shift = seatSoup(positions, pos);
    Topology topo = buildTopology(pos);
    const AnalyzeResult result = analyze(topo, opts.threshold, IDENTITY3);
    const Offset off = result.offset;
    // The part in the print frame, as the site's partPrintTriangles builds it.
    auto tris = std::make_shared<SurfTris>(Soup(pos.size()));
    for (size_t i = 0; i < pos.size(); i += 3) {
        tris->pos[i] = pos[i] + off.x; tris->pos[i + 1] = pos[i + 1] + off.y; tris->pos[i + 2] = pos[i + 2] + off.z;
    }
    const auto seatP = [&](const Vec3d &p) {
        return Vec3d(snap_nm(p[0] - shift.x) + off.x, snap_nm(p[1] - shift.y) + off.y, snap_nm(p[2] - shift.z) + off.z);
    };
    // A drawn wall goes through no buildFins, so set the clearances here, every call.
    applyTunables(tunables);
    DrawnWallOpts dopts;
    dopts.tines = opts.tines;
    dopts.tineDensity = opts.tineDensity;
    dopts.layerHeight = opts.layerHeight;
    dopts.plateOnly = opts.plateOnly;
    dopts.topo = &topo;
    dopts.rot = IDENTITY3;
    dopts.offset = off;
    const DrawnWall r = drawnWall(seatP(a), seatP(b), tris, 0, dopts);
    DrawWallResult res;
    if (!r.ok) { res.reason = r.reason; return res; }
    res.ok = true;
    res.triangles.reserve(r.tris.size() * 3);
    for (const Vec3d &v : r.tris) {
        res.triangles.push_back(float(v.x()));
        res.triangles.push_back(float(v.y()));
        res.triangles.push_back(float(v.z()));
    }
    res.offset = {off.x - shift.x, off.y - shift.y, off.z - shift.z};
    res.stats.length = r.length;
    res.stats.height = r.height;
    res.stats.tines = r.tines;
    res.stats.partAttached = r.partAttached;
    return res;
}

}} // namespace
