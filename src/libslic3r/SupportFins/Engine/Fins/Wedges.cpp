// Port of web/fins/wedges.js -- see Wedges.hpp.
#include "Wedges.hpp"
#include "Seating.hpp"
#include "../Cutout.hpp"
#include "../Ieee754.hpp"
#include "../PropConfig.hpp"
#include "../Solids.hpp"
#include "../Prop/Crest.hpp"
#include "../Prop/Raster.hpp"
#include "../Prop/Surface.hpp"
#include "../Prop/Tines.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_set>

namespace Slic3r { namespace SupportFins {

thread_local PerpConfig PERP;

namespace {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

// JS Math.min / Math.max on two doubles: NaN propagates, -0 < +0.
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NaN;
    return (a < b || (a == b && std::signbit(a))) ? a : b;
}
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NaN;
    return (a > b || (a == b && !std::signbit(a))) ? a : b;
}
// Math.max(...pts.map((q) => q[2])): -Infinity for none.
inline double maxZ(const std::vector<Vec3d> &pts) {
    double m = -INF;
    for (const Vec3d &q : pts) m = jsMax(m, q[2]);
    return m;
}

/** Push a closed solid, flipping winding to outward if its signed volume is negative. */
void pushSolid(const std::vector<Vec3d> &local, Tris &out) {
    double V = 0;
    for (size_t i = 0; i + 2 < local.size(); i += 3) {
        const Vec3d &a = local[i], &b = local[i + 1], &c = local[i + 2];
        V += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2])
              + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
    }
    if (V < 0) {
        for (size_t i = 0; i + 2 < local.size(); i += 3) {
            out.push_back(local[i]); out.push_back(local[i + 2]); out.push_back(local[i + 1]);
        }
    } else {
        for (const Vec3d &v : local) out.push_back(v);
    }
}

/** Extrude a planar ring (a list of [x,y,z]) by +/- half along the horizontal uDir. */
void extrudeRing(const std::vector<Vec3d> &ring, const PatchXY &uDir, double half, Tris &out) {
    const size_t n = ring.size();
    std::vector<Vec3d> lo, hi;
    lo.reserve(n); hi.reserve(n);
    for (const Vec3d &p : ring) lo.emplace_back(p[0] - uDir.x * half, p[1] - uDir.y * half, p[2]);
    for (const Vec3d &p : ring) hi.emplace_back(p[0] + uDir.x * half, p[1] + uDir.y * half, p[2]);
    std::vector<Vec3d> local;
    for (size_t i = 0; i < n; i++) {
        const size_t j = (i + 1) % n;
        local.push_back(lo[i]); local.push_back(lo[j]); local.push_back(hi[j]);
        local.push_back(lo[i]); local.push_back(hi[j]); local.push_back(hi[i]);
    }
    for (size_t i = 1; i + 1 < n; i++) {          // convex fan caps (a monotone wedge is convex)
        local.push_back(hi[0]); local.push_back(hi[i]); local.push_back(hi[i + 1]);
        local.push_back(lo[0]); local.push_back(lo[i + 1]); local.push_back(lo[i]);
    }
    pushSolid(local, out);
}

/**
 * The wedge's blade, with the Cutouts pattern through it when one is picked: a
 * flat-topped section per contact station handed to cutWall, the same as a prop
 * wall's. When cutWall declines (pattern off, too small to be worth it) the blade is
 * the solid extrusion it always was.
 */
constexpr double CUT_TOP = 0.05;
void emitBlade(const std::vector<Vec3d> &top, const std::vector<Vec3d> &ring, const PatchXY &uDir, double half,
               Tris &out) {
    std::vector<CutStation> st;
    std::vector<Section> full;
    const int n = int(top.size());
    for (int i = 0; i < n; i++) {
        const Vec3d &a = top[std::max(0, i - 1)], &b = top[std::min(n - 1, i + 1)];
        double rx = b[0] - a[0], ry = b[1] - a[1];
        const double rn = hypot2(rx, ry);
        if (rn < 1e-9) { st.clear(); break; }   // a vertical step: no (s, z) plane to cut in
        rx /= rn; ry /= rn;
        const double sx = ry, sy = -rx;          // across the blade, as prop/sweep.js
        const Vec3d &p = top[i];
        const double z = p[2];
        auto P = [&](double o, double zz) { return Vec3d(p[0] + sx * o, p[1] + sy * o, zz); };
        full.push_back(Section{P(+half, 0), P(+half, z), P(-half, z), P(-half, 0)});
        // the blade's top is flat, not a tip: ztip a hair under it keeps the top band's
        // section free of repeated corners
        CutStation s;
        s.p = p; s.sx = sx; s.sy = sy; s.top = z; s.ztip = z - CUT_TOP; s.bot = 0; s.botTip = PERP.footH;
        s.taperBot = false;
        st.push_back(s);
    }
    if (!st.empty() && cutWall(st, full, out, CutWallDims{PERP.th, PERP.th, 3})) return;
    extrudeRing(ring, uDir, half, out);
}

/**
 * The interface crest along a blade's `top`, as sections across it (uDir, signed the
 * way emitBlade's sections face so the ribbon winds outward), or nullopt (JS null)
 * when a station repeats in XY (a vertical step: no run to sweep along).
 */
std::optional<std::vector<std::optional<Section>>> crestSections(const std::vector<Vec3d> &top,
                                                                 const PatchXY &uDir, double half) {
    const size_t n = top.size();
    for (size_t i = 1; i < n; i++) {
        if (hypot2(top[i][0] - top[i - 1][0], top[i][1] - top[i - 1][1]) < 1e-6) return std::nullopt;
    }
    const double rx = top[n - 1][0] - top[0][0], ry = top[n - 1][1] - top[0][1];
    const double sgn = uDir.x * ry - uDir.y * rx >= 0 ? 1 : -1;    // uDir . (ry, -rx), as sweep's sx, sy
    std::vector<std::optional<Section>> secs;
    secs.reserve(n);
    for (const Vec3d &q : top) {
        secs.emplace_back(flatCrestRing(
            [&](double o, double z) { return Vec3d(q[0] + uDir.x * sgn * o, q[1] + uDir.y * sgn * o, z); },
            q[2], half));
    }
    return secs;
}

/**
 * A flat foot flange under the wedge's bed footprint (a -> b at z = 0), reaching
 * footHalf past both ends of the run. With `partTris` the flange keeps only its longest
 * stretch along the run where nothing of the part hangs lower than footH + gap over
 * the flange's full width (plus the gap sideways).
 */
void emitFoot(const Vec3d &a, const Vec3d &b, const PatchXY &uDir, Tris &out, const SurfTris *partTris = nullptr) {
    const double sx = b[0] - a[0], sy = b[1] - a[1];
    const double L = hypot2(sx, sy);
    if (L < 1e-6) return;
    const double ux = sx / L, uy = sy / L;                 // along the run (bed footprint)
    const double hw = PERP.th / 2 + PERP.footHalf, hl = L / 2 + PERP.footHalf;
    const double cx = (a[0] + b[0]) / 2, cy = (a[1] + b[1]) / 2;
    auto P = [&](double s, double w, double z) { return Vec3d(cx + ux * s + uDir.x * w, cy + uy * s + uDir.y * w, z); };
    double s0 = -hl, s1 = hl;
    if (partTris) {
        const double step = 0.1, need = PERP.footH + PERP.gap, reach = hw + PERP.gap;
        const double nS = std::ceil((2 * hl) / step), nW = std::ceil((2 * reach) / step);
        auto clearAt = [&](double s) {
            for (double k = 0; k <= nW; k++) {
                const Vec3d xy = P(s, -reach + (2 * reach * k) / nW, 0);
                const std::optional<double> low = surfaceZAt(*partTris, xy[0], xy[1]);
                if (low && *low < need) return false;
            }
            return true;
        };
        std::optional<std::array<double, 2>> best;
        std::optional<double> start;
        for (double i = 0; i <= nS; i++) {
            const double s = -hl + (2 * hl * i) / nS;
            const bool ok = clearAt(s);
            if (ok && !start) start = s;
            if (start && (!ok || i == nS)) {
                const double end = ok ? s : s - (2 * hl) / nS;
                if (!best || end - *start > (*best)[1] - (*best)[0]) best = std::array<double, 2>{{*start, end}};
                start.reset();
            }
        }
        if (!best || (*best)[1] - (*best)[0] < PERP.th) return;   // nowhere to stand a flange
        // Pull back one more sample so the kept edge is clear, not the last clear sample.
        s0 = (*best)[0] > -hl ? (*best)[0] + PERP.gap : (*best)[0];
        s1 = (*best)[1] < hl ? (*best)[1] - PERP.gap : (*best)[1];
    }
    const double rect[4][2] = {{s0, -hw}, {s1, -hw}, {s1, hw}, {s0, hw}};
    Vec3d lo[4], hi[4];
    for (int i = 0; i < 4; i++) lo[i] = P(rect[i][0], rect[i][1], 0);
    for (int i = 0; i < 4; i++) hi[i] = P(rect[i][0], rect[i][1], PERP.footH);
    std::vector<Vec3d> local;
    for (int i = 0; i < 4; i++) {
        const int j = (i + 1) % 4;
        local.push_back(lo[i]); local.push_back(lo[j]); local.push_back(hi[j]);
        local.push_back(lo[i]); local.push_back(hi[j]); local.push_back(hi[i]);
    }
    for (int i = 1; i < 3; i++) {
        local.push_back(hi[0]); local.push_back(hi[i]); local.push_back(hi[i + 1]);
        local.push_back(lo[0]); local.push_back(lo[i + 1]); local.push_back(lo[i]);
    }
    pushSolid(local, out);
}

/**
 * Is a wedge at this u standable AND uninterrupted by a bore? Its first contiguous
 * contact run (from the bed up) is tall enough to matter, and the face does NOT resume
 * ABOVE a gap for a couple of samples (material / void / material is a bore).
 */
bool columnClear(const WallPatch &p, double u) {
    const double nT = jsMax(2, std::ceil((p.t1 - p.t0) / PERP.tStep));
    bool started = false, ended = false;
    double top = 0;
    int resume = 0;
    for (double i = 0; i <= nT; i++) {
        const double t = p.t0 + ((p.t1 - p.t0) * i) / nT;
        const std::optional<double> dev = patchProbe(p, u, t);
        if (!dev) { if (started) ended = true; continue; }
        if (ended) { resume++; continue; }                      // material above a gap
        started = true;
        const double z = zAt(p, *dev, t) - PERP.gap;
        if (z > top) top = z;
    }
    return top >= PERP.minH && resume < 2;
}

// propServesPatch over props[0..n) (get(i) -> const PropRecord &), so wedgeVeto's
// `propServesPatch(p, [q])` needs no copy of q.
template <class Get>
bool servesPatch(const WallPatch &p, size_t n, Get get) {
    if (n == 0) return false;
    double xLo = INF, xHi = -INF, yLo = INF, yHi = -INF;
    for (const double u : {p.u0, p.u1}) for (const double t : {p.t0, p.t1}) {
        const Vec3d q = patchPoint(p, 0, u, t);
        if (q[0] < xLo) xLo = q[0];
        if (q[0] > xHi) xHi = q[0];
        if (q[1] < yLo) yLo = q[1];
        if (q[1] > yHi) yHi = q[1];
    }
    const double m = 8;
    for (size_t i = 0; i < n; i++) {
        const PropRecord &q = get(i);
        for (const Vec3d &pt : q.line) {
            // A tall wall's low TAIL (prop/clearance.js withLowTails) is not a wall under
            // this face; squat props are low by design and still count.
            if (!q.squat && pt[2] < PROP.minHeight) continue;
            if (pt[0] >= xLo - m && pt[0] <= xHi + m && pt[1] >= yLo - m && pt[1] <= yHi + m) return true;
        }
    }
    return false;
}

} // namespace

GripPatches gripPatches(const Topology &topo, const AnalyzeResult &result, const Rot &rot) {
    GripPatches g;
    for (WallPatch &p : findWallPatches(topo, rot, result.offset))
        if (p.n.z < -0.05) g.patches.push_back(std::move(p));
    for (size_t i = 0; i < g.patches.size(); i++) {
        for (const int f : g.patches[i].faces) {
            if (g.faceMap.emplace(f, int(i)).second) g.faceOrder.push_back(f);
        }
    }
    return g;
}

std::vector<double> perpColumns(const WallPatch &p, double lo, double hi, double pitch) {
    const double span = hi - lo;
    if (span <= 0) return {};
    const double nS = jsMax(2, std::ceil(span / 1.0));
    std::vector<bool> clear;
    for (double i = 0; i <= nS; i++) clear.push_back(columnClear(p, lo + (span * i) / nS));

    // contiguous clear samples -> u-bands (the void is the gap between them)
    std::vector<std::array<double, 2>> bands;
    double s = -1;
    for (double i = 0; i <= nS; i++) {
        const bool c = clear[size_t(i)];
        if (c && s < 0) s = i;
        if (s >= 0 && (!c || i == nS)) {
            const double e = c ? i : i - 1;
            if (e >= s) bands.push_back({{lo + (span * s) / nS, lo + (span * e) / nS}});
            s = -1;
        }
    }
    if (bands.empty()) bands.push_back({{lo, hi}});   // nothing read as clear: fall back to one row

    std::vector<double> cols;
    for (const auto &band : bands) {
        const double a = band[0], b = band[1];
        const double w = b - a;
        const double m = jsMax(1, jsMin(PERP.maxRow, js_round(w / pitch)));
        for (double k = 0; k < m; k++) cols.push_back(m == 1 ? (a + b) / 2 : a + (w * k) / (m - 1));
    }
    // cols.slice(0, PERP.maxRow) (maxRow is a non-negative count)
    if (PERP.maxRow >= 0 && cols.size() > size_t(PERP.maxRow)) cols.resize(size_t(PERP.maxRow));
    return cols;
}

PerpFins buildPerpFins(const WallPatch &p, const Topology &topo, const Rot &rot, const Offset &offset,
                       const WedgeOpts &opts) {
    const PatchXY uDir{p.u.x, p.u.y};            // horizontal, across the face (unit)
    const double half = PERP.th / 2;
    const double lo = p.u0 + PERP.inset, hi = p.u1 - PERP.inset;
    PerpFins R;
    if (hi - lo <= 0) return R;
    Tris &out = R.triangles;
    IfaceTags &tags = R.tags;
    int tineTotal = 0, count = 0;
    std::unique_ptr<SurfTris> partTris;   // seated part, built on the first foot that needs it

    for (const double uc : perpColumns(p, lo, hi, opts.pitch ? *opts.pitch : PERP.pitch)) {
        // contact profile up the face at this u (stop at the first hole after starting)
        std::vector<Vec3d> contact;
        const double nT = jsMax(2, std::ceil((p.t1 - p.t0) / PERP.tStep));
        for (double i = 0; i <= nT; i++) {
            const double t = p.t0 + ((p.t1 - p.t0) * i) / nT;
            const std::optional<double> dev = patchProbe(p, uc, t);
            if (!dev) { if (!contact.empty()) break; else continue; }
            const Vec3d w = patchPoint(p, *dev, uc, t);
            if (w[2] > 0.3) contact.push_back(w);
        }
        if (contact.size() < 2) continue;
        std::vector<Vec3d> top;
        top.reserve(contact.size());
        for (const Vec3d &w : contact) top.emplace_back(w[0], w[1], w[2] - PERP.gap);
        if (maxZ(top) < PERP.minH) continue;

        // With the interface crest on the blade stops at the crest's cut and the crest
        // rides on it -- the whole blade or none of it. A vertical step in the contact
        // has no run to sweep the crest along: one body.
        const std::optional<CrestKinds> kinds = crestKinds(contact);
        bool every = bool(kinds);
        if (kinds) for (const CrestKind k : *kinds) if (k == CrestKind::None) { every = false; break; }
        const std::optional<std::vector<std::optional<Section>>> crest =
            every ? crestSections(top, uDir, half) : std::nullopt;
        std::vector<Vec3d> blade;
        if (crest) {
            blade.reserve(top.size());
            for (const Vec3d &q : top) blade.emplace_back(q[0], q[1], flatCut(q[2]));
        } else {
            blade = top;
        }
        // ring: up the bed edge, along the top, down the bed edge (closes along the bed)
        std::vector<Vec3d> ring;
        ring.reserve(blade.size() + 2);
        ring.emplace_back(blade[0][0], blade[0][1], 0);
        for (const Vec3d &q : blade) ring.push_back(q);
        ring.emplace_back(blade.back()[0], blade.back()[1], 0);
        const size_t before = out.size();
        emitBlade(blade, ring, uDir, half, out);
        if (crest) emitCrest(*crest, out, &tags);
        if (!partTris) partTris = std::make_unique<SurfTris>(seatedPartTris(topo, rot, offset));
        emitFoot(Vec3d(top[0][0], top[0][1], 0), Vec3d(top.back()[0], top.back()[1], 0), uDir, out, partTris.get());
        if (opts.tines)
            tineTotal += emitTines(contact, nullptr, topo, rot, offset, out, tineStepFor(opts.tineDensity),
                                   std::nullopt, std::nullopt, std::nullopt, nullptr, &tags);
        if (out.size() > before) {
            count++;
            // One wedge = ring + foot + its tines, all pushed contiguously. Record the
            // range so the UI can address/remove this individual wedge.
            const double hMax = maxZ(top);
            const double sp = hypot2(contact.back()[0] - contact[0][0], contact.back()[1] - contact[0][1]);
            PerpWedge wd;
            wd.triRange = {{before, out.size()}};
            wd.line = contact;
            wd.height = hMax;
            wd.span = sp;
            R.wedges.push_back(std::move(wd));
        }
    }
    if (tags.size() < out.size()) tags.resize(out.size(), 0);
    R.tines = tineTotal;
    R.count = count;
    return R;
}

bool propServesPatch(const WallPatch &p, const std::vector<PropRecord> &props) {
    return servesPatch(p, props.size(), [&](size_t i) -> const PropRecord & { return props[i]; });
}

RasterVeto wedgeVeto(const std::vector<WallPatch> &patches, const Topology &topo, const Rot &rot,
                     const Offset &offset, const WedgeOpts &wedgeOpts) {
    struct State {
        std::vector<WallPatch> patches;
        const Topology *topo;
        Rot rot;
        Offset offset;
        WedgeOpts wedgeOpts;
        const std::vector<PropRecord> *seen = nullptr;   // JS `seen`, compared by identity
        std::vector<size_t> open;                        // indices into patches (JS `open`)
    };
    auto S = std::make_shared<State>(State{patches, &topo, rot, offset, wedgeOpts, nullptr, {}});
    return [S](const PropRecord &q, const std::vector<PropRecord> &normalProps) {
        if (&normalProps != S->seen) {          // once per build: the patches left to wedges
            S->seen = &normalProps;
            S->open.clear();
            for (size_t i = 0; i < S->patches.size(); i++) {
                const WallPatch &p = S->patches[i];
                if (!propServesPatch(p, normalProps)
                    && buildPerpFins(p, *S->topo, S->rot, S->offset, S->wedgeOpts).count > 0)
                    S->open.push_back(i);
            }
        }
        for (const size_t i : S->open)
            if (servesPatch(S->patches[i], 1, [&](size_t) -> const PropRecord & { return q; })) return true;
        return false;
    };
}

BareRegions bareAfterWedges(const Topology &topo, const Rot &rot, const AnalyzeResult &result,
                            const std::vector<int> &servedRegions, const Tris &wedgeTris) {
    const Soup &pos = topo.pos;
    const Offset &o = result.offset;
    const std::unordered_set<int> served(servedRegions.begin(), servedRegions.end());
    const double span2 = PROP.maxUnsupportedSpan * PROP.maxUnsupportedSpan;
    BareRegions bare;
    for (size_t i = 0; i < result.regions.size(); i++) {
        if (served.count(int(i))) continue;
        bool held = false;
        for (const int f : result.regions[i].faces) {
            double cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 3; j++) {
                const size_t k = size_t(f) * 9 + size_t(j) * 3;
                const double x = pos[k], y = pos[k + 1], z = pos[k + 2];
                cx += (rot[0] * x + rot[3] * y + rot[6] * z + o.x) / 3;
                cy += (rot[1] * x + rot[4] * y + rot[7] * z + o.y) / 3;
                cz += (rot[2] * x + rot[5] * y + rot[8] * z + o.z) / 3;
            }
            for (const Vec3d &v : wedgeTris) {
                if (v[2] > cz - 3 && v[2] < cz + 0.5 && js_pow(v[0] - cx, 2) + js_pow(v[1] - cy, 2) <= span2) {
                    held = true;
                    break;
                }
            }
            if (held) break;
        }
        if (!held) bare.push_back(int(i));
    }
    return bare;
}

int unservedAfterWedges(const Topology &topo, const Rot &rot, const AnalyzeResult &result,
                        const std::vector<int> &servedRegions, const Tris &wedgeTris) {
    return int(bareAfterWedges(topo, rot, result, servedRegions, wedgeTris).size());
}

}} // namespace Slic3r::SupportFins
