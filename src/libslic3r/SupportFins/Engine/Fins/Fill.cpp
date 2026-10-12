// Port of web/fins/fill.js -- see Fill.hpp.
#include "Fill.hpp"
#include "Seating.hpp"
#include "../Draw.hpp"
#include "../Ieee754.hpp"
#include "../PropConfig.hpp"
#include "../Prop/Attached.hpp"
#include "../Prop/Brace.hpp"
#include "../Prop/Sweep.hpp"
#include "../Prop/Ties.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>

namespace Slic3r { namespace SupportFins {

thread_local FillConfig FILL;

namespace {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
const double PI = 3.141592653589793;   // Math.PI

// JS Math.min / Math.max on two doubles: NaN propagates, -0 < +0.
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NaN;
    return (a < b || (a == b && std::signbit(a))) ? a : b;
}
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return NaN;
    return (a > b || (a == b && !std::signbit(a))) ? a : b;
}
inline long long gkey(long long i, long long j) { return i * 100003 + j; }

} // namespace

void FillStats::refuse(const std::string &why) {
    for (auto &kv : refused) if (kv.first == why) { kv.second++; return; }
    refused.emplace_back(why, 1);
}

double fillReach(double coverage) {
    const double r0 = PROP.maxUnsupportedSpan / 2;
    if (coverage <= 0.5) return r0;
    return r0 + (FILL.denseReach - r0) * jsMin(1, (coverage - 0.5) / 0.5);
}

std::vector<FillPt> leftoverPoints(const Topology &topo, const AnalyzeResult &result, const Rot &rot,
                                   const Reach &near, const std::vector<uint8_t> &cls) {
    const Soup &pos = topo.pos;
    const Offset &off = result.offset;
    const auto seatV = [&](size_t o) {
        return Vec3d(rot[0] * pos[o] + rot[3] * pos[o + 1] + rot[6] * pos[o + 2] + off.x,
                     rot[1] * pos[o] + rot[4] * pos[o + 1] + rot[7] * pos[o + 2] + off.y,
                     rot[2] * pos[o] + rot[5] * pos[o + 1] + rot[8] * pos[o + 2] + off.z);
    };
    const double s = FILL.sample;
    // JS Map keyed by the string `${i},${j},${k}`: insertion order kept
    std::map<std::tuple<double, double, double>, size_t> index;
    std::vector<FillPt> cells;
    const auto put = [&](double x, double y, double z, double a) {
        if (z < PLATE_Z || near(x, y, z)) return;
        // (+0 for -0: the JS string key prints both as "0")
        const auto k = std::make_tuple(std::floor(x / s) + 0.0, std::floor(y / s) + 0.0, std::floor(z / s) + 0.0);
        auto it = index.find(k);
        if (it != index.end()) {
            FillPt &c = cells[it->second];
            c[0] += x * a; c[1] += y * a; c[2] += z * a; c[3] += a;
        } else {
            index.emplace(k, cells.size());
            cells.push_back(FillPt{{x * a, y * a, z * a, a}});
        }
    };
    for (int f = 0; f < topo.nFaces; f++) {
        if (!result.over[size_t(f)] || cls[size_t(f)] != MUST) continue;
        const Vec3d A = seatV(size_t(f) * 9), B = seatV(size_t(f) * 9 + 3), C = seatV(size_t(f) * 9 + 6);
        const double edge = jsMax(jsMax(hypot3(B[0] - A[0], B[1] - A[1], B[2] - A[2]),
                                        hypot3(C[0] - B[0], C[1] - B[1], C[2] - B[2])),
                                  hypot3(A[0] - C[0], A[1] - C[1], A[2] - C[2]));
        const double n = jsMax(1, std::ceil(edge / s));
        const double a = topo.area[size_t(f)] / (n * n);
        // n x n sub-triangles: centroids on the barycentric grid
        for (double i = 0; i < n; i++) for (double j = 0; i + j < n; j++) {
            const auto one = [&](double u, double v) {
                const double bu = u / n, bv = v / n, bw = 1 - bu - bv;
                put(bw * A[0] + bu * B[0] + bv * C[0], bw * A[1] + bu * B[1] + bv * C[1],
                    bw * A[2] + bu * B[2] + bv * C[2], a);
            };
            one(i + 1.0 / 3, j + 1.0 / 3);
            if (i + j < n - 1) one(i + 2.0 / 3, j + 2.0 / 3);
        }
    }
    std::vector<FillPt> out;
    out.reserve(cells.size());
    for (const FillPt &c : cells) out.push_back(FillPt{{c[0] / c[3], c[1] / c[3], c[2] / c[3], c[3]}});
    return out;
}

std::vector<std::vector<int>> groupsOf(const std::vector<FillPt> &pts) {
    const double L = FILL.link;
    std::unordered_map<long long, std::vector<int>> grid;
    for (size_t i = 0; i < pts.size(); i++) {
        const FillPt &p = pts[i];
        grid[gkey((long long)std::floor(p[0] / L), (long long)std::floor(p[1] / L))].push_back(int(i));
    }
    std::vector<int> parent(pts.size());
    for (size_t i = 0; i < pts.size(); i++) parent[i] = int(i);
    const auto find = [&](int i) {
        while (parent[size_t(i)] != i) i = parent[size_t(i)] = parent[size_t(parent[size_t(i)])];
        return i;
    };
    for (size_t i = 0; i < pts.size(); i++) {
        const FillPt &p = pts[i];
        const long long gi = (long long)std::floor(p[0] / L), gj = (long long)std::floor(p[1] / L);
        for (long long di = -1; di <= 1; di++) for (long long dj = -1; dj <= 1; dj++) {
            auto it = grid.find(gkey(gi + di, gj + dj));
            if (it == grid.end()) continue;
            for (int j : it->second) {
                if (j <= int(i)) continue;
                const FillPt &q = pts[size_t(j)];
                const double d = hypot2(p[0] - q[0], p[1] - q[1]);
                if (d <= L && std::fabs(p[2] - q[2]) <= d + 1) parent[size_t(find(int(i)))] = find(j);
            }
        }
    }
    // JS Map by root, insertion order = first member's index order
    std::unordered_map<int, size_t> by;
    std::vector<std::vector<int>> groups;
    for (size_t i = 0; i < pts.size(); i++) {
        const int r = find(int(i));
        auto it = by.find(r);
        if (it == by.end()) { by.emplace(r, groups.size()); groups.push_back({int(i)}); }
        else groups[it->second].push_back(int(i));
    }
    return groups;
}

namespace {

using Across = std::vector<std::pair<double, int>>;   // [across, index]

/** One candidate: the line along (ux, uy) at across-offset `off`, over the red within
 *  R of it, split at plan gaps over `bridge` with the heaviest run kept. */
std::optional<FillLine> lineAt(const std::vector<FillPt> &pts, const Across &across, double ux, double uy, double off,
                               double R, double bridge, bool axis) {
    Across band;
    for (const auto &e : across)
        if (std::fabs(e.first - off) <= R) band.emplace_back(ux * pts[size_t(e.second)][0] + uy * pts[size_t(e.second)][1], e.second);
    std::stable_sort(band.begin(), band.end(), [](const auto &p, const auto &q) { return p.first - q.first < 0; });
    struct RunT { size_t from = 0, to = 0; double w = 0; };
    std::optional<RunT> run;
    RunT cur;
    for (size_t k = 0; k < band.size(); k++) {
        if (k > 0 && band[k].first - band[k - 1].first > bridge) cur = RunT{k, 0, 0};
        cur.w += pts[size_t(band[k].second)][3];
        cur.to = k;
        if (!run || cur.w > run->w) run = cur;
    }
    if (!run) return std::nullopt;
    double t0 = band[run->from].first, t1 = band[run->to].first;
    if (t1 - t0 < DRAW_MIN_LEN + 0.5) {
        const double m = (t0 + t1) / 2;
        t0 = m - (DRAW_MIN_LEN + 0.5) / 2;
        t1 = m + (DRAW_MIN_LEN + 0.5) / 2;
    }
    // z hint at each end: the red nearest that end
    const auto zAt = [&](double t) {
        double z = 0, bd = INF;
        for (size_t k = run->from; k <= run->to; k++) {
            const double dd = std::fabs(band[k].first - t);
            if (dd < bd) { bd = dd; z = pts[size_t(band[k].second)][2]; }
        }
        return z;
    };
    const auto at = [&](double t) { return Vec3d(ux * t - uy * off, uy * t + ux * off, zAt(t)); };
    FillLine c;
    c.a = at(t0);
    c.b = at(t1);
    for (size_t k = run->from; k <= run->to; k++) c.aims.push_back(band[k].second);
    c.weight = run->w * (axis ? 1 : FILL.axisBias);
    return c;
}

} // namespace

std::vector<FillLine> candidateLines(const std::vector<FillPt> &pts, const std::vector<int> &group, double R,
                                     double bridge, bool narrow) {
    struct Dir { FillLine c; double ux, uy, bestAt; bool axis; };
    std::vector<Dir> dirs;
    std::vector<FillLine> narrowOut;
    for (int d = 0; d < FILL.dirs; d++) {
        const double th = (PI * d) / FILL.dirs, ux = js_cos(th), uy = js_sin(th);
        const bool axis = d == 0 || 2 * d == FILL.dirs;
        // slide a 2R window over the across-coordinate, heaviest window wins
        Across across;
        for (int i : group) across.emplace_back(-uy * pts[size_t(i)][0] + ux * pts[size_t(i)][1], i);
        std::stable_sort(across.begin(), across.end(), [](const auto &p, const auto &q) { return p.first - q.first < 0; });
        double best = -1, bestAt = 0, w = 0;
        size_t lo = 0;
        for (size_t hi = 0; hi < across.size(); hi++) {
            w += pts[size_t(across[hi].second)][3];
            while (across[hi].first - across[lo].first > 2 * R) w -= pts[size_t(across[lo++].second)][3];
            if (w > best) { best = w; bestAt = (across[lo].first + across[hi].first) / 2; }
        }
        const std::optional<FillLine> c = lineAt(pts, across, ux, uy, bestAt, R, bridge, axis);
        if (c) dirs.push_back(Dir{*c, ux, uy, bestAt, axis});
        if (narrow) {
            const double N = FILL.narrow;
            double nb = -1, nAt = 0;
            w = 0; lo = 0;
            for (size_t hi = 0; hi < across.size(); hi++) {
                w += pts[size_t(across[hi].second)][3];
                while (across[hi].first - across[lo].first > 2 * N) w -= pts[size_t(across[lo++].second)][3];
                if (w > nb) { nb = w; nAt = (across[lo].first + across[hi].first) / 2; }
            }
            const std::optional<FillLine> v = lineAt(pts, across, ux, uy, nAt, N, bridge, axis);
            if (v) narrowOut.push_back(*v);
        }
    }
    // the best directions, each also SHIFTED across itself
    std::stable_sort(dirs.begin(), dirs.end(), [](const Dir &p, const Dir &q) { return q.c.weight - p.c.weight < 0; });
    std::vector<FillLine> out;
    for (size_t di = 0; di < dirs.size() && int(di) < FILL.bestDirs; di++) {
        const Dir &D = dirs[di];
        out.push_back(D.c);
        // the JS `line` closure captured this direction's sorted `across`: rebuild it
        Across across;
        for (int i : group) across.emplace_back(-D.uy * pts[size_t(i)][0] + D.ux * pts[size_t(i)][1], i);
        std::stable_sort(across.begin(), across.end(), [](const auto &p, const auto &q) { return p.first - q.first < 0; });
        for (double s : FILL.shifts) {
            const std::optional<FillLine> v = lineAt(pts, across, D.ux, D.uy, D.bestAt + s, R, bridge, D.axis);
            if (v) out.push_back(*v);
        }
    }
    const auto byWeight = [](const FillLine &p, const FillLine &q) { return q.weight - p.weight < 0; };
    std::stable_sort(out.begin(), out.end(), byWeight);
    std::stable_sort(narrowOut.begin(), narrowOut.end(), byWeight);
    out.insert(out.end(), narrowOut.begin(), narrowOut.end());
    return out;
}

std::vector<int> localPatch(const std::vector<FillPt> &pts, const std::vector<int> &g) {
    const double L = FILL.local;
    using K = std::tuple<double, double, double>;
    const auto key = [&](const FillPt &p) {
        return K(std::floor(p[0] / L) + 0.0, std::floor(p[1] / L) + 0.0, std::floor(p[2] / FILL.localZ) + 0.0);
    };
    std::map<K, double> cell;
    for (int i : g) { const K k = key(pts[size_t(i)]); auto it = cell.find(k); cell[k] = (it == cell.end() ? 0 : it->second) + pts[size_t(i)][3]; }
    int seed = g[0];
    double best = -1;
    for (int i : g) { const double w = cell[key(pts[size_t(i)])]; if (w > best) { best = w; seed = i; } }
    const FillPt s = pts[size_t(seed)];
    std::vector<int> out;
    for (int i : g)
        if (hypot2(pts[size_t(i)][0] - s[0], pts[size_t(i)][1] - s[1]) <= L && std::fabs(pts[size_t(i)][2] - s[2]) <= FILL.localZ)
            out.push_back(i);
    return out;
}

namespace {

/** Is there leftover red within FILL.redNear of p (plan and height)? Gridded. */
struct RedGrid {
    double C = 1.5;
    std::unordered_map<long long, std::vector<FillPt>> grid;
    explicit RedGrid(const std::vector<FillPt> &pts) : C(FILL.redNear) {
        for (const FillPt &p : pts) grid[gkey((long long)std::floor(p[0] / C), (long long)std::floor(p[1] / C))].push_back(p);
    }
    bool operator()(const Vec3d &p) const {
        const long long gi = (long long)std::floor(p[0] / C), gj = (long long)std::floor(p[1] / C);
        for (long long di = -1; di <= 1; di++) for (long long dj = -1; dj <= 1; dj++) {
            auto it = grid.find(gkey(gi + di, gj + dj));
            if (it == grid.end()) continue;
            for (const FillPt &q : it->second)
                if (hypot2(p[0] - q[0], p[1] - q[1]) <= C && std::fabs(p[2] - q[2]) <= C) return true;
        }
        return false;
    }
};

using Box6 = std::array<double, 6>;

/** [minX, minY, minZ, maxX, maxY, maxZ] of one triangle. */
Box6 triBox(const Vec3d &a, const Vec3d &b, const Vec3d &c) {
    return Box6{{jsMin(jsMin(a[0], b[0]), c[0]), jsMin(jsMin(a[1], b[1]), c[1]), jsMin(jsMin(a[2], b[2]), c[2]),
                 jsMax(jsMax(a[0], b[0]), c[0]), jsMax(jsMax(a[1], b[1]), c[1]), jsMax(jsMax(a[2], b[2]), c[2])}};
}

constexpr double BOX_EDGE = 2.0;    // mm: largest support-triangle box boxGrid works with (see boxesOf)
// mm: the plate flange's band, a hair over its top. JS: computed at module load from PROP.baseH.
const double FLANGE_TOP = PropConfig{}.baseH + 0.05;

/** fn(box) for pieces of triangle (a, b, c) no more than BOX_EDGE across: split at the
 *  longest edge's midpoint, at most 16 deep. */
template <class Fn>
void boxesOf(Vec3d a, Vec3d b, Vec3d c, Fn &fn, int depth = 0) {
    const Box6 bx = triBox(a, b, c);
    if (depth >= 16 || !(jsMax(jsMax(bx[3] - bx[0], bx[4] - bx[1]), bx[5] - bx[2]) > BOX_EDGE)) { fn(bx); return; }
    const auto d2 = [](const Vec3d &p, const Vec3d &q) {
        return js_pow(p[0] - q[0], 2) + js_pow(p[1] - q[1], 2) + js_pow(p[2] - q[2], 2);
    };
    const double ab = d2(a, b), bc = d2(b, c), ca = d2(c, a);
    // rotate so the longest edge is (a, b)
    if (bc >= ab && bc >= ca) { const Vec3d t = a; a = b; b = c; c = t; }
    else if (ca >= ab && ca >= bc) { const Vec3d t = c; c = b; b = a; a = t; }
    const Vec3d m((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2);
    boxesOf(a, m, c, fn, depth + 1);
    boxesOf(m, b, c, fn, depth + 1);
}

/// A box's owner: { fill, k } (fill wall k and its ribs), or Auto's (JS null).
struct BoxTag { bool fill = false; int k = -1; };
using Skip = std::function<bool(const BoxTag &)>;
const Skip isFill = [](const BoxTag &t) { return t.fill; };

/** Every support triangle's box, gridded in plan, so "does this new wall touch one" is cheap. */
class BoxGrid {
public:
    void add(const Tris &tris, BoxTag tag = BoxTag()) { add(tris, 0, tris.size(), tag); }
    void add(const Tris &tris, size_t from, size_t to, BoxTag tag = BoxTag()) {
        for (size_t i = from; i + 2 < to; i += 3) {
            auto fn = [&](const Box6 &bx) {
                cellsOf(bx, 0, [&](long long k) { grid_[k].push_back(Item{bx, tag}); return false; });
            };
            boxesOf(tris[i], tris[i + 1], tris[i + 2], fn);
        }
    }
    bool hitsBox(const Box6 &b, const Skip &skip = nullptr) const {
        bool hit = false;
        cellsOf(b, 0, [&](long long k) {
            auto it = grid_.find(k);
            if (it == grid_.end()) return false;
            for (const Item &it2 : it->second) {
                if (skip && skip(it2.tag)) continue;
                const Box6 &o = it2.b;
                if (b[0] <= o[3] && o[0] <= b[3] && b[1] <= o[4] && o[1] <= b[4] && b[2] <= o[5] && o[2] <= b[5]) { hit = true; return true; }
            }
            return false;
        });
        return hit;
    }
    // `shared`: two pieces that both start in the flange band, either one wholly within
    // it, are the second pass's joined base, not a hit
    bool hits(const Tris &tris, double pad, bool shared = false, const Skip &skip = nullptr) const {
        return hits(tris, 0, tris.size(), pad, shared, skip);
    }
    bool hits(const Tris &tris, size_t from, size_t to, double pad, bool shared = false, const Skip &skip = nullptr) const {
        bool hit = false;
        auto test = [&](const Box6 &b) {
            if (hit) return;
            cellsOf(b, pad, [&](long long k) {
                auto it = grid_.find(k);
                if (it == grid_.end()) return false;
                for (const Item &it2 : it->second) {
                    if (skip && skip(it2.tag)) continue;
                    const Box6 &o = it2.b;
                    if (b[0] - pad <= o[3] && o[0] <= b[3] + pad && b[1] - pad <= o[4] && o[1] <= b[4] + pad &&
                        b[2] - pad <= o[5] && o[2] <= b[5] + pad &&
                        !(shared && jsMax(b[2], o[2]) <= FLANGE_TOP && jsMin(b[5], o[5]) <= FLANGE_TOP)) { hit = true; return true; }
                }
                return false;
            });
        };
        // the new wall's triangles split the same way as the stored ones
        for (size_t i = from; i + 2 < to && !hit; i += 3) boxesOf(tris[i], tris[i + 1], tris[i + 2], test);
        return hit;
    }
private:
    static constexpr double C = 5;
    struct Item { Box6 b; BoxTag tag; };
    std::unordered_map<long long, std::vector<Item>> grid_;
    // fn(key) -> true stops (the JS callbacks return early once `hit`)
    template <class Fn>
    static void cellsOf(const Box6 &b, double pad, Fn fn) {
        const double i0 = std::floor((b[0] - pad) / C), i1 = std::floor((b[3] + pad) / C);
        const double j0 = std::floor((b[1] - pad) / C), j1 = std::floor((b[4] + pad) / C);
        for (double i = i0; i <= i1; i++)
            for (double j = j0; j <= j1; j++)
                if (fn(gkey((long long)i, (long long)j))) return;
    }
};

/// stationClear's `relax` (JS null = all false).
struct Relax { bool part = false, flange = false, join = false, plate = false; };

/** The lowest top a line's station may have (pass 1 full walls, pass 2 `low` squat). */
double lowest(bool low) { return (low ? PROP.minHeightSquat : PROP.minHeight) + PROP.gap; }

/** Does the wall at station p (its top) keep PROP.sideClear off every support already built? */
bool stationClear(const Vec3d &p, const SurfTris &tris, const BoxGrid &others, const Relax &relax) {
    const double w = PROP.th / 2 + PROP.sideClear;
    const Skip skip = relax.join ? isFill : nullptr;
    if (relax.plate) {
        for (const SurfHit &h : surfaceHitsAt(tris, p[0], p[1])) if (h[0] < p[2] - 1 && h[0] > PROP.gap + 0.5) return false;
    }
    if (relax.part) {
        // the nearest surface below, if it faces UP
        std::optional<SurfHit> below;
        for (const SurfHit &h : surfaceHitsAt(tris, p[0], p[1])) if (h[0] < p[2] - 1 && (!below || h[0] > (*below)[0])) below = h;
        if (below && (*below)[1] > 0 && (*below)[0] > PROP.gap + 0.5) {
            return !others.hitsBox(Box6{{p[0] - w, p[1] - w, (*below)[0], p[0] + w, p[1] + w, p[2]}}, skip);
        }
    }
    if (relax.flange) return !others.hitsBox(Box6{{p[0] - w, p[1] - w, FLANGE_TOP, p[0] + w, p[1] + w, p[2]}}, skip);
    const double wf = footFor(p[2] - PROP.gap) + PROP.sideClear;
    return !others.hitsBox(Box6{{p[0] - w, p[1] - w, PROP.baseH, p[0] + w, p[1] + w, p[2]}}, skip)
        && !others.hitsBox(Box6{{p[0] - wf, p[1] - wf, 0, p[0] + wf, p[1] + wf, PROP.baseH + PROP.gap}}, skip);
}

/// underStretch / settledStretch's pick (JS { a, b, red }).
struct Stretch { Vec3d a, b; int red = 0; };

/** The stretch of candidate line a-b a wall can follow, on the raw stations. */
std::optional<Stretch> underStretch(const Vec3d &ca, const Vec3d &cb, const SurfTris &tris, const RedGrid &redAt,
                                    const BoxGrid &others, const Relax &relax, bool low) {
    const double dx = cb[0] - ca[0], dy = cb[1] - ca[1], len = hypot2(dx, dy);
    const double n = jsMax(PROP.minStations - 1, std::ceil(len / PROP.stationStep));
    struct Top { double x, y, z, nz; };
    std::vector<std::optional<Top>> top;
    for (double k = 0; k <= n; k++) {
        const double t = k / n, x = ca[0] + dx * t, y = ca[1] + dy * t, hint = ca[2] + (cb[2] - ca[2]) * t;
        std::optional<SurfHit> hit;
        // the underside nearest the line's height here (drawnLine's `under` pick)
        for (const SurfHit &h : surfaceHitsAt(tris, x, y))
            if (h[1] < 0 && (!hit || std::fabs(h[0] - hint) < std::fabs((*hit)[0] - hint))) hit = h;
        if (hit) top.push_back(Top{x, y, (*hit)[0], (*hit)[1]}); else top.push_back(std::nullopt);
    }
    std::vector<bool> ok(top.size());
    for (size_t i = 0; i < top.size(); i++) {
        const auto &p = top[i];
        // too low for the wall asked for (`low`, settledStretch's)
        if (!p || p->nz >= 0 || p->z < lowest(low)) { ok[i] = false; continue; }
        ok[i] = stationClear(Vec3d(p->x, p->y, p->z), tris, others, relax);
    }
    std::optional<Stretch> pick;
    long from = -1;
    int red = 0;
    const auto close = [&](long to) {
        if (from < 0 || to < from) return;
        const Top &A = *top[size_t(from)], &B = *top[size_t(to)];
        const double l = hypot2(B.x - A.x, B.y - A.y);
        if (l >= DRAW_MIN_LEN && red > 0 && (!pick || red > pick->red))
            pick = Stretch{Vec3d(A.x, A.y, A.z), Vec3d(B.x, B.y, B.z), red};
    };
    for (size_t i = 0; i < top.size(); i++) {
        if (!ok[i]) { close(long(i) - 1); from = -1; continue; }
        if (from >= 0 && std::fabs(top[i]->z - top[i - 1]->z) > FILL.maxStep) { close(long(i) - 1); from = -1; }
        if (from < 0) { from = long(i); red = 0; }
        if (redAt(Vec3d(top[i]->x, top[i]->y, top[i]->z))) red++;
    }
    close(long(top.size()) - 1);
    return pick;
}

/** The same cut on drawnLine's SETTLED stations -- the top drawnWall actually builds to. */
std::optional<Stretch> settledStretch(const Vec3d &ca, const Vec3d &cb, const SurfTris &tris, const RedGrid &redAt,
                                      const BoxGrid &others, const Relax &relax, bool low) {
    const std::optional<ContactLine> topO = drawnLine(ca, cb, tris, PROP.stationStep, PART_BAND, true);
    if (!topO) return std::nullopt;
    const ContactLine &top = *topO;
    std::vector<bool> down(top.size());
    for (size_t i = 0; i < top.size(); i++) {
        const Vec3d &p = top[i];
        std::optional<SurfHit> best;
        for (const SurfHit &h : surfaceHitsAt(tris, p[0], p[1])) {
            const double d = std::fabs(h[0] - p[2]);
            if (d < 1 && (!best || d < std::fabs((*best)[0] - p[2]))) best = h;
        }
        if (!best || (*best)[1] >= 0 || p[2] < lowest(low)) { down[i] = false; continue; }
        down[i] = stationClear(p, tris, others, relax);
    }
    std::optional<Stretch> pick;
    size_t from = 0;
    int red = 0;
    const size_t L = top.size();
    for (size_t i = 0; i <= L; i++) {
        const bool breaks = i == L || !down[i] || (i > from && std::fabs(top[i][2] - top[i - 1][2]) > FILL.maxStep);
        if (breaks) {
            const double len = i > from ? hypot2(top[i - 1][0] - top[from][0], top[i - 1][1] - top[from][1]) : 0;
            if (len >= DRAW_MIN_LEN && red > 0 && (!pick || red > pick->red)) pick = Stretch{top[from], top[i - 1], red};
            // a jump starts the next stretch at this station; a face looking up skips it
            from = i < L && down[i] ? i : i + 1;
            red = from == i && i < L && redAt(top[i]) ? 1 : 0;
        } else if (redAt(top[i])) red++;
    }
    return pick;
}

/// Each kept fill wall's bottom z per station (drawnWall's floors), nullopt for a squat one.
using Floors = std::vector<std::optional<std::vector<double>>>;

/** BRACES, last: each wall that stands tall and short on the plate gets the ribs that fit. */
int braceFill(std::vector<BuiltProp> &props, const Floors &floors, Tris &triangles, IfaceTags &tags,
              const SurfTris &tris, const Tris &autoTris) {
    std::unique_ptr<BoxGrid> autoGrid;
    std::vector<std::unique_ptr<BoxGrid>> grids(props.size());
    const auto gridOf = [&](size_t k) -> BoxGrid & {
        if (!grids[k]) {
            grids[k] = std::make_unique<BoxGrid>();
            for (const TriRange &r : props[k].triRanges) grids[k]->add(triangles, r[0], r[1]);
        }
        return *grids[k];
    };
    int n = 0;
    for (size_t k = 0; k < props.size(); k++) {
        BuiltProp &q = props[k];
        if (!floors[k]) continue;
        Tris out;
        const auto clear = [&](const Tris &t) {
            if (!autoGrid) { autoGrid = std::make_unique<BoxGrid>(); autoGrid->add(autoTris); }
            if (autoGrid->hits(t, PROP.sideClear, true)) return false;
            for (size_t j = 0; j < props.size(); j++) if (j != k && gridOf(j).hits(t, PROP.sideClear, true)) return false;
            return true;
        };
        const std::vector<double> &fl = *floors[k];
        // (JS floors[k][i] past the end is undefined: NaN here)
        const int built = braceWall(q.line, [&](int i) { return size_t(i) < fl.size() ? fl[size_t(i)] : NaN; }, tris, 0, out, [&](const Tris &t) {
            if (!clear(t)) return false;
            gridOf(k).add(t);            // the next wall's ribs keep clear of these
            return true;
        });
        if (!built) continue;
        const size_t at = triangles.size();
        for (const Vec3d &t : out) { triangles.push_back(t); tags.push_back(0); }
        q.triRanges.push_back(TriRange{at, triangles.size()});
        q.braces = built;
        n += built;
    }
    return n;
}

/** TIES, after the ribs: each fill wall standing tall on the plate is tied to its
 *  nearest neighbours, closest pairs first, TIE.perWall each. */
int tieFill(std::vector<BuiltProp> &props, const Floors &floors, Tris &triangles, IfaceTags &tags,
            const SurfTris &tris, BoxGrid &others) {
    // per wall: its plate stations as [x, y, zMax, floor]
    using P4 = std::array<double, 4>;
    std::vector<std::vector<P4>> plate(props.size());
    for (size_t k = 0; k < props.size(); k++) {
        const BuiltProp &q = props[k];
        if (!floors[k] || q.height < TIE.minHeight) continue;
        for (size_t i = 0; i < q.line.size(); i++) {
            const Vec3d &p = q.line[i];
            const double fz = i < floors[k]->size() ? (*floors[k])[i] : NaN;   // JS undefined: filtered out
            const P4 v{{p[0], p[1], jsMin(TIE.rise * (p[2] - PROP.gap), p[2] - PROP.gap - PROP.tipH - 1), fz}};
            if (v[3] <= BRACE.onPlate) plate[k].push_back(v);
        }
    }
    struct Pair { double d; P4 a, b; size_t k, j; };
    std::vector<Pair> pairs;
    for (size_t k = 0; k < props.size(); k++) for (size_t j = k + 1; j < props.size(); j++) {
        std::optional<Pair> best;
        for (const P4 &a : plate[k]) for (const P4 &b : plate[j]) {
            const double d = hypot2(b[0] - a[0], b[1] - a[1]);
            if (d >= TIE.minSpan && d <= TIE.maxSpan && (!best || d < best->d)) best = Pair{d, a, b, k, j};
        }
        if (best) pairs.push_back(*best);
    }
    std::stable_sort(pairs.begin(), pairs.end(), [](const Pair &p, const Pair &q) { return p.d - q.d < 0; });
    std::vector<int> count(props.size(), 0);
    int n = 0;
    for (const Pair &pr : pairs) {
        const size_t k = pr.k, j = pr.j;
        if (count[k] >= TIE.perWall || count[j] >= TIE.perWall) continue;
        Tris out;
        if (!tieWalls(Vec2d(pr.a[0], pr.a[1]), Vec2d(pr.b[0], pr.b[1]), jsMin(pr.a[2], pr.b[2]), tris, 0, out)) continue;
        if (others.hits(out, PROP.sideClear, true, [&](const BoxTag &t) { return t.fill && (t.k == int(k) || t.k == int(j)); }))
            continue;
        const size_t at = triangles.size();
        for (const Vec3d &t : out) { triangles.push_back(t); tags.push_back(0); }
        props[k].triRanges.push_back(TriRange{at, triangles.size()});
        others.add(out, BoxTag{true, int(k)});
        props[k].ties = props[k].ties.value_or(0) + 1;
        count[k]++; count[j]++;
        n++;
    }
    return n;
}

/// wallFor / tryLines' result: true, CAP, the aims given up on, or (tryLines only) null.
struct WallRes {
    enum Kind { Null, True, Cap, Aims } kind = Null;
    std::vector<int> aims;
};

/// "a — b" / "a -- b" -> "a" (JS reason.split(' — ')[0].split(' -- ')[0])
std::string headOf(const std::string &reason) {
    std::string s = reason;
    const size_t e = s.find(" \xE2\x80\x94 ");
    if (e != std::string::npos) s = s.substr(0, e);
    const size_t d = s.find(" -- ");
    if (d != std::string::npos) s = s.substr(0, d);
    return s;
}

} // namespace

FillResult fillCoverage(const Topology &topo, const AnalyzeResult &result, const Rot &rot, const FillOpts &opts,
                        const FillAuto &built) {
    const double coverage = jsMax(0, jsMin(1, opts.coverage ? *opts.coverage : 0.5));
    const double R = fillReach(coverage);
    const bool oneEach = coverage < FILL.sparseBelow;
    std::vector<Vec3d> tops;
    if (built.fins) for (const FinRecord &w : *built.fins) for (const Vec3d &p : w.line) tops.push_back(p);
    Reach near = reachOf(tops, R);
    const std::vector<uint8_t> cls = classify(topo, result, rot);
    const std::vector<FillPt> pts = leftoverPoints(topo, result, rot, near, cls);
    FillResult res;
    Tris &triangles = res.triangles;
    IfaceTags &tags = res.tags;
    std::vector<BuiltProp> &props = res.props;
    FillStats &stats = res.stats;
    {
        double s = 0;
        for (const FillPt &p : pts) s = s + p[3];
        stats.startArea = s;
    }
    if (pts.empty()) return res;

    const auto trisP = std::make_shared<const SurfTris>(seatedPartTris(topo, rot, result.offset));
    const SurfTris &tris = *trisP;
    static const Tris noTris;
    const Tris &autoTris = built.triangles ? *built.triangles : noTris;
    BoxGrid others;
    others.add(autoTris);
    // Plate only (#218): no wall on the part
    const bool plateOnly = opts.plateOnly;
    Relax strict;
    strict.plate = plateOnly;
    DrawnWallOpts drawOpts;
    drawOpts.under = true;
    drawOpts.tines = opts.tines;
    drawOpts.tineDensity = opts.tineDensity;
    drawOpts.layerHeight = opts.layerHeight;
    drawOpts.plateOnly = plateOnly;
    drawOpts.topo = &topo;
    drawOpts.rot = rot;
    drawOpts.offset = result.offset;
    int tines = 0;
    Floors floors;                              // each kept wall's bottom z per station, for the braces
    std::vector<bool> dead(pts.size(), false);  // points no line could serve (by index into pts)
    const auto bare = [&](int p) { return !dead[size_t(p)] && !near(pts[size_t(p)][0], pts[size_t(p)][1], pts[size_t(p)][2]); };
    const auto full = [&]() { return stats.walls >= FILL.maxWalls || stats.tries >= FILL.maxTries || stats.checks >= FILL.maxChecks; };

    // One wall for group `g` of `live` (live: indices into pts; g: indices into live).
    const auto wallFor = [&](const std::vector<int> &liveIdx, const std::vector<int> &g, bool local) -> WallRes {
        std::vector<FillPt> live;
        live.reserve(liveIdx.size());
        for (int i : liveIdx) live.push_back(pts[size_t(i)]);
        std::vector<FillPt> gp;
        for (int i : g) gp.push_back(live[size_t(i)]);
        const RedGrid redAt(gp);
        std::optional<std::vector<int>> aims;
        int builds = 0;
        bool partOk = local && !plateOnly, flangeOk = local, joinOk = local;
        const auto tryLines = [&](const std::vector<FillLine> &cands) -> WallRes {
            for (const FillLine &c0 : cands) {
                if (full()) return WallRes{WallRes::Cap, {}};
                if (!aims) aims = c0.aims;
                stats.checks++;
                std::optional<Stretch> quick = underStretch(c0.a, c0.b, tris, redAt, others, strict, local);
                std::optional<Stretch> c;
                if (quick) c = settledStretch(quick->a, quick->b, tris, redAt, others, strict, local);
                // ...and relaxed: standing on the part where there is part under it, its
                // flange joining the flanges on the plate
                bool relaxed = false;
                if (!c && (partOk || flangeOk)) {
                    stats.checks++;
                    Relax relax; relax.part = partOk; relax.flange = flangeOk; relax.plate = plateOnly;
                    quick = underStretch(c0.a, c0.b, tris, redAt, others, relax, local);
                    c = quick ? settledStretch(quick->a, quick->b, tris, redAt, others, relax, local) : std::nullopt;
                    relaxed = !!c;
                }
                // ...and last, JOINED: crossing or meeting another fill wall
                bool joined = false;
                if (!c && joinOk) {
                    stats.checks++;
                    Relax relax; relax.part = partOk; relax.flange = true; relax.join = true; relax.plate = plateOnly;
                    quick = underStretch(c0.a, c0.b, tris, redAt, others, relax, local);
                    c = quick ? settledStretch(quick->a, quick->b, tris, redAt, others, relax, local) : std::nullopt;
                    joined = !!c;
                }
                if (!c) { stats.refuse("no underside to follow"); continue; }
                // a group whose lines keep passing the checks but won't build stops here
                if (builds++ >= FILL.groupTries) return WallRes{WallRes::Aims, *aims};
                stats.tries++;
                const DrawnWall r = drawnWall(c->a, c->b, trisP, 0, drawOpts);
                std::string why;
                if (!r.ok) why = headOf(r.reason);
                else if (others.hits(r.tris, PROP.sideClear, local, joined ? isFill : nullptr)) why = "touches a support";
                if (!why.empty()) {
                    stats.refuse(why);
                    // (the relaxation that let it through: on the part, on the plate, or joined)
                    if (joined && why == "touches a support") joinOk = false;
                    else if (relaxed && why == "touches a support") { if (r.partAttached) partOk = false; else flangeOk = false; }
                    continue;
                }
                const Reach reach = reachOf(r.top, R);
                double gain = 0;
                for (int i : g) if (reach(live[size_t(i)][0], live[size_t(i)][1], live[size_t(i)][2])) gain += live[size_t(i)][3];
                if (gain < FILL.minGain) { stats.refuse("holds no new red"); continue; }
                const size_t at = triangles.size();
                for (size_t i = 0; i < r.tris.size(); i++) { triangles.push_back(r.tris[i]); tags.push_back(i < r.tags.size() ? r.tags[i] : 0); }
                others.add(r.tris, BoxTag{true, int(props.size())});
                for (const Vec3d &p : r.top) near.add(p);
                BuiltProp q;
                q.line = r.top;
                q.triRanges = {TriRange{at, triangles.size()}};
                q.height = r.height;
                q.span = r.length;
                q.tines = r.tines;
                q.partAttached = r.partAttached;
                q.squat = r.squat;
                q.fill = true;
                q.joined = joined;
                props.push_back(std::move(q));
                if (joined) stats.joined = stats.joined.value_or(0) + 1;
                floors.push_back(r.floors);
                tines += r.tines;
                stats.walls++;
                return WallRes{WallRes::True, {}};
            }
            return WallRes{};
        };
        if (!local) {
            for (const double bridge : {FILL.bridge, 2 * FILL.sample}) {
                WallRes r = tryLines(candidateLines(live, g, R, bridge));
                if (r.kind != WallRes::Null) return r;
            }
            return WallRes{WallRes::Aims, aims ? *aims : g};
        }
        // LOCAL (the second pass): patch by patch, densest red first
        std::vector<int> rest = g;
        while (!rest.empty()) {
            const std::vector<int> sub = localPatch(live, rest);
            builds = 0;
            partOk = !plateOnly;
            flangeOk = joinOk = true;
            const WallRes r = tryLines(candidateLines(live, sub, R, FILL.bridge, true));
            if (r.kind == WallRes::True || r.kind == WallRes::Cap) return r;
            for (int i : sub) dead[size_t(liveIdx[size_t(i)])] = true;
            std::vector<bool> gone(live.size(), false);
            for (int i : sub) gone[size_t(i)] = true;
            std::vector<int> next;
            for (int i : rest) if (!gone[size_t(i)]) next.push_back(i);
            rest = std::move(next);
        }
        return WallRes{WallRes::Aims, aims ? *aims : g};
    };
    // groups of `live` (indices into pts), heaviest first; each group as indices into live
    const auto heaviest = [&](const std::vector<int> &liveIdx) {
        std::vector<FillPt> live;
        for (int i : liveIdx) live.push_back(pts[size_t(i)]);
        struct GW { std::vector<int> g; double w; };
        std::vector<GW> gs;
        for (std::vector<int> &g : groupsOf(live)) {
            double w = 0;
            for (int i : g) w = w + live[size_t(i)][3];
            gs.push_back(GW{std::move(g), w});
        }
        std::stable_sort(gs.begin(), gs.end(), [](const GW &p, const GW &q) { return q.w - p.w < 0; });
        std::vector<std::vector<int>> out;
        for (GW &x : gs) out.push_back(std::move(x.g));
        return out;
    };
    std::vector<int> all(pts.size());
    for (size_t i = 0; i < pts.size(); i++) all[i] = int(i);

    if (oneEach) {
        // sparse: each group of the red Auto left gets ONE wall, against what is still bare
        for (const std::vector<int> &g0 : heaviest(all)) {
            if (full()) break;
            for (;;) {
                std::vector<int> live;
                for (int i : g0) if (bare(i)) live.push_back(i);
                if (live.empty() || full()) break;
                std::vector<int> idx(live.size());
                for (size_t i = 0; i < live.size(); i++) idx[i] = int(i);
                const WallRes r = wallFor(live, idx, false);
                if (r.kind == WallRes::True || r.kind == WallRes::Cap) break;
                for (int i : r.aims) dead[size_t(live[size_t(i)])] = true;
            }
        }
    } else {
        // until every point is within reach: a wall under the heaviest bare group, regroup, again
        while (!full()) {
            std::vector<int> live;
            for (int i : all) if (bare(i)) live.push_back(i);
            if (live.empty()) break;
            const std::vector<int> g = heaviest(live)[0];
            const WallRes r = wallFor(live, g, false);
            if (r.kind == WallRes::Cap) break;
            if (r.kind != WallRes::True) for (int i : r.aims) dead[size_t(live[size_t(i)])] = true;
        }
        // ...then, with what the caps leave, the red still bare again, patch by patch
        if (!full()) std::fill(dead.begin(), dead.end(), false);
        while (!full()) {
            std::vector<int> live;
            for (int i : all) if (bare(i)) live.push_back(i);
            if (live.empty()) break;
            const WallRes r = wallFor(live, heaviest(live)[0], true);
            if (r.kind == WallRes::Cap) break;
            if (r.kind != WallRes::True) for (int i : r.aims) dead[size_t(live[size_t(i)])] = true;
        }
    }
    stats.ribs = braceFill(props, floors, triangles, tags, tris, autoTris);
    for (size_t k = 0; k < props.size(); k++)
        for (size_t r = 1; r < props[k].triRanges.size(); r++)
            others.add(triangles, props[k].triRanges[r][0], props[k].triRanges[r][1], BoxTag{true, int(k)});
    stats.ties = tieFill(props, floors, triangles, tags, tris, others);
    bool anyLive = false;
    double area = 0;
    for (size_t i = 0; i < pts.size(); i++) {
        const FillPt &p = pts[i];
        if (near(p[0], p[1], p[2])) continue;
        if (!dead[i]) anyLive = true;
        area = area + p[3];
        res.unserved.pts.push_back(Vec3d(p[0], p[1], p[2]));
    }
    stats.capped = full() && anyLive;
    res.unserved.area = area;
    res.tines = tines;
    return res;
}

}} // namespace
