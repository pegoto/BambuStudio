// Port of web/prop/sweep.js. See Sweep.hpp.
#include "Sweep.hpp"
#include "../Cutout.hpp"
#include "../Solids.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

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
} // namespace

// The cap is the load-bearing part: a row of walls is maxUnsupportedSpan apart, so the
// foot must stay under half that spacing or adjacent feet merge into a buttress.
double footFor(double h) {
    const double spanCap = jsMax(PROP.footMin, (PROP.maxUnsupportedSpan - 1) / 2);
    return jsMax(PROP.footMin,
                 jsMin(jsMin(PROP.footMax, spanCap), h * PROP.footRatio));
}

double profileHalf(double z, double top) {
    const double ztip = jsMax(top - PROP.tipH, PROP.baseH + 0.1);
    if (z < PROP.baseH) return footFor(top);      // flat flange (the T's foot)
    if (z < ztip) return PROP.th / 2;             // straight wall (the T's stem)
    return PROP.th / 2                            // neck into the contact tip
         - ((PROP.th - PROP.tip) / 2) * ((z - ztip) / jsMax(1e-6, top - ztip));
}

bool sweep(const ContactLine &line, double zBed, Tris &out, double minH, IfaceTags *tags) {
    std::vector<Section> wall, flange;
    std::vector<CutStation> st;
    std::vector<std::optional<Section>> crest;
    const std::optional<CrestKinds> kinds = crestKinds(line);   // the crest's plan, nullopt when off
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
        if (h < minH) return false;
        const double foot = footFor(h);
        // A low TAIL station (see withLowTails) has less headroom than the flange +
        // tip taper assume, so both shrink with it. Identical to before for h >= 1.2.
        const double flangeH = jsMin(PROP.baseH, h / 2);
        const double ztip = jsMax(top - PROP.tipH, zBed + flangeH + 0.1);
        const double baseTop = zBed + flangeH;
        const double px = p[0], py = p[1];
        auto P = [px, py, sx, sy](double o, double z) { return Vec3d(px + sx * o, py + sy * o, z); };
        // where the interface crest runs, the stem stops at the crest's cut
        const std::optional<CrestCut> c = crestCut(top, ztip, zBed, kinds ? (*kinds)[i] : CrestKind::None);
        crest.push_back(c ? std::optional<Section>(c->ring(P)) : std::nullopt);
        const double zt = c ? c->zt : ztip, zTop = c ? c->cut : top, wTop = c ? c->wc : PROP.tip / 2;

        // the stem: a straight thin wall from the bed up to the breakaway tip
        wall.push_back({
            P(+PROP.th / 2, zBed), P(+PROP.th / 2, zt), P(+wTop, zTop),
            P(-wTop, zTop), P(-PROP.th / 2, zt), P(-PROP.th / 2, zBed),
        });
        // the foot: a flat slab, its own closed solid overlapping the wall's base
        flange.push_back({
            P(+foot, zBed), P(+foot, baseTop), P(-foot, baseTop), P(-foot, zBed),
        });
        CutStation s;
        s.p = p; s.sx = sx; s.sy = sy; s.top = zTop; s.ztip = zt; s.holeTop = ztip; s.tipHalf = wTop;
        s.bot = zBed; s.botTip = baseTop; s.taperBot = false;
        st.push_back(s);
    }
    if (wall.empty()) throw std::invalid_argument("sweep: empty line");   // JS: TypeError in ribbon

    const size_t from = out.size();
    if (!cutWall(st, wall, out, PROP)) ribbon(wall, out);
    ribbon(flange, out);
    if (kinds) { dropCollapsed(out, from, tags); emitCrest(crest, out, tags); }
    return true;
}

bool sweepBetween(const ContactLine &topLine, const FloorLine &botLine, Tris &out, double minH, IfaceTags *tags) {
    const bool welded = PROP.footGap <= 0;
    // only the PART gets the gap: a station whose floor is open plate all across
    // (floorLine's 0) stands on it. Decided per STATION, both sides together.
    std::vector<Section> wall;
    std::vector<CutStation> st;
    std::vector<std::optional<Section>> crest;
    const std::optional<CrestKinds> kinds = crestKinds(topLine);   // nullopt when off
    const size_t n = topLine.size();
    for (size_t i = 0; i < n; i++) {
        const Vec3d &p = topLine[i];
        const Vec3d &a = topLine[i == 0 ? 0 : i - 1];
        const Vec3d &b = topLine[std::min(n - 1, i + 1)];
        double rx = b[0] - a[0], ry = b[1] - a[1];
        const double rn = hypot2(rx, ry);
        if (rn < 1e-9) return false;
        rx /= rn; ry /= rn;
        const double sx = ry, sy = -rx;               // horizontal, across the wall

        const double top = p[2] - PROP.gap;
        const FloorPt &f = botLine.at(i);
        const double g = f[2] > 0 ? PROP.footGap : 0;
        const double bot = f[2] + g;
        // judged on the headroom, not the lifted wall: the gap must not change WHICH
        // walls exist
        if (top - f[2] < minH) return false;
        // a lifted bottom tilts with the floor under each side (floorLine's sideFloors);
        // welded, or with no side floors, it is level at `bot`
        auto side = [&](int k) { return welded || f.length() < 5 ? bot : jsMin(f[k] + g, top - 0.5); };
        const double bN = side(3), bP = side(4);
        const double bHi = jsMax(bN, bP);
        const double h = top - bHi;
        // tapers meet in the middle if short; lifted, the top taper stops 0.1 above the
        // bottom's high side (ending ON it collapsed a section edge into slivers)
        const double taper = jsMin(PROP.tipH, welded ? h / 2 : h - 0.1);
        const double zBotTip = welded ? bot + taper : bHi;
        const double px = p[0], py = p[1];
        auto P = [px, py, sx, sy](double o, double z) { return Vec3d(px + sx * o, py + sy * o, z); };
        // where the interface crest runs, the wall stops at the crest's cut
        const std::optional<CrestCut> c = crestCut(top, top - taper, zBotTip, kinds ? (*kinds)[i] : CrestKind::None);
        crest.push_back(c ? std::optional<Section>(c->ring(P)) : std::nullopt);
        const double zTopTip = c ? c->zt : top - taper, zTop = c ? c->cut : top, wTop = c ? c->wc : PROP.tip / 2;

        // one closed section: th-wide through the middle, tip-wide at the top and, when
        // welded, at the bottom too
        if (welded) {
            wall.push_back({
                P(+PROP.tip / 2, bot), P(+PROP.th / 2, zBotTip),
                P(+PROP.th / 2, zTopTip), P(+wTop, zTop),
                P(-wTop, zTop), P(-PROP.th / 2, zTopTip),
                P(-PROP.th / 2, zBotTip), P(-PROP.tip / 2, bot),
            });
        } else {
            wall.push_back({
                P(+PROP.th / 2, bP), P(+PROP.th / 2, zTopTip), P(+wTop, zTop),
                P(-wTop, zTop), P(-PROP.th / 2, zTopTip), P(-PROP.th / 2, bN),
            });
        }
        CutStation s;
        s.p = p; s.sx = sx; s.sy = sy; s.top = zTop; s.ztip = zTopTip; s.holeTop = top - taper; s.tipHalf = wTop;
        s.bot = bHi; s.botTip = zBotTip; s.taperBot = welded; s.botN = bN; s.botP = bP;
        st.push_back(s);
    }
    if (wall.empty()) throw std::invalid_argument("sweepBetween: empty line");   // JS: TypeError in ribbon

    const size_t from = out.size();
    if (!cutWall(st, wall, out, PROP)) ribbon(wall, out);
    if (kinds) { dropCollapsed(out, from, tags); emitCrest(crest, out, tags); }
    return true;
}

}} // namespace
