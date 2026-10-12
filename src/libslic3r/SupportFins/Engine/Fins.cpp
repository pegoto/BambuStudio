// Port of web/fins.js -- the combined-support entry point: buildFins builds the tined
// breakaway walls (prop.js) under the overhangs, adds angled wedges under the broad
// downward faces no wall reached, the last-resort short walls, the Full-coverage fill,
// lays a bed pad under a part that barely touches the plate, and runs sway braces last.
// See Fins.hpp.
#include "Fins.hpp"
#include "Cutout.hpp"
#include "FinConfig.hpp"
#include "Pieces.hpp"
#include "Planes.hpp"
#include "Prop.hpp"
#include "PropConfig.hpp"
#include "Sway.hpp"
#include "Fins/Fill.hpp"
#include "Fins/Pad.hpp"
#include "Fins/Seating.hpp"
#include "Fins/Shortwalls.hpp"
#include "Fins/Wedges.hpp"
#include "Prop/Crest.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace Slic3r { namespace SupportFins {

namespace {
// JS Math.min / Math.max on two doubles: NaN propagates, -0 < +0.
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    return (a < b || (a == b && std::signbit(a))) ? a : b;
}
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    return (a > b || (a == b && !std::signbit(a))) ? a : b;
}
// Append `src` (with its tags, padded with 0) to `dst` / `dtags`.
void append(Tris &dst, IfaceTags &dtags, const Tris &src, const IfaceTags &stags) {
    if (dtags.size() < dst.size()) dtags.resize(dst.size(), 0);
    for (size_t i = 0; i < src.size(); i++) {
        dst.push_back(src[i]);
        dtags.push_back(i < stags.size() ? stags[i] : 0);
    }
}
} // namespace

double coverPitch(double coverage) {
    const double c = jsMax(0, jsMin(1, coverage));
    return c <= 0.5
        ? FIN.coverSparse + ((0.5 - c) / 0.5) * (FIN.coverExtraSparse - FIN.coverSparse)
        : FIN.coverSparse - ((c - 0.5) / 0.5) * (FIN.coverSparse - FIN.coverDense);
}

void applyTunables(const Tunables &t) {
    // set(obj, key, v): only a finite number
    if (std::isfinite(t.padH)) FIN.padH = t.padH;
    if (std::isfinite(t.padGrab)) PAD.grab = t.padGrab;
    if (t.padStyle == "auto" || t.padStyle == "light" || t.padStyle == "sure" || t.padStyle == "custom") PAD.style = t.padStyle;
    if (t.padCustom) {
        // for (const k of Object.keys(PAD.custom)): h, gap, grip, margin
        if (std::isfinite(t.padCustom->h)) PAD.custom.h = t.padCustom->h;
        if (std::isfinite(t.padCustom->gap)) PAD.custom.gap = t.padCustom->gap;
        if (std::isfinite(t.padCustom->grip)) PAD.custom.grip = t.padCustom->grip;
        if (std::isfinite(t.padCustom->margin)) PAD.custom.margin = t.padCustom->margin;
    }
    if (std::isfinite(t.propGap)) PROP.gap = t.propGap;
    // Tines per wall, exact (0 = off, the spacing rules decide). Calibration only.
    if (t.tinesPerWall && *t.tinesPerWall >= 0) PROP.tinesPerWall = *t.tinesPerWall;
    // One clearance, applied everywhere it is spelled (the wedge keeps its own copy).
    if (std::isfinite(t.propGap)) PERP.gap = t.propGap;
    if (isCutoutPattern(t.cutout)) CUT.pattern = t.cutout;
    // the interface crest's mode (GitHub #21, prop/crest.js)
    if (t.iface) PROP.iface = *t.iface;
}

namespace {

PropOpts propOptsOf(const BuildOpts &opts) {
    PropOpts p;
    p.coverage = opts.coverage;
    p.tines = opts.tines;                 // buildPass: `opts.tines === true`
    p.tineDensity = opts.tineDensity;
    p.layerHeight = opts.layerHeight;
    p.plateOnly = opts.plateOnly;
    p.raster = opts.raster;
    return p;
}

FinRecord finOf(const PropRecord &q) {
    FinRecord f;
    f.height = q.height;
    f.length = q.span;
    f.tines = 0;
    f.id = q.id;
    f.kind = q.kind.empty() ? "prop" : q.kind;
    f.triRanges = q.triRanges;
    f.line = q.line;
    f.span = q.span;
    return f;
}

Built buildFinsCore(Topology &topo, const AnalyzeResult &result, const Rot &rot, const BuildOpts &opts,
                    const RasterVeto &rasterVeto = nullptr);

/** Auto's result plus the fill pass's walls (fins/fill.js), merged the way the short
 *  walls are: triangles appended, props and fins carrying ranges into them. */
Built withFill(const Topology &topo, const AnalyzeResult &result, const Rot &rot, const BuildOpts &opts, Built aut) {
    FillOpts fo;
    fo.coverage = opts.coverage;
    fo.tines = opts.tines;
    fo.tineDensity = opts.tineDensity;
    fo.layerHeight = opts.layerHeight;
    fo.plateOnly = opts.plateOnly;
    const FillResult f = fillCoverage(topo, result, rot, fo, FillAuto{&aut.fins, &aut.triangles});
    const size_t at = aut.triangles.size();
    int wid = int(aut.fins.size());
    std::vector<FinRecord> fins;
    for (const BuiltProp &q0 : f.props) {
        BuiltProp q = q0;
        for (TriRange &r : q.triRanges) r = TriRange{r[0] + at, r[1] + at};
        FinRecord w;
        w.height = q.height;
        w.length = q.span;
        w.tines = q.tines;
        w.id = wid++;
        w.kind = "prop";
        w.fill = true;
        w.triRanges = q.triRanges;
        w.line = q.line;
        w.span = q.span;
        w.joined = q.joined;
        fins.push_back(std::move(w));
        aut.props.push_back(std::move(q));
    }
    const bool withTines = opts.tines;
    append(aut.triangles, aut.tags, f.triangles, f.tags);
    aut.tines = aut.tines + f.tines;
    aut.braceCount = aut.braceCount + (withTines ? int(fins.size()) : 0);
    aut.propCount = aut.propCount + (withTines ? 0 : int(fins.size()));
    for (FinRecord &w : fins) aut.fins.push_back(std::move(w));
    FillRecord fr;
    fr.walls = f.stats.walls;
    fr.joined = f.stats.joined.value_or(0);
    fr.ribs = f.stats.ribs;
    fr.ties = f.stats.ties;
    fr.tries = f.stats.tries;
    fr.checks = f.stats.checks;
    fr.capped = f.stats.capped;
    fr.refused = f.stats.refused;
    fr.bareBefore = f.stats.startArea;
    fr.unservedArea = f.unserved.area;
    fr.unservedPts = f.unserved.pts;
    aut.fill = std::move(fr);
    return aut;
}

Built buildFinsCore(Topology &topo, const AnalyzeResult &result, const Rot &rot, const BuildOpts &opts,
                    const RasterVeto &rasterVeto) {
    const std::string &mode = opts.mode;

    if (mode == "auto" || mode == "stabilize" || mode == "full") {
        const bool withTines = opts.tines;
        // Wide-face coverage (0 sparse .. 1 dense): the prop rows and the wedge row pitch.
        const double coverage = jsMax(0, jsMin(1, opts.coverage));
        const double covPitch = coverPitch(coverage);
        // The wedge candidates: broad down-facing patches, found before the props so a
        // raster wall can be kept off the ones the normal pass leaves to a wedge.
        std::vector<WallPatch> wedgeable;
        for (WallPatch &p : findWallPatches(topo, rot, result.offset))
            if (p.n.z < -0.05 && p.area >= PERP.minArea && (p.u1 - p.u0) >= PERP.minWidth) wedgeable.push_back(std::move(p));
        WedgeOpts wedgeOpts;
        wedgeOpts.tines = withTines;
        wedgeOpts.pitch = covPitch;
        wedgeOpts.tineDensity = opts.tineDensity;
        BuildOpts po = opts;
        po.mode = "prop";
        po.tines = withTines;
        const Built base = buildFinsCore(topo, result, rot, po, wedgeVeto(wedgeable, topo, rot, result.offset, wedgeOpts));

        // ANGLED WEDGES on grippable down-facing patches that NO prop wall reached.
        std::vector<PropRecord> baseProps(base.props.begin(), base.props.end());
        Tris wedgeTris;
        IfaceTags wedgeTags;
        struct WedgeRec { TriRange triRange; std::vector<Vec3d> line; double height, span; };
        std::vector<WedgeRec> wedgeRecs;   // per-wedge records, triRange into wedgeTris (pre-offset)
        int wedgeTines = 0;
        int wedgedPatches = 0;
        for (const WallPatch &p : wedgeable) {                   // downward, broad faces only
            if (propServesPatch(p, baseProps)) continue;          // a prop already stands under it
            const PerpFins w = buildPerpFins(p, topo, rot, result.offset, wedgeOpts);
            if (!w.count) continue;
            const size_t off = wedgeTris.size();
            append(wedgeTris, wedgeTags, w.triangles, w.tags);
            for (const PerpWedge &wd : w.wedges)
                wedgeRecs.push_back(WedgeRec{TriRange{wd.triRange[0] + off, wd.triRange[1] + off}, wd.line, wd.height, wd.span});
            wedgeTines += w.tines;
            wedgedPatches++;
        }
        const int wedgeCount = int(wedgeRecs.size());

        // LAST RESORT: short walls under the regions no wall or wedge reached. Not on a
        // point-seated part with the pad off (the prop-mode gate below).
        const bool noShort = !opts.lastResort || (base.seating.kind == "point" && !base.pad);
        ShortWalls shortW;
        if (!noShort) {
            PropOpts so = propOptsOf(opts);
            so.tines = withTines;
            so.coverage = coverage;
            Tris existing = base.triangles;
            existing.insert(existing.end(), wedgeTris.begin(), wedgeTris.end());
            shortW = lastResortWalls(topo, result, rot, so, base.servedRegions, wedgeTris, existing);
        }

        // Unified per-fin array: each entry carries its triangle segment(s) into the
        // final triangles. base.fins[i] is order-aligned with base.props[i].
        const size_t baseLen = base.triangles.size();
        std::vector<FinRecord> fins;
        for (size_t i = 0; i < base.props.size(); i++) {
            const BuiltProp &q = base.props[i];
            FinRecord f = i < base.fins.size() ? base.fins[i] : FinRecord();
            f.id = q.id;
            f.kind = "prop";
            f.triRanges = q.triRanges;
            f.line = q.line;
            f.span = q.span;
            f.height = q.height;
            fins.push_back(std::move(f));
        }
        int wid = int(fins.size());
        for (const WedgeRec &wd : wedgeRecs) {
            FinRecord f;
            f.height = wd.height;
            f.length = wd.span;
            f.tines = 0;
            f.id = wid++;
            f.kind = "wedge";
            f.triRanges = {TriRange{wd.triRange[0] + baseLen, wd.triRange[1] + baseLen}};
            f.line = wd.line;
            f.span = wd.span;
            fins.push_back(std::move(f));
        }
        const size_t shortAt = baseLen + wedgeTris.size();
        std::vector<BuiltProp> shortProps;
        for (const BuiltProp &q0 : shortW.props) {
            BuiltProp q = q0;
            for (TriRange &r : q.triRanges) r = TriRange{r[0] + shortAt, r[1] + shortAt};
            FinRecord f;
            f.height = q.height;
            f.length = q.span;
            f.tines = q.tines;
            f.id = wid++;
            f.kind = "prop";
            f.isShort = true;
            f.triRanges = q.triRanges;
            f.line = q.line;
            f.span = q.span;
            fins.push_back(std::move(f));
            shortProps.push_back(std::move(q));
        }
        std::vector<int> servedRegions = base.servedRegions;
        servedRegions.insert(servedRegions.end(), shortW.served.begin(), shortW.served.end());
        Built aut = base;
        aut.mode = mode;
        append(aut.triangles, aut.tags, wedgeTris, wedgeTags);
        append(aut.triangles, aut.tags, shortW.triangles, shortW.tags);
        for (BuiltProp &q : shortProps) aut.props.push_back(std::move(q));
        aut.servedRegions = servedRegions;
        aut.tines = base.tines + wedgeTines + shortW.tines;
        // A tined rib/wedge IS the combined support (a "brace"); a tineless one is a plain prop.
        aut.braceCount = withTines ? int(fins.size()) : wedgeCount;
        aut.propCount = withTines ? 0 : int(base.fins.size()) + int(shortW.props.size());
        aut.unserved = wedgedPatches ? unservedAfterWedges(topo, rot, result, servedRegions, wedgeTris)
                                     : base.unserved - int(shortW.served.size());
        aut.fins = std::move(fins);
        // FULL COVERAGE: Auto, then walls under the red it left bare. Not on a
        // point-seated part with the pad off, which gets no walls at all.
        const bool noWalls = base.seating.kind == "point" && !base.pad;
        if (mode == "full" && !noWalls) {
            BuildOpts fo = opts;
            fo.tines = withTines;
            fo.coverage = coverage;
            return withFill(topo, result, rot, fo, std::move(aut));
        }
        return aut;
    }

    // Prop (and any other mode) is its own support, built by its own module.
    const BedContact contact = bedContact(topo, result, rot);
    const Seating seating = seatingOf(result, contact.pts);
    Tris padOut;
    std::optional<PadResult> pad;
    if (opts.bedPad && result.bedArea < FIN.padMinArea)
        pad = buildPad(contact.pts, SurfTris(seatedPartTris(topo, rot, result.offset)), padOut, opts.layerHeight);
    // A part seated on a POINT gets no props -- unless the bed pad is on. Draw builds no
    // walls here: it exports only the pad and hand-placed walls.
    PropsResult built;
    if (mode == "draw" || (seating.kind == "point" && !pad)) {
        built = noProps();
    } else {
        PropOpts po = propOptsOf(opts);
        po.rasterVeto = rasterVeto;
        built = buildProps(topo, result, rot, po);
    }
    Built b;
    b.triangles = std::move(built.triangles);
    b.tags = std::move(built.tags);
    if (b.tags.size() < b.triangles.size()) b.tags.resize(b.triangles.size(), 0);
    b.padTriangles = std::move(padOut);
    b.pad = pad.has_value();
    b.padRecord = pad;
    b.mode = mode;
    for (const PropRecord &q : built.props) b.fins.push_back(finOf(q));
    for (const PropRecord &q : built.props) b.props.push_back(BuiltProp(q));
    b.volume = built.volume;
    b.skipped = built.skipped;
    b.tines = built.tines;
    b.servedRegions = built.servedRegions;
    // regions minus SERVED REGIONS, not minus the prop count
    b.unserved = int(result.regions.size()) - built.served;
    b.sagRisk = built.sagRisk;
    b.seating = seating;
    b.seatingKind = seating.kind;
    return b;
}

Built buildFinsAndBraces(Topology &topo, const AnalyzeResult &result, const Rot &rot, const BuildOpts &opts) {
    if (opts.tunables) applyTunables(*opts.tunables);
    // the interface crest's 'flat' test reads the underside off the seated part
    crestPart(crestOn() ? std::make_shared<const SurfTris>(seatedPartTris(topo, rot, result.offset)) : nullptr,
              opts.layerHeight);
    Built built = buildFinsCore(topo, result, rot, opts);
    // Sway braces are an optional ADD-ON to whatever the mode placed. Draw stands its
    // braces by hand, so it never gets Auto's.
    if (opts.mode == "draw" || !opts.sway || !opts.sway->on) return built;
    // Braces run LAST: hand the props' and wedges' centrelines over as things to stand clear of.
    SwayBuildOpts so;
    static_cast<SwayOpts &>(so) = *opts.sway;
    so.tines = opts.tines;
    so.layerHeight = opts.layerHeight;
    so.iface = PROP.iface;
    for (const FinRecord &f : built.fins) if (!f.line.empty()) so.avoid.walls.push_back(f.line);
    const SwayBraces sw = buildSwayBraces(topo, result, rot, so);
    // Each brace also gets a fin record (per-fin removal).
    const size_t base = built.triangles.size();
    int id = int(built.fins.size());
    for (const FinRecord &f : built.fins) id = int(jsMax(id, f.id + 1));
    for (const SwayRib &r : sw.ribs) {
        FinRecord f;
        f.height = r.height;
        f.length = r.depth;
        f.tines = r.tines;
        f.id = id++;
        f.kind = "sway";
        const TriRange tr = r.triRange ? *r.triRange : TriRange{0, 0};
        f.triRanges = {TriRange{tr[0] + base, tr[1] + base}};
        f.line = r.foot;
        f.span = r.depth;
        built.fins.push_back(std::move(f));
    }
    append(built.triangles, built.tags, sw.triangles, sw.tags);
    SwayResult s;
    s.count = sw.count;
    s.tines = sw.tines;
    s.skipped = sw.skipped;
    s.reason = sw.reason;
    // The outlines travel back so a brace the user then clicks by hand can be checked
    // against these.
    for (const SwayRib &r : sw.ribs) {
        SwayOutline o;
        o.foot = r.foot;
        o.halfW = r.halfW;
        o.th = r.th;
        o.height = r.height;
        for (const SwayLevel &l : r.levels) o.levels.push_back(SwayOutline::Level{l.z, l.a, l.b});
        s.braces.push_back(std::move(o));
    }
    built.sway = std::move(s);
    return built;
}

} // namespace

Built buildFins(Topology &topo, const AnalyzeResult &result, const Rot &rot, const BuildOpts &opts) {
    Built built = buildFinsAndBraces(topo, result, rot, opts);
    crestPart(nullptr);   // the crest's part is only for this build
    // A piece of the part that starts in mid-air needs saying no matter what was placed.
    built.floating = floatingPieces(topo, result, rot);
    return built;
}

}} // namespace
