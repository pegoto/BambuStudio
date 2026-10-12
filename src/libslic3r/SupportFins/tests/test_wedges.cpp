// Parity test for Engine/Fins/Wedges.* (web/fins/wedges.js). Replays what
// tools/wedges_ref.mjs recorded against the upstream JS, per scene:
//   patches   findWallPatches (identity check: the wedge calls address patches by index)
//   grip      gripPatches: the downward patches and the faceMap, in insertion order
//   cols      perpColumns on every patch (coverage pitches, PERP.pitch, the full extent;
//             PERP.gap 0.2 / 0.3)
//   perp      buildPerpFins under 8 states (PLA / PETG x iface off / 'all' x cutout none /
//             diamond) on every wedgeable patch with tines x pitch x tineDensity, and `{}`
//             on every downward patch: triangles in emission order, interface tags, tines,
//             count, and every wedge record field
//   builds    three auto-mode configs: wedgeVeto replayed on the calls buildProps made
//             (same normalProps object identities), on every base prop against base.props
//             and against no walls; propServesPatch for every patch x (all props, each prop);
//             fins.js's wedge loop; bareAfterWedges / unservedAfterWedges
//   bareRot   bareAfterWedges in a frame turned about z
// Counts, booleans, indices and tags are compared exactly, doubles within 1e-9 (every
// double that is not bit-identical is counted and reported).
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/wedges_ref.mjs   # writes refs_wedges/
//   ./build-sf/test_wedges [refs_wedges_dir]
#include "Engine/Cutout.hpp"
#include "Engine/Fins/Seating.hpp"
#include "Engine/Fins/Wedges.hpp"
#include "Engine/Overhangs.hpp"
#include "Engine/Planes.hpp"
#include "Engine/Prop/Crest.hpp"
#include "Engine/Prop/Raster.hpp"
#include "Engine/Prop/Surface.hpp"
#include "Engine/PropConfig.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>

namespace Slic3r { namespace SupportFins {
// Entry.hpp's (it includes Fins.hpp, whose TriRange clashes with Prop/Raster.hpp's)
Offset seatSoup(const Soup &input, Soup &pos);
}}

using namespace Slic3r::SupportFins;
using json = nlohmann::json;
namespace fs = std::filesystem;

static long g_checks = 0, g_fail = 0, g_inexact = 0, g_dchecks = 0;
static std::string g_ctx;

static double num(const json &j) {
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        if (s == "-0") return -0.0;
        if (s == "NaN") return std::numeric_limits<double>::quiet_NaN();
        if (s == "Infinity") return INF;
        if (s == "-Infinity") return -INF;
        std::fprintf(stderr, "bad number %s\n", s.c_str());
        return std::numeric_limits<double>::quiet_NaN();
    }
    return j.get<double>();
}
static void fail(const std::string &what) {
    if (++g_fail <= 40) std::fprintf(stderr, "FAIL %s: %s\n", g_ctx.c_str(), what.c_str());
}
static void checkD(double got, double want, const char *what) {
    g_checks++; g_dchecks++;
    if (std::isnan(got) && std::isnan(want)) return;
    if (got == want) { if (std::signbit(got) != std::signbit(want)) g_inexact++; return; }
    g_inexact++;
    if (std::isfinite(got) && std::isfinite(want) && std::abs(got - want) <= 1e-9) return;
    char b[256];
    std::snprintf(b, sizeof b, "%s: got %.17g want %.17g", what, got, want);
    fail(b);
}
static bool checkI(long got, long want, const char *what) {
    g_checks++;
    if (got != want) { fail(std::string(what) + ": got " + std::to_string(got) + " want " + std::to_string(want)); return false; }
    return true;
}
static std::vector<Vec3d> lineOf(const json &j) {
    std::vector<Vec3d> l;
    for (const json &p : j) l.emplace_back(num(p[0]), num(p[1]), num(p[2]));
    return l;
}
static void checkLine(const std::vector<Vec3d> &got, const json &want, const char *what) {
    if (!checkI(long(got.size()), long(want.size()), what)) return;
    for (size_t i = 0; i < got.size(); i++)
        for (int k = 0; k < 3; k++) checkD(got[i][k], num(want[i][k]), what);
}
static void checkFlat(const Tris &got, const json &want, const char *what) {
    if (!checkI(long(got.size() * 3), long(want.size()), what)) return;
    for (size_t i = 0; i < got.size(); i++)
        for (int k = 0; k < 3; k++) checkD(got[i][k], num(want[i * 3 + k]), what);
}
static void checkIdx(const std::vector<int> &got, const json &want, const char *what) {
    if (!checkI(long(got.size()), long(want.size()), what)) return;
    for (size_t i = 0; i < got.size(); i++) checkI(got[i], want[i].get<long>(), what);
}
static void checkTags(const Tris &out, const IfaceTags &tags, const json &want) {
    checkI(long(tags.size()), long(out.size()), "tags size");
    std::vector<long> runs;
    int cur = 0; long len = 0;
    for (size_t i = 0; i < out.size(); i++) {
        const int v = i < tags.size() && tags[i] ? 1 : 0;
        if (v != cur) { runs.push_back(len); cur = v; len = 0; }
        len++;
    }
    runs.push_back(len);
    if (!checkI(long(runs.size()), long(want.size()), "tag runs")) return;
    for (size_t i = 0; i < runs.size(); i++) checkI(runs[i], want[i].get<long>(), "tag run length");
}
static WedgeOpts optsOf(const json &o) {
    WedgeOpts w;
    w.tines = o["tines"].is_null() ? true : o["tines"].get<bool>();   // JS `opts.tines !== false`
    if (!o["pitch"].is_null()) w.pitch = num(o["pitch"]);
    if (!o["tineDensity"].is_null()) w.tineDensity = num(o["tineDensity"]);
    return w;
}
static PropRecord recOf(const json &j) {
    PropRecord q;
    q.line = lineOf(j["line"]);
    q.squat = j["squat"].get<bool>();
    return q;
}

int main(int argc, char **argv) {
    const fs::path dir = argc > 1 ? fs::path(argv[1])
                                  : fs::path(__FILE__).parent_path().parent_path() / "refs_wedges";
    if (!fs::is_directory(dir)) {
        std::fprintf(stderr, "no dumps in %s: run SUPPORT_FINS_DIR=... node tools/wedges_ref.mjs\n", dir.string().c_str());
        return 1;
    }
    std::vector<fs::path> files;
    for (const auto &e : fs::directory_iterator(dir))
        if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::fprintf(stderr, "no dumps in %s\n", dir.string().c_str()); return 1; }

    const PropConfig PROP0 = PROP;
    const PerpConfig PERP0 = PERP;
    const CutConfig CUT0 = CUT;
    long nPerp = 0, nWedges = 0, nCols = 0, nVeto = 0, nVetoYes = 0, nServes = 0, nBare = 0;

    for (const fs::path &file : files) {
        std::ifstream in(file);
        const json J = json::parse(in);
        const std::string scene = J["model"].get<std::string>() + " " + J["rot"].get<std::string>();
        const long failBefore = g_fail, checksBefore = g_checks, inexactBefore = g_inexact;
        PROP = PROP0; PERP = PERP0; CUT = CUT0;
        crestPart(nullptr);

        // PERP's defaults
        g_ctx = scene + " PERP";
        const json &P0 = J["PERP"];
        checkD(PERP.th, num(P0["th"]), "th"); checkD(PERP.gap, num(P0["gap"]), "gap");
        checkD(PERP.footHalf, num(P0["footHalf"]), "footHalf"); checkD(PERP.footH, num(P0["footH"]), "footH");
        checkD(PERP.pitch, num(P0["pitch"]), "pitch"); checkD(PERP.tStep, num(P0["tStep"]), "tStep");
        checkD(PERP.minH, num(P0["minH"]), "minH"); checkD(PERP.inset, num(P0["inset"]), "inset");
        checkI(PERP.maxRow, P0["maxRow"].get<long>(), "maxRow");
        checkD(PERP.minArea, num(P0["minArea"]), "minArea"); checkD(PERP.minWidth, num(P0["minWidth"]), "minWidth");

        // exactly as computeFins: seatSoup, buildTopology, analyze(..., IDENTITY3)
        Soup posed;
        for (const json &v : J["soup"]) posed.push_back(num(v));
        Soup pos;
        seatSoup(posed, pos);
        Topology topo = buildTopology(std::move(pos));
        const AnalyzeResult res = analyze(topo, DEFAULT_THRESHOLD, IDENTITY3);
        const Rot rot = IDENTITY3;
        const Offset off = res.offset;
        g_ctx = scene + " analyze";
        checkD(off.x, num(J["offset"]["x"]), "offset.x"); checkD(off.y, num(J["offset"]["y"]), "offset.y");
        checkD(off.z, num(J["offset"]["z"]), "offset.z");
        checkI(long(res.regions.size()), J["regions"].get<long>(), "regions");
        auto partTris = std::make_shared<const SurfTris>(seatedPartTris(topo, rot, off));

        // ---- patches
        g_ctx = scene + " patches";
        const std::vector<WallPatch> all = findWallPatches(topo, rot, off);
        if (!checkI(long(all.size()), long(J["patches"].size()), "patch count")) {
            std::printf("%-22s FAIL (patches differ; skipped)\n", scene.c_str());
            continue;
        }
        for (size_t i = 0; i < all.size(); i++) {
            const json &w = J["patches"][i];
            checkI(all[i].faces[0], w["seed"].get<long>(), "seed");
            checkI(long(all[i].faces.size()), w["faces"].get<long>(), "faces");
            checkD(all[i].area, num(w["area"]), "area"); checkD(all[i].n.z, num(w["nz"]), "n.z");
            checkD(all[i].u0, num(w["u0"]), "u0"); checkD(all[i].u1, num(w["u1"]), "u1");
            checkD(all[i].t0, num(w["t0"]), "t0"); checkD(all[i].t1, num(w["t1"]), "t1");
        }

        // ---- gripPatches
        g_ctx = scene + " gripPatches";
        {
            const GripPatches g = gripPatches(topo, res, rot);
            const json &gi = J["grip"]["idx"];
            if (checkI(long(g.patches.size()), long(gi.size()), "grip patches")) {
                for (size_t i = 0; i < g.patches.size(); i++) {
                    const WallPatch &p = g.patches[i], &q = all[gi[i].get<size_t>()];
                    checkI(p.faces == q.faces, 1, "grip patch faces");
                    checkD(p.area, q.area, "grip patch area");
                }
            }
            const json &fm = J["grip"]["faceMap"];
            if (checkI(long(g.faceOrder.size()), long(fm.size()), "faceMap size")) {
                for (size_t i = 0; i < fm.size(); i++) {
                    checkI(g.faceOrder[i], fm[i][0].get<long>(), "faceMap key order");
                    const auto it = g.faceMap.find(g.faceOrder[i]);
                    checkI(it == g.faceMap.end() ? -1 : it->second, fm[i][1].get<long>(), "faceMap value");
                }
            }
            checkI(long(g.faceMap.size()), long(g.faceOrder.size()), "faceMap keys");
        }

        // ---- perpColumns
        g_ctx = scene + " perpColumns";
        for (const json &c : J["cols"]) {
            PERP.gap = num(c["gap"]);
            const std::vector<double> cols = perpColumns(all[c["patch"].get<size_t>()], num(c["lo"]), num(c["hi"]), num(c["pitch"]));
            const json &want = c["out"];
            if (checkI(long(cols.size()), long(want.size()), "columns"))
                for (size_t i = 0; i < cols.size(); i++) checkD(cols[i], num(want[i]), "column u");
            nCols++;
        }
        PERP = PERP0;

        // ---- states: buildFins' entry (applyTunables' effect as dumped, then crestPart)
        const json &states = J["states"];
        auto enter = [&](size_t si) {
            const json &s = states[si];
            PROP = PROP0; PERP = PERP0; CUT = CUT0;
            const std::string iface = s["iface"].get<std::string>();
            PROP.gap = num(s["propGap"]);
            PROP.iface = iface == "all" ? Iface::All : iface == "flat" ? Iface::Flat : Iface::Off;
            PERP.gap = num(s["propGap"]);       // applyTunables: set(PERP, 'gap', t.propGap)
            CUT.pattern = s["cutout"].get<std::string>();
            g_ctx = scene + " state " + std::to_string(si);
            checkD(PERP.gap, num(s["perpGap"]), "PERP.gap after applyTunables");
            checkI(CUT.pattern == s["cut"].get<std::string>(), 1, "CUT.pattern");
            crestPart(crestOn() ? partTris : nullptr, num(s["layerHeight"]));
        };

        // ---- buildPerpFins
        const json &results = J["results"];
        long cur = -1;
        for (const json &e : J["perp"]) {
            const long si = e["state"].get<long>();
            if (si != cur) { enter(size_t(si)); cur = si; }
            const size_t pi = e["patch"].get<size_t>();
            g_ctx = scene + " buildPerpFins state " + std::to_string(si) + " patch " + std::to_string(pi) + " " + e["opts"].dump();
            const PerpFins w = buildPerpFins(all[pi], topo, rot, off, optsOf(e["opts"]));
            const json &r = results[e["res"].get<size_t>()];
            checkFlat(w.triangles, r["triangles"], "triangles");
            checkTags(w.triangles, w.tags, r["tags"]);
            checkI(w.tines, r["tines"].get<long>(), "tines");
            checkI(w.count, r["count"].get<long>(), "count");
            if (checkI(long(w.wedges.size()), long(r["wedges"].size()), "wedges")) {
                for (size_t k = 0; k < w.wedges.size(); k++) {
                    const PerpWedge &wd = w.wedges[k];
                    const json &x = r["wedges"][k];
                    checkI(long(wd.triRange[0]), x["triRange"][0].get<long>(), "triRange[0]");
                    checkI(long(wd.triRange[1]), x["triRange"][1].get<long>(), "triRange[1]");
                    checkLine(wd.line, x["line"], "wedge line");
                    checkD(wd.height, num(x["height"]), "wedge height");
                    checkD(wd.span, num(x["span"]), "wedge span");
                }
            }
            // the JS record's keys are exactly these four
            const json &keys = r["keys"];
            checkI(keys == json::array({"count", "tines", "triangles", "wedges"}), 1, "record keys");
            nPerp++;
            nWedges += w.count;
        }

        // ---- builds: wedgeVeto, propServesPatch, the wedge loop, bareAfterWedges
        Tris wedgeTris0;
        for (const json &b : J["builds"]) {
            const size_t si = b["state"].get<size_t>();
            enter(si);
            g_ctx = scene + " build " + b["opts"].dump();
            std::vector<WallPatch> wedgeable;
            std::vector<int> wIdx;
            for (size_t i = 0; i < all.size(); i++) {
                const WallPatch &p = all[i];
                if (p.n.z < -0.05 && p.area >= PERP.minArea && (p.u1 - p.u0) >= PERP.minWidth) {
                    wedgeable.push_back(p);
                    wIdx.push_back(int(i));
                }
            }
            checkIdx(wIdx, b["wedgeable"], "wedgeable");
            const WedgeOpts wo = optsOf(b["wedgeOpts"]);

            // the normalProps objects buildProps handed the veto, by identity
            std::deque<std::vector<PropRecord>> normals;   // stable addresses
            for (const json &n : b["normals"]) {
                normals.emplace_back();
                for (const json &q : n) normals.back().push_back(recOf(q));
            }
            {
                const RasterVeto veto = wedgeVeto(wedgeable, topo, rot, off, wo);
                const RasterVeto vetoCopy = veto;   // copies share the closure's cache, as JS references do
                size_t k = 0;
                for (const json &c : b["vetoCalls"]) {
                    const PropRecord q = recOf(c);
                    const bool r = (k++ % 2 ? vetoCopy : veto)(q, normals[c["np"].get<size_t>()]);
                    checkI(r, c["r"].get<bool>(), "veto (buildProps call)");
                    nVeto++; nVetoYes += r;
                }
            }
            std::vector<PropRecord> props;
            for (const json &q : b["props"]) props.push_back(recOf(q));
            {
                const RasterVeto veto2 = wedgeVeto(wedgeable, topo, rot, off, wo);
                const RasterVeto veto3 = wedgeVeto(wedgeable, topo, rot, off, wo);
                const std::vector<PropRecord> none;
                for (size_t i = 0; i < props.size(); i++) {
                    const bool r2 = veto2(props[i], props), r3 = veto3(props[i], none);
                    checkI(r2, b["post"][i].get<bool>(), "veto (base props)");
                    checkI(r3, b["postNone"][i].get<bool>(), "veto (no normal walls)");
                    nVeto += 2; nVetoYes += r2 + r3;
                }
            }
            for (size_t i = 0; i < all.size(); i++) {
                const json &s = b["serves"][i];
                checkI(propServesPatch(all[i], props), s[0].get<bool>(), "propServesPatch(p, props)");
                for (size_t k = 0; k < props.size(); k++)
                    checkI(propServesPatch(all[i], std::vector<PropRecord>{props[k]}), s[1][k].get<bool>(), "propServesPatch(p, [q])");
                nServes += 1 + long(props.size());
            }
            // fins.js's wedge loop
            Tris wedgeTris;
            const json &loop = b["loop"];
            if (checkI(long(loop.size()), long(wedgeable.size()), "loop")) {
                for (size_t i = 0; i < wedgeable.size(); i++) {
                    const bool served = propServesPatch(wedgeable[i], props);
                    checkI(served, loop[i]["served"].get<bool>(), "loop served");
                    if (served) continue;
                    const PerpFins w = buildPerpFins(wedgeable[i], topo, rot, off, wo);
                    checkI(w.count, loop[i]["count"].get<long>(), "loop count");
                    if (!w.count) continue;
                    for (const Vec3d &v : w.triangles) wedgeTris.push_back(v);
                }
            }
            checkI(long(wedgeTris.size()), b["wedgeTrisLen"].get<long>(), "wedgeTris length");
            double sum = 0;
            for (const Vec3d &v : wedgeTris) sum = sum + v[0] + 2 * v[1] + 3 * v[2];
            checkD(sum, num(b["wedgeTrisSum"]), "wedgeTris checksum");
            if (b.contains("wedgeTris")) {
                checkFlat(wedgeTris, b["wedgeTris"], "wedgeTris");
                wedgeTris0 = wedgeTris;
            }
            std::vector<int> servedRegions;
            for (const json &r : b["servedRegions"]) servedRegions.push_back(r.get<int>());
            checkIdx(bareAfterWedges(topo, rot, res, servedRegions, wedgeTris), b["bare"], "bareAfterWedges");
            checkI(unservedAfterWedges(topo, rot, res, servedRegions, wedgeTris), b["unserved"].get<long>(), "unservedAfterWedges");
            checkIdx(bareAfterWedges(topo, rot, res, {}, {}), b["bareNone"], "bareAfterWedges (none)");
            checkIdx(bareAfterWedges(topo, rot, res, {}, wedgeTris), b["bareWedges"], "bareAfterWedges (wedges only)");
            nBare += 4;
        }

        // ---- bareAfterWedges in a turned frame
        {
            PROP = PROP0; PERP = PERP0; CUT = CUT0;
            crestPart(nullptr);
            g_ctx = scene + " bareRot";
            const json &br = J["bareRot"];
            Rot rotZ;
            for (int i = 0; i < 9; i++) rotZ[i] = num(br["rot"][i]);
            const AnalyzeResult resZ = analyze(topo, DEFAULT_THRESHOLD, rotZ);
            checkD(resZ.offset.x, num(br["offset"]["x"]), "offset.x"); checkD(resZ.offset.y, num(br["offset"]["y"]), "offset.y");
            checkD(resZ.offset.z, num(br["offset"]["z"]), "offset.z");
            checkI(long(resZ.regions.size()), br["regions"].get<long>(), "regions");
            std::vector<int> served;
            for (const json &r : br["served"]) served.push_back(r.get<int>());
            checkIdx(bareAfterWedges(topo, rotZ, resZ, served, wedgeTris0), br["out"], "bareAfterWedges (rotZ)");
            nBare++;
        }

        std::printf("%-22s %s  checks=%ld inexact=%ld\n", scene.c_str(), g_fail == failBefore ? "ok  " : "FAIL",
                    g_checks - checksBefore, g_inexact - inexactBefore);
    }
    PROP = PROP0; PERP = PERP0; CUT = CUT0;
    crestPart(nullptr);
    std::printf("buildPerpFins calls=%ld wedges=%ld  perpColumns=%ld  veto=%ld (yes %ld)  propServesPatch=%ld  bare=%ld\n",
                nPerp, nWedges, nCols, nVeto, nVetoYes, nServes, nBare);
    std::printf("%s: %ld checks (%ld doubles), %ld failed, %ld doubles not bit-identical\n",
                g_fail ? "FAIL" : "PASS", g_checks, g_dchecks, g_fail, g_inexact);
    return g_fail ? 1 : 0;
}
