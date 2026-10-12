// Parity test for Engine/Sway.* (web/sway.js) against the JS oracle dumps written by
// tools/sway_ref.mjs:
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/sway_ref.mjs   # writes refs_sway/
//   ./build-sf/test_sway [refs_sway_dir] [--own-topology] [--verbose]
//
// dumpDir defaults to <SupportFins>/refs_sway, found via __FILE__. For every dump:
// rebuild topology + analyze from the same seated soup (feeding the JS per-face area /
// normal in unless --own-topology, so the test pins sway.js alone; the C++/JS
// difference is counted and reported), then replay every call the oracle made, in the
// same order:
//   buildSwayBraces for each option variant: count / tines / skipped / reason exact,
//     every rib record field (doubles within 1e-9), triangles same count, order and
//     coordinates, interface tags exact, each rib's tris == its triRange slice;
//   swayClashes on rib pairs and on shifted / squashed copies (the level path),
//     swayClashesWall against the synthetic walls; faceIsUpright on every face;
//   swayAtFace on Draw-style picks (plain, against Auto's braces, against the walls,
//     allowStilt forced off), half through a SwayPatchCache, half without.
// Counts how many compared doubles were not bit-identical (informational; --verbose
// lists them).
#include "Engine/Overhangs.hpp"
#include "Engine/Sway.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

using namespace Slic3r::SupportFins;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

long g_checks = 0, g_fail = 0, g_inexact = 0, g_doubles = 0;
double g_maxErr = 0;
std::string g_ctx;
bool g_ownTopology = false, g_verbose = false;
long g_topoAreaDiff = 0, g_topoNrmDiff = 0, g_bearingDiff = 0;

double num(const json &j) {
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

void fail(const std::string &what) {
    if (++g_fail <= 40) std::printf("FAIL [%s] %s\n", g_ctx.c_str(), what.c_str());
}

void checkD(double got, const json &jw, const std::string &what) {
    const double want = num(jw);
    g_checks++; g_doubles++;
    if (std::isnan(got) && std::isnan(want)) return;
    if (got == want) {
        if (std::signbit(got) != std::signbit(want)) {
            g_inexact++;
            if (g_verbose) std::printf("  inexact [%s] %s: got %g want %g (sign of zero)\n", g_ctx.c_str(), what.c_str(), got, want);
        }
        return;
    }
    g_inexact++;
    if (g_verbose) std::printf("  inexact [%s] %s: got %.17g want %.17g\n", g_ctx.c_str(), what.c_str(), got, want);
    const double err = std::abs(got - want) / std::max(1.0, std::abs(want));
    if (std::isfinite(err)) g_maxErr = std::max(g_maxErr, err);
    if (std::isfinite(got) && std::isfinite(want) && err <= 1e-9) return;
    char b[320];
    std::snprintf(b, sizeof b, "%s: got %.17g want %.17g", what.c_str(), got, want);
    fail(b);
}

void checkI(long got, long want, const std::string &what) {
    g_checks++;
    if (got != want) fail(what + ": got " + std::to_string(got) + " want " + std::to_string(want));
}

void checkB(bool got, bool want, const std::string &what) {
    g_checks++;
    if (got != want) fail(what + ": got " + (got ? "true" : "false") + " want " + (want ? "true" : "false"));
}

void checkS(const std::optional<std::string> &got, const json &jw, const std::string &what) {
    g_checks++;
    if (jw.is_null()) { if (got) fail(what + ": got \"" + *got + "\" want null"); return; }
    const std::string want = jw.get<std::string>();
    if (!got) { fail(what + ": got null want \"" + want + "\""); return; }
    if (*got != want) fail(what + ": got \"" + *got + "\" want \"" + want + "\"");
}

void checkPt(const Vec3d &got, const json &jw, const std::string &what) {
    checkD(got.x(), jw[0], what + ".x");
    checkD(got.y(), jw[1], what + ".y");
    checkD(got.z(), jw[2], what + ".z");
}

void checkTris(const Tris &got, const IfaceTags &tags, const json &flat, const json &jtags, const std::string &what) {
    if (tags.size() != got.size()) fail(what + ": tags size " + std::to_string(tags.size()) + " != tris " + std::to_string(got.size()));
    if (got.size() * 3 != flat.size()) {
        fail(what + ": vertex count got " + std::to_string(got.size()) + " want " + std::to_string(flat.size() / 3));
        return;
    }
    for (size_t i = 0; i < got.size(); i++) {
        for (int k = 0; k < 3; k++) checkD(got[i][k], flat[i * 3 + k], what + "[" + std::to_string(i) + "][" + std::to_string(k) + "]");
        if (i < tags.size()) checkI(tags[i], jtags[i].get<int>(), what + " tag[" + std::to_string(i) + "]");
    }
}

const std::vector<std::string> kRibKeysOk = {"ok", "tris", "tines", "height", "depth", "th", "stilt", "foot", "halfW", "levels"};
const std::vector<std::string> kRibKeysFail = {"ok", "reason"};

void checkRib(const SwayRib &r, const json &e, const std::string &P, bool withTris) {
    std::vector<std::string> keys = e["keys"].get<std::vector<std::string>>();
    if (!keys.empty() && keys.back() == "triRange") keys.pop_back();
    if (keys != (e["ok"].get<bool>() ? kRibKeysOk : kRibKeysFail)) fail(P + ": upstream rib keys changed: " + e["keys"].dump());
    checkB(r.ok, e["ok"].get<bool>(), P + ".ok");
    if (r.ok != e["ok"].get<bool>()) {
        if (r.reason) fail(P + ": C++ reason: " + *r.reason);
        if (e.contains("reason")) fail(P + ": JS reason: " + e["reason"].get<std::string>());
        return;
    }
    if (!r.ok) { checkS(r.reason, e["reason"], P + ".reason"); return; }
    if (r.reason) fail(P + ": ok rib carries a reason");
    checkI(r.tines, e["tines"].get<int>(), P + ".tines");
    checkD(r.height, e["height"], P + ".height");
    checkD(r.depth, e["depth"], P + ".depth");
    checkD(r.th, e["th"], P + ".th");
    checkD(r.stilt, e["stilt"], P + ".stilt");
    checkD(r.halfW, e["halfW"], P + ".halfW");
    checkI(long(r.foot.size()), long(e["foot"].size()), P + ".foot.size");
    for (size_t k = 0; k < std::min(r.foot.size(), e["foot"].size()); k++) checkPt(r.foot[k], e["foot"][k], P + ".foot[" + std::to_string(k) + "]");
    const json &L = e["levels"];
    checkI(long(r.levels.size()), long(L.size()), P + ".levels.size");
    for (size_t k = 0; k < std::min(r.levels.size(), L.size()); k++) {
        const std::string Q = P + ".levels[" + std::to_string(k) + "]";
        checkD(r.levels[k].z, L[k]["z"], Q + ".z");
        checkPt(r.levels[k].a, L[k]["a"], Q + ".a");
        checkPt(r.levels[k].b, L[k]["b"], Q + ".b");
    }
    checkI(long(r.tris.size()), e["nTris"].get<long>(), P + ".tris.size");
    if (e.contains("triRange")) {
        if (!r.triRange) fail(P + ": no triRange");
        else {
            checkI(r.triRange->first, e["triRange"][0].get<long>(), P + ".triRange[0]");
            checkI(r.triRange->second, e["triRange"][1].get<long>(), P + ".triRange[1]");
        }
    } else if (r.triRange) fail(P + ": unexpected triRange");
    if (withTris) checkTris(r.tris, r.tags, e["tris"], e["tags"], P + ".tris");
}

Iface ifaceOf(const json &j) {
    const std::string s = j.get<std::string>();
    if (s == "all") return Iface::All;
    if (s == "flat") return Iface::Flat;
    if (s != "off") fail("unknown iface " + s);
    return Iface::Off;
}

std::vector<std::vector<Vec3d>> wallsOf(const json &j) {
    std::vector<std::vector<Vec3d>> walls;
    for (const json &line : j) {
        std::vector<Vec3d> l;
        for (const json &p : line) l.emplace_back(num(p[0]), num(p[1]), num(p[2]));
        walls.push_back(std::move(l));
    }
    return walls;
}

/// The JS options object -> SwayBuildOpts. Absent numbers keep the struct defaults,
/// which are the JS defaults (settings()).
SwayBuildOpts optsOf(const json &o) {
    SwayBuildOpts s;
    if (o.contains("on")) s.on = o["on"].get<bool>();
    if (o.contains("gripFrom")) s.gripFrom = num(o["gripFrom"]);
    if (o.contains("tineSpacing")) s.tineSpacing = num(o["tineSpacing"]);
    if (o.contains("reach")) s.reach = num(o["reach"]);
    if (o.contains("gap")) s.gap = num(o["gap"]);
    if (o.contains("tines")) s.tines = o["tines"].get<bool>();
    if (o.contains("layerHeight")) s.layerHeight = num(o["layerHeight"]);
    if (o.contains("iface")) s.iface = ifaceOf(o["iface"]);
    if (o.contains("allowStilt")) s.allowStilt = o["allowStilt"].get<bool>();
    if (o.contains("avoid") && o["avoid"].contains("walls")) s.avoid.walls = wallsOf(o["avoid"]["walls"]);
    return s;
}

void checkSwayConstants(const json &S) {
    const std::pair<const char *, double> d[] = {
        {"maxLeanDeg", SWAY.maxLeanDeg}, {"minFaceH", SWAY.minFaceH}, {"minTopFrac", SWAY.minTopFrac},
        {"minPartH", SWAY.minPartH}, {"thMin", SWAY.thMin}, {"thPerMm", SWAY.thPerMm}, {"thMax", SWAY.thMax},
        {"reach", SWAY.reach}, {"topDepth", SWAY.topDepth}, {"minDepth", SWAY.minDepth}, {"maxDepth", SWAY.maxDepth},
        {"topClear", SWAY.topClear}, {"minRibH", SWAY.minRibH}, {"footH", SWAY.footH}, {"footHalf", SWAY.footHalf},
        {"footPad", SWAY.footPad}, {"gap", SWAY.gap}, {"tineReach", SWAY.tineReach}, {"tineW", SWAY.tineW},
        {"tineOverlap", SWAY.tineOverlap}, {"tineSpanMax", SWAY.tineSpanMax}, {"tineSpacing", SWAY.tineSpacing},
        {"minTines", double(SWAY.minTines)}, {"minGripShare", SWAY.minGripShare}, {"stiltMax", SWAY.stiltMax},
        {"stiltMaxFrac", SWAY.stiltMaxFrac}, {"pitch", SWAY.pitch}, {"maxPerFace", double(SWAY.maxPerFace)},
        {"endInset", SWAY.endInset}, {"maxFaces", double(SWAY.maxFaces)}, {"minBearingSep", SWAY.minBearingSep},
        {"clearance", SWAY.clearance}, {"levelStep", SWAY.levelStep}, {"wallHalf", SWAY.wallHalf}};
    size_t known = 1;   // nudges
    for (const auto &kv : d) {
        if (!S.contains(kv.first)) { fail(std::string("SWAY.") + kv.first + " missing upstream"); continue; }
        known++;
        if (num(S[kv.first]) != kv.second) fail(std::string("SWAY.") + kv.first + " differs");
    }
    if (S.size() != known) fail("SWAY has " + std::to_string(S.size()) + " fields upstream, " + std::to_string(known) + " here");
    const json &n = S["nudges"];
    if (n.size() != SWAY.nudges.size()) fail("SWAY.nudges size differs");
    else for (size_t i = 0; i < n.size(); i++) if (num(n[i]) != SWAY.nudges[i]) fail("SWAY.nudges differs");
}

bool runDump(const fs::path &file) {
    const long failBefore = g_fail;
    g_ctx = file.filename().string();
    std::ifstream in(file);
    const json J = json::parse(in);

    checkSwayConstants(J["SWAY"]);
    {   // the lean cut every upright test compares against (libm sin vs V8's fdlibm)
        const double leanCut = std::sin((SWAY.maxLeanDeg * M_PI) / 180);
        if (leanCut != J["leanCut"].get<double>()) fail("leanCut differs from JS (libm sin)");
    }

    Rot rot;
    for (int i = 0; i < 9; i++) rot[i] = J["rot"][i].get<double>();
    Topology topo = buildTopology(J["pos"].get<std::vector<double>>());
    {
        const auto jsArea = J["topoArea"].get<std::vector<double>>();
        const auto jsNrm = J["topoNrm"].get<std::vector<double>>();
        if (jsArea.size() != topo.area.size() || jsNrm.size() != topo.nrm.size()) { fail("topology size differs"); return false; }
        for (size_t i = 0; i < jsArea.size(); i++) g_topoAreaDiff += jsArea[i] != topo.area[i];
        for (size_t i = 0; i < jsNrm.size(); i++) g_topoNrmDiff += jsNrm[i] != topo.nrm[i];
        if (!g_ownTopology) { topo.area = jsArea; topo.nrm = jsNrm; }
    }
    const AnalyzeResult result = analyze(topo, DEFAULT_THRESHOLD, rot);
    checkD(result.offset.x, J["offset"]["x"], "offset.x");
    checkD(result.offset.y, J["offset"]["y"], "offset.y");
    checkD(result.offset.z, J["offset"]["z"], "offset.z");
    {   // the patch bearings buildSwayBraces sorts faces apart by (libm atan2 vs fdlibm)
        const std::vector<WallPatch> patches = findWallPatches(topo, rot, result.offset);
        checkI(long(patches.size()), long(J["bearings"].size()), "patch count");
        for (size_t k = 0; k < std::min(patches.size(), J["bearings"].size()); k++) {
            const double b = std::atan2(patches[k].n.y, patches[k].n.x);
            if (b != J["bearings"][k].get<double>()) g_bearingDiff++;
            checkD(b, J["bearings"][k], "bearing[" + std::to_string(k) + "]");
        }
    }
    const auto walls = wallsOf(J["walls"]);

    // buildSwayBraces, every variant
    std::vector<SwayRib> autoRibs;
    std::string summary;
    static const std::vector<std::string> kResKeys = {"triangles", "count", "tines", "skipped", "ribs", "reason"};
    for (const json &run : J["runs"]) {
        const std::string name = run["name"].get<std::string>();
        const std::string P = "run " + name;
        const json &o = run["out"];
        if (o["keys"].get<std::vector<std::string>>() != kResKeys) fail(P + ": upstream result keys changed: " + o["keys"].dump());
        const SwayBraces s = buildSwayBraces(topo, result, rot, optsOf(run["opts"]));
        checkI(s.count, o["count"].get<int>(), P + ".count");
        checkI(s.tines, o["tines"].get<int>(), P + ".tines");
        checkI(s.skipped, o["skipped"].get<int>(), P + ".skipped");
        checkS(s.reason, o["reason"], P + ".reason");
        checkI(long(s.ribs.size()), long(o["ribs"].size()), P + ".ribs.size");
        checkTris(s.triangles, s.tags, o["triangles"], o["tags"], P + ".triangles");
        for (size_t k = 0; k < std::min(s.ribs.size(), o["ribs"].size()); k++) {
            const SwayRib &r = s.ribs[k];
            checkRib(r, o["ribs"][k], P + ".ribs[" + std::to_string(k) + "]", false);
            // the rib's own tris are its triRange slice of `triangles`
            if (r.triRange && r.triRange->second <= int(s.triangles.size())) {
                bool same = r.triRange->second - r.triRange->first == int(r.tris.size());
                for (int i = 0; same && i < int(r.tris.size()); i++)
                    same = r.tris[i] == s.triangles[r.triRange->first + i] && r.tags[i] == s.tags[r.triRange->first + i];
                if (!same) fail(P + ".ribs[" + std::to_string(k) + "]: tris != triRange slice");
            }
        }
        if (name == "default") autoRibs = s.ribs;
        summary += " " + name + ":" + std::to_string(s.count) + "/" + std::to_string(s.tines);
    }

    // swayClashes / swayClashesWall on the Auto ribs
    for (const json &c : J["clashes"]) {
        const int i = c["i"].get<int>(), j = c["j"].get<int>();
        if (i >= int(autoRibs.size()) || j >= int(autoRibs.size())) { fail("clash index out of range"); continue; }
        checkB(swayClashes(autoRibs[i], {autoRibs[j]}), c["v"].get<bool>(),
               "swayClashes(" + std::to_string(i) + ", [" + std::to_string(j) + "])");
    }
    {
        const auto shifts = J["shifts"].get<std::vector<double>>();
        const json &lc = J["levelClashes"];
        size_t n = 0;
        for (size_t i = 0; i < autoRibs.size(); i++)
            for (size_t j = 0; j < autoRibs.size(); j++)
                for (double dx : shifts) for (double dy : shifts) {
                    const SwayRib &r = autoRibs[j];
                    SwayRib copy;
                    copy.foot = {Vec3d(1e4, 1e4, 0), Vec3d(1e4 + 1, 1e4, 0)};
                    copy.halfW = r.halfW; copy.th = r.th; copy.height = r.height * 0.73;
                    for (const SwayLevel &L : r.levels)
                        copy.levels.push_back({L.z * 0.73, Vec3d(L.a[0] + dx, L.a[1] + dy, L.a[2]), Vec3d(L.b[0] + dx, L.b[1] + dy, L.b[2])});
                    if (n >= lc.size()) { fail("levelClashes: too few in dump"); goto doneLevels; }
                    checkB(swayClashes(autoRibs[i], {copy}), lc[n].get<int>() != 0,
                           "levelClash[" + std::to_string(i) + "," + std::to_string(j) + "," + std::to_string(dx) + "," + std::to_string(dy) + "]");
                    n++;
                }
        checkI(long(n), long(lc.size()), "levelClashes count");
    doneLevels:;
    }
    checkI(long(J["clashAll"].size()), long(autoRibs.size()), "clashAll size");
    for (size_t i = 0; i < std::min(autoRibs.size(), J["clashAll"].size()); i++) {
        std::vector<SwayRib> others;
        for (size_t k = 0; k < autoRibs.size(); k++) if (k != i) others.push_back(autoRibs[k]);
        checkB(swayClashes(autoRibs[i], others), J["clashAll"][i].get<bool>(), "clashAll[" + std::to_string(i) + "]");
        checkB(swayClashesWall(autoRibs[i], walls), J["clashWall"][i].get<bool>(), "clashWall[" + std::to_string(i) + "]");
        for (size_t w = 0; w < walls.size(); w++)
            checkB(swayClashesWall(autoRibs[i], {walls[w]}), J["clashWallEach"][i][w].get<bool>(),
                   "clashWallEach[" + std::to_string(i) + "][" + std::to_string(w) + "]");
    }

    // faceIsUpright on every face
    {
        const json &U = J["upright"];
        checkI(long(U.size()), topo.nFaces, "upright size");
        long up = 0;
        for (int f = 0; f < std::min(topo.nFaces, int(U.size())); f++) {
            const bool u = faceIsUpright(topo, rot, f);
            up += u;
            checkB(u, U[f].get<int>() != 0, "faceIsUpright(" + std::to_string(f) + ")");
        }
        summary += "  upright " + std::to_string(up) + "/" + std::to_string(topo.nFaces);
    }

    // swayAtFace, Draw-mode picks
    SwayPatchCache cache;
    int okPicks = 0;
    for (size_t k = 0; k < J["atFace"].size(); k++) {
        const json &a = J["atFace"][k];
        const std::string P = "atFace[" + std::to_string(k) + "]";
        SwayAvoid avoid;
        const std::string av = a["avoid"].get<std::string>();
        if (av == "auto") avoid.braces = autoRibs;
        else if (av == "walls") avoid.walls = walls;
        const Vec3d point(num(a["point"][0]), num(a["point"][1]), num(a["point"][2]));
        const SwayRib r = swayAtFace(topo, result, rot, a["face"].get<int>(), point, optsOf(a["opts"]), avoid,
                                     k % 2 ? nullptr : &cache);
        checkRib(r, a["r"], P, true);
        okPicks += r.ok;
        if (r.ok) {
            checkB(swayClashes(r, autoRibs), a["vsAuto"].get<bool>(), P + " vs Auto braces");
            checkB(swayClashesWall(r, walls), a["vsWalls"].get<bool>(), P + " vs walls");
        }
    }
    summary += "  atFace ok " + std::to_string(okPicks) + "/" + std::to_string(J["atFace"].size());

    const bool ok = g_fail == failBefore;
    std::printf("%-22s%s  %s\n", g_ctx.c_str(), summary.c_str(), ok ? "ok" : "FAIL");
    return ok;
}

} // namespace

int main(int argc, char **argv) {
    fs::path dir = fs::path(__FILE__).parent_path() / ".." / "refs_sway";
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--own-topology") g_ownTopology = true;
        else if (std::string(argv[i]) == "--verbose") g_verbose = true;
        else dir = argv[i];
    }
    std::vector<fs::path> files;
    std::error_code ec;
    for (const auto &e : fs::directory_iterator(dir, ec))
        if (e.path().extension() == ".json") files.push_back(e.path());
    if (files.empty()) {
        std::printf("FAIL: no dumps in %s -- generate them with tools/sway_ref.mjs (see header)\n", dir.string().c_str());
        return 1;
    }
    std::sort(files.begin(), files.end());
    int bad = 0;
    for (const auto &f : files) if (!runDump(f)) bad++;
    std::printf("%zu dumps, %d failed; %ld checks, %ld doubles compared, %ld not bit-identical, max rel err %.3g\n",
                files.size(), bad, g_checks, g_doubles, g_inexact, g_maxErr);
    std::printf("topology: %s; C++ buildTopology vs JS: %ld face areas, %ld normal components not bit-identical; "
                "%ld patch bearings (atan2) not bit-identical\n",
                g_ownTopology ? "C++ buildTopology" : "JS area/nrm fed in", g_topoAreaDiff, g_topoNrmDiff, g_bearingDiff);
    return bad == 0 && g_fail == 0 ? 0 : 1;
}
