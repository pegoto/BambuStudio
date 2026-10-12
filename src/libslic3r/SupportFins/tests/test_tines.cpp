// Parity test for Engine/Prop/Tines.* (web/prop/tines.js). Replays what
// tools/tines_ref.mjs recorded against the upstream JS:
//   calls   every emitTines call the real pipeline made (buildFins as computeFins calls
//           it: plate walls, part-attached walls, squat walls, fill's drawn walls,
//           wedges), with the caller's exact arguments and PROP state;
//   walls   every distinct wall top line replayed over tineDensity 0/.5/1 x layerHeight
//           .2/.12/.28 x tinesPerWall 0/5 x iface off/'all', plus the all-defaults call
//           shape, the squat call shape (brim minTop) and a frame turned about z;
//   stepfor tineStepFor over a sweep of densities.
// Counts, interface tags, the __TINECAP seam and grip are compared exactly, doubles
// within 1e-9 (and every double that is not bit-identical is counted).
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/tines_ref.mjs   # writes refs_tines/
//   ./build-sf/test_tines [refs_tines_dir]
#include "Engine/Entry.hpp"
#include "Engine/Overhangs.hpp"
#include "Engine/Prop/Tines.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

using namespace Slic3r::SupportFins;
using json = nlohmann::json;
namespace fs = std::filesystem;

static long g_checks = 0, g_fail = 0, g_inexact = 0;
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
static bool isUndef(const json &j) { return j.is_string() && j.get<std::string>() == "undefined"; }

static void fail(const std::string &what) {
    if (++g_fail <= 40) std::fprintf(stderr, "FAIL %s: %s\n", g_ctx.c_str(), what.c_str());
}
static void checkD(double got, double want, const char *what) {
    g_checks++;
    if (std::isnan(got) && std::isnan(want)) return;
    if (got == want) { if (std::signbit(got) != std::signbit(want)) g_inexact++; return; }
    g_inexact++;
    if (std::isfinite(got) && std::isfinite(want) && std::abs(got - want) <= 1e-9) return;
    char b[256];
    std::snprintf(b, sizeof b, "%s: got %.17g want %.17g", what, got, want);
    fail(b);
}
static void checkI(long got, long want, const char *what) {
    g_checks++;
    if (got != want) fail(std::string(what) + ": got " + std::to_string(got) + " want " + std::to_string(want));
}

// a JS numeric argument: "undefined" -> nullopt (the default); null is not expected
static std::optional<double> optNum(const json &j, const char *what) {
    if (isUndef(j)) return std::nullopt;
    if (j.is_null()) { fail(std::string(what) + ": null argument"); return std::nullopt; }
    return num(j);
}
static std::optional<TineBody> optBody(const json &j) {
    if (isUndef(j) || j.is_null()) return std::nullopt;
    return TineBody{j[0].get<int>(), j[1].get<int>()};
}
static std::vector<Vec3d> lineOf(const json &j) {
    std::vector<Vec3d> l;
    for (const json &p : j) l.emplace_back(num(p[0]), num(p[1]), num(p[2]));
    return l;
}
static Iface ifaceOf(const std::string &s) {
    return s == "all" ? Iface::All : s == "flat" ? Iface::Flat : Iface::Off;
}

// what tools/tines_ref.mjs' drive() does: `out` starts with one untagged triangle
struct Result {
    int n = 0;
    Tris out;
    IfaceTags tags;
    TineGrip grip;
    std::vector<TineCap> caps;
    bool threw = false;
};
static Result run(const std::vector<Vec3d> &line, const Topology &topo, const Rot &rot, const Offset &off,
                  std::optional<double> stepArg, std::optional<double> minTop, std::optional<double> tineH,
                  const std::optional<TineBody> &body, bool withGrip, bool withCaps, double gripStart = INF) {
    Result r;
    r.out = {Vec3d(1, 2, 3), Vec3d(4, 5, 6), Vec3d(7, 8, 9)};
    r.tags.assign(3, 0);
    r.grip.z = gripStart;
    TINECAP = withCaps ? &r.caps : nullptr;
    try {
        r.n = emitTines(line, nullptr, topo, rot, off, r.out, stepArg, minTop, tineH, body,
                        withGrip ? &r.grip : nullptr, &r.tags);
    } catch (const std::exception &e) {
        r.threw = true;
        fail(std::string("threw: ") + e.what());
    }
    TINECAP = nullptr;
    return r;
}

static void checkTris(const Result &r, const json &want) {
    checkI(long(r.out.size() - 3) * 3, long(want.size()), "tris length");
    if ((r.out.size() - 3) * 3 != want.size()) return;
    for (size_t i = 3; i < r.out.size(); i++)
        for (int k = 0; k < 3; k++) checkD(r.out[i][k], num(want[(i - 3) * 3 + k]), "tri coord");
}
static void checkTags(const Result &r, const json &want) {
    IfaceTags t = r.tags;
    if (t.size() < r.out.size()) t.resize(r.out.size(), 0);
    for (int i = 0; i < 3; i++) checkI(t[i], 0, "tag before from");
    std::vector<long> runs;
    int cur = 0; long len = 0;
    for (size_t i = 3; i < r.out.size(); i++) {
        const int v = t[i] ? 1 : 0;
        if (v != cur) { runs.push_back(len); cur = v; len = 0; }
        len++;
    }
    runs.push_back(len);
    checkI(long(runs.size()), long(want.size()), "tag runs");
    if (runs.size() == want.size())
        for (size_t i = 0; i < runs.size(); i++) checkI(runs[i], want[i].get<long>(), "tag run length");
}
static void checkCaps(const Result &r, const json &want) {
    checkI(long(r.caps.size()), long(want.size()), "__TINECAP length");
    if (r.caps.size() != want.size()) return;
    for (size_t i = 0; i < r.caps.size(); i++) {
        const TineCap &c = r.caps[i];
        checkD(c.x, num(want[i][0]), "cap.x"); checkD(c.y, num(want[i][1]), "cap.y"); checkD(c.z, num(want[i][2]), "cap.z");
        checkD(c.biteX, num(want[i][3]), "cap.biteX"); checkD(c.biteY, num(want[i][4]), "cap.biteY");
    }
}

int main(int argc, char **argv) {
    const fs::path dir = argc > 1 ? fs::path(argv[1])
                                  : fs::path(__FILE__).parent_path().parent_path() / "refs_tines";
    if (!fs::is_directory(dir)) {
        std::fprintf(stderr, "no dumps in %s: run SUPPORT_FINS_DIR=... node tools/tines_ref.mjs\n", dir.string().c_str());
        return 1;
    }
    const PropConfig PROP0 = PROP;

    // ---- tineStepFor
    {
        std::ifstream in(dir / "stepfor.json");
        if (!in) { std::fprintf(stderr, "no stepfor.json in %s\n", dir.string().c_str()); return 1; }
        const json S = json::parse(in);
        long n = 0;
        for (const json &e : S) {
            PROP = PROP0;
            PROP.tineStep = num(e["tineStep"]); PROP.tineStepSparse = num(e["tineStepSparse"]);
            const json &d = e["density"];
            g_ctx = "tineStepFor(" + d.dump() + ")";
            const std::optional<double> dens = (isUndef(d) || d.is_null()) ? std::nullopt : std::optional<double>(num(d));
            checkD(tineStepFor(dens), num(e["out"]), "tineStepFor");
            n++;
        }
        PROP = PROP0;
        std::printf("tineStepFor %ld inputs  %s\n", n, g_fail ? "FAIL" : "ok");
    }

    std::vector<fs::path> files;
    for (const auto &e : fs::directory_iterator(dir))
        if (e.path().extension() == ".json" && e.path().filename() != "stepfor.json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::fprintf(stderr, "no dumps in %s\n", dir.string().c_str()); return 1; }

    long totCalls = 0, totRuns = 0, totTines = 0;
    for (const fs::path &file : files) {
        std::ifstream in(file);
        const json J = json::parse(in);
        const std::string scene = J["model"].get<std::string>() + " " + J["rot"].get<std::string>();
        const long failBefore = g_fail, checksBefore = g_checks;

        // exactly as computeFins: seatSoup, buildTopology, analyze(..., IDENTITY3)
        Soup posed;
        for (const json &v : J["soup"]) posed.push_back(num(v));
        Soup pos;
        seatSoup(posed, pos);
        Topology topo = buildTopology(std::move(pos));
        const AnalyzeResult res = analyze(topo, DEFAULT_THRESHOLD, IDENTITY3);
        const Offset off{num(J["offset"]["x"]), num(J["offset"]["y"]), num(J["offset"]["z"])};
        g_ctx = scene + " offset";
        checkD(res.offset.x, off.x, "offset.x"); checkD(res.offset.y, off.y, "offset.y"); checkD(res.offset.z, off.z, "offset.z");
        Rot rotZ;
        for (int i = 0; i < 9; i++) rotZ[i] = num(J["rotZ"][i]);
        const Offset offZ{num(J["offsetZ"]["x"]), num(J["offsetZ"]["y"]), num(J["offsetZ"]["z"])};
        {
            const AnalyzeResult rz = analyze(topo, DEFAULT_THRESHOLD, rotZ);
            g_ctx = scene + " offsetZ";
            checkD(rz.offset.x, offZ.x, "offsetZ.x"); checkD(rz.offset.y, offZ.y, "offsetZ.y"); checkD(rz.offset.z, offZ.z, "offsetZ.z");
        }

        // ---- the pipeline's own calls
        long nCalls = 0, nCallTines = 0;
        for (const json &C : J["calls"]) {
            g_ctx = scene + " call #" + std::to_string(nCalls) + " (" + C["src"].get<std::string>() + ")";
            Rot rot;
            for (int i = 0; i < 9; i++) rot[i] = num(C["rot"][i]);
            const Offset coff{num(C["offset"]["x"]), num(C["offset"]["y"]), num(C["offset"]["z"])};
            checkD(coff.x, off.x, "call offset.x"); checkD(coff.y, off.y, "call offset.y"); checkD(coff.z, off.z, "call offset.z");
            PROP = PROP0;
            PROP.gap = num(C["prop"]["gap"]);
            PROP.tinesPerWall = C["prop"]["tinesPerWall"].get<int>();
            PROP.iface = ifaceOf(C["prop"]["iface"]);
            PROP.ifaceLayers = C["prop"]["ifaceLayers"].get<int>();
            const json &G = C["grip"];
            const bool withGrip = !(isUndef(G) || G.is_null());
            const Result r = run(lineOf(C["line"]), topo, rot, coff, optNum(C["stepArg"], "stepArg"),
                                 optNum(C["minTop"], "minTop"), optNum(C["tineH"], "tineH"), optBody(C["body"]),
                                 withGrip, false, withGrip ? num(G) : INF);
            checkI(r.n, C["n"].get<int>(), "count");
            checkTris(r, C["tris"]);
            checkTags(r, C["tags"]);
            if (withGrip) checkD(r.grip.z, num(C["gripAfter"]), "grip.z");
            nCalls++; nCallTines += r.n;
        }

        // ---- replays of every distinct wall
        long nRuns = 0, nRunTines = 0, nWall = 0;
        for (const json &W : J["walls"]) {
            const std::string wctx = scene + " wall #" + std::to_string(nWall++) + " (" + W["src"].get<std::string>() + ")";
            const std::vector<Vec3d> line = lineOf(W["line"]);
            const double gap = num(W["gap"]);
            const std::optional<double> minTop = optNum(W["minTop"], "minTop");
            const std::optional<TineBody> body = optBody(W["body"]);
            for (const json &R : W["runs"]) {
                g_ctx = wctx + " density=" + R["density"].dump() + " tineH=" + R["tineH"].dump() + " tpw=" + R["tpw"].dump();
                const double stepArg = tineStepFor(num(R["density"]));
                for (const bool all : {false, true}) {
                    PROP = PROP0;
                    PROP.gap = gap;
                    PROP.tinesPerWall = R["tpw"].get<int>();
                    PROP.iface = all ? Iface::All : Iface::Off;
                    const Result r = run(line, topo, IDENTITY3, off, stepArg, minTop, num(R["tineH"]), body, true, true);
                    checkI(r.n, R["n"].get<int>(), all ? "count (iface all)" : "count");
                    checkTris(r, R["tris"]);
                    checkTags(r, all ? R["tagsAll"] : R["tagsOff"]);
                    checkD(r.grip.z, num(R["grip"]), "grip.z");
                    checkCaps(r, R["caps"]);
                    nRuns++; nRunTines += r.n;
                }
            }
            PROP = PROP0;
            PROP.gap = gap;
            {   // the six-argument call (wedges.js, tests): every default
                g_ctx = wctx + " defaults";
                const Result r = run(line, topo, IDENTITY3, off, std::nullopt, std::nullopt, std::nullopt, std::nullopt, false, true);
                checkI(r.n, W["dflt"]["n"].get<int>(), "count");
                checkTris(r, W["dflt"]["tris"]);
                checkCaps(r, W["dflt"]["caps"]);
                nRuns++; nRunTines += r.n;
            }
            {   // the squat call shape
                g_ctx = wctx + " squat";
                const Result r = run(line, topo, IDENTITY3, off, tineStepFor(0.0), PROP.squatBrimH, 0.2, std::nullopt, true, true);
                checkI(r.n, W["squat"]["n"].get<int>(), "count");
                checkTris(r, W["squat"]["tris"]);
                checkD(r.grip.z, num(W["squat"]["grip"]), "grip.z");
                checkCaps(r, W["squat"]["caps"]);
                nRuns++; nRunTines += r.n;
            }
            {   // turned about z: non-identity rot
                g_ctx = wctx + " rotated";
                const json &Z = W["rotated"];
                const Result r = run(lineOf(Z["line"]), topo, rotZ, offZ, tineStepFor(1.0), minTop, 0.2, body, true, true);
                checkI(r.n, Z["n"].get<int>(), "count");
                checkTris(r, Z["tris"]);
                checkD(r.grip.z, num(Z["grip"]), "grip.z");
                checkCaps(r, Z["caps"]);
                nRuns++; nRunTines += r.n;
            }
            PROP = PROP0;
        }
        std::printf("%-20s faces=%5d  calls %3ld (%4ld tines)  walls %3ld  replays %5ld (%6ld tines)  checks %9ld  %s\n",
                    scene.c_str(), topo.nFaces, nCalls, nCallTines, nWall, nRuns, nRunTines, g_checks - checksBefore,
                    g_fail == failBefore ? "ok" : "FAIL");
        totCalls += nCalls; totRuns += nRuns; totTines += nCallTines + nRunTines;
    }
    std::printf("%ld pipeline calls, %ld replays, %ld tines; %ld checks, %ld failures, %ld doubles not bit-identical (tolerance 1e-9)\n",
                totCalls, totRuns, totTines, g_checks, g_fail, g_inexact);
    return g_fail ? 1 : 0;
}
