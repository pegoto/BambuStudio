// Pins Engine/Planes (web/planes.js) against the JS oracle dumps written by
// tools/planes_ref.mjs:
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/planes_ref.mjs
//       $SUPPORT_FINS_DIR/prototype/stress/models/cube.stl 40,0,0 refs_planes/cube-x40.json
//   (the full list of dumps is in the header of tools/planes_ref.mjs)
//
// Usage: test_planes [dumpDir] [--own-topology]
//   dumpDir defaults to <SupportFins>/refs_planes, found via __FILE__.
//
// For every dump: rebuild topology + analyze from the same seated soup, then (unless
// --own-topology) replace the topology's per-face area/nrm with the JS values from the
// dump, so the test pins planes.js alone: buildTopology computes them with
// hypot3 (std::hypot), which is not bit-identical to V8's Math.hypot, and on a
// symmetric part (sphere) one ulp of face area reorders equal-area seeds. Then run
// findWallPatches with stats, and check same patch count and order, identical face
// lists, every scalar (and every tris value) within 1e-9 abs/rel, identical stats,
// plus faceAdjacency (exact), patchProbe / patchCovers / patchPoint / tAtZ / zAt on
// sample points, the planes constants, and the libm-derived cut constants (bitwise).
// Also counts how many compared doubles were not bit-identical (informational).
#include "Engine/Planes.hpp"
#include "nlohmann/json.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace Slic3r::SupportFins;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

int g_fail = 0;
long g_compared = 0, g_notBitExact = 0;
double g_maxErr = 0;
std::string g_ctx;
bool g_ownTopology = false;
long g_topoAreaDiff = 0, g_topoNrmDiff = 0;

void fail(const std::string &what) {
    if (++g_fail <= 40) std::printf("FAIL [%s] %s\n", g_ctx.c_str(), what.c_str());
}

bool num(const json &j, double &out) {
    if (j.is_number()) { out = j.get<double>(); return true; }
    return false;
}

void near(const std::string &name, double got, const json &jexp) {
    double exp;
    if (!num(jexp, exp)) { fail(name + ": expected value is not a number: " + jexp.dump()); return; }
    g_compared++;
    if (std::memcmp(&got, &exp, sizeof(double)) != 0 && !(got == 0 && exp == 0)) g_notBitExact++;
    const double err = std::abs(got - exp);
    const double rel = err / std::max(1.0, std::abs(exp));
    g_maxErr = std::max(g_maxErr, rel);
    if (!(rel <= 1e-9)) {
        char buf[256];
        std::snprintf(buf, sizeof buf, "%s: got %.17g expected %.17g", name.c_str(), got, exp);
        fail(buf);
    }
}

void checkStats(const PatchStats &s, const json &js) {
    const std::pair<const char *, int> got[] = {
        {"grown", s.grown}, {"tooSmall", s.tooSmall}, {"notUpright", s.notUpright},
        {"notFlat", s.notFlat}, {"tooShort", s.tooShort}, {"tooNarrow", s.tooNarrow}};
    for (auto it = js.begin(); it != js.end(); ++it) {
        bool known = false;
        for (auto &g : got) known |= it.key() == g.first;
        if (!known) fail("stats: unknown upstream key " + it.key());
    }
    for (auto &g : got) {
        const int exp = js.contains(g.first) ? js[g.first].get<int>() : 0;
        if (g.second != exp) fail(std::string("stats.") + g.first + ": got " + std::to_string(g.second) + " expected " + std::to_string(exp));
    }
}

bool runDump(const fs::path &file) {
    const int failBefore = g_fail;
    g_ctx = file.filename().string();
    std::ifstream in(file);
    json J = json::parse(in);

    // constants
    const json &C = J["constants"];
    near("MAX_LEAN_DEG", MAX_LEAN_DEG, C["MAX_LEAN_DEG"]);
    near("NORMAL_AGREE_DEG", NORMAL_AGREE_DEG, C["NORMAL_AGREE_DEG"]);
    near("FLAT_TOL_IN", FLAT_TOL_IN, C["FLAT_TOL_IN"]);
    near("GROW_SLACK", GROW_SLACK, C["GROW_SLACK"]);
    near("MIN_PATCH_H", MIN_PATCH_H, C["MIN_PATCH_H"]);
    near("MIN_PATCH_W", MIN_PATCH_W, C["MIN_PATCH_W"]);
    near("MIN_PATCH_AREA", MIN_PATCH_AREA, C["MIN_PATCH_AREA"]);
    // the cuts findWallPatches derives with Math.sin / Math.cos: must be bit-identical,
    // they decide which faces are upright / agree with their seed
    {
        const double leanCut = std::sin((MAX_LEAN_DEG * M_PI) / 180) + 1e-6;
        const double agreeCut = std::cos((NORMAL_AGREE_DEG * M_PI) / 180);
        if (leanCut != J["leanCut"].get<double>()) fail("leanCut differs from JS (libm sin)");
        if (agreeCut != J["agreeCut"].get<double>()) fail("agreeCut differs from JS (libm cos)");
    }

    Rot rot;
    for (int i = 0; i < 9; i++) rot[i] = J["rot"][i].get<double>();
    Soup pos = J["pos"].get<std::vector<double>>();
    Topology topo = buildTopology(std::move(pos));
    {   // informational: how far the C++ buildTopology is from the JS one
        const auto jsArea = J["topoArea"].get<std::vector<double>>();
        const auto jsNrm = J["topoNrm"].get<std::vector<double>>();
        if (jsArea.size() != topo.area.size() || jsNrm.size() != topo.nrm.size()) { fail("topology size differs"); return false; }
        for (size_t i = 0; i < jsArea.size(); i++) g_topoAreaDiff += jsArea[i] != topo.area[i];
        for (size_t i = 0; i < jsNrm.size(); i++) g_topoNrmDiff += jsNrm[i] != topo.nrm[i];
        if (!g_ownTopology) { topo.area = jsArea; topo.nrm = jsNrm; }
    }
    const AnalyzeResult result = analyze(topo, DEFAULT_THRESHOLD, rot);
    near("offset.x", result.offset.x, J["offset"]["x"]);
    near("offset.y", result.offset.y, J["offset"]["y"]);
    near("offset.z", result.offset.z, J["offset"]["z"]);

    // faceAdjacency
    {
        const FaceAdjacency adj = faceAdjacency(topo);
        if (adj.start != J["adjacency"]["start"].get<std::vector<int>>()) fail("faceAdjacency.start differs");
        if (adj.nbr != J["adjacency"]["nbr"].get<std::vector<int>>()) fail("faceAdjacency.nbr differs");
    }

    PatchStats stats;
    const std::vector<WallPatch> patches = findWallPatches(topo, rot, result.offset, &stats);
    checkStats(stats, J["stats"]);
    // and without stats: same patches
    if (findWallPatches(topo, rot, result.offset).size() != patches.size()) fail("stats-less call returns a different count");

    const json &JP = J["patches"];
    if (patches.size() != JP.size()) {
        fail("patch count: got " + std::to_string(patches.size()) + " expected " + std::to_string(JP.size()));
        return false;
    }
    static const std::vector<std::string> kKeys = {"faces", "area", "mid", "n", "u", "t", "h", "d", "u0", "u1",
                                                   "t0", "t1", "z0", "z1", "lean", "flatness", "tris"};
    for (size_t k = 0; k < patches.size(); k++) {
        const WallPatch &p = patches[k];
        const json &e = JP[k];
        const std::string P = "patch[" + std::to_string(k) + "].";
        if (e["keys"].get<std::vector<std::string>>() != kKeys) fail(P + "upstream patch keys changed: " + e["keys"].dump());
        if (p.faces != e["faces"].get<std::vector<int>>()) fail(P + "faces differ");
        near(P + "area", p.area, e["area"]);
        near(P + "mid.x", p.mid.x, e["mid"]["x"]);
        near(P + "mid.y", p.mid.y, e["mid"]["y"]);
        if (e["mid"].size() != 2) fail(P + "mid has extra keys");
        near(P + "n.x", p.n.x, e["n"]["x"]);
        near(P + "n.y", p.n.y, e["n"]["y"]);
        near(P + "n.z", p.n.z, e["n"]["z"]);
        near(P + "u.x", p.u.x, e["u"]["x"]);
        near(P + "u.y", p.u.y, e["u"]["y"]);
        if (e["u"].size() != 2) fail(P + "u has extra keys");
        near(P + "t.x", p.t.x, e["t"]["x"]);
        near(P + "t.y", p.t.y, e["t"]["y"]);
        near(P + "t.z", p.t.z, e["t"]["z"]);
        near(P + "h", p.h, e["h"]);
        near(P + "d", p.d, e["d"]);
        near(P + "u0", p.u0, e["u0"]);
        near(P + "u1", p.u1, e["u1"]);
        near(P + "t0", p.t0, e["t0"]);
        near(P + "t1", p.t1, e["t1"]);
        near(P + "z0", p.z0, e["z0"]);
        near(P + "z1", p.z1, e["z1"]);
        near(P + "lean", p.lean, e["lean"]);
        near(P + "flatness", p.flatness, e["flatness"]);
        const json &et = e["tris"];
        if (p.tris.size() != et.size()) fail(P + "tris size differs");
        else for (size_t i = 0; i < p.tris.size(); i++) near(P + "tris[" + std::to_string(i) + "]", p.tris[i], et[i]);

        // the other exports, on this patch
        const json &S = J["samples"][k];
        for (size_t i = 0; i < S["probes"].size(); i++) {
            const json &s = S["probes"][i];
            const double u = s["u"].get<double>(), t = s["t"].get<double>();
            const std::optional<double> pr = patchProbe(p, u, t);
            const std::string Q = P + "probe[" + std::to_string(i) + "]";
            if (pr.has_value() != !s["probe"].is_null()) fail(Q + ": coverage differs");
            else if (pr) near(Q, *pr, s["probe"]);
            if (patchCovers(p, u, t) != s["covers"].get<bool>()) fail(Q + ": patchCovers differs");
        }
        for (size_t i = 0; i < S["points"].size(); i++) {
            const json &s = S["points"][i];
            const Vec3d q = patchPoint(p, s["w"].get<double>(), s["u"].get<double>(), s["t"].get<double>());
            const std::string Q = P + "patchPoint[" + std::to_string(i) + "]";
            near(Q + ".x", q.x(), s["p"][0]);
            near(Q + ".y", q.y(), s["p"][1]);
            near(Q + ".z", q.z(), s["p"][2]);
        }
        for (size_t i = 0; i < S["tAtZ"].size(); i++) {
            const json &s = S["tAtZ"][i];
            near(P + "tAtZ[" + std::to_string(i) + "]", tAtZ(p, s["w"].get<double>(), s["z"].get<double>()), s["t"]);
        }
        for (size_t i = 0; i < S["zAt"].size(); i++) {
            const json &s = S["zAt"][i];
            near(P + "zAt[" + std::to_string(i) + "]", zAt(p, s["w"].get<double>(), s["t"].get<double>()), s["z"]);
        }
    }
    const bool ok = g_fail == failBefore;
    std::printf("%-28s %4zu patches  stats{grown %d, tooSmall %d, notUpright %d, notFlat %d, tooShort %d, tooNarrow %d}  %s\n",
                g_ctx.c_str(), patches.size(), stats.grown, stats.tooSmall, stats.notUpright, stats.notFlat,
                stats.tooShort, stats.tooNarrow, ok ? "ok" : "FAIL");
    return ok;
}

} // namespace

int main(int argc, char **argv) {
    fs::path dir = fs::path(__FILE__).parent_path() / ".." / "refs_planes";
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--own-topology") g_ownTopology = true;
        else dir = argv[i];
    }
    std::vector<fs::path> files;
    std::error_code ec;
    for (const auto &e : fs::directory_iterator(dir, ec))
        if (e.path().extension() == ".json") files.push_back(e.path());
    if (files.empty()) {
        std::printf("FAIL: no dumps in %s -- generate them with tools/planes_ref.mjs (see header)\n", dir.string().c_str());
        return 1;
    }
    std::sort(files.begin(), files.end());
    int bad = 0;
    for (const auto &f : files) if (!runDump(f)) bad++;
    std::printf("%zu dumps, %d failed; %ld doubles compared, %ld not bit-identical, max rel err %.3g\n",
                files.size(), bad, g_compared, g_notBitExact, g_maxErr);
    std::printf("topology: %s; C++ buildTopology vs JS: %ld face areas, %ld normal components not bit-identical\n",
                g_ownTopology ? "C++ buildTopology" : "JS area/nrm fed in", g_topoAreaDiff, g_topoNrmDiff);
    return bad == 0 && g_fail == 0 ? 0 : 1;
}
