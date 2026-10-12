// Pins Engine/Cutout (web/cutout.js) against the upstream JS. The dumps come from
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/cutout_ref.mjs
// (refs_cutout/holesFor.json, refs_cutout/cutWall.json; gitignored). holesFor must
// match exactly; cutWall must return the same flag and emit the same triangles in
// the same order, every coordinate within 1e-9.
#include "Engine/Cutout.hpp"
#include "nlohmann/json.hpp"
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

using namespace Slic3r::SupportFins;
using nlohmann::json;
namespace fs = std::filesystem;

static int failures = 0;
#define FAIL(...) do { std::fprintf(stderr, __VA_ARGS__); std::fputc('\n', stderr); failures++; } while (0)

static json load(const fs::path &p) {
    std::ifstream f(p);
    if (!f) { std::fprintf(stderr, "cannot open %s (run tools/cutout_ref.mjs)\n", p.string().c_str()); std::exit(2); }
    return json::parse(f);
}

static Vec3d v3(const json &a) { return Vec3d(a[0].get<double>(), a[1].get<double>(), a[2].get<double>()); }
static std::optional<double> opt(const json &q, const char *k) {
    if (q.contains(k) && !q[k].is_null()) return q[k].get<double>();
    return std::nullopt;
}

static void checkCut(const json &c) {
    // CUT's numbers must be upstream's defaults
    const CutConfig d;
    const std::pair<const char *, double> nums[] = {
        {"pitch", d.pitch}, {"web", d.web}, {"rail", d.rail}, {"post", d.post}, {"minHalf", d.minHalf},
        {"minHole", d.minHole}, {"slope", d.slope}, {"maxSlope", d.maxSlope}, {"eps", d.eps},
        {"latPitch", d.latPitch}, {"latStrut", d.latStrut}, {"latSlope", d.latSlope}, {"roofMin", d.roofMin},
        {"minArea", d.minArea}, {"minSaved", d.minSaved}};
    for (const auto &[k, v] : nums)
        if (c.at(k).get<double>() != v) FAIL("CUT.%s: C++ %.17g, JS %.17g", k, v, c.at(k).get<double>());
    if (c.size() != std::size(nums) + 1) FAIL("CUT has %zu keys in JS", c.size());
}

int main(int argc, char **argv) {
    const fs::path dir = argc > 1 ? fs::path(argv[1]) : fs::path(__FILE__).parent_path().parent_path() / "refs_cutout";

    // ---- holesFor + CUT + CUTOUT_PATTERNS
    const json hf = load(dir / "holesFor.json");
    checkCut(hf["CUT"]);
    const auto &pats = hf["CUTOUT_PATTERNS"];
    if (pats.size() != CUTOUT_PATTERNS.size()) FAIL("CUTOUT_PATTERNS size %zu vs %zu", CUTOUT_PATTERNS.size(), pats.size());
    else for (size_t i = 0; i < pats.size(); i++)
        if (pats[i].get<std::string>() != CUTOUT_PATTERNS[i]) FAIL("CUTOUT_PATTERNS[%zu]", i);
    size_t nHoles = 0;
    for (const auto &c : hf["cases"]) {
        const std::string kind = c["kind"];
        const double w = c["w"], H = c["H"];
        const auto got = holesFor(kind, w, H);
        const auto &exp = c["holes"];
        if (got.size() != exp.size()) { FAIL("holesFor(%s, %g, %g): %zu holes, JS %zu", kind.c_str(), w, H, got.size(), exp.size()); continue; }
        for (size_t i = 0; i < got.size(); i++) {
            nHoles++;
            if (got[i].a != exp[i]["a"].get<double>() || got[i].z0 != exp[i]["z0"].get<double>() || got[i].z1 != exp[i]["z1"].get<double>())
                FAIL("holesFor(%s, %g, %g)[%zu]: {%.17g %.17g %.17g} vs JS {%.17g %.17g %.17g}", kind.c_str(), w, H, i,
                     got[i].a, got[i].z0, got[i].z1, exp[i]["a"].get<double>(), exp[i]["z0"].get<double>(), exp[i]["z1"].get<double>());
        }
    }
    std::printf("holesFor: %zu cases, %zu holes\n", hf["cases"].size(), nHoles);

    // ---- cutWall
    const json cw = load(dir / "cutWall.json");
    checkCut(cw["CUT"]);
    size_t nCases = 0, nCut = 0, nTris = 0;
    double maxDev = 0;
    for (const auto &c : cw["cases"]) {
        const std::string name = c["name"];
        std::vector<CutStation> st;
        for (const auto &q : c["st"]) {
            CutStation s;
            s.p = v3(q["p"]);
            s.sx = q["sx"]; s.sy = q["sy"]; s.top = q["top"]; s.ztip = q["ztip"];
            s.holeTop = opt(q, "holeTop"); s.tipHalf = opt(q, "tipHalf");
            s.bot = q["bot"]; s.botTip = q["botTip"]; s.taperBot = q["taperBot"];
            s.botN = opt(q, "botN"); s.botP = opt(q, "botP");
            st.push_back(s);
        }
        std::vector<Section> full;
        for (const auto &sec : c["full"]) { Section S; for (const auto &v : sec) S.push_back(v3(v)); full.push_back(S); }
        const CutWallDims wall{c["wall"]["th"], c["wall"]["tip"], c["wall"]["minStations"]};
        CUT = CutConfig();
        CUT.pattern = c["pattern"];
        Tris out;
        const bool ret = cutWall(st, full, out, wall);
        const bool expRet = c["ret"];
        nCases++;
        if (ret != expRet) { FAIL("%s: cutWall returned %d, JS %d", name.c_str(), ret, expRet); continue; }
        if (ret) nCut++;
        const auto &tris = c["tris"];
        if (out.size() * 3 != tris.size()) { FAIL("%s: %zu triangles, JS %zu", name.c_str(), out.size() / 3, tris.size() / 9); continue; }
        nTris += out.size() / 3;
        size_t bad = 0;
        for (size_t i = 0; i < out.size(); i++)
            for (int k = 0; k < 3; k++) {
                const double dv = std::fabs(out[i][k] - tris[i * 3 + k].get<double>());
                if (!(dv <= 1e-9)) {
                    if (bad++ < 3) FAIL("%s: tri %zu vertex %zu[%d]: %.17g vs JS %.17g", name.c_str(), i / 3, i % 3, k, out[i][k], tris[i * 3 + k].get<double>());
                } else maxDev = std::max(maxDev, dv);
            }
    }
    std::printf("cutWall: %zu cases (%zu cut), %zu triangles, max |dev| %.3g\n", nCases, nCut, nTris, maxDev);

    // CUT.pattern as applyTunables sets it: only from CUTOUT_PATTERNS
    for (const char *p : CUTOUT_PATTERNS) if (!isCutoutPattern(p)) FAIL("isCutoutPattern(%s)", p);
    if (isCutoutPattern("bogus") || isCutoutPattern("")) FAIL("isCutoutPattern accepts junk");

    if (failures) { std::printf("test_cutout: %d FAILURES\n", failures); return 1; }
    std::printf("test_cutout: OK\n");
    return 0;
}
