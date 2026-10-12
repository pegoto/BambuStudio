// Parity test for Engine/Inside.* (web/inside.js) and Engine/Kiss.* (web/kiss.js).
// Replays, call for call and in the same order, what tools/inside_ref.mjs ran against
// the upstream JS, and compares: booleans and the generation counter exactly, doubles
// within 1e-9 (and counts how many are not bit-identical).
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/inside_ref.mjs   # writes refs_inside/
//   ./build-sf/test_inside [refs_inside_dir]
#include "Engine/Entry.hpp"
#include "Engine/Inside.hpp"
#include "Engine/Kiss.hpp"
#include "Engine/Overhangs.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
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

static void checkB(bool got, bool want, const char *what) {
    g_checks++;
    if (got != want) fail(std::string(what) + ": got " + (got ? "true" : "false") + " want " + (want ? "true" : "false"));
}

static void checkI(int64_t got, int64_t want, const char *what) {
    g_checks++;
    if (got != want) fail(std::string(what) + ": got " + std::to_string(got) + " want " + std::to_string(want));
}

static Vec3d vec(const json &p) { return Vec3d(num(p[0]), num(p[1]), num(p[2])); }

int main(int argc, char **argv) {
    const fs::path dir = argc > 1 ? fs::path(argv[1])
                                  : fs::path(__FILE__).parent_path().parent_path() / "refs_inside";
    if (!fs::is_directory(dir)) {
        std::fprintf(stderr, "no dumps in %s: run SUPPORT_FINS_DIR=... node tools/inside_ref.mjs\n", dir.string().c_str());
        return 1;
    }
    std::vector<fs::path> files;
    for (const auto &e : fs::directory_iterator(dir)) if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::fprintf(stderr, "no dumps in %s\n", dir.string().c_str()); return 1; }

    for (const fs::path &file : files) {
        std::ifstream in(file);
        const json J = json::parse(in);
        const std::string model = J["model"];
        const long failBefore = g_fail, checksBefore = g_checks;

        // exactly as computeFins: seatSoup, buildTopology, analyze(..., IDENTITY3)
        Soup posed;
        for (const json &v : J["soup"]) posed.push_back(num(v));
        Soup pos;
        seatSoup(posed, pos);
        Topology topo = buildTopology(std::move(pos));
        const AnalyzeResult res = analyze(topo, DEFAULT_THRESHOLD, IDENTITY3);

        long nIn = 0, nNear = 0, nKiss = 0, nSolid = 0, nPierce = 0;
        for (const json &F : J["frames"]) {
            const std::string fname = F["name"];
            Rot rot;
            for (int i = 0; i < 9; i++) rot[i] = num(F["rot"][i]);
            const Offset off{num(F["offset"]["x"]), num(F["offset"]["y"]), num(F["offset"]["z"])};
            g_ctx = model + "/" + fname + " offset";
            if (fname == "seated") {
                checkD(res.offset.x, off.x, "offset.x"); checkD(res.offset.y, off.y, "offset.y"); checkD(res.offset.z, off.z, "offset.z");
            } else {
                const AnalyzeResult r2 = analyze(topo, DEFAULT_THRESHOLD, rot);
                checkD(r2.offset.x, off.x, "offset.x"); checkD(r2.offset.y, off.y, "offset.y"); checkD(r2.offset.z, off.z, "offset.z");
            }

            const json &P = F["points"];
            std::vector<Vec3d> pts;
            for (const json &p : P) pts.push_back(vec(p));

            // insidePart
            for (size_t i = 0; i < pts.size(); i++) {
                g_ctx = model + "/" + fname + " insidePart #" + std::to_string(i);
                checkB(insidePart(topo, rot, off, pts[i].x(), pts[i].y(), pts[i].z()), F["inside"][i].get<bool>(), "inside");
                nIn++;
            }

            // nearestPart, default rMax and a few others
            for (const json &N : F["nearest"]) {
                const bool dflt = N["rMax"].is_null();
                const double rMax = dflt ? 0 : num(N["rMax"]);
                for (size_t i = 0; i < pts.size(); i++) {
                    g_ctx = model + "/" + fname + " nearestPart rMax=" + (dflt ? std::string("default") : std::to_string(rMax)) + " #" + std::to_string(i);
                    const auto h = dflt ? nearestPart(topo, rot, off, pts[i].x(), pts[i].y(), pts[i].z())
                                        : nearestPart(topo, rot, off, pts[i].x(), pts[i].y(), pts[i].z(), rMax);
                    const json &w = N["res"][i];
                    checkB(bool(h), !w.is_null(), "found");
                    if (h && !w.is_null()) { checkD(h->d, num(w[0]), "d"); checkD(h->cosUp, num(w[1]), "cosUp"); }
                    nNear++;
                }
            }

            // kissEnds
            for (size_t i = 0; i < F["kiss"].size(); i++) {
                const json &K = F["kiss"][i];
                g_ctx = model + "/" + fname + " kissEnds #" + std::to_string(i);
                const json &a = K["args"];
                const KissEnds e = kissEnds(topo, rot, off, num(a[0]), num(a[1]), num(a[2]), num(a[3]), num(a[4]), num(a[5]),
                                            KissOpts{num(a[6]), num(a[7]), num(a[8])});
                checkD(e.bot[0], num(K["out"][0]), "bot.left");
                checkD(e.bot[1], num(K["out"][1]), "bot.right");
                checkD(e.top[0], num(K["out"][2]), "top.left");
                checkD(e.top[1], num(K["out"][3]), "top.right");
                nKiss++;
            }

            // solidClearance, in the JS's call order (the stamp generation carries over)
            for (size_t i = 0; i < F["solids"].size(); i++) {
                const json &S = F["solids"][i];
                g_ctx = model + "/" + fname + " solidClearance #" + std::to_string(i);
                Tris verts;
                for (const json &v : S["verts"]) verts.push_back(vec(v));
                if (!S["setGen"].is_null()) topo._scGen = S["setGen"].get<int64_t>();
                const auto h = S["rMax"].is_null() ? solidClearance(topo, rot, off, verts)
                                                   : solidClearance(topo, rot, off, verts, num(S["rMax"]));
                const json &w = S["res"];
                checkB(bool(h), !w.is_null(), "found");
                if (h && !w.is_null()) {
                    checkD(h->d, num(w[0]), "d"); checkD(h->cosUp, num(w[1]), "cosUp");
                    checkD(h->x, num(w[2]), "x"); checkD(h->y, num(w[3]), "y"); checkD(h->z, num(w[4]), "z");
                    if (num(w[0]) == 0) nPierce++;
                }
                checkI(topo._scGen, S["genAfter"].get<int64_t>(), "_scGen after");
                nSolid++;
            }
        }
        std::printf("%-10s faces=%5d  insidePart %5ld  nearestPart %6ld  kissEnds %5ld  solidClearance %4ld (%ld touching)  checks %7ld  %s\n",
                    model.c_str(), topo.nFaces, nIn, nNear, nKiss, nSolid, nPierce, g_checks - checksBefore,
                    g_fail == failBefore ? "ok" : "FAIL");
    }
    std::printf("%ld checks, %ld failures, %ld doubles not bit-identical (tolerance 1e-9)\n", g_checks, g_fail, g_inexact);
    return g_fail ? 1 : 0;
}
