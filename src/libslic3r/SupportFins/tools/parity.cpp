// Parity runner: feeds the oracle dumps (tools/ref.mjs) through the C++ entry and
// compares. Exit 0 when every dump passes.
//
//   sf_parity [--verbose] [--tol 1e-3] ref1.json ref2.json ...
//
// What is compared, per dump:
//   overFaces / smallFaces   exact (analysis parity)
//   stats                    exact counts (regions, walls, tines, unserved, floating, pad/fin/sway tri counts)
//   pieces                   same count, kinds, ranges
//   triangles                ordered: vertex-by-vertex within tol; and unordered: multiset of
//                            canonical triangles within tol (reported separately, so a reorder
//                            is visible as "unordered 100% / ordered 40%")
#include "Engine/Entry.hpp"
#include "nlohmann/json.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>

using json = nlohmann::json;
using namespace Slic3r::SupportFins;

static EntryOptions optionsFrom(const json &o) {
    EntryOptions e;
    auto get = [&](const char *k) -> const json * { auto it = o.find(k); return it == o.end() || it->is_null() ? nullptr : &*it; };
    if (auto v = get("mode")) e.mode = v->get<std::string>();
    if (auto v = get("bedPad")) e.bedPad = v->get<bool>();
    if (auto v = get("tines")) e.tines = v->get<bool>();
    if (auto v = get("tineDensity")) e.tineDensity = v->get<double>();
    if (auto v = get("coverage")) e.coverage = v->get<double>();
    if (auto v = get("plateOnly")) e.plateOnly = v->get<bool>();
    if (auto v = get("layerHeight")) e.layerHeight = v->get<double>();
    if (auto v = get("threshold")) e.threshold = v->get<double>();
    if (auto v = get("material")) e.material = v->get<std::string>();
    if (auto v = get("padStyle")) e.padStyle = v->get<std::string>();
    if (auto v = get("cutout")) e.cutout = v->get<std::string>();
    if (auto v = get("sway")) {
        SwayOpts s;
        if (v->contains("on")) s.on = (*v)["on"].get<bool>();
        if (v->contains("gripFrom")) s.gripFrom = (*v)["gripFrom"].get<double>();
        if (v->contains("tineSpacing")) s.tineSpacing = (*v)["tineSpacing"].get<double>();
        if (v->contains("reach")) s.reach = (*v)["reach"].get<double>();
        if (v->contains("gap")) s.gap = (*v)["gap"].get<double>();
        e.sway = s;
    }
    return e;
}

struct Cmp { int fails = 0; bool verbose = false; const char *name = ""; };
template<class A, class B> static void expect_eq(Cmp &c, const char *what, const A &a, const B &b) {
    if (!(a == b)) { c.fails++; std::printf("  [%s] %s: ref=%s got=%s\n", c.name, what, std::to_string(b).c_str(), std::to_string(a).c_str()); }
}

int main(int argc, char **argv) {
    bool verbose = false; double tol = 1e-3; std::vector<std::string> files;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--verbose") verbose = true;
        else if (a == "--tol") tol = std::atof(argv[++i]);
        else files.push_back(a);
    }
    if (files.empty()) { std::fprintf(stderr, "usage: sf_parity [--verbose] [--tol t] ref.json...\n"); return 2; }
    int failedFiles = 0;
    for (const std::string &file : files) {
        std::ifstream in(file);
        if (!in) { std::printf("%s: cannot open\n", file.c_str()); failedFiles++; continue; }
        json ref = json::parse(in);
        Soup soup = ref["soup"].get<Soup>();
        EntryOptions opts = optionsFrom(ref["options"]);
        auto t0 = std::chrono::steady_clock::now();
        EntryResult r;
        try { r = computeFins(soup, opts); }
        catch (const std::exception &e) { std::printf("%s: threw %s\n", file.c_str(), e.what()); failedFiles++; continue; }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        Cmp c; c.verbose = verbose; c.name = file.c_str();

        expect_eq(c, "overFaces.size", r.overFaces.size(), ref["overFaces"].size());
        expect_eq(c, "smallFaces.size", r.smallFaces.size(), ref["smallFaces"].size());
        if (r.overFaces.size() == ref["overFaces"].size() && r.overFaces != ref["overFaces"].get<std::vector<int>>()) { c.fails++; std::printf("  [%s] overFaces differ\n", c.name); }
        if (r.smallFaces.size() == ref["smallFaces"].size() && r.smallFaces != ref["smallFaces"].get<std::vector<int>>()) { c.fails++; std::printf("  [%s] smallFaces differ\n", c.name); }
        const json &s = ref["stats"];
        expect_eq(c, "stats.overhangRegions", r.stats.overhangRegions, s["overhangRegions"].get<int>());
        expect_eq(c, "stats.braces", r.stats.braces, s["braces"].get<int>());
        expect_eq(c, "stats.props", r.stats.props, s["props"].get<int>());
        expect_eq(c, "stats.tines", r.stats.tines, s["tines"].get<int>());
        expect_eq(c, "stats.unserved", r.stats.unserved, s["unserved"].is_null() ? 0 : s["unserved"].get<int>());
        expect_eq(c, "stats.floating", r.stats.floating, s["floating"].get<int>());
        expect_eq(c, "stats.swayBraces", r.stats.swayBraces, s["swayBraces"].get<int>());
        expect_eq(c, "stats.swayTines", r.stats.swayTines, s["swayTines"].get<int>());
        expect_eq(c, "stats.finTriangles", r.stats.finTriangles, s["finTriangles"].get<int>());
        expect_eq(c, "stats.swayTriangles", r.stats.swayTriangles, s["swayTriangles"].get<int>());
        expect_eq(c, "stats.padTriangles", r.stats.padTriangles, s["padTriangles"].get<int>());
        const double offTol = 1e-6;
        if (std::abs(r.offset.x - ref["offset"]["x"].get<double>()) > offTol || std::abs(r.offset.y - ref["offset"]["y"].get<double>()) > offTol || std::abs(r.offset.z - ref["offset"]["z"].get<double>()) > offTol) {
            c.fails++; std::printf("  [%s] offset: ref=(%g,%g,%g) got=(%g,%g,%g)\n", c.name, ref["offset"]["x"].get<double>(), ref["offset"]["y"].get<double>(), ref["offset"]["z"].get<double>(), r.offset.x, r.offset.y, r.offset.z);
        }
        // pieces
        const json &rp = ref["pieces"];
        expect_eq(c, "pieces.size", r.pieces.size(), rp.size());
        for (size_t i = 0; i < std::min(r.pieces.size(), rp.size()); i++) {
            if (r.pieces[i].kind != rp[i]["kind"].get<std::string>()) { c.fails++; std::printf("  [%s] piece %zu kind: ref=%s got=%s\n", c.name, i, rp[i]["kind"].get<std::string>().c_str(), r.pieces[i].kind.c_str()); }
            auto rr = rp[i]["ranges"].get<std::vector<std::pair<int, int>>>();
            if (r.pieces[i].ranges != rr) { c.fails++; std::printf("  [%s] piece %zu ranges differ (ref %zu ranges, got %zu)\n", c.name, i, rr.size(), r.pieces[i].ranges.size()); if (verbose) for (size_t k = 0; k < std::max(rr.size(), r.pieces[i].ranges.size()); k++) std::printf("      ref=[%d,%d) got=[%d,%d)\n", k < rr.size() ? rr[k].first : -1, k < rr.size() ? rr[k].second : -1, k < r.pieces[i].ranges.size() ? r.pieces[i].ranges[k].first : -1, k < r.pieces[i].ranges.size() ? r.pieces[i].ranges[k].second : -1); }
        }
        // triangles
        std::vector<float> rt = ref["triangles"].get<std::vector<float>>();
        const size_t nRef = rt.size() / 9, nGot = r.triangles.size() / 9;
        size_t ordered = 0;
        for (size_t t = 0; t < std::min(nRef, nGot); t++) {
            bool ok = true;
            for (size_t k = 0; k < 9; k++) if (std::abs(double(rt[t * 9 + k]) - double(r.triangles[t * 9 + k])) > tol) { ok = false; break; }
            if (ok) ordered++;
            else if (verbose && ordered == t) {   // first mismatch, in order
                std::printf("  [%s] first ordered mismatch at tri %zu:\n      ref=", c.name, t);
                for (size_t k = 0; k < 9; k++) std::printf("%g ", rt[t * 9 + k]);
                std::printf("\n      got=");
                for (size_t k = 0; k < 9; k++) std::printf("%g ", r.triangles[t * 9 + k]);
                std::printf("\n");
            }
        }
        // unordered: canonical key = vertices quantised to tol, sorted
        auto keyOf = [&](const float *v) {
            std::array<std::array<long long, 3>, 3> vs;
            for (int i = 0; i < 3; i++) for (int k = 0; k < 3; k++) vs[i][k] = llround(double(v[i * 3 + k]) / tol);
            std::sort(vs.begin(), vs.end());
            return vs;
        };
        std::map<std::array<std::array<long long, 3>, 3>, int> bag;
        for (size_t t = 0; t < nRef; t++) bag[keyOf(&rt[t * 9])]++;
        size_t unordered = 0;
        for (size_t t = 0; t < nGot; t++) { auto it = bag.find(keyOf(&r.triangles[t * 9])); if (it != bag.end() && it->second > 0) { it->second--; unordered++; } }
        const bool triOk = nRef == nGot && ordered == nRef;
        if (!triOk) c.fails++;
        std::printf("%s: %s  tris ref=%zu got=%zu ordered=%.1f%% unordered=%.1f%%  walls=%d/%d tines=%d/%d  %.0f ms (js %d ms)\n",
                    file.c_str(), c.fails ? "FAIL" : "ok", nRef, nGot,
                    nRef ? 100.0 * ordered / nRef : 100.0, nRef ? 100.0 * unordered / nRef : 100.0,
                    r.stats.braces + r.stats.props, s["braces"].get<int>() + s["props"].get<int>(),
                    r.stats.tines, s["tines"].get<int>(), ms, ref.value("ms", 0));
        if (c.fails) failedFiles++;
    }
    std::printf("%zu dumps, %d failed\n", files.size(), failedFiles);
    return failedFiles ? 1 : 0;
}
