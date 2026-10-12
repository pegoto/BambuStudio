// Pins Engine/Prop (web/prop.js buildProps / buildShortWalls) and Engine/Draw (web/draw.js
// drawnLine / drawnWall, plugins/shared/engine/draw_entry.js drawWall) against the
// upstream JS. The oracle is tools/prop_ref.mjs: every stress model at 0,0,0 and 40,0,0
// plus extra poses, each seated + analyzed as computeFins does, then a sequence of calls
// per config (tunables, iface, cutout, crestPart as buildFins sets it) recorded in order;
// one JSON per scene in refs_prop/ (next to tests/):
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/prop_ref.mjs
//   ./build-prop/test_prop [refs_prop_dir]
// The calls are replayed in the same order (module state -- PROP, CUT, the crest's part,
// the tine capture -- carries between them as it does in the JS). Counts, flags, strings,
// tags and orders exact; doubles within 1e-9, and the not-bit-identical ones counted.
// The auto-mode raster veto (fins/wedges.js wedgeVeto, not ported yet) is replayed from
// its recorded verdicts, in call order, checking each call's wall.
#include "Engine/Cutout.hpp"
#include "Engine/Draw.hpp"
#include "Engine/Fins/Seating.hpp"
#include "Engine/Materials.hpp"
#include "Engine/Overhangs.hpp"
#include "Engine/Prop.hpp"
#include "Engine/Prop/Crest.hpp"
#include "Engine/Prop/Tines.hpp"
#include "Engine/PropConfig.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace Slic3r::SupportFins;
using json = nlohmann::json;
namespace fs = std::filesystem;

// Entry.hpp's (not included: Entry.hpp pulls Fins.hpp, whose TriRange clashes with
// Prop/Raster.hpp's). Its drawWall forwarder is exercised through Draw's drawWall.
namespace Slic3r { namespace SupportFins {
Offset seatSoup(const Soup &input, Soup &pos);
}}

namespace {

struct Stats {
    long   checks = 0, fails = 0, inexact = 0;
    double maxDiff = 0;
    std::map<std::string, long> casesByFn, failsByFn;
} S;
std::string g_ctx, g_fn;

void fail(const std::string &what) {
    S.fails++;
    S.failsByFn[g_fn]++;
    if (S.fails <= 40) std::fprintf(stderr, "FAIL [%s] %s\n", g_ctx.c_str(), what.c_str());
}

double N(const json &j) {
    if (j.is_number()) return j.get<double>();
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        if (s == "Infinity") return INF;
        if (s == "-Infinity") return -INF;
        if (s == "NaN") return NAN;
    }
    throw std::runtime_error("not a number: " + j.dump());
}
std::string fmt(double v) { char b[64]; std::snprintf(b, sizeof b, "%.17g", v); return b; }

void eqD(double cpp, double js, const std::string &what) {
    S.checks++;
    if (std::isnan(cpp) && std::isnan(js)) return;
    if (std::isinf(cpp) || std::isinf(js)) {
        if (cpp != js) fail(what + ": cpp " + fmt(cpp) + " js " + fmt(js));
        return;
    }
    const double d = std::abs(cpp - js);
    if (d != 0) S.inexact++;
    S.maxDiff = std::max(S.maxDiff, d);
    if (!(d <= 1e-9)) fail(what + ": cpp " + fmt(cpp) + " js " + fmt(js));
}
void eqD(double cpp, const json &js, const std::string &what) { eqD(cpp, N(js), what); }
void eqI(long cpp, long js, const std::string &what) {
    S.checks++;
    if (cpp != js) fail(what + ": cpp " + std::to_string(cpp) + " js " + std::to_string(js));
}
void eqB(bool cpp, bool js, const std::string &what) { eqI(cpp ? 1 : 0, js ? 1 : 0, what); }
void eqS(const std::string &cpp, const std::string &js, const std::string &what) {
    S.checks++;
    if (cpp != js) fail(what + ": cpp \"" + cpp + "\" js \"" + js + "\"");
}
// JS `x ?? false` on an optional boolean field
bool B(const json &j) { return j.is_boolean() ? j.get<bool>() : false; }

Vec3d V3(const json &p) { return Vec3d(N(p[0]), N(p[1]), N(p[2])); }
void eqLine(const ContactLine &cpp, const json &js, const std::string &what) {
    if (cpp.size() != js.size()) { fail(what + ": length cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    for (size_t i = 0; i < cpp.size(); i++)
        for (int k = 0; k < 3; k++) eqD(cpp[i][k], js[i][k], what + "[" + std::to_string(i) + "][" + std::to_string(k) + "]");
}
void eqTris(const Tris &cpp, const json &js, const std::string &what) {
    if (cpp.size() * 3 != js.size()) {
        fail(what + ": length cpp " + std::to_string(cpp.size() * 3) + " js " + std::to_string(js.size()));
        return;
    }
    for (size_t i = 0; i < cpp.size(); i++)
        for (int k = 0; k < 3; k++) eqD(cpp[i][k], js[i * 3 + size_t(k)], what + "[" + std::to_string(i) + "]");
}
void eqTags(const IfaceTags &cpp, size_t n, const json &js, const std::string &what) {
    if (n != js.size()) { fail(what + ": length cpp " + std::to_string(n) + " js " + std::to_string(js.size())); return; }
    long bad = 0;
    for (size_t i = 0; i < n; i++) {
        const int c = i < cpp.size() ? cpp[i] : 0;
        if (c != js[i].get<int>()) bad++;
    }
    eqI(bad, 0, what + " mismatched tags");
}

// ---- buildProps' result ------------------------------------------------------------
void eqProp(const PropRecord &q, const json &j, const std::string &w) {
    eqI(q.region, j["region"].get<long>(), w + ".region");
    eqI(q.tines, j["tines"].get<long>(), w + ".tines");
    eqB(q.caps.has_value(), !j["caps"].is_null(), w + ".caps null");
    if (q.caps && !j["caps"].is_null()) {
        eqI(long((*q.caps)[0]), j["caps"][0].get<long>(), w + ".caps[0]");
        eqI(long((*q.caps)[1]), j["caps"][1].get<long>(), w + ".caps[1]");
    }
    eqD(q.grip, j["grip"], w + ".grip");
    eqD(q.span, j["span"], w + ".span");
    eqD(q.height, j["height"], w + ".height");
    eqD(q.area, j["area"], w + ".area");
    eqI(q.stations, j["stations"].get<long>(), w + ".stations");
    eqB(q.trimmed.has_value(), !j["trimmed"].is_null(), w + ".trimmed null");
    if (q.trimmed && !j["trimmed"].is_null()) eqI(*q.trimmed, j["trimmed"].get<long>(), w + ".trimmed");
    eqD(q.volume, j["volume"], w + ".volume");
    eqLine(q.line, j["line"], w + ".line");
    eqI(q.id, j["id"].get<long>(), w + ".id");
    eqS(q.kind, j["kind"].get<std::string>(), w + ".kind");
    const json &tr = j["triRanges"];
    if (q.triRanges.size() != tr.size()) fail(w + ".triRanges length");
    else for (size_t i = 0; i < tr.size(); i++) {
        eqI(long(q.triRanges[i][0]), tr[i][0].get<long>(), w + ".triRanges[0]");
        eqI(long(q.triRanges[i][1]), tr[i][1].get<long>(), w + ".triRanges[1]");
    }
    eqB(q.partAttached, B(j["partAttached"]), w + ".partAttached");
    eqB(q.inBore, B(j["inBore"]), w + ".inBore");
    eqB(q.squat, B(j["squat"]), w + ".squat");
    eqB(q.triRange.has_value(), !j["triRange"].is_null(), w + ".triRange null");
    if (q.triRange && !j["triRange"].is_null()) {
        eqI(long((*q.triRange)[0]), j["triRange"][0].get<long>(), w + ".triRange[0]");
        eqI(long((*q.triRange)[1]), j["triRange"][1].get<long>(), w + ".triRange[1]");
    }
    eqB(q.raster, B(j["raster"]), w + ".raster");
}

void eqResult(const PropsResult &r, const json &j, const std::string &w) {
    eqTris(r.triangles, j["triangles"], w + ".triangles");
    eqTags(r.tags, r.triangles.size(), j["tags"], w + ".tags");
    const json &ps = j["props"];
    eqI(long(r.props.size()), long(ps.size()), w + ".props.length");
    for (size_t i = 0; i < std::min(r.props.size(), ps.size()); i++) eqProp(r.props[i], ps[i], w + ".props[" + std::to_string(i) + "]");
    const json &sk = j["skipped"];
    eqI(r.skipped.noLine, sk["noLine"].get<long>(), w + ".skipped.noLine");
    eqI(r.skipped.wanders, sk["wanders"].get<long>(), w + ".skipped.wanders");
    eqI(r.skipped.stub, sk["stub"].get<long>(), w + ".skipped.stub");
    eqI(r.skipped.blocked, sk["blocked"].get<long>(), w + ".skipped.blocked");
    eqI(r.skipped.degenerate, sk["degenerate"].get<long>(), w + ".skipped.degenerate");
    eqI(r.skipped.buried, sk["buried"].get<long>(), w + ".skipped.buried");
    eqI(r.skipped.weld, sk["weld"].get<long>(), w + ".skipped.weld");
    eqI(r.skipped.sliver, sk["sliver"].get<long>(), w + ".skipped.sliver");
    eqI(r.skipped.onPart, sk["onPart"].get<long>(), w + ".skipped.onPart");
    eqI(r.served, j["served"].get<long>(), w + ".served");
    const json &sr = j["servedRegions"];
    if (sr.is_null()) eqI(long(r.servedRegions.size()), 0, w + ".servedRegions (undefined)");
    else {
        eqI(long(r.servedRegions.size()), long(sr.size()), w + ".servedRegions.length");
        for (size_t i = 0; i < std::min(r.servedRegions.size(), sr.size()); i++)
            eqI(r.servedRegions[i], sr[i].get<long>(), w + ".servedRegions[" + std::to_string(i) + "]");
    }
    eqI(r.tines, j["tines"].is_null() ? 0 : j["tines"].get<long>(), w + ".tines");
    eqB(r.sagRisk, B(j["sagRisk"]), w + ".sagRisk");
    const json &sg = j["sagRegions"];
    eqB(r.sagRegions.has_value(), !sg.is_null(), w + ".sagRegions null");
    if (r.sagRegions && !sg.is_null()) {
        eqI(long(r.sagRegions->order.size()), long(sg.size()), w + ".sagRegions.size");
        for (size_t i = 0; i < std::min(r.sagRegions->order.size(), sg.size()); i++)
            eqI(r.sagRegions->order[i], sg[i].get<long>(), w + ".sagRegions[i]");
    }
    eqD(r.volume, j["volume"], w + ".volume");
    const json &rr = j["rasterRegions"];
    eqB(r.rasterRegions.has_value(), !rr.is_null(), w + ".rasterRegions null");
    if (r.rasterRegions && !rr.is_null()) eqI(*r.rasterRegions, rr.get<long>(), w + ".rasterRegions");
}

PropOpts propOptsOf(const json &o) {
    PropOpts p;
    if (o.contains("step")) p.step = N(o["step"]);
    if (o.contains("coverage")) p.coverage = N(o["coverage"]);
    p.tines = o.contains("tines") && o["tines"].is_boolean() && o["tines"].get<bool>();
    if (o.contains("tineDensity")) p.tineDensity = N(o["tineDensity"]);
    if (o.contains("layerHeight")) p.layerHeight = N(o["layerHeight"]);
    p.plateOnly = o.contains("plateOnly") && o["plateOnly"].is_boolean() && o["plateOnly"].get<bool>();
    p.raster = !(o.contains("raster") && o["raster"].is_boolean() && !o["raster"].get<bool>());
    return p;
}

// ---- one scene ---------------------------------------------------------------------
void runFile(const fs::path &f) {
    std::ifstream in(f);
    const json J = json::parse(in);
    const std::string scene = J["scene"].get<std::string>();
    Soup posed;
    for (const auto &v : J["soup"]) posed.push_back(N(v));
    Soup pos;
    seatSoup(posed, pos);
    Topology topo = buildTopology(pos);
    const AnalyzeResult result = analyze(topo, 45, IDENTITY3);
    g_fn = "scene"; g_ctx = scene;
    eqI(long(result.regions.size()), J["regions"].get<long>(), "regions");
    const auto partTris = std::make_shared<const SurfTris>(seatedPartTris(topo, IDENTITY3, result.offset));

    int idx = 0;
    for (const json &c : J["calls"]) {
        const std::string fn = c["fn"].get<std::string>();
        g_fn = fn;
        g_ctx = scene + " #" + std::to_string(idx++) + " " + fn + (c.contains("kind") ? " " + c["kind"].get<std::string>() : "");
        S.casesByFn[fn]++;
        if (fn == "cfg") {
            PROP = PropConfig{};
            CUT = CutConfig{};
            const Material *mat = material_of(c["material"].get<std::string>());
            PROP.gap = mat->propGap;
            const std::string iface = c["iface"].get<std::string>();
            PROP.iface = iface == "flat" ? Iface::Flat : iface == "all" ? Iface::All : Iface::Off;
            CUT.pattern = c["cutout"].get<std::string>();
            const double lh = c["layerHeight"].is_null() ? 0 : N(c["layerHeight"]);
            crestPart(crestOn() ? partTris : nullptr, lh);
            g_ctx += " cfg " + std::to_string(c["cfg"].get<int>());
            eqD(PROP.gap, c["gap"], "gap");
        } else if (fn == "buildProps" || fn == "buildShortWalls") {
            PropOpts o = propOptsOf(c["opts"]);
            size_t vetoAt = 0;
            const json *veto = c.contains("veto") && !c["veto"].is_null() ? &c["veto"] : nullptr;
            if (veto) {
                o.rasterVeto = [&](const PropRecord &q, const std::vector<PropRecord> &) {
                    if (vetoAt >= veto->size()) { fail("rasterVeto called more often than in JS"); return false; }
                    const json &v = (*veto)[vetoAt++];
                    eqI(long(q.line.size()), v["n"].get<long>(), "veto wall stations");
                    if (!q.line.empty() && !v["p0"].is_null())
                        for (int k = 0; k < 3; k++) eqD(q.line[0][k], v["p0"][k], "veto wall line[0]");
                    return v["r"].get<bool>();
                };
            }
            std::vector<TineCap> cap;
            const bool capture = c.contains("capture") && c["capture"].get<bool>();
            if (capture) TINECAP = &cap;
            PropsResult r;
            if (fn == "buildProps") {
                r = buildProps(topo, result, IDENTITY3, o);
            } else {
                RegionSet regions;
                for (const auto &v : c["regions"]) regions.add(v.get<int>());
                r = buildShortWalls(topo, result, IDENTITY3, o, regions);
            }
            TINECAP = nullptr;
            if (veto) eqI(long(vetoAt), long(veto->size()), "rasterVeto calls");
            eqResult(r, c["out"], "out");
            if (capture) {
                const json &tc = c["tinecap"];
                eqI(long(cap.size()), long(tc.size()), "tinecap.length");
                for (size_t i = 0; i < std::min(cap.size(), tc.size()); i++) {
                    eqD(cap[i].x, tc[i][0], "tinecap.x"); eqD(cap[i].y, tc[i][1], "tinecap.y");
                    eqD(cap[i].z, tc[i][2], "tinecap.z"); eqD(cap[i].biteX, tc[i][3], "tinecap.biteX");
                    eqD(cap[i].biteY, tc[i][4], "tinecap.biteY");
                }
            }
        } else if (fn == "drawnLine") {
            const Vec3d a = V3(c["a"]), b = V3(c["b"]);
            const std::optional<ContactLine> l = c["step"].is_null()
                ? drawnLine(a, b, *partTris)
                : drawnLine(a, b, *partTris, N(c["step"]), N(c["band"]), c["under"].get<bool>());
            eqB(l.has_value(), !c["out"].is_null(), "null");
            if (l && !c["out"].is_null()) eqLine(*l, c["out"], "line");
        } else if (fn == "drawnWall") {
            const json &oj = c["opts"];
            DrawnWallOpts o;
            o.under = B(oj.value("under", json()));
            o.plateOnly = B(oj.value("plateOnly", json()));
            o.brace = B(oj.value("brace", json()));
            o.tines = B(oj.value("tines", json()));
            if (oj.contains("tineDensity")) o.tineDensity = N(oj["tineDensity"]);
            if (oj.contains("layerHeight")) o.layerHeight = N(oj["layerHeight"]);
            if (B(oj.value("topo", json()))) { o.topo = &topo; o.rot = IDENTITY3; o.offset = result.offset; }
            const DrawnWall r = drawnWall(V3(c["a"]), V3(c["b"]), partTris, N(c["zBed"]), o);
            const json &j = c["out"];
            eqB(r.ok, j["ok"].get<bool>(), "ok");
            eqS(r.reason, j["reason"].is_null() ? std::string() : j["reason"].get<std::string>(), "reason");
            if (j["tris"].is_null()) eqI(long(r.tris.size()), 0, "tris (none)");
            else {
                eqTris(r.tris, j["tris"], "tris");
                eqTags(r.tags, r.tris.size(), j["tags"], "tags");
            }
            if (r.ok && j["ok"].get<bool>()) {
                eqD(r.length, j["length"], "length");
                eqD(r.height, j["height"], "height");
                eqB(r.partAttached, B(j["partAttached"]), "partAttached");
                eqB(r.squat, B(j["squat"]), "squat");
                eqI(r.tines, j["tines"].get<long>(), "tines");
                eqB(r.braces.has_value(), !j["braces"].is_null(), "braces null");
                if (r.braces && !j["braces"].is_null()) eqI(*r.braces, j["braces"].get<long>(), "braces");
                eqLine(r.top, j["top"], "top");
                eqB(r.floors.has_value(), !j["floors"].is_null(), "floors null");
                if (r.floors && !j["floors"].is_null()) {
                    eqI(long(r.floors->size()), long(j["floors"].size()), "floors.length");
                    for (size_t i = 0; i < std::min(r.floors->size(), j["floors"].size()); i++) eqD((*r.floors)[i], j["floors"][i], "floors[i]");
                }
            }
        } else if (fn == "drawWall") {
            const json &oj = c["options"];
            DrawWallOptions o;
            if (oj.contains("material")) o.material = oj["material"].get<std::string>();
            if (oj.contains("tines")) o.tines = oj["tines"].get<bool>();
            if (oj.contains("tineDensity")) o.tineDensity = N(oj["tineDensity"]);
            if (oj.contains("layerHeight")) o.layerHeight = N(oj["layerHeight"]);
            if (oj.contains("plateOnly")) o.plateOnly = oj["plateOnly"].get<bool>();
            if (oj.contains("threshold")) o.threshold = N(oj["threshold"]);
            if (oj.contains("cutout")) o.cutout = oj["cutout"].get<std::string>();
            const DrawWallResult r = drawWall(posed, V3(c["a"]), V3(c["b"]), o);
            const json &j = c["out"];
            eqB(r.ok, j["ok"].get<bool>(), "ok");
            if (!j["ok"].get<bool>()) eqS(r.reason, j["reason"].get<std::string>(), "reason");
            else if (r.ok) {
                const json &t = j["triangles"];
                eqI(long(r.triangles.size()), long(t.size()), "triangles.length");
                if (r.triangles.size() == t.size())
                    for (size_t i = 0; i < t.size(); i++) eqD(double(r.triangles[i]), t[i], "triangles[i]");
                eqD(r.offset.x, j["offset"]["x"], "offset.x");
                eqD(r.offset.y, j["offset"]["y"], "offset.y");
                eqD(r.offset.z, j["offset"]["z"], "offset.z");
                eqD(r.stats.length, j["stats"]["length"], "stats.length");
                eqD(r.stats.height, j["stats"]["height"], "stats.height");
                eqI(r.stats.tines, j["stats"]["tines"].get<long>(), "stats.tines");
                eqB(r.stats.partAttached, j["stats"]["partAttached"].get<bool>(), "stats.partAttached");
            }
        } else {
            fail("unknown call " + fn);
        }
    }
    PROP = PropConfig{};
    CUT = CutConfig{};
    crestPart(nullptr);
}

} // namespace

int main(int argc, char **argv) {
    const fs::path dir = argc > 1 ? fs::path(argv[1]) : fs::path(__FILE__).parent_path().parent_path() / "refs_prop";
    if (!fs::is_directory(dir)) {
        std::fprintf(stderr, "no dumps in %s: run SUPPORT_FINS_DIR=... node tools/prop_ref.mjs\n", dir.string().c_str());
        return 1;
    }
    std::vector<fs::path> files;
    for (const auto &e : fs::directory_iterator(dir)) if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::fprintf(stderr, "no dumps in %s\n", dir.string().c_str()); return 1; }

    // fixed points: coverRowSpan, noProps, the drawWall entry's argument checks
    {
        g_fn = "unit"; g_ctx = "coverRowSpan";
        eqD(coverRowSpan(0), 30, "c=0");
        eqD(coverRowSpan(0.5), 12, "c=0.5");
        eqD(coverRowSpan(1), 6, "c=1");
        eqD(coverRowSpan(-3), 30, "c<0");
        eqD(coverRowSpan(0.25), 12 + (0.25 / 0.5) * (30 - 12.0), "c=0.25");
        eqB(std::isnan(coverRowSpan(NAN)), true, "NaN");
        g_ctx = "noProps";
        const PropsResult n = noProps();
        eqB(n.triangles.empty() && n.props.empty() && n.served == 0 && n.volume == 0 && n.skipped.onPart == 0, true, "empty");
        g_ctx = "drawWall throws";
        const Soup tri = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        bool t1 = false, t2 = false, t3 = false;
        try { drawWall(tri, Vec3d(NAN, 0, 0), Vec3d(0, 0, 0)); } catch (const std::invalid_argument &) { t1 = true; }
        try { DrawWallOptions o; o.material = "abs"; drawWall(tri, Vec3d(0, 0, 0), Vec3d(1, 0, 0), o); } catch (const std::invalid_argument &) { t2 = true; }
        try { drawWall({}, Vec3d(0, 0, 0), Vec3d(1, 0, 0)); } catch (const std::invalid_argument &) { t3 = true; }
        eqB(t1 && t2 && t3, true, "invalid_argument");
        PROP = PropConfig{};
        CUT = CutConfig{};
    }

    for (const fs::path &f : files) {
        const long before = S.fails;
        const auto t0 = std::chrono::steady_clock::now();
        runFile(f);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("%-22s %s (%.0f ms)\n", f.stem().string().c_str(), S.fails == before ? "ok" : "FAIL", ms);
    }
    std::printf("cases:");
    for (const auto &[fn, n] : S.casesByFn) std::printf(" %s=%ld", fn.c_str(), n);
    std::printf("\nchecks %ld, failures %ld, non-bit-identical doubles %ld (max diff %.3g)\n",
                S.checks, S.fails, S.inexact, S.maxDiff);
    if (S.fails) {
        std::printf("failures by call:");
        for (const auto &[fn, n] : S.failsByFn) std::printf(" %s=%ld", fn.c_str(), n);
        std::printf("\n");
    }
    return S.fails ? 1 : 0;
}
