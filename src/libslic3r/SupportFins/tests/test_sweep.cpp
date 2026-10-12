// Pins Engine/Prop/Sweep, Clearance, Squat against the upstream JS (web/prop/sweep.js,
// clearance.js, squat.js). The oracle is tools/sweep_ref.mjs, which runs prop.js's plate
// path, attached.js's part-attached path and draw.js-style calls over real contact lines
// (stress models posed like tools/ref.mjs), plus synthetic grids, and records every call
// into the three modules; one JSON per scene in refs_sweep/ (next to tests/):
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/sweep_ref.mjs
//   ./build-sf/test_sweep [refs_sweep_dir]
// Each recorded call is replayed on the dumped inputs: counts, flags, reasons, interface
// tags exact; doubles within 1e-9 (and the count of not-bit-identical ones reported).
// `out` already holds `from` triangles' vertices at each call (an earlier wall, its
// tines): a synthetic prefix of that length, with tags, stands in and must come back
// untouched.
#include "Engine/Cutout.hpp"
#include "Engine/Inside.hpp"
#include "Engine/Overhangs.hpp"
#include "Engine/Prop/Clearance.hpp"
#include "Engine/Prop/Crest.hpp"
#include "Engine/Prop/Mold.hpp"
#include "Engine/Prop/Squat.hpp"
#include "Engine/Prop/Surface.hpp"
#include "Engine/Prop/Sweep.hpp"
#include "Engine/PropConfig.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
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

Vec3d V3(const json &p) { return Vec3d(N(p[0]), N(p[1]), N(p[2])); }
ContactLine line3(const json &a) {
    ContactLine l;
    for (const auto &p : a) l.push_back(V3(p));
    return l;
}
FloorLine floorOf(const json &a) {
    FloorLine f;
    for (const auto &p : a) {
        if (p.size() == 5) f.emplace_back(N(p[0]), N(p[1]), N(p[2]), N(p[3]), N(p[4]));
        else if (p.size() == 3) f.emplace_back(N(p[0]), N(p[1]), N(p[2]));
        else throw std::runtime_error("floor station of length " + std::to_string(p.size()));
    }
    return f;
}
Mask maskOf(const json &a) { Mask m; for (const auto &v : a) m.push_back(v.get<bool>()); return m; }
void eqLine(const ContactLine &cpp, const json &js, const std::string &what) {
    if (cpp.size() != js.size()) { fail(what + ": length cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    for (size_t i = 0; i < cpp.size(); i++)
        for (int k = 0; k < 3; k++) eqD(cpp[i][k], js[i][k], what + "[" + std::to_string(i) + "][" + std::to_string(k) + "]");
}
void eqMask(const Mask &cpp, const json &js, const std::string &what) {
    if (cpp.size() != js.size()) { fail(what + ": length cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    for (size_t i = 0; i < cpp.size(); i++) eqB(cpp[i], js[i].get<bool>(), what + "[" + std::to_string(i) + "]");
}
void eqRun(const std::optional<Run> &cpp, const json &js, const std::string &what) {
    eqB(cpp.has_value(), !js.is_null(), what + " null");
    if (cpp && !js.is_null()) { eqI((*cpp)[0], js[0].get<long>(), what + "[0]"); eqI((*cpp)[1], js[1].get<long>(), what + "[1]"); }
}

// The synthetic prefix standing in for `from` vertices already in `out`.
Tris prefixOf(size_t from) {
    Tris t;
    for (size_t i = 0; i < from; i++) t.push_back(Vec3d(0.5 * double(i), -double(i), 0.25 * double(i % 7)));
    return t;
}
IfaceTags prefixTags(size_t from) {
    IfaceTags t(from);
    for (size_t i = 0; i < from; i++) t[i] = uint8_t(i % 5 == 0 ? 1 : 0);
    return t;
}
// out[from..] and its tags against the dump; the prefix untouched.
void eqOut(const Tris &out, const IfaceTags &tags, size_t from, const json &c, const std::string &what) {
    const Tris pre = prefixOf(from);
    const IfaceTags preT = prefixTags(from);
    bool preOk = out.size() >= from;
    for (size_t i = 0; preOk && i < from; i++) preOk = out[i] == pre[i] && (i < tags.size() ? tags[i] : 0) == preT[i];
    eqB(preOk, true, what + " prefix untouched");
    const json &js = c["out"], &jt = c["tags"];
    const size_t n = out.size() >= from ? out.size() - from : 0;
    if (n * 3 != js.size()) { fail(what + ": vertex count cpp " + std::to_string(n) + " js " + std::to_string(js.size() / 3)); return; }
    S.checks++;
    for (size_t i = 0; i < n; i++) {
        for (int k = 0; k < 3; k++) eqD(out[from + i][k], js[i * 3 + size_t(k)], what + " v" + std::to_string(i) + "[" + std::to_string(k) + "]");
        const int t = from + i < tags.size() ? tags[from + i] : 0;
        eqI(t, jt[i].get<long>(), what + " tag v" + std::to_string(i));
    }
}

std::shared_ptr<SurfTris> g_part;

void applyCfg(const json &cfg, bool scene) {
    PROP = PropConfig{};
    CUT = CutConfig{};
    for (const auto &[k, v] : cfg["prop"].items()) {
        if (k == "gap") PROP.gap = N(v);
        else if (k == "footGap") PROP.footGap = N(v);
        else if (k == "footMin") PROP.footMin = N(v);
        else if (k == "footMax") PROP.footMax = N(v);
        else if (k == "footRatio") PROP.footRatio = N(v);
        else if (k == "maxUnsupportedSpan") PROP.maxUnsupportedSpan = N(v);
        else if (k == "tipH") PROP.tipH = N(v);
        else if (k == "baseH") PROP.baseH = N(v);
        else if (k == "th") PROP.th = N(v);
        else if (k == "tip") PROP.tip = N(v);
        else if (k == "ifaceLayers") PROP.ifaceLayers = v.get<int>();
        else if (k == "iface") {
            if (v.is_boolean()) PROP.iface = Iface::Off;
            else PROP.iface = v.get<std::string>() == "all" ? Iface::All : Iface::Flat;
        } else throw std::runtime_error("unknown PROP override " + k);
    }
    if (cfg.contains("pattern")) CUT.pattern = cfg["pattern"].get<std::string>();
    const double lh = cfg.contains("layerHeight") && !cfg["layerHeight"].is_null() ? N(cfg["layerHeight"]) : 0.0;
    // fins.js: crestPart(crestOn() ? seatedPartTris : null, layerHeight); the synthetic
    // cases run with no part
    crestPart(scene && crestOn() ? g_part : nullptr, lh);
}

void checkConsts(const json &P0) {
    const PropConfig def;
    eqD(def.gap, P0["gap"], "PROP.gap"); eqD(def.footGap, P0["footGap"], "PROP.footGap");
    eqD(def.th, P0["th"], "PROP.th"); eqD(def.tip, P0["tip"], "PROP.tip");
    eqD(def.baseH, P0["baseH"], "PROP.baseH"); eqD(def.tipH, P0["tipH"], "PROP.tipH");
    eqD(def.footMin, P0["footMin"], "PROP.footMin"); eqD(def.footMax, P0["footMax"], "PROP.footMax");
    eqD(def.footRatio, P0["footRatio"], "PROP.footRatio"); eqD(def.maxUnsupportedSpan, P0["maxUnsupportedSpan"], "PROP.maxUnsupportedSpan");
    eqD(def.minHeight, P0["minHeight"], "PROP.minHeight"); eqD(def.minHeightSquat, P0["minHeightSquat"], "PROP.minHeightSquat");
    eqD(def.minSpanSquat, P0["minSpanSquat"], "PROP.minSpanSquat"); eqD(def.squatBrimH, P0["squatBrimH"], "PROP.squatBrimH");
    eqD(def.squatBrimW, P0["squatBrimW"], "PROP.squatBrimW"); eqI(def.minStations, P0["minStations"].get<long>(), "PROP.minStations");
    eqI(def.clearProbes, P0["clearProbes"].get<long>(), "PROP.clearProbes"); eqD(def.sideClear, P0["sideClear"], "PROP.sideClear");
    eqD(def.minSpan, P0["minSpan"], "PROP.minSpan"); eqD(def.minSpanShort, P0["minSpanShort"], "PROP.minSpanShort");
    eqD(def.maxShortAspect, P0["maxShortAspect"], "PROP.maxShortAspect"); eqD(def.minSpanTube, P0["minSpanTube"], "PROP.minSpanTube");
}

void runFile(const fs::path &file) {
    std::ifstream in(file);
    const json d = json::parse(in);
    const std::string scene = d["scene"].get<std::string>();
    const bool isScene = d.contains("pos");
    g_fn = "consts"; g_ctx = scene + " consts";
    checkConsts(d["PROP"]);

    Topology topo;
    Offset off;
    const Rot rot = IDENTITY3;
    std::map<std::string, std::shared_ptr<SurfTris>> tris;
    if (isScene) {
        Soup pos; pos.reserve(d["pos"].size());
        for (const auto &v : d["pos"]) pos.push_back(N(v));
        topo = buildTopology(std::move(pos));
        const AnalyzeResult res = analyze(topo, 45, IDENTITY3);
        off = res.offset;
        g_fn = "offset"; g_ctx = scene + " offset";
        eqD(off.x, d["offset"]["x"], "offset.x"); eqD(off.y, d["offset"]["y"], "offset.y"); eqD(off.z, d["offset"]["z"], "offset.z");
        for (const auto &[id, arr] : d["tris"].items()) {
            Soup s; s.reserve(arr.size());
            for (const auto &v : arr) s.push_back(N(v));
            tris[id] = std::make_shared<SurfTris>(std::move(s));
        }
        g_part = tris.at("part");
    }

    int ci = 0;
    long curCfg = -2;
    for (const json &c : d["cases"]) {
        const std::string fn = c["fn"].get<std::string>();
        g_fn = fn;
        g_ctx = scene + " #" + std::to_string(ci++) + " " + fn + (c.contains("src") ? "/" + c["src"].get<std::string>() : "");
        S.casesByFn[fn]++;
        const long cfg = c.contains("cfg") ? c["cfg"].get<long>() : -1;
        if (cfg != curCfg) {
            if (cfg >= 0) applyCfg(d["cfgs"][size_t(cfg)], isScene);
            else { PROP = PropConfig{}; CUT = CutConfig{}; crestPart(nullptr); }
            curCfg = cfg;
        }
        g_ctx += " cfg" + std::to_string(cfg);

        if (fn == "sweep" || fn == "sweepSquat" || fn == "sweepBetween") {
            const size_t from = c["from"].get<size_t>();
            Tris out = prefixOf(from);
            IfaceTags tags = prefixTags(from);
            bool ret;
            if (fn == "sweep") {
                const ContactLine line = line3(c["line"]);
                ret = c["minH"].is_null() ? sweep(line, N(c["zBed"]), out, PROP.minHeight, &tags)
                                          : sweep(line, N(c["zBed"]), out, N(c["minH"]), &tags);
                // untracked tags: the same triangles
                Tris out2 = prefixOf(from);
                const bool ret2 = c["minH"].is_null() ? sweep(line, N(c["zBed"]), out2) : sweep(line, N(c["zBed"]), out2, N(c["minH"]));
                eqB(ret2 == ret && out2 == out, true, "sweep without tags agrees");
            } else if (fn == "sweepSquat") {
                ret = sweepSquat(line3(c["line"]), N(c["zBed"]), out, &tags);
            } else {
                const ContactLine top = line3(c["top"]);
                const FloorLine floor = floorOf(c["floor"]);
                ret = c["minH"].is_null() ? sweepBetween(top, floor, out, PROP.minHeight, &tags)
                                          : sweepBetween(top, floor, out, N(c["minH"]), &tags);
            }
            eqB(ret, c["ret"].get<bool>(), "ret");
            eqOut(out, tags, from, c, fn);
            if (c.contains("hit")) {
                const auto hit = solidClearance(topo, rot, off, Tris(out.begin() + long(from), out.end()), 0.25);
                eqB(hit.has_value(), !c["hit"].is_null(), "solidClearance null");
                if (hit && !c["hit"].is_null()) {
                    eqD(hit->d, c["hit"]["d"], "hit.d"); eqD(hit->cosUp, c["hit"]["cosUp"], "hit.cosUp");
                    eqD(hit->x, c["hit"]["x"], "hit.x"); eqD(hit->y, c["hit"]["y"], "hit.y"); eqD(hit->z, c["hit"]["z"], "hit.z");
                    eqB(welds(*hit), c["welds"].get<bool>(), "welds(hit)");
                }
            }

        } else if (fn == "buildSquatBed") {
            const size_t from = c["from"].get<size_t>();
            Tris out = prefixOf(from);
            IfaceTags tags = prefixTags(from);
            const ContactLine line = line3(c["line"]);
            const ContactLine lineCopy = line;
            Mask claimed;
            const bool hasClaimed = !c["claimed"].is_null();
            if (hasClaimed) claimed = maskOf(c["claimed"]);
            const auto placed = buildSquatBed(line, *tris.at(c["tris"].get<std::string>()), topo, rot, off, out,
                                              hasClaimed ? &claimed : nullptr, &tags);
            eqB(line == lineCopy, true, "line untouched");
            const json &P = c["placed"];
            eqI(long(placed.size()), long(P.size()), "placed count");
            for (size_t i = 0; i < std::min(placed.size(), P.size()); i++) {
                const SquatWall &w = placed[i];
                const json &j = P[i];
                const std::string at = "placed[" + std::to_string(i) + "].";
                eqD(w.span, j["span"], at + "span"); eqD(w.height, j["height"], at + "height");
                eqI(w.stations, j["stations"].get<long>(), at + "stations"); eqD(w.volume, j["volume"], at + "volume");
                eqB(w.squat, j["squat"].get<bool>(), at + "squat");
                eqLine(w.line, j["line"], at + "line");
                eqI(long(w.triRange[0]), j["triRange"][0].get<long>(), at + "triRange[0]");
                eqI(long(w.triRange[1]), j["triRange"][1].get<long>(), at + "triRange[1]");
            }
            eqOut(out, tags, from, c, fn);

        } else if (fn == "stationIsClear" || fn == "stationCertified") {
            const ContactLine line = line3(c["line"]);
            const bool cert = fn == "stationCertified";
            for (size_t k = 0; k < line.size(); k++) {
                const bool r = cert ? stationCertified(line, k, topo, rot, off) : stationIsClear(line, k, topo, rot, off);
                eqB(r, c["res"][k].get<bool>(), fn + "[" + std::to_string(k) + "]");
            }
            if (!cert) eqB(pathToPlateIsClear(line, topo, rot, off), c["path"].get<bool>(), "pathToPlateIsClear");

        } else if (fn == "longestRun") {
            eqRun(longestRun(maskOf(c["inp"])), c["out"], "longestRun");

        } else if (fn == "withLowTails") {
            eqMask(withLowTails(maskOf(c["tall"]), maskOf(c["low"])), c["out"], "withLowTails");

        } else if (fn == "bodyMask") {
            const ContactLine pts = line3(c["pts"]);
            const Mask body = bodyMask(pts);
            eqMask(body, c["out"], "bodyMask");
            eqRun(tallBody(body), c["tallBody"], "tallBody");
            eqD(tallSpan(pts, body), c["tallSpan"], "tallSpan");
            int i = 0;
            for (const bool st : {false, true}) for (const bool sh : {false, true})
                eqD(minSpanFor(pts, body, st, sh), c["minSpanFor"][size_t(i++)], "minSpanFor");

        } else if (fn == "insertFloorStations") {
            ContactLine line = line3(c["inp"]);
            ContactLine &r = insertFloorStations(line);
            eqB(&r == &line, true, "returns line");
            eqLine(line, c["out"], "insertFloorStations");

        } else if (fn == "footFor") {
            for (size_t i = 0; i < c["hs"].size(); i++) eqD(footFor(N(c["hs"][i])), c["out"][i], "footFor(" + c["hs"][i].dump() + ")");

        } else if (fn == "profileHalf") {
            for (const json &k : c["cases"]) eqD(profileHalf(N(k[0]), N(k[1])), k[2], "profileHalf(" + k[0].dump() + "," + k[1].dump() + ")");

        } else if (fn == "welds") {
            for (const json &k : c["cases"]) {
                NearestHit h; h.d = N(k[0]); h.cosUp = N(k[1]);
                SolidHit s; s.d = h.d; s.cosUp = h.cosUp;
                eqB(welds(h), k[2].get<bool>(), "welds(" + k[0].dump() + "," + k[1].dump() + ")");
                eqB(welds(s), k[2].get<bool>(), "welds(solid)");
            }

        } else {
            fail("unknown case fn " + fn);
        }
    }
    PROP = PropConfig{}; CUT = CutConfig{}; crestPart(nullptr); g_part.reset();
}

} // namespace

int main(int argc, char **argv) {
    const fs::path dir = argc > 1 ? fs::path(argv[1]) : fs::path(__FILE__).parent_path().parent_path() / "refs_sweep";
    if (!fs::is_directory(dir)) {
        std::fprintf(stderr, "no dumps in %s: run SUPPORT_FINS_DIR=... node tools/sweep_ref.mjs\n", dir.string().c_str());
        return 1;
    }
    std::vector<fs::path> files;
    for (const auto &e : fs::directory_iterator(dir)) if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::fprintf(stderr, "no dumps in %s\n", dir.string().c_str()); return 1; }

    // the JS throws (a TypeError in ribbon) on an empty line; here std::invalid_argument
    {
        g_fn = "empty"; g_ctx = "empty line";
        Tris out;
        bool t1 = false, t2 = false, t3 = false;
        try { sweep({}, 0, out); } catch (const std::invalid_argument &) { t1 = true; }
        try { sweepSquat({}, 0, out); } catch (const std::invalid_argument &) { t2 = true; }
        try { sweepBetween({}, {}, out); } catch (const std::invalid_argument &) { t3 = true; }
        eqB(t1 && t2 && t3 && out.empty(), true, "empty line throws");
    }

    for (const fs::path &f : files) {
        const long before = S.fails;
        runFile(f);
        std::printf("%-22s %s\n", f.stem().string().c_str(), S.fails == before ? "ok" : "FAIL");
    }
    std::printf("cases:");
    for (const auto &[fn, n] : S.casesByFn) std::printf(" %s=%ld", fn.c_str(), n);
    std::printf("\nchecks %ld, failures %ld, non-bit-identical doubles %ld (max diff %.3g)\n",
                S.checks, S.fails, S.inexact, S.maxDiff);
    for (const auto &[fn, n] : S.failsByFn) std::printf("  failures in %s: %ld\n", fn.c_str(), n);
    return S.fails ? 1 : 0;
}
