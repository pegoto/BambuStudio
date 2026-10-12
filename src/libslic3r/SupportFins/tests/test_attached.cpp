// Pins Engine/Prop/Attached and Engine/Prop/Raster against the upstream JS
// (web/prop/attached.js, raster.js). The oracle is tools/attached_ref.mjs: on stress
// models (and generated tunnels) posed like tools/ref.mjs, per config, it runs
// buildPartAttached over every line prop.js buildPass would hand it (normal pass and
// raster pass, every region) plus prop.js's part-attached tine glue, rasterTracks /
// rasterRest for every region, and the race (withRaster driving prop.js buildPass, with
// the tine capture and a rasterVeto), recording everything; one JSON per scene in
// refs_attached/ (next to tests/):
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/attached_ref.mjs
//   ./build/test_attached [refs_attached_dir]
// buildPass (prop.js) is not ported: the race replays the JS passes' dumped results as
// `normal` and as what `rasterPass` returns, and the JS veto's recorded decisions as
// `rasterVeto`; withRaster / raceRegions / keepers / ... run for real on them.
// Counts, flags, reasons, tags exact; doubles within 1e-9 (and the count of
// not-bit-identical ones reported).
#include "Engine/Inside.hpp"
#include "Engine/Overhangs.hpp"
#include "Engine/Prop/Attached.hpp"
#include "Engine/Prop/Clearance.hpp"
#include "Engine/Prop/Contact.hpp"
#include "Engine/Prop/Crest.hpp"
#include "Engine/Prop/Mold.hpp"
#include "Engine/Prop/Raster.hpp"
#include "Engine/Prop/Rival.hpp"
#include "Engine/Prop/Surface.hpp"
#include "Engine/Prop/Tines.hpp"
#include "Engine/Prop/Tracks.hpp"
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
    long   attachedOk = 0, inBore = 0, swaps = 0, vetoed = 0;
    std::map<std::string, long> floored;
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
    if (cpp != js) fail(what + ": cpp '" + cpp + "' js '" + js + "'");
}

Vec3d V3(const json &p) { return Vec3d(N(p[0]), N(p[1]), N(p[2])); }
ContactLine line3(const json &a) {
    ContactLine l;
    for (const auto &p : a) l.push_back(V3(p));
    return l;
}
void eqLine(const ContactLine &cpp, const json &js, const std::string &what) {
    if (cpp.size() != js.size()) { fail(what + ": length cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    S.checks++;
    for (size_t i = 0; i < cpp.size(); i++)
        for (int k = 0; k < 3; k++) eqD(cpp[i][k], js[i][k], what + "[" + std::to_string(i) + "][" + std::to_string(k) + "]");
}
void eqFloor(const FloorLine &cpp, const json &js, const std::string &what) {
    if (cpp.size() != js.size()) { fail(what + ": length cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    for (size_t i = 0; i < cpp.size(); i++) {
        eqI(long(cpp[i].length()), long(js[i].size()), what + "[" + std::to_string(i) + "].length");
        for (size_t k = 0; k < std::min(cpp[i].length(), js[i].size()); k++)
            eqD(cpp[i][int(k)], js[i][k], what + "[" + std::to_string(i) + "][" + std::to_string(k) + "]");
    }
}
// Flat triangles + tag string against out[from..] and its tags.
void eqTris(const Tris &out, const IfaceTags &tags, size_t from, const json &flat, const std::string &tagStr,
            const std::string &what) {
    const size_t n = out.size() >= from ? out.size() - from : 0;
    if (n * 3 != flat.size() || n != tagStr.size()) {
        fail(what + ": vertex count cpp " + std::to_string(n) + " js " + std::to_string(flat.size() / 3));
        return;
    }
    S.checks++;
    long tagFails = 0;
    for (size_t i = 0; i < n; i++) {
        for (int k = 0; k < 3; k++) eqD(out[from + i][k], flat[i * 3 + size_t(k)], what + " v" + std::to_string(i) + "[" + std::to_string(k) + "]");
        const int t = from + i < tags.size() ? tags[from + i] : 0;
        if (t != tagStr[i] - '0') tagFails++;
    }
    eqI(tagFails, 0, what + " tags differing");
}

// ---- the scene -------------------------------------------------------------------------
struct RegionData {
    std::shared_ptr<SurfTris> tris;
    std::vector<Vec3d> pts;
    double area = 0;
};
Topology g_topo;
AnalyzeResult g_res;
Offset g_off;
const Rot g_rot = IDENTITY3;
std::shared_ptr<SurfTris> g_part;
std::vector<RegionData> g_regions;

void applyCfg(const json &cfg) {
    PROP = PropConfig{};
    for (const auto &[k, v] : cfg["prop"].items()) {
        if (k == "gap") PROP.gap = N(v);
        else if (k == "footGap") PROP.footGap = N(v);
        else if (k == "iface") PROP.iface = v.get<std::string>() == "all" ? Iface::All : Iface::Flat;
        else throw std::runtime_error("unknown PROP override " + k);
    }
    const double lh = cfg["layerHeight"].is_null() ? 0.0 : N(cfg["layerHeight"]);
    // fins.js: crestPart(crestOn() ? seatedPartTris : null, layerHeight)
    crestPart(crestOn() ? g_part : nullptr, lh);
}

void checkConsts(const json &d) {
    const PropConfig def;
    const json &P0 = d["PROP"];
    g_fn = "consts"; g_ctx = d["scene"].get<std::string>() + " consts";
    eqD(def.gap, P0["gap"], "PROP.gap"); eqD(def.footGap, P0["footGap"], "PROP.footGap");
    eqD(def.th, P0["th"], "PROP.th"); eqD(def.minHeight, P0["minHeight"], "PROP.minHeight");
    eqI(def.minStations, P0["minStations"].get<long>(), "PROP.minStations"); eqD(def.sideClear, P0["sideClear"], "PROP.sideClear");
    eqD(def.minSpanPart, P0["minSpanPart"], "PROP.minSpanPart"); eqD(def.maxUnsupportedSpan, P0["maxUnsupportedSpan"], "PROP.maxUnsupportedSpan");
    eqD(def.squatBrimW, P0["squatBrimW"], "PROP.squatBrimW"); eqD(def.stationStep, P0["stationStep"], "PROP.stationStep");
    eqD(def.tineH, P0["tineH"], "PROP.tineH");
    const json &c = d["consts"];
    eqD(PART_BAND, c["PART_BAND"], "PART_BAND"); eqD(RACE_REL, c["RACE_REL"], "RACE_REL"); eqD(RACE_ABS, c["RACE_ABS"], "RACE_ABS");
    eqD(RASTER_SKIP_HELD, c["RASTER_SKIP_HELD"], "RASTER_SKIP_HELD"); eqD(GRIP_RISE, c["GRIP_RISE"], "GRIP_RISE");
    eqD(GRIP_RISE_REL, c["GRIP_RISE_REL"], "GRIP_RISE_REL");
}

// ---- buildProps records / results ------------------------------------------------------
PropRecord propOf(const json &j) {
    PropRecord q;
    q.region = j.at("region").get<int>();
    q.tines = j.value("tines", 0);
    if (j.contains("caps")) q.caps = std::array<size_t, 2>{j["caps"][0].get<size_t>(), j["caps"][1].get<size_t>()};
    if (j.contains("grip")) q.grip = N(j["grip"]);
    if (j.contains("span")) q.span = N(j["span"]);
    if (j.contains("height")) q.height = N(j["height"]);
    if (j.contains("area")) q.area = N(j["area"]);
    if (j.contains("stations")) q.stations = j["stations"].get<int>();
    if (j.contains("trimmed")) q.trimmed = j["trimmed"].get<int>();
    if (j.contains("volume")) q.volume = N(j["volume"]);
    if (j.contains("line")) q.line = line3(j["line"]);
    q.id = j.at("id").get<int>();
    q.kind = j.at("kind").get<std::string>();
    for (const auto &r : j.at("triRanges")) q.triRanges.push_back({r[0].get<size_t>(), r[1].get<size_t>()});
    q.partAttached = j.value("partAttached", false);
    q.inBore = j.value("inBore", false);
    q.squat = j.value("squat", false);
    if (j.contains("triRange")) q.triRange = TriRange{j["triRange"][0].get<size_t>(), j["triRange"][1].get<size_t>()};
    q.raster = j.value("raster", false);
    return q;
}
void eqProp(const PropRecord &q, const json &j, const std::string &at) {
    eqI(q.region, j["region"].get<long>(), at + "region");
    eqI(q.tines, j.value("tines", 0), at + "tines");
    eqB(q.caps.has_value(), j.contains("caps"), at + "caps?");
    if (q.caps && j.contains("caps")) { eqI(long((*q.caps)[0]), j["caps"][0].get<long>(), at + "caps[0]"); eqI(long((*q.caps)[1]), j["caps"][1].get<long>(), at + "caps[1]"); }
    eqD(q.grip, j.contains("grip") ? N(j["grip"]) : INF, at + "grip");
    eqD(q.span, j.contains("span") ? N(j["span"]) : 0, at + "span");
    eqD(q.height, j.contains("height") ? N(j["height"]) : 0, at + "height");
    eqD(q.area, j.contains("area") ? N(j["area"]) : 0, at + "area");
    eqI(q.stations, j.value("stations", 0), at + "stations");
    eqB(q.trimmed.has_value(), j.contains("trimmed"), at + "trimmed?");
    if (q.trimmed && j.contains("trimmed")) eqI(*q.trimmed, j["trimmed"].get<long>(), at + "trimmed");
    eqD(q.volume, j.contains("volume") ? N(j["volume"]) : 0, at + "volume");
    eqLine(q.line, j.contains("line") ? j["line"] : json::array(), at + "line");
    eqI(q.id, j["id"].get<long>(), at + "id");
    eqS(q.kind, j["kind"].get<std::string>(), at + "kind");
    const json &tr = j["triRanges"];
    eqI(long(q.triRanges.size()), long(tr.size()), at + "triRanges.length");
    for (size_t i = 0; i < std::min(q.triRanges.size(), tr.size()); i++) {
        eqI(long(q.triRanges[i][0]), tr[i][0].get<long>(), at + "triRanges[" + std::to_string(i) + "][0]");
        eqI(long(q.triRanges[i][1]), tr[i][1].get<long>(), at + "triRanges[" + std::to_string(i) + "][1]");
    }
    eqB(q.partAttached, j.value("partAttached", false), at + "partAttached");
    eqB(q.inBore, j.value("inBore", false), at + "inBore");
    eqB(q.squat, j.value("squat", false), at + "squat");
    eqB(q.triRange.has_value(), j.contains("triRange"), at + "triRange?");
    if (q.triRange && j.contains("triRange")) { eqI(long((*q.triRange)[0]), j["triRange"][0].get<long>(), at + "triRange[0]"); eqI(long((*q.triRange)[1]), j["triRange"][1].get<long>(), at + "triRange[1]"); }
    eqB(q.raster, j.value("raster", false), at + "raster");
}

const char *SKIPPED_KEYS[] = {"noLine", "wanders", "stub", "blocked", "degenerate", "buried", "weld", "sliver", "onPart"};
int *skippedAt(Skipped &s, const std::string &k) {
    if (k == "noLine") return &s.noLine;
    if (k == "wanders") return &s.wanders;
    if (k == "stub") return &s.stub;
    if (k == "blocked") return &s.blocked;
    if (k == "degenerate") return &s.degenerate;
    if (k == "buried") return &s.buried;
    if (k == "weld") return &s.weld;
    if (k == "sliver") return &s.sliver;
    if (k == "onPart") return &s.onPart;
    throw std::runtime_error("unknown skipped key " + k);
}

PropsResult resultOf(const json &j) {
    PropsResult r;
    const json &t = j.at("triangles");
    for (size_t i = 0; i < t.size(); i += 3) r.triangles.push_back(Vec3d(N(t[i]), N(t[i + 1]), N(t[i + 2])));
    const std::string tags = j["tags"].get<std::string>();
    for (char c : tags) r.tags.push_back(uint8_t(c - '0'));
    for (const auto &q : j["props"]) r.props.push_back(propOf(q));
    for (const auto &[k, v] : j["skipped"].items()) *skippedAt(r.skipped, k) = v.get<int>();
    r.served = j["served"].get<int>();
    for (const auto &v : j["servedRegions"]) r.servedRegions.push_back(v.get<int>());
    r.tines = j["tines"].get<int>();
    r.sagRisk = j["sagRisk"].get<bool>();
    if (j.contains("sagRegions")) { r.sagRegions.emplace(); for (const auto &v : j["sagRegions"]) r.sagRegions->add(v.get<int>()); }
    r.volume = N(j["volume"]);
    if (j.contains("rasterRegions")) r.rasterRegions = j["rasterRegions"].get<int>();
    return r;
}
// `srcs`: the passes a raced result's `refs` index (JS vertex identity).
void eqResult(const PropsResult &r, const json &j, const std::vector<const PropsResult *> &srcs, const std::string &at) {
    eqI(long(r.props.size()), long(j["props"].size()), at + "props.length");
    for (size_t i = 0; i < std::min(r.props.size(), j["props"].size()); i++) eqProp(r.props[i], j["props"][i], at + "props[" + std::to_string(i) + "].");
    Skipped sk = r.skipped;
    for (const char *k : SKIPPED_KEYS) eqI(*skippedAt(sk, k), j["skipped"][k].get<long>(), at + "skipped." + k);
    eqI(long(j["skipped"].size()), 9, at + "skipped keys");
    eqI(r.served, j["served"].get<long>(), at + "served");
    eqI(long(r.servedRegions.size()), long(j["servedRegions"].size()), at + "servedRegions.length");
    for (size_t i = 0; i < std::min(r.servedRegions.size(), j["servedRegions"].size()); i++) eqI(r.servedRegions[i], j["servedRegions"][i].get<long>(), at + "servedRegions[i]");
    eqI(r.tines, j["tines"].get<long>(), at + "tines");
    eqB(r.sagRisk, j["sagRisk"].get<bool>(), at + "sagRisk");
    eqD(r.volume, j["volume"], at + "volume");
    eqB(r.sagRegions.has_value(), j.contains("sagRegions"), at + "sagRegions?");
    if (r.sagRegions && j.contains("sagRegions")) {
        eqI(long(r.sagRegions->order.size()), long(j["sagRegions"].size()), at + "sagRegions.size");
        for (size_t i = 0; i < std::min(r.sagRegions->order.size(), j["sagRegions"].size()); i++) eqI(r.sagRegions->order[i], j["sagRegions"][i].get<long>(), at + "sagRegions[i]");
    }
    eqB(r.rasterRegions.has_value(), j.contains("rasterRegions"), at + "rasterRegions?");
    if (r.rasterRegions && j.contains("rasterRegions")) eqI(*r.rasterRegions, j["rasterRegions"].get<long>(), at + "rasterRegions");
    eqI(long(r.triangles.size()), j["nTri"].get<long>(), at + "triangles.length");
    if (j.contains("triangles")) {
        eqTris(r.triangles, r.tags, 0, j["triangles"], j["tags"].get<std::string>(), at + "triangles");
    } else {
        // expand the refs: every vertex must be exactly the JS vertex it was copied from
        Tris want;
        for (const auto &run : j["refs"]) {
            const PropsResult &src = *srcs.at(run[0].get<size_t>());
            const size_t s = run[1].get<size_t>(), n = run[2].get<size_t>();
            for (size_t i = s; i < s + n; i++) want.push_back(src.triangles.at(i));
        }
        bool same = want.size() == r.triangles.size();
        for (size_t i = 0; same && i < want.size(); i++) same = want[i] == r.triangles[i];
        eqB(same, true, at + "triangles (by reference)");
        const std::string tags = j["tags"].get<std::string>();
        long tagFails = 0;
        for (size_t i = 0; i < std::min(tags.size(), r.triangles.size()); i++)
            if ((i < r.tags.size() ? r.tags[i] : 0) != tags[i] - '0') tagFails++;
        eqI(tagFails, 0, at + "tags differing");
    }
}

std::vector<TineCap> capsOf(const json &j) {
    std::vector<TineCap> v;
    for (const auto &c : j) v.push_back({N(c[0]), N(c[1]), N(c[2]), N(c[3]), N(c[4])});
    return v;
}
void eqCaps(const std::vector<TineCap> &cpp, const json &js, const std::string &what) {
    eqI(long(cpp.size()), long(js.size()), what + ".length");
    for (size_t i = 0; i < std::min(cpp.size(), js.size()); i++) {
        const TineCap &c = cpp[i];
        eqD(c.x, js[i][0], what + ".x"); eqD(c.y, js[i][1], what + ".y"); eqD(c.z, js[i][2], what + ".z");
        eqD(c.biteX, js[i][3], what + ".biteX"); eqD(c.biteY, js[i][4], what + ".biteY");
    }
}
PropRefs refsOf(const std::vector<PropRecord> &props, int region = -2) {
    PropRefs r;
    for (const PropRecord &q : props) if (region == -2 || q.region == region) r.push_back(&q);
    return r;
}

// ---- the cases -------------------------------------------------------------------------
void runAttached(const json &c, const json &cfg) {
    const ContactLine line = line3(c["line"]);
    const size_t from = c["from"].get<size_t>();
    // the oracle's PREFIX: two triangles, the second tagged
    Tris out = {Vec3d(5, 5, 0), Vec3d(6, 5, 0), Vec3d(5, 6, 0), Vec3d(7, 5, 0), Vec3d(8, 5, 0), Vec3d(7, 6, 0)};
    IfaceTags tags = {0, 0, 0, 1, 1, 1};
    const Tris pre = out;
    eqI(long(from), long(out.size()), "from");
    const PartAttached pa = buildPartAttached(line, *g_part, g_topo, g_rot, g_off, out, &tags);
    eqB(pa.ok, c["ok"].get<bool>(), "ok");
    eqB(pa.floored.has_value(), !c["floored"].is_null(), "floored?");
    if (pa.floored && !c["floored"].is_null()) eqS(*pa.floored, c["floored"].get<std::string>(), "floored");
    eqB(!pa.ok && !pa.floored, c["empty"].get<bool>(), "{}");
    eqB(pa.prop.has_value(), !c["prop"].is_null(), "prop?");
    if (pa.ok) S.attachedOk++;
    if (pa.floored) S.floored[*pa.floored]++;
    if (pa.prop && !c["prop"].is_null()) {
        const PartAttachedProp &p = *pa.prop;
        const json &j = c["prop"];
        eqD(p.span, j["span"], "prop.span"); eqD(p.height, j["height"], "prop.height");
        eqI(p.stations, j["stations"].get<long>(), "prop.stations"); eqD(p.volume, j["volume"], "prop.volume");
        eqB(p.partAttached, j["partAttached"].get<bool>(), "prop.partAttached");
        eqB(p.inBore, j["inBore"].get<bool>(), "prop.inBore");
        if (p.inBore) S.inBore++;
        eqLine(p.line, j["line"], "prop.line");
        eqI(long(j.size()), 7, "prop keys");
    }
    eqI(long(out.size()), c["wallEnd"].get<long>(), "wall end");
    bool preOk = out.size() >= from;
    for (size_t i = 0; preOk && i < from; i++) preOk = out[i] == pre[i] && tags[i] == (i >= 3 ? 1 : 0);
    eqB(preOk, true, "prefix untouched");
    eqB(tags.size() <= out.size(), true, "tags no longer than out");
    // prop.js's part-attached tine glue
    if (pa.ok && cfg["tines"].get<bool>()) {
        TineGrip grip;
        ContactLine topLine;
        for (const Vec3d &p : pa.prop->line) topLine.push_back(Vec3d(p[0], p[1], p[2] + PROP.gap));
        const double tineHeight = cfg["layerHeight"].is_null() ? PROP.tineH : N(cfg["layerHeight"]);
        const int nT = emitTines(topLine, g_part.get(), g_topo, g_rot, g_off, out, tineStepFor(std::nullopt), std::nullopt,
                                 tineHeight, std::nullopt, &grip, &tags);
        eqI(nT, c["nT"].get<long>(), "emitTines count");
        eqD(grip.z, c["grip"], "grip.z");
    } else {
        eqB(c.contains("nT"), false, "no tines");
    }
    eqTris(out, tags, from, c["out"], c["tags"].get<std::string>(), "out");

    // the settled top / floor, and the internals on them
    ContactLine top = line;
    contourTop(top, *g_part, PART_BAND);
    lowerSag(top, *g_part, PART_BAND);
    settleTop(top, *g_part, 0.25, PART_BAND);
    const FloorLine floor = floorLine(top, *g_part);
    eqLine(top, c["top"], "top");
    eqFloor(floor, c["floor"], "floor");
    if (c.contains("buried") && top.size() == c["top"].size() && floor.size() == top.size()) {
        const ContactLine jt = line3(c["top"]);
        for (size_t k = 0; k < top.size(); k++) {
            const std::string at = "[" + std::to_string(k) + "]";
            eqB(buriedColumn(top, floor, k, g_topo, g_rot, g_off), c["buried"][k].get<bool>(), "buriedColumn" + at);
            eqB(clearBetween(top, floor, k, g_topo, g_rot, g_off), c["clear"][k].get<bool>(), "clearBetween" + at);
            eqB(enclosedColumn(top, floor, k, g_topo, g_rot, g_off), c["enclosed"][k].get<bool>(), "enclosedColumn" + at);
        }
    }
}

void runRasterTracks(const json &c) {
    const int ri = c["region"].get<int>();
    const RegionData &R = g_regions.at(size_t(ri));
    Support support;
    support.topo = &g_topo; support.rot = g_rot; support.offset = g_off;
    const Tracks lines = rasterTracks(R.pts, *R.tris, *g_part, N(c["step"]), N(c["rowSpan"]), &support);
    eqI(long(lines.size()), long(c["lines"].size()), "lines");
    for (size_t i = 0; i < std::min(lines.size(), c["lines"].size()); i++) eqLine(lines[i], c["lines"][i], "line[" + std::to_string(i) + "]");
    eqB(lines.spacing.has_value(), !c["spacing"].is_null(), "spacing?");
    if (lines.spacing && !c["spacing"].is_null()) eqD(*lines.spacing, c["spacing"], "spacing");
}

void runRasterRest(const json &c) {
    const ContactLine line = line3(c["line"]);
    for (const json &r : c["runs"]) {
        const Run run{r["run"][0].get<int>(), r["run"][1].get<int>()};
        const std::vector<RasterPiece> rest = rasterRest(line, run);
        const std::string at = "run [" + std::to_string(run[0]) + "," + std::to_string(run[1]) + "] ";
        eqI(long(rest.size()), long(r["rest"].size()), at + "pieces");
        for (size_t i = 0; i < std::min(rest.size(), r["rest"].size()); i++) {
            eqI(rest[i].from, r["rest"][i]["from"].get<long>(), at + "from");
            eqLine(rest[i], r["rest"][i]["line"], at + "piece");
        }
    }
}

void runRace(const json &c, const json &cfg) {
    const PropsResult normal = resultOf(c["normal"]);
    const bool hasRec = c.contains("raster");
    const PropsResult rasterRes = hasRec ? resultOf(c["raster"]) : PropsResult{};
    const json want = c["want"];
    eqB(hasRec, !want.empty(), "rasterPass called iff anything wanted");

    // the tine capture as buildProps leaves it after the normal pass
    std::vector<TineCap> cap;
    const bool capOn = !c["capBefore"].is_null();
    if (capOn) cap = capsOf(c["capBefore"]);
    TINECAP = capOn ? &cap : nullptr;

    // the veto: the JS decisions replayed, in call order (they pushed tine captures too)
    RasterVeto veto;
    size_t vetoAt = 0;
    const bool vetoOn = cfg["veto"].is_string();
    const json vetoCalls = c["vetoCalls"];
    if (vetoOn) {
        veto = [&](const PropRecord &q, const std::vector<PropRecord> &np) {
            if (vetoCalls.is_null() || vetoAt >= vetoCalls.size()) { fail("veto called more than in JS"); return false; }
            const json &v = vetoCalls[vetoAt++];
            eqI(q.id, v["id"].get<long>(), "veto q.id");
            eqI(q.region, v["region"].get<long>(), "veto q.region");
            eqI(long(np.size()), long(normal.props.size()), "veto normalProps");
            if (!v["caps"].is_null()) {
                if (!TINECAP) fail("veto: capture off in C++, on in JS");
                else for (const TineCap &t : capsOf(v["caps"])) TINECAP->push_back(t);
            }
            if (v["d"].get<bool>()) S.vetoed++;
            return v["d"].get<bool>();
        };
    }
    int passes = 0;
    const RasterPass pass = [&](const RegionSet &regions) {
        passes++;
        eqI(long(regions.order.size()), long(want.size()), "want.size");
        for (size_t i = 0; i < std::min(regions.order.size(), want.size()); i++) eqI(regions.order[i], want[i].get<long>(), "want[i]");
        // the raster pass captures into a fresh array
        eqB(TINECAP != nullptr, capOn, "raster pass capture on");
        if (TINECAP) {
            eqB(TINECAP != &cap && TINECAP->empty(), true, "raster pass capture fresh");
            if (!c["rasterCaps"].is_null()) for (const TineCap &t : capsOf(c["rasterCaps"])) TINECAP->push_back(t);
        }
        return rasterRes;
    };
    const PropsResult raced = withRaster(g_topo, g_res, g_rot, veto, normal, c["capFrom"].get<size_t>(), pass);
    eqI(passes, hasRec ? 1 : 0, "rasterPass calls");
    if (vetoOn && hasRec) eqI(long(vetoAt), long(vetoCalls.size()), "veto calls");
    eqB(TINECAP == (capOn ? &cap : nullptr), true, "capture restored");
    TINECAP = nullptr;
    if (capOn) eqCaps(cap, c["capAfter"], "capAfter");
    else eqB(c["capAfter"].is_null(), true, "capAfter null");

    // b: the raster props raceRegions saw (the vetoed ones), or the all-regions pass
    PropsResult b = hasRec ? rasterRes : resultOf(c["rasterAll"]);
    if (hasRec && vetoOn && !vetoCalls.is_null()) {
        std::vector<PropRecord> kept;
        for (size_t i = 0; i < b.props.size(); i++) if (!(i < vetoCalls.size() && vetoCalls[i]["d"].get<bool>())) kept.push_back(b.props[i]);
        b.props = std::move(kept);
    }
    if (c["same"].get<bool>()) {
        eqResult(raced, c["normal"], {}, "same: ");
    } else {
        S.swaps++;
        eqResult(raced, c["raced"], {&normal, &b}, "raced: ");
    }
    if (c.contains("direct")) {
        const PropsResult direct = raceRegions(overhangFaces(g_topo, g_res, g_rot), g_res, normal, b);
        eqResult(direct, c["direct"], {&normal, &b}, "direct: ");
    }

    // the internals
    const std::vector<OverFace> faces = overhangFaces(g_topo, g_res, g_rot);
    eqI(long(faces.size()), long(c["faces"].size()), "faces");
    for (size_t i = 0; i < std::min(faces.size(), c["faces"].size()); i++) {
        const json &f = c["faces"][i];
        eqD(faces[i].cx, f[0], "face.cx"); eqD(faces[i].cy, f[1], "face.cy"); eqD(faces[i].cz, f[2], "face.cz");
        eqD(faces[i].area, f[3], "face.area"); eqI(faces[i].region, f[4].get<long>(), "face.region");
    }
    const RegionSet wanted = rasterWanted(faces, g_res, normal);
    eqI(long(wanted.order.size()), long(c["wanted"].size()), "rasterWanted.size");
    for (size_t i = 0; i < std::min(wanted.order.size(), c["wanted"].size()); i++) eqI(wanted.order[i], c["wanted"][i].get<long>(), "rasterWanted[i]");
    const std::vector<Vec3d> normalTops = wallTops(refsOf(normal.props));
    for (const json &e : c["regions"]) {
        const int ri = e["ri"].get<int>();
        const std::string at = "region " + std::to_string(ri) + " ";
        const PropRefs qa = refsOf(normal.props, ri), qb = refsOf(b.props, ri);
        const std::vector<OverFace> near = nearRegion(faces, ri);
        eqI(long(near.size()), e["near"].get<long>(), at + "nearRegion");
        eqD(heldBy(near, normalTops), e["nearHeld"], at + "heldBy(near, all)");
        eqD(heldBy(near, wallTops(qa)), e["ha"], at + "ha");
        eqD(heldBy(near, wallTops(qb)), e["hb"], at + "hb");
        eqB(e.contains("keepers"), !qb.empty(), at + "keepers run");
        if (e.contains("keepers")) {
            const std::optional<PropRefs> k = keepers(qa, qb, near);
            eqB(k.has_value(), !e["keepers"].is_null(), at + "keepers null");
            if (k && !e["keepers"].is_null()) {
                eqI(long(k->size()), long(e["keepers"].size()), at + "keepers.length");
                for (size_t i = 0; i < std::min(k->size(), e["keepers"].size()); i++) {
                    const long idx = long(std::find(qa.begin(), qa.end(), (*k)[i]) - qa.begin());
                    eqI(idx, e["keepers"][i].get<long>(), at + "keepers[i]");
                }
            }
        }
    }
    const size_t na = std::min<size_t>(30, normal.props.size()), nb = std::min<size_t>(30, b.props.size());
    for (size_t i = 0; i < na; i++) {
        for (size_t k = 0; k < nb; k++) eqD(planGap(normal.props[i].line, b.props[k].line), c["planGap"][i][k], "planGap");
        eqD(halfFoot(normal.props[i]), c["halfFoot"][0][i], "halfFoot(normal)");
        eqD(gripZ({&normal.props[i]}), c["gripZ"][0][i], "gripZ([q])");
    }
    for (size_t k = 0; k < nb; k++) eqD(halfFoot(b.props[k]), c["halfFoot"][1][k], "halfFoot(raster)");
    eqD(gripZ(refsOf(normal.props)), c["gripZ"][1], "gripZ(normal)");
    eqD(gripZ(refsOf(b.props)), c["gripZ"][2], "gripZ(raster)");
    eqD(gripZ({}), c["gripZ"][3], "gripZ([])");
    const double zs[] = {0, 0.5, 2, 40, INF};
    for (int i = 0; i < 5; i++) eqD(gripRise(zs[i]), c["gripRise"][i], "gripRise");
}

void runFile(const fs::path &file) {
    std::ifstream in(file);
    const json d = json::parse(in);
    const std::string scene = d["scene"].get<std::string>();
    checkConsts(d);

    Soup pos; pos.reserve(d["pos"].size());
    for (const auto &v : d["pos"]) pos.push_back(N(v));
    g_topo = buildTopology(std::move(pos));
    g_res = analyze(g_topo, N(d["threshold"]), IDENTITY3);
    g_off = g_res.offset;
    g_fn = "scene"; g_ctx = scene;
    eqD(g_off.x, d["offset"]["x"], "offset.x"); eqD(g_off.y, d["offset"]["y"], "offset.y"); eqD(g_off.z, d["offset"]["z"], "offset.z");
    eqI(long(g_res.regions.size()), d["nRegions"].get<long>(), "regions");

    // prop.js buildPass: the seated part, then per region its tris / pts
    const Soup &P = g_topo.pos;
    Soup part(P.size());
    Vec3d v;
    for (int f = 0; f < g_topo.nFaces; f++) for (int i = 0; i < 3; i++) {
        seat(P, size_t(f) * 9 + size_t(i) * 3, g_rot, g_off, v);
        for (int k = 0; k < 3; k++) part[size_t(f) * 9 + size_t(i) * 3 + size_t(k)] = v[k];
    }
    g_part = std::make_shared<SurfTris>(std::move(part));
    g_regions.clear();
    for (const Region &g : g_res.regions) {
        RegionData R;
        Soup rt(g.faces.size() * 9);
        for (size_t k = 0; k < g.faces.size(); k++) {
            R.area += g_topo.area[size_t(g.faces[k])];
            double gx = 0, gy = 0, gz = 0;
            for (int i = 0; i < 3; i++) {
                seat(P, size_t(g.faces[k]) * 9 + size_t(i) * 3, g_rot, g_off, v);
                for (int c = 0; c < 3; c++) rt[k * 9 + size_t(i) * 3 + size_t(c)] = v[c];
                R.pts.push_back(v);
                gx += v[0]; gy += v[1]; gz += v[2];
            }
            R.pts.push_back(Vec3d(gx / 3, gy / 3, gz / 3));
        }
        R.tris = std::make_shared<SurfTris>(std::move(rt));
        g_regions.push_back(std::move(R));
    }

    int ci = 0;
    long curCfg = -1;
    for (const json &c : d["cases"]) {
        const std::string fn = c["fn"].get<std::string>();
        g_fn = fn;
        const long cfg = c["cfg"].get<long>();
        g_ctx = scene + " #" + std::to_string(ci++) + " " + fn + " cfg" + std::to_string(cfg)
              + (c.contains("src") ? "/" + c["src"].get<std::string>() : "")
              + (c.contains("region") ? " r" + std::to_string(c["region"].get<long>()) : "");
        S.casesByFn[fn]++;
        const json &cfgJ = d["cfgs"][size_t(cfg)];
        if (cfg != curCfg) { applyCfg(cfgJ); curCfg = cfg; }
        if (fn == "attached") runAttached(c, cfgJ);
        else if (fn == "rasterTracks") runRasterTracks(c);
        else if (fn == "rasterRest") runRasterRest(c);
        else if (fn == "race") runRace(c, cfgJ);
        else fail("unknown case fn " + fn);
    }
    PROP = PropConfig{}; crestPart(nullptr); g_part.reset(); g_regions.clear(); TINECAP = nullptr;
}

} // namespace

int main(int argc, char **argv) {
    const fs::path dir = argc > 1 ? fs::path(argv[1]) : fs::path(__FILE__).parent_path().parent_path() / "refs_attached";
    if (!fs::is_directory(dir)) {
        std::fprintf(stderr, "no dumps in %s: run SUPPORT_FINS_DIR=... node tools/attached_ref.mjs\n", dir.string().c_str());
        return 1;
    }
    std::vector<fs::path> files;
    for (const auto &e : fs::directory_iterator(dir)) if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::fprintf(stderr, "no dumps in %s\n", dir.string().c_str()); return 1; }

    // rasterRest / planGap / gripZ edge cases the scenes do not reach
    {
        g_fn = "edge"; g_ctx = "edge";
        PROP = PropConfig{};
        eqI(long(rasterRest({}, Run{0, 0}).size()), 0, "rasterRest(empty)");
        eqD(planGap({}, {Vec3d(0, 0, 0)}), INF, "planGap(empty)");
        eqD(planGap({Vec3d(3, 4, 0)}, {Vec3d(0, 0, 9)}), 5, "planGap(points)");
        eqD(planGap({Vec3d(1, 1, 0)}, {Vec3d(0, 0, 0), Vec3d(0, 0, 0)}), hypot2(1, 1), "planGap(zero segment)");
        eqD(gripZ({}), INF, "gripZ([])");
    }

    for (const fs::path &f : files) {
        const long before = S.fails;
        runFile(f);
        std::printf("%-24s %s\n", f.stem().string().c_str(), S.fails == before ? "ok" : "FAIL");
    }
    std::printf("cases:");
    for (const auto &[fn, n] : S.casesByFn) std::printf(" %s=%ld", fn.c_str(), n);
    std::printf("\npart-attached walls %ld (inBore %ld), floored:", S.attachedOk, S.inBore);
    for (const auto &[w, n] : S.floored) std::printf(" %s=%ld", w.c_str(), n);
    std::printf("; races swapped %ld, raster walls vetoed %ld\n", S.swaps, S.vetoed);
    std::printf("checks %ld, failures %ld, non-bit-identical doubles %ld (max diff %.3g)\n",
                S.checks, S.fails, S.inexact, S.maxDiff);
    for (const auto &[fn, n] : S.failsByFn) std::printf("  failures in %s: %ld\n", fn.c_str(), n);
    return S.fails ? 1 : 0;
}
