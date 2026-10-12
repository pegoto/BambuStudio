// Pins Engine/Fins (web/fins.js buildFins / applyTunables / coverPitch) and its helpers
// Engine/Fins/Shortwalls (web/fins/shortwalls.js) and Engine/Fins/Fill (web/fins/fill.js)
// against the upstream JS, on what the end-to-end parity dumps (tools/ref.mjs) do not
// reach: modes prop / full / draw / stabilize / unknown, lastResort and raster off, the
// pad off, tunables with padCustom / iface / tinesPerWall / every pad style / cutouts,
// sway with every option, the point-seated gate, the fill pass's caps, and the fill and
// short-wall internals. The oracle is tools/fins_ref.mjs, one JSON per scene in
// refs_fins/ (next to tests/):
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/fins_ref.mjs
//   ./build-fins/test_fins [refs_fins_dir]
// Triangles are compared through hashes of their float64 bit patterns (whole list, per
// fin record) and the interface tags through a hash and a count; every record field,
// line, stat and module-state value is compared in full (doubles within 1e-9, and the
// not-bit-identical ones counted).
#include "Engine/Cutout.hpp"
#include "Engine/Entry.hpp"
#include "Engine/FinConfig.hpp"
#include "Engine/Materials.hpp"
#include "Engine/Fins.hpp"
#include "Engine/Fins/Coverage.hpp"
#include "Engine/Fins/Fill.hpp"
#include "Engine/Fins/Pad.hpp"
#include "Engine/Fins/Shortwalls.hpp"
#include "Engine/Fins/Wedges.hpp"
#include "Engine/Overhangs.hpp"
#include "Engine/Prop/Crest.hpp"
#include "Engine/PropConfig.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using namespace Slic3r::SupportFins;
using json = nlohmann::ordered_json;   // keeps a JS object's key order (fill's `refused`)
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
    if (S.fails <= 60) std::fprintf(stderr, "FAIL [%s] %s\n", g_ctx.c_str(), what.c_str());
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
void eqU(uint32_t cpp, const json &js, const std::string &what) {
    S.checks++;
    if (cpp != js.get<uint32_t>()) fail(what + ": cpp " + std::to_string(cpp) + " js " + std::to_string(js.get<uint32_t>()));
}
void eqB(bool cpp, bool js, const std::string &what) { eqI(cpp ? 1 : 0, js ? 1 : 0, what); }
void eqS(const std::string &cpp, const std::string &js, const std::string &what) {
    S.checks++;
    if (cpp != js) fail(what + ": cpp \"" + cpp + "\" js \"" + js + "\"");
}
// JS `x ?? false` on an optional boolean field
bool B(const json &j) { return j.is_boolean() ? j.get<bool>() : false; }
Vec3d V3(const json &p) { return Vec3d(N(p[0]), N(p[1]), N(p[2])); }

void eqLine(const std::vector<Vec3d> &cpp, const json &js, const std::string &what) {
    if (js.is_null()) { eqI(long(cpp.size()), 0, what + " (null)"); return; }
    if (cpp.size() != js.size()) { fail(what + ": length cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    for (size_t i = 0; i < cpp.size(); i++)
        for (int k = 0; k < 3; k++) eqD(cpp[i][k], js[i][k], what + "[" + std::to_string(i) + "][" + std::to_string(k) + "]");
}
void eqRanges(const std::vector<TriRange> &cpp, const json &js, const std::string &what) {
    if (cpp.size() != js.size()) { fail(what + ": count cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    for (size_t i = 0; i < cpp.size(); i++) {
        eqI(long(cpp[i][0]), js[i][0].get<long>(), what + "[" + std::to_string(i) + "][0]");
        eqI(long(cpp[i][1]), js[i][1].get<long>(), what + "[" + std::to_string(i) + "][1]");
    }
}
void eqInts(const std::vector<int> &cpp, const json &js, const std::string &what) {
    if (cpp.size() != js.size()) { fail(what + ": count cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    long bad = 0;
    for (size_t i = 0; i < cpp.size(); i++) if (cpp[i] != js[i].get<int>()) bad++;
    eqI(bad, 0, what + " mismatched entries");
}

// ---- the dump's hashes (tools/fins_ref.mjs hashTris / hashTags) --------------------
uint32_t hv(uint32_t h, double x) {
    uint32_t w[2];
    std::memcpy(w, &x, 8);   // little-endian, as the JS Uint32Array view over a Float64Array
    h = (h ^ w[0]) * 16777619u;
    return (h ^ w[1]) * 16777619u;
}
uint32_t hashTris(const Tris &t, size_t a, size_t b) {
    uint32_t h = 2166136261u;
    for (size_t i = a; i < b && i < t.size(); i++) { h = hv(h, t[i][0]); h = hv(h, t[i][1]); h = hv(h, t[i][2]); }
    return h;
}
void eqTris(const Tris &t, const IfaceTags *tags, const json &j, const std::string &what) {
    eqI(long(t.size()), j["n"].get<long>(), what + " count");
    eqU(hashTris(t, 0, t.size()), j["hash"], what + " hash");
    if (!tags) return;
    uint32_t h = 2166136261u;
    long n = 0;
    for (size_t i = 0; i < t.size(); i++) {
        const uint32_t v = i < tags->size() ? (*tags)[i] : 0;
        n += v;
        h = (h ^ v) * 16777619u;
    }
    eqI(n, j["tags"]["n"].get<long>(), what + " tagged");
    eqU(h, j["tags"]["h"], what + " tags hash");
}

// ---- module state -----------------------------------------------------------------
void resetState() {
    PROP = PropConfig{}; PERP = PerpConfig{}; CUT = CutConfig{}; FIN = FinConfig{}; PAD = PadConfig{}; FILL = FillConfig{};
    crestPart(nullptr);
}
std::string ifaceName(Iface i) { return i == Iface::Flat ? "flat" : i == Iface::All ? "all" : "off"; }
void eqState(const json &s) {
    eqD(FIN.padH, s["FINpadH"], "FIN.padH");
    eqD(PAD.grab, s["PADgrab"], "PAD.grab");
    eqS(PAD.style, s["PADstyle"].get<std::string>(), "PAD.style");
    eqD(PAD.custom.h, s["PADcustom"]["h"], "PAD.custom.h");
    eqD(PAD.custom.gap, s["PADcustom"]["gap"], "PAD.custom.gap");
    eqD(PAD.custom.grip, s["PADcustom"]["grip"], "PAD.custom.grip");
    eqD(PAD.custom.margin, s["PADcustom"]["margin"], "PAD.custom.margin");
    eqD(PROP.gap, s["PROPgap"], "PROP.gap");
    eqD(PERP.gap, s["PERPgap"], "PERP.gap");
    eqS(ifaceName(PROP.iface), s["PROPiface"].get<std::string>(), "PROP.iface");
    eqI(PROP.tinesPerWall, s["PROPtinesPerWall"].get<long>(), "PROP.tinesPerWall");
    eqS(CUT.pattern, s["CUTpattern"].get<std::string>(), "CUT.pattern");
}

// ---- options ----------------------------------------------------------------------
BuildOpts optsOf(const json &b) {
    const json &o = b["opts"];
    BuildOpts bo;
    bo.mode = o["mode"].get<std::string>();
    bo.bedPad = o["bedPad"].get<bool>();
    bo.tines = o["tines"].get<bool>();
    bo.tineDensity = N(o["tineDensity"]);
    bo.layerHeight = N(o["layerHeight"]);
    bo.coverage = N(o["coverage"]);
    bo.plateOnly = o["plateOnly"].get<bool>();
    bo.lastResort = o["lastResort"].get<bool>();
    bo.raster = o["raster"].get<bool>();
    if (o.contains("sway") && !o["sway"].is_null()) {
        const json &w = o["sway"];
        SwayOpts s;
        if (w.contains("on")) s.on = w["on"].get<bool>();
        if (w.contains("gripFrom")) s.gripFrom = N(w["gripFrom"]);
        if (w.contains("tineSpacing")) s.tineSpacing = N(w["tineSpacing"]);
        if (w.contains("reach")) s.reach = N(w["reach"]);
        if (w.contains("gap")) s.gap = N(w["gap"]);
        bo.sway = s;
    }
    const json &t = b["tunables"];
    if (!t.is_null()) {
        Tunables tu;
        tu.padH = N(t["padH"]); tu.padGrab = N(t["padGrab"]); tu.propGap = N(t["propGap"]);
        tu.padStyle = t["padStyle"].get<std::string>();
        tu.cutout = t["cutout"].get<std::string>();
        if (t.contains("padCustom")) {
            Tunables::PadCustom c;
            const json &pc = t["padCustom"];
            c.h = N(pc["h"]); c.gap = N(pc["gap"]); c.grip = N(pc["grip"]); c.margin = N(pc["margin"]);
            tu.padCustom = c;
        }
        if (t.contains("iface")) {
            const json &i = t["iface"];
            if (i.is_boolean()) { if (!i.get<bool>()) tu.iface = Iface::Off; }
            else tu.iface = i.get<std::string>() == "all" ? Iface::All : Iface::Flat;
        }
        if (t.contains("tinesPerWall")) tu.tinesPerWall = t["tinesPerWall"].get<int>();
        bo.tunables = tu;
    }
    return bo;
}
void applyFillCfg(const json &f) {
    if (f.is_null()) return;
    if (f.contains("maxWalls")) FILL.maxWalls = f["maxWalls"].get<int>();
    if (f.contains("maxTries")) FILL.maxTries = f["maxTries"].get<int>();
    if (f.contains("groupTries")) FILL.groupTries = f["groupTries"].get<int>();
    if (f.contains("maxChecks")) FILL.maxChecks = f["maxChecks"].get<int>();
}

// ---- records ----------------------------------------------------------------------
void eqProp(const BuiltProp &q, const json &j, const std::string &w) {
    if (!j["region"].is_null()) eqI(q.region, j["region"].get<long>(), w + ".region");
    if (!j["tines"].is_null()) eqI(q.tines, j["tines"].get<long>(), w + ".tines");
    if (!j["span"].is_null()) eqD(q.span, j["span"], w + ".span");
    if (!j["height"].is_null()) eqD(q.height, j["height"], w + ".height");
    if (!j["area"].is_null()) eqD(q.area, j["area"], w + ".area");
    if (!j["volume"].is_null()) eqD(q.volume, j["volume"], w + ".volume");
    if (!j["grip"].is_null()) eqD(q.grip, j["grip"], w + ".grip");
    if (!j["stations"].is_null()) eqI(q.stations, j["stations"].get<long>(), w + ".stations");
    if (!j["trimmed"].is_null()) eqI(q.trimmed.value_or(-999), j["trimmed"].get<long>(), w + ".trimmed");
    if (!j["id"].is_null()) eqI(q.id, j["id"].get<long>(), w + ".id");
    eqLine(q.line, j["line"], w + ".line");
    eqRanges(q.triRanges, j["triRanges"], w + ".triRanges");
    eqB(q.partAttached, B(j["partAttached"]), w + ".partAttached");
    eqB(q.inBore, B(j["inBore"]), w + ".inBore");
    eqB(q.squat, B(j["squat"]), w + ".squat");
    eqB(q.raster, B(j["raster"]), w + ".raster");
    eqB(q.isShort, B(j["short"]), w + ".short");
    eqB(q.fill, B(j["fill"]), w + ".fill");
    eqB(q.joined, B(j["joined"]), w + ".joined");
    eqB(q.braces.has_value(), !j["braces"].is_null(), w + ".braces set");
    if (q.braces && !j["braces"].is_null()) eqI(*q.braces, j["braces"].get<long>(), w + ".braces");
    eqB(q.ties.has_value(), !j["ties"].is_null(), w + ".ties set");
    if (q.ties && !j["ties"].is_null()) eqI(*q.ties, j["ties"].get<long>(), w + ".ties");
}
void eqProps(const std::vector<BuiltProp> &ps, const json &j, const std::string &w) {
    if (ps.size() != j.size()) { fail(w + " count: cpp " + std::to_string(ps.size()) + " js " + std::to_string(j.size())); return; }
    for (size_t i = 0; i < ps.size(); i++) eqProp(ps[i], j[i], w + "[" + std::to_string(i) + "]");
}
void eqRefused(const std::vector<std::pair<std::string, int>> &r, const json &j, const std::string &w) {
    if (r.size() != j.size()) { fail(w + " keys: cpp " + std::to_string(r.size()) + " js " + std::to_string(j.size())); return; }
    size_t i = 0;
    for (auto it = j.begin(); it != j.end(); ++it, ++i) {
        eqS(r[i].first, it.key(), w + " key " + std::to_string(i));
        eqI(r[i].second, it.value().get<long>(), w + "[" + it.key() + "]");
    }
}

void eqBuilt(const Built &b, const json &j) {
    eqTris(b.triangles, &b.tags, j["triangles"], "triangles");
    eqTris(b.padTriangles, nullptr, j["padTriangles"], "padTriangles");
    const json &pad = j["pad"];
    eqB(b.pad, !pad.is_null(), "pad built");
    eqB(b.padRecord.has_value(), !pad.is_null(), "pad record");
    if (b.padRecord && !pad.is_null()) {
        const PadResult &p = *b.padRecord;
        eqD(p.r1, pad["r1"], "pad.r1"); eqD(p.r2, pad["r2"], "pad.r2");
        eqI(p.cells, pad["cells"].get<long>(), "pad.cells"); eqD(p.height, pad["height"], "pad.height");
        eqI(p.points, pad["points"].get<long>(), "pad.points"); eqB(p.oval, pad["oval"].get<bool>(), "pad.oval");
        eqS(p.style, pad["style"].get<std::string>(), "pad.style");
        eqB(p.autoSure.has_value(), pad.contains("autoSure"), "pad.autoSure set");
        if (p.autoSure && pad.contains("autoSure")) eqB(*p.autoSure, pad["autoSure"].get<bool>(), "pad.autoSure");
        eqB(p.smallFoot, pad["smallFoot"].get<bool>(), "pad.smallFoot");
        eqD(p.outline, pad["outline"], "pad.outline");
    }
    eqS(b.mode, j["mode"].get<std::string>(), "mode");
    const json &fins = j["fins"];
    if (b.fins.size() != fins.size()) fail("fins count: cpp " + std::to_string(b.fins.size()) + " js " + std::to_string(fins.size()));
    else for (size_t i = 0; i < fins.size(); i++) {
        const FinRecord &f = b.fins[i];
        const json &F = fins[i];
        const std::string w = "fins[" + std::to_string(i) + "]";
        eqI(f.id, F["id"].get<long>(), w + ".id");
        eqS(f.kind, F["kind"].get<std::string>(), w + ".kind");
        eqD(f.height, F["height"], w + ".height");
        eqD(f.length, F["length"], w + ".length");
        if (!F["span"].is_null()) eqD(f.span, F["span"], w + ".span");
        eqI(f.tines, F["tines"].get<long>(), w + ".tines");
        eqRanges(f.triRanges, F["triRanges"], w + ".triRanges");
        eqLine(f.line, F["line"], w + ".line");
        eqB(f.fill, F["fill"].get<bool>(), w + ".fill");
        eqB(f.joined, F["joined"].get<bool>(), w + ".joined");
        eqB(f.isShort, F["short"].get<bool>(), w + ".short");
        for (size_t r = 0; r < std::min(f.triRanges.size(), F["hashes"].size()); r++)
            eqU(hashTris(b.triangles, f.triRanges[r][0], f.triRanges[r][1]), F["hashes"][r], w + " range " + std::to_string(r) + " hash");
    }
    eqProps(b.props, j["props"], "props");
    if (!j["volume"].is_null()) eqD(b.volume, j["volume"], "volume");
    if (!j["skipped"].is_null()) {
        const json &s = j["skipped"];
        const Skipped &k = b.skipped;
        eqI(k.noLine, s["noLine"].get<long>(), "skipped.noLine"); eqI(k.wanders, s["wanders"].get<long>(), "skipped.wanders");
        eqI(k.stub, s["stub"].get<long>(), "skipped.stub"); eqI(k.blocked, s["blocked"].get<long>(), "skipped.blocked");
        eqI(k.degenerate, s["degenerate"].get<long>(), "skipped.degenerate"); eqI(k.buried, s["buried"].get<long>(), "skipped.buried");
        eqI(k.weld, s["weld"].get<long>(), "skipped.weld"); eqI(k.sliver, s["sliver"].get<long>(), "skipped.sliver");
        eqI(k.onPart, s["onPart"].get<long>(), "skipped.onPart");
    }
    if (!j["servedRegions"].is_null()) eqInts(b.servedRegions, j["servedRegions"], "servedRegions");
    // (prop / draw mode: JS leaves braceCount / propCount undefined, read as 0 by the entry)
    eqI(b.braceCount, j["braceCount"].is_null() ? 0 : j["braceCount"].get<long>(), "braceCount");
    eqI(b.propCount, j["propCount"].is_null() ? 0 : j["propCount"].get<long>(), "propCount");
    eqI(b.tines, j["tines"].get<long>(), "tines");
    eqI(b.unserved, j["unserved"].get<long>(), "unserved");
    const json &sw = j["sway"];
    eqB(b.sway.has_value(), !sw.is_null(), "sway set");
    if (b.sway && !sw.is_null()) {
        eqI(b.sway->count, sw["count"].get<long>(), "sway.count");
        eqI(b.sway->tines, sw["tines"].get<long>(), "sway.tines");
        eqI(b.sway->skipped, sw["skipped"].get<long>(), "sway.skipped");
        eqS(b.sway->reason.value_or("<null>"), sw["reason"].is_null() ? "<null>" : sw["reason"].get<std::string>(), "sway.reason");
        const json &br = sw["braces"];
        if (b.sway->braces.size() != br.size()) fail("sway.braces count");
        else for (size_t i = 0; i < br.size(); i++) {
            const SwayOutline &o = b.sway->braces[i];
            const std::string w = "sway.braces[" + std::to_string(i) + "]";
            eqLine(o.foot, br[i]["foot"], w + ".foot");
            eqD(o.halfW, br[i]["halfW"], w + ".halfW"); eqD(o.th, br[i]["th"], w + ".th"); eqD(o.height, br[i]["height"], w + ".height");
            const json &lv = br[i]["levels"];
            if (o.levels.size() != lv.size()) { fail(w + ".levels count"); continue; }
            for (size_t k = 0; k < lv.size(); k++) {
                eqD(o.levels[k].z, lv[k]["z"], w + ".levels.z");
                for (int c = 0; c < 3; c++) { eqD(o.levels[k].a[c], lv[k]["a"][c], w + ".levels.a"); eqD(o.levels[k].b[c], lv[k]["b"][c], w + ".levels.b"); }
            }
        }
    }
    const json &fl = j["fill"];
    eqB(b.fill.has_value(), !fl.is_null(), "fill set");
    if (b.fill && !fl.is_null()) {
        const FillRecord &f = *b.fill;
        eqI(f.walls, fl["walls"].get<long>(), "fill.walls");
        eqI(f.joined, fl["joined"].get<long>(), "fill.joined");
        eqB(f.ribs.has_value(), fl.contains("ribs") && !fl["ribs"].is_null(), "fill.ribs set");
        if (f.ribs && fl.contains("ribs") && !fl["ribs"].is_null()) eqI(*f.ribs, fl["ribs"].get<long>(), "fill.ribs");
        eqB(f.ties.has_value(), fl.contains("ties") && !fl["ties"].is_null(), "fill.ties set");
        if (f.ties && fl.contains("ties") && !fl["ties"].is_null()) eqI(*f.ties, fl["ties"].get<long>(), "fill.ties");
        eqI(f.tries, fl["tries"].get<long>(), "fill.tries");
        eqI(f.checks, fl["checks"].get<long>(), "fill.checks");
        eqB(f.capped, fl["capped"].get<bool>(), "fill.capped");
        eqRefused(f.refused, fl["refused"], "fill.refused");
        eqD(f.bareBefore, fl["bareBefore"], "fill.bareBefore");
        eqD(f.unservedArea, fl["unservedArea"], "fill.unservedArea");
        eqLine(f.unservedPts, fl["unservedPts"], "fill.unservedPts");
    }
    const json &fp = j["floating"];
    if (b.floating.size() != fp.size()) fail("floating count");
    else for (size_t i = 0; i < fp.size(); i++) {
        eqI(b.floating[i].faces, fp[i]["faces"].get<long>(), "floating.faces");
        for (int k = 0; k < 3; k++) eqD(b.floating[i].lowest[k], fp[i]["lowest"][k], "floating.lowest");
        eqD(b.floating[i].drop, fp[i]["drop"], "floating.drop");
    }
    const json &se = j["seating"];
    if (!se.is_null()) {
        eqS(b.seating.kind, se["kind"].get<std::string>(), "seating.kind");
        eqS(b.seatingKind, se["kind"].get<std::string>(), "seatingKind");
        eqD(b.seating.span, se["span"], "seating.span");
        eqD(b.seating.bedArea, se["bedArea"], "seating.bedArea");
    }
    eqB(b.sagRisk, B(j["sagRisk"]), "sagRisk");
}

std::vector<FillPt> ptsOf(const json &j) {
    std::vector<FillPt> out;
    for (const json &p : j) out.push_back(FillPt{{N(p[0]), N(p[1]), N(p[2]), N(p[3])}});
    return out;
}
void eqLines(const std::vector<FillLine> &cpp, const json &js, const std::string &w) {
    if (cpp.size() != js.size()) { fail(w + " count: cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    for (size_t i = 0; i < cpp.size(); i++) {
        const std::string wi = w + "[" + std::to_string(i) + "]";
        for (int k = 0; k < 3; k++) { eqD(cpp[i].a[k], js[i]["a"][k], wi + ".a"); eqD(cpp[i].b[k], js[i]["b"][k], wi + ".b"); }
        eqD(cpp[i].weight, js[i]["weight"], wi + ".weight");
        eqInts(cpp[i].aims, js[i]["aims"], wi + ".aims");
    }
}

/// The inner auto build (fins_ref.mjs optsFor('auto')): PLA, every default.
BuildOpts autoOpts() {
    BuildOpts bo;
    bo.mode = "auto"; bo.bedPad = true; bo.tines = true; bo.tineDensity = 0; bo.layerHeight = 0.2; bo.coverage = 0.5;
    bo.plateOnly = false; bo.lastResort = true; bo.raster = true;
    Tunables t;
    t.padH = 0.5; t.padGrab = 0.05; t.propGap = 0.2; t.padStyle = "auto"; t.cutout = "none";
    const Material *m = material_of("pla");
    t.padH = m->padH; t.padGrab = m->padGrab; t.propGap = m->propGap;
    bo.tunables = t;
    return bo;
}

void runFile(const fs::path &file) {
    std::ifstream in(file);
    const json J = json::parse(in);
    const std::string scene = J["scene"];
    Soup soup;
    for (const json &v : J["soup"]) soup.push_back(N(v));
    Rot rot;
    for (int i = 0; i < 9; i++) rot[size_t(i)] = N(J["rot"][size_t(i)]);
    Topology topo;
    if (J["seated"].get<bool>()) {
        Soup pos;
        seatSoup(soup, pos);
        topo = buildTopology(std::move(pos));
    } else {
        topo = buildTopology(soup);
    }
    const AnalyzeResult result = analyze(topo, 45, rot);
    g_fn = "scene"; g_ctx = scene;
    eqI(long(result.regions.size()), J["regions"].get<long>(), "regions");

    for (const json &b : J["builds"]) {
        const std::string cfg = b["cfg"];
        g_fn = "buildFins:" + b["opts"]["mode"].get<std::string>();
        g_ctx = scene + " " + cfg;
        S.casesByFn[g_fn]++;
        resetState();
        applyFillCfg(b["fillCfg"]);
        const BuildOpts bo = optsOf(b);
        const Built built = buildFins(topo, result, rot, bo);
        eqBuilt(built, b["out"]);
        eqState(b["state"]);

        // Entry.cpp's sway merge (fins_entry.js `{ gap: mat.propGap, ...sway }`): through
        // computeFins, a PETG build with no gap of its own must land on this build exactly
        if (scene == "bar-0" && cfg == "swayPetg") {
            g_fn = "computeFins"; S.casesByFn[g_fn]++;
            resetState();
            EntryOptions eo;
            eo.material = "petg"; eo.layerHeight = 0.12;
            SwayOpts s; s.on = true; s.tineSpacing = 3; s.reach = 0.5;   // gap unset
            eo.sway = s;
            const EntryResult e = computeFins(soup, eo);
            std::vector<float> want;
            for (const Vec3d &v : built.triangles) for (int k = 0; k < 3; k++) want.push_back(float(v[k]));
            for (const Vec3d &v : built.padTriangles) for (int k = 0; k < 3; k++) want.push_back(float(v[k]));
            eqB(e.triangles == want, true, "computeFins(petg, sway without gap) = buildFins(sway gap = petg propGap)");
            eqI(e.stats.swayBraces, built.sway ? built.sway->count : -1, "computeFins swayBraces");
        }
    }

    // ---- the internals, on Auto's result ----
    const json &inner = J["inner"];
    if (inner.is_null()) { resetState(); return; }
    resetState();
    const BuildOpts ao = autoOpts();
    const Built aut = buildFins(topo, result, rot, ao);
    for (const json &c : inner["covs"]) {
        const double coverage = N(c["coverage"]);
        g_fn = "fill internals"; g_ctx = scene + " coverage " + fmt(coverage);
        S.casesByFn[g_fn]++;
        const double R = fillReach(coverage);
        eqD(R, c["R"], "fillReach");
        std::vector<Vec3d> tops;
        for (const FinRecord &w : aut.fins) for (const Vec3d &p : w.line) tops.push_back(p);
        const Reach near = reachOf(tops, R);
        const std::vector<uint8_t> cls = classify(topo, result, rot);
        const std::vector<FillPt> pts = leftoverPoints(topo, result, rot, near, cls);
        const json &P = c["pts"];
        if (pts.size() != P.size()) { fail("leftoverPoints count: cpp " + std::to_string(pts.size()) + " js " + std::to_string(P.size())); continue; }
        for (size_t i = 0; i < pts.size(); i++) for (int k = 0; k < 4; k++) eqD(pts[i][size_t(k)], P[i][size_t(k)], "pts");
        // the rest from the JS points, so one stage's slip does not cascade
        const std::vector<FillPt> jp = ptsOf(P);
        const std::vector<std::vector<int>> groups = groupsOf(jp);
        const json &G = c["groups"];
        if (groups.size() != G.size()) fail("groupsOf count");
        else for (size_t i = 0; i < groups.size(); i++) eqInts(groups[i], G[i], "groups[" + std::to_string(i) + "]");
        // heaviest group, as fillCoverage's heaviest()
        std::vector<std::pair<std::vector<int>, double>> gw;
        for (const json &g : G) {
            std::vector<int> gi = g.get<std::vector<int>>();
            double w = 0;
            for (int i : gi) w = w + jp[size_t(i)][3];
            gw.emplace_back(gi, w);
        }
        std::stable_sort(gw.begin(), gw.end(), [](const auto &p, const auto &q) { return q.second - p.second < 0; });
        if (gw.empty()) continue;
        const std::vector<int> &g0 = gw[0].first;
        eqLines(candidateLines(jp, g0, R, FILL.bridge), c["cand"], "candidateLines(bridge)");
        eqLines(candidateLines(jp, g0, R, 2 * FILL.sample), c["cand2"], "candidateLines(2 x sample)");
        eqLines(candidateLines(jp, g0, R, FILL.bridge, true), c["candNarrow"], "candidateLines(narrow)");
        eqInts(localPatch(jp, g0), c["local"], "localPatch");
    }
    for (const json &f : inner["fill"]) {
        const double coverage = N(f["coverage"]);
        g_fn = "fillCoverage"; g_ctx = scene + " coverage " + fmt(coverage);
        S.casesByFn[g_fn]++;
        FillOpts fo;
        fo.coverage = coverage; fo.tines = true; fo.tineDensity = 0.0; fo.layerHeight = 0.2; fo.plateOnly = false;
        const FillResult r = fillCoverage(topo, result, rot, fo, FillAuto{&aut.fins, &aut.triangles});
        eqTris(r.triangles, &r.tags, f["triangles"], "triangles");
        eqProps(r.props, f["props"], "props");
        eqI(r.tines, f["tines"].get<long>(), "tines");
        eqD(r.unserved.area, f["unserved"]["area"], "unserved.area");
        eqLine(r.unserved.pts, f["unserved"]["pts"], "unserved.pts");
        const json &s = f["stats"];
        eqI(r.stats.walls, s["walls"].get<long>(), "stats.walls");
        eqB(r.stats.joined.has_value(), s.contains("joined"), "stats.joined set");
        if (r.stats.joined && s.contains("joined")) eqI(*r.stats.joined, s["joined"].get<long>(), "stats.joined");
        eqB(r.stats.ribs.has_value(), s.contains("ribs"), "stats.ribs set");
        if (r.stats.ribs && s.contains("ribs")) eqI(*r.stats.ribs, s["ribs"].get<long>(), "stats.ribs");
        eqB(r.stats.ties.has_value(), s.contains("ties"), "stats.ties set");
        if (r.stats.ties && s.contains("ties")) eqI(*r.stats.ties, s["ties"].get<long>(), "stats.ties");
        eqI(r.stats.tries, s["tries"].get<long>(), "stats.tries");
        eqI(r.stats.checks, s["checks"].get<long>(), "stats.checks");
        eqB(r.stats.capped, s["capped"].get<bool>(), "stats.capped");
        eqRefused(r.stats.refused, s["refused"], "stats.refused");
        eqD(r.stats.startArea, s["startArea"], "stats.startArea");
    }
    for (const json &sh : inner["short"]) {
        const std::string label = sh["label"];
        g_fn = "lastResortWalls"; g_ctx = scene + " " + label;
        S.casesByFn[g_fn]++;
        PropOpts so;
        so.coverage = 0.5; so.tines = true; so.tineDensity = 0.0; so.layerHeight = 0.2; so.plateOnly = false; so.raster = true;
        static const Tris none;
        const ShortWalls s = lastResortWalls(topo, result, rot, so, {}, none, label == "auto" ? aut.triangles : none);
        eqTris(s.triangles, &s.tags, sh["triangles"], "triangles");
        eqProps(s.props, sh["props"], "props");
        eqInts(s.served, sh["served"], "served");
        eqI(s.tines, sh["tines"].get<long>(), "tines");
    }
    resetState();
}

} // namespace

int main(int argc, char **argv) {
    const fs::path dir = argc > 1 ? fs::path(argv[1]) : fs::path(__FILE__).parent_path().parent_path() / "refs_fins";
    if (!fs::is_directory(dir)) {
        std::fprintf(stderr, "no dumps in %s: run SUPPORT_FINS_DIR=... node tools/fins_ref.mjs\n", dir.string().c_str());
        return 1;
    }
    std::vector<fs::path> files;
    for (const auto &e : fs::directory_iterator(dir)) if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::fprintf(stderr, "no dumps in %s\n", dir.string().c_str()); return 1; }

    // fixed points: FILL's defaults, coverPitch, fillReach, applyTunables' guards
    {
        std::ifstream in(files[0]);
        const json J = json::parse(in);
        const json &D = J["defaults"];
        g_fn = "unit"; g_ctx = "defaults";
        resetState();
        const json &F = D["FILL"];
        eqD(FILL.sample, F["sample"], "FILL.sample"); eqD(FILL.link, F["link"], "FILL.link");
        eqD(FILL.narrow, F["narrow"], "FILL.narrow"); eqD(FILL.local, F["local"], "FILL.local");
        eqD(FILL.localZ, F["localZ"], "FILL.localZ"); eqD(FILL.bridge, F["bridge"], "FILL.bridge");
        eqI(FILL.dirs, F["dirs"].get<long>(), "FILL.dirs"); eqD(FILL.axisBias, F["axisBias"], "FILL.axisBias");
        eqI(FILL.bestDirs, F["bestDirs"].get<long>(), "FILL.bestDirs");
        eqI(long(FILL.shifts.size()), long(F["shifts"].size()), "FILL.shifts");
        for (size_t i = 0; i < std::min(FILL.shifts.size(), F["shifts"].size()); i++) eqD(FILL.shifts[i], F["shifts"][i], "FILL.shifts[i]");
        eqD(FILL.sparseBelow, F["sparseBelow"], "FILL.sparseBelow"); eqD(FILL.denseReach, F["denseReach"], "FILL.denseReach");
        eqD(FILL.minGain, F["minGain"], "FILL.minGain"); eqD(FILL.maxStep, F["maxStep"], "FILL.maxStep");
        eqD(FILL.redNear, F["redNear"], "FILL.redNear"); eqI(FILL.maxWalls, F["maxWalls"].get<long>(), "FILL.maxWalls");
        eqI(FILL.maxTries, F["maxTries"].get<long>(), "FILL.maxTries"); eqI(FILL.groupTries, F["groupTries"].get<long>(), "FILL.groupTries");
        eqI(FILL.maxChecks, F["maxChecks"].get<long>(), "FILL.maxChecks");
        const double cs[] = {0, 0.25, 0.5, 0.75, 1};
        for (size_t i = 0; i < 5; i++) eqD(coverPitch(cs[i]), D["coverPitch"][i], "coverPitch");
        const double fs_[] = {0, 0.5, 0.75, 1};
        for (size_t i = 0; i < 4; i++) eqD(fillReach(fs_[i]), D["fillReach"][i], "fillReach");
        eqB(std::isnan(coverPitch(NAN)), true, "coverPitch(NaN)");

        g_ctx = "applyTunables";
        // JS applyTunables({}): nothing but the always-present fields (here: all NaN / unknown)
        Tunables t;
        t.padH = NAN; t.padGrab = NAN; t.propGap = NAN; t.padStyle = ""; t.cutout = "bogus";
        PROP.iface = Iface::All; PROP.tinesPerWall = 3; PAD.custom.h = 0.9;
        applyTunables(t);
        eqB(PROP.iface == Iface::All, true, "iface left alone");
        eqI(PROP.tinesPerWall, 3, "tinesPerWall left alone");
        eqD(PAD.custom.h, 0.9, "padCustom left alone");
        eqD(PROP.gap, 0.2, "gap left alone"); eqD(PERP.gap, 0.2, "PERP.gap left alone");
        eqS(CUT.pattern, "none", "cutout left alone"); eqS(PAD.style, "auto", "style left alone");
        t.tinesPerWall = -1;
        applyTunables(t);
        eqI(PROP.tinesPerWall, 3, "negative tinesPerWall ignored");
        t.iface = Iface::Off; t.tinesPerWall = 0; t.propGap = 0.3;
        applyTunables(t);
        eqB(PROP.iface == Iface::Off, true, "iface false -> Off");
        eqI(PROP.tinesPerWall, 0, "tinesPerWall 0");
        eqD(PROP.gap, 0.3, "gap"); eqD(PERP.gap, 0.3, "PERP.gap");
        resetState();
    }

    for (const fs::path &f : files) {
        const long before = S.fails;
        const auto t0 = std::chrono::steady_clock::now();
        try { runFile(f); }
        catch (const std::exception &e) { g_fn = "exception"; fail(f.stem().string() + " threw " + e.what()); }
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
