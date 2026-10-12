// Pins the prop leaf modules (Engine/Prop/Surface, Contact, Mold, Crest, Brace, Ties)
// against the upstream JS (web/prop/surface.js, contact.js, mold.js, crest.js,
// brace.js, ties.js). The oracle is tools/propleaf_ref.mjs, which writes one JSON per
// scene into refs_propleaf/ (next to tests/); regenerate with
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/propleaf_ref.mjs
// Every case feeds the dumped inputs to the C++ and checks the outputs: ints/bools
// exact, doubles within 1e-9, triangles in the same order.
#include "Engine/Prop/Brace.hpp"
#include "Engine/Prop/Contact.hpp"
#include "Engine/Prop/Crest.hpp"
#include "Engine/Prop/Mold.hpp"
#include "Engine/Prop/Surface.hpp"
#include "Engine/Prop/Ties.hpp"
#include "Engine/PropConfig.hpp"
#include "Engine/Solids.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
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
std::string g_ctx;   // current case, for messages
std::string g_fn;

void fail(const std::string &what) {
    S.fails++;
    S.failsByFn[g_fn]++;
    if (S.fails <= 40) std::fprintf(stderr, "FAIL [%s] %s\n", g_ctx.c_str(), what.c_str());
}

// A dumped number: JSON number, or the strings "Infinity" / "-Infinity" / "NaN" the
// oracle writes for non-finite values.
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
std::vector<Vec3d> line3(const json &a) {
    std::vector<Vec3d> l;
    for (const auto &p : a) l.push_back(V3(p));
    return l;
}
void eqLine(const std::vector<Vec3d> &cpp, const json &js, const std::string &what) {
    if (cpp.size() != js.size()) { fail(what + ": length cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    for (size_t i = 0; i < cpp.size(); i++)
        for (int k = 0; k < 3; k++) eqD(cpp[i][k], js[i][k], what + "[" + std::to_string(i) + "][" + std::to_string(k) + "]");
}
// Triangles, flattened as the oracle dumps them ([x, y, z] per vertex, emission order).
void eqTris(const Tris &cpp, const json &js, const std::string &what) {
    if (cpp.size() * 3 != js.size()) { fail(what + ": vertex count cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size() / 3)); return; }
    for (size_t i = 0; i < cpp.size(); i++)
        for (int k = 0; k < 3; k++) eqD(cpp[i][k], js[i * 3 + k], what + " v" + std::to_string(i) + "[" + std::to_string(k) + "]");
}
Tris trisOf(const json &flat) {
    Tris t;
    for (size_t i = 0; i + 2 < flat.size(); i += 3) t.push_back(Vec3d(N(flat[i]), N(flat[i + 1]), N(flat[i + 2])));
    return t;
}

// PROP from the defaults plus the case's overrides (the oracle's withProp).
void setProp(const json &c) {
    PROP = PropConfig{};
    if (!c.contains("prop")) return;
    for (const auto &[k, v] : c["prop"].items()) {
        if (k == "gap") PROP.gap = N(v);
        else if (k == "tip") PROP.tip = N(v);
        else if (k == "th") PROP.th = N(v);
        else if (k == "footGap") PROP.footGap = N(v);
        else if (k == "footMin") PROP.footMin = N(v);
        else if (k == "sideClear") PROP.sideClear = N(v);
        else if (k == "ifaceLayers") PROP.ifaceLayers = v.get<int>();
        else if (k == "iface") {
            if (v.is_boolean()) PROP.iface = Iface::Off;
            else PROP.iface = v.get<std::string>() == "all" ? Iface::All : Iface::Flat;
        } else throw std::runtime_error("unknown PROP override " + k);
    }
}

CrestKind kindOf(const json &k) {
    if (k.is_null()) return CrestKind::None;
    return k.get<std::string>() == "flat" ? CrestKind::Flat : CrestKind::Slope;
}

double bandOf(const json &b) { return N(b); }

// --------------------------------------------------------------------------------
void runScene(const fs::path &file) {
    std::ifstream in(file);
    const json d = json::parse(in);
    const std::string scene = d["scene"].get<std::string>();

    // module constants
    g_fn = "consts"; g_ctx = scene + " consts";
    {
        const json &P0 = d["consts"]["PROP"];
        const PropConfig def;
        eqD(def.gap, P0["gap"], "PROP.gap"); eqD(def.tip, P0["tip"], "PROP.tip"); eqD(def.th, P0["th"], "PROP.th");
        eqD(def.footGap, P0["footGap"], "PROP.footGap"); eqD(def.footMin, P0["footMin"], "PROP.footMin");
        eqD(def.footRatio, P0["footRatio"], "PROP.footRatio"); eqD(def.baseH, P0["baseH"], "PROP.baseH");
        eqD(def.tipH, P0["tipH"], "PROP.tipH"); eqD(def.sideClear, P0["sideClear"], "PROP.sideClear");
        eqD(def.tineH, P0["tineH"], "PROP.tineH"); eqD(def.ifaceFlatDeg, P0["ifaceFlatDeg"], "PROP.ifaceFlatDeg");
        eqD(def.maxShortAspect, P0["maxShortAspect"], "PROP.maxShortAspect");
        eqI(def.ifaceLayers, P0["ifaceLayers"].get<int>(), "PROP.ifaceLayers");
        const json &B = d["consts"]["BRACE"];
        eqD(BRACE.aspect, B["aspect"], "BRACE.aspect"); eqD(BRACE.rise, B["rise"], "BRACE.rise");
        eqD(BRACE.onPlate, B["onPlate"], "BRACE.onPlate"); eqD(BRACE.minReach, B["minReach"], "BRACE.minReach");
        eqD(BRACE.step, B["step"], "BRACE.step");
        const json &T = d["consts"]["TIE"];
        eqD(TIE.slope, T["slope"], "TIE.slope"); eqD(TIE.thick, T["thick"], "TIE.thick");
        eqD(TIE.minSpan, T["minSpan"], "TIE.minSpan"); eqD(TIE.maxSpan, T["maxSpan"], "TIE.maxSpan");
        eqD(TIE.minHeight, T["minHeight"], "TIE.minHeight"); eqD(TIE.rise, T["rise"], "TIE.rise");
        eqI(TIE.perWall, T["perWall"].get<int>(), "TIE.perWall"); eqD(TIE.step, T["step"], "TIE.step");
        eqD(TIE.into, T["into"], "TIE.into");
        if (TIE.slope != N(T["slope"])) std::fprintf(stderr, "note: TIE.slope differs in the last bit (%s vs %s)\n", fmt(TIE.slope).c_str(), fmt(N(T["slope"])).c_str());
    }

    std::map<std::string, std::shared_ptr<SurfTris>> tris;
    for (const auto &[id, arr] : d["tris"].items()) {
        Soup s; s.reserve(arr.size());
        for (const auto &v : arr) s.push_back(N(v));
        tris[id] = std::make_shared<SurfTris>(std::move(s));
    }

    int ci = 0;
    for (const json &c : d["cases"]) {
        const std::string fn = c["fn"].get<std::string>();
        g_fn = fn;
        g_ctx = scene + " #" + std::to_string(ci++) + " " + fn;
        S.casesByFn[fn]++;
        setProp(c);

        if (fn == "seat") {
            Soup pos; for (const auto &v : c["pos"]) pos.push_back(N(v));
            Rot r; for (int k = 0; k < 9; k++) r[size_t(k)] = N(c["rot"][k]);
            Offset o{N(c["off"]["x"]), N(c["off"]["y"]), N(c["off"]["z"])};
            Vec3d out(9, 9, 9);
            const Vec3d &ref = seat(pos, c["i"].get<size_t>(), r, o, out);
            eqB(&ref == &out, true, "seat returns out");
            const Vec3d v = seat(pos, c["i"].get<size_t>(), r, o);
            for (int k = 0; k < 3; k++) { eqD(out[k], c["out"][k], "seat"); eqD(v[k], c["out"][k], "seat value"); }

        } else if (fn == "surface") {
            const SurfTris &t = *tris.at(c["tris"].get<std::string>());
            const json &pts = c["pts"];
            for (size_t i = 0; i < pts.size(); i++) {
                const double x = N(pts[i][0]), y = N(pts[i][1]);
                const std::string at = "(" + fmt(x) + "," + fmt(y) + ")";
                const auto z = surfaceZAt(t, x, y);
                const json &jz = c["zAt"][i];
                eqB(z.has_value(), !jz.is_null(), "surfaceZAt null " + at);
                if (z && !jz.is_null()) eqD(*z, jz, "surfaceZAt " + at);
                const auto zs = surfaceZsAt(t, x, y);
                const json &jzs = c["zs"][i];
                eqI(long(zs.size()), long(jzs.size()), "surfaceZsAt count " + at);
                if (zs.size() == jzs.size()) for (size_t k = 0; k < zs.size(); k++) eqD(zs[k], jzs[k], "surfaceZsAt " + at);
                const auto hits = surfaceHitsAt(t, x, y);
                const json &jh = c["hits"][i];
                eqI(long(hits.size()), long(jh.size()), "surfaceHitsAt count " + at);
                if (hits.size() == jh.size())
                    for (size_t k = 0; k < hits.size(); k++) { eqD(hits[k][0], jh[k][0], "hit z " + at); eqD(hits[k][1], jh[k][1], "hit nz " + at); }
            }

        } else if (fn == "straightness") {
            eqD(straightness(line3(c["line"])), c["out"], "straightness");

        } else if (fn == "contactLine") {
            const SurfTris &t = *tris.at(c["tris"].get<std::string>());
            const auto out = contactLine(line3(c["pts"]), t, c["nSamples"].get<int>());
            eqB(out.has_value(), !c["out"].is_null(), "contactLine null");
            if (out && !c["out"].is_null()) {
                eqLine(*out, c["out"], "contactLine");
                eqD(straightness(*out), c["straight"], "straightness(contactLine)");
            }

        } else if (fn == "passes") {
            const SurfTris &t = *tris.at(c["tris"].get<std::string>());
            const double band = bandOf(c["band"]);
            const ContactLine inp = line3(c["line"]);
            { ContactLine l = inp; ContactLine &r = contourTop(l, t, band); eqB(&r == &l, true, "returns line"); eqLine(l, c["contourTop"], "contourTop"); }
            { ContactLine l = inp; lowerSag(l, t, band); eqLine(l, c["lowerSag"], "lowerSag"); }
            { ContactLine l = inp; settleTop(l, t, 0.25, band); eqLine(l, c["settleTop"], "settleTop"); }
            { ContactLine l = inp; settleTop(l, t, 0.6, band); eqLine(l, c["settleTop06"], "settleTop step 0.6"); }
            { ContactLine l = inp; contourTop(l, t, band); lowerSag(l, t, band); settleTop(l, t, 0.25, band); eqLine(l, c["chain"], "chain"); }
            eqD(straightness(inp), c["straight"], "straightness");

        } else if (fn == "floorLine") {
            const FloorLine fl = floorLine(line3(c["top"]), *tris.at("part"), N(c["margin"]), c["mold"].get<bool>());
            const json &js = c["out"];
            eqI(long(fl.size()), long(js.size()), "floorLine length");
            if (fl.size() == js.size())
                for (size_t i = 0; i < fl.size(); i++) {
                    eqI(long(fl[i].length()), long(js[i].size()), "floorLine station arity");
                    if (fl[i].length() == js[i].size())
                        for (size_t k = 0; k < js[i].size(); k++) eqD(fl[i][int(k)], js[i][k], "floorLine[" + std::to_string(i) + "][" + std::to_string(k) + "]");
                }

        } else if (fn == "moldLine") {
            const Mold m = moldLine(line3(c["top"]), *tris.at("part"));
            eqLine(m.top, c["out"]["top"], "moldLine.top");
            const json &js = c["out"]["floor"];
            eqI(long(m.floor.size()), long(js.size()), "moldLine.floor length");
            if (m.floor.size() == js.size())
                for (size_t i = 0; i < m.floor.size(); i++) {
                    eqI(long(m.floor[i].length()), long(js[i].size()), "moldLine.floor arity");
                    if (m.floor[i].length() == js[i].size())
                        for (size_t k = 0; k < js[i].size(); k++) eqD(m.floor[i][int(k)], js[i][k], "moldLine.floor");
                }

        } else if (fn == "crestCut") {
            crestPart(nullptr, c["layerHeight"].is_null() ? 0.0 : N(c["layerHeight"]));
            eqD(crestH(), c["crestH"], "crestH");
            const Frame2 P = [](double o, double z) { return Vec3d(1 + 0.3 * o, 2 - 0.7 * o, z); };
            for (const json &k : c["cuts"]) {
                const double th = k["th"].is_null() ? PROP.th : N(k["th"]);
                const double tip = k["tip"].is_null() ? PROP.tip : N(k["tip"]);
                const auto cc = k["th"].is_null()
                    ? crestCut(N(k["top"]), N(k["ztip"]), N(k["floor"]), kindOf(k["kind"]))
                    : crestCut(N(k["top"]), N(k["ztip"]), N(k["floor"]), kindOf(k["kind"]), th, tip);
                eqB(cc.has_value(), !k["out"].is_null(), "crestCut null");
                if (cc && !k["out"].is_null()) {
                    eqD(cc->cut, k["out"]["cut"], "crestCut.cut");
                    eqD(cc->zt, k["out"]["zt"], "crestCut.zt");
                    eqD(cc->wc, k["out"]["wc"], "crestCut.wc");
                    eqLine(cc->ring(P), k["out"]["ring"], "crestCut.ring");
                }
            }
            const double ft[3] = {0.3, 2, 7.25};
            for (int i = 0; i < 3; i++) eqD(flatCut(ft[i]), c["flatCut"][i], "flatCut");
            eqLine(flatCrestRing(P, 2, 0.5), c["flatCrestRing"][0], "flatCrestRing");
            eqLine(flatCrestRing(P, 7.25, 0.8), c["flatCrestRing"][1], "flatCrestRing");

        } else if (fn == "crestWall") {
            crestPart(c["part"].is_null() ? nullptr : tris.at("part"), c["layerHeight"].is_null() ? 0.0 : N(c["layerHeight"]));
            eqB(crestOn(), c["on"].get<bool>(), "crestOn");
            eqD(crestH(), c["crestH"], "crestH");
            const ContactLine line = line3(c["line"]);
            const auto kinds = crestKinds(line);
            eqB(kinds.has_value(), !c["kinds"].is_null(), "crestKinds null");
            if (kinds && !c["kinds"].is_null()) {
                eqI(long(kinds->size()), long(c["kinds"].size()), "crestKinds length");
                if (kinds->size() == c["kinds"].size())
                    for (size_t i = 0; i < kinds->size(); i++) eqI(int((*kinds)[i]), int(kindOf(c["kinds"][i])), "crestKinds[" + std::to_string(i) + "]");
            }
            // sweep.js's loop (without cutWall), as the oracle runs it
            const double zBed = 0;
            std::vector<Section> wall, flange;
            std::vector<std::optional<Section>> cr;
            bool ok = true;
            const int n = int(line.size());
            for (int i = 0; i < n; i++) {
                const Vec3d p = line[size_t(i)];
                const Vec3d a = line[size_t(std::max(0, i - 1))];
                const Vec3d b = line[size_t(std::min(n - 1, i + 1))];
                double rx = b[0] - a[0], ry = b[1] - a[1];
                const double rn = hypot2(rx, ry);
                if (rn < 1e-9) { ok = false; break; }
                rx /= rn; ry /= rn;
                const double sx = ry, sy = -rx;
                const double top = p[2] - PROP.gap;
                const double h = top - zBed;
                if (h < 0.3) { ok = false; break; }
                const double foot = 2.0;
                const double flangeH = std::min(PROP.baseH, h / 2);
                const double ztip = std::max(top - PROP.tipH, zBed + flangeH + 0.1);
                const double baseTop = zBed + flangeH;
                const Frame2 P = [&](double o, double z) { return Vec3d(p[0] + sx * o, p[1] + sy * o, z); };
                const auto cc = crestCut(top, ztip, zBed, kinds ? (*kinds)[size_t(i)] : CrestKind::None);
                cr.push_back(cc ? std::optional<Section>(cc->ring(P)) : std::nullopt);
                const double zt = cc ? cc->zt : ztip, zTop = cc ? cc->cut : top, wTop = cc ? cc->wc : PROP.tip / 2;
                wall.push_back({P(+PROP.th / 2, zBed), P(+PROP.th / 2, zt), P(+wTop, zTop),
                                P(-wTop, zTop), P(-PROP.th / 2, zt), P(-PROP.th / 2, zBed)});
                flange.push_back({P(+foot, zBed), P(+foot, baseTop), P(-foot, baseTop), P(-foot, zBed)});
            }
            eqB(ok, c["ok"].get<bool>(), "harness ok");
            Tris out;
            IfaceTags tags;
            if (ok) {
                out.push_back(Vec3d(0, 0, 0)); out.push_back(Vec3d(1, 0, 0)); out.push_back(Vec3d(0, 1, 0));
                const size_t from = out.size();
                ribbon(wall, out);
                ribbon(flange, out);
                if (kinds) { dropCollapsed(out, from, &tags); emitCrest(cr, out, &tags); }
                const size_t fromTine = out.size();
                for (int k = 0; k < 2; k++) { out.push_back(Vec3d(k, 5, 1)); out.push_back(Vec3d(k + 1, 5, 1)); out.push_back(Vec3d(k, 6, 1.5)); }
                if (crestOn() && PROP.iface == Iface::All) markInterface(out, fromTine, &tags);
            }
            eqTris(out, c["out"], "crest wall");
            if (tags.size() < out.size()) tags.resize(out.size(), 0);
            eqI(long(tags.size()), long(c["tags"].size()), "tags length");
            if (tags.size() == c["tags"].size())
                for (size_t i = 0; i < tags.size(); i++) eqI(tags[i], c["tags"][i].get<int>(), "tag[" + std::to_string(i) + "]");
            const InterfaceSplit sp = splitInterface(out, &tags);
            eqI(long(sp.body.size()), c["nBody"].get<long>(), "split body");
            eqI(long(sp.iface.size()), c["nIface"].get<long>(), "split iface");
            // the split is the triangles in order, by their first vertex's tag
            Tris body, iface;
            for (size_t i = 0; i + 2 < out.size(); i += 3) {
                Tris &dst = tags[i] == 1 ? iface : body;
                dst.push_back(out[i]); dst.push_back(out[i + 1]); dst.push_back(out[i + 2]);
            }
            eqB(body == sp.body && iface == sp.iface, true, "split order");
            crestPart(nullptr);

        } else if (fn == "dropCollapsed") {
            Tris out = trisOf(c["inp"]);
            IfaceTags tags(out.size());
            for (size_t i = 0; i < tags.size(); i++) tags[i] = uint8_t(i % 2);
            const Tris before = out;
            const size_t from = c["from"].get<size_t>();
            dropCollapsed(out, from, &tags);
            eqTris(out, c["out"], "dropCollapsed");
            Tris plain = before;
            dropCollapsed(plain, from);
            eqTris(plain, c["out"], "dropCollapsed (no tags)");
            // the tags moved with their vertices
            eqI(long(tags.size()), long(out.size()), "dropCollapsed tags length");
            size_t w = from;
            for (size_t i = from; i + 2 < before.size() && w + 2 < tags.size(); i += 3) {
                if (before[i] == before[i + 1] || before[i + 1] == before[i + 2] || before[i] == before[i + 2]) continue;
                for (int k = 0; k < 3; k++) eqI(tags[w + size_t(k)], long((i + size_t(k)) % 2), "dropCollapsed tag");
                w += 3;
            }

        } else if (fn == "emitCrest") {
            std::vector<std::optional<Section>> sections;
            for (const json &s : c["sections"]) {
                if (s.is_null()) sections.push_back(std::nullopt);
                else sections.push_back(line3(s));
            }
            Tris out{Vec3d(9, 9, 9), Vec3d(8, 8, 8), Vec3d(7, 7, 7)};
            IfaceTags tags;
            emitCrest(sections, out, &tags);
            markInterface(out, 1, &tags);
            eqTris(out, c["out"], "emitCrest");
            eqI(long(tags.size()), long(c["tags"].size()), "tags length");
            if (tags.size() == c["tags"].size())
                for (size_t i = 0; i < tags.size(); i++) eqI(tags[i], c["tags"][i].get<int>(), "tag");
            const InterfaceSplit sp = splitInterface(out, &tags);
            eqTris(sp.body, c["body"], "splitInterface body");
            eqTris(sp.iface, c["iface"], "splitInterface iface");
            const InterfaceSplit sp0 = splitInterface(out, nullptr);
            eqI(long(sp0.body.size()), long(out.size()), "untagged split");

        } else if (fn == "braceWall") {
            const ContactLine top = line3(c["top"]);
            std::vector<double> floors;
            for (const auto &f : c["floors"]) floors.push_back(N(f));
            const std::string ck = c["clear"].get<std::string>();
            std::vector<std::array<double, 4>> calls;
            int nCall = 0;
            std::function<bool(const Tris &)> clear;
            if (ck != "none")
                clear = [&](const Tris &t) {
                    calls.push_back({double(t.size()), t[0][0], t[0][1], t[0][2]});
                    nCall++;
                    return ck == "all" ? true : ck == "never" ? false : nCall % 2 == 0;
                };
            Tris out;
            const int built = braceWall(top, [&](int i) { return floors[size_t(i)]; }, *tris.at("part"), N(c["zBed"]), out, clear);
            eqI(built, c["built"].get<long>(), "braceWall built");
            eqI(long(calls.size()), long(c["calls"].size()), "clear calls");
            if (calls.size() == c["calls"].size())
                for (size_t i = 0; i < calls.size(); i++) for (int k = 0; k < 4; k++) eqD(calls[i][size_t(k)], c["calls"][i][k], "clear call");
            eqTris(out, c["out"], "braceWall tris");

        } else if (fn == "tieWalls") {
            Tris out{Vec3d(1, 2, 3)};   // out untouched on refusal: check it is only appended to
            const Vec2d a(N(c["a"][0]), N(c["a"][1])), b(N(c["b"][0]), N(c["b"][1]));
            const int n = tieWalls(a, b, N(c["zMax"]), *tris.at("part"), N(c["zBed"]), out);
            eqI(n, c["n"].get<long>(), "tieWalls n");
            eqB(out.size() >= 1 && out[0] == Vec3d(1, 2, 3), true, "tieWalls appends");
            out.erase(out.begin());
            eqTris(out, c["out"], "tieWalls tris");

        } else {
            fail("unknown case " + fn);
        }
    }
    PROP = PropConfig{};
}

} // namespace

int main(int argc, char **argv) {
    fs::path dir = argc > 1 ? fs::path(argv[1]) : fs::path(__FILE__).parent_path().parent_path() / "refs_propleaf";
    if (!fs::is_directory(dir)) {
        std::fprintf(stderr, "no oracle dumps in %s -- run: SUPPORT_FINS_DIR=... node tools/propleaf_ref.mjs\n", dir.string().c_str());
        return 1;
    }
    std::vector<fs::path> files;
    for (const auto &e : fs::directory_iterator(dir)) if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::fprintf(stderr, "no .json in %s\n", dir.string().c_str()); return 1; }
    for (const auto &f : files) {
        const long f0 = S.fails, c0 = S.checks;
        try { runScene(f); }
        catch (const std::exception &e) { g_fn = "exception"; g_ctx = f.filename().string(); fail(e.what()); }
        std::printf("%-26s %8ld checks  %s\n", f.filename().string().c_str(), S.checks - c0, S.fails == f0 ? "ok" : "FAIL");
    }
    std::printf("cases:");
    for (const auto &[fn, n] : S.casesByFn) std::printf(" %s=%ld", fn.c_str(), n);
    std::printf("\n%ld checks, %ld failures, %ld doubles not bit-identical (max |diff| %.3g)\n", S.checks, S.fails, S.inexact, S.maxDiff);
    for (const auto &[fn, n] : S.failsByFn) std::printf("  failures in %s: %ld\n", fn.c_str(), n);
    std::printf(S.fails ? "test_propleaf: FAILED\n" : "test_propleaf: PASSED\n");
    return S.fails ? 1 : 0;
}
