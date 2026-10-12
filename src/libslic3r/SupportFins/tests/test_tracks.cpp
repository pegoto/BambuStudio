// Pins Engine/Prop/Tracks, Lattice and Rival against the upstream JS (web/prop/tracks.js,
// lattice.js, rival.js). The oracle is tools/tracks_ref.mjs, which writes one JSON per
// scene into refs_tracks/ (next to tests/); regenerate with
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/tracks_ref.mjs
// Every scene is rebuilt here from the dumped soup (buildTopology + analyze under the
// dumped rotation), every region / patch / strut is seated the way prop.js seats it,
// and every dumped call is replayed: counts, flags and face lists exact, doubles within
// 1e-9 (and every non-bit-identical double is counted). rival.js is driven through
// buildPass's patch loop with the same stand-in wall builder the oracle uses.
#include "Engine/Overhangs.hpp"
#include "Engine/Prop/Lattice.hpp"
#include "Engine/Prop/Rival.hpp"
#include "Engine/Prop/Surface.hpp"
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
#include <string>
#include <vector>

using namespace Slic3r::SupportFins;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

struct Stats {
    long   checks = 0, fails = 0, inexact = 0;
    double maxDiff = 0;
    std::map<std::string, long> casesByFn, failsByFn, inexactByFn;
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
    if (d != 0) { S.inexact++; S.inexactByFn[g_fn]++; }
    S.maxDiff = std::max(S.maxDiff, d);
    if (!(d <= 1e-9)) fail(what + ": cpp " + fmt(cpp) + " js " + fmt(js));
}
void eqD(double cpp, const json &js, const std::string &what) { eqD(cpp, N(js), what); }
void eqI(long cpp, long js, const std::string &what) {
    S.checks++;
    if (cpp != js) fail(what + ": cpp " + std::to_string(cpp) + " js " + std::to_string(js));
}
void eqB(bool cpp, bool js, const std::string &what) { eqI(cpp ? 1 : 0, js ? 1 : 0, what); }
void eqFaces(const std::vector<int> &cpp, const json &js, const std::string &what) {
    eqI(long(cpp.size()), long(js.size()), what + " size");
    if (cpp.size() != js.size()) return;
    for (size_t i = 0; i < cpp.size(); i++)
        if (cpp[i] != js[i].get<int>()) { eqI(cpp[i], js[i].get<int>(), what + "[" + std::to_string(i) + "]"); return; }
    S.checks++;
}
void eqLine(const ContactLine &cpp, const json &js, const std::string &what) {
    if (cpp.size() != js.size()) { fail(what + ": length cpp " + std::to_string(cpp.size()) + " js " + std::to_string(js.size())); return; }
    for (size_t i = 0; i < cpp.size(); i++)
        for (int k = 0; k < 3; k++) eqD(cpp[i][k], js[i][k], what + "[" + std::to_string(i) + "][" + std::to_string(k) + "]");
}
void eqLines(const std::vector<ContactLine> &cpp, const json &js, const std::string &what) {
    eqI(long(cpp.size()), long(js.size()), what + " count");
    if (cpp.size() != js.size()) return;
    for (size_t i = 0; i < cpp.size(); i++) eqLine(cpp[i], js[i], what + " line " + std::to_string(i));
}
void eqFlat(const std::vector<double> &cpp, const json &js, const std::string &what) {
    eqI(long(cpp.size()), long(js.size()), what + " size");
    if (cpp.size() != js.size()) return;
    for (size_t i = 0; i < cpp.size(); i++) eqD(cpp[i], js[i], what + "[" + std::to_string(i) + "]");
}

// One seated face list (prop.js's regionTris/regionPts, patchTris/pts).
struct Seat {
    std::shared_ptr<SurfTris> tris;
    std::vector<Vec3d>        pts;
};

struct TestProp {   // the oracle's stand-in prop record
    int region = -1, stations = 0, id = 0;
    std::vector<std::pair<int, int>> triRanges;
};

void runScene(const fs::path &file) {
    std::ifstream in(file);
    const json d = json::parse(in);
    const std::string scene = d["scene"].get<std::string>();
    g_fn = "scene"; g_ctx = scene;
    PROP = PropConfig{};

    // constants
    {
        const json &P = d["PROP"];
        eqD(PROP.stationStep, P["stationStep"], "PROP.stationStep");
        eqD(PROP.maxUnsupportedSpan, P["maxUnsupportedSpan"], "PROP.maxUnsupportedSpan");
        eqD(PROP.splitAgreeDeg, P["splitAgreeDeg"], "PROP.splitAgreeDeg");
        eqD(PROP.tubeSpreadDeg, P["tubeSpreadDeg"], "PROP.tubeSpreadDeg");
        eqD(PROP.tubeCurvedFrac, P["tubeCurvedFrac"], "PROP.tubeCurvedFrac");
        eqD(PROP.tubeMinArea, P["tubeMinArea"], "PROP.tubeMinArea");
        eqD(PROP.tubeSmallMinArea, P["tubeSmallMinArea"], "PROP.tubeSmallMinArea");
        eqD(PROP.tubeConvexFrac, P["tubeConvexFrac"], "PROP.tubeConvexFrac");
        eqD(PROP.tubeTwoSidedFrac, P["tubeTwoSidedFrac"], "PROP.tubeTwoSidedFrac");
        eqD(PROP.contourSlopeMin, P["contourSlopeMin"], "PROP.contourSlopeMin");
        eqD(PROP.edgeInset, P["edgeInset"], "PROP.edgeInset");
        eqD(PROP.edgeFlatDeg, P["edgeFlatDeg"], "PROP.edgeFlatDeg");
        eqD(PROP.lowBandMin, P["lowBandMin"], "PROP.lowBandMin");
        eqD(PROP.squatBrimW, P["squatBrimW"], "PROP.squatBrimW");
        eqD(PROP.squatBrimH, P["squatBrimH"], "PROP.squatBrimH");
        eqD(PROP.minHeightSquat, P["minHeightSquat"], "PROP.minHeightSquat");
        eqD(PROP.maxWander, P["maxWander"], "PROP.maxWander");
        eqI(PROP.minStations, P["minStations"].get<int>(), "PROP.minStations");
        eqD(MIN_REGION_AREA, d["MIN_REGION_AREA"], "MIN_REGION_AREA");
    }

    // the part, rebuilt as the oracle built it
    Soup pos;
    pos.reserve(d["pos"].size());
    for (const auto &v : d["pos"]) pos.push_back(N(v));
    Rot rot;
    for (int k = 0; k < 9; k++) rot[size_t(k)] = N(d["rotM"][k]);
    Topology topo = buildTopology(pos);
    const AnalyzeResult result = analyze(topo, 45, rot);
    const Offset off = result.offset;
    eqD(off.x, d["offset"]["x"], "offset.x"); eqD(off.y, d["offset"]["y"], "offset.y"); eqD(off.z, d["offset"]["z"], "offset.z");
    const json &jreg = d["regions"];
    eqI(long(result.regions.size()), long(jreg.size()), "region count");
    if (result.regions.size() != jreg.size()) return;
    for (size_t ri = 0; ri < jreg.size(); ri++) {
        eqFaces(result.regions[ri].faces, jreg[ri]["faces"], "region " + std::to_string(ri) + " faces");
        eqD(result.regions[ri].area, jreg[ri]["area"], "region area");
    }
    const Support support{&topo, rot, off};
    const double step = PROP.stationStep;

    auto seatFaces = [&](const std::vector<int> &faces) {
        Seat s;
        Soup tris(faces.size() * 9);
        for (size_t k = 0; k < faces.size(); k++) {
            double gx = 0, gy = 0, gz = 0;
            for (size_t i = 0; i < 3; i++) {
                Vec3d v;
                seat(topo.pos, size_t(faces[k]) * 9 + i * 3, rot, off, v);
                for (int c = 0; c < 3; c++) tris[k * 9 + i * 3 + size_t(c)] = v[c];
                s.pts.push_back(v);
                gx += v[0]; gy += v[1]; gz += v[2];
            }
            s.pts.push_back(Vec3d(gx / 3, gy / 3, gz / 3));
        }
        s.tris = std::make_shared<SurfTris>(std::move(tris));
        return s;
    };
    const Seated seated = [&](int f) {
        std::array<Vec3d, 3> t;
        for (size_t i = 0; i < 3; i++) seat(topo.pos, size_t(f) * 9 + i * 3, rot, off, t[i]);
        return t;
    };

    // every array the dump refers to by id, built the oracle's way
    std::map<std::string, Seat> ids;
    std::vector<std::vector<PropPatch>> splits(result.regions.size());
    std::vector<std::optional<Struts>> struts(result.regions.size());
    std::vector<std::vector<PatchPtr>> strutPs(result.regions.size());
    for (size_t ri = 0; ri < result.regions.size(); ri++) {
        const std::string r = "r" + std::to_string(ri);
        ids[r] = seatFaces(result.regions[ri].faces);
        splits[ri] = splitRegion(topo, result.regions[ri].faces, rot);
        for (size_t k = 0; k < splits[ri].size(); k++) ids[r + "p" + std::to_string(k)] = seatFaces(splits[ri][k].faces);
        struts[ri] = latticeStruts(topo, result.regions[ri].faces, seated);
        if (struts[ri]) {
            strutPs[ri] = strutPatches(topo, rot, *struts[ri], seated, step, int(ri), ids[r].tris);
            for (size_t k = 0; k < strutPs[ri].size(); k++)
                if (!strutPs[ri][k]->lines) ids[r + "s" + std::to_string(k)] = seatFaces(strutPs[ri][k]->faces);
        }
    }
    auto seatOf = [&](const json &id) -> const Seat * {
        if (!id.is_string()) return nullptr;
        auto it = ids.find(id.get<std::string>());
        if (it == ids.end()) { fail("unknown array id " + id.dump()); return nullptr; }
        return &it->second;
    };
    auto probeOf = [&](const json &p, RowProbe &probe) -> bool {
        if (p.is_null()) return false;
        const Seat *s = seatOf(p["tris"]);
        if (!s) return false;
        probe.support = &support; probe.patchTris = s->tris.get();
        probe.ux = N(p["ux"]); probe.uy = N(p["uy"]); probe.vx = N(p["vx"]); probe.vy = N(p["vy"]);
        probe.uLo = N(p["uLo"]); probe.uHi = N(p["uHi"]);
        return true;
    };
    std::function<void(const PropPatch &, const json &, const std::string &, const std::shared_ptr<SurfTris> &)> eqPatch =
        [&](const PropPatch &p, const json &j, const std::string &w, const std::shared_ptr<SurfTris> &regionTris) {
        eqFaces(p.faces, j["faces"], w + " faces");
        eqD(p.area, j["area"], w + " area");
        eqI(p.region, j["region"].is_null() ? -1 : j["region"].get<int>(), w + " region");
        eqB(p.strut, j["strut"].get<bool>(), w + " strut");
        eqB(p.axis.has_value(), !j["axis"].is_null(), w + " axis?");
        if (p.axis && !j["axis"].is_null()) { eqD((*p.axis)[0], j["axis"][0], w + " axis.x"); eqD((*p.axis)[1], j["axis"][1], w + " axis.y"); }
        eqB(p.lines.has_value(), !j["lines"].is_null(), w + " lines?");
        if (p.lines && !j["lines"].is_null()) eqLines(*p.lines, j["lines"], w + " lines");
        eqB(bool(p.reach), !j["reach"].is_null(), w + " reach?");
        if (p.reach && !j["reach"].is_null()) eqFlat(p.reach->pos, j["reach"], w + " reach");
        eqB(bool(p.tris), !j["tris"].is_null(), w + " tris?");
        if (p.tris) eqB(p.tris.get() == regionTris.get(), true, w + " tris is regionTris");
        eqB(p.smallTube.has_value(), !j["smallTube"].is_null(), w + " smallTube?");
        if (p.smallTube && !j["smallTube"].is_null()) {
            eqI(long(p.smallTube->size()), long(j["smallTube"].size()), w + " smallTube size");
            if (p.smallTube->size() == j["smallTube"].size())
                for (size_t k = 0; k < p.smallTube->size(); k++)
                    eqPatch(*(*p.smallTube)[k], j["smallTube"][k], w + " smallTube " + std::to_string(k), regionTris);
        }
    };

    int ci = 0;
    for (const json &c : d["cases"]) {
        const std::string fn = c["fn"].get<std::string>();
        g_fn = fn;
        g_ctx = scene + " #" + std::to_string(ci++) + " " + fn;
        S.casesByFn[fn]++;
        PROP = PropConfig{};
        const int ri = c.contains("region") ? c["region"].get<int>() : -1;
        const std::string r = "r" + std::to_string(ri);

        if (fn == "splitRegion") {
            const auto &sp = splits[size_t(ri)];
            eqI(long(sp.size()), long(c["out"].size()), "patch count");
            if (sp.size() != c["out"].size()) continue;
            for (size_t k = 0; k < sp.size(); k++) {
                eqFaces(sp[k].faces, c["out"][k]["faces"], "patch " + std::to_string(k));
                eqD(sp[k].area, c["out"][k]["area"], "patch area");
            }
        } else if (fn == "tubeLine") {
            const Seat &s = ids[r];
            const auto out = tubeLine(topo, result.regions[size_t(ri)].faces, rot, s.pts, *s.tris, N(c["step"]));
            eqB(out.has_value(), !c["out"].is_null(), "tube?");
            if (out && !c["out"].is_null()) eqLines(*out, c["out"], "tube");
        } else if (fn == "patchTracks") {
            const std::string src = c["src"].get<std::string>();
            const int k = c["patch"].get<int>();
            const std::string id = src == "region" ? r : r + (src == "patch" ? "p" : "s") + std::to_string(k);
            const Seat *s = seatOf(json(id));
            if (!s) continue;
            std::optional<Vec2d> axis;
            if (!c["axis"].is_null()) axis = Vec2d(N(c["axis"][0]), N(c["axis"][1]));
            if (src == "strut") {   // the axis must be the strut patch's own
                const PropPatch &sp = *strutPs[size_t(ri)][size_t(k)];
                eqB(sp.axis.has_value(), axis.has_value(), "strut axis?");
                if (sp.axis && axis) { eqD((*sp.axis)[0], (*axis)[0], "strut axis.x"); eqD((*sp.axis)[1], (*axis)[1], "strut axis.y"); axis = sp.axis; }
            }
            const Tracks out = patchTracks(s->pts, *s->tris, N(c["step"]), c["support"].get<bool>() ? &support : nullptr,
                                           N(c["span"]), axis);
            eqLines(out, c["out"], "tracks");
            eqB(out.spacing.has_value(), !c["spacing"].is_null(), "spacing?");
            if (out.spacing && !c["spacing"].is_null()) eqD(*out.spacing, c["spacing"], "spacing");
        } else if (fn == "latticeStruts") {
            const auto &st = struts[size_t(ri)];
            eqB(st.has_value(), !c["out"].is_null(), "struts?");
            if (st && !c["out"].is_null()) {
                eqI(long(st->size()), long(c["out"]["groups"].size()), "strut count");
                if (st->size() == c["out"]["groups"].size())
                    for (size_t k = 0; k < st->size(); k++) eqFaces((*st)[k], c["out"]["groups"][k], "strut " + std::to_string(k));
                eqI(st->lost, c["out"]["lost"].get<int>(), "lost");
            }
        } else if (fn == "strutPatches") {
            const auto &sp = strutPs[size_t(ri)];
            eqI(long(sp.size()), long(c["out"].size()), "strut patch count");
            if (sp.size() != c["out"].size()) continue;
            for (size_t k = 0; k < sp.size(); k++) eqPatch(*sp[k], c["out"][k], "strut patch " + std::to_string(k), ids[r].tris);
        } else if (fn == "edgePoints") {
            const Seat *t = seatOf(c["tris"]), *p = seatOf(c["pts"]);
            if (!t || !p) continue;
            const auto out = edgePoints(t->tris->pos, p->pts, N(c["gap"]));
            std::vector<double> flat;
            for (const Vec3d &v : out) { flat.push_back(v[0]); flat.push_back(v[1]); flat.push_back(v[2]); }
            eqFlat(flat, c["out"], "edgePoints");
        } else if (fn == "convexAbout") {
            const Seat *t = seatOf(c["tris"]);
            if (!t) continue;
            AreaNormals rn;
            for (const auto &q : c["rn"]) rn.emplace_back(Vec3d(N(q[0]), N(q[1]), N(q[2])), N(q[3]));
            eqB(convexAbout(rn, t->tris->pos, N(c["cx"]), N(c["cy"]), N(c["ux"]), N(c["uy"])), c["out"].get<bool>(), "convexAbout");
        } else if (fn == "rowOffsets") {
            RowProbe probe;
            const bool has = probeOf(c["probe"], probe);
            if (!has && !c["probe"].is_null()) continue;
            const RowOffsets out = rowOffsets(N(c["vLo"]), N(c["vHi"]), N(c["rowSpan"]), has ? &probe : nullptr);
            eqFlat(out, c["rows"], "rows");
            eqD(out.spacing, c["spacing"], "spacing");
        } else if (fn == "lowEdges") {
            RowProbe probe;
            if (!probeOf(c["probe"], probe)) continue;
            const LowEdges out = lowEdges(probe, N(c["vLo"]), N(c["vHi"]));
            eqI(long(out.size()), long(c["edges"].size()), "edge count");
            if (out.size() == c["edges"].size())
                for (size_t k = 0; k < out.size(); k++) { eqD(out[k].v, c["edges"][k][0], "edge v"); eqI(out[k].dir, c["edges"][k][1].get<int>(), "edge dir"); }
            for (const auto &q : c["low"]) eqB(out.low(N(q[0])), q[1].get<bool>(), "low(" + fmt(N(q[0])) + ")");
        } else if (fn == "freeEdge") {
            RowProbe probe;
            if (!probeOf(c["probe"], probe)) continue;
            eqB(freeEdge(probe, N(c["vEdge"]), c["dir"].get<int>()), c["out"].get<bool>(), "freeEdge");
        } else {
            fail("unknown case " + fn);
        }
    }

    // ---- rival.js through buildPass's patch loop (the oracle's stand-in walls) -------
    Soup partTris(pos.size());
    for (int f = 0; f < topo.nFaces; f++)
        for (size_t i = 0; i < 3; i++) {
            Vec3d v;
            seat(topo.pos, size_t(f) * 9 + i * 3, rot, off, v);
            for (int c = 0; c < 3; c++) partTris[size_t(f) * 9 + i * 3 + size_t(c)] = v[c];
        }
    int run = 0;
    for (const json &jr : d["rival"]) {
        const std::string mode = jr["mode"].get<std::string>();
        g_fn = "rival";
        g_ctx = scene + " rival " + mode + " coverage " + fmt(N(jr["coverage"]));
        S.casesByFn["rival"]++;
        run++;
        PROP = PropConfig{};
        const double rowSpan = N(jr["rowSpan"]);
        std::vector<PatchPtr> patches;
        int slivers0 = 0;
        for (size_t ri = 0; ri < result.regions.size(); ri++) {
            const std::vector<int> &rFaces = result.regions[ri].faces;
            const Seat rs = seatFaces(rFaces);
            const std::shared_ptr<const SurfTris> regionTris = rs.tris;
            double regionArea = 0;
            for (int f : rFaces) regionArea += topo.area[size_t(f)];
            if (mode == "raster") {
                const auto st = latticeStruts(topo, rFaces, seated);
                if (st) {
                    if (st->lost) slivers0++;
                    for (auto &p : strutPatches(topo, rot, *st, seated, step, int(ri), regionTris)) patches.push_back(p);
                    continue;
                }
                Tracks lines = patchTracks(rs.pts, *rs.tris, step, &support, rowSpan);
                if (!lines.empty()) {
                    auto p = std::make_shared<PropPatch>();
                    p->faces = rFaces; p->area = regionArea; p->region = int(ri); p->tris = regionTris;
                    p->lines = std::vector<ContactLine>(lines.begin(), lines.end());
                    patches.push_back(p);
                }
                continue;
            }
            auto tube = tubeLine(topo, rFaces, rot, rs.pts, *rs.tris, step);
            const bool small = regionArea < PROP.tubeMinArea;
            std::vector<PatchPtr> split;
            if (small || !tube || tube->empty())
                for (PropPatch &p : splitRegion(topo, rFaces, rot)) split.push_back(std::make_shared<PropPatch>(std::move(p)));
            if (tube && !tube->empty()) {
                auto p = std::make_shared<PropPatch>();
                p->faces = rFaces; p->area = regionArea; p->region = int(ri); p->tris = regionTris;
                p->lines = std::move(*tube);
                if (small) p->smallTube = split;
                patches.push_back(p);
                continue;
            }
            for (const PatchPtr &p : split) {
                if (p->area < MIN_REGION_AREA) { slivers0++; continue; }
                p->region = int(ri); p->tris = regionTris;
                patches.push_back(p);
            }
        }
        Tris out;
        IfaceTags tags;
        std::vector<TestProp> props;
        Skipped skipped;
        skipped.sliver = slivers0;
        RegionSet served;
        Tally tally;
        int nextId = 0;
        auto rivalry = tubeRivalry<TestProp>({out, &tags, props, skipped, served, tally}, topo, result, partTris);
        std::vector<std::array<double, 5>> order;
        for (size_t pi = 0; pi < patches.size(); pi++) {
            const PatchPtr patch = patches[pi];
            const std::optional<RivalMark> mark = rivalry.begin(patch);
            order.push_back({double(patch->region), double(patch->faces.size()), patch->area, patch->rival ? 1.0 : 0.0,
                             patch->smallTube ? 1.0 : 0.0});
            std::vector<ContactLine> lines;
            if (patch->lines) lines = *patch->lines;
            else {
                const Seat s = seatFaces(patch->faces);
                lines = patchTracks(s.pts, *s.tris, step, &support, rowSpan, patch->axis);
            }
            if (lines.empty()) { skipped.noLine++; continue; }
            for (size_t li = 0; li < lines.size(); li++) {
                const ContactLine &line = lines[li];
                if (mode == "weak" && patch->smallTube) { skipped.weld++; continue; }   // the tube always loses
                if (line.size() < 4) { skipped.stub++; continue; }
                if ((li + patch->faces.size()) % 4 == 3) { skipped.blocked++; continue; }
                const int tri0 = int(out.size());
                for (size_t k = 0; k + 1 < line.size(); k++) {
                    const Vec3d &a = line[k], &b = line[k + 1];
                    tags.resize(out.size(), 0);
                    out.push_back(Vec3d(a[0], a[1], a[2] - PROP.gap)); tags.push_back(k % 2 ? 1 : 0);
                    out.push_back(Vec3d(b[0], b[1], b[2] - PROP.gap)); tags.push_back(0);
                    out.push_back(Vec3d(a[0], a[1], 0)); tags.push_back(0);
                }
                tally.tines += int(line.size());
                if (line.size() > 25) tally.sagRisk = true;
                served.add(patch->region);
                props.push_back(TestProp{patch->region, int(line.size()), nextId++, {std::pair<int, int>(tri0, int(out.size()))}});
            }
            if (patch->smallTube) rivalry.tubeDone(patch, *mark, patches);
        }
        rivalry.finish();

        const json &jo = jr["order"];
        eqI(long(order.size()), long(jo.size()), "patch order count");
        if (order.size() == jo.size())
            for (size_t i = 0; i < order.size(); i++) {
                eqI(long(order[i][0]), jo[i][0].get<long>(), "order region");
                eqI(long(order[i][1]), jo[i][1].get<long>(), "order faces");
                eqD(order[i][2], jo[i][2], "order area");
                eqI(long(order[i][3]), jo[i][3].get<long>(), "order rival");
                eqI(long(order[i][4]), jo[i][4].get<long>(), "order smallTube");
            }
        std::vector<double> flat;
        for (const Vec3d &v : out) { flat.push_back(v[0]); flat.push_back(v[1]); flat.push_back(v[2]); }
        eqFlat(flat, jr["out"], "out");
        eqI(long(out.size()), long(jr["tags"].size()), "tags size");
        if (out.size() == jr["tags"].size())
            for (size_t i = 0; i < out.size(); i++) eqI(i < tags.size() ? tags[i] : 0, jr["tags"][i].get<int>(), "tag " + std::to_string(i));
        eqI(long(props.size()), long(jr["props"].size()), "props");
        if (props.size() == jr["props"].size())
            for (size_t i = 0; i < props.size(); i++) {
                const json &q = jr["props"][i];
                eqI(props[i].region, q["region"].get<int>(), "prop region");
                eqI(props[i].stations, q["stations"].get<int>(), "prop stations");
                eqI(props[i].id, q["id"].get<int>(), "prop id");
                eqI(props[i].triRanges[0].first, q["triRanges"][0][0].get<int>(), "prop triRange a");
                eqI(props[i].triRanges[0].second, q["triRanges"][0][1].get<int>(), "prop triRange b");
            }
        const json &js = jr["skipped"];
        eqI(skipped.noLine, js["noLine"].get<int>(), "skipped.noLine");
        eqI(skipped.wanders, js["wanders"].get<int>(), "skipped.wanders");
        eqI(skipped.stub, js["stub"].get<int>(), "skipped.stub");
        eqI(skipped.blocked, js["blocked"].get<int>(), "skipped.blocked");
        eqI(skipped.degenerate, js["degenerate"].get<int>(), "skipped.degenerate");
        eqI(skipped.buried, js["buried"].get<int>(), "skipped.buried");
        eqI(skipped.weld, js["weld"].get<int>(), "skipped.weld");
        eqI(skipped.sliver, js["sliver"].get<int>(), "skipped.sliver");
        eqI(skipped.onPart, js["onPart"].get<int>(), "skipped.onPart");
        eqFaces(served.order, jr["served"], "served");
        eqI(tally.tines, jr["tines"].get<int>(), "tines");
        eqB(tally.sagRisk, jr["sagRisk"].get<bool>(), "sagRisk");
    }
}

} // namespace

int main(int argc, char **argv) {
    fs::path dir = fs::path(__FILE__).parent_path().parent_path() / "refs_tracks";
    if (argc > 1) dir = argv[1];
    std::vector<fs::path> files;
    if (fs::exists(dir))
        for (const auto &e : fs::directory_iterator(dir))
            if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) {
        std::fprintf(stderr, "no reference dumps in %s; run SUPPORT_FINS_DIR=... node tools/tracks_ref.mjs\n", dir.string().c_str());
        return 1;
    }
    for (const auto &f : files) {
        try {
            runScene(f);
        } catch (const std::exception &e) {
            g_fn = "exception";
            fail(f.filename().string() + ": " + e.what());
        }
    }
    std::printf("test_tracks: %zu scenes\n", files.size());
    for (const auto &[fn, n] : S.casesByFn)
        std::printf("  %-14s %6ld cases  %ld fails  %ld inexact\n", fn.c_str(), n, S.failsByFn[fn], S.inexactByFn[fn]);
    std::printf("checks %ld, fails %ld, non-bit-identical doubles %ld (max |diff| %.3g)\n", S.checks, S.fails, S.inexact, S.maxDiff);
    return S.fails ? 1 : 0;
}
