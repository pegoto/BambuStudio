// Parity test for Engine/Fins/Pad.* (web/fins/pad.js) and Engine/Fins/Coverage.*
// (web/fins/coverage.js), plus the PAD half of applyTunables (Engine/Fins.cpp).
// Replays what tools/pad_ref.mjs ran against the upstream JS and compares: counts,
// booleans, strings and classes exactly; doubles within 1e-9 (and counts how many are
// not bit-identical); the pad's triangles same count, same order, same coordinates.
//
//   SUPPORT_FINS_DIR=/path/to/support-fins node tools/pad_ref.mjs   # writes refs_pad/
//   ./build-sf/test_pad [refs_pad_dir]
#include "Engine/Entry.hpp"
#include "Engine/FinConfig.hpp"
#include "Engine/Fins.hpp"
#include "Engine/Fins/Coverage.hpp"
#include "Engine/Fins/Pad.hpp"
#include "Engine/Fins/Seating.hpp"
#include "Engine/Materials.hpp"
#include "Engine/Overhangs.hpp"
#include "Engine/Prop/Surface.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

using namespace Slic3r::SupportFins;
using json = nlohmann::json;
namespace fs = std::filesystem;

static long g_checks = 0, g_fail = 0, g_inexact = 0;
static std::string g_ctx;
static const bool g_verbose = std::getenv("PAD_VERBOSE") != nullptr;   // print the first non-bit-identical doubles

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
    if (g_verbose && g_inexact <= 30) std::fprintf(stderr, "inexact %s: %s got %.17g want %.17g\n", g_ctx.c_str(), what, got, want);
    if (std::isfinite(got) && std::isfinite(want) && std::abs(got - want) <= 1e-9) return;
    char b[256];
    std::snprintf(b, sizeof b, "%s: got %.17g want %.17g", what, got, want);
    fail(b);
}
static void checkB(bool got, bool want, const char *what) {
    g_checks++;
    if (got != want) fail(std::string(what) + ": got " + (got ? "true" : "false") + " want " + (want ? "true" : "false"));
}
static void checkI(long got, long want, const char *what) {
    g_checks++;
    if (got != want) fail(std::string(what) + ": got " + std::to_string(got) + " want " + std::to_string(want));
}
static void checkS(const std::string &got, const std::string &want, const char *what) {
    g_checks++;
    if (got != want) {
        size_t i = 0;
        while (i < got.size() && i < want.size() && got[i] == want[i]) i++;
        fail(std::string(what) + ": differ at " + std::to_string(i) + " (len got " + std::to_string(got.size()) +
             " want " + std::to_string(want.size()) + ")");
    }
}

static std::vector<double> readBin(const fs::path &p) {
    std::ifstream in(p, std::ios::binary);
    in.seekg(0, std::ios::end);
    const std::streamoff n = in.tellg();
    in.seekg(0);
    std::vector<double> v(size_t(n) / sizeof(double));
    in.read(reinterpret_cast<char *>(v.data()), std::streamsize(v.size() * sizeof(double)));
    return v;
}

int main(int argc, char **argv) {
    const fs::path dir = argc > 1 ? fs::path(argv[1])
                                  : fs::path(__FILE__).parent_path().parent_path() / "refs_pad";
    if (!fs::is_directory(dir)) {
        std::fprintf(stderr, "no dumps in %s: run SUPPORT_FINS_DIR=... node tools/pad_ref.mjs\n", dir.string().c_str());
        return 1;
    }
    std::vector<fs::path> files;
    for (const auto &e : fs::directory_iterator(dir)) if (e.path().extension() == ".json") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::fprintf(stderr, "no dumps in %s\n", dir.string().c_str()); return 1; }

    // the defaults, before any applyTunables
    g_ctx = "PAD defaults";
    checkD(PAD.cell, 1.2, "cell"); checkD(PAD.grab, 0.05, "grab"); checkS(PAD.style, "auto", "style");
    checkD(PAD.custom.h, 0.5, "custom.h"); checkD(PAD.custom.gap, 0.0, "custom.gap");
    checkD(PAD.custom.grip, 0.05, "custom.grip"); checkD(PAD.custom.margin, 4.0, "custom.margin");
    checkD(PAD.brimGap, 0.12, "brimGap"); checkD(PAD.minGripOutline, 20.0, "minGripOutline"); checkD(PAD.brimCell, 0.1, "brimCell");
    // applyTunables ignores an unknown style and non-finite numbers
    {
        Tunables t;
        t.padStyle = "bogus";
        t.padGrab = std::numeric_limits<double>::quiet_NaN();
        t.padCustom.margin = INF;
        t.padCustom.gap = 0.25;
        applyTunables(t);
        g_ctx = "applyTunables guards";
        checkS(PAD.style, "auto", "style kept"); checkD(PAD.grab, 0.05, "grab kept");
        checkD(PAD.custom.margin, 4.0, "custom.margin kept"); checkD(PAD.custom.gap, 0.25, "custom.gap set");
        applyTunables(Tunables{});
        checkD(PAD.custom.gap, 0.0, "custom.gap reset");
    }

    long nPads = 0, nVerts = 0, nCls = 0, nReach = 0;
    for (const fs::path &file : files) {
        std::ifstream in(file);
        const json J = json::parse(in);
        const std::string scene = J["scene"];
        const long failBefore = g_fail, checksBefore = g_checks;
        fs::path binPath = file; binPath.replace_extension(".bin");
        const std::vector<double> bin = readBin(binPath);

        // exactly as computeFins: seatSoup, buildTopology, analyze(..., IDENTITY3)
        Soup posed;
        for (const json &v : J["soup"]) posed.push_back(num(v));
        Soup pos;
        seatSoup(posed, pos);
        Topology topo = buildTopology(std::move(pos));
        const AnalyzeResult res = analyze(topo, DEFAULT_THRESHOLD, IDENTITY3);

        g_ctx = scene + " seating";
        checkD(res.bedArea, num(J["bedArea"]), "bedArea");
        checkB(res.bedArea < FIN.padMinArea, J["gate"].get<bool>(), "pad gate (bedArea < padMinArea)");

        // ---- classify, seated (before anything else re-analyzes the topology)
        {
            const json &C = J["classify"][0];
            g_ctx = scene + " classify seated";
            std::string over, cls;
            for (uint8_t o : res.over) over += char('0' + o);
            checkS(over, C["over"].get<std::string>(), "over");
            for (uint8_t c : classify(topo, res, IDENTITY3)) cls += char('0' + c);
            checkS(cls, C["cls"].get<std::string>(), "cls");
            nCls += long(cls.size());
        }

        // ---- bedContact (the pad's input) and buildPad
        const BedContact contact = bedContact(topo, res, IDENTITY3);
        g_ctx = scene + " bedContact";
        checkI(long(contact.pts.size()), long(J["contact"].size()), "contact count");
        std::vector<Vec2d> pts;
        for (const json &p : J["contact"]) pts.emplace_back(num(p[0]), num(p[1]));
        for (size_t i = 0; i < std::min(pts.size(), contact.pts.size()); i++) {
            checkD(contact.pts[i][0], pts[i][0], "contact x"); checkD(contact.pts[i][1], pts[i][1], "contact y");
        }
        checkD(contact.mx, num(J["mx"]), "mx"); checkD(contact.my, num(J["my"]), "my");
        const SurfTris partTris(seatedPartTris(topo, IDENTITY3, res.offset));

        for (const json &P : J["pads"]) {
            const std::string cfg = P["config"];
            g_ctx = scene + " pad " + cfg;
            const Material *m = material_of(P["mat"].get<std::string>());
            Tunables t;
            t.padH = m->padH; t.padGrab = m->padGrab; t.propGap = m->propGap;
            t.padStyle = P["style"].get<std::string>();
            t.padCustom.h = num(P["custom"]["h"]); t.padCustom.gap = num(P["custom"]["gap"]);
            t.padCustom.grip = num(P["custom"]["grip"]); t.padCustom.margin = num(P["custom"]["margin"]);
            applyTunables(t);
            const json &S = P["state"];
            checkD(PAD.grab, num(S["grab"]), "PAD.grab"); checkS(PAD.style, S["style"].get<std::string>(), "PAD.style");
            checkD(PAD.custom.h, num(S["custom"]["h"]), "PAD.custom.h"); checkD(PAD.custom.gap, num(S["custom"]["gap"]), "PAD.custom.gap");
            checkD(PAD.custom.grip, num(S["custom"]["grip"]), "PAD.custom.grip");
            checkD(PAD.custom.margin, num(S["custom"]["margin"]), "PAD.custom.margin");
            checkD(PAD.cell, num(S["cell"]), "PAD.cell"); checkD(PAD.brimGap, num(S["brimGap"]), "PAD.brimGap");
            checkD(PAD.minGripOutline, num(S["minGripOutline"]), "PAD.minGripOutline"); checkD(PAD.brimCell, num(S["brimCell"]), "PAD.brimCell");
            checkD(FIN.padH, num(S["padH"]), "FIN.padH"); checkD(FIN.tineH, num(S["tineH"]), "FIN.tineH");

            std::vector<Vec2d> in = pts;
            if (P.contains("contactCount")) in.resize(P["contactCount"].get<size_t>());
            Tris out;
            const auto pad = P["layerH"].is_null() ? buildPad(in, partTris, out) : buildPad(in, partTris, out, num(P["layerH"]));
            const json &W = P["pad"];
            checkB(bool(pad), !W.is_null(), "pad built");
            if (pad && !W.is_null()) {
                checkD(pad->r1, num(W["r1"]), "r1"); checkD(pad->r2, num(W["r2"]), "r2");
                checkI(pad->cells, W["cells"].get<long>(), "cells");
                checkD(pad->height, num(W["height"]), "height");
                checkI(pad->points, W["points"].get<long>(), "points");
                checkB(pad->oval, W["oval"].get<bool>(), "oval");
                checkS(pad->style, W["style"].get<std::string>(), "style");
                checkB(bool(pad->autoSure), W.contains("autoSure"), "autoSure present");
                if (pad->autoSure && W.contains("autoSure")) checkB(*pad->autoSure, W["autoSure"].get<bool>(), "autoSure");
                checkB(pad->smallFoot, W["smallFoot"].get<bool>(), "smallFoot");
                checkD(pad->outline, num(W["outline"]), "outline");
            }
            const long want = P["nVerts"].get<long>();
            checkI(long(out.size()), want, "triangle vertices");
            const size_t at = P["binAt"].get<size_t>();
            if (long(out.size()) == want && at + out.size() * 3 <= bin.size()) {
                long bad = 0;
                for (size_t i = 0; i < out.size(); i++) for (int k = 0; k < 3; k++) {
                    const long fb = g_fail;
                    char what[64];
                    std::snprintf(what, sizeof what, "vertex %zu[%d]", i, k);
                    checkD(out[i][k], bin[at + i * 3 + k], what);
                    if (g_fail != fb && ++bad > 5) break;
                }
            } else if (long(out.size()) == want) fail("bin too short");
            nPads++;
            nVerts += long(out.size());
        }

        // ---- reachOf
        {
            const json &RJ = J["reach"];
            std::vector<Vec3d> tops, half, rest;
            for (const json &p : RJ["tops"]) tops.emplace_back(num(p[0]), num(p[1]), num(p[2]));
            for (size_t i = 0; i < tops.size(); i++) (i % 2 == 0 ? half : rest).push_back(tops[i]);
            std::vector<Vec3d> qs;
            for (const json &p : RJ["queries"]) qs.emplace_back(num(p[0]), num(p[1]), num(p[2]));
            for (const json &run : RJ["runs"]) {
                const double R = num(run["R"]);
                g_ctx = scene + " reachOf R=" + std::to_string(R);
                auto bits = [&](const Reach &near) {
                    std::string s;
                    for (const Vec3d &q : qs) s += near(q[0], q[1], q[2]) ? '1' : '0';
                    return s;
                };
                Reach near = reachOf(half, R);
                checkS(bits(near), run["before"].get<std::string>(), "before add");
                for (const Vec3d &p : rest) near.add(p);
                checkS(bits(near), run["after"].get<std::string>(), "after add");
                checkS(bits(reachOf(tops, R)), run["all"].get<std::string>(), "all tops");
                nReach += 3 * long(qs.size());
            }
        }

        // ---- classify, under an arbitrary rotation
        {
            const json &C = J["classify"][1];
            g_ctx = scene + " classify rotated";
            Rot rot;
            for (int i = 0; i < 9; i++) rot[i] = num(C["rot"][i]);
            const AnalyzeResult r2 = analyze(topo, DEFAULT_THRESHOLD, rot);
            checkD(r2.offset.x, num(C["offset"]["x"]), "offset.x"); checkD(r2.offset.y, num(C["offset"]["y"]), "offset.y");
            checkD(r2.offset.z, num(C["offset"]["z"]), "offset.z");
            std::string over, cls;
            for (uint8_t o : r2.over) over += char('0' + o);
            checkS(over, C["over"].get<std::string>(), "over");
            for (uint8_t c : classify(topo, r2, rot)) cls += char('0' + c);
            checkS(cls, C["cls"].get<std::string>(), "cls");
            nCls += long(cls.size());
        }

        std::printf("%-14s faces=%5d contact=%3zu pads=%2zu  tops=%4zu queries=%5zu  checks %8ld  %s\n", scene.c_str(),
                    topo.nFaces, pts.size(), J["pads"].size(), J["reach"]["tops"].size(), J["reach"]["queries"].size(),
                    g_checks - checksBefore, g_fail == failBefore ? "ok" : "FAIL");
    }
    std::printf("%ld pads (%ld triangle vertices), %ld classified faces, %ld reach queries\n", nPads, nVerts, nCls, nReach);
    std::printf("%ld checks, %ld failures, %ld doubles not bit-identical (tolerance 1e-9)\n", g_checks, g_fail, g_inexact);
    return g_fail ? 1 : 0;
}
