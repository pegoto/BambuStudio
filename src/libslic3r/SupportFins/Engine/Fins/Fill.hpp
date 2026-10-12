// Port of web/fins/fill.js -- FULL COVERAGE, the fill pass (mode 'full' only, after
// Auto's walls, wedges and last resort, so Auto is unchanged).
//
// What it aims at is the coverage scoreboard's number: the MUST red (Coverage.hpp
// classify) that no wall top reaches (reachOf). That red is sampled into points, and
// walls are added one at a time, each built by Draw's builder (Draw.hpp drawnWall:
// part-attached, plate, squat fallback, tines).
//
// CONNECTED: points are grouped by DISTANCE (FILL.link in plan, and not one above the
// other), not by region, and a wall's line may cross a gap of up to FILL.bridge.
// THE LINE: FILL.dirs directions slid across the group, the line that keeps the most
// red within reach wins (x or y on a near-tie); the best few also shifted across
// themselves, and every direction at its heaviest FILL.narrow band. When no line across
// a group builds, it is tried patch by patch around its densest red (FILL.local).
// THE SLIDER: below FILL.sparseBelow every group gets at most ONE wall; from there up,
// walls are added until every point is within reach (fillReach).
// GUARDRAILS: a wall is kept only if it holds new red and keeps PROP.sideClear off every
// support already built; FILL.maxWalls / maxTries / maxChecks bound the work (counts,
// not a clock). Red no wall can reach is returned (`unserved`), never hidden.
//
// Braces (prop/brace.js ribs) and ties (prop/ties.js) are added last, after every wall.
#pragma once
#include "../Fins.hpp"
#include "../Math.hpp"
#include "../Overhangs.hpp"
#include "../Prop/Crest.hpp"
#include "../Prop/Surface.hpp"
#include "Coverage.hpp"
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// JS FILL. Mutable module state upstream (tests/fill.test.js sets maxWalls), so
/// thread_local like FIN and PROP.
struct FillConfig {
    double sample = 1.0;       // mm: leftover red is merged into points this far apart
    double link = 12.0;        // mm in plan: points this close are one group (maxUnsupportedSpan)
    double narrow = 1.0;       // mm: each direction also tried at its heaviest band this narrow
    double local = 6.0;        // mm: a group no line across builds on is tried patch by patch, this radius...
    double localZ = 3.0;       // ...and this far up or down
    double bridge = 8.0;       // mm: a line may cross this much plan gap between red it serves
    int    dirs = 12;          // line directions tried, 180 / dirs apart
    double axisBias = 0.97;    // a diagonal must serve 1/0.97 x an x/y line's red to beat it
    int    bestDirs = 3;       // directions tried per group, best first...
    std::vector<double> shifts = {-2, 2, -4, 4};   // ...each also this many mm across itself
    double sparseBelow = 0.25; // coverage under this: one wall per group, then stop
    double denseReach = 4.0;   // mm: the reach at coverage 1 (the middle is maxUnsupportedSpan/2)
    double minGain = 0.5;      // mm2 of new red a wall must hold to be kept
    double maxStep = 2.5;      // mm: a line's top may climb or drop this much between stations
    double redNear = 1.5;      // mm: a station this close to leftover red counts toward its stretch
    int    maxWalls = 40;
    int    maxTries = 120;     // drawnWall calls, kept or not
    int    groupTries = 12;    // drawnWall calls one group gets before its red is given up on
    int    maxChecks = 2000;   // line checks (underStretch, then settledStretch), built or not
};
extern thread_local FillConfig FILL;

/// The reach the slider asks for: R(0.5) = 6 mm (the held rule), R(1) = denseReach.
double fillReach(double coverage);

/// fillCoverage's `opts` (JS { ...opts, tines: withTines, coverage } from fins.js).
struct FillOpts {
    std::optional<double> coverage;    // JS `opts.coverage ?? 0.5`
    bool tines = true;                 // JS `opts.tines ?? true`
    std::optional<double> tineDensity;
    std::optional<double> layerHeight;
    bool plateOnly = false;            // JS `opts.plateOnly === true`
};

/// The fill pass's stats (JS { walls, joined?, ribs, ties, tries, checks, capped, refused,
/// startArea }). `refused` keeps a JS object's insertion order.
struct FillStats {
    int    walls = 0;
    std::optional<int> joined;         // JS: created on the first joined wall
    std::optional<int> ribs, ties;     // JS: undefined when there was no leftover red
    int    tries = 0, checks = 0;
    bool   capped = false;
    std::vector<std::pair<std::string, int>> refused;
    double startArea = 0;
    void refuse(const std::string &why);
};

/// fillCoverage's result (JS { triangles, props, tines, unserved: { area, pts }, stats }).
/// `tags` rides with `triangles`. props: fill: true, triRanges into `triangles` (the
/// wall, then its ribs, then each tie it is wall A of).
struct FillResult {
    Tris      triangles;
    IfaceTags tags;
    std::vector<BuiltProp> props;
    int       tines = 0;
    struct { double area = 0; std::vector<Vec3d> pts; } unserved;
    FillStats stats;
};

/// The input fillCoverage reads off Auto's result: its fins' contact lines and every
/// support triangle.
struct FillAuto {
    const std::vector<FinRecord> *fins = nullptr;
    const Tris *triangles = nullptr;
};

/// fillCoverage(topo, result, rot, opts, built): the fill pass over Auto's result.
FillResult fillCoverage(const Topology &topo, const AnalyzeResult &result, const Rot &rot, const FillOpts &opts,
                        const FillAuto &built);

// ---- module-internal in the JS; exposed for tests ---------------------------------

/// A leftover point [x, y, z, area].
using FillPt = std::array<double, 4>;
/// leftoverPoints(topo, result, rot, near, cls): leftover MUST red merged into FILL.sample cells.
std::vector<FillPt> leftoverPoints(const Topology &topo, const AnalyzeResult &result, const Rot &rot,
                                   const Reach &near, const std::vector<uint8_t> &cls);
/// groupsOf(pts): index groups, linked within FILL.link in plan, no steeper than 45deg (+1 mm).
std::vector<std::vector<int>> groupsOf(const std::vector<FillPt> &pts);
/// A candidate line (JS { a, b, weight, aims }), a/b = [x, y, zHint].
struct FillLine {
    Vec3d a = Vec3d::Zero(), b = Vec3d::Zero();
    double weight = 0;
    std::vector<int> aims;
};
/// candidateLines(pts, group, R, bridge, narrow)
std::vector<FillLine> candidateLines(const std::vector<FillPt> &pts, const std::vector<int> &group, double R,
                                     double bridge, bool narrow = false);
/// localPatch(pts, g)
std::vector<int> localPatch(const std::vector<FillPt> &pts, const std::vector<int> &g);

}} // namespace
