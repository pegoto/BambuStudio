// Port of web/fins/coverage.js -- which red counts, and what holds it: the one rule
// the engine and the coverage scoreboard share, so the Full coverage fill pass aims at
// exactly the number the scoreboard reports.
//
// Every red face must get something, except a TINY HOLE's ceiling (a model issue -- it
// stays flagged, unsupported); the NEAR-PLATE strip is reported apart. Every other
// overhang face is MUST. A face is HELD (reachOf) when a wall-top point lies within R
// of its centroid in plan and 0..1.5 mm below it, or it sits on the plate (z < PLATE_Z).
#pragma once
#include "../Overhangs.hpp"
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace Slic3r { namespace SupportFins {

inline constexpr double PLATE_Z = 0.6;   // a face this low is held by the plate itself

/// reachOf(tops, R)'s result: the JS returns a closure `near(x, y, z)` with a `.add(p)`
/// method; here an object with operator() and add(). The tops are gridded by plan cell.
struct Reach {
    double R = 0;
    double cell = 1;   // Math.max(R, 1)
    // JS Map keyed by the NUMBER i * 100003 + j (so distinct cells may share a key,
    // exactly as upstream); per key the points in insertion order.
    std::unordered_map<double, std::vector<Vec3d>> grid;
    /// near.add(p): grow the set (the fill pass adds its walls' tops).
    void add(const Vec3d &p);
    /// near(cx, cy, cz): is there a wall-top point within R of (cx, cy) in plan and
    /// -0.05..1.5 mm under cz?
    bool operator()(double cx, double cy, double cz) const;
};

/// reachOf(tops, R)
Reach reachOf(const std::vector<Vec3d> &tops, double R);

inline constexpr uint8_t MUST = 0, HOLE = 1, LOW = 2;

/// classify(topo, res, rot): per face (indexed by face; 0 for faces that aren't
/// overhang, `res.over`), its class MUST, HOLE or LOW (JS Uint8Array). Faces are grouped
/// into connected patches (topo adjacency), and a patch is a HOLE as a whole; LOW is per
/// face.
std::vector<uint8_t> classify(const Topology &topo, const AnalyzeResult &res, const Rot &rot);

}} // namespace
