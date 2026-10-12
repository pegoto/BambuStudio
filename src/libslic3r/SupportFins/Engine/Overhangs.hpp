// Port of web/overhangs.js -- overhang analysis in print space (Z up, mm, bed at z = 0).
//   buildTopology()  welds vertices and finds face adjacency (rotation-invariant, once per mesh)
//   analyze()        applies an orientation, re-seats on the plate, classifies, clusters regions
#pragma once
#include "Math.hpp"
#include <memory>
#include <vector>

namespace Slic3r { namespace SupportFins {

inline constexpr double BED_EPS = 0.35;           // mm; a face this close to the plate IS the bottom
inline constexpr double MIN_REGION_AREA = 12.0;   // mm^2; ignore slivers
inline constexpr double DEFAULT_THRESHOLD = 45;   // degrees from the plate
inline constexpr double ANGLE_EPS = 1e-4;         // ~0.008 degrees of slack

struct InsideGrid;   // Inside.hpp

struct Topology {
    Soup                 pos;      // 9 per face, the input soup (unrotated, unseated)
    int                  nFaces = 0;
    std::vector<double>  nrm;      // 3 per face, from the winding (float64)
    std::vector<double>  area;     // per face
    std::vector<int>     adjA, adjB; // face adjacency via shared welded edges
    int                  vertexCount = 0;
    int                  edgeCount = 0;
    // scratch, reused across analyze() calls (JS _zr/_parent/_over/_kept/_onBed)
    std::vector<double>  _zr;
    std::vector<int>     _parent;
    std::vector<uint8_t> _over, _kept, _onBed;
    // inside.js caches, written onto the topo object upstream (topo._insideGrid,
    // topo._scSeen, topo._scGen). mutable: the queries take a const Topology&, as JS
    // callers just hand the topo along. Built from `pos` only (orientation-free); a
    // copy of the Topology shares the grid, which stays valid because pos is copied too.
    mutable std::shared_ptr<const InsideGrid> _insideGrid;
    mutable std::vector<int32_t>              _scSeen;
    mutable int64_t                           _scGen = 0;   // JS `topo._scGen | 0`: undefined -> 0
};

struct Region {
    std::vector<int> faces;
    double           area = 0;
};

struct AnalyzeResult {
    // over/kept/onBed alias the topology's scratch buffers in the JS; here they are
    // copies taken at the end of analyze() so a later analyze() cannot refill them
    // under a caller (fins_entry.js reads `over` before buildFins for that reason).
    std::vector<uint8_t> over, kept, onBed;
    std::vector<Region>  regions;          // area >= MIN_REGION_AREA, largest first (stable)
    int    rawRegionCount = 0;
    double overArea = 0, bedArea = 0;
    int    overFaceCount = 0;
    Offset offset;                          // where the rotated part sits (drop onto the plate)
    struct { double x = 0, y = 0, z = 0; } size;
};

/// buildTopology(geometry) for a soup (9 doubles per face).
Topology buildTopology(Soup pos);

/// analyze(topo, thresholdDeg, rot)
AnalyzeResult analyze(Topology &topo, double thresholdDeg = DEFAULT_THRESHOLD, const Rot &rot = IDENTITY3);

}} // namespace
