// Port of web/fins/seating.js -- how the part sits on the plate: seatedPartTris lays
// the part out in print space, bedContact finds where it touches down (and its centre
// of mass), seatingOf calls that contact a face, an edge or a point.
#pragma once
#include "../Overhangs.hpp"
#include <string>

namespace Slic3r { namespace SupportFins {

/// The whole part, seated into print space, as a flat triangle array (9 per face).
Soup seatedPartTris(const Topology &topo, const Rot &rot, const Offset &offset);

struct BedContact {
    std::vector<Vec2d> pts;   // vertices under BED_EPS, seated (x, y)
    double mx = 0, my = 0;    // area-weighted centre of mass (x, y)
};
BedContact bedContact(const Topology &topo, const AnalyzeResult &result, const Rot &rot);

inline constexpr double POINT_FOOTPRINT = 2.0;   // mm; contact narrower than this is a point

struct Seating {
    std::string kind;   // face | point | edge
    double span = 0;
    double bedArea = 0;
};
Seating seatingOf(const AnalyzeResult &result, const std::vector<Vec2d> &contactPts);

}} // namespace
