// Port of web/pieces.js -- the closed shells a mesh is made of, and which start in mid-air.
#pragma once
#include "Overhangs.hpp"

namespace Slic3r { namespace SupportFins {

struct FloatingPiece {
    int    faces = 0;
    Vec3d  lowest = Vec3d::Zero();
    double drop = 0;
};

/// floatingPieces(topo, result, rot): pieces that start in mid-air, in the seated frame.
std::vector<FloatingPiece> floatingPieces(const Topology &topo, const AnalyzeResult &result, const Rot &rot = IDENTITY3);

}} // namespace
