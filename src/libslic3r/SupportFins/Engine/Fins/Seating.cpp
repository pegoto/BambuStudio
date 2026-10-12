#include "Seating.hpp"
#include "../FinConfig.hpp"

namespace Slic3r { namespace SupportFins {

Soup seatedPartTris(const Topology &topo, const Rot &rot, const Offset &offset) {
    const Soup &pos = topo.pos;
    const int nFaces = topo.nFaces;
    const double ox = offset.x, oy = offset.y, oz = offset.z;
    Soup tris(size_t(nFaces) * 9);
    for (int f = 0; f < nFaces; f++) {
        for (int i = 0; i < 3; i++) {
            const size_t o = size_t(f) * 9 + i * 3;
            const double x = pos[o], y = pos[o + 1], z = pos[o + 2];
            tris[o] = rot[0] * x + rot[3] * y + rot[6] * z + ox;
            tris[o + 1] = rot[1] * x + rot[4] * y + rot[7] * z + oy;
            tris[o + 2] = rot[2] * x + rot[5] * y + rot[8] * z + oz;
        }
    }
    return tris;
}

BedContact bedContact(const Topology &topo, const AnalyzeResult &result, const Rot &rot) {
    const Soup &pos = topo.pos;
    const int nFaces = topo.nFaces;
    const auto &area = topo.area;
    const double ox = result.offset.x, oy = result.offset.y, oz = result.offset.z;
    double mx = 0, my = 0, mw = 0;
    BedContact out;
    for (int f = 0; f < nFaces; f++) {
        double gx = 0, gy = 0;
        for (int i = 0; i < 3; i++) {
            const size_t o = size_t(f) * 9 + i * 3;
            const double x = pos[o], y = pos[o + 1], z = pos[o + 2];
            const double wx = rot[0] * x + rot[3] * y + rot[6] * z + ox;
            const double wy = rot[1] * x + rot[4] * y + rot[7] * z + oy;
            const double wz = rot[2] * x + rot[5] * y + rot[8] * z + oz;
            gx += wx; gy += wy;
            if (wz < BED_EPS) out.pts.emplace_back(wx, wy);
        }
        mx += (gx / 3) * area[f]; my += (gy / 3) * area[f]; mw += area[f];
    }
    if (mw > 0) { mx /= mw; my /= mw; }
    out.mx = mx; out.my = my;
    return out;
}

Seating seatingOf(const AnalyzeResult &result, const std::vector<Vec2d> &contactPts) {
    double minX = INF, maxX = -INF, minY = INF, maxY = -INF;
    for (const Vec2d &p : contactPts) {
        if (p.x() < minX) minX = p.x(); if (p.x() > maxX) maxX = p.x();
        if (p.y() < minY) minY = p.y(); if (p.y() > maxY) maxY = p.y();
    }
    const double span = contactPts.empty() ? 0 : hypot2(maxX - minX, maxY - minY);
    Seating s;
    s.kind = result.bedArea >= FIN.padMinArea ? "face" : span < POINT_FOOTPRINT ? "point" : "edge";
    s.span = span; s.bedArea = result.bedArea;
    return s;
}

}} // namespace
