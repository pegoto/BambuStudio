#include "Pieces.hpp"
#include <algorithm>
#include <map>
#include <unordered_map>

namespace Slic3r { namespace SupportFins {

std::vector<FloatingPiece> floatingPieces(const Topology &topo, const AnalyzeResult &result, const Rot &rot) {
    const Soup &pos = topo.pos;
    const int nFaces = topo.nFaces;
    const double RESTS_ON = 0.3;
    const Offset &off = result.offset;
    std::vector<int> parent(nFaces);
    for (int f = 0; f < nFaces; f++) parent[f] = f;
    auto find = [&](int x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    for (size_t e = 0; e < topo.adjA.size(); e++) {
        const int ra = find(topo.adjA[e]), rb = find(topo.adjB[e]);
        if (ra != rb) parent[ra] = rb;
    }
    std::vector<double> P(size_t(nFaces) * 9);
    for (size_t p = 0; p < size_t(nFaces) * 9; p += 3) {
        P[p] = rot[0] * pos[p] + rot[3] * pos[p + 1] + rot[6] * pos[p + 2] + off.x;
        P[p + 1] = rot[1] * pos[p] + rot[4] * pos[p + 1] + rot[7] * pos[p + 2] + off.y;
        P[p + 2] = rot[2] * pos[p] + rot[5] * pos[p + 1] + rot[8] * pos[p + 2] + off.z;
    }
    struct Group { int faces = 0; bool hasLowest = false; Vec3d lowest = Vec3d::Zero(); double vol = 0; };
    std::vector<int> root(nFaces);
    std::vector<int> order;                 // JS Map insertion order
    std::unordered_map<int, Group> pieces;
    for (int f = 0; f < nFaces; f++) {
        const int r = (root[f] = find(f));
        auto it = pieces.find(r);
        if (it == pieces.end()) { it = pieces.emplace(r, Group{}).first; order.push_back(r); }
        Group &g = it->second;
        g.faces++;
        const size_t t = size_t(f) * 9;
        for (size_t i = 0; i < 9; i += 3) {
            if (!g.hasLowest || P[t + i + 2] < g.lowest.z()) { g.hasLowest = true; g.lowest = Vec3d(P[t + i], P[t + i + 1], P[t + i + 2]); }
        }
        g.vol += P[t] * (P[t + 4] * P[t + 8] - P[t + 5] * P[t + 7])
               - P[t + 1] * (P[t + 3] * P[t + 8] - P[t + 5] * P[t + 6])
               + P[t + 2] * (P[t + 3] * P[t + 7] - P[t + 4] * P[t + 6]);
    }
    if (pieces.size() < 2) return {};

    std::vector<std::pair<int, const Group *>> lifted;
    for (int r : order) {
        const Group &g = pieces[r];
        if (g.vol > 0 && g.lowest.z() >= BED_EPS) lifted.emplace_back(r, &g);
    }
    if (lifted.empty()) return {};

    double x0 = INF, y0 = INF, x1 = -INF, y1 = -INF;
    for (size_t p = 0; p < P.size(); p += 3) {
        if (P[p] < x0) x0 = P[p]; if (P[p] > x1) x1 = P[p];
        if (P[p + 1] < y0) y0 = P[p + 1]; if (P[p + 1] > y1) y1 = P[p + 1];
    }
    const int G = std::max(1, std::min(512, int(js_round(std::sqrt(nFaces / 4.0)))));
    const double cw = std::max(1e-6, (x1 - x0) / G), ch = std::max(1e-6, (y1 - y0) / G);
    auto cx = [&](double x) { return std::min(G - 1, std::max(0, int(std::floor((x - x0) / cw)))); };
    auto cy = [&](double y) { return std::min(G - 1, std::max(0, int(std::floor((y - y0) / ch)))); };
    std::unordered_map<int, std::vector<int>> cells;
    for (int f = 0; f < nFaces; f++) {
        const size_t t = size_t(f) * 9;
        const int i0 = cx(std::min({P[t], P[t + 3], P[t + 6]})), i1 = cx(std::max({P[t], P[t + 3], P[t + 6]}));
        const int j0 = cy(std::min({P[t + 1], P[t + 4], P[t + 7]})), j1 = cy(std::max({P[t + 1], P[t + 4], P[t + 7]}));
        for (int i = i0; i <= i1; i++)
            for (int j = j0; j <= j1; j++)
                cells[i * G + j].push_back(f);
    }

    std::vector<FloatingPiece> out;
    for (const auto &[r, g] : lifted) {
        const double px = g->lowest.x(), py = g->lowest.y(), pz = g->lowest.z();
        double below = -INF;
        auto it = cells.find(cx(px) * G + cy(py));
        if (it != cells.end()) {
            for (int f : it->second) {
                if (root[f] == r) continue;
                const size_t t = size_t(f) * 9;
                const double ax = P[t], ay = P[t + 1], bx = P[t + 3], by = P[t + 4], qx = P[t + 6], qy = P[t + 7];
                const double d = (by - qy) * (ax - qx) + (qx - bx) * (ay - qy);
                if (std::abs(d) < 1e-12) continue;
                const double l1 = ((by - qy) * (px - qx) + (qx - bx) * (py - qy)) / d;
                const double l2 = ((qy - ay) * (px - qx) + (ax - qx) * (py - qy)) / d;
                const double l3 = 1 - l1 - l2;
                if (l1 < -1e-9 || l2 < -1e-9 || l3 < -1e-9) continue;
                const double z = l1 * P[t + 2] + l2 * P[t + 5] + l3 * P[t + 8];
                if (z <= pz + 1e-6 && z > below) below = z;
            }
        }
        const double drop = below == -INF ? pz : pz - below;
        if (drop > RESTS_ON) out.push_back({g->faces, g->lowest, drop});
    }
    return out;
}

}} // namespace
