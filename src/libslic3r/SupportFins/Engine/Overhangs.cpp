#include "Overhangs.hpp"
#include <algorithm>
#include <unordered_map>

namespace Slic3r { namespace SupportFins {

namespace {
struct Key3 {
    int64_t x, y, z;
    bool operator==(const Key3 &o) const { return x == o.x && y == o.y && z == o.z; }
};
struct Key3Hash {
    size_t operator()(const Key3 &k) const {
        uint64_t h = uint64_t(k.x) * 0x9E3779B97F4A7C15ull;
        h ^= uint64_t(k.y) + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
        h ^= uint64_t(k.z) + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
        return size_t(h);
    }
};
/// Quantise to 1e-3 mm so vertices shared between triangles weld together (JS key3).
inline Key3 key3(double x, double y, double z) {
    return {int64_t(js_round(x * 1000)), int64_t(js_round(y * 1000)), int64_t(js_round(z * 1000))};
}
} // namespace

Topology buildTopology(Soup pos_in) {
    Topology t;
    t.pos = std::move(pos_in);
    const Soup &pos = t.pos;
    const int nFaces = int(pos.size() / 9);
    t.nFaces = nFaces;
    t.nrm.assign(size_t(nFaces) * 3, 0.0);
    t.area.assign(nFaces, 0.0);

    std::unordered_map<Key3, int, Key3Hash> weld;
    weld.reserve(size_t(nFaces) * 2);
    std::vector<int> vid(size_t(nFaces) * 3);

    for (int f = 0; f < nFaces; f++) {
        const size_t o = size_t(f) * 9;
        const double ax = pos[o], ay = pos[o + 1], az = pos[o + 2];
        const double bx = pos[o + 3], by = pos[o + 4], bz = pos[o + 5];
        const double cx = pos[o + 6], cy = pos[o + 7], cz = pos[o + 8];
        const double ux = bx - ax, uy = by - ay, uz = bz - az;
        const double vx = cx - ax, vy = cy - ay, vz = cz - az;
        const double px = uy * vz - uz * vy;
        const double py = uz * vx - ux * vz;
        const double pz = ux * vy - uy * vx;
        const double len = hypot3(px, py, pz);
        if (len > 1e-12) {
            t.nrm[size_t(f) * 3] = px / len;
            t.nrm[size_t(f) * 3 + 1] = py / len;
            t.nrm[size_t(f) * 3 + 2] = pz / len;
        }
        t.area[f] = 0.5 * len;
        for (int i = 0; i < 3; i++) {
            const Key3 k = key3(pos[o + i * 3], pos[o + i * 3 + 1], pos[o + i * 3 + 2]);
            auto it = weld.find(k);
            int id;
            if (it == weld.end()) { id = int(weld.size()); weld.emplace(k, id); }
            else id = it->second;
            vid[size_t(f) * 3 + i] = id;
        }
    }

    // face adjacency via shared welded edges
    std::unordered_map<uint64_t, int> firstFace;
    firstFace.reserve(size_t(nFaces) * 3);
    for (int f = 0; f < nFaces; f++) {
        for (int i = 0; i < 3; i++) {
            const int a = vid[size_t(f) * 3 + i], b = vid[size_t(f) * 3 + (i + 1) % 3];
            const uint64_t k = a < b ? (uint64_t(a) << 32 | uint64_t(b)) : (uint64_t(b) << 32 | uint64_t(a));
            auto it = firstFace.find(k);
            if (it == firstFace.end()) firstFace.emplace(k, f);
            else { t.adjA.push_back(it->second); t.adjB.push_back(f); }
        }
    }
    t.vertexCount = int(weld.size());
    t.edgeCount = int(firstFace.size());
    t._zr.assign(size_t(nFaces) * 3, 0.0);
    t._parent.assign(nFaces, 0);
    t._over.assign(nFaces, 0);
    t._kept.assign(nFaces, 0);
    t._onBed.assign(nFaces, 0);
    return t;
}

AnalyzeResult analyze(Topology &topo, double thresholdDeg, const Rot &rot) {
    const Soup &pos = topo.pos;
    const int nFaces = topo.nFaces;
    const auto &nrm = topo.nrm;
    const auto &area = topo.area;
    const double cut = -(std::cos((thresholdDeg * M_PI) / 180) + ANGLE_EPS);

    auto &zr = topo._zr;
    double minX = INF, maxX = -INF, minY = INF, maxY = -INF, minZ = INF, maxZ = -INF;
    for (int v = 0, p = 0; v < nFaces * 3; v++, p += 3) {
        const double x = pos[p], y = pos[p + 1], z = pos[p + 2];
        const double xr = rot[0] * x + rot[3] * y + rot[6] * z;
        const double yr = rot[1] * x + rot[4] * y + rot[7] * z;
        const double zz = rot[2] * x + rot[5] * y + rot[8] * z;
        zr[v] = zz;
        if (xr < minX) minX = xr; if (xr > maxX) maxX = xr;
        if (yr < minY) minY = yr; if (yr > maxY) maxY = yr;
        if (zz < minZ) minZ = zz; if (zz > maxZ) maxZ = zz;
    }

    auto &onBed = topo._onBed; std::fill(onBed.begin(), onBed.end(), 0);
    auto &over = topo._over;   std::fill(over.begin(), over.end(), 0);
    double bedArea = 0, overArea = 0; int overFaceCount = 0;

    for (int f = 0; f < nFaces; f++) {
        const size_t z3 = size_t(f) * 3;
        const double h = std::max({zr[z3], zr[z3 + 1], zr[z3 + 2]}) - minZ;
        if (h < BED_EPS) { onBed[f] = 1; bedArea += area[f]; continue; }
        const double nzr = rot[2] * nrm[z3] + rot[5] * nrm[z3 + 1] + rot[8] * nrm[z3 + 2];
        if (nzr < cut) { over[f] = 1; overArea += area[f]; overFaceCount++; }
    }

    // union-find over adjacent overhang faces
    auto &parent = topo._parent;
    for (int f = 0; f < nFaces; f++) parent[f] = f;
    auto find = [&](int x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    for (size_t e = 0; e < topo.adjA.size(); e++) {
        const int a = topo.adjA[e], b = topo.adjB[e];
        if (!over[a] || !over[b]) continue;
        const int ra = find(a), rb = find(b);
        if (ra != rb) parent[ra] = rb;
    }

    // JS: Map keyed by root, insertion order = first face seen per root
    std::vector<int> rootIndex(nFaces, -1);
    std::vector<Region> raw;
    for (int f = 0; f < nFaces; f++) {
        if (!over[f]) continue;
        const int r = find(f);
        if (rootIndex[r] < 0) { rootIndex[r] = int(raw.size()); raw.push_back(Region{{f}, area[f]}); }
        else { Region &g = raw[rootIndex[r]]; g.faces.push_back(f); g.area += area[f]; }
    }

    AnalyzeResult res;
    for (const Region &g : raw) if (g.area >= MIN_REGION_AREA) res.regions.push_back(g);
    std::stable_sort(res.regions.begin(), res.regions.end(), [](const Region &a, const Region &b) { return a.area > b.area; });

    auto &kept = topo._kept; std::fill(kept.begin(), kept.end(), 0);
    for (const Region &g : res.regions) for (int f : g.faces) kept[f] = 1;

    res.over = over; res.kept = kept; res.onBed = onBed;
    res.rawRegionCount = int(raw.size());
    res.overArea = overArea; res.bedArea = bedArea; res.overFaceCount = overFaceCount;
    res.offset = {-(minX + maxX) / 2, -(minY + maxY) / 2, -minZ};
    res.size = {maxX - minX, maxY - minY, maxZ - minZ};
    return res;
}

}} // namespace
