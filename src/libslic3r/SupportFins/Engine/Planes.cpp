// Port of web/planes.js -- see Planes.hpp.
#include "Planes.hpp"
#include "Ieee754.hpp"
#include <algorithm>
#include <cmath>

namespace Slic3r { namespace SupportFins {

namespace {

/// Rotate + seat one raw vertex into print space.
inline void worldVertex(const Soup &pos, size_t o, const Rot &rot, const Offset &offset, double out[3]) {
    const double x = pos[o], y = pos[o + 1], z = pos[o + 2];
    out[0] = rot[0] * x + rot[3] * y + rot[6] * z + offset.x;
    out[1] = rot[1] * x + rot[4] * y + rot[7] * z + offset.y;
    out[2] = rot[2] * x + rot[5] * y + rot[8] * z + offset.z;
}

/// The face group fitPatch() measures (JS `{ faces, area: total }`).
struct FaceGroup {
    std::vector<int> faces;
    double           area = 0;
};

/// Fit a plane to one face group and measure it, or return nullopt if it is not a
/// usable fin site.
///
/// The plane is `n . q = d`, with `n` the outward 3D normal. Its frame is:
///   n  out of the part
///   u  across the face, always HORIZONTAL -- so the fin's length runs level,
///      and a tine's width lies in the layer plane no matter how the part leans
///   t  up the face, n x u, which always has a positive z component
std::optional<WallPatch> fitPatch(const Topology &topo, FaceGroup &g, const std::vector<double> &rn,
                                  const Rot &rot, const Offset &offset, PatchStats *stats) {
    const Soup &pos = topo.pos;
    const std::vector<double> &area = topo.area;

    // area-weighted mean normal: a patch's big faces describe its plane, and a fan
    // of slivers in one corner should not be able to tilt it
    double nx = 0, ny = 0, nz = 0;
    for (const int f : g.faces) {
        nx += rn[size_t(f) * 3] * area[f];
        ny += rn[size_t(f) * 3 + 1] * area[f];
        nz += rn[size_t(f) * 3 + 2] * area[f];
    }
    const double nl = hypot3(nx, ny, nz);
    if (nl < 1e-9) return std::nullopt;
    nx /= nl; ny /= nl; nz /= nl;

    const double h = hypot2(nx, ny);           // horizontal share of the normal
    if (h < 1e-6) {                            // a floor or ceiling, not a wall
        if (stats) stats->notUpright++;
        return std::nullopt;
    }
    const double ux = -ny / h, uy = nx / h;    // horizontal, across the face

    // t = n x u, up the face. Built from n's OWN horizontal direction (nx/h, ny/h),
    // not from u's components (that yields a vector not even in the patch plane).
    const double tx = -nz * (nx / h), ty = -nz * (ny / h), tz = h;

    // The plane is a SUPPORTING plane -- it touches the outermost point of the patch
    // and the whole surface lies behind it -- not a mean plane through the middle, so
    // the standoff is a floor rather than an average.
    double v[3];
    double d = -INF;
    for (const int f : g.faces) {
        for (int i = 0; i < 3; i++) {
            worldVertex(pos, size_t(f) * 9 + i * 3, rot, offset, v);
            const double dn = v[0] * nx + v[1] * ny + v[2] * nz;
            if (dn > d) d = dn;
        }
    }

    // flatness, extent, and the (u, t) projection in one pass
    // (u, t, dn) per vertex: the in-plane coordinates plus the SIGNED distance
    // from the fitted plane, which is what lets each tine size itself later.
    std::vector<double> tris(g.faces.size() * 9, 0.0);
    double devOut = -INF, devIn = INF;
    double u0 = INF, u1 = -INF, t0 = INF, t1 = -INF;
    double z0 = INF, z1 = -INF;
    for (size_t k = 0; k < g.faces.size(); k++) {
        const int f = g.faces[k];
        for (int i = 0; i < 3; i++) {
            worldVertex(pos, size_t(f) * 9 + i * 3, rot, offset, v);
            const double dn = v[0] * nx + v[1] * ny + v[2] * nz - d;
            if (dn > devOut) devOut = dn;
            if (dn < devIn) devIn = dn;
            const double u = v[0] * ux + v[1] * uy;
            const double t = v[0] * tx + v[1] * ty + v[2] * tz;
            tris[k * 9 + i * 3] = u;
            tris[k * 9 + i * 3 + 1] = t;
            tris[k * 9 + i * 3 + 2] = dn;
            if (u < u0) u0 = u;
            if (u > u1) u1 = u;
            if (t < t0) t0 = t;
            if (t > t1) t1 = t;
            if (v[2] < z0) z0 = v[2];
            if (v[2] > z1) z1 = v[2];
        }
    }

    if (devIn < -FLAT_TOL_IN) { if (stats) stats->notFlat++; return std::nullopt; }
    if (t1 - t0 < MIN_PATCH_H) { if (stats) stats->tooShort++; return std::nullopt; }
    if (u1 - u0 < MIN_PATCH_W) { if (stats) stats->tooNarrow++; return std::nullopt; }

    // where this site sits, for keeping chosen fins apart in space
    const double umid = (u0 + u1) / 2, tmid = (t0 + t1) / 2;

    WallPatch p;
    p.faces = std::move(g.faces);
    p.area = g.area;
    p.mid = {nx * d + ux * umid + tx * tmid, ny * d + uy * umid + ty * tmid};
    p.n = {nx, ny, nz};
    p.u = {ux, uy};
    p.t = {tx, ty, tz};
    p.h = h; p.d = d;
    p.u0 = u0; p.u1 = u1; p.t0 = t0; p.t1 = t1; p.z0 = z0; p.z1 = z1;
    p.lean = std::abs(js_asin(nz) * 180 / M_PI);
    p.flatness = std::max(devOut, -devIn);   // never NaN here, so std::max == Math.max
    p.tris = std::move(tris);
    return p;
}

} // namespace

std::vector<WallPatch> findWallPatches(const Topology &topo, const Rot &rot, const Offset &offset, PatchStats *stats) {
    const Soup &pos = topo.pos;
    const int nFaces = topo.nFaces;
    const std::vector<double> &nrm = topo.nrm;
    const std::vector<double> &area = topo.area;
    // A hair of slack past the exact sine: a cube on its edge (45 degrees) lands on
    // sin(45) to the last float bit, so a bare `<=` let winding round-off decide.
    const double leanCut = js_sin((MAX_LEAN_DEG * M_PI) / 180) + 1e-6;
    const double agreeCut = js_cos((NORMAL_AGREE_DEG * M_PI) / 180);

    // rotated face normals, kept because both the grouping test and the plane fit
    // need them and they are not worth recomputing per edge
    std::vector<double> rn(size_t(nFaces) * 3, 0.0);
    std::vector<uint8_t> upright(nFaces, 0);
    for (int f = 0; f < nFaces; f++) {
        const size_t i = size_t(f) * 3;
        const double x = nrm[i], y = nrm[i + 1], z = nrm[i + 2];
        const double nx = rot[0] * x + rot[3] * y + rot[6] * z;
        const double ny = rot[1] * x + rot[4] * y + rot[7] * z;
        const double nz = rot[2] * x + rot[5] * y + rot[8] * z;
        rn[i] = nx; rn[i + 1] = ny; rn[i + 2] = nz;
        if (std::abs(nz) <= leanCut && hypot2(nx, ny) > 1e-9) upright[f] = 1;
    }

    // Region-grow each patch from a seed face, testing every candidate against the
    // SEED's plane rather than against its neighbour's: pairwise agreement creeps
    // (a cylindrical boss merges into one "flat" group), a fixed seed plane cannot.
    const FaceAdjacency adj = faceAdjacency(topo);
    const std::vector<int> &start = adj.start;
    const std::vector<int> &nbr = adj.nbr;
    std::vector<int> seeds;
    for (int f = 0; f < nFaces; f++) if (upright[f]) seeds.push_back(f);
    // biggest face defines the plane (JS sort is stable)
    std::stable_sort(seeds.begin(), seeds.end(), [&](int a, int b) { return area[b] - area[a] < 0; });

    std::vector<uint8_t> taken(nFaces, 0);
    double v[3] = {0, 0, 0};
    std::vector<WallPatch> patches;
    std::vector<int> queue;

    for (const int seed : seeds) {
        if (taken[seed]) continue;
        const double sx = rn[size_t(seed) * 3], sy = rn[size_t(seed) * 3 + 1], sz = rn[size_t(seed) * 3 + 2];

        double sd = 0;
        for (int i = 0; i < 3; i++) {
            worldVertex(pos, size_t(seed) * 9 + i * 3, rot, offset, v);
            sd += v[0] * sx + v[1] * sy + v[2] * sz;
        }
        sd /= 3;

        taken[seed] = 1;
        FaceGroup g;
        g.faces.push_back(seed);
        double total = area[seed];
        queue.clear();
        queue.push_back(seed);

        while (!queue.empty()) {
            const int f = queue.back();
            queue.pop_back();
            for (int e = start[f]; e < start[f + 1]; e++) {
                const int gf = nbr[e];
                if (taken[gf] || !upright[gf]) continue;
                const double dot = rn[size_t(gf) * 3] * sx + rn[size_t(gf) * 3 + 1] * sy + rn[size_t(gf) * 3 + 2] * sz;
                if (dot < agreeCut) continue;
                double outward = -INF, inward = INF;
                for (int i = 0; i < 3; i++) {
                    worldVertex(pos, size_t(gf) * 9 + i * 3, rot, offset, v);
                    const double dn = v[0] * sx + v[1] * sy + v[2] * sz - sd;
                    if (dn > outward) outward = dn;
                    if (dn < inward) inward = dn;
                }
                if (outward > GROW_SLACK || inward < -FLAT_TOL_IN) continue;
                taken[gf] = 1;
                g.faces.push_back(gf);
                total += area[gf];
                queue.push_back(gf);
            }
        }

        if (stats) stats->grown++;
        if (total < MIN_PATCH_AREA) {
            if (stats) stats->tooSmall++;
            continue;
        }
        g.area = total;
        std::optional<WallPatch> patch = fitPatch(topo, g, rn, rot, offset, stats);
        if (patch) patches.push_back(std::move(*patch));
    }

    std::stable_sort(patches.begin(), patches.end(), [](const WallPatch &a, const WallPatch &b) { return b.area - a.area < 0; });
    return patches;
}

FaceAdjacency faceAdjacency(const Topology &topo) {
    const int nFaces = topo.nFaces;
    const std::vector<int> &adjA = topo.adjA, &adjB = topo.adjB;
    FaceAdjacency r;
    std::vector<int> &start = r.start;
    start.assign(size_t(nFaces) + 1, 0);
    for (size_t e = 0; e < adjA.size(); e++) { start[adjA[e] + 1]++; start[adjB[e] + 1]++; }
    for (int f = 0; f < nFaces; f++) start[f + 1] += start[f];
    r.nbr.assign(size_t(start[nFaces]), 0);
    std::vector<int> cur(start.begin(), start.begin() + nFaces);
    for (size_t e = 0; e < adjA.size(); e++) {
        r.nbr[cur[adjA[e]]++] = adjB[e];
        r.nbr[cur[adjB[e]]++] = adjA[e];
    }
    return r;
}

bool patchCovers(const WallPatch &patch, double u, double t) {
    return patchProbe(patch, u, t).has_value();
}

std::optional<double> patchProbe(const WallPatch &patch, double u, double t) {
    const std::vector<double> &tri = patch.tris;
    for (size_t i = 0; i < tri.size(); i += 9) {
        const double ax = tri[i], ay = tri[i + 1];
        const double bx = tri[i + 3], by = tri[i + 4];
        const double cx = tri[i + 6], cy = tri[i + 7];

        // Skip slivers. A triangle with no projected area gives d1 = d2 = d3 = 0,
        // which reads as "inside" and makes the whole coverage test pass for every
        // point -- resurrecting the tine-into-thin-air case this exists to prevent.
        const double den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
        if (std::abs(den) < 1e-9) continue;

        const double l1 = ((by - cy) * (u - cx) + (cx - bx) * (t - cy)) / den;
        const double l2 = ((cy - ay) * (u - cx) + (ax - cx) * (t - cy)) / den;
        const double l3 = 1 - l1 - l2;
        if (l1 < -1e-9 || l2 < -1e-9 || l3 < -1e-9) continue;

        return l1 * tri[i + 2] + l2 * tri[i + 5] + l3 * tri[i + 8];
    }
    return std::nullopt;
}

Vec3d patchPoint(const WallPatch &p, double w, double u, double t) {
    const double dw = p.d + w;
    return Vec3d(p.n.x * dw + p.u.x * u + p.t.x * t,
                 p.n.y * dw + p.u.y * u + p.t.y * t,
                 p.n.z * dw + p.t.z * t);
}

}} // namespace Slic3r::SupportFins
