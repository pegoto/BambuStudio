// Port of web/prop/surface.js -- reading the seated part: `seat` rotates a raw vertex
// into print space, and `surfaceZAt` / `surfaceZsAt` / `surfaceHitsAt` answer "what
// part surface is directly above (x, y)" -- the lowest one, all of them, or all of
// them with their facing -- through a cached XY bucket grid.
#pragma once
#include "../Math.hpp"
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// Rotate + seat one raw vertex (pos[i..i+2]) into print space; writes and returns `out`.
Vec3d &seat(const Soup &pos, size_t i, const Rot &rot, const Offset &off, Vec3d &out);
/// The same, as a fresh vertex (JS `seat(pos, i, rot, off, [0, 0, 0])`).
inline Vec3d seat(const Soup &pos, size_t i, const Rot &rot, const Offset &off) {
    Vec3d v; seat(pos, i, rot, off, v); return v;
}

inline constexpr int ZGRID = 64;

/// The XY bucket grid surface.js builds over a flat triangle array (JS buildZGrid's g).
struct ZGrid {
    std::vector<int32_t> start;   // ZGRID*ZGRID + 1 prefix counts
    std::vector<int32_t> items;   // element offsets (f, multiples of 9) of the triangles per cell
    double minX = INF, minY = INF, maxX = -INF, maxY = -INF;
    double sx = 0, sy = 0;
};

/// JS buildZGrid(tris): build the grid over a flat triangle array (9 doubles per face).
ZGrid buildZGrid(const Soup &tris);

/// A flat triangle array (JS `tris`, a Float64Array of 9 doubles per face: a region, a
/// patch, or the whole seated part) with the XY grid surface.js caches on it. The JS
/// caches the grid on the array object (a WeakMap keyed by it); here it rides on the
/// object that owns the array and is built on the first query. JS never mutates an
/// array it has queried; if you must, call reset() afterwards.
struct SurfTris {
    Soup pos;
    SurfTris() = default;
    explicit SurfTris(Soup p) : pos(std::move(p)) {}
    size_t size() const { return pos.size(); }            // JS tris.length
    double operator[](size_t i) const { return pos[i]; }
    const ZGrid &grid() const;
    void reset() { _grid.reset(); }
private:
    mutable std::shared_ptr<const ZGrid> _grid;
};

/// The lowest surface height of `tris` directly above (x, y), or nullopt (JS null)
/// where no triangle covers the column.
std::optional<double> surfaceZAt(const SurfTris &tris, double x, double y);

/// One hit of surfaceHitsAt: [z, nz], nz the hit triangle's unit normal z.
using SurfHit = std::array<double, 2>;

/// Every hit above (x, y) as [z, nz], in triangle-index order within the grid cell.
std::vector<SurfHit> surfaceHitsAt(const SurfTris &tris, double x, double y);

/// Every part-surface height directly above (x, y), in the grid cell's order.
std::vector<double> surfaceZsAt(const SurfTris &tris, double x, double y);

}} // namespace
