#include "Entry.hpp"
#include "Materials.hpp"
#include <algorithm>
#include <stdexcept>

namespace Slic3r { namespace SupportFins {

Offset seatSoup(const Soup &input, Soup &pos) {
    if (input.empty() || input.size() % 9 != 0)
        throw std::invalid_argument("positions must be a non-empty triangle soup (9 floats/face)");
    double x0 = INF, x1 = -INF, y0 = INF, y1 = -INF, z0 = INF;
    for (size_t i = 0; i < input.size(); i += 3) {
        const double x = input[i], y = input[i + 1], z = input[i + 2];
        if (x < x0) x0 = x; if (x > x1) x1 = x;
        if (y < y0) y0 = y; if (y > y1) y1 = y;
        if (z < z0) z0 = z;
    }
    const double cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    pos.resize(input.size());
    for (size_t i = 0; i < input.size(); i += 3) {
        pos[i] = snap_nm(input[i] - cx);
        pos[i + 1] = snap_nm(input[i + 1] - cy);
        pos[i + 2] = snap_nm(input[i + 2] - z0);
    }
    return {cx, cy, z0};
}

static void pick(const char *name, const std::string &v, std::initializer_list<const char *> allowed) {
    for (const char *a : allowed) if (v == a) return;
    throw std::invalid_argument(std::string(name) + " has an unknown value: " + v);
}

EntryResult computeFins(const Soup &positions, const EntryOptions &opts) {
    // seat.js clearances(opts)
    const Material *mat = material_of(opts.material);
    if (!mat) throw std::invalid_argument("material must be one of pla, petg, got " + opts.material);
    pick("padStyle", opts.padStyle, {"off", "auto", "light", "sure"});
    pick("cutout", opts.cutout, {"none", "diamond", "triangle", "arch", "lattice"});
    Tunables tunables;
    tunables.padH = mat->padH; tunables.padGrab = mat->padGrab; tunables.propGap = mat->propGap;
    tunables.padStyle = opts.padStyle == "off" ? "auto" : opts.padStyle;
    tunables.cutout = opts.cutout;

    Soup pos;
    const Offset shift = seatSoup(positions, pos);
    Topology topo = buildTopology(std::move(pos));
    AnalyzeResult result = analyze(topo, opts.threshold, IDENTITY3);
    EntryResult out;
    for (int f = 0; f < int(result.over.size()); f++) {
        if (result.kept[f]) out.overFaces.push_back(f); else if (result.over[f]) out.smallFaces.push_back(f);
    }

    BuildOpts bo;
    bo.mode = opts.mode;
    bo.bedPad = opts.bedPad && opts.padStyle != "off";
    bo.tines = opts.tines;
    bo.tineDensity = opts.tineDensity;
    bo.layerHeight = opts.layerHeight;
    bo.coverage = opts.coverage;
    bo.plateOnly = opts.plateOnly;
    if (opts.sway) { bo.sway = *opts.sway; /* gap: the host's, else the material's */ }
    bo.tunables = tunables;
    Built built = buildFins(topo, result, IDENTITY3, bo);

    const size_t finTris = built.triangles.size() / 3, padTris = built.padTriangles.size() / 3;
    out.triangles.reserve((finTris + padTris) * 9);
    for (const Vec3d &v : built.triangles) { out.triangles.push_back(float(v.x())); out.triangles.push_back(float(v.y())); out.triangles.push_back(float(v.z())); }
    for (const Vec3d &v : built.padTriangles) { out.triangles.push_back(float(v.x())); out.triangles.push_back(float(v.y())); out.triangles.push_back(float(v.z())); }

    // piecesOf: fin records carry element ranges (vertex triples -> /3), the pad after them
    int swayTris = 0;
    for (const FinRecord &f : built.fins) {
        PieceRange p; p.id = std::to_string(f.id); p.kind = f.kind;
        for (const TriRange &r : f.triRanges) {
            p.ranges.emplace_back(r.first / 3, r.second / 3);
            if (f.kind == "sway") swayTris += (r.second - r.first) / 3;
        }
        out.pieces.push_back(std::move(p));
    }
    if (padTris) out.pieces.push_back({"pad", "pad", {{int(finTris), int(finTris + padTris)}}});

    out.offset = {result.offset.x - shift.x, result.offset.y - shift.y, result.offset.z - shift.z};
    EntryStats &s = out.stats;
    s.overhangRegions = int(result.regions.size());
    s.finTriangles = int(finTris) - swayTris;
    s.swayTriangles = swayTris;
    s.padTriangles = int(padTris);
    s.braces = built.braceCount; s.props = built.propCount; s.tines = built.tines;
    if (built.sway) { s.swayBraces = built.sway->count; s.swayTines = built.sway->tines; s.swaySkipped = built.sway->skipped; s.swayReason = built.sway->reason; }
    s.unserved = built.unserved;
    s.floating = int(built.floating.size());
    s.floatingDrop = built.floating.empty() ? 0 : built.floating[0].drop;
    return out;
}

}} // namespace
