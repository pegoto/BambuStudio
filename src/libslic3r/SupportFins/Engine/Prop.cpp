// Port of web/prop.js -- buildProps / buildShortWalls / noProps / coverRowSpan. See
// Prop.hpp. The pieces live in Prop/ (one header per web/prop/ module).
#include "Prop.hpp"
#include "Inside.hpp"
#include "Prop/Attached.hpp"
#include "Prop/Clearance.hpp"
#include "Prop/Contact.hpp"
#include "Prop/Crest.hpp"
#include "Prop/Lattice.hpp"
#include "Prop/Squat.hpp"
#include "Prop/Surface.hpp"
#include "Prop/Sweep.hpp"
#include "Prop/Tines.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <string>

namespace Slic3r { namespace SupportFins {

namespace {
// JS Math.min / Math.max: NaN if either is NaN; -0 < +0.
inline double jsMin(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    if (a == b) return std::signbit(a) ? a : b;
    return a < b ? a : b;
}
inline double jsMax(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
    if (a == b) return std::signbit(a) ? b : a;
    return a > b ? a : b;
}
// JS `out.length = n`, with the parallel interface tags.
inline void truncate(Tris &out, IfaceTags &tags, size_t n) {
    if (out.size() > n) out.resize(n);
    if (tags.size() > n) tags.resize(n);
}
// JS skipped[reason]++ for the reasons buildPass / buildPartAttached name.
void bump(Skipped &s, const std::string &why) {
    if (why == "noLine") s.noLine++;
    else if (why == "wanders") s.wanders++;
    else if (why == "stub") s.stub++;
    else if (why == "blocked") s.blocked++;
    else if (why == "degenerate") s.degenerate++;
    else if (why == "buried") s.buried++;
    else if (why == "weld") s.weld++;
    else if (why == "sliver") s.sliver++;
    else if (why == "onPart") s.onPart++;
    // UPSTREAM: any other key would create a NaN-valued property; none is produced.
}

/// One placement pass. `raster` nullptr = the normal placement for every region; a set
/// of region indices = raster tracks (rasterTracks) for those regions only.
PropsResult buildPass(const Topology &topo, const AnalyzeResult &result, const Rot &rot, const PropOpts &opts,
                      const RegionSet *raster) {
    const Soup &pos = topo.pos;
    const double step = opts.step ? *opts.step : PROP.stationStep;
    // Wide-face coverage (0 sparse .. 1 dense) sets the row spacing via coverRowSpan.
    const double coverage = jsMax(0, jsMin(1, opts.coverage ? *opts.coverage : 0.5));
    const double rowSpan = coverRowSpan(coverage);
    // sagRisk warns ONLY when the user asked for row pitch wider than the anti-sag cap.
    const bool wantSparse = rowSpan > PROP.maxUnsupportedSpan + 0.5;
    const double zBed = 0;
    const Offset off = result.offset;
    const bool withTines = opts.tines;
    const RegionSet *only = opts.onlyRegions ? &*opts.onlyRegions : nullptr;
    const bool shortW = opts.shortWalls;
    // tines emitted; sagRisk: the user chose sub-cap spacing AND a real row landed wide
    Tally tally;

    PropsResult res;
    Tris &out = res.triangles;
    IfaceTags &tags = res.tags;
    std::vector<PropRecord> &props = res.props;
    // Per-fin identity, assigned in build order.
    int nextId = 0;
    Skipped &skipped = res.skipped;
    // Plate only (#218): every support stands on the plate.
    const bool plateOnly = opts.plateOnly;
    Vec3d v(0, 0, 0);

    // The whole part, seated once, for the part-attached floor probe.
    auto partTrisP = std::make_shared<SurfTris>(Soup(pos.size()));
    Soup &partTris = partTrisP->pos;
    double minX = INF, maxX = -INF, minY = INF, maxY = -INF, minZ = INF, maxZ = -INF;
    for (int f = 0; f < topo.nFaces; f++) {
        for (int i = 0; i < 3; i++) {
            const size_t o = size_t(f) * 9 + size_t(i) * 3;
            seat(pos, o, rot, off, v);
            partTris[o] = v[0];
            partTris[o + 1] = v[1];
            partTris[o + 2] = v[2];
            if (v[0] < minX) minX = v[0]; if (v[0] > maxX) maxX = v[0];
            if (v[1] < minY) minY = v[1]; if (v[1] > maxY) maxY = v[1];
            if (v[2] < minZ) minZ = v[2]; if (v[2] > maxZ) maxZ = v[2];
        }
    }
    (void)minX; (void)maxX; (void)minY; (void)maxY; (void)minZ; (void)maxZ;   // UPSTREAM: computed, never read
    const SurfTris &part = *partTrisP;

    // DENSE grip comb by default; the "Tine grip" setting loosens it.
    const double tineStepEff = tineStepFor(opts.tineDensity);
    // A tine is exactly one slicer layer tall.
    const double tineHeight = opts.layerHeight ? *opts.layerHeight : PROP.tineH;
    // where the tests' tine capture stands; nullopt when off
    const auto capAt = []() -> std::optional<size_t> {
        return TINECAP ? std::optional<size_t>(TINECAP->size()) : std::nullopt;
    };
    const auto capsOf = [](std::optional<size_t> c0, std::optional<size_t> c1) -> std::optional<std::array<size_t, 2>> {
        if (!c0) return std::nullopt;
        return std::array<size_t, 2>{*c0, c1 ? *c1 : 0};
    };
    const Support support{&topo, rot, off};

    // The support unit is the locally-straight sub-patch, not the connected region.
    std::vector<PatchPtr> patches;
    for (int ri = 0; ri < int(result.regions.size()); ri++) {
        if (only && !only->has(ri)) continue;
        const std::vector<int> &rFaces = result.regions[size_t(ri)].faces;
        // Seat the WHOLE region's triangles once, shared by all its sub-patches.
        auto regionTrisP = std::make_shared<SurfTris>(Soup(rFaces.size() * 9));
        Soup &regionTris = regionTrisP->pos;
        std::vector<Vec3d> regionPts;
        double regionArea = 0;
        for (size_t k = 0; k < rFaces.size(); k++) {
            regionArea += topo.area[size_t(rFaces[k])];
            double gx = 0, gy = 0, gz = 0;
            for (int i = 0; i < 3; i++) {
                seat(pos, size_t(rFaces[k]) * 9 + size_t(i) * 3, rot, off, v);
                regionTris[k * 9 + size_t(i) * 3] = v[0];
                regionTris[k * 9 + size_t(i) * 3 + 1] = v[1];
                regionTris[k * 9 + size_t(i) * 3 + 2] = v[2];
                regionPts.push_back(Vec3d(v[0], v[1], v[2]));
                gx += v[0]; gy += v[1]; gz += v[2];
            }
            regionPts.push_back(Vec3d(gx / 3, gy / 3, gz / 3));
        }
        const std::shared_ptr<const SurfTris> regionTrisC = regionTrisP;

        // Raster pass: the whole footprint's tracks, no tube route, no split.
        if (raster) {
            if (!raster->has(ri)) continue;
            // A lattice net (issue #121): its struts stand in for the raster tracks.
            const Seated seated = [&pos, &rot, &off](int f) {
                return std::array<Vec3d, 3>{seat(pos, size_t(f) * 9, rot, off), seat(pos, size_t(f) * 9 + 3, rot, off),
                                            seat(pos, size_t(f) * 9 + 6, rot, off)};
            };
            const std::optional<Struts> struts = latticeStruts(topo, rFaces, seated);
            if (struts) {
                if (struts->lost) skipped.sliver++;   // faces no strut reached: dropped, so counted
                for (const PatchPtr &p : strutPatches(topo, rot, *struts, seated, step, ri, regionTrisC)) patches.push_back(p);
                continue;
            }
            Tracks lines = rasterTracks(regionPts, *regionTrisC, part, step, rowSpan, &support);
            // the sparse-coverage sag warning, per region: it only counts if this region swaps
            if (wantSparse && lines.spacing && *lines.spacing > PROP.maxUnsupportedSpan) tally.sagRegions.add(ri);
            if (!lines.empty()) {
                auto p = std::make_shared<PropPatch>();
                p->faces = rFaces;
                p->area = regionArea;
                p->region = ri;
                p->tris = regionTrisC;
                p->lines = std::vector<ContactLine>(lines.begin(), lines.end());
                patches.push_back(p);
            }
            continue;
        }

        // Curved region whose lowest line is straight = a tube: ONE wall under that line.
        // A SMALL tube keeps the region's split too, to race it (prop/rival.js).
        const std::optional<std::vector<ContactLine>> tube = tubeLine(topo, rFaces, rot, regionPts, *regionTrisC, step);
        const bool small = regionArea < PROP.tubeMinArea;
        std::optional<std::vector<PropPatch>> split;
        if (small || !(tube && !tube->empty())) split = splitRegion(topo, rFaces, rot);
        if (tube && !tube->empty()) {
            auto p = std::make_shared<PropPatch>();
            p->faces = rFaces;
            p->area = regionArea;
            p->region = ri;
            p->tris = regionTrisC;
            p->lines = *tube;
            if (small) {
                std::vector<PatchPtr> st;
                for (const PropPatch &q : *split) st.push_back(std::make_shared<PropPatch>(q));
                p->smallTube = std::move(st);
            }
            patches.push_back(p);
            continue;
        }

        for (const PropPatch &q : *split) {
            if (q.area < MIN_REGION_AREA) { skipped.sliver++; continue; }
            auto p = std::make_shared<PropPatch>(q);
            p->region = ri;
            p->tris = regionTrisC;
            patches.push_back(p);
        }
    }
    RegionSet servedRegions;

    // A small tube competes with its region's patches: see Prop/Rival.hpp.
    TubeRivalry<PropRecord> rivalry =
        tubeRivalry<PropRecord>(RivalAcc<PropRecord>{out, &tags, props, skipped, servedRegions, tally}, topo, result,
                                partTris);

    for (size_t pi = 0; pi < patches.size(); pi++) {
        const PatchPtr patch = patches[pi];   // by value: tubeDone appends to `patches`
        const SurfTris &regionTris = *patch->tris;
        const std::optional<RivalMark> mark = rivalry.begin(patch);   // a small tube undoes to here
        std::vector<ContactLine> lines;
        if (patch->lines) {
            // A tube's lowest-line track(s), fitted and resampled by tubeLine.
            lines = *patch->lines;
        } else {
            // The patch's own geometry: vertices plus face centroids for the frame fit,
            // and its triangles for asking "does the patch cover this station".
            std::vector<Vec3d> pts;
            SurfTris patchTris{Soup(patch->faces.size() * 9)};
            Soup &pt = patchTris.pos;
            for (size_t k = 0; k < patch->faces.size(); k++) {
                const int f = patch->faces[k];
                double gx = 0, gy = 0, gz = 0;
                for (int i = 0; i < 3; i++) {
                    seat(pos, size_t(f) * 9 + size_t(i) * 3, rot, off, v);
                    pts.push_back(Vec3d(v[0], v[1], v[2]));
                    pt[k * 9 + size_t(i) * 3] = v[0];
                    pt[k * 9 + size_t(i) * 3 + 1] = v[1];
                    pt[k * 9 + size_t(i) * 3 + 2] = v[2];
                    gx += v[0]; gy += v[1]; gz += v[2];
                }
                pts.push_back(Vec3d(gx / 3, gy / 3, gz / 3));
            }
            const Tracks tr = patchTracks(pts, patchTris, step, &support, rowSpan, patch->axis);
            // The user chose sub-cap spacing AND this face landed a multi-row gap wider
            // than the cap: flag it so the UI can warn (never blocks).
            if (wantSparse && !tr.empty() && tr.spacing && *tr.spacing > PROP.maxUnsupportedSpan) tally.sagRisk = true;
            lines.assign(tr.begin(), tr.end());
        }
        if (lines.empty()) {
            skipped.noLine++;
            // JS `continue` skips the tubeDone below
            continue;
        }

        // An index loop: a raster track's leftover pieces are appended mid-loop (rasterRest).
        for (size_t li = 0; li < lines.size(); li++) {
            ContactLine line = lines[li];
            // PART-ATTACHED first: a support stands on the floor under the overhang, not
            // stilted to the plate through the part. Works on a COPY of the line.
            const size_t tri0 = out.size();
            PartAttached pa = buildPartAttached(line, part, topo, rot, off, out, &tags);
            // (Plate only: undo it and take the plate path, as a bed overhang would)
            if (plateOnly && (pa.ok || pa.floored)) { truncate(out, tags, tri0); skipped.onPart++; pa = PartAttached{}; }
            if (pa.ok && patch->smallTube) {
                // A small tube's line is new to this path: hold it to the measured clearance.
                const Tris solid(out.begin() + std::ptrdiff_t(tri0), out.end());
                const std::optional<SolidHit> hit = solidClearance(topo, rot, off, solid, 0.25);
                if (hit && welds(*hit)) {
                    truncate(out, tags, tri0);
                    skipped.weld++;
                    continue;
                }
            }
            if (pa.ok) {
                servedRegions.add(patch->region);
                int nT = 0;
                const std::optional<size_t> c0 = capAt();
                TineGrip grip;
                // pa.prop.line already carries the wall top (surface minus gap); add the
                // gap back so emitTines reads it as the surface, like the plate path does.
                if (withTines) {
                    std::vector<Vec3d> topLine;
                    for (const Vec3d &p : pa.prop->line) topLine.push_back(Vec3d(p[0], p[1], p[2] + PROP.gap));
                    nT = emitTines(topLine, &part, topo, rot, off, out, tineStepEff, std::nullopt, tineHeight,
                                   std::nullopt, &grip, &tags);
                    tally.tines += nT;
                }
                // wall + tines are contiguous -> one segment.
                PropRecord q;
                q.span = pa.prop->span;
                q.height = pa.prop->height;
                q.stations = pa.prop->stations;
                q.volume = pa.prop->volume;
                q.partAttached = pa.prop->partAttached;
                q.inBore = pa.prop->inBore;
                q.line = pa.prop->line;
                q.area = patch->area;
                q.region = patch->region;
                q.tines = nT;
                q.caps = capsOf(c0, capAt());
                q.grip = grip.z;
                q.trimmed = int(line.size()) - pa.prop->stations;
                q.id = nextId++;
                q.kind = "prop";
                q.triRanges = {TriRange{tri0, out.size()}};
                props.push_back(std::move(q));
                continue;
            }
            if (pa.floored) { bump(skipped, *pa.floored); continue; }

            // Finish the top against the WHOLE region, not just this patch.
            contourTop(line, regionTris);
            lowerSag(line, regionTris);
            // pin the wall's low end to the squat floor, not the nearest 1mm station
            if (!insertFloorStations(line).empty()) contourTop(line, regionTris);

            // SQUAT BED PASS: the near-bed stations too low for the flanged wall, run
            // AFTER the tall wall so it skips exactly what that wall's tail covered.
            // squatLine is a deep copy taken now, before settleTop mutates the line.
            const ContactLine squatLine = line;
            Mask claimed(line.size(), false);
            const auto runSquat = [&]() {
                for (const SquatWall &sq : buildSquatBed(squatLine, regionTris, topo, rot, off, out, &claimed, &tags)) {
                    // a squat wall's base is the thin brim, so tines attach from squatBrimH up
                    const size_t t0 = out.size();
                    const std::optional<size_t> c0 = capAt();
                    TineGrip grip;
                    int nT = 0;
                    if (withTines) {
                        std::vector<Vec3d> sl;
                        for (const Vec3d &p : sq.line) sl.push_back(Vec3d(p[0], p[1], p[2] + PROP.gap));
                        nT = emitTines(sl, &regionTris, topo, rot, off, out, tineStepEff, PROP.squatBrimH, tineHeight,
                                       std::nullopt, &grip, &tags);
                    }
                    tally.tines += nT;
                    servedRegions.add(patch->region);
                    // the wall (sq.triRange) and its tines are NON-contiguous in `out`
                    std::vector<TriRange> segs{TriRange{sq.triRange[0], sq.triRange[1]}};
                    if (out.size() > t0) segs.push_back(TriRange{t0, out.size()});
                    PropRecord q;
                    q.span = sq.span;
                    q.height = sq.height;
                    q.stations = sq.stations;
                    q.volume = sq.volume;
                    q.squat = sq.squat;
                    q.line = sq.line;
                    q.triRange = TriRange{sq.triRange[0], sq.triRange[1]};
                    q.area = patch->area;
                    q.region = patch->region;
                    q.tines = nT;
                    q.caps = capsOf(c0, capAt());
                    q.grip = grip.z;
                    q.id = nextId++;
                    q.kind = "prop";
                    q.triRanges = std::move(segs);
                    props.push_back(std::move(q));
                }
            };

            // A track is straight in XY by construction: a tripwire for the frame fit.
            if (straightness(line) > PROP.maxWander) { skipped.wanders++; runSquat(); continue; }

            // Trim to the longest run that can actually carry a wall.
            Mask clear(line.size());
            for (size_t k = 0; k < line.size(); k++)
                clear[k] = line[k][2] - PROP.gap >= PROP.minHeightSquat && stationIsClear(line, k, topo, rot, off);
            Mask tall0(line.size());
            for (size_t k = 0; k < line.size(); k++) tall0[k] = clear[k] && line[k][2] - PROP.gap >= PROP.minHeight;
            const Mask usable = withLowTails(tall0, clear);
            const std::optional<Run> run = longestRun(usable);
            if (!run || (*run)[1] - (*run)[0] < PROP.minStations) { skipped.blocked++; runSquat(); continue; }
            if (raster && !patch->strut) {      // a strut takes one wall, as it would standing alone
                for (const RasterPiece &rest : rasterRest(line, *run)) {
                    lines.push_back(ContactLine(rest.begin(), rest.end()));
                    // the piece runs its own squat pass; this line's must not double it
                    for (int k = rest.from; k < rest.from + int(rest.size()); k++) claimed[size_t(k)] = true;
                }
            }
            ContactLine sub(line.begin() + (*run)[0], line.begin() + (*run)[1]);

            const Mask body = bodyMask(sub);    // before settleTop -- see bodyMask
            if (tallSpan(sub, body) < minSpanFor(sub, body, !!patch->smallTube, shortW)) { skipped.stub++; runSquat(); continue; }

            // Last, on the trimmed run only: put the closest approach exactly on spec.
            settleTop(sub, regionTris);

            // Re-trim against the settled line: on height, and on the measured clearance.
            Mask lowA(sub.size()), tallA(sub.size());
            for (size_t k = 0; k < sub.size(); k++)
                lowA[k] = sub[k][2] - PROP.gap >= PROP.minHeightSquat && stationCertified(sub, k, topo, rot, off);
            for (size_t k = 0; k < sub.size(); k++) tallA[k] = lowA[k] && sub[k][2] - PROP.gap >= PROP.minHeight;

            // Sweep, then MEASURE the finished solid against the whole part; a weld trims
            // the station that owns it and tries again, up to a few rounds.
            bool placed = false;
            std::optional<std::string> reason;
            for (int tries = 0; tries < 4 && !placed; tries++) {
                const std::optional<Run> run2 = longestRun(withLowTails(tallA, lowA));
                if (!run2 || (*run2)[1] - (*run2)[0] < PROP.minStations) { reason = "blocked"; break; }
                const ContactLine settled(sub.begin() + (*run2)[0], sub.begin() + (*run2)[1]);
                const double span2 = hypot2(settled[settled.size() - 1][0] - settled[0][0],
                                            settled[settled.size() - 1][1] - settled[0][1]);
                const Mask settledBody(body.begin() + (*run2)[0], body.begin() + (*run2)[1]);
                if (tallSpan(settled, settledBody) < minSpanFor(settled, settledBody, !!patch->smallTube, shortW)) {
                    reason = "stub";
                    break;
                }

                const size_t before = out.size();
                if (!sweep(settled, zBed, out, PROP.minHeightSquat, &tags)) {
                    truncate(out, tags, before);
                    reason = "degenerate";
                    break;
                }

                // 0.25 reach: the tightest threshold is 0.205.
                const Tris solid(out.begin() + std::ptrdiff_t(before), out.end());
                const std::optional<SolidHit> hit = solidClearance(topo, rot, off, solid, 0.25);
                if (hit && welds(*hit)) {
                    truncate(out, tags, before);
                    size_t kBest = 0;
                    double dBest = INF;
                    for (size_t k = 0; k < settled.size(); k++) {
                        const double dx = settled[k][0] - hit->x, dy = settled[k][1] - hit->y;
                        if (dx * dx + dy * dy < dBest) { dBest = dx * dx + dy * dy; kBest = k; }
                    }
                    const long at = long((*run2)[0]) + long(kBest);
                    for (const long k : {std::max(0L, at - 1), at, std::min(long(lowA.size()) - 1, at + 1)}) {
                        lowA[size_t(k)] = false;
                        tallA[size_t(k)] = false;
                    }
                    reason = "weld";
                    continue;
                }

                // The TALLEST point: what the wall costs to print.
                double maxZ0 = -INF;
                for (const Vec3d &p : settled) maxZ0 = jsMax(maxZ0, p[2]);
                const double top = maxZ0 - PROP.gap;
                // signed volume of the emitted solid (divergence theorem)
                double vol = 0;
                for (size_t i = before; i < out.size(); i += 3) {
                    const Vec3d &a = out[i], &b = out[i + 1], &c = out[i + 2];
                    vol += (a[0] * (b[1] * c[2] - b[2] * c[1])
                          + a[1] * (b[2] * c[0] - b[0] * c[2])
                          + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
                }
                servedRegions.add(patch->region);
                // The grip comb along this wall's settled top.
                const std::optional<size_t> c0 = capAt();
                TineGrip grip;
                int nT = 0;
                if (withTines) {
                    const std::optional<Run> tb = tallBody(settledBody);
                    nT = emitTines(settled, &regionTris, topo, rot, off, out, tineStepEff, std::nullopt, tineHeight,
                                   tb ? std::optional<TineBody>(TineBody{(*tb)[0], (*tb)[1]}) : std::nullopt, &grip, &tags);
                }
                tally.tines += nT;
                PropRecord q;
                q.region = patch->region;
                q.tines = nT;
                q.caps = capsOf(c0, capAt());
                q.grip = grip.z;
                q.span = span2;
                q.height = top - zBed;
                q.area = patch->area;
                q.stations = int(settled.size());
                q.trimmed = int(line.size()) - int(settled.size());
                q.volume = std::abs(vol);
                // the centreline, so a coverage check can ask what this wall reaches
                for (const Vec3d &p : settled) q.line.push_back(Vec3d(p[0], p[1], p[2] - PROP.gap));
                q.id = nextId++;
                q.kind = "prop";
                // `before` marks where this wall's triangles begin; tines follow it.
                q.triRanges = {TriRange{before, out.size()}};
                props.push_back(std::move(q));
                for (int k = (*run)[0] + (*run2)[0]; k < (*run)[0] + (*run2)[1]; k++) claimed[size_t(k)] = true;
                placed = true;
            }
            if (!placed && reason) bump(skipped, *reason);
            runSquat();
        }
        if (patch->smallTube) rivalry.tubeDone(patch, *mark, patches);
    }
    rivalry.finish();

    // `served` counts REGIONS with at least one wall.
    res.served = int(servedRegions.size());
    res.servedRegions = servedRegions.order;
    res.tines = tally.tines;
    res.sagRisk = tally.sagRisk;
    if (raster) res.sagRegions = tally.sagRegions;
    double volume = 0;
    for (const PropRecord &q : props) volume = volume + q.volume;
    res.volume = volume;
    if (tags.size() < out.size()) tags.resize(out.size(), 0);   // untagged tail reads 0 (Crest.hpp)
    return res;
}
} // namespace

PropsResult noProps() { return PropsResult{}; }

double coverRowSpan(double coverage) {
    const double c = jsMax(0, jsMin(1, coverage));
    const double cap = PROP.maxUnsupportedSpan;
    return c <= 0.5
        ? cap + ((0.5 - c) / 0.5) * (COVER_SPARSE_SPAN - cap)   // 30 .. 12
        : cap - ((c - 0.5) / 0.5) * (cap / 2);                  // 12 .. 6
}

PropsResult buildProps(const Topology &topo, const AnalyzeResult &result, const Rot &rot, const PropOpts &opts) {
    const size_t capFrom = TINECAP ? TINECAP->size() : 0;   // tests' tine capture: see withRaster
    PropsResult normal = buildPass(topo, result, rot, opts, nullptr);
    if (!opts.raster) return normal;
    return withRaster(topo, result, rot, opts.rasterVeto, normal, capFrom,
                      [&](const RegionSet &regions) { return buildPass(topo, result, rot, opts, &regions); });
}

PropsResult buildShortWalls(const Topology &topo, const AnalyzeResult &result, const Rot &rot, const PropOpts &opts,
                            const RegionSet &regions) {
    PropOpts o = opts;
    o.onlyRegions = regions;
    o.shortWalls = true;
    return buildPass(topo, result, rot, o, nullptr);
}

}} // namespace
