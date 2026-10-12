// Port of web/prop/rival.js -- a small tube against the patch path, for one region.
//
// A SMALL tube (under tubeMinArea, see tubeLine) is built first, taken back out, the
// region's split patches are built from the same starting point, and whichever holds
// more overhang area stays. A tie keeps the patches. buildProps drives it from its
// patch loop:
//
//   auto mark = rivalry.begin(patch);           // top of every patch
//   ...build the patch's walls...
//   if (patch->smallTube) rivalry.tubeDone(patch, *mark, patches);
//   rivalry.finish();                           // after the loop
//
// `acc` is everything the loop accumulates -- { out, props, skipped, served, tally }
// -- so a route can be snapshotted and undone. The prop record type is buildProps'
// (Prop.hpp, not ported yet), so TubeRivalry is a template over it: PropT must be
// copyable and have `triRanges`, a container of [first, end) pairs (TriRange).
//
// JS patches are objects and the loop is `for (const patch of patches)` over an array
// tubeDone appends to; in C++ iterate by index over std::vector<PatchPtr> and hold the
// PatchPtr by value (a push_back may reallocate).
#pragma once
#include "../Math.hpp"
#include "../Overhangs.hpp"
#include "Crest.hpp"
#include "Surface.hpp"
#include "Tracks.hpp"
#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <tuple>
#include <type_traits>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// buildProps' `skipped` counters (prop.js noProps()). `for (const k in skipped)`
/// visits all nine.
struct Skipped {
    int noLine = 0, wanders = 0, stub = 0, blocked = 0, degenerate = 0, buried = 0, weld = 0, sliver = 0, onPart = 0;
};

/// A JS Set of region indices: insertion-ordered (prop.js returns `[...servedRegions]`).
struct RegionSet {
    std::vector<int> order;
    bool   has(int r) const;
    void   add(int r);
    void   clear() { order.clear(); }
    size_t size() const { return order.size(); }
};

/// buildProps' `tally`: { tines, sagRisk, sagRegions }. rival.js touches tines and sagRisk.
struct Tally {
    int       tines = 0;
    bool      sagRisk = false;
    RegionSet sagRegions;
};

/// rival.js snapshot(): { out, props, tines, skipped, sagRisk, served }. `out` and
/// `props` are lengths.
struct RivalMark {
    size_t out = 0, props = 0;
    int    tines = 0;
    Skipped skipped;
    bool   sagRisk = false;
    std::vector<int> served;     // the Set's members, in order
};

/// The tube's built walls, taken back out: JS `{ mark, end, tris, props, held }`.
/// `tags` rides with `tris` (crest interface tags, Crest.hpp); the props live in the
/// TubeRivalry (they are of its template type), `props` indexes them.
struct RivalTube {
    RivalMark mark, end;
    Tris      tris;
    IfaceTags tags;
    size_t    props = 0;
    double    held = 0;
};

/// rival.js's per-region record: JS `{ last, tube, faces, tris, reach, slivers,
/// region, start }`. `last` is compared by identity only (JS `p !== r.last`), so it
/// is a raw pointer to the PropPatch the patches vector keeps alive; nullptr is JS
/// undefined (no rival patch big enough).
struct Rival {
    const PropPatch *last = nullptr;
    RivalTube        tube;
    std::vector<int> faces;
    std::shared_ptr<const SurfTris> tris, reach;
    int  slivers = 0;
    int  region = -1;
    std::optional<RivalMark> start;
};

/// rival.js heldArea and its lazily built overFaces cache: overhang area held by
/// support vertices tris[from..], judged over every overhang face within 2 x
/// maxUnsupportedSpan of `own`'s (flat xyz) bbox.
class RivalHeld {
public:
    RivalHeld(const Topology &topo, const AnalyzeResult &result, const Soup &partTris)
        : topo_(topo), result_(result), partTris_(partTris) {}
    double operator()(const Soup &own, const Tris &tris, size_t from);
private:
    const Topology      &topo_;
    const AnalyzeResult &result_;
    const Soup          &partTris_;
    bool overBuilt_ = false;
    std::vector<std::array<double, 4>> overFaces_;   // [cx, cy, cz, area]
};

/// JS `acc`, by reference. tags nullptr: `out`'s interface tags are not tracked.
template <class PropT>
struct RivalAcc {
    Tris               &out;
    IfaceTags          *tags;
    std::vector<PropT> &props;
    Skipped            &skipped;
    RegionSet          &served;
    Tally              &tally;
};

/// Skipped arithmetic for keepTube: a += b - c, field by field.
void addSkippedDelta(Skipped &a, const Skipped &b, const Skipped &c);

/// tubeRivalry(acc, topo, result, partTris)'s returned object.
template <class PropT>
class TubeRivalry {
public:
    TubeRivalry(RivalAcc<PropT> acc, const Topology &topo, const AnalyzeResult &result, const Soup &partTris)
        : acc_(acc), held_(topo, result, partTris) {}

    /// Top of every patch: settle the previous one; the mark a small tube undoes to
    /// (nullopt = JS null, for a patch with no smallTube).
    std::optional<RivalMark> begin(const PatchPtr &patch) {
        settle(lastPatch_);
        lastPatch_ = patch;
        if (patch->rival && !patch->rival->start) patch->rival->start = snapshot();
        return patch->smallTube ? std::optional<RivalMark>(snapshot()) : std::nullopt;
    }

    /// A small tube's walls are built. Take them back out and queue the region's
    /// patches on `patches` to have a go; with none big enough, the tube keeps it.
    void tubeDone(const PatchPtr &patch, const RivalMark &mark, std::vector<PatchPtr> &patches) {
        auto rival = std::make_shared<Rival>();
        RivalTube &tube = rival->tube;
        tube.mark = mark;
        tube.end = snapshot();
        tube.tris.assign(acc_.out.begin() + std::ptrdiff_t(mark.out), acc_.out.end());
        if (acc_.tags)
            for (size_t i = mark.out; i < acc_.out.size(); i++) tube.tags.push_back(i < acc_.tags->size() ? (*acc_.tags)[i] : 0);
        tube.props = tubeProps_.size();
        tubeProps_.emplace_back(acc_.props.begin() + std::ptrdiff_t(mark.props), acc_.props.end());
        tube.held = held_(ownOf(patch->reach, patch->tris), acc_.out, mark.out);
        restore(mark);
        std::vector<PatchPtr> rivals;
        for (const PatchPtr &p : *patch->smallTube)
            if (p->area >= MIN_REGION_AREA) rivals.push_back(p);
        // slivers count only if the patch path keeps the region, as they did before
        rival->last = rivals.empty() ? nullptr : rivals.back().get();
        rival->faces = patch->faces;
        rival->tris = patch->tris;
        rival->reach = patch->reach;
        rival->slivers = int(patch->smallTube->size() - rivals.size());
        rival->region = patch->region;
        if (rivals.empty()) keepTube(*rival);
        for (const PatchPtr &p : rivals) {
            p->region = patch->region;
            p->tris = patch->tris;
            p->rival = rival;
            patches.push_back(p);            // for...of reaches patches pushed mid-loop
        }
    }

    /// After the patch loop: settle the last region.
    void finish() { settle(lastPatch_); }

private:
    static const Soup &ownOf(const std::shared_ptr<const SurfTris> &reach, const std::shared_ptr<const SurfTris> &tris) {
        return (reach ? reach : tris)->pos;   // JS r.reach ?? r.tris
    }

    RivalMark snapshot() const {
        RivalMark m;
        m.out = acc_.out.size();
        m.props = acc_.props.size();
        m.tines = acc_.tally.tines;
        m.skipped = acc_.skipped;
        m.sagRisk = acc_.tally.sagRisk;
        m.served = acc_.served.order;
        return m;
    }
    void restore(const RivalMark &m) {
        acc_.out.resize(m.out);
        if (acc_.tags && acc_.tags->size() > m.out) acc_.tags->resize(m.out);
        acc_.props.erase(acc_.props.begin() + std::ptrdiff_t(m.props), acc_.props.end());
        acc_.tally.tines = m.tines;
        acc_.tally.sagRisk = m.sagRisk;
        acc_.skipped = m.skipped;
        acc_.served.clear();
        for (int r : m.served) acc_.served.add(r);
    }
    // Undo the region's patches and put its tube's walls back instead, moved to the
    // end of `out` (so every triRange shifts by the same amount).
    void keepTube(Rival &r) {
        const RivalTube &t = r.tube;
        if (r.start) restore(*r.start);
        const long shift = long(acc_.out.size()) - long(t.mark.out);
        if (acc_.tags && !t.tags.empty()) {
            acc_.tags->resize(acc_.out.size(), 0);
            acc_.tags->insert(acc_.tags->end(), t.tags.begin(), t.tags.end());
        }
        acc_.out.insert(acc_.out.end(), t.tris.begin(), t.tris.end());
        for (const PropT &q : tubeProps_[t.props]) {
            PropT q2 = q;
            for (auto &tr : q2.triRanges) {
                using A = std::remove_reference_t<decltype(std::get<0>(tr))>;
                using B = std::remove_reference_t<decltype(std::get<1>(tr))>;
                std::get<0>(tr) = A(long(std::get<0>(tr)) + shift);
                std::get<1>(tr) = B(long(std::get<1>(tr)) + shift);
            }
            acc_.props.push_back(std::move(q2));
        }
        acc_.tally.tines += t.end.tines - t.mark.tines;
        addSkippedDelta(acc_.skipped, t.end.skipped, t.mark.skipped);
        acc_.tally.sagRisk = acc_.tally.sagRisk || t.end.sagRisk;
        if (!tubeProps_[t.props].empty()) acc_.served.add(r.region);
        else acc_.skipped.sliver += r.slivers;   // nothing either way: dropped, as before
    }
    // A small tube's region is settled once its last patch is done, however that patch
    // exits; its patches' output is then the tail of `out`.
    void settle(const PatchPtr &p) {
        if (!p) return;
        const std::shared_ptr<Rival> r = p->rival;
        if (!r || p.get() != r->last) return;
        if (r->tube.held > held_(ownOf(r->reach, r->tris), acc_.out, r->start->out) + 1e-6) keepTube(*r);
        else acc_.skipped.sliver += r->slivers;
    }

    RivalAcc<PropT> acc_;
    RivalHeld       held_;
    PatchPtr        lastPatch_;
    std::vector<std::vector<PropT>> tubeProps_;   // RivalTube::props indexes this
};

/// tubeRivalry(acc, topo, result, partTris). `partTris` is the whole part seated
/// (flat, 9 per face, topology face order), as buildProps builds it.
template <class PropT>
TubeRivalry<PropT> tubeRivalry(RivalAcc<PropT> acc, const Topology &topo, const AnalyzeResult &result,
                               const Soup &partTris) {
    return TubeRivalry<PropT>(acc, topo, result, partTris);
}

}} // namespace
