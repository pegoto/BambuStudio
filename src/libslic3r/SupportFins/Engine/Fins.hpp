// Port of web/fins.js -- the combined-support entry point: buildFins(). The pieces
// live in Engine/Fins/ (config, seating, pad, wedges, shortwalls, coverage, fill),
// exactly as upstream's web/fins/.
//
// The structs below are the shape fins_entry.js reads off buildFins' result, plus what
// else upstream's result carries (props, servedRegions, seating, the pad record, the
// fill pass's stats, the sway brace outlines, skipped, volume). Entry.cpp and the
// parity runner read triangles / padTriangles / fins / braceCount / propCount / tines /
// unserved / sway / floating; those keep their meaning.
#pragma once
#include "Math.hpp"
#include "Overhangs.hpp"
#include "Pieces.hpp"
#include "Prop.hpp"
#include "PropConfig.hpp"
#include "Fins/Pad.hpp"
#include "Fins/Seating.hpp"
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// fins.js applyTunables(t): the clearances travelling with a build request. Every
/// field is optional upstream (`set` skips a non-finite number, an unknown style or
/// pattern is ignored): a NaN number, an empty/unknown string or a nullopt here is
/// "absent" and leaves the module state alone.
struct Tunables {
    double      padH = 0.5;                 // FIN.padH
    double      padGrab = 0.05;             // PAD.grab
    double      propGap = 0.2;              // PROP.gap and PERP.gap
    std::string padStyle = "auto";          // auto | light | sure | custom
    /// PAD.custom: applied only when present (JS `if (t.padCustom)`), then each key only
    /// if finite.
    struct PadCustom { double h = 0.5, gap = 0.0, grip = 0.05, margin = 4.0; };
    std::optional<PadCustom> padCustom;
    std::string cutout = "none";            // none | diamond | triangle | arch | lattice
    /// PROP.iface: only when present (JS: only for false / 'flat' / 'all'; false = Off).
    std::optional<Iface> iface;
    /// PROP.tinesPerWall: only when present and >= 0 (JS Number.isInteger && >= 0).
    std::optional<int> tinesPerWall;
};

/// opts.sway (web/sway.js options), forwarded whole; ignored unless `on`. A non-finite
/// number reads as missing in sway.js (its own default). `gap` NaN = the host set none:
/// computeFins then gives the braces the material's propGap (fins_entry.js
/// `{ gap: mat.propGap, ...sway }`), as the site does.
struct SwayOpts {
    bool   on = false;
    double gripFrom = 0;
    double tineSpacing = 6;
    double reach = 0.15;
    double gap = std::numeric_limits<double>::quiet_NaN();
};

/// buildFins(topo, result, rot, opts) options.
struct BuildOpts {
    std::string mode = "auto";        // auto | prop | full | draw | stabilize (anything else: prop)
    bool        bedPad = true;
    bool        tines = true;
    double      tineDensity = 0;      // 0..1
    double      layerHeight = 0.2;
    double      coverage = 0.5;       // 0..1
    bool        plateOnly = false;
    bool        lastResort = true;    // tests' baseline switch (JS `lastResort: false` skips it)
    bool        raster = true;        // JS `raster: false` runs buildProps' normal pass alone
    std::optional<SwayOpts> sway;
    std::optional<Tunables> tunables; // nullopt: JS no opts.tunables (applyTunables does nothing)
};

/// One fin's record (fins.js `fins[]`): what the UI addresses, removes and reports.
/// (Upstream also carries rows/stilt/lean/bearing = 0 and site = null on every record.)
struct FinRecord {
    int         id = 0;
    std::string kind;        // prop | wedge | sway
    double      height = 0, length = 0, span = 0;
    int         tines = 0;
    std::vector<TriRange> triRanges;
    std::vector<Vec3d>    line;   // the wall's centreline, seated frame (may be empty)
    bool        fill = false, joined = false, isShort = false;   // JS fill / joined / short
};

/// One prop of buildFins' `props`: buildProps' record plus what fins.js, fins/shortwalls.js
/// and fins/fill.js add on their own copies. A fill wall carries only line / triRanges /
/// height / span / tines / partAttached / squat (+ braces / ties): its other PropRecord
/// fields stay at their defaults (JS undefined).
struct BuiltProp : PropRecord {
    bool isShort = false;             // JS short: true (the last-resort pass)
    bool fill = false;                // JS fill: true (the Full-coverage pass)
    bool joined = false;              // JS joined: true (fill: met another fill wall)
    std::optional<int> braces;        // fill: ribs braceFill added
    std::optional<int> ties;          // fill: ties tieFill added (wall A of each)
    BuiltProp() = default;
    explicit BuiltProp(const PropRecord &q) : PropRecord(q) {}
};

/// One sway brace outline (fins.js `sway.braces[]`: { foot, halfW, th, height, levels }).
struct SwayOutline {
    struct Level { double z = 0; Vec3d a = Vec3d::Zero(), b = Vec3d::Zero(); };
    std::vector<Vec3d> foot;
    double halfW = 0, th = 0, height = 0;
    std::vector<Level> levels;
};

struct SwayResult {
    int count = 0, tines = 0, skipped = 0;
    std::optional<std::string> reason;
    std::vector<SwayOutline> braces;
};

/// fins.js withFill's `fill` record.
struct FillRecord {
    int    walls = 0, joined = 0;
    std::optional<int> ribs, ties;    // JS undefined when the fill had no leftover red
    int    tries = 0, checks = 0;
    bool   capped = false;
    std::vector<std::pair<std::string, int>> refused;   // insertion order (a JS object)
    double bareBefore = 0, unservedArea = 0;
    std::vector<Vec3d> unservedPts;
};

/// buildFins' result.
struct Built {
    Tris        triangles;        // fins | sway braces, vertex triples (seated frame)
    IfaceTags   tags;             // interface tags parallel to `triangles` (JS v[3] === 1)
    Tris        padTriangles;     // the bed pad, vertex triples
    bool        pad = false;      // a pad was built
    std::optional<PadResult> padRecord;   // JS `pad` (null: none)
    std::string mode;
    std::vector<FinRecord> fins;
    std::vector<BuiltProp> props;
    double      volume = 0;
    Skipped     skipped;
    std::vector<int> servedRegions;
    int         braceCount = 0;   // tined walls/wedges
    int         propCount = 0;    // plain walls
    int         tines = 0;
    int         unserved = 0;
    std::optional<SwayResult> sway;
    std::optional<FillRecord> fill;
    std::vector<FloatingPiece> floating;
    std::string seatingKind;      // face | edge | point (= seating.kind)
    Seating     seating;
    bool        sagRisk = false;
};

/// fins.js applyTunables(t): write the build request's clearances into FIN/PROP/PAD/PERP/CUT.
void applyTunables(const Tunables &t);

/// fins.js coverPitch(coverage): the wedge row pitch for a coverage setting.
double coverPitch(double coverage);

/// fins.js buildFins(topo, result, rot, opts).
Built buildFins(Topology &topo, const AnalyzeResult &result, const Rot &rot, const BuildOpts &opts);

}} // namespace
