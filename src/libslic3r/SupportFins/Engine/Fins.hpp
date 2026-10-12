// Port of web/fins.js -- the combined-support entry point: buildFins(). The pieces
// live in Engine/Fins/ (config, seating, pad, wedges, shortwalls, coverage, fill),
// exactly as upstream's web/fins/.
//
// The structs below are the shape fins_entry.js reads off buildFins' result. The
// owner of the fins.js port may add fields; the ones here are read by Entry.cpp and
// the parity runner and must keep their meaning.
#pragma once
#include "Math.hpp"
#include "Overhangs.hpp"
#include "Pieces.hpp"
#include "PropConfig.hpp"
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// fins.js applyTunables(t): the clearances travelling with a build request.
struct Tunables {
    double      padH = 0.5;
    double      padGrab = 0.05;
    double      propGap = 0.2;
    std::string padStyle = "auto";          // auto | light | sure | custom
    struct { double h = 0.5, gap = 0.0, grip = 0.05, margin = 4.0; } padCustom;
    std::string cutout = "none";            // none | diamond | triangle | arch | lattice
    Iface       iface = Iface::Off;
    int         tinesPerWall = 0;
};

/// opts.sway (web/sway.js options), forwarded whole; ignored unless `on`.
struct SwayOpts {
    bool   on = false;
    double gripFrom = 0;
    double tineSpacing = 6;
    double reach = 0.15;
    double gap = 0.2;        // the material's propGap unless the host sets its own
};

/// buildFins(topo, result, rot, opts) options.
struct BuildOpts {
    std::string mode = "auto";        // auto | prop | full | draw | stabilize
    bool        bedPad = true;
    bool        tines = true;
    double      tineDensity = 0;      // 0..1
    double      layerHeight = 0.2;
    double      coverage = 0.5;       // 0..1
    bool        plateOnly = false;
    bool        lastResort = true;    // tests' baseline switch
    std::optional<SwayOpts> sway;
    Tunables    tunables;
};

/// A range [first, end) of ELEMENTS of Built::triangles (vertex triples here, so a
/// triangle range is divided by 3 to get triangle indices; fins_entry.js piecesOf).
using TriRange = std::pair<int, int>;

/// One fin's record (fins.js `fins[]`): what the UI addresses, removes and reports.
struct FinRecord {
    int         id = 0;
    std::string kind;        // prop | wedge | sway
    double      height = 0, length = 0, span = 0;
    int         tines = 0;
    std::vector<TriRange> triRanges;
    std::vector<Vec3d>    line;   // the wall's centreline, seated frame (may be empty)
    bool        fill = false, joined = false, isShort = false;
};

struct SwayResult {
    int count = 0, tines = 0, skipped = 0;
    std::optional<std::string> reason;
};

/// buildFins' result, the subset Entry.cpp and the tests consume.
struct Built {
    Tris        triangles;        // fins | sway braces, vertex triples (seated frame)
    Tris        padTriangles;     // the bed pad, vertex triples
    bool        pad = false;      // a pad was built
    std::string mode;
    std::vector<FinRecord> fins;
    int         braceCount = 0;   // tined walls/wedges
    int         propCount = 0;    // plain walls
    int         tines = 0;
    int         unserved = 0;
    std::optional<SwayResult> sway;
    std::vector<FloatingPiece> floating;
    std::string seatingKind;      // face | edge | point
    bool        sagRisk = false;
};

/// fins.js applyTunables(t): write the build request's clearances into FIN/PROP/PAD/PERP/CUT.
void applyTunables(const Tunables &t);

/// fins.js buildFins(topo, result, rot, opts).
Built buildFins(Topology &topo, const AnalyzeResult &result, const Rot &rot, const BuildOpts &opts);

}} // namespace
