// Port of plugins/shared/engine/fins_entry.js + seat.js -- the one function a host
// calls: a posed part as triangle soup in (mm, z up, anywhere on the plate), the fin +
// bed-pad triangles out, in the SEATED frame, with the offset to map them back.
#pragma once
#include "Fins.hpp"
#include <string>

namespace Slic3r { namespace SupportFins {

/// fins_entry.js ENGINE_DEFAULTS merged with a host's options.
struct EntryOptions {
    std::string mode = "auto";
    bool        bedPad = true;
    bool        tines = true;
    double      tineDensity = 0;
    double      coverage = 0.5;
    bool        plateOnly = false;
    double      layerHeight = 0.2;
    double      threshold = DEFAULT_THRESHOLD;
    std::string material = "pla";
    std::string padStyle = "auto";   // off | auto | light | sure
    std::string cutout = "none";
    std::optional<SwayOpts> sway;    // gap defaults to the material's propGap
};

struct PieceRange {
    std::string id, kind;
    std::vector<std::pair<int, int>> ranges;   // [first, end) TRIANGLE indices into EntryResult::triangles
};

struct EntryStats {
    int overhangRegions = 0;
    int finTriangles = 0, swayTriangles = 0, padTriangles = 0;
    int braces = 0, props = 0, tines = 0;
    int swayBraces = 0, swayTines = 0, swaySkipped = 0;
    std::optional<std::string> swayReason;
    int unserved = 0;
    int floating = 0;
    double floatingDrop = 0;
};

struct EntryResult {
    std::vector<float>      triangles;   // 9 per face: fins | sway | pad, SEATED frame (float32 like upstream)
    Offset                  offset;      // seated = input + offset
    std::vector<PieceRange> pieces;
    std::vector<int>        overFaces, smallFaces;
    EntryStats              stats;
};

/// seat.js seatSoup(): centre the part's XY bbox at the origin, lowest point at z = 0,
/// snap to a 1 nm grid. Returns the shift (pos = input - shift).
Offset seatSoup(const Soup &input, Soup &pos);

/// fins_entry.js computeFins(positions, options). Throws std::invalid_argument on a bad option.
EntryResult computeFins(const Soup &positions, const EntryOptions &options);

/// draw_entry.js drawWall's result when it builds (ok): one drawn wall, SEATED frame.
struct EntryDrawResult {
    bool               ok = false;
    std::vector<float> triangles;        // 9 per face, float32 like upstream (seat.js flatten)
    Offset             offset;           // seated = input + offset
    struct {
        double length = 0, height = 0;
        int    tines = 0;
        bool   partAttached = false;     // it stands on the part below, not the plate
    } stats;
};

/// draw_entry.js drawWall(positions, a, b, options): one wall along the line the user
/// drew from `a` to `b` (same frame as `positions`). Reads material, tines, tineDensity,
/// layerHeight, plateOnly, threshold, padStyle (validated) and cutout from `options`.
/// On a refusal returns ok = false and sets `reason` (the site's words, UTF-8).
/// Forwards to Draw.hpp's drawWall. Throws std::invalid_argument like computeFins.
EntryDrawResult drawWall(const Soup &positions, const Vec3d &a, const Vec3d &b, const EntryOptions &options,
                         std::string &reason);

}} // namespace
