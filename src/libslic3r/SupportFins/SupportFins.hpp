///|/ Support Fins -- native breakaway support fins for BambuStudio.
///|/
///|/ Port of the printfins.com engine (https://github.com/gittrahan/support-fins, MIT,
///|/ (c) Matthew Trahan; technique by Slant3D). Engine/ is the line-for-line port of
///|/ the upstream JS; this header is the libslic3r-facing API the GUI and the model
///|/ code use. Nothing in Engine/ knows about Slic3r types.
///|/
///|/ A fin set is generated for an object as it sits on the plate (instance 0's
///|/ transform applied) and stored as ordinary MODEL_PART ModelVolumes tagged with a
///|/ SupportFinInfo, so Prepare, slicing, export and the project file all see plain
///|/ geometry. The options that produced them live on the ModelObject
///|/ (ModelObject::support_fin_options) so they can be regenerated after a rotation.
#pragma once

#include "libslic3r/Point.hpp"
#include "admesh/stl.h"

#include <cereal/types/optional.hpp>
#include <cereal/types/string.hpp>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r {

class ModelObject;
class ModelVolume;

/// The user-facing settings, 1:1 with upstream plugins/shared/engine/options.json
/// (key names kept, snake_cased). layer_height is host-supplied: the print preset's.
struct SupportFinOptions
{
    std::string material         = "pla";     // pla | petg
    double      threshold        = 45.0;      // deg from the plate, 30..70
    bool        tines            = true;
    double      tine_density     = 0.0;       // 0..1 (the site's slider / 100)
    double      layer_height     = 0.2;       // mm, from the print preset
    std::string pad_style        = "auto";    // off | auto | light | sure
    bool        sway             = false;
    double      sway_grip_from   = 0.0;       // mm up, 0..2000
    double      sway_tine_spacing = 6.0;      // mm, 2..30
    double      sway_reach       = 0.15;      // 0.05..0.5 (percent / 100)
    std::string cutout           = "none";    // none | diamond | triangle | arch | lattice
    double      coverage         = 0.5;       // 0..1
    bool        plate_only       = false;

    bool operator==(const SupportFinOptions &o) const;
    bool operator!=(const SupportFinOptions &o) const { return !(*this == o); }

    /// Clamp/normalise to the ranges in options.json; returns false if a choice was invalid.
    bool validate();

    /// key=value;key=value serialisation for the 3MF model config (single line, no newlines).
    std::string to_string() const;
    static std::optional<SupportFinOptions> from_string(const std::string &s);

    template<class Archive> void serialize(Archive &ar) {
        ar(material, threshold, tines, tine_density, layer_height, pad_style, sway, sway_grip_from,
           sway_tine_spacing, sway_reach, cutout, coverage, plate_only);
    }
};

/// Tag on every generated ModelVolume. kind is upstream's piece kind.
struct SupportFinInfo
{
    std::string kind;            // prop | wedge | sway | pad | drawn
    int         piece_id = 0;    // upstream's per-build id (stable for the same part+options)
    /// Hash of (posed part soup, options) at generation: a regenerate check compares it
    /// to the current one to flag stale fins after a rotation/scale/edit.
    std::string source_hash;
    /// Draw mode: the two clicked points (seated frame) so a drawn wall can be rebuilt.
    bool  drawn = false;
    Vec3d draw_a = Vec3d::Zero();
    Vec3d draw_b = Vec3d::Zero();

    std::string to_string() const;
    static std::optional<SupportFinInfo> from_string(const std::string &s);

    template<class Archive> void serialize(Archive &ar) {
        ar(kind, piece_id, source_hash, drawn, draw_a, draw_b);
    }
};

struct SupportFinStats
{
    int overhang_regions = 0;
    int walls = 0;        // tined walls + plain walls (upstream braces + props)
    int tines = 0;
    int sway_braces = 0, sway_tines = 0, sway_skipped = 0;
    std::optional<std::string> sway_reason;
    int unserved = 0;     // overhangs too shallow for a fin this way up
    int floating = 0;     // pieces of the part that start in mid-air
    double floating_drop = 0;
    int fin_triangles = 0, sway_triangles = 0, pad_triangles = 0;
};

struct SupportFinPiece
{
    std::string          kind;   // prop | wedge | sway | pad
    int                  id = 0;
    indexed_triangle_set mesh;   // in the caller's (posed, world mm) frame
};

struct SupportFinResult
{
    std::vector<SupportFinPiece> pieces;
    std::vector<int>             over_faces;    // input faces in supported overhang regions (paint red)
    std::vector<int>             small_faces;   // input faces past the threshold but too small to fin (amber)
    SupportFinStats              stats;
};

/// Run the engine on a posed part (world mm, Z up, anywhere on the plate; model-part
/// volumes only, already transformed by the instance and volume matrices, outward
/// winding). Pieces come back in the same frame. Throws std::invalid_argument on a bad option.
SupportFinResult generate_support_fins(const indexed_triangle_set &posed_part, const SupportFinOptions &options);

/// Draw mode: one hand-drawn wall from a to b (world mm, where the two clicks hit the
/// part). Returns the wall, or nullopt with `reason` set to upstream's user-facing words.
std::optional<SupportFinPiece> draw_support_fin_wall(const indexed_triangle_set &posed_part, const Vec3d &a, const Vec3d &b,
                                                     const SupportFinOptions &options, std::string &reason);

/// The one-line result every host shows (upstream host_report / engine/report.js).
std::string support_fins_report(const SupportFinStats &stats);

/// The posed model-part soup of an object as generate_support_fins wants it: instance
/// `instance_idx`'s transform x each model-part volume's matrix, fin volumes excluded,
/// winding restored on a mirrored transform. Empty when the object has no model parts.
indexed_triangle_set support_fins_posed_part(const ModelObject &object, int instance_idx = 0);

/// Hash of (posed part, options). SupportFinInfo::source_hash is this hash of the posed part
/// built WITHOUT the instance's translation (see object_source_hash in SupportFins.cpp), so a
/// world-frame support_fins_posed_part() hash is not comparable with a stored source_hash.
std::string support_fins_source_hash(const indexed_triangle_set &posed_part, const SupportFinOptions &options);

/// Generate (or regenerate) the fin set of an object in place: removes the object's
/// existing non-drawn fin volumes, runs the engine on the posed part, and adds one
/// MODEL_PART volume per piece, each placed so it lands exactly where the engine put it
/// relative to the part (volume matrix = instance transform inverse). Drawn walls are
/// kept and re-tagged with the new hash. Stores `options` on the object. Returns the
/// stats for the readout. Throws std::invalid_argument on a bad option.
SupportFinStats apply_support_fins(ModelObject &object, const SupportFinOptions &options, int instance_idx = 0);

/// Add one drawn wall to the object (Draw mode). Returns false (with reason) if the engine refused.
bool add_drawn_support_fin(ModelObject &object, const Vec3d &a, const Vec3d &b, const SupportFinOptions &options,
                           std::string &reason, int instance_idx = 0);

/// Remove every fin volume (drawn ones too) and the stored options. Returns how many were removed.
int clear_support_fins(ModelObject &object);

/// Whether the object's fins were generated for a different pose/mesh/options than it has now.
bool support_fins_stale(const ModelObject &object, int instance_idx = 0);

} // namespace Slic3r
