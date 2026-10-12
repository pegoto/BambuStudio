// Port of web/prop/contact.js -- contact lines: the polyline a wall's top follows
// under an overhang, and the passes that bring it to exactly `gap` below the part:
// `contactLine` (lowest underside per bucket), `lowerSag` (spans that sag above the
// surface), `contourTop` (the tip's full width, not just its centre), `settleTop`
// (the true closest approach) -- plus the `straightness` gate a line must pass.
//
// A contact line is the JS array of [x, y, z] stations: std::vector<Vec3d>, z the part
// surface above the station (the wall top is z - PROP.gap). The passes edit it in
// place and return it, as the JS does.
#pragma once
#include "../Math.hpp"
#include "Surface.hpp"
#include <optional>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// A contact line / polyline of stations [x, y, z].
using ContactLine = std::vector<Vec3d>;

/// RMS perpendicular deviation from the best-fit XY axis over the chord it spans;
/// INF for fewer than 3 points or a zero chord. A precondition, not a quality score.
double straightness(const ContactLine &line);

/// The lowest-surface polyline under a region: principal horizontal axis, then the
/// minimum-z sample in each of `nSamples` buckets along it, each re-fitted to the
/// surface above its own xy, then contourTop + lowerSag. nullopt (JS null) when there
/// are fewer than 3 points, no spread, or fewer than 3 filled buckets.
std::optional<ContactLine> contactLine(const std::vector<Vec3d> &pts, const SurfTris &tris, int nSamples);

/// Lower the ends of any segment whose midpoint passes above the surface (3 passes).
/// `band` caps how far below the wall a surface may be and still pull it down.
ContactLine &lowerSag(ContactLine &line, const SurfTris &tris, double band = INF);

/// Lower each station until the wall's whole tip-wide top clears the part (both tip
/// corners, across the wall only), within `band` of the station.
ContactLine &contourTop(ContactLine &line, const SurfTris &tris, double band = INF);

/// Shift each station down so the closest approach of the built top edge (sampled
/// every `step` at both tip corners and the centre) is exactly PROP.gap. Lower only.
ContactLine &settleTop(ContactLine &line, const SurfTris &tris, double step = 0.25, double band = INF);

}} // namespace
