// Port of web/prop/clearance.js -- which stations of a line may carry a wall:
// `stationIsClear` (can it reach the plate through open air), `stationCertified`
// (does the settled top measure clean against the whole part), `welds` (the one
// weld gate), and the run/mask helpers that turn per-station verdicts into the span
// actually swept.
//
// Masks are the JS boolean arrays (std::vector<bool>); runs are the JS [a, b] pairs.
#pragma once
#include "../Inside.hpp"
#include "../Math.hpp"
#include "../Overhangs.hpp"
#include "../PropConfig.hpp"
#include "Contact.hpp"
#include <array>
#include <optional>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// A per-station boolean mask (JS array of booleans).
using Mask = std::vector<bool>;
/// A station range (JS [a, b]): half-open [a, b) from longestRun, inclusive
/// [first, last] from tallBody.
using Run = std::array<int, 2>;

/// Can a wall under station `k` of `line` reach the plate, or is the part in the way?
/// Probes the full height, outside the wall's own profile (half + 0.12 / 0.24 /
/// sideClear) and on the centreline, by insidePart.
bool stationIsClear(const ContactLine &line, size_t k, const Topology &topo, const Rot &rot, const Offset &offset);

/// Certify a SETTLED station by measuring (nearestPart) its cross-section outline --
/// the top flat and both flanks every 2 mm -- against the whole part; false if any
/// probe welds.
bool stationCertified(const ContactLine &line, size_t k, const Topology &topo, const Rot &rot, const Offset &offset);

/// Would a wall this close to the part weld to it? From above (cosUp > 0.7) it is the
/// breakaway interface and must clear 0.675 x gap; sideways a flank, 0.205.
bool welds(double d, double cosUp);
inline bool welds(const NearestHit &hit) { return welds(hit.d, hit.cosUp); }
inline bool welds(const SolidHit &hit) { return welds(hit.d, hit.cosUp); }

/// True when EVERY station can reach the plate.
bool pathToPlateIsClear(const ContactLine &line, const Topology &topo, const Rot &rot, const Offset &offset);

/// The longest contiguous run of true stations, as [a, b) (the first of equal
/// lengths), or nullopt (JS null) when there is none.
std::optional<Run> longestRun(const Mask &usable);

/// longestRun(tall) extended at each end through the contiguous `low` stations, as a
/// mask the length of `tall`; all false when `tall` has no run.
Mask withLowTails(const Mask &tall, const Mask &low);

/// Which stations are a run's full-height BODY (top - gap >= minHeight).
Mask bodyMask(const ContactLine &pts);

/// [first, last] true station of `mask`, or nullopt when the body is < 2 stations.
std::optional<Run> tallBody(const Mask &mask);

/// XY span from the first to the last body station; 0 without a body.
double tallSpan(const ContactLine &pts, const Mask &mask);

/// The least body span a run needs: minSpanTube for a small tube, else minSpan, or
/// (`shortWall`, JS `short`) relaxed toward minSpanShort for a low wall.
double minSpanFor(const ContactLine &pts, const Mask &mask, bool smallTube = false, bool shortWall = false);

/// Where a line's END dips under the squat floor, insert one station exactly at it
/// (interpolated). Mutates and returns `line`.
ContactLine &insertFloorStations(ContactLine &line);

}} // namespace
