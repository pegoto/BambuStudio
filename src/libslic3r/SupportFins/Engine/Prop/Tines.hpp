// Port of web/prop/tines.js -- GRIP TINES: the one-layer nubs a wall reaches into the
// part with, so a tilted part can't peel off it. `emitTines` lays the comb along a
// wall, `biteDirsAt` (internal) aims each nub at the nearest part face, `tineStepFor`
// maps the "Tine grip" setting to nub spacing.
//
// emitTines takes POSITIONAL arguments, as upstream does (there is no options object):
//   emitTines(line, tris, topo, rot, offset, out, stepArg, minTop, tineH, body, grip)
// The trailing five are JS defaulted parameters; std::nullopt / nullptr is JS
// `undefined` / `null` (a defaulted one then takes its default AT CALL TIME, from the
// current PROP). Call shapes upstream:
//   prop.js plate wall     (settled, regionTris, ..., tineStepEff, undefined, tineHeight, tallBody(..), grip)
//   prop.js part-attached  (topLine,  partTris,   ..., tineStepEff, undefined, tineHeight, null, grip)
//   prop.js squat wall     (sqLine,   regionTris, ..., tineStepEff, PROP.squatBrimH, tineHeight, null, grip)
//   draw.js drawnWall      (line, tris, ..., tineStepFor(density), minTop|undefined, layerHeight ?? PROP.tineH)
//   fins/wedges.js         (contact, null, topo, rot, offset, out, tineStepFor(density))
//
// The interface tag (prop/crest.js): under PROP.iface == All the whole comb is tagged,
// through the caller's IfaceTags (nullptr: untracked), parallel to `out`.
#pragma once
#include "../Math.hpp"
#include "../Overhangs.hpp"
#include "../PropConfig.hpp"
#include "../Solids.hpp"
#include "Crest.hpp"
#include "Surface.hpp"
#include <array>
#include <optional>
#include <vector>

namespace Slic3r { namespace SupportFins {

/// One placed tine as the JS test seam records it (globalThis.__TINECAP entries):
/// the seed (x, y, zMid) and the bite heading.
struct TineCap { double x = 0, y = 0, z = 0, biteX = 0, biteY = 0; };

/// JS globalThis.__TINECAP: when non-null, emitTines appends every tine it places.
/// nullptr (the default, JS undefined) = off. prop.js / raster.js read its length.
extern thread_local std::vector<TineCap> *TINECAP;

/// emitTines' `grip` ({ z }): z is lowered to the lowest tine BOTTOM it places (not the
/// lowest emitted vertex -- a wall step under a tine sits lower). Callers start at INF.
struct TineGrip { double z = INF; };

/// emitTines' `body` = [i0, i1]: the station range of a tall wall's full-height body
/// (prop/clearance.js tallBody). Sizes the spacing only.
using TineBody = std::array<int, 2>;

/// How far a wall step stays inside its tine's footprint.
inline constexpr double STEP_INSET = 0.01;

/// tineStepFor(density): nub spacing (mm) for a "Tine grip" setting in [0 sparse .. 1
/// dense]. nullopt (JS undefined/null) is dense. NaN propagates (JS Math.min/max).
double tineStepFor(std::optional<double> density = std::nullopt);

/// emitTines(line, tris, topo, rot, offset, out, stepArg = PROP.tineStep,
///           minTop = PROP.baseH + 0.2, tineH = PROP.tineH, body = null, grip = null):
/// lay a comb of grip tines along a placed wall's settled TOP contour `line` (each
/// station's z is the part surface above it; the wall top is PROP.gap under that),
/// appending closed solids to `out`, and return how many tines landed. `tris` is unused
/// upstream (kept for the call shape). topo/rot/offset as insidePart takes them.
/// Throws std::out_of_range where the JS would throw a TypeError (reading past `line`).
int emitTines(const std::vector<Vec3d> &line, const SurfTris *tris, const Topology &topo, const Rot &rot,
              const Offset &offset, Tris &out,
              std::optional<double> stepArg = std::nullopt, std::optional<double> minTop = std::nullopt,
              std::optional<double> tineH = std::nullopt, const std::optional<TineBody> &body = std::nullopt,
              TineGrip *grip = nullptr, IfaceTags *tags = nullptr);

}} // namespace
